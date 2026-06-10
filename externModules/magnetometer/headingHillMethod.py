from __future__ import annotations

import json
import math
from datetime import datetime, timezone
from typing import Any

import numpy as np
import scipy.linalg


METHOD_NAME = "Heading Hill Magnetometer Method"
METHOD_VERSION = "1.0.0"
SCHEMA_VERSION = "1"

MIN_RING_SAMPLES = 40
MIN_HILL_SAMPLES = 8
FIT_EPS = 1e-12

_last_report = ""


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")


def get_info() -> dict:
    return {
        "name": METHOD_NAME,
        "version": METHOD_VERSION,
        "type": "method",
        "supports_calibrate": True,
        "supports_load_params": True,
        "supports_save_params": True,
        "supports_process": True,
        "input_schema": {
            "timestamp_mcu": "int",
            "timestamp_pc_rx": "int",
            "timestamp_pc_est": "int|None",
            "mag_x": "float",
            "mag_y": "float",
            "mag_z": "float",
            "heading": "float|None",
            "flags": "str",
        },
        "output_schema": {
            "timestamp_mcu": "int",
            "timestamp_pc_rx": "int",
            "timestamp_pc_est": "int|None",
            "mag_x": "float",
            "mag_y": "float",
            "mag_z": "float",
            "heading": "float|None",
            "flags": "str",
        },
        "stream_requirements": {
            "calibrate": [
                {
                    "slot": "mag_input",
                    "kind": "imu.magnetometer_vector",
                    "required": True,
                    "label": "Magnetometer input",
                    "description": "Raw or upstream-corrected magnetometer vector",
                },
                {
                    "slot": "accel_input",
                    "kind": "imu.accel_vector",
                    "required": True,
                    "label": "Accelerometer pitch input",
                    "description": "Pitch angle or accelerometer vector for hill passes",
                },
            ],
            "process": [
                {
                    "slot": "mag_input",
                    "kind": "imu.magnetometer_vector",
                    "required": True,
                    "label": "Magnetometer input",
                    "description": "Magnetometer vector to correct",
                },
                {
                    "slot": "accel_input",
                    "kind": "imu.accel_vector",
                    "required": False,
                    "label": "Accelerometer pitch input",
                    "description": "Optional live pitch for hill leakage correction",
                },
            ],
        },
        "config_schema": [
            {"key": "vertical_horizontal_ratio", "type": "float", "label": "H_v / H_h", "description": "Vertical-to-horizontal magnetic field ratio for hill correction."},
            {"key": "flat_pitch_deg", "type": "float", "label": "Flat pitch threshold deg"},
            {"key": "hill_pitch_min_deg", "type": "float", "label": "Hill pitch minimum deg"},
            {"key": "max_sync_delta_ms", "type": "float", "label": "Max accel sync delta ms"},
            {"key": "pitch_sign", "type": "float", "label": "Pitch sign"},
            {"key": "robust", "type": "choice", "label": "Robust fit", "choices": ["irls", "none"]},
            {"key": "max_iter", "type": "int", "label": "Ellipse IRLS iterations"},
            {"key": "heading_iterations", "type": "int", "label": "Heading correction iterations"},
            {"key": "heading_offset_deg", "type": "float", "label": "Heading offset deg"},
        ],
    }


def get_default_config() -> dict[str, object]:
    return {
        "vertical_horizontal_ratio": 0.0,
        "flat_pitch_deg": 3.0,
        "hill_pitch_min_deg": 2.0,
        "max_sync_delta_ms": 250.0,
        "pitch_sign": 1.0,
        "robust": "irls",
        "max_iter": 4,
        "heading_iterations": 2,
        "heading_offset_deg": 0.0,
    }


def validate_dataset(dataset):
    records = _records_from_dataset(dataset)
    if len(records) < MIN_RING_SAMPLES:
        return {"ok": False, "warnings": [f"need at least {MIN_RING_SAMPLES} magnetometer samples"]}
    return None


def _value(record: Any, name: str, default: Any = None) -> Any:
    if isinstance(record, dict):
        return record.get(name, default)
    return getattr(record, name, default)


def _first_value(record: Any, names: tuple[str, ...], default: Any = None) -> Any:
    for name in names:
        value = _value(record, name, None)
        if value is not None and value != "":
            return value
    return default


def _safe_float(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _config_source(config: Any) -> list[dict[str, Any]]:
    if not isinstance(config, dict):
        return []
    sources = [config]
    params = config.get("params")
    if isinstance(params, dict):
        sources.append(params)
    return sources


def _config_float(config: Any, keys: tuple[str, ...], default: float) -> float:
    for source in _config_source(config):
        for key in keys:
            value = _safe_float(source.get(key))
            if value is not None:
                return value
    return default


def _config_int(config: Any, key: str, default: int) -> int:
    if not isinstance(config, dict):
        return default
    value = config.get(key, default)
    try:
        return max(0, int(value))
    except (TypeError, ValueError):
        return default


def _config_str(config: Any, key: str, default: str) -> str:
    if not isinstance(config, dict):
        return default
    return str(config.get(key, default))


def _stream_records(config: Any, slot: str) -> list[Any]:
    if not isinstance(config, dict):
        return []
    stream_inputs = config.get("stream_inputs")
    if not isinstance(stream_inputs, dict):
        return []
    payload = stream_inputs.get(slot)
    if not isinstance(payload, dict):
        return []
    records = payload.get("records", [])
    return list(records) if isinstance(records, list) else []


def _records_from_dataset(dataset: Any) -> list[Any]:
    if dataset is None or not hasattr(dataset, "records"):
        return []
    return [
        record
        for record in dataset.records
        if _value(record, "stream_id", "") == "raw_magnetometer"
        and "heading_only" not in str(_value(record, "flags", "") or "")
    ]


def _mag_records(dataset: Any, config: Any) -> list[Any]:
    configured = _stream_records(config, "mag_input")
    if configured:
        return configured
    return _records_from_dataset(dataset)


def _timestamp(record: Any) -> float | None:
    for key in ("timestamp_mcu", "timestamp_pc_est", "timestamp_pc_rx"):
        value = _safe_float(_value(record, key))
        if value is not None:
            return value
    return None


def _compute_pitch_deg(record: Any) -> float | None:
    pitch = _safe_float(_value(record, "pitch_deg"))
    if pitch is not None:
        return pitch
    ax = _safe_float(_first_value(record, ("acc_x", "accel_x")))
    ay = _safe_float(_first_value(record, ("acc_y", "accel_y")))
    az = _safe_float(_first_value(record, ("acc_z", "accel_z")))
    if ax is None or ay is None or az is None:
        return None
    magnitude = math.sqrt(ax * ax + ay * ay + az * az)
    if magnitude <= 1e-9:
        return None
    return math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))


def _aligned_pitch_deg(mag_record: Any, accel_records: list[Any], index: int, max_delta_ms: float | None) -> float | None:
    direct = _compute_pitch_deg(mag_record)
    if direct is not None:
        return direct
    if not accel_records:
        return None
    mag_ts = _timestamp(mag_record)
    if mag_ts is None:
        if index < len(accel_records):
            return _compute_pitch_deg(accel_records[index])
        return _compute_pitch_deg(accel_records[-1])

    best_record: Any | None = None
    best_delta = float("inf")
    for accel_record in accel_records:
        accel_ts = _timestamp(accel_record)
        if accel_ts is None:
            continue
        delta = abs(accel_ts - mag_ts)
        if delta < best_delta:
            best_delta = delta
            best_record = accel_record
    if best_record is None:
        if index < len(accel_records):
            return _compute_pitch_deg(accel_records[index])
        return None
    if max_delta_ms is not None and best_delta > max_delta_ms:
        return None
    return _compute_pitch_deg(best_record)


def _build_accel_pitch_index(accel_records: list[Any]) -> tuple[np.ndarray, np.ndarray, list[float | None]]:
    timestamp_pitch_pairs: list[tuple[float, float]] = []
    indexed_pitches: list[float | None] = []
    for record in accel_records:
        pitch = _compute_pitch_deg(record)
        indexed_pitches.append(pitch)
        if pitch is None:
            continue
        timestamp = _timestamp(record)
        if timestamp is not None:
            timestamp_pitch_pairs.append((timestamp, pitch))

    timestamp_pitch_pairs.sort(key=lambda item: item[0])
    if not timestamp_pitch_pairs:
        return np.asarray([], dtype=float), np.asarray([], dtype=float), indexed_pitches
    timestamps = np.asarray([item[0] for item in timestamp_pitch_pairs], dtype=float)
    pitches = np.asarray([item[1] for item in timestamp_pitch_pairs], dtype=float)
    return timestamps, pitches, indexed_pitches


def _indexed_pitch(indexed_pitches: list[float | None], index: int) -> float | None:
    if not indexed_pitches:
        return None
    if index < len(indexed_pitches):
        return indexed_pitches[index]
    return indexed_pitches[-1]


def _aligned_pitch_deg_indexed(
    mag_record: Any,
    index: int,
    max_delta_ms: float | None,
    accel_timestamps: np.ndarray,
    accel_pitches: np.ndarray,
    indexed_pitches: list[float | None],
) -> float | None:
    direct = _compute_pitch_deg(mag_record)
    if direct is not None:
        return direct
    if not indexed_pitches:
        return None

    mag_ts = _timestamp(mag_record)
    if mag_ts is None or len(accel_timestamps) == 0:
        return _indexed_pitch(indexed_pitches, index)

    position = int(np.searchsorted(accel_timestamps, mag_ts))
    candidates: list[int] = []
    if position < len(accel_timestamps):
        candidates.append(position)
    if position > 0:
        candidates.append(position - 1)
    if not candidates:
        return _indexed_pitch(indexed_pitches, index)

    best = min(candidates, key=lambda item: abs(float(accel_timestamps[item]) - mag_ts))
    best_delta = abs(float(accel_timestamps[best]) - mag_ts)
    if max_delta_ms is not None and best_delta > max_delta_ms:
        return None
    return float(accel_pitches[best])


def _extract_samples(dataset: Any, config: Any) -> list[dict[str, Any]]:
    mag_records = _mag_records(dataset, config)
    accel_records = _stream_records(config, "accel_input")
    max_delta = _config_float(config, ("max_sync_delta_ms",), 250.0)
    pitch_sign = _config_float(config, ("pitch_sign",), 1.0)
    accel_timestamps, accel_pitches, indexed_pitches = _build_accel_pitch_index(accel_records)
    samples: list[dict[str, Any]] = []
    for index, record in enumerate(mag_records):
        try:
            mag = np.array(
                [
                    float(_value(record, "mag_x")),
                    float(_value(record, "mag_y")),
                    float(_value(record, "mag_z", 0.0)),
                ],
                dtype=float,
            )
        except (TypeError, ValueError):
            continue
        if not np.all(np.isfinite(mag)):
            continue
        pitch = _aligned_pitch_deg_indexed(
            record,
            index,
            max_delta,
            accel_timestamps,
            accel_pitches,
            indexed_pitches,
        )
        samples.append(
            {
                "record": record,
                "mag": mag,
                "pitch_rad": None if pitch is None else math.radians(pitch_sign * pitch),
                "flags": str(_value(record, "flags", "") or ""),
                "timestamp_mcu": _value(record, "timestamp_mcu"),
                "timestamp_pc_rx": _value(record, "timestamp_pc_rx"),
                "timestamp_pc_est": _value(record, "timestamp_pc_est"),
            }
        )
    return samples


def _normalize_points(points: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    mu = points.mean(axis=0)
    scale = points.std(axis=0)
    scale[scale <= FIT_EPS] = 1.0
    return (points - mu) / scale, mu, scale


def _fit_ellipse_direct(points: np.ndarray, weights: np.ndarray | None = None) -> np.ndarray:
    x_val = points[:, 0]
    y_val = points[:, 1]
    if weights is None:
        weight = np.ones(len(points), dtype=float)
    else:
        weight = np.maximum(np.asarray(weights, dtype=float), 0.0)
    if len(weight) != len(points):
        raise ValueError("weights length must match point count")
    if float(np.sum(weight)) <= FIT_EPS:
        raise ValueError("ellipse fit weights are all zero")

    sqrt_weight = np.sqrt(weight)
    d1 = np.column_stack([x_val * x_val, x_val * y_val, y_val * y_val]) * sqrt_weight[:, None]
    d2 = np.column_stack([x_val, y_val, np.ones_like(x_val)]) * sqrt_weight[:, None]

    s1 = d1.T @ d1
    s2 = d1.T @ d2
    s3 = d2.T @ d2
    reg = 1e-12 * max(float(np.trace(s3)) / 3.0, 1.0)
    s3 = s3 + reg * np.eye(3)

    try:
        t_matrix = -np.linalg.solve(s3, s2.T)
    except np.linalg.LinAlgError:
        t_matrix = -np.linalg.pinv(s3) @ s2.T
    m_matrix = s1 + s2 @ t_matrix
    constraint = np.array(
        [
            [0.0, 0.0, 2.0],
            [0.0, -1.0, 0.0],
            [2.0, 0.0, 0.0],
        ],
        dtype=float,
    )

    _, eigvecs = scipy.linalg.eig(m_matrix, constraint)
    eigvecs = np.real_if_close(eigvecs, tol=1000).real
    candidates: list[np.ndarray] = []
    for idx in range(eigvecs.shape[1]):
        quadratic = eigvecs[:, idx]
        if not np.all(np.isfinite(quadratic)):
            continue
        a_coef, b_coef, c_coef = quadratic
        if 4.0 * a_coef * c_coef - b_coef * b_coef <= FIT_EPS:
            continue
        linear = t_matrix @ quadratic
        theta = np.r_[quadratic, linear]
        norm = float(np.linalg.norm(theta))
        if norm > FIT_EPS and np.all(np.isfinite(theta)):
            candidates.append(theta / norm)
    if not candidates:
        raise ValueError("no valid ellipse candidate")

    design = np.column_stack([x_val * x_val, x_val * y_val, y_val * y_val, x_val, y_val, np.ones_like(x_val)])
    return min(candidates, key=lambda item: float(np.mean((design @ item) ** 2)))


def _conic_to_center_w(theta: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    a_coef, b_coef, c_coef, d_coef, e_coef, f_coef = [float(value) for value in theta]
    q_matrix = np.array([[a_coef, b_coef / 2.0], [b_coef / 2.0, c_coef]], dtype=float)
    d_vector = np.array([d_coef, e_coef], dtype=float)

    if np.any(np.linalg.eigvalsh(q_matrix) <= FIT_EPS):
        q_matrix = -q_matrix
        d_vector = -d_vector
        f_coef = -f_coef
    if np.any(np.linalg.eigvalsh(q_matrix) <= FIT_EPS):
        raise ValueError("fitted conic is not an ellipse")

    center = -0.5 * np.linalg.solve(q_matrix, d_vector)
    radius_param = float(center @ q_matrix @ center - f_coef)
    if radius_param <= FIT_EPS:
        raise ValueError("invalid ellipse normalization")
    return center, q_matrix / radius_param


def _denormalize(center_n: np.ndarray, w_n: np.ndarray, mu: np.ndarray, scale: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    scale_matrix = np.diag(scale)
    inv_scale = np.diag(1.0 / scale)
    center = mu + scale_matrix @ center_n
    w_matrix = inv_scale.T @ w_n @ inv_scale
    return center, w_matrix


def _sqrt_spd(w_matrix: np.ndarray) -> np.ndarray:
    values, vectors = np.linalg.eigh(w_matrix)
    if np.any(values <= FIT_EPS):
        raise ValueError("calibration matrix is not positive definite")
    return vectors @ np.diag(np.sqrt(values)) @ vectors.T


def _axis_radius_from_unit_w(w_unit: np.ndarray) -> float:
    values = np.linalg.eigvalsh(w_unit)
    if np.any(values <= FIT_EPS):
        raise ValueError("ellipse axis matrix is not positive definite")
    return float(1.0 / math.sqrt(float(np.min(values))))


def _radial_residual_unit(points: np.ndarray, center: np.ndarray, w_unit: np.ndarray) -> np.ndarray:
    calibration = _sqrt_spd(w_unit)
    corrected = (calibration @ (points - center).T).T
    return np.linalg.norm(corrected, axis=1) - 1.0


def _huber_weights(residual: np.ndarray) -> np.ndarray:
    median = float(np.median(residual))
    sigma = 1.4826 * float(np.median(np.abs(residual - median))) + FIT_EPS
    scaled = residual / (1.345 * sigma)
    weights = np.ones_like(scaled)
    mask = np.abs(scaled) > 1.0
    weights[mask] = 1.0 / np.abs(scaled[mask])
    return weights


def _fit_center_w_unit(points: np.ndarray, *, robust: str, max_iter: int) -> tuple[np.ndarray, np.ndarray]:
    normalized, mu, scale = _normalize_points(points)
    theta = _fit_ellipse_direct(normalized)
    if robust.strip().lower() == "irls":
        weights = np.ones(len(points), dtype=float)
        for _ in range(max_iter):
            center_n, w_unit_n = _conic_to_center_w(theta)
            center, w_unit = _denormalize(center_n, w_unit_n, mu, scale)
            weights = _huber_weights(_radial_residual_unit(points, center, w_unit))
            theta = _fit_ellipse_direct(normalized, weights=weights)
    center_n, w_unit_n = _conic_to_center_w(theta)
    return _denormalize(center_n, w_unit_n, mu, scale)


def _angular_coverage_rad(points_xy: np.ndarray) -> float:
    if len(points_xy) < 2:
        return 0.0
    center = np.median(points_xy, axis=0)
    angles = np.sort((np.arctan2(points_xy[:, 1] - center[1], points_xy[:, 0] - center[0]) + 2.0 * np.pi) % (2.0 * np.pi))
    gaps = np.diff(np.r_[angles, angles[0] + 2.0 * np.pi])
    return float(2.0 * np.pi - np.max(gaps))


def _flag_tokens(flags: str) -> set[str]:
    normalized = flags.lower()
    for char in "|,;:/\\":
        normalized = normalized.replace(char, " ")
    normalized = normalized.replace("-", "_")
    return {token for token in normalized.split() if token}


def _segment_label(flags: str) -> str:
    tokens = _flag_tokens(flags)
    joined = " ".join(sorted(tokens))
    if any(token in tokens for token in ("hill_forward", "forward_hill", "hill_fwd", "fwd_hill", "forward", "fwd", "uphill")):
        return "forward"
    if any(token in tokens for token in ("hill_backward", "backward_hill", "hill_back", "back_hill", "backward", "back", "reverse", "downhill")):
        return "backward"
    if "hill" in tokens and ("+" in flags or "plus" in tokens):
        return "forward"
    if "hill" in tokens and ("-" in flags or "minus" in tokens):
        return "backward"
    if "ring" in tokens or "flat_ring" in tokens or "calib_ring" in tokens or "circle" in tokens:
        return "ring"
    if "hill_forward" in joined:
        return "forward"
    if "hill_backward" in joined:
        return "backward"
    return ""


def _ring_mask(samples: list[dict[str, Any]], config: Any) -> tuple[np.ndarray, str, list[str]]:
    labels = np.array([_segment_label(sample["flags"]) for sample in samples], dtype=object)
    pitch = np.array([float("nan") if sample["pitch_rad"] is None else float(sample["pitch_rad"]) for sample in samples])
    points = np.asarray([sample["mag"][:2] for sample in samples], dtype=float)
    warnings: list[str] = []
    explicit = labels == "ring"
    if int(np.sum(explicit)) >= MIN_RING_SAMPLES:
        return explicit, "flags:ring", warnings

    flat_pitch_deg = _config_float(config, ("flat_pitch_deg",), 3.0)
    pitch_mask = np.isfinite(pitch) & (np.abs(np.degrees(pitch)) <= flat_pitch_deg)
    if int(np.sum(pitch_mask)) >= MIN_RING_SAMPLES:
        coverage = _angular_coverage_rad(points[pitch_mask])
        if coverage >= math.radians(220.0):
            warnings.append("ring segment inferred from low-pitch samples")
            return pitch_mask, "pitch:auto", warnings

    all_mask = np.ones(len(samples), dtype=bool)
    warnings.append("ring segment flags were not found; fitting the flat ellipse on all magnetometer samples")
    return all_mask, "all:fallback", warnings


def _unit_vectors(values: np.ndarray) -> np.ndarray:
    norms = np.linalg.norm(values, axis=1)
    result = np.zeros_like(values)
    mask = norms > FIT_EPS
    result[mask] = values[mask] / norms[mask, None]
    return result


def _split_hill_masks(
    samples: list[dict[str, Any]],
    ring_mask: np.ndarray,
    normalized_xy: np.ndarray,
    config: Any,
) -> tuple[dict[str, np.ndarray], str, list[str]]:
    labels = np.array([_segment_label(sample["flags"]) for sample in samples], dtype=object)
    forward = labels == "forward"
    backward = labels == "backward"
    warnings: list[str] = []
    if int(np.sum(forward)) >= MIN_HILL_SAMPLES and int(np.sum(backward)) >= MIN_HILL_SAMPLES:
        return {"forward": forward, "backward": backward}, "flags", warnings

    pitch = np.array([float("nan") if sample["pitch_rad"] is None else float(sample["pitch_rad"]) for sample in samples])
    min_pitch_deg = _config_float(config, ("hill_pitch_min_deg",), 2.0)
    hill = (~ring_mask) & np.isfinite(pitch) & (np.abs(np.degrees(pitch)) >= min_pitch_deg)
    if int(np.sum(hill)) < 2 * MIN_HILL_SAMPLES:
        hill = np.isfinite(pitch) & (np.abs(np.degrees(pitch)) >= min_pitch_deg)
    if int(np.sum(hill)) < MIN_HILL_SAMPLES:
        raise ValueError("need hill samples with pitch data; add hill_forward/hill_backward flags or record accelerometer pitch")

    units = _unit_vectors(normalized_xy[hill])
    if len(units) >= 2 * MIN_HILL_SAMPLES:
        cov = units.T @ units
        values, vectors = np.linalg.eigh(cov)
        axis = vectors[:, int(np.argmax(values))]
        dots = units @ axis
        hill_indices = np.flatnonzero(hill)
        auto_forward = np.zeros(len(samples), dtype=bool)
        auto_backward = np.zeros(len(samples), dtype=bool)
        auto_forward[hill_indices[dots >= 0.0]] = True
        auto_backward[hill_indices[dots < 0.0]] = True
        if int(np.sum(auto_forward)) >= MIN_HILL_SAMPLES and int(np.sum(auto_backward)) >= MIN_HILL_SAMPLES:
            warnings.append("hill pass segments inferred from two opposite heading clusters")
            return {"forward": auto_forward, "backward": auto_backward}, "heading:auto", warnings

    warnings.append("only one reliable hill heading cluster found; using rough per-sample heading fallback")
    return {"single": hill}, "rough_heading:auto", warnings


def _estimate_base_gamma(mask: np.ndarray, normalized_xy: np.ndarray, pitch_rad: np.ndarray, flat_pitch_deg: float) -> float:
    pass_indices = np.flatnonzero(mask)
    if not len(pass_indices):
        raise ValueError("empty hill pass")
    finite_indices = pass_indices[np.isfinite(pitch_rad[pass_indices])]
    if len(finite_indices):
        abs_pitch_deg = np.abs(np.degrees(pitch_rad[finite_indices]))
        min_pitch_deg = float(np.min(abs_pitch_deg))
        if min_pitch_deg <= flat_pitch_deg:
            selected = finite_indices[abs_pitch_deg <= min_pitch_deg + 0.25]
        else:
            selected = np.asarray([], dtype=int)
    else:
        selected = np.asarray([], dtype=int)
    if not len(selected):
        finite_or_all = finite_indices if len(finite_indices) else pass_indices
        ordered = finite_or_all[np.argsort(np.abs(pitch_rad[finite_or_all]))]
        count = max(3, min(len(ordered), max(3, len(ordered) // 5)))
        selected = ordered[:count]
    vectors = _unit_vectors(normalized_xy[selected])
    mean_vec = vectors.mean(axis=0)
    norm = float(np.linalg.norm(mean_vec))
    if norm <= FIT_EPS:
        mean_vec = _unit_vectors(normalized_xy[pass_indices]).mean(axis=0)
        norm = float(np.linalg.norm(mean_vec))
    if norm <= FIT_EPS:
        raise ValueError("cannot estimate base hill heading")
    mean_vec = mean_vec / norm
    return float(math.atan2(mean_vec[1], mean_vec[0]))


def _fit_eta(
    masks: dict[str, np.ndarray],
    normalized_xy: np.ndarray,
    pitch_rad: np.ndarray,
    *,
    k_ratio: float,
    flat_pitch_deg: float,
    robust: str,
) -> tuple[np.ndarray, dict[str, float], dict[str, Any]]:
    rows: list[tuple[float, np.ndarray, str]] = []
    gammas: dict[str, float] = {}
    for name, mask in masks.items():
        if name == "single":
            indices = np.flatnonzero(mask & np.isfinite(pitch_rad))
            for index in indices:
                gamma = float(math.atan2(normalized_xy[index, 1], normalized_xy[index, 0]))
                theta = float(pitch_rad[index])
                cos_gamma = math.cos(gamma)
                z_val = -cos_gamma * math.sin(theta) + k_ratio * (math.cos(theta) - 1.0)
                if abs(z_val) <= 1e-6:
                    continue
                base = np.array([math.cos(gamma), math.sin(gamma)], dtype=float)
                g_vec = np.array([cos_gamma * (math.cos(theta) - 1.0) + k_ratio * math.sin(theta), 0.0], dtype=float)
                rows.append((z_val, normalized_xy[index] - base - g_vec, name))
            continue

        gamma = _estimate_base_gamma(mask, normalized_xy, pitch_rad, flat_pitch_deg)
        gammas[name] = gamma
        cos_gamma = math.cos(gamma)
        base = np.array([math.cos(gamma), math.sin(gamma)], dtype=float)
        indices = np.flatnonzero(mask & np.isfinite(pitch_rad))
        for index in indices:
            theta = float(pitch_rad[index])
            z_val = -cos_gamma * math.sin(theta) + k_ratio * (math.cos(theta) - 1.0)
            if abs(z_val) <= 1e-6:
                continue
            g_vec = np.array([cos_gamma * (math.cos(theta) - 1.0) + k_ratio * math.sin(theta), 0.0], dtype=float)
            rows.append((z_val, normalized_xy[index] - base - g_vec, name))

    if len(rows) < MIN_HILL_SAMPLES:
        raise ValueError("hill excitation is too small for leakage estimation")

    z = np.asarray([row[0] for row in rows], dtype=float)
    e = np.asarray([row[1] for row in rows], dtype=float)
    weights = np.ones(len(rows), dtype=float)
    eta = np.zeros(2, dtype=float)
    iterations = 4 if robust.strip().lower() == "irls" else 1
    for _ in range(iterations):
        denom = float(np.sum(weights * z * z))
        if denom <= FIT_EPS:
            raise ValueError("hill excitation is degenerate")
        eta = np.sum((weights * z)[:, None] * e, axis=0) / denom
        residual = e - z[:, None] * eta[None, :]
        if iterations == 1:
            break
        residual_norm = np.linalg.norm(residual, axis=1)
        weights = _huber_weights(residual_norm)

    residual_before = e
    residual_after = e - z[:, None] * eta[None, :]
    before_sse = float(np.sum(residual_before * residual_before))
    after_sse = float(np.sum(residual_after * residual_after))
    excitation = float(np.sum(weights * z * z))
    sigma2 = after_sse / max(1, 2 * len(rows) - 2)
    metrics = {
        "hill_sample_count": int(len(rows)),
        "hill_excitation": excitation,
        "hill_rms_before": math.sqrt(before_sse / max(1, len(rows))),
        "hill_rms_after": math.sqrt(after_sse / max(1, len(rows))),
        "hill_improvement_gain": 0.0 if before_sse <= FIT_EPS else 1.0 - after_sse / before_sse,
        "eta_sigma": math.sqrt(max(0.0, sigma2 / max(excitation, FIT_EPS))),
    }
    return eta, gammas, metrics


def calibrate(dataset, config=None):
    global _last_report
    samples = _extract_samples(dataset, config)
    if len(samples) < MIN_RING_SAMPLES:
        raise ValueError(f"need at least {MIN_RING_SAMPLES} magnetometer samples")
    if not any(sample["pitch_rad"] is not None for sample in samples):
        raise ValueError("heading hill calibration requires accelerometer pitch records")

    warnings: list[str] = []
    ring_mask, ring_source, ring_warnings = _ring_mask(samples, config)
    warnings.extend(ring_warnings)
    ring_points = np.asarray([sample["mag"][:2] for sample, use in zip(samples, ring_mask) if use], dtype=float)
    if len(ring_points) < MIN_RING_SAMPLES:
        raise ValueError(f"need at least {MIN_RING_SAMPLES} flat ring samples")

    robust = _config_str(config, "robust", "irls")
    max_iter = _config_int(config, "max_iter", 4)
    center, w_unit = _fit_center_w_unit(ring_points, robust=robust, max_iter=max_iter)
    w_sqrt = _sqrt_spd(w_unit)
    all_xy = np.asarray([sample["mag"][:2] for sample in samples], dtype=float)
    normalized_xy = (w_sqrt @ (all_xy - center).T).T

    masks, hill_source, hill_warnings = _split_hill_masks(samples, ring_mask, normalized_xy, config)
    warnings.extend(hill_warnings)
    pitch_rad = np.array([float("nan") if sample["pitch_rad"] is None else float(sample["pitch_rad"]) for sample in samples])
    k_ratio = _config_float(config, ("vertical_horizontal_ratio", "field_ratio", "k", "H_v_over_H_h"), 0.0)
    flat_pitch_deg = _config_float(config, ("flat_pitch_deg",), 3.0)
    eta, gammas, hill_metrics = _fit_eta(
        masks,
        normalized_xy,
        pitch_rad,
        k_ratio=k_ratio,
        flat_pitch_deg=flat_pitch_deg,
        robust=robust,
    )

    ring_residual = _radial_residual_unit(ring_points, center, w_unit)
    ring_rms = math.sqrt(float(np.mean(ring_residual * ring_residual)))
    output_scale = _axis_radius_from_unit_w(w_unit)
    coverage = _angular_coverage_rad(ring_points)
    condition_number = float(np.linalg.cond(w_unit))
    max_pitch = float(np.nanmax(np.abs(np.degrees(pitch_rad)))) if np.any(np.isfinite(pitch_rad)) else 0.0
    heading_offset_deg = _config_float(config, ("heading_offset_deg", "psi0_deg"), 0.0)
    pitch_sign = _config_float(config, ("pitch_sign",), 1.0)

    gamma_turn_quality = None
    if "forward" in gammas and "backward" in gammas:
        diff = abs(((math.degrees(gammas["backward"] - gammas["forward"]) + 180.0) % 360.0) - 180.0)
        gamma_turn_quality = abs(diff - 180.0)

    if abs(k_ratio) <= FIT_EPS:
        warnings.append("vertical_horizontal_ratio is 0.0; set k=H_v/H_h for best hill correction")

    _last_report = (
        f"Ring samples: {len(ring_points)} ({ring_source})\n"
        f"Hill samples: {hill_metrics['hill_sample_count']} ({hill_source})\n"
        f"Ring center: ({center[0]:.6f}, {center[1]:.6f})\n"
        f"Ring RMS residual: {ring_rms:.6f}\n"
        f"Ring coverage deg: {math.degrees(coverage):.6f}\n"
        f"Condition number: {condition_number:.6f}\n"
        f"Output scale: {output_scale:.6f}\n"
        f"k=H_v/H_h: {k_ratio:.6f}\n"
        f"eta: ({eta[0]:.8f}, {eta[1]:.8f})\n"
        f"Max pitch deg: {max_pitch:.6f}\n"
        f"Hill excitation: {hill_metrics['hill_excitation']:.8f}\n"
        f"Hill RMS before: {hill_metrics['hill_rms_before']:.8f}\n"
        f"Hill RMS after: {hill_metrics['hill_rms_after']:.8f}\n"
        f"Hill gain: {hill_metrics['hill_improvement_gain']:.8f}\n"
        f"Warnings: {'; '.join(warnings) if warnings else '-'}"
    )

    return {
        "algorithm_name": METHOD_NAME,
        "algorithm_version": METHOD_VERSION,
        "schema_version": SCHEMA_VERSION,
        "created_at": _utc_now(),
        "params": {
            "offset_x": float(center[0]),
            "offset_y": float(center[1]),
            "w_unit_00": float(w_unit[0, 0]),
            "w_unit_01": float(w_unit[0, 1]),
            "w_unit_10": float(w_unit[1, 0]),
            "w_unit_11": float(w_unit[1, 1]),
            "w_sqrt_00": float(w_sqrt[0, 0]),
            "w_sqrt_01": float(w_sqrt[0, 1]),
            "w_sqrt_10": float(w_sqrt[1, 0]),
            "w_sqrt_11": float(w_sqrt[1, 1]),
            "eta_x": float(eta[0]),
            "eta_y": float(eta[1]),
            "vertical_horizontal_ratio": float(k_ratio),
            "heading_offset_deg": float(heading_offset_deg),
            "pitch_sign": float(pitch_sign),
            "heading_iterations": _config_int(config, "heading_iterations", 2),
            "output_scale": float(output_scale),
            "ring_sample_count": int(len(ring_points)),
            "ring_source": ring_source,
            "hill_source": hill_source,
            "ring_rms_residual": float(ring_rms),
            "coverage_rad": float(coverage),
            "coverage_deg": float(math.degrees(coverage)),
            "condition_number": condition_number,
            "max_pitch_deg": max_pitch,
            "gamma_forward_rad": None if "forward" not in gammas else float(gammas["forward"]),
            "gamma_backward_rad": None if "backward" not in gammas else float(gammas["backward"]),
            "gamma_turn_error_deg": gamma_turn_quality,
            **hill_metrics,
            "warnings": warnings,
        },
    }


def get_last_report() -> str:
    return _last_report


def load_params(path):
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def save_params(path, params):
    payload = dict(params or {})
    payload.setdefault("algorithm_name", METHOD_NAME)
    payload.setdefault("algorithm_version", METHOD_VERSION)
    payload.setdefault("schema_version", SCHEMA_VERSION)
    payload.setdefault("created_at", _utc_now())
    payload.setdefault("params", {})
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)


def _pitch_from_process_sample(sample: dict[str, Any]) -> float | None:
    pitch = _compute_pitch_deg(sample)
    if pitch is not None:
        return pitch
    inputs = sample.get("inputs")
    if isinstance(inputs, dict):
        accel_input = inputs.get("accel_input")
        if accel_input is not None:
            return _compute_pitch_deg(accel_input)
    return None


def process(sample, params):
    inner = params.get("params", {}) if isinstance(params, dict) else {}
    offset = np.array(
        [
            float(inner.get("offset_x", 0.0)),
            float(inner.get("offset_y", 0.0)),
        ],
        dtype=float,
    )
    w_sqrt = np.array(
        [
            [float(inner.get("w_sqrt_00", 1.0)), float(inner.get("w_sqrt_01", 0.0))],
            [float(inner.get("w_sqrt_10", 0.0)), float(inner.get("w_sqrt_11", 1.0))],
        ],
        dtype=float,
    )
    eta = np.array([float(inner.get("eta_x", 0.0)), float(inner.get("eta_y", 0.0))], dtype=float)
    k_ratio = float(inner.get("vertical_horizontal_ratio", 0.0))
    output_scale = float(inner.get("output_scale", 1.0))
    heading_offset_deg = float(inner.get("heading_offset_deg", 0.0))
    pitch_sign = float(inner.get("pitch_sign", 1.0))
    iterations = max(0, int(inner.get("heading_iterations", 2)))

    raw_xy = np.array([float(sample["mag_x"]), float(sample["mag_y"])], dtype=float)
    corrected = w_sqrt @ (raw_xy - offset)
    pitch_deg = _pitch_from_process_sample(sample)
    if pitch_deg is not None:
        theta = math.radians(pitch_sign * pitch_deg)
        corrected_iter = corrected.copy()
        for _ in range(iterations + 1):
            gamma = math.atan2(float(corrected_iter[1]), float(corrected_iter[0]))
            z_val = -math.cos(gamma) * math.sin(theta) + k_ratio * (math.cos(theta) - 1.0)
            corrected_iter = corrected - eta * z_val
        corrected = corrected_iter

    mag_x = float(corrected[0] * output_scale)
    mag_y = float(corrected[1] * output_scale)
    mag_z = float(sample.get("mag_z", 0.0))
    heading = None
    if not (math.isclose(mag_x, 0.0, abs_tol=1e-9) and math.isclose(mag_y, 0.0, abs_tol=1e-9)):
        heading = (math.degrees(math.atan2(mag_x, mag_y)) + heading_offset_deg + 360.0) % 360.0

    return {
        "timestamp_mcu": sample["timestamp_mcu"],
        "timestamp_pc_rx": sample["timestamp_pc_rx"],
        "timestamp_pc_est": sample.get("timestamp_pc_est"),
        "mag_x": mag_x,
        "mag_y": mag_y,
        "mag_z": mag_z,
        "heading": heading,
        "flags": "heading_hill_rt",
    }
