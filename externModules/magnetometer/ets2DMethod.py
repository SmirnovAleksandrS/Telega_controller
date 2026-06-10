from __future__ import annotations

import json
import math
from datetime import datetime, timezone
from typing import Any

import numpy as np
import scipy.linalg
from scipy.optimize import least_squares


METHOD_NAME = "2D ETS Magnetometer Method"
METHOD_VERSION = "1.0.0"
SCHEMA_VERSION = "1"

MIN_RAW_SAMPLES = 50
DEFAULT_MAX_ITER = 4
COVERAGE_WARNING_RAD = 1.5 * math.pi
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
    }


def get_default_config() -> dict[str, object]:
    return {
        "field_radius": None,
        "robust": "irls",
        "max_iter": DEFAULT_MAX_ITER,
        "refine": False,
        "expected_noise_std": None,
    }


def validate_dataset(dataset):
    points = _extract_points_from_dataset(dataset)
    if len(points) < MIN_RAW_SAMPLES:
        return {"ok": False, "warnings": [f"need at least {MIN_RAW_SAMPLES} raw_magnetometer samples"]}

    warnings: list[str] = []
    coverage = _angular_coverage_rad(points[:, :2])
    if coverage < COVERAGE_WARNING_RAD:
        warnings.append(
            f"2D angular coverage is low ({math.degrees(coverage):.1f} deg); "
            "ETS 2D calibration may be unstable"
        )
    return {"ok": True, "warnings": warnings} if warnings else None


def _value(record: Any, name: str, default: Any = None) -> Any:
    if isinstance(record, dict):
        return record.get(name, default)
    return getattr(record, name, default)


def _is_heading_only(record: Any) -> bool:
    return "heading_only" in str(_value(record, "flags", "") or "")


def _records_from_config(config: Any) -> list[Any]:
    if not isinstance(config, dict):
        return []
    stream_inputs = config.get("stream_inputs")
    if not isinstance(stream_inputs, dict):
        return []
    mag_input = stream_inputs.get("mag_input")
    if not isinstance(mag_input, dict):
        return []
    records = mag_input.get("records", [])
    return list(records) if isinstance(records, list) else []


def _extract_points_from_records(records: list[Any]) -> np.ndarray:
    points: list[tuple[float, float, float]] = []
    for record in records:
        if _is_heading_only(record):
            continue
        try:
            point = (
                float(_value(record, "mag_x")),
                float(_value(record, "mag_y")),
                float(_value(record, "mag_z", 0.0)),
            )
        except (TypeError, ValueError):
            continue
        if all(math.isfinite(value) for value in point):
            points.append(point)
    return np.asarray(points, dtype=float).reshape((-1, 3))


def _extract_points_from_dataset(dataset: Any) -> np.ndarray:
    if dataset is None or not hasattr(dataset, "records"):
        return np.empty((0, 3), dtype=float)
    records = [
        record
        for record in dataset.records
        if getattr(record, "stream_id", "") == "raw_magnetometer"
    ]
    return _extract_points_from_records(records)


def _extract_points(dataset: Any, config: Any = None) -> np.ndarray:
    configured = _extract_points_from_records(_records_from_config(config))
    if len(configured):
        return configured
    return _extract_points_from_dataset(dataset)


def _config_float(config: Any, *keys: str) -> float | None:
    if not isinstance(config, dict):
        return None
    sources = [config]
    params = config.get("params")
    if isinstance(params, dict):
        sources.append(params)
    for source in sources:
        for key in keys:
            if key not in source or source[key] in (None, ""):
                continue
            try:
                value = float(source[key])
            except (TypeError, ValueError):
                continue
            if math.isfinite(value) and value > 0.0:
                return value
    return None


def _config_bool(config: Any, key: str, default: bool) -> bool:
    if not isinstance(config, dict):
        return default
    value = config.get(key, default)
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


def _config_int(config: Any, key: str, default: int) -> int:
    if not isinstance(config, dict):
        return default
    try:
        return max(0, int(config.get(key, default)))
    except (TypeError, ValueError):
        return default


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

    eigvals, eigvecs = scipy.linalg.eig(m_matrix, constraint)
    _ = eigvals
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


def _conic_to_center_w(theta: np.ndarray, field_radius: float = 1.0) -> tuple[np.ndarray, np.ndarray]:
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
    w_matrix = q_matrix * (field_radius * field_radius / radius_param)
    return center, w_matrix


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


def _extract_ets_params(w_matrix: np.ndarray) -> tuple[float, float, float, np.ndarray, np.ndarray]:
    w11 = float(w_matrix[0, 0])
    w12 = float(w_matrix[0, 1])
    w22 = float(w_matrix[1, 1])
    if w11 <= FIT_EPS or w22 <= FIT_EPS:
        raise ValueError("invalid ETS quadratic form")
    sin_rho = -w12 / math.sqrt(w11 * w22)
    sin_rho = max(-1.0, min(1.0, sin_rho))
    rho = math.asin(sin_rho)
    cos_rho = math.cos(rho)
    if abs(cos_rho) <= FIT_EPS:
        raise ValueError("invalid ETS nonorthogonality angle")
    a_scale = 1.0 / (cos_rho * math.sqrt(w11))
    b_scale = 1.0 / (cos_rho * math.sqrt(w22))
    forward = np.array(
        [
            [a_scale, 0.0],
            [b_scale * math.sin(rho), b_scale * math.cos(rho)],
        ],
        dtype=float,
    )
    calibration = np.linalg.inv(forward)
    return a_scale, b_scale, rho, forward, calibration


def _radial_residual(points: np.ndarray, center: np.ndarray, calibration: np.ndarray, field_radius: float) -> np.ndarray:
    corrected = (calibration @ (points - center).T).T
    return np.linalg.norm(corrected, axis=1) - field_radius


def _huber_weights(residual: np.ndarray) -> np.ndarray:
    median = float(np.median(residual))
    sigma = 1.4826 * float(np.median(np.abs(residual - median))) + FIT_EPS
    scaled = residual / (1.345 * sigma)
    weights = np.ones_like(scaled)
    mask = np.abs(scaled) > 1.0
    weights[mask] = 1.0 / np.abs(scaled[mask])
    return weights


def _fit_center_w_unit(points: np.ndarray, *, robust: str, max_iter: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    normalized, mu, scale = _normalize_points(points)
    theta = _fit_ellipse_direct(normalized)
    robust_key = robust.strip().lower()
    if robust_key == "irls":
        weights = np.ones(len(points), dtype=float)
        for _ in range(max_iter):
            center_n, w_unit_n = _conic_to_center_w(theta, field_radius=1.0)
            center, w_unit = _denormalize(center_n, w_unit_n, mu, scale)
            field_radius = _axis_radius_from_unit_w(w_unit)
            _, _, _, _, calibration = _extract_ets_params(w_unit * field_radius * field_radius)
            weights = _huber_weights(_radial_residual(points, center, calibration, field_radius))
            theta = _fit_ellipse_direct(normalized, weights=weights)
        return center, w_unit, weights > 0.0
    if robust_key == "trim":
        center_n, w_unit_n = _conic_to_center_w(theta, field_radius=1.0)
        center, w_unit = _denormalize(center_n, w_unit_n, mu, scale)
        field_radius = _axis_radius_from_unit_w(w_unit)
        _, _, _, _, calibration = _extract_ets_params(w_unit * field_radius * field_radius)
        residual = _radial_residual(points, center, calibration, field_radius)
        sigma = 1.4826 * float(np.median(np.abs(residual - np.median(residual)))) + FIT_EPS
        inliers = np.abs(residual) <= 3.0 * sigma
        if int(np.sum(inliers)) >= MIN_RAW_SAMPLES:
            theta = _fit_ellipse_direct(normalized[inliers])
            center_n, w_unit_n = _conic_to_center_w(theta, field_radius=1.0)
            center, w_unit = _denormalize(center_n, w_unit_n, mu, scale)
            return center, w_unit, inliers
    center_n, w_unit_n = _conic_to_center_w(theta, field_radius=1.0)
    center, w_unit = _denormalize(center_n, w_unit_n, mu, scale)
    return center, w_unit, np.ones(len(points), dtype=bool)


def _unpack_refine_params(params: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    ox, oy, s11, c21, s22 = params
    center = np.array([ox, oy], dtype=float)
    calibration = np.array(
        [
            [math.exp(float(s11)), 0.0],
            [float(c21), math.exp(float(s22))],
        ],
        dtype=float,
    )
    return center, calibration


def _refine_solution(
    points: np.ndarray,
    center: np.ndarray,
    calibration: np.ndarray,
    field_radius: float,
    *,
    expected_noise_std: float | None,
) -> tuple[np.ndarray, np.ndarray, bool, float | None]:
    p0 = np.array(
        [
            center[0],
            center[1],
            math.log(max(float(calibration[0, 0]), FIT_EPS)),
            calibration[1, 0],
            math.log(max(float(calibration[1, 1]), FIT_EPS)),
        ],
        dtype=float,
    )

    def residual(params: np.ndarray) -> np.ndarray:
        refined_center, refined_calibration = _unpack_refine_params(params)
        return _radial_residual(points, refined_center, refined_calibration, field_radius)

    f_scale = expected_noise_std if expected_noise_std is not None else max(1e-6, 0.02 * field_radius)
    result = least_squares(residual, p0, loss="soft_l1", f_scale=f_scale, max_nfev=200)
    refined_center, refined_calibration = _unpack_refine_params(result.x)
    return refined_center, refined_calibration, bool(result.success), float(result.cost)


def _angular_coverage_rad(points_xy: np.ndarray) -> float:
    if len(points_xy) < 2:
        return 0.0
    center = np.median(points_xy, axis=0)
    angles = np.sort((np.arctan2(points_xy[:, 1] - center[1], points_xy[:, 0] - center[0]) + 2.0 * np.pi) % (2.0 * np.pi))
    gaps = np.diff(np.r_[angles, angles[0] + 2.0 * np.pi])
    return float(2.0 * np.pi - np.max(gaps))


def calibrate(dataset, config=None):
    global _last_report
    points3 = _extract_points(dataset, config)
    if len(points3) < MIN_RAW_SAMPLES:
        raise ValueError(f"need at least {MIN_RAW_SAMPLES} raw_magnetometer samples for calibration")

    points = points3[:, :2]
    requested_radius = _config_float(config, "field_radius", "horizontal_field_radius", "B_h", "b_h")
    robust = str(config.get("robust", "irls") if isinstance(config, dict) else "irls")
    max_iter = _config_int(config, "max_iter", DEFAULT_MAX_ITER)
    expected_noise_std = _config_float(config, "expected_noise_std", "noise_std")

    center, w_unit, inliers = _fit_center_w_unit(points, robust=robust, max_iter=max_iter)
    field_radius_source = "config"
    if requested_radius is None:
        requested_radius = _axis_radius_from_unit_w(w_unit)
        field_radius_source = "estimated_major_axis"
    field_radius = float(requested_radius)
    w_matrix = w_unit * (field_radius * field_radius)
    a_scale, b_scale, rho, forward_matrix, calibration = _extract_ets_params(w_matrix)

    optimizer_success = False
    optimizer_cost = None
    refined = _config_bool(config, "refine", False)
    if refined:
        center, calibration, optimizer_success, optimizer_cost = _refine_solution(
            points,
            center,
            calibration,
            field_radius,
            expected_noise_std=expected_noise_std,
        )
        w_matrix = calibration.T @ calibration
        a_scale, b_scale, rho, forward_matrix, _ = _extract_ets_params(w_matrix)

    residual = _radial_residual(points, center, calibration, field_radius)
    rms = math.sqrt(float(np.mean(residual * residual)))
    max_abs = float(np.max(np.abs(residual)))
    rel_rms = rms / field_radius if field_radius > FIT_EPS else float("inf")
    condition_number = float(np.linalg.cond(w_matrix))
    coverage = _angular_coverage_rad(points)

    _last_report = (
        f"Raw samples: {len(points)}\n"
        f"Inliers: {int(np.sum(inliers))}\n"
        f"Offset: ({center[0]:.6f}, {center[1]:.6f})\n"
        f"Field radius: {field_radius:.6f} ({field_radius_source})\n"
        f"ETS a: {a_scale:.6f}\n"
        f"ETS b: {b_scale:.6f}\n"
        f"ETS rho_deg: {math.degrees(rho):.6f}\n"
        f"Angular coverage deg: {math.degrees(coverage):.6f}\n"
        f"Condition number: {condition_number:.6f}\n"
        f"RMS residual: {rms:.6f}\n"
        f"Relative RMS residual: {rel_rms:.6f}\n"
        f"Max abs residual: {max_abs:.6f}\n"
        f"Refined: {refined}\n"
        f"Optimizer success: {optimizer_success}"
    )

    c00, c01 = float(calibration[0, 0]), float(calibration[0, 1])
    c10, c11 = float(calibration[1, 0]), float(calibration[1, 1])
    return {
        "algorithm_name": METHOD_NAME,
        "algorithm_version": METHOD_VERSION,
        "schema_version": SCHEMA_VERSION,
        "created_at": _utc_now(),
        "params": {
            "offset_x": float(center[0]),
            "offset_y": float(center[1]),
            "c00": c00,
            "c01": c01,
            "c10": c10,
            "c11": c11,
            "w00": float(w_matrix[0, 0]),
            "w01": float(w_matrix[0, 1]),
            "w10": float(w_matrix[1, 0]),
            "w11": float(w_matrix[1, 1]),
            "field_radius": field_radius,
            "field_radius_source": field_radius_source,
            "ets_a": float(a_scale),
            "ets_b": float(b_scale),
            "ets_rho_rad": float(rho),
            "ets_rho_deg": float(math.degrees(rho)),
            "forward_l00": float(forward_matrix[0, 0]),
            "forward_l01": float(forward_matrix[0, 1]),
            "forward_l10": float(forward_matrix[1, 0]),
            "forward_l11": float(forward_matrix[1, 1]),
            "sample_count": int(len(points)),
            "inlier_count": int(np.sum(inliers)),
            "rms_residual": rms,
            "max_abs_residual": max_abs,
            "rel_rms_residual": rel_rms,
            "coverage_rad": coverage,
            "coverage_deg": float(math.degrees(coverage)),
            "condition_number": condition_number,
            "refined": bool(refined),
            "optimizer_success": bool(optimizer_success),
            "optimizer_cost": optimizer_cost,
            "z_policy": "passthrough",
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
    payload.setdefault(
        "params",
        {
            "offset_x": 0.0,
            "offset_y": 0.0,
            "c00": 1.0,
            "c01": 0.0,
            "c10": 0.0,
            "c11": 1.0,
            "field_radius": 1.0,
        },
    )
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)


def process(sample, params):
    inner = params.get("params", {}) if isinstance(params, dict) else {}
    offset = np.array(
        [
            float(inner.get("offset_x", 0.0)),
            float(inner.get("offset_y", 0.0)),
        ],
        dtype=float,
    )
    calibration = np.array(
        [
            [float(inner.get("c00", 1.0)), float(inner.get("c01", 0.0))],
            [float(inner.get("c10", 0.0)), float(inner.get("c11", 1.0))],
        ],
        dtype=float,
    )
    raw_xy = np.array([float(sample["mag_x"]), float(sample["mag_y"])], dtype=float)
    corrected_xy = calibration @ (raw_xy - offset)
    mag_x = float(corrected_xy[0])
    mag_y = float(corrected_xy[1])
    mag_z = float(sample["mag_z"])

    heading = None
    if not (math.isclose(mag_x, 0.0, abs_tol=1e-9) and math.isclose(mag_y, 0.0, abs_tol=1e-9)):
        heading = (math.degrees(math.atan2(mag_x, mag_y)) + 360.0) % 360.0

    return {
        "timestamp_mcu": sample["timestamp_mcu"],
        "timestamp_pc_rx": sample["timestamp_pc_rx"],
        "timestamp_pc_est": sample.get("timestamp_pc_est"),
        "mag_x": mag_x,
        "mag_y": mag_y,
        "mag_z": mag_z,
        "heading": heading,
        "flags": "ets_2d_rt",
    }
