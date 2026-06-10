from __future__ import annotations

import json
import math
from datetime import datetime, timezone
from typing import Any

import numpy as np


METHOD_NAME = "Papafotis MAG.I.C.AL. Magnetometer Method"
METHOD_VERSION = "1.0.0"
SCHEMA_VERSION = "1"

MIN_RAW_SAMPLES = 12
FIT_EPS = 1e-12
COVERAGE_GOOD_RATIO = 0.20
COVERAGE_LIMITED_RATIO = 0.05

_last_report = ""


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")


def get_info() -> dict[str, Any]:
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
                    "description": "Raw magnetometer vector used for Papafotis static-position calibration.",
                },
                {
                    "slot": "accel_input",
                    "kind": "imu.accel_vector",
                    "required": False,
                    "label": "Accelerometer input",
                    "description": "Optional accelerometer vector used to improve quasi-static segment detection.",
                },
                {
                    "slot": "gyro_input",
                    "kind": "imu.gyro_vector",
                    "required": False,
                    "label": "Gyroscope input",
                    "description": "Optional gyroscope vector used as the primary stationarity signal.",
                },
            ],
            "process": [
                {
                    "slot": "mag_input",
                    "kind": "imu.magnetometer_vector",
                    "required": True,
                    "label": "Magnetometer input",
                    "description": "Magnetometer vector to correct with m_cal = H_m @ y_raw + v_m.",
                }
            ],
        },
        "config_schema": [
            {"key": "window_s", "type": "float", "label": "Window s", "description": "Sliding stationarity window duration."},
            {"key": "step_s", "type": "float", "label": "Step s", "description": "Sliding stationarity window step."},
            {"key": "min_segment_s", "type": "float", "label": "Min segment s"},
            {"key": "merge_gap_s", "type": "float", "label": "Merge gap s"},
            {"key": "trim_edge_s", "type": "float", "label": "Trim edge s"},
            {"key": "min_samples_per_segment", "type": "int", "label": "Min samples/segment"},
            {"key": "min_static_segments", "type": "int", "label": "Min static segments"},
            {"key": "mag_norm_std_threshold", "type": "float", "label": "Mag norm std threshold"},
            {"key": "mag_axis_std_threshold", "type": "float", "label": "Mag axis std threshold"},
            {"key": "acc_norm_std_threshold", "type": "float", "label": "Accel norm std threshold"},
            {"key": "acc_axis_std_threshold", "type": "float", "label": "Accel axis std threshold"},
            {"key": "gyro_rms_threshold", "type": "float", "label": "Gyro RMS threshold"},
            {"key": "max_sync_delta_ms", "type": "float", "label": "Max sync delta ms", "description": "Max timestamp delta for optional accel/gyro alignment. Empty disables the limit."},
            {"key": "trim_ratio", "type": "float", "label": "Robust trim ratio"},
            {"key": "duplicate_angle_deg", "type": "float", "label": "Duplicate angle deg"},
            {"key": "max_points", "type": "int", "label": "Max selected points", "description": "0 keeps all non-duplicate static points."},
            {"key": "field_radius", "type": "float", "label": "Output field radius", "description": "Leave empty for normalized unit output."},
            {"key": "init", "type": "choice", "label": "Initialization", "choices": ["ellipsoid", "identity"]},
            {"key": "max_iter", "type": "int", "label": "Max ALS iterations"},
            {"key": "tol_norm", "type": "float", "label": "Norm tolerance"},
            {"key": "tol_improvement", "type": "float", "label": "Improvement tolerance"},
            {"key": "fallback_to_raw_samples", "type": "bool", "label": "Fallback to raw samples"},
        ],
    }


def get_default_config() -> dict[str, object]:
    return {
        "window_s": 1.0,
        "step_s": 0.2,
        "min_segment_s": 1.0,
        "merge_gap_s": 0.3,
        "trim_edge_s": 0.2,
        "min_samples_per_segment": 8,
        "min_static_segments": 12,
        "mag_norm_std_threshold": 1.0,
        "mag_axis_std_threshold": 2.0,
        "acc_norm_std_threshold": 0.03,
        "acc_axis_std_threshold": 0.03,
        "gyro_rms_threshold": 3.0,
        "max_sync_delta_ms": None,
        "trim_ratio": 0.1,
        "duplicate_angle_deg": 5.0,
        "max_points": 0,
        "field_radius": None,
        "init": "ellipsoid",
        "max_iter": 100,
        "tol_norm": 1e-5,
        "tol_improvement": 1e-7,
        "fallback_to_raw_samples": False,
    }


def validate_dataset(dataset: Any):
    if dataset is None or not hasattr(dataset, "records"):
        return {"ok": False, "warnings": ["dataset object is missing records"]}
    mag_records = [
        record
        for record in dataset.records
        if not _is_heading_only(record)
        and _vector(record, (("mag_x", "mag_y", "mag_z"), ("magn_x", "magn_y", "magn_z"))) is not None
    ]
    if len(mag_records) < MIN_RAW_SAMPLES:
        return {"ok": False, "warnings": [f"need at least {MIN_RAW_SAMPLES} magnetometer samples"]}
    return None


def _value(record: Any, name: str, default: Any = None) -> Any:
    if isinstance(record, dict):
        return record.get(name, default)
    return getattr(record, name, default)


def _safe_float(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _config_sources(config: Any) -> list[dict[str, Any]]:
    if not isinstance(config, dict):
        return []
    sources = [config]
    params = config.get("params")
    if isinstance(params, dict):
        sources.append(params)
    method_config = config.get("method_config")
    if isinstance(method_config, dict):
        sources.append(method_config)
    return sources


def _config_float(config: Any, key: str, default: float | None = None, *, positive: bool = False) -> float | None:
    for source in _config_sources(config):
        if key not in source or source[key] in (None, ""):
            continue
        value = _safe_float(source[key])
        if value is None:
            continue
        if positive and value <= 0.0:
            continue
        return value
    return default


def _config_int(config: Any, key: str, default: int, *, minimum: int = 0) -> int:
    for source in _config_sources(config):
        if key not in source or source[key] in (None, ""):
            continue
        try:
            return max(minimum, int(source[key]))
        except (TypeError, ValueError):
            continue
    return default


def _config_bool(config: Any, key: str, default: bool) -> bool:
    for source in _config_sources(config):
        if key not in source:
            continue
        value = source[key]
        if isinstance(value, str):
            return value.strip().lower() in {"1", "true", "yes", "on"}
        return bool(value)
    return default


def _config_str(config: Any, key: str, default: str) -> str:
    for source in _config_sources(config):
        if key in source and source[key] not in (None, ""):
            return str(source[key])
    return default


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


def _dataset_records(dataset: Any, stream_ids: tuple[str, ...]) -> list[Any]:
    if dataset is None or not hasattr(dataset, "records"):
        return []
    return [record for record in dataset.records if str(_value(record, "stream_id", "")) in stream_ids]


def _timestamp_ms(record: Any, fallback: int) -> float:
    for key in ("timestamp_mcu", "timestamp_pc_est", "timestamp_pc_rx"):
        value = _safe_float(_value(record, key))
        if value is not None:
            return value
    return float(fallback)


def _vector(record: Any, names: tuple[tuple[str, str, str], ...]) -> tuple[float, float, float] | None:
    for x_name, y_name, z_name in names:
        x_val = _safe_float(_value(record, x_name))
        y_val = _safe_float(_value(record, y_name))
        z_val = _safe_float(_value(record, z_name))
        if x_val is None or y_val is None or z_val is None:
            continue
        return (x_val, y_val, z_val)
    return None


def _is_heading_only(record: Any) -> bool:
    return "heading_only" in str(_value(record, "flags", "") or "")


def _mag_records(dataset: Any, config: Any) -> list[Any]:
    configured = _stream_records(config, "mag_input")
    if configured:
        return configured
    return [
        record
        for record in _dataset_records(dataset, ("raw_magnetometer",))
        if not _is_heading_only(record)
    ]


def _sensor_records(dataset: Any, config: Any, slot: str, stream_ids: tuple[str, ...]) -> list[Any]:
    configured = _stream_records(config, slot)
    if configured:
        return configured
    return _dataset_records(dataset, stream_ids)


def _extract_mag_series(dataset: Any, config: Any) -> tuple[np.ndarray, np.ndarray, list[Any], np.ndarray | None, np.ndarray | None]:
    rows: list[tuple[float, tuple[float, float, float], Any, tuple[float, float, float] | None, tuple[float, float, float] | None]] = []
    for index, record in enumerate(_mag_records(dataset, config)):
        if _is_heading_only(record):
            continue
        mag = _vector(record, (("mag_x", "mag_y", "mag_z"), ("magn_x", "magn_y", "magn_z")))
        if mag is None:
            continue
        if math.isclose(mag[0], 0.0, abs_tol=FIT_EPS) and math.isclose(mag[1], 0.0, abs_tol=FIT_EPS) and math.isclose(mag[2], 0.0, abs_tol=FIT_EPS):
            continue
        rows.append(
            (
                _timestamp_ms(record, index),
                mag,
                record,
                _vector(record, (("acc_x", "acc_y", "acc_z"), ("accel_x", "accel_y", "accel_z"))),
                _vector(record, (("gyro_x", "gyro_y", "gyro_z"),)),
            )
        )
    rows.sort(key=lambda item: item[0])
    if not rows:
        return (
            np.empty((0,), dtype=float),
            np.empty((0, 3), dtype=float),
            [],
            None,
            None,
        )

    times = np.asarray([row[0] for row in rows], dtype=float)
    mag = np.asarray([row[1] for row in rows], dtype=float)
    records = [row[2] for row in rows]

    direct_acc_values = [row[3] for row in rows]
    direct_gyro_values = [row[4] for row in rows]
    direct_acc = None
    direct_gyro = None
    if sum(value is not None for value in direct_acc_values) >= max(3, len(rows) // 2):
        direct_acc = np.asarray(
            [value if value is not None else (float("nan"), float("nan"), float("nan")) for value in direct_acc_values],
            dtype=float,
        )
    if sum(value is not None for value in direct_gyro_values) >= max(3, len(rows) // 2):
        direct_gyro = np.asarray(
            [value if value is not None else (float("nan"), float("nan"), float("nan")) for value in direct_gyro_values],
            dtype=float,
        )
    return times, mag, records, direct_acc, direct_gyro


def _extract_sensor_series(records: list[Any], names: tuple[tuple[str, str, str], ...]) -> tuple[np.ndarray, np.ndarray]:
    rows: list[tuple[float, tuple[float, float, float]]] = []
    for index, record in enumerate(records):
        vector = _vector(record, names)
        if vector is None:
            continue
        rows.append((_timestamp_ms(record, index), vector))
    rows.sort(key=lambda item: item[0])
    if not rows:
        return np.empty((0,), dtype=float), np.empty((0, 3), dtype=float)
    return np.asarray([row[0] for row in rows], dtype=float), np.asarray([row[1] for row in rows], dtype=float)


def _align_series(
    target_times_ms: np.ndarray,
    sensor_times_ms: np.ndarray,
    sensor_values: np.ndarray,
    *,
    max_delta_ms: float | None,
) -> np.ndarray | None:
    if len(target_times_ms) == 0 or len(sensor_times_ms) == 0:
        return None
    aligned = np.full((len(target_times_ms), 3), np.nan, dtype=float)
    indices = np.searchsorted(sensor_times_ms, target_times_ms)
    for row, target_time in enumerate(target_times_ms):
        candidates: list[int] = []
        idx = int(indices[row])
        if idx < len(sensor_times_ms):
            candidates.append(idx)
        if idx > 0:
            candidates.append(idx - 1)
        if not candidates:
            continue
        best = min(candidates, key=lambda candidate: abs(float(sensor_times_ms[candidate] - target_time)))
        delta = abs(float(sensor_times_ms[best] - target_time))
        if max_delta_ms is not None and delta > max_delta_ms:
            continue
        aligned[row, :] = sensor_values[best, :]
    if int(np.sum(np.all(np.isfinite(aligned), axis=1))) < 3:
        return None
    return aligned


def _optional_sensor_arrays(
    dataset: Any,
    config: Any,
    mag_times_ms: np.ndarray,
    direct_acc: np.ndarray | None,
    direct_gyro: np.ndarray | None,
) -> tuple[np.ndarray | None, np.ndarray | None]:
    max_delta_ms = _config_float(config, "max_sync_delta_ms", None, positive=True)

    accel = direct_acc
    if accel is None:
        accel_records = _sensor_records(dataset, config, "accel_input", ("raw_accelerometer", "raw_tilt", "imu_accel"))
        acc_times, acc_values = _extract_sensor_series(
            accel_records,
            (("acc_x", "acc_y", "acc_z"), ("accel_x", "accel_y", "accel_z")),
        )
        accel = _align_series(mag_times_ms, acc_times, acc_values, max_delta_ms=max_delta_ms)

    gyro = direct_gyro
    if gyro is None:
        gyro_records = _sensor_records(dataset, config, "gyro_input", ("raw_gyroscope", "imu_gyro"))
        gyro_times, gyro_values = _extract_sensor_series(gyro_records, (("gyro_x", "gyro_y", "gyro_z"),))
        gyro = _align_series(mag_times_ms, gyro_times, gyro_values, max_delta_ms=max_delta_ms)

    return accel, gyro


def _std(values: np.ndarray) -> float:
    finite = values[np.isfinite(values)]
    if len(finite) < 2:
        return float("inf")
    return float(np.std(finite, ddof=0))


def _axis_max_std(values: np.ndarray) -> float:
    if len(values) < 2:
        return float("inf")
    axis_std: list[float] = []
    for axis in range(values.shape[1]):
        axis_std.append(_std(values[:, axis]))
    return max(axis_std)


def _window_feature_rows(
    times_s: np.ndarray,
    mag: np.ndarray,
    accel: np.ndarray | None,
    gyro: np.ndarray | None,
    config: Any,
) -> list[dict[str, Any]]:
    if len(times_s) < 3:
        return []
    window_s = max(0.02, float(_config_float(config, "window_s", 1.0, positive=True) or 1.0))
    step_s = max(0.01, float(_config_float(config, "step_s", 0.2, positive=True) or 0.2))
    mag_norm_threshold = float(_config_float(config, "mag_norm_std_threshold", 1.0, positive=True) or 1.0)
    mag_axis_threshold = float(_config_float(config, "mag_axis_std_threshold", 2.0, positive=True) or 2.0)
    acc_norm_threshold = float(_config_float(config, "acc_norm_std_threshold", 0.03, positive=True) or 0.03)
    acc_axis_threshold = float(_config_float(config, "acc_axis_std_threshold", 0.03, positive=True) or 0.03)
    gyro_threshold = float(_config_float(config, "gyro_rms_threshold", 3.0, positive=True) or 3.0)

    t_min = float(times_s[0])
    t_max = float(times_s[-1])
    if t_max - t_min < window_s:
        starts = np.asarray([t_min], dtype=float)
        window_s = max(0.0, t_max - t_min)
    else:
        starts = np.arange(t_min, t_max - window_s + step_s * 0.5, step_s)

    rows: list[dict[str, Any]] = []
    min_window_samples = 3
    for start in starts:
        end = float(start + window_s)
        i0 = int(np.searchsorted(times_s, start, side="left"))
        i1 = int(np.searchsorted(times_s, end, side="right"))
        if i1 - i0 < min_window_samples:
            continue

        mag_window = mag[i0:i1]
        mag_norm = np.linalg.norm(mag_window, axis=1)
        mag_norm_std = _std(mag_norm)
        mag_axis_std = _axis_max_std(mag_window)

        static = mag_norm_std < mag_norm_threshold and mag_axis_std < mag_axis_threshold

        acc_norm_std = None
        acc_axis_std = None
        if accel is not None:
            acc_window = accel[i0:i1]
            valid = np.all(np.isfinite(acc_window), axis=1)
            if int(np.sum(valid)) >= min_window_samples:
                acc_valid = acc_window[valid]
                acc_norm_std = _std(np.linalg.norm(acc_valid, axis=1))
                acc_axis_std = _axis_max_std(acc_valid)
                static = static and acc_norm_std < acc_norm_threshold and acc_axis_std < acc_axis_threshold

        gyro_rms = None
        if gyro is not None:
            gyro_window = gyro[i0:i1]
            valid = np.all(np.isfinite(gyro_window), axis=1)
            if int(np.sum(valid)) >= min_window_samples:
                gyro_norm2 = np.sum(gyro_window[valid] * gyro_window[valid], axis=1)
                gyro_rms = math.sqrt(float(np.mean(gyro_norm2)))
                static = static and gyro_rms < gyro_threshold

        rows.append(
            {
                "start_s": float(start),
                "end_s": end,
                "i0": i0,
                "i1": i1,
                "n_samples": i1 - i0,
                "static": bool(static),
                "gyro_rms": gyro_rms,
                "acc_norm_std": acc_norm_std,
                "acc_axis_max_std": acc_axis_std,
                "mag_norm_std": mag_norm_std,
                "mag_axis_max_std": mag_axis_std,
            }
        )
    return rows


def _sample_static_mask(times_s: np.ndarray, windows: list[dict[str, Any]]) -> np.ndarray:
    mask = np.zeros(len(times_s), dtype=bool)
    for row in windows:
        if not row["static"]:
            continue
        mask[int(row["i0"]):int(row["i1"])] = True
    return mask


def _segments_from_mask(times_s: np.ndarray, mask: np.ndarray, config: Any) -> list[dict[str, Any]]:
    if len(mask) == 0 or not np.any(mask):
        return []
    merge_gap_s = float(_config_float(config, "merge_gap_s", 0.3, positive=True) or 0.3)
    min_segment_s = float(_config_float(config, "min_segment_s", 1.0, positive=True) or 1.0)
    trim_edge_s = float(_config_float(config, "trim_edge_s", 0.2) or 0.0)
    min_samples = _config_int(config, "min_samples_per_segment", 8, minimum=1)

    runs: list[tuple[int, int]] = []
    start: int | None = None
    for index, is_static in enumerate(mask):
        if is_static and start is None:
            start = index
        if start is not None and (not is_static or index == len(mask) - 1):
            end = index if is_static and index == len(mask) - 1 else index - 1
            runs.append((start, end))
            start = None

    merged: list[list[int]] = []
    for run_start, run_end in runs:
        if not merged:
            merged.append([run_start, run_end])
            continue
        gap_s = float(times_s[run_start] - times_s[merged[-1][1]])
        if gap_s <= merge_gap_s:
            merged[-1][1] = run_end
        else:
            merged.append([run_start, run_end])

    segments: list[dict[str, Any]] = []
    for run_start, run_end in merged:
        raw_start_s = float(times_s[run_start])
        raw_end_s = float(times_s[run_end])
        raw_duration = max(0.0, raw_end_s - raw_start_s)
        if raw_duration < min_segment_s:
            continue

        trim_start_s = raw_start_s + max(0.0, trim_edge_s)
        trim_end_s = raw_end_s - max(0.0, trim_edge_s)
        if trim_end_s <= trim_start_s:
            trim_start_s = raw_start_s
            trim_end_s = raw_end_s
        i0 = int(np.searchsorted(times_s, trim_start_s, side="left"))
        i1 = int(np.searchsorted(times_s, trim_end_s, side="right"))
        i0 = max(run_start, min(i0, run_end + 1))
        i1 = max(i0, min(i1, run_end + 1))
        if i1 - i0 < min_samples:
            continue
        segments.append(
            {
                "segment_id": len(segments),
                "start_index": i0,
                "end_index": i1,
                "raw_start_s": raw_start_s,
                "raw_end_s": raw_end_s,
                "start_s": float(times_s[i0]),
                "end_s": float(times_s[i1 - 1]),
                "duration_s": raw_duration,
                "n_samples": i1 - i0,
            }
        )
    return segments


def _trimmed_mean(values: np.ndarray, trim_ratio: float) -> np.ndarray:
    if len(values) == 0:
        return np.zeros(values.shape[1], dtype=float)
    ratio = min(0.45, max(0.0, float(trim_ratio)))
    drop = int(math.floor(len(values) * ratio))
    result: list[float] = []
    for axis in range(values.shape[1]):
        axis_values = np.sort(values[:, axis])
        if drop > 0 and len(axis_values) > 2 * drop:
            axis_values = axis_values[drop:-drop]
        result.append(float(np.mean(axis_values)))
    return np.asarray(result, dtype=float)


def _segment_points(
    segments: list[dict[str, Any]],
    mag: np.ndarray,
    accel: np.ndarray | None,
    gyro: np.ndarray | None,
    config: Any,
) -> tuple[np.ndarray, list[dict[str, Any]]]:
    trim_ratio = float(_config_float(config, "trim_ratio", 0.1) or 0.0)
    points: list[np.ndarray] = []
    summaries: list[dict[str, Any]] = []
    for segment in segments:
        i0 = int(segment["start_index"])
        i1 = int(segment["end_index"])
        mag_values = mag[i0:i1]
        if len(mag_values) == 0:
            continue
        mag_mean = _trimmed_mean(mag_values, trim_ratio)
        mag_std = np.std(mag_values, axis=0, ddof=0) if len(mag_values) > 1 else np.zeros(3, dtype=float)
        mag_norm_std = _std(np.linalg.norm(mag_values, axis=1))

        acc_mean = None
        acc_std = None
        acc_norm_std = None
        if accel is not None:
            acc_values = accel[i0:i1]
            valid = np.all(np.isfinite(acc_values), axis=1)
            if int(np.sum(valid)) >= 2:
                acc_valid = acc_values[valid]
                acc_mean = _trimmed_mean(acc_valid, trim_ratio)
                acc_std = np.std(acc_valid, axis=0, ddof=0)
                acc_norm_std = _std(np.linalg.norm(acc_valid, axis=1))

        gyro_rms = None
        if gyro is not None:
            gyro_values = gyro[i0:i1]
            valid = np.all(np.isfinite(gyro_values), axis=1)
            if int(np.sum(valid)) >= 2:
                gyro_norm2 = np.sum(gyro_values[valid] * gyro_values[valid], axis=1)
                gyro_rms = math.sqrt(float(np.mean(gyro_norm2)))

        summary = {
            **segment,
            "mx_mean": float(mag_mean[0]),
            "my_mean": float(mag_mean[1]),
            "mz_mean": float(mag_mean[2]),
            "mx_std": float(mag_std[0]),
            "my_std": float(mag_std[1]),
            "mz_std": float(mag_std[2]),
            "mag_norm_std": float(mag_norm_std),
            "mag_axis_max_std": float(max(mag_std)),
            "gyro_rms": gyro_rms,
            "acc_norm_std": acc_norm_std,
            "acc_axis_max_std": None if acc_std is None else float(max(acc_std)),
        }
        if acc_mean is not None and acc_std is not None:
            summary.update(
                {
                    "ax_mean": float(acc_mean[0]),
                    "ay_mean": float(acc_mean[1]),
                    "az_mean": float(acc_mean[2]),
                    "ax_std": float(acc_std[0]),
                    "ay_std": float(acc_std[1]),
                    "az_std": float(acc_std[2]),
                }
            )
        points.append(mag_mean)
        summaries.append(summary)
    if not points:
        return np.empty((0, 3), dtype=float), []
    return np.vstack(points), summaries


def _unit_directions(points: np.ndarray) -> np.ndarray:
    norms = np.linalg.norm(points, axis=1)
    valid = norms > FIT_EPS
    if not np.all(valid):
        points = points[valid]
        norms = norms[valid]
    if len(points) == 0:
        return np.empty((0, 3), dtype=float)
    return points / norms[:, None]


def _angle_rad(a: np.ndarray, b: np.ndarray) -> float:
    dot = float(np.clip(a @ b, -1.0, 1.0))
    return float(math.acos(dot))


def _quality_score(summary: dict[str, Any]) -> float:
    mag_axis = _safe_float(summary.get("mag_axis_max_std")) or 0.0
    mag_norm = _safe_float(summary.get("mag_norm_std")) or 0.0
    duration = max(_safe_float(summary.get("duration_s")) or 0.0, FIT_EPS)
    return mag_axis + mag_norm + 1.0 / duration


def _select_diverse_points(
    points: np.ndarray,
    summaries: list[dict[str, Any]],
    config: Any,
) -> tuple[np.ndarray, list[dict[str, Any]], list[int]]:
    if len(points) == 0:
        return points, summaries, []
    min_angle = math.radians(float(_config_float(config, "duplicate_angle_deg", 5.0) or 0.0))
    directions = _unit_directions(points)
    if len(directions) != len(points):
        return points, summaries, list(range(len(points)))

    order = sorted(range(len(points)), key=lambda idx: (_quality_score(summaries[idx]), idx))
    kept: list[int] = []
    for idx in order:
        direction = directions[idx]
        if all(_angle_rad(direction, directions[other]) >= min_angle for other in kept):
            kept.append(idx)
    kept.sort()

    max_points = _config_int(config, "max_points", 0, minimum=0)
    if max_points > 0 and len(kept) > max_points:
        kept = _farthest_point_indices(directions[kept], max_points, kept)
        kept.sort()
    return points[kept], [summaries[idx] for idx in kept], kept


def _farthest_point_indices(directions: np.ndarray, max_points: int, source_indices: list[int]) -> list[int]:
    if len(directions) <= max_points:
        return list(source_indices)
    center = np.mean(directions, axis=0)
    first = int(np.argmax(np.linalg.norm(directions - center, axis=1)))
    chosen_local = [first]
    min_dist = np.linalg.norm(directions - directions[first], axis=1)
    while len(chosen_local) < max_points:
        next_idx = int(np.argmax(min_dist))
        if next_idx in chosen_local:
            break
        chosen_local.append(next_idx)
        candidate_dist = np.linalg.norm(directions - directions[next_idx], axis=1)
        min_dist = np.minimum(min_dist, candidate_dist)
    return [source_indices[idx] for idx in chosen_local]


def _coverage_stats(points: np.ndarray) -> dict[str, Any]:
    directions = _unit_directions(points)
    if len(directions) < 3:
        singular = [0.0, 0.0, 0.0]
        ratio = 0.0
    else:
        centered = directions - np.mean(directions, axis=0)
        values = np.linalg.svd(centered, compute_uv=False)
        padded = list(values) + [0.0, 0.0, 0.0]
        singular = [float(padded[0]), float(padded[1]), float(padded[2])]
        ratio = 0.0 if singular[0] <= FIT_EPS else float(singular[2] / singular[0])
    if ratio >= COVERAGE_GOOD_RATIO:
        quality = "good_3d"
    elif ratio >= COVERAGE_LIMITED_RATIO:
        quality = "limited_3d"
    else:
        quality = "nearly_planar"
    return {
        "singular_values": singular,
        "s3_s1": ratio,
        "quality": quality,
    }


def _normalize_points(points: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    mu = points.mean(axis=0)
    scale = points.std(axis=0)
    scale[scale <= FIT_EPS] = 1.0
    return (points - mu) / scale, mu, scale


def _fit_ellipsoid_svd(points: np.ndarray) -> np.ndarray:
    x_val = points[:, 0]
    y_val = points[:, 1]
    z_val = points[:, 2]
    design = np.column_stack(
        [
            x_val * x_val,
            y_val * y_val,
            z_val * z_val,
            2.0 * x_val * y_val,
            2.0 * x_val * z_val,
            2.0 * y_val * z_val,
            2.0 * x_val,
            2.0 * y_val,
            2.0 * z_val,
            np.ones_like(x_val),
        ]
    )
    _, _, vh = np.linalg.svd(design, full_matrices=False)
    theta = vh[-1]
    norm = float(np.linalg.norm(theta))
    if norm <= FIT_EPS or not np.all(np.isfinite(theta)):
        raise ValueError("ellipsoid initialization is degenerate")
    return theta / norm


def _quadric_to_center_w(theta: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    a_coef, b_coef, c_coef, d_coef, e_coef, f_coef, g_coef, h_coef, i_coef, j_coef = [
        float(value) for value in theta
    ]
    q_matrix = np.array(
        [
            [a_coef, d_coef, e_coef],
            [d_coef, b_coef, f_coef],
            [e_coef, f_coef, c_coef],
        ],
        dtype=float,
    )
    g_vector = np.array([g_coef, h_coef, i_coef], dtype=float)

    if np.any(np.linalg.eigvalsh(q_matrix) <= FIT_EPS):
        q_matrix = -q_matrix
        g_vector = -g_vector
        j_coef = -j_coef
    if np.any(np.linalg.eigvalsh(q_matrix) <= FIT_EPS):
        raise ValueError("ellipsoid initialization did not produce an ellipsoid")

    center = -np.linalg.solve(q_matrix, g_vector)
    radius_param = float(center @ q_matrix @ center - j_coef)
    if radius_param <= FIT_EPS:
        raise ValueError("ellipsoid initialization normalization is invalid")
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


def _identity_initialization(points: np.ndarray) -> tuple[np.ndarray, np.ndarray, str, str]:
    center = np.mean(points, axis=0)
    centered = points - center
    radii = np.linalg.norm(centered, axis=1)
    radius = float(np.median(radii[radii > FIT_EPS])) if np.any(radii > FIT_EPS) else 1.0
    h_matrix = np.eye(3, dtype=float) / max(radius, FIT_EPS)
    v_vector = -h_matrix @ center
    return h_matrix, v_vector, "identity", ""


def _ellipsoid_initialization(points: np.ndarray) -> tuple[np.ndarray, np.ndarray, str, str]:
    normalized, mu, scale = _normalize_points(points)
    theta = _fit_ellipsoid_svd(normalized)
    center_n, w_n = _quadric_to_center_w(theta)
    center, w_matrix = _denormalize(center_n, w_n, mu, scale)
    h_matrix = _sqrt_spd(w_matrix)
    v_vector = -h_matrix @ center
    return h_matrix, v_vector, "ellipsoid", ""


def _initial_calibration(points: np.ndarray, config: Any) -> tuple[np.ndarray, np.ndarray, str, str]:
    init = _config_str(config, "init", "ellipsoid").strip().lower()
    if init != "identity":
        try:
            return _ellipsoid_initialization(points)
        except Exception as exc:
            h_matrix, v_vector, _, _ = _identity_initialization(points)
            return h_matrix, v_vector, "identity", f"ellipsoid initialization failed: {exc}"
    return _identity_initialization(points)


def _solve_affine(q_matrix: np.ndarray, m_matrix: np.ndarray) -> tuple[np.ndarray, bool, float]:
    rank = int(np.linalg.matrix_rank(q_matrix, tol=1e-10))
    if rank >= 4:
        solution = np.linalg.lstsq(q_matrix, m_matrix, rcond=None)[0]
        return solution.T, False, 0.0

    ridge = 1e-8 * max(1.0, float(np.trace(q_matrix.T @ q_matrix)))
    regularizer = ridge * np.eye(q_matrix.shape[1], dtype=float)
    regularizer[-1, -1] *= 0.01
    solution = np.linalg.solve(q_matrix.T @ q_matrix + regularizer, q_matrix.T @ m_matrix)
    return solution.T, True, float(ridge)


def _papafotis_als(
    points: np.ndarray,
    config: Any,
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    h_matrix, v_vector, init_source, init_warning = _initial_calibration(points, config)
    max_iter = _config_int(config, "max_iter", 100, minimum=1)
    tol_norm = float(_config_float(config, "tol_norm", 1e-5, positive=True) or 1e-5)
    tol_improvement = float(_config_float(config, "tol_improvement", 1e-7, positive=True) or 1e-7)

    q_matrix = np.column_stack([points, np.ones(len(points), dtype=float)])
    history: list[dict[str, float | bool]] = []
    prev_error: float | None = None
    used_ridge = False
    ridge_lambda = 0.0
    converged_reason = "max_iter"

    for iteration in range(1, max_iter + 1):
        z_matrix = (h_matrix @ points.T).T + v_vector
        norms = np.linalg.norm(z_matrix, axis=1)
        valid = norms > FIT_EPS
        if int(np.sum(valid)) < 4:
            raise ValueError("Papafotis ALS produced too few non-zero projected vectors")

        m_matrix = z_matrix[valid] / norms[valid, None]
        a_matrix, ridge, ridge_value = _solve_affine(q_matrix[valid], m_matrix)
        used_ridge = used_ridge or ridge
        ridge_lambda = max(ridge_lambda, ridge_value)
        h_matrix = a_matrix[:, :3]
        v_vector = a_matrix[:, 3]

        corrected = (h_matrix @ points.T).T + v_vector
        residual = np.linalg.norm(corrected, axis=1) - 1.0
        norm_error = math.sqrt(float(np.mean(residual * residual)))
        improvement = float("inf") if prev_error is None else abs(prev_error - norm_error)
        history.append(
            {
                "iteration": float(iteration),
                "norm_error": float(norm_error),
                "improvement": float(improvement if math.isfinite(improvement) else 0.0),
                "used_ridge": bool(ridge),
            }
        )
        if norm_error < tol_norm:
            converged_reason = "tol_norm"
            break
        if prev_error is not None and improvement < tol_improvement:
            converged_reason = "tol_improvement"
            break
        prev_error = norm_error

    diagnostics = {
        "initialization": init_source,
        "initialization_warning": init_warning,
        "iterations": len(history),
        "converged_reason": converged_reason,
        "used_ridge": used_ridge,
        "ridge_lambda": ridge_lambda,
        "history": history,
    }
    return h_matrix, v_vector, diagnostics


def _residual_stats(points: np.ndarray, h_matrix: np.ndarray, v_vector: np.ndarray, field_radius: float) -> dict[str, float]:
    corrected = (h_matrix @ points.T).T + v_vector
    radii = np.linalg.norm(corrected, axis=1)
    residual = radii - field_radius
    return {
        "mean_radius": float(np.mean(radii)),
        "std_radius": float(np.std(radii, ddof=0)),
        "rms_residual": math.sqrt(float(np.mean(residual * residual))),
        "max_abs_residual": float(np.max(np.abs(residual))),
    }


def _raw_fallback_points(mag: np.ndarray, config: Any) -> tuple[np.ndarray, list[dict[str, Any]], str]:
    min_segments = _config_int(config, "min_static_segments", 12, minimum=4)
    if len(mag) < min_segments:
        raise ValueError(f"need at least {min_segments} magnetometer samples for raw fallback")
    max_points = _config_int(config, "max_points", 0, minimum=0)
    target = max_points if max_points > 0 else max(min_segments, min(200, len(mag)))
    if len(mag) > target:
        indices = np.linspace(0, len(mag) - 1, target, dtype=int)
        points = mag[indices]
    else:
        indices = np.arange(len(mag), dtype=int)
        points = mag
    summaries = [
        {
            "segment_id": int(idx),
            "start_index": int(idx),
            "end_index": int(idx + 1),
            "start_s": 0.0,
            "end_s": 0.0,
            "duration_s": 0.0,
            "n_samples": 1,
            "mx_mean": float(point[0]),
            "my_mean": float(point[1]),
            "mz_mean": float(point[2]),
            "mag_norm_std": 0.0,
            "mag_axis_max_std": 0.0,
            "fallback_raw_sample": True,
        }
        for idx, point in zip(indices, points)
    ]
    return points, summaries, "raw_sample_fallback"


def calibrate(dataset, config=None):
    global _last_report

    times_ms, mag, records, direct_acc, direct_gyro = _extract_mag_series(dataset, config)
    del records
    if len(mag) < MIN_RAW_SAMPLES:
        raise ValueError(f"need at least {MIN_RAW_SAMPLES} magnetometer samples")

    accel, gyro = _optional_sensor_arrays(dataset, config, times_ms, direct_acc, direct_gyro)
    times_s = (times_ms - times_ms[0]) / 1000.0
    windows = _window_feature_rows(times_s, mag, accel, gyro, config)
    mask = _sample_static_mask(times_s, windows)
    segments = _segments_from_mask(times_s, mask, config)
    points, segment_summaries = _segment_points(segments, mag, accel, gyro, config)

    warnings: list[str] = []
    source = "static_segments"
    min_segments = _config_int(config, "min_static_segments", 12, minimum=4)
    if len(points) < min_segments:
        if _config_bool(config, "fallback_to_raw_samples", False):
            warnings.append(
                f"static detector found {len(points)} usable segments; falling back to evenly sampled raw points"
            )
            points, segment_summaries, source = _raw_fallback_points(mag, config)
        else:
            raise ValueError(
                f"need at least {min_segments} quasi-static segments, found {len(points)}; "
                "relax thresholds or enable fallback_to_raw_samples"
            )

    selected_points, selected_summaries, selected_indices = _select_diverse_points(points, segment_summaries, config)
    if len(selected_points) < min_segments:
        raise ValueError(
            f"need at least {min_segments} diverse quasi-static segments, selected {len(selected_points)} after duplicate removal"
        )

    coverage = _coverage_stats(selected_points)
    if coverage["quality"] == "limited_3d":
        warnings.append(
            f"orientation coverage is limited (s3/s1={coverage['s3_s1']:.4f}); full 3D calibration is a baseline"
        )
    elif coverage["quality"] == "nearly_planar":
        warnings.append(
            f"orientation coverage is nearly planar (s3/s1={coverage['s3_s1']:.4f}); full 3D calibration is poorly conditioned"
        )

    h_unit, v_unit, als = _papafotis_als(selected_points, config)
    output_radius = _config_float(config, "field_radius", None, positive=True)
    field_radius_source = "config"
    if output_radius is None:
        output_radius = 1.0
        field_radius_source = "normalized_unit"
    h_matrix = h_unit * float(output_radius)
    v_vector = v_unit * float(output_radius)

    stats = _residual_stats(selected_points, h_matrix, v_vector, float(output_radius))
    corrected_all = (h_matrix @ mag.T).T + v_vector
    all_radii = np.linalg.norm(corrected_all, axis=1)
    raw_norms = np.linalg.norm(mag, axis=1)
    condition_number = float(np.linalg.cond(h_matrix)) if np.all(np.isfinite(h_matrix)) else float("inf")

    init_warning = str(als.get("initialization_warning") or "")
    if init_warning:
        warnings.append(init_warning)
    if bool(als.get("used_ridge")):
        warnings.append("ALS used ridge regularization because selected points were rank-deficient")

    _last_report = (
        f"Papafotis MAG.I.C.AL. magnetometer calibration\n"
        f"Raw samples: {len(mag)}\n"
        f"Stationarity windows: {len(windows)}\n"
        f"Static windows: {sum(1 for row in windows if row['static'])}\n"
        f"Detected static segments: {len(segments)}\n"
        f"Usable static points: {len(points)} ({source})\n"
        f"Selected diverse points: {len(selected_points)}\n"
        f"Removed duplicate/static points: {max(0, len(points) - len(selected_points))}\n"
        f"Coverage s3/s1: {coverage['s3_s1']:.6f} ({coverage['quality']})\n"
        f"Initialization: {als['initialization']}\n"
        f"ALS iterations: {als['iterations']} ({als['converged_reason']})\n"
        f"Field radius: {float(output_radius):.6f} ({field_radius_source})\n"
        f"Condition number H_m: {condition_number:.6f}\n"
        f"Segment RMS residual: {stats['rms_residual']:.8f}\n"
        f"Segment std radius: {stats['std_radius']:.8f}\n"
        f"Segment max abs residual: {stats['max_abs_residual']:.8f}\n"
        f"Raw norm std all samples: {float(np.std(raw_norms, ddof=0)):.8f}\n"
        f"Calibrated norm std all samples: {float(np.std(all_radii, ddof=0)):.8f}\n"
        f"Accel used: {accel is not None}\n"
        f"Gyro used: {gyro is not None}\n"
        f"Warnings: {'; '.join(warnings) if warnings else '-'}"
    )

    return {
        "algorithm_name": METHOD_NAME,
        "algorithm_version": METHOD_VERSION,
        "schema_version": SCHEMA_VERSION,
        "created_at": _utc_now(),
        "params": {
            "h00": float(h_matrix[0, 0]),
            "h01": float(h_matrix[0, 1]),
            "h02": float(h_matrix[0, 2]),
            "h10": float(h_matrix[1, 0]),
            "h11": float(h_matrix[1, 1]),
            "h12": float(h_matrix[1, 2]),
            "h20": float(h_matrix[2, 0]),
            "h21": float(h_matrix[2, 1]),
            "h22": float(h_matrix[2, 2]),
            "v0": float(v_vector[0]),
            "v1": float(v_vector[1]),
            "v2": float(v_vector[2]),
            "field_radius": float(output_radius),
            "field_radius_source": field_radius_source,
            "raw_sample_count": int(len(mag)),
            "stationarity_window_count": int(len(windows)),
            "static_window_count": int(sum(1 for row in windows if row["static"])),
            "detected_segment_count": int(len(segments)),
            "used_point_count": int(len(selected_points)),
            "candidate_point_count": int(len(points)),
            "selected_source": source,
            "selected_indices": [int(value) for value in selected_indices],
            "coverage_singular_values": [float(value) for value in coverage["singular_values"]],
            "coverage_s3_s1": float(coverage["s3_s1"]),
            "coverage_quality": str(coverage["quality"]),
            "als_initialization": str(als["initialization"]),
            "als_iterations": int(als["iterations"]),
            "als_converged_reason": str(als["converged_reason"]),
            "als_used_ridge": bool(als["used_ridge"]),
            "als_ridge_lambda": float(als["ridge_lambda"]),
            "condition_number": condition_number,
            "mean_radius": float(stats["mean_radius"]),
            "std_radius": float(stats["std_radius"]),
            "rms_residual": float(stats["rms_residual"]),
            "max_abs_residual": float(stats["max_abs_residual"]),
            "raw_norm_std_all_samples": float(np.std(raw_norms, ddof=0)),
            "calibrated_norm_std_all_samples": float(np.std(all_radii, ddof=0)),
            "accel_used": bool(accel is not None),
            "gyro_used": bool(gyro is not None),
            "warnings": warnings,
        },
        "calibration_summary": {
            "segments": selected_summaries,
            "coverage": coverage,
            "als_history": als["history"],
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
    payload.setdefault(
        "params",
        {
            "h00": 1.0,
            "h01": 0.0,
            "h02": 0.0,
            "h10": 0.0,
            "h11": 1.0,
            "h12": 0.0,
            "h20": 0.0,
            "h21": 0.0,
            "h22": 1.0,
            "v0": 0.0,
            "v1": 0.0,
            "v2": 0.0,
            "field_radius": 1.0,
        },
    )
    payload.setdefault("calibration_summary", {})
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)


def _params_to_runtime(params: Any) -> tuple[np.ndarray, np.ndarray]:
    inner = params.get("params", {}) if isinstance(params, dict) else {}
    h_matrix = np.array(
        [
            [float(inner.get("h00", 1.0)), float(inner.get("h01", 0.0)), float(inner.get("h02", 0.0))],
            [float(inner.get("h10", 0.0)), float(inner.get("h11", 1.0)), float(inner.get("h12", 0.0))],
            [float(inner.get("h20", 0.0)), float(inner.get("h21", 0.0)), float(inner.get("h22", 1.0))],
        ],
        dtype=float,
    )
    v_vector = np.array(
        [
            float(inner.get("v0", 0.0)),
            float(inner.get("v1", 0.0)),
            float(inner.get("v2", 0.0)),
        ],
        dtype=float,
    )
    return h_matrix, v_vector


def process(sample, params):
    h_matrix, v_vector = _params_to_runtime(params)
    raw = np.array([float(sample["mag_x"]), float(sample["mag_y"]), float(sample["mag_z"])], dtype=float)
    corrected = h_matrix @ raw + v_vector
    mag_x = float(corrected[0])
    mag_y = float(corrected[1])
    mag_z = float(corrected[2])

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
        "flags": "papafotis_magical_rt",
    }
