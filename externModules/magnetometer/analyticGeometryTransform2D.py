from __future__ import annotations

import json
import math
from datetime import datetime, timezone


METHOD_NAME = "Analytic Geometry Transform 2D"
METHOD_VERSION = "1.0.0"
SCHEMA_VERSION = "1"

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


def validate_dataset(dataset):
    if dataset is None or not hasattr(dataset, "records"):
        return {"ok": False, "warnings": ["dataset object is missing records"]}

    raw_records = [record for record in dataset.records if getattr(record, "stream_id", "") == "raw_magnetometer"]
    if len(raw_records) < 8:
        return {"ok": False, "warnings": ["need at least 8 raw_magnetometer samples"]}
    return None


def _extract_raw_xy_points(dataset) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    for record in dataset.records:
        if getattr(record, "stream_id", "") != "raw_magnetometer":
            continue
        points.append((float(record.mag_x), float(record.mag_y)))
    return points


def _find_major_axis_endpoints(points: list[tuple[float, float]]) -> tuple[int, int, float]:
    best_i = 0
    best_j = 1
    best_dist_sq = -1.0
    for idx in range(len(points)):
        x_i, y_i = points[idx]
        for jdx in range(idx + 1, len(points)):
            x_j, y_j = points[jdx]
            dx = x_i - x_j
            dy = y_i - y_j
            dist_sq = dx * dx + dy * dy
            if dist_sq > best_dist_sq:
                best_i = idx
                best_j = jdx
                best_dist_sq = dist_sq
    if best_dist_sq <= 0.0:
        raise ValueError("major axis estimation failed")
    return (best_i, best_j, math.sqrt(best_dist_sq) * 0.5)


def _normalize_direction(x_val: float, y_val: float) -> tuple[float, float]:
    length = math.hypot(x_val, y_val)
    if length <= 1e-12:
        raise ValueError("rotation angle is degenerate")
    return (x_val / length, y_val / length)


def _rotate_minus_beta(x_val: float, y_val: float, cos_beta: float, sin_beta: float) -> tuple[float, float]:
    return (
        cos_beta * x_val + sin_beta * y_val,
        -sin_beta * x_val + cos_beta * y_val,
    )


def calibrate(dataset, config=None):
    global _last_report
    _ = config

    points = _extract_raw_xy_points(dataset)
    if len(points) < 8:
        raise ValueError("need at least 8 raw_magnetometer samples for calibration")

    a1, a2, semi_major = _find_major_axis_endpoints(points)
    x1, y1 = points[a1]
    x2, y2 = points[a2]
    center_x = 0.5 * (x1 + x2)
    center_y = 0.5 * (y1 + y2)
    if semi_major <= 1e-12:
        raise ValueError("semi-major axis is degenerate")

    cos_beta, sin_beta = _normalize_direction(x1 - center_x, y1 - center_y)

    rotated_points = [
        _rotate_minus_beta(x_val - center_x, y_val - center_y, cos_beta, sin_beta)
        for x_val, y_val in points
    ]
    x_values = [point[0] for point in rotated_points]
    y_values = [point[1] for point in rotated_points]
    x_span = max(x_values) - min(x_values)
    y_span = max(y_values) - min(y_values)
    if x_span <= 1e-12 or y_span <= 1e-12:
        raise ValueError("ellipse spans are degenerate")

    x_scale = max(1.0, y_span / x_span)
    y_scale = max(1.0, x_span / y_span)

    _last_report = (
        f"Raw samples: {len(points)}\n"
        f"Major axis endpoints: {a1}, {a2}\n"
        f"Center V0: ({center_x:.6f}, {center_y:.6f})\n"
        f"beta_deg: {math.degrees(math.atan2(sin_beta, cos_beta)):.6f}\n"
        f"x_span: {x_span:.6f}\n"
        f"y_span: {y_span:.6f}\n"
        f"x_scale: {x_scale:.6f}\n"
        f"y_scale: {y_scale:.6f}"
    )

    return {
        "algorithm_name": METHOD_NAME,
        "algorithm_version": METHOD_VERSION,
        "schema_version": SCHEMA_VERSION,
        "created_at": _utc_now(),
        "params": {
            "offset_x": center_x,
            "offset_y": center_y,
            "cos_beta": cos_beta,
            "sin_beta": sin_beta,
            "x_scale": x_scale,
            "y_scale": y_scale,
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
            "cos_beta": 1.0,
            "sin_beta": 0.0,
            "x_scale": 1.0,
            "y_scale": 1.0,
        },
    )
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)


def process(sample, params):
    inner = params.get("params", {}) if isinstance(params, dict) else {}
    offset_x = float(inner.get("offset_x", 0.0))
    offset_y = float(inner.get("offset_y", 0.0))
    cos_beta = float(inner.get("cos_beta", 1.0))
    sin_beta = float(inner.get("sin_beta", 0.0))
    x_scale = float(inner.get("x_scale", 1.0))
    y_scale = float(inner.get("y_scale", 1.0))

    translated_x = float(sample["mag_x"]) - offset_x
    translated_y = float(sample["mag_y"]) - offset_y
    rotated_x, rotated_y = _rotate_minus_beta(translated_x, translated_y, cos_beta, sin_beta)

    mag_x = x_scale * rotated_x
    mag_y = y_scale * rotated_y
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
        "flags": "analytic_geom_2d_rt",
    }
