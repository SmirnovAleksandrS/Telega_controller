from __future__ import annotations

import json
import math
from datetime import datetime, timezone


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")


def get_info() -> dict:
    return {
        "name": "Half Scale Magnetometer Method",
        "version": "1.0.0",
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


def calibrate(dataset, config=None):
    _ = dataset
    _ = config
    return {
        "algorithm_name": "Half Scale Magnetometer Method",
        "algorithm_version": "1.0.0",
        "schema_version": "1",
        "created_at": _utc_now(),
        "params": {
            "scale": 0.5,
        },
    }


def load_params(path):
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def save_params(path, params):
    payload = dict(params or {})
    payload.setdefault("algorithm_name", "Half Scale Magnetometer Method")
    payload.setdefault("algorithm_version", "1.0.0")
    payload.setdefault("schema_version", "1")
    payload.setdefault("created_at", _utc_now())
    payload.setdefault("params", {"scale": 0.5})
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)


def process(sample, params):
    scale = 0.5
    if isinstance(params, dict):
        inner = params.get("params")
        if isinstance(inner, dict):
            try:
                scale = float(inner.get("scale", scale))
            except (TypeError, ValueError):
                scale = 0.5

    mag_x = float(sample["mag_x"]) * scale
    mag_y = float(sample["mag_y"]) * scale
    mag_z = float(sample["mag_z"]) * scale
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
        "flags": "half_scale_rt",
    }
