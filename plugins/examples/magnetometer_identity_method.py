from __future__ import annotations

import json
from datetime import datetime, timezone


def get_info() -> dict:
    return {
        "name": "Identity Magnetometer Method",
        "version": "1.0.0",
        "type": "method",
        "supports_calibrate": True,
        "supports_load_params": True,
        "supports_save_params": True,
        "supports_process": True,
        "input_schema": {
            "timestamp_mcu": "int",
            "mag_x": "float",
            "mag_y": "float",
            "mag_z": "float",
        },
        "output_schema": {
            "mag_x": "float",
            "mag_y": "float",
            "mag_z": "float",
            "heading": "float|None",
        },
    }


def calibrate(dataset, config=None):
    return {
        "algorithm_name": "Identity Magnetometer Method",
        "algorithm_version": "1.0.0",
        "schema_version": "1",
        "created_at": datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z"),
        "params": {
            "offset": [0.0, 0.0, 0.0],
        },
    }


def load_params(path):
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def save_params(path, params):
    payload = dict(params)
    payload.setdefault("algorithm_name", "Identity Magnetometer Method")
    payload.setdefault("algorithm_version", "1.0.0")
    payload.setdefault("schema_version", "1")
    payload.setdefault("created_at", datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z"))
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)


def process(sample, params):
    return dict(sample)
