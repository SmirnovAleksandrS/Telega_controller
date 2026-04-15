from __future__ import annotations

import json
import math
from typing import Any

from externModules.imu_calibration.common import (
    axis_variances,
    choose_best_quasi_static_config,
    fit_accelerometer_affine,
    matrix_vector_mul,
    utc_now,
    vector_sub,
)

METHOD_NAME = "Procedure 3 Quasi-Static Accelerometer Calibration"
METHOD_VERSION = "1.0.0"
SCHEMA_VERSION = "1"
REQUIRED_STATIC_SET_COUNT = 24
RECOMMENDED_STATIC_SET_COUNT = 30
THRESHOLDS_G2 = [5.8e-5 * float(index) for index in range(1, 7)]
STATIC_TIMES_S = [0.05 * float(index) for index in range(1, 11)]

_last_report = ""


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
            "acc_x": "float",
            "acc_y": "float",
            "acc_z": "float",
            "roll_deg": "float|None",
            "pitch_deg": "float|None",
            "flags": "str",
        },
        "output_schema": {
            "timestamp_mcu": "int",
            "timestamp_pc_rx": "int",
            "timestamp_pc_est": "int|None",
            "acc_x": "float",
            "acc_y": "float",
            "acc_z": "float",
            "roll_deg": "float|None",
            "pitch_deg": "float|None",
            "flags": "str",
        },
        "stream_requirements": {
            "calibrate": [
                {
                    "slot": "acc_input",
                    "kind": "imu.accel_vector",
                    "label": "Accelerometer input",
                    "description": "Raw accelerometer vector used to detect quasi-static segments and estimate affine calibration.",
                }
            ],
            "process": [
                {
                    "slot": "acc_input",
                    "kind": "imu.accel_vector",
                    "label": "Accelerometer input",
                    "description": "Accelerometer vector to correct with the calibrated affine model.",
                    "scope": "process",
                }
            ],
        },
    }


def validate_dataset(dataset):
    if dataset is None or not hasattr(dataset, "records"):
        return {"ok": False, "warnings": ["dataset object is missing records"]}
    raw_records = [
        record
        for record in dataset.records
        if getattr(record, "stream_id", "") == "raw_accelerometer" and "tilt_only" not in (getattr(record, "flags", "") or "")
    ]
    if len(raw_records) < 32:
        return {"ok": False, "warnings": ["need at least 32 raw_accelerometer samples"]}
    return None


def _compute_tilt_deg(ax: float, ay: float, az: float) -> tuple[float | None, float | None]:
    if not all(math.isfinite(value) for value in (ax, ay, az)):
        return (None, None)
    magnitude = math.sqrt(ax * ax + ay * ay + az * az)
    if magnitude <= 1e-9:
        return (None, None)
    roll_deg = math.degrees(math.atan2(ay, az))
    pitch_deg = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
    return (roll_deg, pitch_deg)


def _record_to_sample(record: Any) -> dict[str, Any]:
    return {
        "timestamp_mcu": int(getattr(record, "timestamp_mcu", 0)),
        "timestamp_pc_rx": int(getattr(record, "timestamp_pc_rx", 0)),
        "timestamp_pc_est": getattr(record, "timestamp_pc_est", None),
        "acc_x": float(getattr(record, "acc_x", 0.0)),
        "acc_y": float(getattr(record, "acc_y", 0.0)),
        "acc_z": float(getattr(record, "acc_z", 0.0)),
        "roll_deg": getattr(record, "roll_deg", None),
        "pitch_deg": getattr(record, "pitch_deg", None),
        "flags": str(getattr(record, "flags", "") or ""),
    }


def _raw_accelerometer_samples(dataset) -> list[dict[str, Any]]:
    return [
        _record_to_sample(record)
        for record in getattr(dataset, "records", [])
        if getattr(record, "stream_id", "") == "raw_accelerometer" and "tilt_only" not in (getattr(record, "flags", "") or "")
    ]


def _params_payload(bias: tuple[float, float, float], matrix: list[list[float]]) -> dict[str, float]:
    return {
        "bias_x": bias[0],
        "bias_y": bias[1],
        "bias_z": bias[2],
        "m00": matrix[0][0],
        "m01": matrix[0][1],
        "m02": matrix[0][2],
        "m10": matrix[1][0],
        "m11": matrix[1][1],
        "m12": matrix[1][2],
        "m20": matrix[2][0],
        "m21": matrix[2][1],
        "m22": matrix[2][2],
    }


def _params_to_runtime(params: Any) -> tuple[tuple[float, float, float], list[list[float]]]:
    inner = params.get("params", {}) if isinstance(params, dict) else {}
    bias = (
        float(inner.get("bias_x", 0.0)),
        float(inner.get("bias_y", 0.0)),
        float(inner.get("bias_z", 0.0)),
    )
    matrix = [
        [float(inner.get("m00", 1.0)), float(inner.get("m01", 0.0)), float(inner.get("m02", 0.0))],
        [float(inner.get("m10", 0.0)), float(inner.get("m11", 1.0)), float(inner.get("m12", 0.0))],
        [float(inner.get("m20", 0.0)), float(inner.get("m21", 0.0)), float(inner.get("m22", 1.0))],
    ]
    return (bias, matrix)


def calibrate(dataset, config=None):
    global _last_report
    del config

    samples = _raw_accelerometer_samples(dataset)
    if len(samples) < 32:
        raise ValueError("need at least 32 raw_accelerometer samples for calibration")

    best = choose_best_quasi_static_config(
        samples,
        thresholds_g2=THRESHOLDS_G2,
        static_times_s=STATIC_TIMES_S,
        vector_getter=lambda sample: (
            float(sample["acc_x"]),
            float(sample["acc_y"]),
            float(sample["acc_z"]),
        ),
    )
    segments = list(best["segments"])
    if len(segments) < REQUIRED_STATIC_SET_COUNT:
        raise ValueError(
            f"need at least {REQUIRED_STATIC_SET_COUNT} quasi-static stops, found {len(segments)}"
        )

    mean_vectors = [tuple(segment["mean_vector"]) for segment in segments]
    fitted = fit_accelerometer_affine(mean_vectors)
    bias = tuple(fitted["bias"])
    matrix = fitted["matrix"]
    variances = axis_variances(mean_vectors)

    warnings: list[str] = []
    if len(segments) < RECOMMENDED_STATIC_SET_COUNT:
        warnings.append(
            f"dataset has {len(segments)} quasi-static stops; {RECOMMENDED_STATIC_SET_COUNT}+ recommended"
        )
    axis_labels = ("x", "y", "z")
    for axis_label, variance in zip(axis_labels, variances):
        if variance < 0.2:
            warnings.append(f"accelerometer {axis_label}-axis variance is low ({variance:.4f} g^2)")

    metadata = getattr(dataset, "metadata", {}) if hasattr(dataset, "metadata") else {}
    summary = {
        "source_dataset_name": dataset.summary()["name"] if hasattr(dataset, "summary") else getattr(dataset, "name", ""),
        "shared_source_path": str(metadata.get("shared_source_path", "") or getattr(dataset, "source_path", "") or ""),
        "initial_static_duration_s": float(segments[0]["duration_s"]) if segments else 0.0,
        "quasi_static_threshold_g2": float(best["threshold_g2"]),
        "static_time_s": float(best["static_time_s"]),
        "static_set_count": len(segments),
        "required_static_set_count": REQUIRED_STATIC_SET_COUNT,
        "recommended_static_set_count": RECOMMENDED_STATIC_SET_COUNT,
        "matched_sample_count": len(samples),
        "dropped_unmatched_sample_count": 0,
        "dataset_overlap_ratio": 1.0,
        "acc_variance_x_g": float(variances[0]),
        "acc_variance_y_g": float(variances[1]),
        "acc_variance_z_g": float(variances[2]),
        "accelerometer_cost": float(fitted["cost"]),
    }
    _last_report = (
        f"Procedure 3 accelerometer calibration\n"
        f"Static sets: {len(segments)}\n"
        f"Threshold g^2: {best['threshold_g2']:.7f}\n"
        f"Static time s: {best['static_time_s']:.2f}\n"
        f"Cost: {fitted['cost']:.8f}\n"
        f"Bias: ({bias[0]:.6f}, {bias[1]:.6f}, {bias[2]:.6f})\n"
        f"Variance xyz g^2: ({variances[0]:.4f}, {variances[1]:.4f}, {variances[2]:.4f})\n"
        f"Warnings: {len(warnings)}"
    )
    return {
        "algorithm_name": METHOD_NAME,
        "algorithm_version": METHOD_VERSION,
        "schema_version": SCHEMA_VERSION,
        "created_at": utc_now(),
        "params": _params_payload(bias, matrix),
        "calibration_summary": summary,
        "warnings": warnings,
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
    payload.setdefault("created_at", utc_now())
    payload.setdefault("params", _params_payload((0.0, 0.0, 0.0), [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]))
    payload.setdefault("calibration_summary", {})
    payload.setdefault("warnings", [])
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)


def process(sample, params):
    bias, matrix = _params_to_runtime(params)
    raw = (
        float(sample["acc_x"]),
        float(sample["acc_y"]),
        float(sample["acc_z"]),
    )
    corrected = matrix_vector_mul(matrix, vector_sub(raw, bias))
    roll_deg, pitch_deg = _compute_tilt_deg(corrected[0], corrected[1], corrected[2])
    return {
        "timestamp_mcu": sample["timestamp_mcu"],
        "timestamp_pc_rx": sample["timestamp_pc_rx"],
        "timestamp_pc_est": sample.get("timestamp_pc_est"),
        "acc_x": corrected[0],
        "acc_y": corrected[1],
        "acc_z": corrected[2],
        "roll_deg": roll_deg,
        "pitch_deg": pitch_deg,
        "flags": "procedure3_acc_rt",
    }

