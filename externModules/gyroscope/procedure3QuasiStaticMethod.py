from __future__ import annotations

import json
import math
from typing import Any

from externModules.imu_calibration.common import (
    axis_variances,
    choose_best_quasi_static_config,
    fit_gyroscope_matrix,
    matrix_vector_mul,
    normalize,
    pair_series_by_timestamp,
    utc_now,
    vector_sub,
)

METHOD_NAME = "Procedure 3 Quasi-Static Gyroscope Calibration"
METHOD_VERSION = "1.0.0"
SCHEMA_VERSION = "1"
REQUIRED_STATIC_SET_COUNT = 24
RECOMMENDED_STATIC_SET_COUNT = 30
MIN_BIAS_WINDOW_S = 2.0
RECOMMENDED_BIAS_WINDOW_S = 20.0
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
            "gyro_x": "float",
            "gyro_y": "float",
            "gyro_z": "float",
            "rate_mag": "float|None",
            "flags": "str",
        },
        "output_schema": {
            "timestamp_mcu": "int",
            "timestamp_pc_rx": "int",
            "timestamp_pc_est": "int|None",
            "gyro_x": "float",
            "gyro_y": "float",
            "gyro_z": "float",
            "rate_mag": "float|None",
            "flags": "str",
        },
        "stream_requirements": {
            "calibrate": [
                {
                    "slot": "gyro_input",
                    "kind": "imu.gyro_vector",
                    "label": "Gyroscope input",
                    "description": "Raw gyroscope vector to calibrate offline.",
                },
                {
                    "slot": "acc_input",
                    "kind": "imu.accel_vector",
                    "label": "Accelerometer reference",
                    "description": "Calibrated accelerometer vector from the accelerometer tab used as gravity reference.",
                },
            ],
            "process": [
                {
                    "slot": "gyro_input",
                    "kind": "imu.gyro_vector",
                    "label": "Gyroscope input",
                    "description": "Gyroscope vector to correct with the calibrated model.",
                    "scope": "process",
                }
            ],
        },
    }


def validate_dataset(dataset):
    if dataset is None or not hasattr(dataset, "records"):
        return {"ok": False, "warnings": ["dataset object is missing records"]}
    raw_records = [record for record in dataset.records if getattr(record, "stream_id", "") == "raw_gyroscope"]
    if len(raw_records) < 32:
        return {"ok": False, "warnings": ["need at least 32 raw_gyroscope samples"]}
    return None


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


def _rate_mag(vector: tuple[float, float, float]) -> float:
    return math.sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2])


def _build_motion_intervals(matched_pairs: list[tuple[dict[str, Any], dict[str, Any]]], segments: list[dict[str, Any]]) -> list[dict[str, Any]]:
    intervals: list[dict[str, Any]] = []
    for left, right in zip(segments[:-1], segments[1:]):
        start_index = int(left["mid_index"])
        end_index = int(right["mid_index"])
        if end_index - start_index < 2:
            continue
        gyro_samples = [pair[1] for pair in matched_pairs[start_index:end_index + 1]]
        if len(gyro_samples) < 2:
            continue
        intervals.append(
            {
                "gyro_samples": gyro_samples,
                "gravity_start": normalize(tuple(left["mean_vector"])),
                "gravity_end": normalize(tuple(right["mean_vector"])),
            }
        )
    return intervals


def calibrate(dataset, config=None):
    global _last_report
    if not isinstance(config, dict):
        raise ValueError("gyroscope procedure 3 requires calibration config with stream inputs")
    stream_inputs = config.get("stream_inputs", {})
    gyro_records = list(stream_inputs.get("gyro_input", {}).get("records", []))
    acc_records = list(stream_inputs.get("acc_input", {}).get("records", []))
    selected_producers = config.get("selected_producers", {})
    acc_producer = selected_producers.get("acc_input", {})
    if not gyro_records:
        raise ValueError("selected gyroscope input has no offline records")
    if not acc_records:
        raise ValueError("selected accelerometer input has no offline records")
    if str(acc_producer.get("origin", "")) != "method":
        raise ValueError("gyroscope procedure 3 requires calibrated accelerometer method output")

    paired = pair_series_by_timestamp(acc_records, gyro_records)
    if paired["matched_count"] <= 0:
        raise ValueError("no exact timestamp overlap between accelerometer and gyroscope inputs")
    matched_pairs = list(paired["pairs"])
    matched_acc_records = [pair[0] for pair in matched_pairs]

    best = choose_best_quasi_static_config(
        matched_acc_records,
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

    first_segment = segments[0]
    bias_window_s = float(first_segment["duration_s"])
    if bias_window_s < MIN_BIAS_WINDOW_S:
        raise ValueError(
            f"initial static pause is too short for bias estimation ({bias_window_s:.2f}s, need >= {MIN_BIAS_WINDOW_S:.1f}s)"
        )

    first_segment_gyro = [pair[1] for pair in matched_pairs[first_segment["start_index"]:first_segment["end_index"] + 1]]
    if not first_segment_gyro:
        raise ValueError("initial static segment has no matched gyroscope samples")
    bias = (
        sum(float(sample["gyro_x"]) for sample in first_segment_gyro) / float(len(first_segment_gyro)),
        sum(float(sample["gyro_y"]) for sample in first_segment_gyro) / float(len(first_segment_gyro)),
        sum(float(sample["gyro_z"]) for sample in first_segment_gyro) / float(len(first_segment_gyro)),
    )

    intervals = _build_motion_intervals(matched_pairs, segments)
    if not intervals:
        raise ValueError("not enough motion intervals between quasi-static states")
    fitted = fit_gyroscope_matrix(intervals, bias=bias)
    matrix = fitted["matrix"]

    mean_vectors = [tuple(segment["mean_vector"]) for segment in segments]
    variances = axis_variances(mean_vectors)
    warnings: list[str] = []
    if len(segments) < RECOMMENDED_STATIC_SET_COUNT:
        warnings.append(
            f"dataset has {len(segments)} quasi-static stops; {RECOMMENDED_STATIC_SET_COUNT}+ recommended"
        )
    if bias_window_s < RECOMMENDED_BIAS_WINDOW_S:
        warnings.append(
            f"initial static pause is {bias_window_s:.2f}s; {RECOMMENDED_BIAS_WINDOW_S:.0f}s recommended for bias estimation"
        )
    if float(paired["overlap_ratio"]) < 0.95:
        warnings.append(
            f"timestamp overlap ratio is low ({paired['overlap_ratio']:.3f})"
        )
    axis_labels = ("x", "y", "z")
    for axis_label, variance in zip(axis_labels, variances):
        if variance < 0.2:
            warnings.append(f"accelerometer reference {axis_label}-axis variance is low ({variance:.4f} g^2)")

    metadata = getattr(dataset, "metadata", {}) if hasattr(dataset, "metadata") else {}
    summary = {
        "source_dataset_name": dataset.summary()["name"] if hasattr(dataset, "summary") else getattr(dataset, "name", ""),
        "shared_source_path": str(metadata.get("shared_source_path", "") or getattr(dataset, "source_path", "") or ""),
        "initial_static_duration_s": bias_window_s,
        "quasi_static_threshold_g2": float(best["threshold_g2"]),
        "static_time_s": float(best["static_time_s"]),
        "static_set_count": len(segments),
        "required_static_set_count": REQUIRED_STATIC_SET_COUNT,
        "recommended_static_set_count": RECOMMENDED_STATIC_SET_COUNT,
        "matched_sample_count": int(paired["matched_count"]),
        "dropped_unmatched_sample_count": int(paired["dropped_count"]),
        "dataset_overlap_ratio": float(paired["overlap_ratio"]),
        "acc_variance_x_g": float(variances[0]),
        "acc_variance_y_g": float(variances[1]),
        "acc_variance_z_g": float(variances[2]),
        "gyroscope_cost": float(fitted["cost"]),
        "bias_window_s": bias_window_s,
    }
    _last_report = (
        f"Procedure 3 gyroscope calibration\n"
        f"Matched samples: {paired['matched_count']}\n"
        f"Static sets: {len(segments)}\n"
        f"Threshold g^2: {best['threshold_g2']:.7f}\n"
        f"Static time s: {best['static_time_s']:.2f}\n"
        f"Bias window s: {bias_window_s:.2f}\n"
        f"Bias: ({bias[0]:.6f}, {bias[1]:.6f}, {bias[2]:.6f})\n"
        f"Cost: {fitted['cost']:.8f}\n"
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
        float(sample["gyro_x"]),
        float(sample["gyro_y"]),
        float(sample["gyro_z"]),
    )
    corrected = matrix_vector_mul(matrix, vector_sub(raw, bias))
    return {
        "timestamp_mcu": sample["timestamp_mcu"],
        "timestamp_pc_rx": sample["timestamp_pc_rx"],
        "timestamp_pc_est": sample.get("timestamp_pc_est"),
        "gyro_x": corrected[0],
        "gyro_y": corrected[1],
        "gyro_z": corrected[2],
        "rate_mag": _rate_mag(corrected),
        "flags": "procedure3_gyro_rt",
    }

