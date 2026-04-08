from __future__ import annotations

import csv
import math
import time
from dataclasses import dataclass, field
from typing import Any

from app.magnetometer_dataset import Dataset, SampleRecord
from app.magnetometer_plugin_loader import LoadedMethodPlugin, run_method_process


@dataclass(slots=True)
class MetricRow:
    key: str
    label: str
    value: float | None
    units: str
    status: str
    notes: str = ""

    def value_text(self) -> str:
        if self.value is None:
            return "—"
        return format(self.value, ".6g")

    def to_csv_row(self, *, dataset_name: str, method_name: str, method_version: str) -> dict[str, str]:
        return {
            "dataset_name": dataset_name,
            "method_name": method_name,
            "method_version": method_version,
            "metric_key": self.key,
            "metric_label": self.label,
            "value": "" if self.value is None else format(self.value, ".12g"),
            "units": self.units,
            "status": self.status,
            "notes": self.notes,
        }


@dataclass(slots=True)
class MetricsReport:
    dataset_name: str
    method_name: str
    method_version: str
    rows: list[MetricRow] = field(default_factory=list)
    summary_text: str = ""
    warnings: list[str] = field(default_factory=list)
    error_text: str | None = None

    def export_csv(self, path: str) -> None:
        fieldnames = (
            "dataset_name",
            "method_name",
            "method_version",
            "metric_key",
            "metric_label",
            "value",
            "units",
            "status",
            "notes",
        )
        with open(path, "w", encoding="utf-8", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=fieldnames)
            writer.writeheader()
            for row in self.rows:
                writer.writerow(
                    row.to_csv_row(
                        dataset_name=self.dataset_name,
                        method_name=self.method_name,
                        method_version=self.method_version,
                    )
                )


def _raw_magnetometer_records(dataset: Dataset) -> list[SampleRecord]:
    exact = [record for record in dataset.records if record.stream_id == "raw_magnetometer"]
    if exact:
        return sorted(exact, key=lambda record: record.timestamp_mcu)
    fallback = [
        record
        for record in dataset.records
        if record.stream_type == "raw" and "heading_only" not in str(record.flags)
    ]
    return sorted(fallback, key=lambda record: record.timestamp_mcu)


def _reference_heading_records(dataset: Dataset) -> list[SampleRecord]:
    refs: list[SampleRecord] = []
    for record in dataset.records:
        if record.heading is None:
            continue
        stream_id = record.stream_id.lower()
        producer_name = record.producer_name.lower()
        if record.stream_type == "reference" or "gnss" in stream_id or "gnss" in producer_name:
            refs.append(record)
    return sorted(refs, key=lambda record: record.timestamp_mcu)


def _sample_from_record(record: SampleRecord) -> dict[str, Any]:
    return {
        "stream_id": record.stream_id,
        "stream_type": record.stream_type,
        "producer_name": record.producer_name,
        "producer_version": record.producer_version,
        "timestamp_mcu": record.timestamp_mcu,
        "timestamp_pc_rx": record.timestamp_pc_rx,
        "timestamp_pc_est": record.timestamp_pc_est,
        "mag_x": record.mag_x,
        "mag_y": record.mag_y,
        "mag_z": record.mag_z,
        "heading": record.heading,
        "flags": record.flags,
    }


def _normalize_output(output: dict[str, Any]) -> dict[str, Any]:
    mx = float(output["mag_x"])
    my = float(output["mag_y"])
    mz = float(output["mag_z"])
    if not all(math.isfinite(value) for value in (mx, my, mz)):
        raise ValueError("process() output contains non-finite magnetometer values")

    heading_raw = output.get("heading")
    if heading_raw is None or heading_raw == "":
        heading = None
        if not (math.isclose(mx, 0.0, abs_tol=1e-9) and math.isclose(my, 0.0, abs_tol=1e-9)):
            heading = (math.degrees(math.atan2(mx, my)) + 360.0) % 360.0
    else:
        heading = float(heading_raw)
        if not math.isfinite(heading):
            raise ValueError("process() output heading must be finite or null")

    normalized = dict(output)
    normalized["mag_x"] = mx
    normalized["mag_y"] = my
    normalized["mag_z"] = mz
    normalized["heading"] = heading
    return normalized


def _solve_3x3(matrix: list[list[float]], vector: list[float]) -> tuple[float, float, float] | None:
    augmented = [row[:] + [value] for row, value in zip(matrix, vector)]
    size = 3
    for col in range(size):
        pivot_row = max(range(col, size), key=lambda row: abs(augmented[row][col]))
        if math.isclose(augmented[pivot_row][col], 0.0, abs_tol=1e-12):
            return None
        if pivot_row != col:
            augmented[col], augmented[pivot_row] = augmented[pivot_row], augmented[col]

        pivot = augmented[col][col]
        for idx in range(col, size + 1):
            augmented[col][idx] /= pivot

        for row in range(size):
            if row == col:
                continue
            factor = augmented[row][col]
            for idx in range(col, size + 1):
                augmented[row][idx] -= factor * augmented[col][idx]

    return (augmented[0][3], augmented[1][3], augmented[2][3])


def fit_circle_kasa(points: list[tuple[float, float]]) -> tuple[float, float, float] | None:
    if len(points) < 3:
        return None

    sum_x = sum(x for x, _ in points)
    sum_y = sum(y for _, y in points)
    sum_xx = sum(x * x for x, _ in points)
    sum_yy = sum(y * y for _, y in points)
    sum_xy = sum(x * y for x, y in points)
    sum_z = sum(x * x + y * y for x, y in points)
    sum_xz = sum(x * (x * x + y * y) for x, y in points)
    sum_yz = sum(y * (x * x + y * y) for x, y in points)
    n = float(len(points))

    solution = _solve_3x3(
        [
            [sum_xx, sum_xy, sum_x],
            [sum_xy, sum_yy, sum_y],
            [sum_x, sum_y, n],
        ],
        [-sum_xz, -sum_yz, -sum_z],
    )
    if solution is None:
        return None

    d_coef, e_coef, f_coef = solution
    center_x = -0.5 * d_coef
    center_y = -0.5 * e_coef
    radius_sq = center_x * center_x + center_y * center_y - f_coef
    if radius_sq <= 0.0:
        return None
    return (center_x, center_y, math.sqrt(radius_sq))


def _fallback_circle(points: list[tuple[float, float]]) -> tuple[float, float, float]:
    center_x = sum(x for x, _ in points) / len(points)
    center_y = sum(y for _, y in points) / len(points)
    radii = [math.hypot(x - center_x, y - center_y) for x, y in points]
    return (center_x, center_y, sum(radii) / len(radii))


def _circular_diff_deg(a_deg: float, b_deg: float) -> float:
    diff = (a_deg - b_deg + 180.0) % 360.0 - 180.0
    return diff


def _rmse_vs_reference(
    outputs: list[dict[str, Any]],
    references: list[SampleRecord],
    *,
    tolerance_ms: int = 250,
) -> tuple[float | None, str]:
    if not references:
        return (None, "No GNSS/reference heading stream in dataset")

    ref_index = 0
    squared_errors: list[float] = []
    for output in outputs:
        heading = output.get("heading")
        if heading is None:
            continue
        ts = int(output["timestamp_mcu"])
        while ref_index + 1 < len(references) and references[ref_index + 1].timestamp_mcu <= ts:
            ref_index += 1
        candidates = [references[ref_index]]
        if ref_index + 1 < len(references):
            candidates.append(references[ref_index + 1])
        best = min(candidates, key=lambda record: abs(record.timestamp_mcu - ts))
        if abs(best.timestamp_mcu - ts) > tolerance_ms or best.heading is None:
            continue
        error = _circular_diff_deg(float(heading), float(best.heading))
        squared_errors.append(error * error)

    if not squared_errors:
        return (None, "No time-aligned GNSS/reference headings available")
    return (math.sqrt(sum(squared_errors) / len(squared_errors)), f"Matched {len(squared_errors)} reference samples")


def compute_metrics_report(dataset: Dataset | None, plugin: LoadedMethodPlugin | None) -> MetricsReport:
    dataset_name = "—" if dataset is None else dataset.summary()["name"]
    method_name = "—" if plugin is None else plugin.name
    method_version = "—" if plugin is None else plugin.version

    if dataset is None:
        return MetricsReport(
            dataset_name=dataset_name,
            method_name=method_name,
            method_version=method_version,
            summary_text="Select an active dataset to compute metrics.",
            error_text="no_dataset",
        )
    if plugin is None:
        return MetricsReport(
            dataset_name=dataset_name,
            method_name=method_name,
            method_version=method_version,
            summary_text="Select a method to compute metrics.",
            error_text="no_method",
        )
    if plugin.module is None or not plugin.supports_process():
        return MetricsReport(
            dataset_name=dataset_name,
            method_name=method_name,
            method_version=method_version,
            summary_text=f"{plugin.name} does not support process(sample, params).",
            error_text="no_process",
        )

    raw_records = _raw_magnetometer_records(dataset)
    if not raw_records:
        return MetricsReport(
            dataset_name=dataset_name,
            method_name=method_name,
            method_version=method_version,
            summary_text="Dataset has no raw magnetometer stream to analyze.",
            error_text="no_raw_records",
        )

    outputs: list[dict[str, Any]] = []
    process_durations_ms: list[float] = []
    for record in raw_records:
        sample = _sample_from_record(record)
        started_at = time.perf_counter()
        result = run_method_process(plugin, sample)
        process_durations_ms.append((time.perf_counter() - started_at) * 1000.0)
        if not result.ok or result.output is None:
            message = "Realtime process() failed during metrics evaluation."
            if result.diagnostics is not None:
                message = result.diagnostics.error_text
            return MetricsReport(
                dataset_name=dataset_name,
                method_name=method_name,
                method_version=method_version,
                summary_text=f"Metrics failed: {message}",
                warnings=list(result.warnings),
                error_text="process_failed",
            )
        try:
            merged_output = dict(sample)
            merged_output.update(result.output)
            outputs.append(_normalize_output(merged_output))
        except Exception as exc:
            return MetricsReport(
                dataset_name=dataset_name,
                method_name=method_name,
                method_version=method_version,
                summary_text=f"Metrics failed: {exc}",
                error_text="invalid_process_output",
            )

    points = [(float(output["mag_x"]), float(output["mag_y"])) for output in outputs]
    circle = fit_circle_kasa(points)
    notes: list[str] = []
    if circle is None:
        circle = _fallback_circle(points)
        notes.append("Circle fit fallback used centroid approximation")
    center_x, center_y, _ = circle

    radii = [math.hypot(x - center_x, y - center_y) for x, y in points]
    mean_radius = sum(radii) / len(radii)
    radius_var = sum((radius - mean_radius) ** 2 for radius in radii) / len(radii)
    radius_std = math.sqrt(max(radius_var, 0.0))
    center_offset = math.hypot(center_x, center_y)
    mean_process_runtime = sum(process_durations_ms) / len(process_durations_ms)
    rmse_value, rmse_note = _rmse_vs_reference(outputs, _reference_heading_records(dataset))
    calibration_runtime = plugin.calibration_runtime_s

    rows = [
        MetricRow(
            key="rmse_vs_gnss_heading",
            label="RMSE vs GNSS heading",
            value=rmse_value,
            units="deg",
            status="ok" if rmse_value is not None else "unavailable",
            notes=rmse_note,
        ),
        MetricRow(
            key="mean_radius",
            label="Mean radius",
            value=mean_radius,
            units="a.u.",
            status="ok",
            notes="Computed in XY plane after circle fit",
        ),
        MetricRow(
            key="radius_std",
            label="Radius std",
            value=radius_std,
            units="a.u.",
            status="ok",
            notes="Standard deviation of XY radii around fitted center",
        ),
        MetricRow(
            key="center_offset",
            label="Center offset",
            value=center_offset,
            units="a.u.",
            status="ok",
            notes=f"Center=({center_x:.6g}, {center_y:.6g})",
        ),
        MetricRow(
            key="calibration_runtime",
            label="Calibration runtime",
            value=calibration_runtime,
            units="s",
            status="ok" if calibration_runtime is not None else "unavailable",
            notes="Last calibration runtime for selected method",
        ),
        MetricRow(
            key="mean_process_runtime",
            label="Mean process runtime",
            value=mean_process_runtime,
            units="ms/sample",
            status="ok",
            notes=f"Measured across {len(process_durations_ms)} raw samples",
        ),
    ]

    if notes:
        rows[1].notes = f"{rows[1].notes}; {'; '.join(notes)}"

    return MetricsReport(
        dataset_name=dataset_name,
        method_name=method_name,
        method_version=method_version,
        rows=rows,
        summary_text=f"Metrics for {dataset_name} / {plugin.name} ({len(outputs)} processed samples)",
        warnings=notes,
        error_text=None,
    )
