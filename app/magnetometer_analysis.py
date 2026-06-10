from __future__ import annotations

import math
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Mapping

RAW_METHOD_ID = "__raw_magnetometer__"


@dataclass(frozen=True, slots=True)
class AnalysisDatasetSummary:
    name: str
    row_count: int
    source_count: int
    time_range: str
    source_path: str
    is_active: bool = False


@dataclass(frozen=True, slots=True)
class AnalysisDataAvailability:
    has_active_dataset: bool
    has_raw_magnetometer: bool
    has_gnss_heading: bool
    has_tilt: bool
    has_derived_outputs: bool
    raw_sample_count: int = 0
    gnss_sample_count: int = 0
    tilt_sample_count: int = 0
    derived_stream_count: int = 0
    notes: tuple[str, ...] = ()


@dataclass(frozen=True, slots=True)
class AnalysisMethodState:
    method_id: str
    name: str
    version: str
    file_path: str
    status: str
    status_text: str
    calibrated: bool
    supports_process: bool
    analyze_enabled: bool
    view_enabled: bool
    warning_count: int
    warnings: tuple[str, ...] = ()
    params_profile_path: str = ""
    calibration_dataset_name: str = ""
    calibration_runtime_s: float | None = None
    offline_record_count: int = 0
    derived_stream_id: str = ""


@dataclass(frozen=True, slots=True)
class AnalysisFigureSlot:
    key: str
    title: str
    tab: str
    description: str
    required_data: tuple[str, ...] = ()


@dataclass(frozen=True, slots=True)
class AnalysisDatasetRecords:
    name: str
    source_path: str
    is_active: bool
    raw_records: tuple[Any, ...] = ()
    reference_records: tuple[Any, ...] = ()


@dataclass(frozen=True, slots=True)
class AnalysisSnapshot:
    active_dataset: AnalysisDatasetSummary | None
    datasets: tuple[AnalysisDatasetSummary, ...]
    availability: AnalysisDataAvailability
    methods: tuple[AnalysisMethodState, ...]
    figure_slots: tuple[AnalysisFigureSlot, ...]
    selected_method_id: str | None = None
    raw_records: tuple[Any, ...] = ()
    reference_records: tuple[Any, ...] = ()
    tilt_records: tuple[Any, ...] = ()
    derived_records_by_method_id: Mapping[str, tuple[Any, ...]] = field(default_factory=dict)
    dataset_records: tuple[AnalysisDatasetRecords, ...] = ()
    created_at: str = field(default_factory=lambda: datetime.now().isoformat(timespec="seconds"))


@dataclass(frozen=True, slots=True)
class TiltBinMetric:
    label: str
    lower_deg: float
    upper_deg: float | None
    count: int
    mae_deg: float | None
    rmse_deg: float | None
    p95_deg: float | None


@dataclass(frozen=True, slots=True)
class MethodHeadingAnalysis:
    method_id: str
    name: str
    times_s: tuple[float, ...]
    mag_x: tuple[float, ...]
    mag_y: tuple[float, ...]
    mag_z: tuple[float, ...]
    heading_deg: tuple[float, ...]
    radius_xy: tuple[float, ...]
    matched_times_s: tuple[float, ...] = ()
    gnss_heading_deg: tuple[float, ...] = ()
    aligned_heading_deg: tuple[float, ...] = ()
    aligned_error_deg: tuple[float, ...] = ()
    absolute_error_deg: tuple[float, ...] = ()
    delta_times_s: tuple[float, ...] = ()
    delta_error_deg: tuple[float, ...] = ()
    roll_deg: tuple[float, ...] = ()
    pitch_deg: tuple[float, ...] = ()
    alpha_deg: tuple[float, ...] = ()
    tilt_bins: tuple[TiltBinMetric, ...] = ()
    metrics: Mapping[str, float | int | None] = field(default_factory=dict)


@dataclass(frozen=True, slots=True)
class DatasetCloudAnalysis:
    name: str
    source_path: str
    is_active: bool
    magnet_label: str
    times_s: tuple[float, ...]
    mag_x: tuple[float, ...]
    mag_y: tuple[float, ...]
    mag_z: tuple[float, ...]
    heading_deg: tuple[float, ...]


@dataclass(frozen=True, slots=True)
class MagnetometerAnalysisResult:
    raw_times_s: tuple[float, ...]
    raw_x: tuple[float, ...]
    raw_y: tuple[float, ...]
    raw_z: tuple[float, ...]
    raw_heading_deg: tuple[float, ...]
    reference_times_s: tuple[float, ...]
    reference_heading_deg: tuple[float, ...]
    tilt_times_s: tuple[float, ...]
    tilt_roll_deg: tuple[float, ...]
    tilt_pitch_deg: tuple[float, ...]
    tilt_alpha_deg: tuple[float, ...]
    methods: tuple[MethodHeadingAnalysis, ...]
    dataset_clouds: tuple[DatasetCloudAnalysis, ...]
    stress_rows: tuple[Mapping[str, Any], ...]
    warnings: tuple[str, ...] = ()


def summarize_dataset(dataset: Any | None, *, is_active: bool = False) -> AnalysisDatasetSummary | None:
    if dataset is None:
        return None
    summary = dataset.summary() if callable(getattr(dataset, "summary", None)) else {}
    source_path = str(getattr(dataset, "source_path", "") or "")
    return AnalysisDatasetSummary(
        name=str(summary.get("name") or getattr(dataset, "name", "dataset")),
        row_count=_coerce_int(summary.get("row_count"), len(getattr(dataset, "records", []))),
        source_count=_coerce_int(summary.get("source_count"), 0),
        time_range=str(summary.get("time_range") or "-"),
        source_path=source_path,
        is_active=bool(is_active),
    )


def build_data_availability(
    active_dataset: Any | None,
    paired_accelerometer_dataset: Any | None,
    method_states: tuple[AnalysisMethodState, ...] = (),
) -> AnalysisDataAvailability:
    raw_records = raw_magnetometer_records(active_dataset)
    gnss_records = reference_heading_records(active_dataset)
    tilt_records = tilt_records_from_dataset(paired_accelerometer_dataset)
    records = list(getattr(active_dataset, "records", []) or [])
    derived_stream_ids = {
        state.derived_stream_id
        for state in method_states
        if state.derived_stream_id and state.offline_record_count > 0
    }
    derived_stream_ids.update(
        str(getattr(record, "stream_id", ""))
        for record in records
        if getattr(record, "stream_type", "") == "derived"
    )

    notes: list[str] = []
    if active_dataset is None:
        notes.append("No active magnetometer dataset loaded")
    if not raw_records:
        notes.append("Raw magnetometer stream is unavailable")
    if not gnss_records:
        notes.append("GNSS/reference heading stream is unavailable")
    if not tilt_records:
        notes.append("Tilt stream is unavailable")

    return AnalysisDataAvailability(
        has_active_dataset=active_dataset is not None,
        has_raw_magnetometer=bool(raw_records),
        has_gnss_heading=bool(gnss_records),
        has_tilt=bool(tilt_records),
        has_derived_outputs=bool(derived_stream_ids),
        raw_sample_count=len(raw_records),
        gnss_sample_count=len(gnss_records),
        tilt_sample_count=len(tilt_records),
        derived_stream_count=len(derived_stream_ids),
        notes=tuple(notes),
    )


def build_method_state(method_id: str, plugin: Any, *, offline_record_count: int = 0) -> AnalysisMethodState:
    calibrated = getattr(plugin, "calibration_params", None) is not None
    supports_process = _safe_bool_call(plugin, "supports_process")
    warnings = tuple(str(item) for item in _safe_list_call(plugin, "effective_warnings"))
    derived_stream_id = str(getattr(plugin, "derived_stream_id", "") or f"derived_{method_id}")
    analyze_enabled = bool(calibrated and supports_process and offline_record_count > 0 and getattr(plugin, "status", "") != "error")
    view_enabled = bool(supports_process and getattr(plugin, "status", "") != "error")
    return AnalysisMethodState(
        method_id=method_id,
        name=str(getattr(plugin, "name", method_id)),
        version=str(getattr(plugin, "version", "-")),
        file_path=str(getattr(plugin, "file_path", "")),
        status=str(_safe_call(plugin, "display_status", default=getattr(plugin, "status", "partial"))),
        status_text=str(_safe_call(plugin, "display_status_text", default=getattr(plugin, "status", "partial"))),
        calibrated=calibrated,
        supports_process=supports_process,
        analyze_enabled=analyze_enabled,
        view_enabled=view_enabled,
        warning_count=len(warnings),
        warnings=warnings,
        params_profile_path=str(getattr(plugin, "params_profile_path", "") or ""),
        calibration_dataset_name=str(getattr(plugin, "calibration_dataset_name", "") or ""),
        calibration_runtime_s=getattr(plugin, "calibration_runtime_s", None),
        offline_record_count=max(0, int(offline_record_count)),
        derived_stream_id=derived_stream_id,
    )


def default_figure_slots() -> tuple[AnalysisFigureSlot, ...]:
    return (
        AnalysisFigureSlot("overview_readiness", "Readiness summary", "Overview", "Dataset and method readiness table"),
        AnalysisFigureSlot("raw_xy", "Raw XY cloud", "Calibration cloud", "Raw magnetometer points in mx,my plane", ("raw",)),
        AnalysisFigureSlot("calibrated_xy", "Calibrated XY cloud", "Calibration cloud", "Selected calibrated outputs in mx,my plane", ("raw", "derived")),
        AnalysisFigureSlot("radius_time", "Radius over time", "Calibration cloud", "Calibrated horizontal radius over time", ("derived",)),
        AnalysisFigureSlot("heading_coverage", "Heading coverage", "Calibration cloud", "Heading coverage histogram", ("raw",)),
        AnalysisFigureSlot("heading_time", "Heading vs GNSS", "Heading vs GNSS", "GNSS and magnetometer heading over time", ("gnss", "derived")),
        AnalysisFigureSlot("aligned_error_time", "Aligned error over time", "Heading vs GNSS", "Aligned heading error over time", ("gnss", "derived")),
        AnalysisFigureSlot("error_distribution", "Error distribution", "Heading vs GNSS", "Histogram and boxplot placeholders", ("gnss", "derived")),
        AnalysisFigureSlot("error_vs_heading", "Error vs GNSS heading", "Heading vs GNSS", "Heading-dependent error placeholder", ("gnss", "derived")),
        AnalysisFigureSlot("delta_heading_error", "Delta-heading error", "Dynamics", "Heading-change error over time", ("gnss", "derived")),
        AnalysisFigureSlot("closure_metrics", "Closure metrics", "Dynamics", "Closure and segment tables", ("gnss", "derived")),
        AnalysisFigureSlot("tilt_timeline", "Tilt timeline", "Tilt sensitivity", "Heading, pitch, roll and alpha timeline", ("gnss", "tilt", "derived")),
        AnalysisFigureSlot("error_vs_alpha", "Error vs tilt", "Tilt sensitivity", "Heading error vs total tilt", ("gnss", "tilt", "derived")),
        AnalysisFigureSlot("tilt_bins", "Tilt-bin metrics", "Tilt sensitivity", "RMSE/P95 by tilt bins", ("gnss", "tilt", "derived")),
        AnalysisFigureSlot("magnet_cloud_compare", "Magnet cloud comparison", "Magnet/stress", "With/without magnet cloud comparison", ("raw",)),
        AnalysisFigureSlot("magnet_stress_modes", "Stress modes A-D", "Magnet/stress", "Matched and cross-configuration placeholders", ("gnss", "derived")),
    )


def raw_magnetometer_records(dataset: Any | None) -> list[Any]:
    if dataset is None:
        return []
    records = list(getattr(dataset, "records", []) or [])
    exact = [
        record
        for record in records
        if getattr(record, "stream_id", "") == "raw_magnetometer"
        and "heading_only" not in str(getattr(record, "flags", ""))
    ]
    if exact:
        return sorted(exact, key=lambda record: _timestamp_ms(record) or 0)
    fallback = [
        record
        for record in records
        if getattr(record, "stream_type", "") == "raw"
        and "heading_only" not in str(getattr(record, "flags", ""))
        and _finite_float(getattr(record, "mag_x", None)) is not None
        and _finite_float(getattr(record, "mag_y", None)) is not None
    ]
    return sorted(fallback, key=lambda record: _timestamp_ms(record) or 0)


def reference_heading_records(dataset: Any | None) -> list[Any]:
    if dataset is None:
        return []
    records = list(getattr(dataset, "records", []) or [])
    refs: list[Any] = []
    for record in records:
        if _finite_float(getattr(record, "heading", None)) is None:
            continue
        stream_id = str(getattr(record, "stream_id", "")).lower()
        producer_name = str(getattr(record, "producer_name", "")).lower()
        if (
            stream_id == "gnss_heading"
            or getattr(record, "stream_type", "") == "reference"
            or "gnss" in stream_id
            or "gnss" in producer_name
        ):
            refs.append(record)
    return sorted(refs, key=lambda record: _timestamp_ms(record) or 0)


def tilt_records_from_dataset(dataset: Any | None) -> list[Any]:
    records = list(getattr(dataset, "records", []) or [])
    tilt_records = [
        record
        for record in records
        if getattr(record, "stream_id", "") == "raw_tilt"
        or getattr(record, "roll_deg", None) is not None
        or getattr(record, "pitch_deg", None) is not None
    ]
    return sorted(tilt_records, key=lambda record: _timestamp_ms(record) or 0)


def heading_from_xy_deg(mx: float, my: float) -> float | None:
    if math.isclose(mx, 0.0, abs_tol=1e-12) and math.isclose(my, 0.0, abs_tol=1e-12):
        return None
    return wrap_360_deg(math.degrees(math.atan2(mx, my)))


def wrap_360_deg(value: float) -> float:
    wrapped = value % 360.0
    return wrapped + 360.0 if wrapped < 0.0 else wrapped


def wrap_180_deg(value: float) -> float:
    wrapped = (value + 180.0) % 360.0 - 180.0
    if math.isclose(wrapped, -180.0, abs_tol=1e-12) and value > 0.0:
        return 180.0
    return wrapped


def unwrap_degrees(values: list[float] | tuple[float, ...]) -> list[float]:
    if not values:
        return []
    unwrapped = [float(values[0])]
    prev_raw = float(values[0])
    for value in values[1:]:
        raw = float(value)
        unwrapped.append(unwrapped[-1] + wrap_180_deg(raw - prev_raw))
        prev_raw = raw
    return unwrapped


def circular_offset_deg(method_heading_deg: list[float] | tuple[float, ...], reference_heading_deg: list[float] | tuple[float, ...]) -> float | None:
    if not method_heading_deg or len(method_heading_deg) != len(reference_heading_deg):
        return None
    sin_sum = 0.0
    cos_sum = 0.0
    count = 0
    for method_heading, reference_heading in zip(method_heading_deg, reference_heading_deg):
        diff_rad = math.radians(wrap_180_deg(float(method_heading) - float(reference_heading)))
        sin_sum += math.sin(diff_rad)
        cos_sum += math.cos(diff_rad)
        count += 1
    if count == 0 or (math.isclose(sin_sum, 0.0, abs_tol=1e-12) and math.isclose(cos_sum, 0.0, abs_tol=1e-12)):
        return None
    return wrap_180_deg(math.degrees(math.atan2(sin_sum, cos_sum)))


def build_analysis_result(
    snapshot: AnalysisSnapshot,
    *,
    method_ids: tuple[str, ...] | list[str] | set[str] | None = None,
) -> MagnetometerAnalysisResult:
    base_ts = _base_timestamp(snapshot)
    raw_points = _magnetometer_series(snapshot.raw_records, base_ts)
    reference_points = _heading_series(snapshot.reference_records, base_ts)
    tilt_points = _tilt_series(snapshot.tilt_records, base_ts)
    methods_by_id = {method.method_id: method for method in snapshot.methods}
    if method_ids is None:
        selected_ids = [RAW_METHOD_ID]
        selected_ids.extend(method.method_id for method in snapshot.methods if method.analyze_enabled or method.view_enabled)
    else:
        selected_ids = [
            method_id
            for method_id in method_ids
            if method_id == RAW_METHOD_ID or method_id in methods_by_id
        ]

    warnings: list[str] = []
    if not snapshot.raw_records:
        warnings.append("No raw magnetometer records in snapshot")
    if not snapshot.reference_records:
        warnings.append("No GNSS/reference heading records in snapshot")
    if not snapshot.tilt_records:
        warnings.append("No paired tilt records in snapshot")

    method_results: list[MethodHeadingAnalysis] = []
    for method_id in selected_ids:
        if method_id == RAW_METHOD_ID:
            if not snapshot.raw_records:
                warnings.append("Raw magnetometer: no raw records")
                continue
            method_results.append(
                _build_method_analysis(
                    _raw_method_state(len(snapshot.raw_records)),
                    tuple(snapshot.raw_records),
                    reference_points,
                    tilt_points,
                    base_ts,
                )
            )
            continue
        method_state = methods_by_id[method_id]
        records = tuple(snapshot.derived_records_by_method_id.get(method_id, ()))
        if not records:
            warnings.append(f"{method_state.name}: no offline derived records")
            continue
        method_results.append(_build_method_analysis(method_state, records, reference_points, tilt_points, base_ts))

    dataset_clouds = tuple(
        _build_dataset_cloud(record_set, base_ts)
        for record_set in snapshot.dataset_records
        if record_set.raw_records
    )
    if not dataset_clouds and raw_points["x"]:
        active = snapshot.active_dataset
        dataset_clouds = (
            DatasetCloudAnalysis(
                name="active dataset" if active is None else active.name,
                source_path="" if active is None else active.source_path,
                is_active=True,
                magnet_label=_magnet_label("" if active is None else f"{active.name} {active.source_path}"),
                times_s=raw_points["times"],
                mag_x=raw_points["x"],
                mag_y=raw_points["y"],
                mag_z=raw_points["z"],
                heading_deg=raw_points["heading"],
            ),
        )

    stress_rows = tuple(_build_stress_rows(snapshot, method_results))
    return MagnetometerAnalysisResult(
        raw_times_s=raw_points["times"],
        raw_x=raw_points["x"],
        raw_y=raw_points["y"],
        raw_z=raw_points["z"],
        raw_heading_deg=raw_points["heading"],
        reference_times_s=reference_points["times"],
        reference_heading_deg=reference_points["heading"],
        tilt_times_s=tilt_points["times"],
        tilt_roll_deg=tilt_points["roll"],
        tilt_pitch_deg=tilt_points["pitch"],
        tilt_alpha_deg=tilt_points["alpha"],
        methods=tuple(method_results),
        dataset_clouds=dataset_clouds,
        stress_rows=stress_rows,
        warnings=tuple(warnings),
    )


def raw_method_state(record_count: int = 0) -> AnalysisMethodState:
    return _raw_method_state(record_count)


def _raw_method_state(record_count: int = 0) -> AnalysisMethodState:
    return AnalysisMethodState(
        method_id=RAW_METHOD_ID,
        name="Raw magnetometer",
        version="builtin",
        file_path="",
        status="ready" if record_count else "partial",
        status_text="raw data" if record_count else "no raw data",
        calibrated=False,
        supports_process=True,
        analyze_enabled=record_count > 0,
        view_enabled=record_count > 0,
        warning_count=0,
        offline_record_count=record_count,
        derived_stream_id="raw_magnetometer",
    )


def _build_method_analysis(
    method: AnalysisMethodState,
    records: tuple[Any, ...],
    reference_points: Mapping[str, tuple[float, ...]],
    tilt_points: Mapping[str, tuple[float, ...]],
    base_ts: int,
) -> MethodHeadingAnalysis:
    series = _magnetometer_series(records, base_ts)
    metrics: dict[str, float | int | None] = _radius_metrics(series["x"], series["y"], series["radius"])
    metrics["sample_count"] = len(series["heading"])
    metrics["matched_count"] = 0

    matched_times: list[float] = []
    matched_headings: list[float] = []
    gnss_headings: list[float] = []
    if reference_points["times"] and series["times"]:
        for index, (time_s, heading) in enumerate(zip(series["times"], series["heading"])):
            ref_heading = _interp_heading_at(reference_points["times"], reference_points["heading"], time_s)
            if ref_heading is None:
                continue
            matched_times.append(time_s)
            matched_headings.append(heading)
            gnss_headings.append(ref_heading)

    aligned_heading: list[float] = []
    aligned_errors: list[float] = []
    absolute_errors: list[float] = []
    delta_times: list[float] = []
    delta_errors: list[float] = []
    roll_values: list[float] = []
    pitch_values: list[float] = []
    alpha_values: list[float] = []
    tilt_bins: tuple[TiltBinMetric, ...] = ()

    offset = circular_offset_deg(matched_headings, gnss_headings)
    metrics["alignment_offset_deg"] = offset
    if offset is not None:
        for method_heading, ref_heading in zip(matched_headings, gnss_headings):
            aligned_heading.append(wrap_360_deg(method_heading - offset))
            aligned_errors.append(wrap_180_deg(method_heading - ref_heading - offset))
            absolute_errors.append(wrap_180_deg(method_heading - ref_heading))
        metrics.update(_error_metrics("aligned", aligned_errors))
        metrics.update(_error_metrics("absolute", absolute_errors))
        metrics["matched_count"] = len(aligned_errors)

        for idx in range(1, len(matched_headings)):
            delta_method = wrap_180_deg(matched_headings[idx] - matched_headings[idx - 1])
            delta_ref = wrap_180_deg(gnss_headings[idx] - gnss_headings[idx - 1])
            delta_times.append(matched_times[idx])
            delta_errors.append(wrap_180_deg(delta_method - delta_ref))
        metrics.update(_error_metrics("delta", delta_errors))
        if len(matched_headings) >= 2:
            method_closure = wrap_180_deg(matched_headings[-1] - matched_headings[0])
            ref_closure = wrap_180_deg(gnss_headings[-1] - gnss_headings[0])
            metrics["closure_method_deg"] = method_closure
            metrics["closure_reference_deg"] = ref_closure
            metrics["closure_relative_deg"] = wrap_180_deg(method_closure - ref_closure)

    if tilt_points["times"] and matched_times and aligned_errors:
        for time_s in matched_times:
            roll = _interp_numeric_at(tilt_points["times"], tilt_points["roll"], time_s)
            pitch = _interp_numeric_at(tilt_points["times"], tilt_points["pitch"], time_s)
            if roll is None or pitch is None:
                roll_values.append(float("nan"))
                pitch_values.append(float("nan"))
                alpha_values.append(float("nan"))
                continue
            roll_values.append(roll)
            pitch_values.append(pitch)
            alpha_values.append(math.hypot(roll, pitch))
        paired = [
            (error, roll, pitch, alpha)
            for error, roll, pitch, alpha in zip(aligned_errors, roll_values, pitch_values, alpha_values)
            if all(math.isfinite(value) for value in (error, roll, pitch, alpha))
        ]
        if paired:
            clean_errors = [item[0] for item in paired]
            clean_roll = [item[1] for item in paired]
            clean_pitch = [item[2] for item in paired]
            clean_alpha = [item[3] for item in paired]
            metrics["tilt_matched_count"] = len(paired)
            metrics["tilt_roll_corr"] = _correlation(clean_errors, clean_roll)
            metrics["tilt_pitch_corr"] = _correlation(clean_errors, clean_pitch)
            metrics["tilt_alpha_corr_abs"] = _correlation([abs(error) for error in clean_errors], clean_alpha)
            tilt_bins = tuple(_tilt_bin_metrics(clean_errors, clean_alpha))

    return MethodHeadingAnalysis(
        method_id=method.method_id,
        name=method.name,
        times_s=series["times"],
        mag_x=series["x"],
        mag_y=series["y"],
        mag_z=series["z"],
        heading_deg=series["heading"],
        radius_xy=series["radius"],
        matched_times_s=tuple(matched_times),
        gnss_heading_deg=tuple(gnss_headings),
        aligned_heading_deg=tuple(aligned_heading),
        aligned_error_deg=tuple(aligned_errors),
        absolute_error_deg=tuple(absolute_errors),
        delta_times_s=tuple(delta_times),
        delta_error_deg=tuple(delta_errors),
        roll_deg=tuple(roll_values),
        pitch_deg=tuple(pitch_values),
        alpha_deg=tuple(alpha_values),
        tilt_bins=tilt_bins,
        metrics=metrics,
    )


def _build_dataset_cloud(record_set: AnalysisDatasetRecords, base_ts: int) -> DatasetCloudAnalysis:
    series = _magnetometer_series(record_set.raw_records, base_ts)
    return DatasetCloudAnalysis(
        name=record_set.name,
        source_path=record_set.source_path,
        is_active=record_set.is_active,
        magnet_label=_magnet_label(f"{record_set.name} {record_set.source_path}"),
        times_s=series["times"],
        mag_x=series["x"],
        mag_y=series["y"],
        mag_z=series["z"],
        heading_deg=series["heading"],
    )


def _build_stress_rows(snapshot: AnalysisSnapshot, methods: list[MethodHeadingAnalysis]) -> list[Mapping[str, Any]]:
    active_label = _magnet_label(
        "" if snapshot.active_dataset is None else f"{snapshot.active_dataset.name} {snapshot.active_dataset.source_path}"
    )
    states = {method.method_id: method for method in snapshot.methods}
    rows: list[Mapping[str, Any]] = []
    for result in methods:
        state = states.get(result.method_id)
        calibration_name = "" if state is None else state.calibration_dataset_name
        calibration_label = _magnet_label(calibration_name)
        rows.append(
            {
                "method": result.name,
                "calibration": calibration_label,
                "validation": active_label,
                "mode": _stress_mode(calibration_label, active_label),
                "rmse_deg": result.metrics.get("aligned_rmse_deg"),
                "p95_deg": result.metrics.get("aligned_p95_deg"),
                "matched_count": result.metrics.get("matched_count"),
            }
        )
    return rows


def _magnetometer_series(records: tuple[Any, ...], base_ts: int) -> dict[str, tuple[float, ...]]:
    points: list[tuple[float, float, float, float, float, float]] = []
    for record in sorted(records, key=lambda item: _timestamp_ms(item) or 0):
        ts = _timestamp_ms(record)
        mx = _finite_float(getattr(record, "mag_x", None))
        my = _finite_float(getattr(record, "mag_y", None))
        mz = _finite_float(getattr(record, "mag_z", None))
        if ts is None or mx is None or my is None or mz is None:
            continue
        heading = _finite_float(getattr(record, "heading", None))
        if heading is None:
            heading = heading_from_xy_deg(mx, my)
        if heading is None:
            continue
        points.append(((ts - base_ts) / 1000.0, mx, my, mz, wrap_360_deg(heading), math.hypot(mx, my)))
    return {
        "times": tuple(item[0] for item in points),
        "x": tuple(item[1] for item in points),
        "y": tuple(item[2] for item in points),
        "z": tuple(item[3] for item in points),
        "heading": tuple(item[4] for item in points),
        "radius": tuple(item[5] for item in points),
    }


def _heading_series(records: tuple[Any, ...], base_ts: int) -> dict[str, tuple[float, ...]]:
    points: list[tuple[float, float]] = []
    seen: dict[int, float] = {}
    for record in records:
        ts = _timestamp_ms(record)
        heading = _finite_float(getattr(record, "heading", None))
        if ts is None or heading is None:
            continue
        seen[ts] = wrap_360_deg(heading)
    for ts in sorted(seen):
        points.append(((ts - base_ts) / 1000.0, seen[ts]))
    return {
        "times": tuple(item[0] for item in points),
        "heading": tuple(item[1] for item in points),
    }


def _tilt_series(records: tuple[Any, ...], base_ts: int) -> dict[str, tuple[float, ...]]:
    points: list[tuple[float, float, float, float]] = []
    for record in sorted(records, key=lambda item: _timestamp_ms(item) or 0):
        ts = _timestamp_ms(record)
        roll = _finite_float(getattr(record, "roll_deg", None))
        pitch = _finite_float(getattr(record, "pitch_deg", None))
        if ts is None:
            continue
        if roll is None or pitch is None:
            roll, pitch = _tilt_from_accel(record)
        if roll is None or pitch is None:
            continue
        points.append(((ts - base_ts) / 1000.0, roll, pitch, math.hypot(roll, pitch)))
    return {
        "times": tuple(item[0] for item in points),
        "roll": tuple(item[1] for item in points),
        "pitch": tuple(item[2] for item in points),
        "alpha": tuple(item[3] for item in points),
    }


def _tilt_from_accel(record: Any) -> tuple[float | None, float | None]:
    ax = _finite_float(getattr(record, "acc_x", None))
    ay = _finite_float(getattr(record, "acc_y", None))
    az = _finite_float(getattr(record, "acc_z", None))
    if ax is None or ay is None or az is None:
        return (None, None)
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    if math.isclose(norm, 0.0, abs_tol=1e-12):
        return (None, None)
    roll = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
    return (roll, pitch)


def _interp_heading_at(times: tuple[float, ...], headings: tuple[float, ...], target_time: float) -> float | None:
    if len(times) == 0 or len(times) != len(headings):
        return None
    if target_time < times[0] or target_time > times[-1]:
        return None
    if len(times) == 1:
        return headings[0] if math.isclose(target_time, times[0], abs_tol=1e-9) else None
    unwrapped = unwrap_degrees(headings)
    value = _interp_unwrapped(times, tuple(unwrapped), target_time)
    return None if value is None else wrap_360_deg(value)


def _interp_numeric_at(times: tuple[float, ...], values: tuple[float, ...], target_time: float) -> float | None:
    if len(times) == 0 or len(times) != len(values):
        return None
    if target_time < times[0] or target_time > times[-1]:
        return None
    return _interp_unwrapped(times, values, target_time)


def _interp_unwrapped(times: tuple[float, ...], values: tuple[float, ...], target_time: float) -> float | None:
    if len(times) == 1:
        return values[0] if math.isclose(target_time, times[0], abs_tol=1e-9) else None
    lo = 0
    hi = len(times) - 1
    while lo <= hi:
        mid = (lo + hi) // 2
        if math.isclose(times[mid], target_time, abs_tol=1e-12):
            return values[mid]
        if times[mid] < target_time:
            lo = mid + 1
        else:
            hi = mid - 1
    right = lo
    left = right - 1
    if left < 0 or right >= len(times):
        return None
    dt = times[right] - times[left]
    if math.isclose(dt, 0.0, abs_tol=1e-12):
        return values[left]
    ratio = (target_time - times[left]) / dt
    return values[left] + (values[right] - values[left]) * ratio


def _error_metrics(prefix: str, errors_deg: list[float] | tuple[float, ...]) -> dict[str, float | int | None]:
    values = [float(value) for value in errors_deg if math.isfinite(float(value))]
    abs_values = [abs(value) for value in values]
    if not values:
        return {
            f"{prefix}_count": 0,
            f"{prefix}_mae_deg": None,
            f"{prefix}_rmse_deg": None,
            f"{prefix}_medae_deg": None,
            f"{prefix}_p95_deg": None,
            f"{prefix}_maxae_deg": None,
            f"{prefix}_bias_deg": None,
        }
    return {
        f"{prefix}_count": len(values),
        f"{prefix}_mae_deg": sum(abs_values) / len(abs_values),
        f"{prefix}_rmse_deg": math.sqrt(sum(value * value for value in values) / len(values)),
        f"{prefix}_medae_deg": _percentile(abs_values, 0.50),
        f"{prefix}_p95_deg": _percentile(abs_values, 0.95),
        f"{prefix}_maxae_deg": max(abs_values),
        f"{prefix}_bias_deg": sum(values) / len(values),
    }


def _radius_metrics(xs: tuple[float, ...], ys: tuple[float, ...], radii: tuple[float, ...]) -> dict[str, float | int | None]:
    values = [radius for radius in radii if math.isfinite(radius)]
    if not values:
        return {
            "radius_mean": None,
            "radius_std": None,
            "radius_cv": None,
            "center_offset": None,
        }
    mean_radius = sum(values) / len(values)
    if len(values) > 1:
        variance = sum((value - mean_radius) ** 2 for value in values) / (len(values) - 1)
    else:
        variance = 0.0
    mean_x = sum(xs) / len(xs) if xs else 0.0
    mean_y = sum(ys) / len(ys) if ys else 0.0
    return {
        "radius_mean": mean_radius,
        "radius_std": math.sqrt(max(variance, 0.0)),
        "radius_cv": None if math.isclose(mean_radius, 0.0, abs_tol=1e-12) else math.sqrt(max(variance, 0.0)) / abs(mean_radius),
        "center_offset": math.hypot(mean_x, mean_y),
    }


def _tilt_bin_metrics(errors: list[float], alphas: list[float]) -> list[TiltBinMetric]:
    bins = (
        ("0..3 deg", 0.0, 3.0),
        ("3..6 deg", 3.0, 6.0),
        ("6..10 deg", 6.0, 10.0),
        (">=10 deg", 10.0, None),
    )
    rows: list[TiltBinMetric] = []
    for label, lower, upper in bins:
        selected = [
            error
            for error, alpha in zip(errors, alphas)
            if alpha >= lower and (upper is None or alpha < upper)
        ]
        metrics = _error_metrics("bin", selected)
        rows.append(
            TiltBinMetric(
                label=label,
                lower_deg=lower,
                upper_deg=upper,
                count=int(metrics["bin_count"] or 0),
                mae_deg=metrics["bin_mae_deg"],
                rmse_deg=metrics["bin_rmse_deg"],
                p95_deg=metrics["bin_p95_deg"],
            )
        )
    return rows


def _percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * q
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    ratio = position - lower
    return ordered[lower] * (1.0 - ratio) + ordered[upper] * ratio


def _correlation(xs: list[float], ys: list[float]) -> float | None:
    if len(xs) < 2 or len(xs) != len(ys):
        return None
    mean_x = sum(xs) / len(xs)
    mean_y = sum(ys) / len(ys)
    num = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
    den_x = math.sqrt(sum((x - mean_x) ** 2 for x in xs))
    den_y = math.sqrt(sum((y - mean_y) ** 2 for y in ys))
    den = den_x * den_y
    if math.isclose(den, 0.0, abs_tol=1e-12):
        return None
    return num / den


def _base_timestamp(snapshot: AnalysisSnapshot) -> int:
    timestamps: list[int] = []
    for records in (
        snapshot.raw_records,
        snapshot.reference_records,
        snapshot.tilt_records,
        *(tuple(records) for records in snapshot.derived_records_by_method_id.values()),
    ):
        for record in records:
            ts = _timestamp_ms(record)
            if ts is not None:
                timestamps.append(ts)
    for record_set in snapshot.dataset_records:
        for record in record_set.raw_records:
            ts = _timestamp_ms(record)
            if ts is not None:
                timestamps.append(ts)
    return min(timestamps) if timestamps else 0


def _timestamp_ms(record: Any) -> int | None:
    for attr in ("timestamp_mcu", "timestamp_pc_est", "timestamp_pc_rx"):
        value = getattr(record, attr, None)
        if value is None:
            continue
        try:
            return int(value)
        except (TypeError, ValueError):
            continue
    return None


def _finite_float(value: Any) -> float | None:
    if value is None or value == "":
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _magnet_label(text: str) -> str:
    lower = text.lower()
    without_tokens = ("without magnet", "no magnet", "no_magnet", "clear", "vclear", "без магнит", "bez magn")
    magnet_text = lower.replace("magnetometer", "")
    with_tokens = ("with magnet", "with_magnet", "magnet_on", "_magnet", "magnet-", "magnit", "магнит", "mag_")
    if any(token in lower for token in without_tokens):
        return "without magnet"
    if any(token in magnet_text for token in with_tokens):
        return "with magnet"
    return "unknown"


def _stress_mode(calibration_label: str, validation_label: str) -> str:
    if calibration_label == "without magnet" and validation_label == "without magnet":
        return "A matched without magnet"
    if calibration_label == "with magnet" and validation_label == "with magnet":
        return "B matched with magnet"
    if calibration_label == "without magnet" and validation_label == "with magnet":
        return "C cross without->with"
    if calibration_label == "with magnet" and validation_label == "without magnet":
        return "D cross with->without"
    return "unknown"


def _coerce_int(value: Any, default: int) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return int(default)


def _safe_call(obj: Any, name: str, *, default: Any) -> Any:
    func = getattr(obj, name, None)
    if not callable(func):
        return default
    try:
        return func()
    except Exception:
        return default


def _safe_bool_call(obj: Any, name: str) -> bool:
    return bool(_safe_call(obj, name, default=False))


def _safe_list_call(obj: Any, name: str) -> list[Any]:
    result = _safe_call(obj, name, default=[])
    if isinstance(result, (list, tuple, set)):
        return list(result)
    return []
