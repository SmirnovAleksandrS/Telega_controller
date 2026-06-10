from __future__ import annotations

import types
import unittest

from app.accelerometer_dataset import AccelerometerDataset, AccelerometerSampleRecord
from app.magnetometer_analysis import (
    AnalysisSnapshot,
    RAW_METHOD_ID,
    build_analysis_result,
    build_data_availability,
    build_method_state,
    circular_offset_deg,
    default_figure_slots,
    summarize_dataset,
    unwrap_degrees,
    wrap_180_deg,
)
from app.magnetometer_dataset import Dataset, SampleRecord
from app.magnetometer_plugin_loader import LoadedMethodPlugin


class MagnetometerAnalysisContractsTests(unittest.TestCase):
    def test_availability_detects_raw_gnss_and_paired_tilt(self) -> None:
        dataset = Dataset("session")
        dataset.extend([
            SampleRecord("raw_magnetometer", "raw", "Raw Magnetometer", "builtin", 100, 110, 108, 1.0, 0.0, 0.0, 90.0, ""),
            SampleRecord("gnss_heading", "reference", "GNSS Heading", "builtin", 100, 111, 109, 0.0, 0.0, 0.0, 91.0, "heading_only"),
        ])
        acc_dataset = AccelerometerDataset("session")
        acc_dataset.extend([
            AccelerometerSampleRecord("raw_tilt", "raw", "Raw Tilt", "builtin", 100, 110, 108, 0.0, 0.0, 1.0, 0.0, 1.5, ""),
        ])

        availability = build_data_availability(dataset, acc_dataset)

        self.assertTrue(availability.has_active_dataset)
        self.assertTrue(availability.has_raw_magnetometer)
        self.assertTrue(availability.has_gnss_heading)
        self.assertTrue(availability.has_tilt)
        self.assertEqual(availability.raw_sample_count, 1)
        self.assertEqual(availability.gnss_sample_count, 1)
        self.assertEqual(availability.tilt_sample_count, 1)

    def test_method_state_marks_calibrated_from_params(self) -> None:
        plugin = _make_plugin()
        plugin.calibration_params = {"algorithm_name": "Identity"}

        state = build_method_state("method_1", plugin, offline_record_count=3)

        self.assertTrue(state.calibrated)
        self.assertTrue(state.supports_process)
        self.assertTrue(state.analyze_enabled)
        self.assertTrue(state.view_enabled)
        self.assertEqual(state.offline_record_count, 3)

    def test_method_without_process_or_params_is_shown_but_not_analyzable(self) -> None:
        plugin = _make_plugin(supports_process=False)

        state = build_method_state("method_2", plugin, offline_record_count=0)

        self.assertFalse(state.calibrated)
        self.assertFalse(state.supports_process)
        self.assertFalse(state.analyze_enabled)
        self.assertFalse(state.view_enabled)

    def test_summarize_dataset_uses_dataset_summary(self) -> None:
        dataset = Dataset("session")
        dataset.append(SampleRecord("raw_magnetometer", "raw", "Raw Magnetometer", "builtin", 10, 11, 10, 1.0, 0.0, 0.0, 90.0, ""))

        summary = summarize_dataset(dataset, is_active=True)

        self.assertIsNotNone(summary)
        assert summary is not None
        self.assertEqual(summary.name, "session")
        self.assertEqual(summary.row_count, 1)
        self.assertTrue(summary.is_active)

    def test_angle_helpers_handle_wraparound(self) -> None:
        self.assertAlmostEqual(wrap_180_deg(358.0), -2.0)
        self.assertAlmostEqual(wrap_180_deg(-358.0), 2.0)
        self.assertEqual(unwrap_degrees((350.0, 10.0, 20.0)), [350.0, 370.0, 380.0])
        self.assertAlmostEqual(circular_offset_deg((20.0, 40.0, 60.0), (350.0, 10.0, 30.0)) or 0.0, 30.0)

    def test_build_analysis_result_computes_aligned_absolute_delta_and_tilt_metrics(self) -> None:
        plugin = _make_plugin()
        plugin.calibration_params = {"algorithm_name": "Synthetic"}
        state = build_method_state("method_1", plugin, offline_record_count=3)
        raw_records = (
            SampleRecord("raw_magnetometer", "raw", "Raw Magnetometer", "builtin", 0, 0, 0, 0.0, 1.0, 0.0, 0.0, ""),
            SampleRecord("raw_magnetometer", "raw", "Raw Magnetometer", "builtin", 1000, 1000, 1000, 1.0, 0.0, 0.0, 90.0, ""),
            SampleRecord("raw_magnetometer", "raw", "Raw Magnetometer", "builtin", 2000, 2000, 2000, 0.0, -1.0, 0.0, 180.0, ""),
        )
        gnss_records = (
            SampleRecord("gnss_heading", "reference", "GNSS Heading", "builtin", 0, 0, 0, 0.0, 0.0, 0.0, 350.0, "heading_only"),
            SampleRecord("gnss_heading", "reference", "GNSS Heading", "builtin", 1000, 1000, 1000, 0.0, 0.0, 0.0, 10.0, "heading_only"),
            SampleRecord("gnss_heading", "reference", "GNSS Heading", "builtin", 2000, 2000, 2000, 0.0, 0.0, 0.0, 30.0, "heading_only"),
        )
        derived_records = (
            SampleRecord("derived_method_1", "derived", "Synthetic", "1.0.0", 0, 0, 0, 1.0, 0.0, 0.0, 20.0, ""),
            SampleRecord("derived_method_1", "derived", "Synthetic", "1.0.0", 1000, 1000, 1000, 1.0, 0.0, 0.0, 40.0, ""),
            SampleRecord("derived_method_1", "derived", "Synthetic", "1.0.0", 2000, 2000, 2000, 1.0, 0.0, 0.0, 60.0, ""),
        )
        tilt_records = (
            AccelerometerSampleRecord("raw_tilt", "raw", "Raw Tilt", "builtin", 0, 0, 0, 0.0, 0.0, 1.0, 1.0, 1.0, ""),
            AccelerometerSampleRecord("raw_tilt", "raw", "Raw Tilt", "builtin", 1000, 1000, 1000, 0.0, 0.0, 1.0, 4.0, 0.0, ""),
            AccelerometerSampleRecord("raw_tilt", "raw", "Raw Tilt", "builtin", 2000, 2000, 2000, 0.0, 0.0, 1.0, 11.0, 0.0, ""),
        )
        dataset = Dataset("validation_clear", records=list(raw_records) + list(gnss_records))
        availability = build_data_availability(dataset, None, (state,))
        snapshot = AnalysisSnapshot(
            active_dataset=summarize_dataset(dataset, is_active=True),
            datasets=(summarize_dataset(dataset, is_active=True),),
            availability=availability,
            methods=(state,),
            figure_slots=default_figure_slots(),
            raw_records=raw_records,
            reference_records=gnss_records,
            tilt_records=tilt_records,
            derived_records_by_method_id={"method_1": derived_records},
        )

        result = build_analysis_result(snapshot, method_ids=("method_1",))

        self.assertEqual(len(result.methods), 1)
        method = result.methods[0]
        self.assertAlmostEqual(method.metrics["alignment_offset_deg"] or 0.0, 30.0)
        self.assertIsNotNone(method.metrics["aligned_rmse_deg"])
        self.assertAlmostEqual(float(method.metrics["aligned_rmse_deg"] or 0.0), 0.0)
        self.assertAlmostEqual(method.metrics["absolute_rmse_deg"] or 0.0, 30.0)
        self.assertIsNotNone(method.metrics["delta_rmse_deg"])
        self.assertAlmostEqual(float(method.metrics["delta_rmse_deg"] or 0.0), 0.0)
        self.assertEqual(method.metrics["matched_count"], 3)
        self.assertEqual(len(method.tilt_bins), 4)
        self.assertEqual(sum(item.count for item in method.tilt_bins), 3)

    def test_build_analysis_result_can_compare_raw_as_method(self) -> None:
        dataset = Dataset("raw_validation")
        raw_records = (
            SampleRecord("raw_magnetometer", "raw", "Raw Magnetometer", "builtin", 0, 0, 0, 0.0, 1.0, 0.0, 0.0, ""),
            SampleRecord("raw_magnetometer", "raw", "Raw Magnetometer", "builtin", 1000, 1000, 1000, 1.0, 0.0, 0.0, 90.0, ""),
            SampleRecord("raw_magnetometer", "raw", "Raw Magnetometer", "builtin", 2000, 2000, 2000, 0.0, -1.0, 0.0, 180.0, ""),
        )
        gnss_records = (
            SampleRecord("gnss_heading", "reference", "GNSS Heading", "builtin", 0, 0, 0, 0.0, 0.0, 0.0, 10.0, "heading_only"),
            SampleRecord("gnss_heading", "reference", "GNSS Heading", "builtin", 1000, 1000, 1000, 0.0, 0.0, 0.0, 100.0, "heading_only"),
            SampleRecord("gnss_heading", "reference", "GNSS Heading", "builtin", 2000, 2000, 2000, 0.0, 0.0, 0.0, 190.0, "heading_only"),
        )
        dataset.extend(list(raw_records) + list(gnss_records))
        snapshot = AnalysisSnapshot(
            active_dataset=summarize_dataset(dataset, is_active=True),
            datasets=(summarize_dataset(dataset, is_active=True),),
            availability=build_data_availability(dataset, None),
            methods=(),
            figure_slots=default_figure_slots(),
            raw_records=raw_records,
            reference_records=gnss_records,
        )

        result = build_analysis_result(snapshot, method_ids=(RAW_METHOD_ID,))

        self.assertEqual(len(result.methods), 1)
        raw_method = result.methods[0]
        self.assertEqual(raw_method.method_id, RAW_METHOD_ID)
        self.assertEqual(raw_method.name, "Raw magnetometer")
        self.assertAlmostEqual(raw_method.metrics["alignment_offset_deg"] or 0.0, -10.0)
        self.assertAlmostEqual(float(raw_method.metrics["aligned_rmse_deg"] or 0.0), 0.0)
        self.assertEqual(raw_method.metrics["matched_count"], 3)


def _make_plugin(*, supports_process: bool = True) -> LoadedMethodPlugin:
    return LoadedMethodPlugin(
        method_id="",
        name="Identity",
        version="1.0.0",
        file_path="/tmp/identity.py",
        info={
            "name": "Identity",
            "version": "1.0.0",
            "type": "method",
            "supports_calibrate": True,
            "supports_load_params": True,
            "supports_save_params": True,
            "supports_process": supports_process,
            "input_schema": {"mag": "xyz"},
            "output_schema": {"mag": "xyz", "heading": "deg"},
        },
        module=types.SimpleNamespace(process=lambda sample, params: dict(sample)),
        status="ready",
    )


if __name__ == "__main__":
    unittest.main()
