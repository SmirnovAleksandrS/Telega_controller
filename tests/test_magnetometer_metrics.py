from __future__ import annotations

import os
import tempfile
import types
import unittest

from app.magnetometer_dataset import Dataset, SampleRecord
from app.magnetometer_metrics import compute_metrics_report, fit_circle_kasa
from app.magnetometer_plugin_loader import LoadedMethodPlugin


class MagnetometerMetricsTests(unittest.TestCase):
    def test_fit_circle_kasa_recovers_circle_parameters(self) -> None:
        points = [
            (7.0, -1.0),
            (2.0, 4.0),
            (-3.0, -1.0),
            (2.0, -6.0),
        ]

        fitted = fit_circle_kasa(points)

        self.assertIsNotNone(fitted)
        assert fitted is not None
        cx, cy, radius = fitted
        self.assertAlmostEqual(cx, 2.0, places=6)
        self.assertAlmostEqual(cy, -1.0, places=6)
        self.assertAlmostEqual(radius, 5.0, places=6)

    def test_metrics_report_marks_rmse_unavailable_without_reference_stream(self) -> None:
        dataset = Dataset("session")
        dataset.extend([
            SampleRecord("raw_magnetometer", "raw", "Raw Magnetometer", "builtin", 100, 110, 108, 2.0, 0.0, 0.0, 90.0, ""),
            SampleRecord("raw_magnetometer", "raw", "Raw Magnetometer", "builtin", 200, 210, 208, 0.0, 2.0, 0.0, 0.0, ""),
            SampleRecord("raw_magnetometer", "raw", "Raw Magnetometer", "builtin", 300, 310, 308, -2.0, 0.0, 0.0, 270.0, ""),
            SampleRecord("raw_magnetometer", "raw", "Raw Magnetometer", "builtin", 400, 410, 408, 0.0, -2.0, 0.0, 180.0, ""),
        ])
        plugin = LoadedMethodPlugin(
            method_id="method_1",
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
                "supports_process": True,
                "input_schema": {"mag": "xyz"},
                "output_schema": {"mag": "xyz", "heading": "deg"},
            },
            module=types.SimpleNamespace(process=lambda sample, params: dict(sample)),
            status="ready",
        )
        plugin.calibration_runtime_s = 0.75

        report = compute_metrics_report(dataset, plugin)

        self.assertEqual(len(report.rows), 6)
        rmse_row = report.rows[0]
        self.assertEqual(rmse_row.key, "rmse_vs_gnss_heading")
        self.assertIsNone(rmse_row.value)
        self.assertEqual(rmse_row.status, "unavailable")
        self.assertIn("No GNSS/reference", rmse_row.notes)
        self.assertEqual(report.rows[4].value, 0.75)

    def test_metrics_report_computes_rmse_with_reference_stream(self) -> None:
        dataset = Dataset("session")
        dataset.extend([
            SampleRecord("raw_magnetometer", "raw", "Raw Magnetometer", "builtin", 100, 110, 108, 1.0, 0.0, 0.0, 90.0, ""),
            SampleRecord("gnss_heading", "reference", "GNSS Heading", "builtin", 100, 111, 109, 0.0, 0.0, 0.0, 90.0, ""),
            SampleRecord("raw_magnetometer", "raw", "Raw Magnetometer", "builtin", 200, 210, 208, 0.0, 1.0, 0.0, 0.0, ""),
            SampleRecord("gnss_heading", "reference", "GNSS Heading", "builtin", 200, 211, 209, 0.0, 0.0, 0.0, 0.0, ""),
            SampleRecord("raw_magnetometer", "raw", "Raw Magnetometer", "builtin", 300, 310, 308, -1.0, 0.0, 0.0, 270.0, ""),
            SampleRecord("gnss_heading", "reference", "GNSS Heading", "builtin", 300, 311, 309, 0.0, 0.0, 0.0, 270.0, ""),
        ])
        plugin = LoadedMethodPlugin(
            method_id="method_2",
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
                "supports_process": True,
                "input_schema": {"mag": "xyz"},
                "output_schema": {"mag": "xyz", "heading": "deg"},
            },
            module=types.SimpleNamespace(process=lambda sample, params: dict(sample)),
            status="ready",
        )

        report = compute_metrics_report(dataset, plugin)

        self.assertEqual(report.rows[0].status, "ok")
        self.assertAlmostEqual(report.rows[0].value or 0.0, 0.0, places=6)
        self.assertIn("Matched 3", report.rows[0].notes)

    def test_metrics_report_export_writes_csv(self) -> None:
        dataset = Dataset("session")
        dataset.extend([
            SampleRecord("raw_magnetometer", "raw", "Raw Magnetometer", "builtin", 100, 110, 108, 1.0, 0.0, 0.0, 90.0, ""),
            SampleRecord("raw_magnetometer", "raw", "Raw Magnetometer", "builtin", 200, 210, 208, 0.0, 1.0, 0.0, 0.0, ""),
            SampleRecord("raw_magnetometer", "raw", "Raw Magnetometer", "builtin", 300, 310, 308, -1.0, 0.0, 0.0, 270.0, ""),
        ])
        plugin = LoadedMethodPlugin(
            method_id="method_3",
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
                "supports_process": True,
                "input_schema": {"mag": "xyz"},
                "output_schema": {"mag": "xyz", "heading": "deg"},
            },
            module=types.SimpleNamespace(process=lambda sample, params: dict(sample)),
            status="ready",
        )

        report = compute_metrics_report(dataset, plugin)

        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "metrics.csv")
            report.export_csv(path)
            with open(path, "r", encoding="utf-8") as fh:
                text = fh.read()

        self.assertIn("metric_key,metric_label,value,units,status,notes", text)
        self.assertIn("mean_radius", text)


if __name__ == "__main__":
    unittest.main()
