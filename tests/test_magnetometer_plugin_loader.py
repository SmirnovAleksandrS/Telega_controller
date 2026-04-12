from __future__ import annotations

import math
import os
import tempfile
import textwrap
import unittest

from app.magnetometer_dataset import Dataset, SampleRecord
from app.magnetometer_plugin_loader import (
    load_method_plugin,
    load_method_params,
    run_method_calibration,
    run_method_process,
    save_method_params,
)


class MagnetometerPluginLoaderTests(unittest.TestCase):
    def test_valid_plugin_loads_successfully(self) -> None:
        plugin_code = textwrap.dedent(
            """
            import json

            def get_info():
                return {
                    "name": "Example Loader Plugin",
                    "version": "1.0.0",
                    "type": "method",
                    "supports_calibrate": True,
                    "supports_load_params": True,
                    "supports_save_params": True,
                    "supports_process": True,
                    "input_schema": {"mag_x": "float", "mag_y": "float", "mag_z": "float"},
                    "output_schema": {"heading": "float"},
                }

            def calibrate(dataset, config=None):
                return {"offset": [0.0, 0.0, 0.0]}

            def load_params(path):
                with open(path, "r", encoding="utf-8") as fh:
                    return json.load(fh)

            def save_params(path, params):
                with open(path, "w", encoding="utf-8") as fh:
                    json.dump(params, fh)

            def process(sample, params):
                return sample
            """
        )
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "valid_plugin.py")
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(plugin_code)

            plugin = load_method_plugin(path)

            self.assertEqual(plugin.status, "ready")
            self.assertEqual(plugin.name, "Example Loader Plugin")
            self.assertEqual(plugin.version, "1.0.0")
            self.assertIsNotNone(plugin.module)
            self.assertIsNone(plugin.diagnostics)

    def test_run_method_calibration_returns_warning_for_missing_header(self) -> None:
        plugin_code = textwrap.dedent(
            """
            def get_info():
                return {
                    "name": "Headerless Calibration Plugin",
                    "version": "1.0.0",
                    "type": "method",
                    "supports_calibrate": True,
                    "supports_load_params": True,
                    "supports_save_params": True,
                    "supports_process": True,
                    "input_schema": {"mag_x": "float"},
                    "output_schema": {"heading": "float"},
                }

            def calibrate(dataset, config=None):
                return {"offset": [0.0, 0.0, 0.0]}

            def load_params(path):
                return {}

            def save_params(path, params):
                return None

            def process(sample, params):
                return sample
            """
        )
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "headerless_plugin.py")
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(plugin_code)

            plugin = load_method_plugin(path)
            dataset = Dataset(
                "session",
                records=[
                    SampleRecord(
                        stream_id="raw_magnetometer",
                        stream_type="raw",
                        producer_name="Raw Magnetometer",
                        producer_version="builtin",
                        timestamp_mcu=1000,
                        timestamp_pc_rx=1005,
                        timestamp_pc_est=1004,
                        mag_x=1.0,
                        mag_y=0.0,
                        mag_z=0.0,
                        heading=90.0,
                    )
                ],
            )

            result = run_method_calibration(plugin, dataset)

            self.assertTrue(result.ok)
            self.assertIsNone(result.diagnostics)
            self.assertTrue(any("missing header keys" in warning for warning in result.warnings))

    def test_run_method_calibration_captures_runtime_exception(self) -> None:
        plugin_code = textwrap.dedent(
            """
            def get_info():
                return {
                    "name": "Exploding Calibration Plugin",
                    "version": "1.0.0",
                    "type": "method",
                    "supports_calibrate": True,
                    "supports_load_params": True,
                    "supports_save_params": True,
                    "supports_process": True,
                    "input_schema": {"mag_x": "float"},
                    "output_schema": {"heading": "float"},
                }

            def calibrate(dataset, config=None):
                raise RuntimeError("boom")

            def load_params(path):
                return {}

            def save_params(path, params):
                return None

            def process(sample, params):
                return sample
            """
        )
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "exploding_plugin.py")
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(plugin_code)

            plugin = load_method_plugin(path)
            dataset = Dataset("session")

            result = run_method_calibration(plugin, dataset)

            self.assertFalse(result.ok)
            self.assertIsNotNone(result.diagnostics)
            assert result.diagnostics is not None
            self.assertEqual(result.diagnostics.last_action, "calibrate")
            self.assertIn("boom", result.diagnostics.error_text)

    def test_invalid_plugin_returns_error_with_diagnostics(self) -> None:
        plugin_code = textwrap.dedent(
            """
            def get_info():
                return {
                    "name": "Broken Plugin",
                    "version": "0.1.0",
                    "type": "method",
                    "supports_calibrate": True,
                    "supports_load_params": True,
                    "supports_save_params": True,
                    "supports_process": True,
                    "input_schema": {},
                    "output_schema": {},
                }

            def calibrate(dataset, config=None):
                return {}

            def load_params(path):
                return {}

            def save_params(path, params):
                return None
            """
        )
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "invalid_plugin.py")
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(plugin_code)

            plugin = load_method_plugin(path)

            self.assertEqual(plugin.status, "error")
            self.assertIsNotNone(plugin.diagnostics)
            assert plugin.diagnostics is not None
            self.assertEqual(plugin.diagnostics.last_action, "validate_api")
            self.assertIn("process", plugin.diagnostics.error_text)

    def test_save_and_load_method_params_round_trip(self) -> None:
        plugin = load_method_plugin("plugins/examples/magnetometer_identity_method.py")

        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "identity_params.json")
            params = {
                "algorithm_name": "Identity Magnetometer Method",
                "algorithm_version": "1.0.0",
                "schema_version": "1",
                "created_at": "2026-04-05T12:00:00Z",
                "params": {"offset": [1.0, 2.0, 3.0]},
            }

            save_result = save_method_params(plugin, path, params)
            load_result = load_method_params(plugin, path)

            self.assertTrue(save_result.ok)
            self.assertEqual(save_result.warnings, [])
            self.assertTrue(load_result.ok)
            self.assertEqual(load_result.warnings, [])
            self.assertEqual(load_result.params["params"]["offset"], [1.0, 2.0, 3.0])

    def test_load_method_params_reports_mismatch_as_warning(self) -> None:
        plugin = load_method_plugin("plugins/examples/magnetometer_identity_method.py")

        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "mismatch_params.json")
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(
                    textwrap.dedent(
                        """
                        {
                          "algorithm_name": "Other Method",
                          "algorithm_version": "9.9.9",
                          "schema_version": "1",
                          "created_at": "2026-04-05T12:00:00Z",
                          "params": {
                            "offset": [0.0, 0.0, 0.0]
                          }
                        }
                        """
                    ).strip()
                )

            result = load_method_params(plugin, path)

            self.assertTrue(result.ok)
            self.assertEqual(len(result.warnings), 2)
            self.assertTrue(any("algorithm_name mismatch" in warning for warning in result.warnings))
            self.assertTrue(any("algorithm_version mismatch" in warning for warning in result.warnings))

    def test_load_method_params_rejects_missing_header(self) -> None:
        plugin = load_method_plugin("plugins/examples/magnetometer_identity_method.py")

        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "broken_params.json")
            with open(path, "w", encoding="utf-8") as fh:
                fh.write('{"params": {"offset": [0.0, 0.0, 0.0]}}')

            result = load_method_params(plugin, path)

            self.assertFalse(result.ok)
            self.assertIsNotNone(result.diagnostics)
            assert result.diagnostics is not None
            self.assertEqual(result.diagnostics.last_action, "validate_params_header")
            self.assertIn("missing required header keys", result.diagnostics.error_text)

    def test_run_method_process_rejects_non_dict_output(self) -> None:
        plugin_code = textwrap.dedent(
            """
            def get_info():
                return {
                    "name": "Bad Process Plugin",
                    "version": "1.0.0",
                    "type": "method",
                    "supports_calibrate": True,
                    "supports_load_params": True,
                    "supports_save_params": True,
                    "supports_process": True,
                    "input_schema": {"mag_x": "float"},
                    "output_schema": {"mag_x": "float"},
                }

            def calibrate(dataset, config=None):
                return {"algorithm_name": "Bad Process Plugin", "algorithm_version": "1.0.0", "schema_version": "1", "created_at": "2026-04-06T12:00:00Z", "params": {}}

            def load_params(path):
                return {}

            def save_params(path, params):
                return None

            def process(sample, params):
                return 123
            """
        )
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "bad_process.py")
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(plugin_code)

            plugin = load_method_plugin(path)
            result = run_method_process(plugin, {"mag_x": 1.0}, {"params": {}})

            self.assertFalse(result.ok)
            self.assertIsNotNone(result.diagnostics)
            assert result.diagnostics is not None
            self.assertEqual(result.diagnostics.last_action, "validate_process_output")

    def test_external_half_scale_method_loads_and_scales_signal(self) -> None:
        plugin = load_method_plugin("externModules/magnetometer/half_scale_method.py")

        self.assertEqual(plugin.status, "ready")
        result = run_method_process(
            plugin,
            {
                "timestamp_mcu": 100,
                "timestamp_pc_rx": 110,
                "timestamp_pc_est": 108,
                "mag_x": 4.0,
                "mag_y": 2.0,
                "mag_z": -6.0,
                "heading": 63.4,
            },
            {
                "algorithm_name": "Half Scale Magnetometer Method",
                "algorithm_version": "1.0.0",
                "schema_version": "1",
                "created_at": "2026-04-07T10:00:00Z",
                "params": {"scale": 0.5},
            },
        )

        self.assertTrue(result.ok)
        assert result.output is not None
        self.assertEqual(result.output["mag_x"], 2.0)
        self.assertEqual(result.output["mag_y"], 1.0)
        self.assertEqual(result.output["mag_z"], -3.0)

    def test_external_analytic_geometry_method_calibrates_synthetic_ellipse(self) -> None:
        plugin = load_method_plugin("externModules/magnetometer/analyticGeometryTransform2D.py")
        self.assertEqual(plugin.status, "ready")

        center_x = 12.0
        center_y = -7.0
        major_radius = 8.0
        minor_radius = 4.0
        beta_deg = 30.0
        beta_rad = math.radians(beta_deg)

        records: list[SampleRecord] = []
        for idx in range(120):
            angle = 2.0 * math.pi * idx / 120.0
            x_axis = major_radius * math.cos(angle)
            y_axis = minor_radius * math.sin(angle)
            raw_x = center_x + math.cos(beta_rad) * x_axis - math.sin(beta_rad) * y_axis
            raw_y = center_y + math.sin(beta_rad) * x_axis + math.cos(beta_rad) * y_axis
            records.append(
                SampleRecord(
                    stream_id="raw_magnetometer",
                    stream_type="raw",
                    producer_name="Raw Magnetometer",
                    producer_version="builtin",
                    timestamp_mcu=1000 + idx,
                    timestamp_pc_rx=2000 + idx,
                    timestamp_pc_est=1500 + idx,
                    mag_x=raw_x,
                    mag_y=raw_y,
                    mag_z=0.0,
                    heading=None,
                    flags="",
                )
            )

        dataset = Dataset("synthetic_ellipse", records=records)
        calibration = run_method_calibration(plugin, dataset)

        self.assertTrue(calibration.ok)
        assert calibration.params is not None
        params = calibration.params["params"]
        self.assertAlmostEqual(params["offset_x"], center_x, places=6)
        self.assertAlmostEqual(params["offset_y"], center_y, places=6)
        self.assertAlmostEqual(params["x_scale"], 1.0, places=6)
        self.assertAlmostEqual(params["y_scale"], major_radius / minor_radius, places=6)

        corrected_radii: list[float] = []
        for record in records[:12]:
            result = run_method_process(
                plugin,
                {
                    "timestamp_mcu": record.timestamp_mcu,
                    "timestamp_pc_rx": record.timestamp_pc_rx,
                    "timestamp_pc_est": record.timestamp_pc_est,
                    "mag_x": record.mag_x,
                    "mag_y": record.mag_y,
                    "mag_z": record.mag_z,
                    "heading": record.heading,
                    "flags": record.flags,
                },
                calibration.params,
            )
            self.assertTrue(result.ok)
            assert result.output is not None
            corrected_radii.append(math.hypot(result.output["mag_x"], result.output["mag_y"]))

        for radius in corrected_radii:
            self.assertAlmostEqual(radius, major_radius, places=5)

    def test_external_improved_affine_method_calibrates_tilted_near_circular_ellipse(self) -> None:
        plugin = load_method_plugin("externModules/magnetometer/improvedAffineMethod.py")
        self.assertEqual(plugin.status, "ready")
        self.assertEqual(plugin.version, "1.3.0")

        center = (5.0, -3.0, 2.5)
        normal = (0.35, -0.45, 0.82)
        normal_len = math.sqrt(sum(value * value for value in normal))
        normal = tuple(value / normal_len for value in normal)
        ref = (1.0, 0.0, 0.0) if abs(normal[0]) < 0.9 else (0.0, 1.0, 0.0)
        axis_u = (
            normal[1] * ref[2] - normal[2] * ref[1],
            normal[2] * ref[0] - normal[0] * ref[2],
            normal[0] * ref[1] - normal[1] * ref[0],
        )
        axis_u_len = math.sqrt(sum(value * value for value in axis_u))
        axis_u = tuple(value / axis_u_len for value in axis_u)
        axis_v = (
            normal[1] * axis_u[2] - normal[2] * axis_u[1],
            normal[2] * axis_u[0] - normal[0] * axis_u[2],
            normal[0] * axis_u[1] - normal[1] * axis_u[0],
        )

        major_radius = 8.0
        minor_radius = 7.4
        theta = math.radians(37.0)

        records: list[SampleRecord] = []
        for idx in range(180):
            angle = 2.0 * math.pi * idx / 180.0
            ellipse_major = major_radius * math.cos(angle)
            ellipse_minor = minor_radius * math.sin(angle)
            plane_u = math.cos(theta) * ellipse_major - math.sin(theta) * ellipse_minor
            plane_v = math.sin(theta) * ellipse_major + math.cos(theta) * ellipse_minor
            point = (
                center[0] + axis_u[0] * plane_u + axis_v[0] * plane_v,
                center[1] + axis_u[1] * plane_u + axis_v[1] * plane_v,
                center[2] + axis_u[2] * plane_u + axis_v[2] * plane_v,
            )
            records.append(
                SampleRecord(
                    stream_id="raw_magnetometer",
                    stream_type="raw",
                    producer_name="Raw Magnetometer",
                    producer_version="builtin",
                    timestamp_mcu=2000 + idx,
                    timestamp_pc_rx=3000 + idx,
                    timestamp_pc_est=2500 + idx,
                    mag_x=point[0],
                    mag_y=point[1],
                    mag_z=point[2],
                    heading=None,
                    flags="",
                )
            )

        dataset = Dataset("tilted_ellipse", records=records)
        calibration = run_method_calibration(plugin, dataset)

        self.assertTrue(calibration.ok)
        assert calibration.params is not None
        self.assertIn("Raw major axis:", calibration.report)
        self.assertIn("Raw minor axis:", calibration.report)
        self.assertIn("Raw axis delta:", calibration.report)
        self.assertIn("Corrected major axis:", calibration.report)
        self.assertIn("Corrected minor axis:", calibration.report)
        self.assertIn("Corrected axis delta:", calibration.report)
        self.assertIn("Used angular sectors:", calibration.report)
        self.assertIn("Phase weighting: elliptic phase atan2(y'/b, x'/a)", calibration.report)
        self.assertIn("Stage projected_centered:", calibration.report)
        self.assertIn("Stage rotated_axes:", calibration.report)
        self.assertIn("Stage scaled_circle:", calibration.report)
        self.assertIn("Stage horizontal_xy:", calibration.report)
        self.assertIn("fit_center_x=", calibration.report)
        self.assertIn("robust_radius_span=", calibration.report)
        self.assertIn("robust_radius_mad=", calibration.report)

        report_values: dict[str, float] = {}
        for line in calibration.report.splitlines():
            if ":" not in line:
                continue
            key, value = line.split(":", 1)
            try:
                report_values[key.strip()] = float(value.strip())
            except ValueError:
                continue

        params = calibration.params["params"]
        self.assertAlmostEqual(params["offset_x"], center[0], places=5)
        self.assertAlmostEqual(params["offset_y"], center[1], places=5)
        self.assertAlmostEqual(params["offset_z"], center[2], places=5)
        self.assertAlmostEqual(params["major_radius"], major_radius, places=5)
        self.assertAlmostEqual(params["minor_radius"], minor_radius, places=5)
        self.assertAlmostEqual(params["axis_ratio"], major_radius / minor_radius, places=5)
        self.assertGreater(report_values["Used angular sectors"], 0.0)
        self.assertLess(report_values["Corrected axis delta"], report_values["Raw axis delta"])

        corrected_norms: list[float] = []
        corrected_xy_radii: list[float] = []
        corrected_z_values: list[float] = []
        for record in records[:24]:
            result = run_method_process(
                plugin,
                {
                    "timestamp_mcu": record.timestamp_mcu,
                    "timestamp_pc_rx": record.timestamp_pc_rx,
                    "timestamp_pc_est": record.timestamp_pc_est,
                    "mag_x": record.mag_x,
                    "mag_y": record.mag_y,
                    "mag_z": record.mag_z,
                    "heading": record.heading,
                    "flags": record.flags,
                },
                calibration.params,
            )
            self.assertTrue(result.ok)
            assert result.output is not None
            corrected_xy_radii.append(math.hypot(result.output["mag_x"], result.output["mag_y"]))
            corrected_z_values.append(abs(result.output["mag_z"]))
            corrected_norms.append(
                math.sqrt(
                    result.output["mag_x"] * result.output["mag_x"]
                    + result.output["mag_y"] * result.output["mag_y"]
                    + result.output["mag_z"] * result.output["mag_z"]
                )
            )

        for radius in corrected_norms:
            self.assertAlmostEqual(radius, major_radius, places=4)
        for radius in corrected_xy_radii:
            self.assertAlmostEqual(radius, major_radius, places=4)
        self.assertLess(max(corrected_z_values), 1e-5)


if __name__ == "__main__":
    unittest.main()
