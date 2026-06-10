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

    def test_plugin_loads_optional_stream_requirements(self) -> None:
        plugin_code = textwrap.dedent(
            """
            def get_info():
                return {
                    "name": "Stream Aware Plugin",
                    "version": "1.0.0",
                    "type": "method",
                    "supports_calibrate": True,
                    "supports_load_params": True,
                    "supports_save_params": True,
                    "supports_process": True,
                    "input_schema": {"mag_x": "float", "mag_y": "float", "mag_z": "float"},
                    "output_schema": {"mag_x": "float", "mag_y": "float", "mag_z": "float", "heading": "float"},
                    "stream_requirements": {
                        "calibrate": [
                            {
                                "slot": "mag_input",
                                "kind": "imu.magnetometer_vector",
                                "required": True,
                                "label": "Magnetometer input",
                                "description": "Corrected vector used during calibration"
                            }
                        ],
                        "process": [
                            {
                                "slot": "mag_input",
                                "kind": "imu.magnetometer_vector",
                                "required": True
                            }
                        ]
                    },
                }

            def calibrate(dataset, config=None):
                return {"algorithm_name": "Stream Aware Plugin", "algorithm_version": "1.0.0", "schema_version": "1", "created_at": "2026-04-12T12:00:00Z", "params": {}}

            def load_params(path):
                return {}

            def save_params(path, params):
                return None

            def process(sample, params):
                return sample
            """
        )
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "stream_aware.py")
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(plugin_code)

            plugin = load_method_plugin(path)

            self.assertEqual(plugin.status, "ready")
            self.assertEqual(plugin.stream_requirements["calibrate"][0].slot, "mag_input")
            self.assertEqual(plugin.stream_requirements["calibrate"][0].kind, "imu.magnetometer_vector")
            self.assertEqual(plugin.stream_requirements["process"][0].scope, "process")

    def test_invalid_stream_requirements_reject_plugin(self) -> None:
        plugin_code = textwrap.dedent(
            """
            def get_info():
                return {
                    "name": "Broken Stream Plugin",
                    "version": "1.0.0",
                    "type": "method",
                    "supports_calibrate": True,
                    "supports_load_params": True,
                    "supports_save_params": True,
                    "supports_process": True,
                    "input_schema": {"mag_x": "float"},
                    "output_schema": {"mag_x": "float"},
                    "stream_requirements": {
                        "calibrate": [{"slot": "x", "kind": "unknown.kind"}]
                    },
                }

            def calibrate(dataset, config=None):
                return {}

            def load_params(path):
                return {}

            def save_params(path, params):
                return None

            def process(sample, params):
                return sample
            """
        )
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "broken_stream.py")
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(plugin_code)

            plugin = load_method_plugin(path)

            self.assertEqual(plugin.status, "error")
            assert plugin.diagnostics is not None
            self.assertEqual(plugin.diagnostics.last_action, "validate_stream_requirements")

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

    def test_run_method_calibration_passes_stream_config(self) -> None:
        plugin_code = textwrap.dedent(
            """
            def get_info():
                return {
                    "name": "Config Echo Plugin",
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
                return {
                    "algorithm_name": "Config Echo Plugin",
                    "algorithm_version": "1.0.0",
                    "schema_version": "1",
                    "created_at": "2026-04-12T12:00:00Z",
                    "params": {
                        "stream_bindings": dict(config.get("stream_bindings", {})),
                        "stream_input_slots": sorted(config.get("stream_inputs", {}).keys()),
                    },
                }

            def load_params(path):
                return {}

            def save_params(path, params):
                return None

            def process(sample, params):
                return sample
            """
        )
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "config_echo.py")
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(plugin_code)

            plugin = load_method_plugin(path)
            dataset = Dataset("session")

            result = run_method_calibration(
                plugin,
                dataset,
                config={
                    "stream_bindings": {"mag_input": "builtin::raw_magnetometer"},
                    "stream_inputs": {"mag_input": {"records": []}},
                },
            )

            self.assertTrue(result.ok)
            assert result.params is not None
            self.assertEqual(result.params["params"]["stream_bindings"]["mag_input"], "builtin::raw_magnetometer")
            self.assertEqual(result.params["params"]["stream_input_slots"], ["mag_input"])

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

    def test_external_plane_ellipse_methods_handle_large_coordinate_scale(self) -> None:
        center = (12000.0, -7000.0, 2500.0)
        major_radius = 8000.0
        minor_radius = 4200.0

        records: list[SampleRecord] = []
        for idx in range(180):
            angle = 2.0 * math.pi * idx / 180.0
            records.append(
                SampleRecord(
                    stream_id="raw_magnetometer",
                    stream_type="raw",
                    producer_name="Raw Magnetometer",
                    producer_version="builtin",
                    timestamp_mcu=3000 + idx,
                    timestamp_pc_rx=4000 + idx,
                    timestamp_pc_est=3500 + idx,
                    mag_x=center[0] + major_radius * math.cos(angle),
                    mag_y=center[1] + minor_radius * math.sin(angle),
                    mag_z=center[2],
                    heading=None,
                    flags="",
                )
            )

        dataset = Dataset("large_scale_ellipse", records=records)
        method_paths = (
            "externModules/magnetometer/simpleHardIroning.py",
            "externModules/magnetometer/improvedAffineMethod.py",
        )

        for method_path in method_paths:
            with self.subTest(method_path=method_path):
                plugin = load_method_plugin(method_path)
                calibration = run_method_calibration(plugin, dataset)

                diagnostic = calibration.diagnostics.error_text if calibration.diagnostics else ""
                self.assertTrue(calibration.ok, diagnostic)
                assert calibration.params is not None
                params = calibration.params["params"]
                self.assertAlmostEqual(params["offset_x"], center[0], places=5)
                self.assertAlmostEqual(params["offset_y"], center[1], places=5)
                self.assertAlmostEqual(params["offset_z"], center[2], places=5)
                if method_path.endswith("improvedAffineMethod.py"):
                    self.assertAlmostEqual(params["major_radius"], major_radius, places=5)
                    self.assertAlmostEqual(params["minor_radius"], minor_radius, places=5)

    def test_external_ets_2d_method_calibrates_synthetic_ellipse(self) -> None:
        plugin = load_method_plugin("externModules/magnetometer/ets2DMethod.py")
        self.assertEqual(plugin.status, "ready")

        field_radius = 50.0
        offset = (12.0, -7.0)
        a_scale = 1.25
        b_scale = 0.80
        rho = math.radians(20.0)
        l_matrix = (
            (a_scale, 0.0),
            (b_scale * math.sin(rho), b_scale * math.cos(rho)),
        )

        records: list[SampleRecord] = []
        for idx in range(180):
            angle = 2.0 * math.pi * idx / 180.0
            bx = field_radius * math.cos(angle)
            by = field_radius * math.sin(angle)
            raw_x = offset[0] + l_matrix[0][0] * bx + l_matrix[0][1] * by
            raw_y = offset[1] + l_matrix[1][0] * bx + l_matrix[1][1] * by
            records.append(
                SampleRecord(
                    stream_id="raw_magnetometer",
                    stream_type="raw",
                    producer_name="Raw Magnetometer",
                    producer_version="builtin",
                    timestamp_mcu=5000 + idx,
                    timestamp_pc_rx=6000 + idx,
                    timestamp_pc_est=5500 + idx,
                    mag_x=raw_x,
                    mag_y=raw_y,
                    mag_z=3.0,
                    heading=None,
                    flags="",
                )
            )

        dataset = Dataset("ets_2d_ellipse", records=records)
        calibration = run_method_calibration(
            plugin,
            dataset,
            config={"field_radius": field_radius, "robust": "none"},
        )

        self.assertTrue(calibration.ok)
        assert calibration.params is not None
        self.assertIn("ETS rho_deg:", calibration.report)
        params = calibration.params["params"]
        self.assertAlmostEqual(params["offset_x"], offset[0], places=6)
        self.assertAlmostEqual(params["offset_y"], offset[1], places=6)
        self.assertLess(params["rel_rms_residual"], 1e-8)

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
            self.assertAlmostEqual(math.hypot(result.output["mag_x"], result.output["mag_y"]), field_radius, places=6)
            self.assertAlmostEqual(result.output["mag_z"], 3.0, places=6)

    def test_external_ets_3d_method_calibrates_synthetic_ellipsoid(self) -> None:
        plugin = load_method_plugin("externModules/magnetometer/ets3DMethod.py")
        self.assertEqual(plugin.status, "ready")

        field_radius = 50.0
        offset = (12.0, -7.0, 4.5)
        l_matrix = (
            (1.15, 0.08, -0.04),
            (0.02, 0.85, 0.07),
            (-0.03, 0.05, 1.35),
        )
        golden_angle = math.pi * (3.0 - math.sqrt(5.0))

        records: list[SampleRecord] = []
        for idx in range(240):
            z_unit = 1.0 - 2.0 * (idx + 0.5) / 240.0
            xy_radius = math.sqrt(max(0.0, 1.0 - z_unit * z_unit))
            angle = idx * golden_angle
            bx = field_radius * xy_radius * math.cos(angle)
            by = field_radius * xy_radius * math.sin(angle)
            bz = field_radius * z_unit
            raw_x = offset[0] + l_matrix[0][0] * bx + l_matrix[0][1] * by + l_matrix[0][2] * bz
            raw_y = offset[1] + l_matrix[1][0] * bx + l_matrix[1][1] * by + l_matrix[1][2] * bz
            raw_z = offset[2] + l_matrix[2][0] * bx + l_matrix[2][1] * by + l_matrix[2][2] * bz
            records.append(
                SampleRecord(
                    stream_id="raw_magnetometer",
                    stream_type="raw",
                    producer_name="Raw Magnetometer",
                    producer_version="builtin",
                    timestamp_mcu=7000 + idx,
                    timestamp_pc_rx=8000 + idx,
                    timestamp_pc_est=7500 + idx,
                    mag_x=raw_x,
                    mag_y=raw_y,
                    mag_z=raw_z,
                    heading=None,
                    flags="",
                )
            )

        dataset = Dataset("ets_3d_ellipsoid", records=records)
        calibration = run_method_calibration(
            plugin,
            dataset,
            config={"field_radius": field_radius, "robust": "none"},
        )

        self.assertTrue(calibration.ok)
        assert calibration.params is not None
        self.assertIn("3D coverage ratio:", calibration.report)
        params = calibration.params["params"]
        self.assertAlmostEqual(params["offset_x"], offset[0], places=5)
        self.assertAlmostEqual(params["offset_y"], offset[1], places=5)
        self.assertAlmostEqual(params["offset_z"], offset[2], places=5)
        self.assertLess(params["rel_rms_residual"], 1e-8)

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
            corrected_radius = math.sqrt(
                result.output["mag_x"] * result.output["mag_x"]
                + result.output["mag_y"] * result.output["mag_y"]
                + result.output["mag_z"] * result.output["mag_z"]
            )
            self.assertAlmostEqual(corrected_radius, field_radius, places=5)

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

    def test_external_heading_hill_method_calibrates_synthetic_hill_passes(self) -> None:
        plugin = load_method_plugin("externModules/magnetometer/headingHillMethod.py")
        self.assertEqual(plugin.status, "ready")

        center = (12.0, -7.0)
        scale = 80.0
        k_ratio = 0.35
        eta = (0.18, -0.11)
        gamma_forward = math.radians(35.0)
        gamma_backward = gamma_forward + math.pi

        records: list[SampleRecord] = []
        accel_records: list[dict[str, object]] = []

        def add_sample(idx: int, r_x: float, r_y: float, pitch_deg: float, flags: str) -> None:
            mag_x = center[0] + scale * r_x
            mag_y = center[1] + scale * r_y
            timestamp = 1000 + idx
            records.append(
                SampleRecord(
                    stream_id="raw_magnetometer",
                    stream_type="raw",
                    producer_name="Raw Magnetometer",
                    producer_version="builtin",
                    timestamp_mcu=timestamp,
                    timestamp_pc_rx=timestamp + 5,
                    timestamp_pc_est=timestamp + 3,
                    mag_x=mag_x,
                    mag_y=mag_y,
                    mag_z=0.0,
                    heading=None,
                    flags=flags,
                )
            )
            accel_records.append(
                {
                    "stream_id": "raw_tilt",
                    "stream_type": "raw",
                    "timestamp_mcu": timestamp,
                    "timestamp_pc_rx": timestamp + 5,
                    "timestamp_pc_est": timestamp + 3,
                    "acc_x": 0.0,
                    "acc_y": 0.0,
                    "acc_z": 1.0,
                    "pitch_deg": pitch_deg,
                    "roll_deg": 0.0,
                    "flags": flags,
                }
            )

        for idx in range(144):
            angle = 2.0 * math.pi * idx / 144.0
            add_sample(idx, math.cos(angle), math.sin(angle), 0.0, "ring")

        def hill_point(gamma: float, theta: float) -> tuple[float, float]:
            cos_gamma = math.cos(gamma)
            ideal_x = cos_gamma * math.cos(theta) + k_ratio * math.sin(theta)
            ideal_y = math.sin(gamma)
            z_val = -cos_gamma * math.sin(theta) + k_ratio * (math.cos(theta) - 1.0)
            return (ideal_x + eta[0] * z_val, ideal_y + eta[1] * z_val)

        base_idx = 144
        pitch_profile = [0.0, 2.0, 4.0, 7.0, 10.0, 13.0, 16.0, 13.0, 10.0, 7.0, 4.0, 2.0, 0.0]
        for pass_name, gamma in (("hill_forward", gamma_forward), ("hill_backward", gamma_backward)):
            for pitch_deg in pitch_profile:
                theta = math.radians(pitch_deg)
                r_x, r_y = hill_point(gamma, theta)
                add_sample(base_idx, r_x, r_y, pitch_deg, pass_name)
                base_idx += 1

        dataset = Dataset("heading_hill_synthetic", records=records)
        mag_records = [
            {
                "timestamp_mcu": record.timestamp_mcu,
                "timestamp_pc_rx": record.timestamp_pc_rx,
                "timestamp_pc_est": record.timestamp_pc_est,
                "mag_x": record.mag_x,
                "mag_y": record.mag_y,
                "mag_z": record.mag_z,
                "heading": record.heading,
                "flags": record.flags,
            }
            for record in records
        ]
        config = {
            "stream_inputs": {
                "mag_input": {"records": mag_records},
                "accel_input": {"records": accel_records},
            },
            "vertical_horizontal_ratio": k_ratio,
            "robust": "none",
            "max_sync_delta_ms": 10.0,
        }

        calibration = run_method_calibration(plugin, dataset, config=config)

        self.assertTrue(calibration.ok)
        assert calibration.params is not None
        params = calibration.params["params"]
        self.assertAlmostEqual(params["eta_x"], eta[0], places=3)
        self.assertAlmostEqual(params["eta_y"], eta[1], places=3)
        self.assertLess(params["hill_rms_after"], params["hill_rms_before"])
        self.assertGreater(params["hill_improvement_gain"], 0.9)
        self.assertIn("Hill samples:", calibration.report)

        sample_index = 144 + 6
        result = run_method_process(
            plugin,
            {
                **mag_records[sample_index],
                "inputs": {"accel_input": accel_records[sample_index]},
            },
            calibration.params,
        )

        self.assertTrue(result.ok)
        assert result.output is not None
        theta = math.radians(pitch_profile[6])
        expected_x = math.cos(gamma_forward) * math.cos(theta) + k_ratio * math.sin(theta)
        expected_y = math.sin(gamma_forward)
        expected_heading = (math.degrees(math.atan2(expected_x, expected_y)) + 360.0) % 360.0
        self.assertAlmostEqual(result.output["mag_x"] / scale, expected_x, delta=0.002)
        self.assertAlmostEqual(result.output["mag_y"] / scale, expected_y, delta=0.002)
        self.assertAlmostEqual(result.output["heading"], expected_heading, delta=0.1)

    def test_heading_hill_timestamp_pitch_index_matches_linear_lookup(self) -> None:
        from externModules.magnetometer import headingHillMethod as method

        accel_records = [
            {"timestamp_mcu": 3000, "pitch_deg": 30.0},
            {"timestamp_mcu": 1000, "pitch_deg": 10.0},
            {"timestamp_mcu": 2000, "pitch_deg": 20.0},
        ]
        accel_timestamps, accel_pitches, indexed_pitches = method._build_accel_pitch_index(accel_records)
        mag_records = [
            {"timestamp_mcu": 990},
            {"timestamp_mcu": 1510},
            {"timestamp_mcu": 2600},
            {"pitch_deg": -5.0, "timestamp_mcu": 2500},
        ]

        for index, record in enumerate(mag_records):
            legacy = method._aligned_pitch_deg(record, accel_records, index, 750.0)
            indexed = method._aligned_pitch_deg_indexed(
                record,
                index,
                750.0,
                accel_timestamps,
                accel_pitches,
                indexed_pitches,
            )
            self.assertEqual(indexed, legacy)

        self.assertIsNone(
            method._aligned_pitch_deg_indexed(
                {"timestamp_mcu": 5000},
                0,
                100.0,
                accel_timestamps,
                accel_pitches,
                indexed_pitches,
            )
        )


if __name__ == "__main__":
    unittest.main()
