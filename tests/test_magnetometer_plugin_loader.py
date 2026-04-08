from __future__ import annotations

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


if __name__ == "__main__":
    unittest.main()
