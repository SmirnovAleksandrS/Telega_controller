from __future__ import annotations

import math
import os
import tempfile
import textwrap
import unittest

from app.accelerometer_tab import AccelerometerTab
from app.accelerometer_dataset import AccelerometerDataset, AccelerometerSampleRecord
from app.accelerometer_plugin_loader import (
    load_method_plugin as load_accelerometer_plugin,
    run_method_calibration as run_accelerometer_calibration,
    run_method_process as run_accelerometer_process,
)
from app.dialogs import build_method_info_payload
from app.gui_app import VirtualControllerApp
from app.gyroscope_dataset import GyroscopeDataset, GyroscopeSampleRecord
from app.gyroscope_plugin_loader import (
    load_method_plugin as load_gyroscope_plugin,
    run_method_calibration as run_gyroscope_calibration,
    run_method_process as run_gyroscope_process,
)
from app.imu_dataset import ImuDataset, ImuSampleRecord
from comm.protocol import ImuData


class _DummySensorTab:
    def __init__(self) -> None:
        self.live_updates: list[dict[str, object]] = []
        self.rows: list[tuple[int, object]] = []
        self.summary: dict[str, str] | None = None
        self.recording_state: tuple[bool, bool, bool, bool] | None = None
        self.clear_dataset_enabled: bool | None = None
        self.dataset_choices: tuple[list[str], int | None] | None = None
        self.source_states: tuple[dict[str, dict[str, object]], str] | None = None
        self.method_states: tuple[dict[str, dict[str, object]], str | None] | None = None
        self.selected_method_details: dict[str, str] | None = None
        self.selected_method_actions: dict[str, bool] | None = None
        self.heading_routing: dict[str, object] | None = None
        self.primary_output_display: dict[str, object] | None = None
        self.method_dataset_clouds: dict[str, list[object]] | None = None
        self.selected_indices: list[int] = []
        self.view_state: dict[str, object] = {"view_options": {}, "auto_fit": True}

    def update_live_imu(self, **kwargs: object) -> None:
        self.live_updates.append(kwargs)

    def update_dataset_summary(self, **kwargs: str) -> None:
        self.summary = dict(kwargs)

    def set_dataset_choices(self, labels: list[str], active_index: int | None) -> None:
        self.dataset_choices = (list(labels), active_index)

    def set_recording_state(
        self,
        *,
        is_recording: bool,
        can_save: bool,
        has_dataset: bool = False,
        can_concatenate: bool = False,
    ) -> None:
        self.recording_state = (is_recording, can_save, has_dataset, can_concatenate)

    def clear_dataset_rows(self) -> None:
        self.rows.clear()

    def set_dataset_records(self, records: list[object]) -> None:
        self.clear_dataset_rows()
        for row_id, record in enumerate(records, start=1):
            self.rows.append((row_id, record))

    def append_dataset_record(self, row_id: int, record: object) -> None:
        self.rows.append((row_id, record))

    def get_selected_dataset_row_indices(self) -> list[int]:
        return list(self.selected_indices)

    def set_source_states(self, source_states: dict[str, dict[str, object]], selected_source_id: str) -> None:
        self.source_states = ({source_id: dict(state) for source_id, state in source_states.items()}, selected_source_id)

    def set_method_states(self, method_states: dict[str, dict[str, object]], selected_method_id: str | None) -> None:
        self.method_states = ({method_id: dict(state) for method_id, state in method_states.items()}, selected_method_id)

    def update_selected_method_details(self, *, name: str, version: str, path: str, status: str, capabilities: str) -> None:
        self.selected_method_details = {
            "name": name,
            "version": version,
            "path": path,
            "status": status,
            "capabilities": capabilities,
        }

    def set_selected_method_actions(
        self,
        *,
        can_calibrate: bool,
        can_load_params: bool,
        can_save_params: bool,
        can_clear_params: bool,
        can_enable_realtime: bool,
        can_disable_realtime: bool,
    ) -> None:
        self.selected_method_actions = {
            "can_calibrate": can_calibrate,
            "can_load_params": can_load_params,
            "can_save_params": can_save_params,
            "can_clear_params": can_clear_params,
            "can_enable_realtime": can_enable_realtime,
            "can_disable_realtime": can_disable_realtime,
        }

    def set_clear_dataset_enabled(self, enabled: bool) -> None:
        self.clear_dataset_enabled = enabled

    def set_heading_routing(self, *, stream_choices: list[tuple[str, str]], primary_stream_id: str) -> None:
        self.heading_routing = {
            "stream_choices": list(stream_choices),
            "primary_stream_id": primary_stream_id,
        }

    def set_primary_output_display(self, **kwargs: object) -> None:
        self.primary_output_display = dict(kwargs)

    def set_method_dataset_clouds(self, method_clouds: dict[str, list[object]]) -> None:
        self.method_dataset_clouds = {method_id: list(records) for method_id, records in method_clouds.items()}

    def set_metrics_report(self, **_kwargs: object) -> None:
        return None

    def get_view_state(self) -> dict[str, object]:
        return dict(self.view_state)

    def apply_view_state(self, state: dict[str, object]) -> None:
        self.view_state = dict(state)


def _fibonacci_vectors(count: int) -> list[tuple[float, float, float]]:
    vectors: list[tuple[float, float, float]] = []
    golden_angle = 3.141592653589793 * (3.0 - 5.0 ** 0.5)
    for index in range(count):
        y = 1.0 - (2.0 * index) / float(max(1, count - 1))
        radius = max(0.0, 1.0 - y * y) ** 0.5
        theta = golden_angle * index
        vectors.append((math.cos(theta) * radius, y, math.sin(theta) * radius))
    return vectors


def _normalize(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    magnitude = math.sqrt(sum(component * component for component in vector))
    if magnitude <= 1e-9:
        return (0.0, 0.0, 0.0)
    return tuple(component / magnitude for component in vector)


def _cross(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _lerp_normalized(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    ratio: float,
) -> tuple[float, float, float]:
    return _normalize(
        (
            start[0] * (1.0 - ratio) + end[0] * ratio,
            start[1] * (1.0 - ratio) + end[1] * ratio,
            start[2] * (1.0 - ratio) + end[2] * ratio,
        )
    )


def _make_static_accelerometer_dataset() -> tuple[AccelerometerDataset, tuple[float, float, float], tuple[float, float, float]]:
    dataset = AccelerometerDataset("acc_synth")
    bias = (0.08, -0.04, 0.06)
    gains = (1.08, 0.93, 1.05)
    timestamp = 0
    for vector in _fibonacci_vectors(24):
        raw = (
            vector[0] / gains[0] + bias[0],
            vector[1] / gains[1] + bias[1],
            vector[2] / gains[2] + bias[2],
        )
        for _ in range(30):
            dataset.append(
                AccelerometerSampleRecord(
                    "raw_accelerometer",
                    "raw",
                    "Raw Accelerometer",
                    "builtin",
                    timestamp,
                    timestamp + 2,
                    timestamp + 1,
                    raw[0],
                    raw[1],
                    raw[2],
                    None,
                    None,
                    "",
                )
            )
            timestamp += 20
    return (dataset, bias, gains)


def _make_procedure3_gyro_fixture() -> tuple[GyroscopeDataset, list[dict[str, object]], tuple[float, float, float], tuple[float, float, float]]:
    dataset = GyroscopeDataset("gyro_synth")
    acc_records: list[dict[str, object]] = []
    bias = (0.03, -0.02, 0.01)
    gains = (1.04, 0.97, 1.02)
    dt_ms = 20
    timestamp = 0
    static_vectors = _fibonacci_vectors(24)

    def append_sample(
        gravity: tuple[float, float, float],
        true_omega: tuple[float, float, float],
    ) -> None:
        nonlocal timestamp
        raw_gyro = (
            true_omega[0] / gains[0] + bias[0],
            true_omega[1] / gains[1] + bias[1],
            true_omega[2] / gains[2] + bias[2],
        )
        acc_records.append(
            {
                "timestamp_mcu": timestamp,
                "timestamp_pc_rx": timestamp + 2,
                "timestamp_pc_est": timestamp + 1,
                "acc_x": gravity[0],
                "acc_y": gravity[1],
                "acc_z": gravity[2],
                "roll_deg": None,
                "pitch_deg": None,
                "flags": "derived_acc",
            }
        )
        dataset.append(
            GyroscopeSampleRecord(
                "raw_gyroscope",
                "raw",
                "Raw Gyroscope",
                "builtin",
                timestamp,
                timestamp + 2,
                timestamp + 1,
                raw_gyro[0],
                raw_gyro[1],
                raw_gyro[2],
                None,
                "",
            )
        )
        timestamp += dt_ms

    for index, gravity in enumerate(static_vectors):
        static_count = 1100 if index == 0 else 30
        for _ in range(static_count):
            append_sample(gravity, (0.0, 0.0, 0.0))
        if index == len(static_vectors) - 1:
            continue
        next_gravity = static_vectors[index + 1]
        cross = _cross(gravity, next_gravity)
        axis = _normalize(cross if math.sqrt(_dot(cross, cross)) > 1e-9 else (0.0, 0.0, 1.0))
        angle = math.acos(max(-1.0, min(1.0, _dot(_normalize(gravity), _normalize(next_gravity)))))
        duration_s = (25 * dt_ms) / 1000.0
        omega = (
            axis[0] * angle / duration_s,
            axis[1] * angle / duration_s,
            axis[2] * angle / duration_s,
        )
        for step in range(25):
            ratio = (step + 1) / 25.0
            append_sample(_lerp_normalized(gravity, next_gravity, ratio), omega)
    return (dataset, acc_records, bias, gains)


class SensorDatasetTests(unittest.TestCase):
    def test_accelerometer_dataset_csv_round_trip_keeps_metadata(self) -> None:
        dataset = AccelerometerDataset("acc", metadata={"sources": [{"source_id": "raw_accelerometer"}]})
        dataset.append(
            AccelerometerSampleRecord(
                "raw_accelerometer",
                "raw",
                "Raw Accelerometer",
                "builtin",
                1000,
                1010,
                1005,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                "",
            )
        )

        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "acc.csv")
            dataset.to_csv(path)
            reopened = AccelerometerDataset.from_csv(path)

        self.assertEqual(len(reopened.records), 1)
        self.assertEqual(reopened.records[0].timestamp_pc_est, 1005)
        self.assertAlmostEqual(reopened.records[0].pitch_deg or 0.0, 0.0)
        self.assertEqual(reopened.metadata["sources"][0]["source_id"], "raw_accelerometer")

    def test_gyroscope_dataset_trim_delete_and_concatenate(self) -> None:
        dataset = GyroscopeDataset("gyro")
        dataset.extend(
            [
                GyroscopeSampleRecord("a", "raw", "Raw Gyroscope", "builtin", 10, 20, 21, 1.0, 0.0, 0.0, 1.0, ""),
                GyroscopeSampleRecord("b", "raw", "Raw Gyroscope", "builtin", 11, 21, 22, 2.0, 0.0, 0.0, 2.0, ""),
                GyroscopeSampleRecord("c", "raw", "Raw Gyroscope", "builtin", 12, 22, 23, 3.0, 0.0, 0.0, 3.0, ""),
            ]
        )
        dataset.trim(0, 1)
        dataset.delete_rows([0])

        other = GyroscopeDataset("other")
        other.append(GyroscopeSampleRecord("d", "derived", "Method", "1.0", 20, 30, 31, 0.0, 1.0, 0.0, 1.0, ""))
        merged = dataset.concatenate(other, name="merged")

        self.assertEqual([record.stream_id for record in dataset.records], ["b"])
        self.assertEqual(merged.name, "merged")
        self.assertEqual([record.stream_id for record in merged.records], ["b", "d"])


class SensorPluginLoaderTests(unittest.TestCase):
    def test_accelerometer_plugin_loader_infers_accel_vector_stream(self) -> None:
        plugin_code = textwrap.dedent(
            """
            def get_info():
                return {
                    "name": "Accel Identity",
                    "version": "1.0.0",
                    "type": "method",
                    "supports_calibrate": True,
                    "supports_load_params": True,
                    "supports_save_params": True,
                    "supports_process": True,
                    "input_schema": {"acc_x": "float", "acc_y": "float", "acc_z": "float"},
                    "output_schema": {"acc_x": "float", "acc_y": "float", "acc_z": "float", "roll_deg": "float", "pitch_deg": "float"},
                    "stream_requirements": {
                        "calibrate": [{"slot": "acc_input", "kind": "imu.accel_vector"}],
                        "process": [{"slot": "acc_input", "kind": "imu.accel_vector", "scope": "process"}],
                    },
                }

            def calibrate(dataset, config=None):
                return {"algorithm_name": "Accel Identity", "algorithm_version": "1.0.0", "schema_version": "1", "created_at": "2026-04-15T00:00:00Z", "params": {}}

            def load_params(path):
                return {}

            def save_params(path, params):
                return None

            def process(sample, params):
                return sample
            """
        )
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "acc_plugin.py")
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(plugin_code)
            plugin = load_accelerometer_plugin(path)

        self.assertEqual(plugin.status, "ready")
        self.assertEqual(plugin.stream_requirements["calibrate"][0].kind, "imu.accel_vector")

    def test_gyroscope_plugin_loader_infers_gyro_vector_stream(self) -> None:
        plugin_code = textwrap.dedent(
            """
            def get_info():
                return {
                    "name": "Gyro Identity",
                    "version": "1.0.0",
                    "type": "method",
                    "supports_calibrate": True,
                    "supports_load_params": True,
                    "supports_save_params": True,
                    "supports_process": True,
                    "input_schema": {"gyro_x": "float", "gyro_y": "float", "gyro_z": "float"},
                    "output_schema": {"gyro_x": "float", "gyro_y": "float", "gyro_z": "float", "rate_mag": "float"},
                    "stream_requirements": {
                        "calibrate": [{"slot": "gyro_input", "kind": "imu.gyro_vector"}],
                        "process": [{"slot": "gyro_input", "kind": "imu.gyro_vector", "scope": "process"}],
                    },
                }

            def calibrate(dataset, config=None):
                return {"algorithm_name": "Gyro Identity", "algorithm_version": "1.0.0", "schema_version": "1", "created_at": "2026-04-15T00:00:00Z", "params": {}}

            def load_params(path):
                return {}

            def save_params(path, params):
                return None

            def process(sample, params):
                return sample
            """
        )
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "gyro_plugin.py")
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(plugin_code)
            plugin = load_gyroscope_plugin(path)

        self.assertEqual(plugin.status, "ready")
        self.assertEqual(plugin.stream_requirements["calibrate"][0].kind, "imu.gyro_vector")


class AccelerometerViewRegressionTests(unittest.TestCase):
    def test_accelerometer_cloud_renderer_uses_acc_axes(self) -> None:
        record = AccelerometerSampleRecord(
            "raw_accelerometer",
            "raw",
            "Raw Accelerometer",
            "builtin",
            1000,
            1010,
            1005,
            1.0,
            -2.0,
            3.0,
            0.0,
            0.0,
            "",
        )
        tab = object.__new__(AccelerometerTab)
        drawn_points: list[tuple[tuple[float, float], str]] = []
        tab._iter_visible_dataset_clouds = lambda: [
            {
                "stream_id": "raw_accelerometer",
                "title": "Raw Accelerometer",
                "records": [record],
                "color": "#d14b4b",
            }
        ]
        tab._canvas_dimensions = lambda: (640.0, 480.0)
        tab._sample_cloud_records = lambda records, *, per_stream_budget: records
        tab._project_scene_point = lambda x_val, y_val, z_val, *, projection=None: (x_val, y_val)
        tab._draw_cloud_point = lambda point, *, color, size=2.5: drawn_points.append((point, color))

        AccelerometerTab._draw_dataset_clouds(tab, "XY")

        self.assertEqual(drawn_points, [((1.0, -2.0), "#d14b4b")])

    def test_accelerometer_reference_radius_uses_raw_accelerometer_card(self) -> None:
        class _BoolVar:
            def __init__(self, value: bool) -> None:
                self._value = value

            def get(self) -> bool:
                return self._value

        class _Card:
            def __init__(self, visible: bool) -> None:
                self.show_var = _BoolVar(visible)

        tab = object.__new__(AccelerometerTab)
        tab._dataset_cloud_max_radius = 0.0
        tab._method_dataset_cloud_max_radius = 0.0
        tab._current_mag_vector = (3.0, 4.0, 12.0)
        tab._source_cards = {"raw_accelerometer": _Card(True)}
        tab._iter_visible_derived_streams = lambda: []

        radius = AccelerometerTab._reference_radius(tab)

        self.assertEqual(radius, 13.0)


class SensorWorkflowTests(unittest.TestCase):
    def _make_headless_app(self) -> VirtualControllerApp:
        app = object.__new__(VirtualControllerApp)
        app.magnetometer_tab = _DummySensorTab()
        app.accelerometer_tab = _DummySensorTab()
        app.gyroscope_tab = _DummySensorTab()
        app._calibration_stream_snapshots = {}
        app._save_settings = lambda: None
        app._show_error = lambda *_args, **_kwargs: None
        app._show_warning = lambda *_args, **_kwargs: None
        app._show_info = lambda *_args, **_kwargs: None
        app._remember_dialog_dir = lambda *_args, **_kwargs: None
        app._remember_sensor_dialog_dir = lambda *_args, **_kwargs: None
        app._estimate_pc_event_ms_from_mcu = lambda timestamp: timestamp + 5
        return app

    def test_accelerometer_live_update_records_raw_sample(self) -> None:
        app = self._make_headless_app()
        app._on_accelerometer_start_record()

        app._update_accelerometer_live_view(
            ImuData(ts_ms=1000, accel=(0.0, 0.0, 1.0), magn=(0.0, 0.0, 0.0), gyro=(0.1, 0.2, 0.3)),
            1010,
        )

        self.assertEqual(len(app._acc_dataset.records), 1)
        self.assertEqual(app._acc_dataset.records[0].stream_id, "raw_accelerometer")

    def test_shared_recording_writes_unified_imu_dataset(self) -> None:
        app = self._make_headless_app()
        imu = ImuData(ts_ms=4000, accel=(0.1, 0.2, 0.97), magn=(0.4, -0.2, 0.6), gyro=(0.3, 0.0, -0.1))

        app._on_accelerometer_start_record()
        app._update_magnetometer_live_view(imu, 4010)
        app._update_accelerometer_live_view(imu, 4010)
        app._update_gyroscope_live_view(imu, 4010)
        app._on_accelerometer_stop_record()

        self.assertIsNotNone(app._imu_dataset)
        self.assertEqual(
            {record.stream_id for record in app._imu_dataset.records},
            {"raw_magnetometer", "raw_heading", "raw_accelerometer", "raw_tilt", "raw_gyroscope"},
        )

    def test_loading_unified_dataset_activates_all_sensor_projections(self) -> None:
        unified = ImuDataset("imu_test")
        unified.extend(
            [
                ImuSampleRecord("raw_accelerometer", "raw", "Raw Accelerometer", "builtin", 100, 101, 100, acc_x=0.0, acc_y=0.0, acc_z=1.0),
                ImuSampleRecord("raw_tilt", "raw", "Raw Tilt", "builtin", 100, 101, 100, acc_x=0.0, acc_y=0.0, acc_z=1.0, roll_deg=0.0, pitch_deg=0.0),
                ImuSampleRecord("raw_gyroscope", "raw", "Raw Gyroscope", "builtin", 100, 101, 100, gyro_x=0.1, gyro_y=0.2, gyro_z=0.3),
                ImuSampleRecord("raw_magnetometer", "raw", "Raw Magnetometer", "builtin", 100, 101, 100, mag_x=1.0, mag_y=0.0, mag_z=0.5),
                ImuSampleRecord("raw_heading", "raw", "Raw Heading", "builtin", 100, 101, 100, mag_x=1.0, mag_y=0.0, mag_z=0.5, heading=12.0),
            ]
        )
        app = self._make_headless_app()
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "imu.csv")
            unified.to_csv(path)
            loaded = app._load_sensor_dataset_paths("acc", (path,))

        self.assertEqual(loaded, 1)
        self.assertIsNotNone(app._imu_dataset)
        self.assertIsNotNone(app._acc_dataset)
        self.assertIsNotNone(app._gyro_dataset)
        self.assertIsNotNone(app._mag_dataset)
        self.assertEqual(len(app._acc_dataset.records), 2)
        self.assertEqual(len(app._gyro_dataset.records), 1)
        self.assertEqual(len(app._mag_dataset.records), 2)

    def test_gyroscope_live_update_records_raw_sample(self) -> None:
        app = self._make_headless_app()
        app._on_gyroscope_start_record()

        app._update_gyroscope_live_view(
            ImuData(ts_ms=2000, accel=(0.0, 0.0, 1.0), magn=(0.0, 0.0, 0.0), gyro=(1.0, 2.0, 2.0)),
            2010,
        )

        self.assertEqual(len(app._gyro_dataset.records), 1)
        self.assertEqual(app._gyro_dataset.records[0].stream_id, "raw_gyroscope")

    def test_accelerometer_save_and_reload_dataset(self) -> None:
        app = self._make_headless_app()
        app._on_accelerometer_start_record()
        app._update_accelerometer_live_view(
            ImuData(ts_ms=3000, accel=(0.0, 1.0, 0.0), magn=(0.0, 0.0, 0.0), gyro=(0.0, 0.0, 0.0)),
            3010,
        )
        app._on_accelerometer_stop_record()

        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "acc.csv")
            self.assertTrue(app._save_sensor_dataset_to_path("acc", path))

            app._on_clear_accelerometer_datasets()
            loaded = app._load_sensor_dataset_paths("acc", (path,))

        self.assertEqual(loaded, 1)
        self.assertIsNotNone(app._acc_dataset)
        self.assertEqual(len(app._acc_dataset.records), 1)
        self.assertEqual(app._acc_dataset.records[0].stream_id, "raw_accelerometer")


class ImuDatasetTests(unittest.TestCase):
    def test_unified_dataset_roundtrip_and_projection(self) -> None:
        dataset = ImuDataset("imu_roundtrip", metadata={"schema_version": "1"})
        dataset.extend(
            [
                ImuSampleRecord("raw_accelerometer", "raw", "Raw Accelerometer", "builtin", 10, 11, 10, acc_x=0.0, acc_y=0.0, acc_z=1.0),
                ImuSampleRecord("raw_tilt", "raw", "Raw Tilt", "builtin", 10, 11, 10, acc_x=0.0, acc_y=0.0, acc_z=1.0, roll_deg=0.0, pitch_deg=0.0),
                ImuSampleRecord("raw_gyroscope", "raw", "Raw Gyroscope", "builtin", 10, 11, 10, gyro_x=1.0, gyro_y=2.0, gyro_z=3.0, rate_mag=3.74),
                ImuSampleRecord("raw_magnetometer", "raw", "Raw Magnetometer", "builtin", 10, 11, 10, mag_x=0.1, mag_y=0.2, mag_z=0.3),
                ImuSampleRecord("raw_heading", "raw", "Raw Heading", "builtin", 10, 11, 10, mag_x=0.1, mag_y=0.2, mag_z=0.3, heading=18.0),
            ]
        )

        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "imu.csv")
            dataset.to_csv(path)
            reopened = ImuDataset.from_csv(path)

        self.assertEqual(len(reopened.records), 5)
        self.assertEqual(len(reopened.project_accelerometer_dataset().records), 2)
        self.assertEqual(len(reopened.project_gyroscope_dataset().records), 1)
        self.assertEqual(len(reopened.project_magnetometer_dataset().records), 2)


class Procedure3PluginTests(unittest.TestCase):
    def test_accelerometer_procedure3_calibration_roundtrip(self) -> None:
        dataset, _bias, _gains = _make_static_accelerometer_dataset()
        plugin = load_accelerometer_plugin("externModules/accelerometer/procedure3QuasiStaticMethod.py")

        result = run_accelerometer_calibration(plugin, dataset, config={})

        self.assertTrue(result.ok)
        self.assertIsInstance(result.params, dict)
        summary = result.params.get("calibration_summary", {})
        self.assertGreaterEqual(summary.get("static_set_count", 0), 24)
        self.assertIn("accelerometer_cost", summary)

        process_result = run_accelerometer_process(
            plugin,
            {
                "timestamp_mcu": 1,
                "timestamp_pc_rx": 2,
                "timestamp_pc_est": 1,
                "acc_x": 0.2,
                "acc_y": -0.1,
                "acc_z": 1.0,
            },
            result.params,
        )
        self.assertTrue(process_result.ok)
        self.assertIn("roll_deg", process_result.output)
        self.assertIn("pitch_deg", process_result.output)

    def test_gyroscope_procedure3_calibration_uses_calibrated_acc_input(self) -> None:
        dataset, acc_records, _bias, _gains = _make_procedure3_gyro_fixture()
        plugin = load_gyroscope_plugin("externModules/gyroscope/procedure3QuasiStaticMethod.py")

        result = run_gyroscope_calibration(
            plugin,
            dataset,
            config={
                "stream_inputs": {
                    "gyro_input": {"records": [dict(record) for record in [
                        {
                            "timestamp_mcu": record.timestamp_mcu,
                            "timestamp_pc_rx": record.timestamp_pc_rx,
                            "timestamp_pc_est": record.timestamp_pc_est,
                            "gyro_x": record.gyro_x,
                            "gyro_y": record.gyro_y,
                            "gyro_z": record.gyro_z,
                            "rate_mag": record.rate_mag,
                            "flags": record.flags,
                        }
                        for record in dataset.records
                    ]]},
                    "acc_input": {"records": list(acc_records)},
                },
                "selected_producers": {
                    "gyro_input": {"origin": "builtin", "kind": "imu.gyro_vector"},
                    "acc_input": {"origin": "method", "kind": "imu.accel_vector", "details": {"source_sensor": "acc"}},
                },
            },
        )

        self.assertTrue(result.ok)
        summary = result.params.get("calibration_summary", {})
        self.assertGreater(summary.get("matched_sample_count", 0), 0)
        self.assertGreaterEqual(summary.get("bias_window_s", 0.0), 2.0)
        self.assertIn("gyroscope_cost", summary)

        process_result = run_gyroscope_process(
            plugin,
            {
                "timestamp_mcu": 1,
                "timestamp_pc_rx": 2,
                "timestamp_pc_est": 1,
                "gyro_x": 0.4,
                "gyro_y": -0.1,
                "gyro_z": 0.2,
            },
            result.params,
        )
        self.assertTrue(process_result.ok)
        self.assertIn("rate_mag", process_result.output)

    def test_gyro_routing_requires_accelerometer_method_output(self) -> None:
        app = SensorWorkflowTests()._make_headless_app()
        acc_dataset, _bias, _gains = _make_static_accelerometer_dataset()
        gyro_dataset = GyroscopeDataset("gyro")
        gyro_dataset.append(
            GyroscopeSampleRecord("raw_gyroscope", "raw", "Raw Gyroscope", "builtin", 10, 11, 10, 0.0, 0.0, 0.0, 0.0, "")
        )
        app._acc_datasets = [acc_dataset]
        app._gyro_datasets = [gyro_dataset]
        app._acc_dataset = acc_dataset
        app._gyro_dataset = gyro_dataset

        acc_plugin = load_accelerometer_plugin("externModules/accelerometer/procedure3QuasiStaticMethod.py")
        acc_plugin.method_id = "method_1"
        acc_plugin.derived_stream_id = "derived_method_1"
        acc_plugin.calibration_params = {
            "algorithm_name": acc_plugin.name,
            "algorithm_version": acc_plugin.version,
            "schema_version": "1",
            "created_at": "2026-04-15T00:00:00Z",
            "params": {},
        }
        gyro_plugin = load_gyroscope_plugin("externModules/gyroscope/procedure3QuasiStaticMethod.py")
        gyro_plugin.method_id = "method_2"
        gyro_plugin.derived_stream_id = "derived_method_2"
        app._acc_methods = {acc_plugin.method_id: acc_plugin}
        app._gyro_methods = {gyro_plugin.method_id: gyro_plugin}
        gyro_plugin.stream_bindings["calibrate"] = {
            "gyro_input": "builtin::raw_gyroscope",
            "acc_input": "builtin::raw_accelerometer",
        }

        app._update_sensor_method_routing_state("gyro")
        validation = gyro_plugin.routing_validation["calibrate"]
        self.assertFalse(validation.ok)
        self.assertTrue(any("calibrated accelerometer method output" in issue for issue in validation.issues))
        acc_candidates = validation.slot_candidates["acc_input"]
        self.assertTrue(any(not candidate["selectable"] for candidate in acc_candidates if candidate["origin"] == "builtin"))

    def test_persisted_warnings_and_summary_appear_in_method_info(self) -> None:
        plugin = load_accelerometer_plugin("externModules/accelerometer/procedure3QuasiStaticMethod.py")
        plugin.calibration_params = {
            "algorithm_name": plugin.name,
            "algorithm_version": plugin.version,
            "schema_version": "1",
            "created_at": "2026-04-15T00:00:00Z",
            "params": {},
            "warnings": ["persisted dataset warning"],
            "calibration_summary": {"static_set_count": 24, "matched_sample_count": 720},
        }

        payload = build_method_info_payload(
            plugin.info,
            file_path=plugin.file_path,
            status_text=plugin.display_status_text(),
            warnings=plugin.effective_warnings(),
            calibration_params=plugin.calibration_params,
        )

        self.assertIn("persisted dataset warning", plugin.effective_warnings())
        self.assertIn("static_set_count", payload["summary_text"])
        self.assertIn("matched_sample_count", payload["summary_text"])


if __name__ == "__main__":
    unittest.main()
