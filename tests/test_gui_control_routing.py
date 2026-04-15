from __future__ import annotations

import collections
import os
import queue
import struct
import tempfile
import types
import unittest

from app.calibration_streams import SCOPE_CALIBRATE, infer_stream_requirements, make_method_producer_id
from app.dialogs import format_method_info_text
from app.gui_app import VirtualControllerApp
from app.magnetometer_dataset import Dataset, SampleRecord
from app.magnetometer_plugin_loader import CalibrationRunResult, LoadedMethodPlugin, ParamIoResult, PluginDiagnostics
from app.manual_tab import ManualControlState
from comm.serial_worker import RxEvent
from comm.protocol import (
    Frame,
    ImuData,
    MotorPidData,
    TYPE_C0_CONTROL,
    TYPE_D3_SENSOR_TENSOR,
    build_pid_req,
    build_set_motor_pid,
    parse_frame,
)
from comm.time_sync import TimeModel
from runtime.contracts import AutopilotMode


class _DummyCoordTab:
    def __init__(self) -> None:
        self.running = None
        self.stop_expected_calls = 0

    def get_autopilot_mode(self) -> AutopilotMode:
        return AutopilotMode.BUILTIN_PURE_PURSUIT

    def step_controller(self, _dt_s: float, force_internal_pose: bool = False) -> tuple[int, int]:
        self.force_internal_pose = force_internal_pose
        return (1600, 1400)

    def controller_finished(self) -> bool:
        return False

    def set_running(self, value: bool) -> None:
        self.running = value

    def stop_expected(self) -> None:
        self.stop_expected_calls += 1


class _DummyMagnetometerTab:
    def __init__(self) -> None:
        self.live_updates: list[dict[str, object]] = []
        self.rows: list[tuple[int, object]] = []
        self.summary: dict[str, str] | None = None
        self.recording_state: tuple[bool, bool, bool, bool] | None = None
        self.clear_dataset_enabled: bool | None = None
        self.cleared = 0
        self.selected_indices: list[int] = []
        self.dataset_choices: tuple[list[str], int | None] | None = None
        self.source_states: tuple[dict[str, dict[str, object]], str] | None = None
        self.method_states: tuple[dict[str, dict[str, object]], str | None] | None = None
        self.selected_method_details: dict[str, str] | None = None
        self.selected_method_actions: dict[str, bool] | None = None
        self.heading_routing: dict[str, object] | None = None
        self.primary_output_display: dict[str, object] | None = None
        self.metrics_report: dict[str, object] | None = None
        self.method_dataset_clouds: dict[str, list[object]] | None = None
        self.view_state: dict[str, object] = {
            "projection_mode": "XY",
            "auto_fit": True,
            "view_options": {},
        }

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
        self.cleared += 1
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
        snapshot = {
            source_id: dict(state)
            for source_id, state in source_states.items()
        }
        self.source_states = (snapshot, selected_source_id)

    def set_method_states(self, method_states: dict[str, dict[str, object]], selected_method_id: str | None) -> None:
        snapshot = {
            method_id: dict(state)
            for method_id, state in method_states.items()
        }
        self.method_states = (snapshot, selected_method_id)

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

    def set_primary_output_display(self, *, heading: float | None, source_label: str) -> None:
        self.primary_output_display = {
            "heading": heading,
            "source_label": source_label,
        }

    def set_metrics_report(self, *, rows: list[dict[str, str]], summary_text: str, can_export: bool) -> None:
        self.metrics_report = {
            "rows": list(rows),
            "summary_text": summary_text,
            "can_export": can_export,
        }

    def set_method_dataset_clouds(self, method_clouds: dict[str, list[object]]) -> None:
        self.method_dataset_clouds = {
            method_id: list(records)
            for method_id, records in method_clouds.items()
        }

    def get_view_state(self) -> dict[str, object]:
        return dict(self.view_state)

    def apply_view_state(self, state: dict[str, object]) -> None:
        self.view_state = dict(state)


class _DummyManualTab:
    def __init__(self) -> None:
        self.pid_values: tuple[float, float, float, float, float, float] | None = None

    def set_motor_pid_values(
        self,
        left_p: float,
        left_i: float,
        left_d: float,
        right_p: float,
        right_i: float,
        right_d: float,
    ) -> None:
        self.pid_values = (left_p, left_i, left_d, right_p, right_i, right_d)


class GuiControlRoutingTests(unittest.TestCase):
    def test_format_method_info_text_includes_runtime_report(self) -> None:
        text = format_method_info_text(
            {
                "name": "Improved Affine Magnetometer Method",
                "version": "1.0.0",
                "type": "method",
                "supports_calibrate": True,
                "supports_load_params": True,
                "supports_save_params": True,
                "supports_process": True,
                "input_schema": {"mag_x": "float"},
                "output_schema": {"mag_x": "float"},
            },
            file_path="/tmp/improvedAffineMethod.py",
            status_text="READY",
            warnings=["calibration not completed"],
            calibration_dataset_name="session_1",
            calibration_runtime_s=0.123,
            params_profile_path="/tmp/params.json",
            calibration_params={"params": {"offset_x": 1.0}},
            calibration_report="Raw major axis: 10.0\nCorrected major axis: 9.0",
            stream_requirements={"calibrate": [{"slot": "mag_input", "kind": "imu.magnetometer_vector"}]},
            stream_bindings={"calibrate": {"mag_input": "builtin::raw_magnetometer"}},
            routing_validation={"ok": True, "issues": [], "cycle_paths": []},
        )

        self.assertIn('"status": "READY"', text)
        self.assertIn('"calibration_dataset_name": "session_1"', text)
        self.assertIn('"params_profile_path": "/tmp/params.json"', text)
        self.assertIn('"stream_requirements"', text)
        self.assertIn('"stream_bindings"', text)
        self.assertIn('"routing_validation"', text)
        self.assertIn("Calibration report:", text)
        self.assertIn("Raw major axis: 10.0", text)
        self.assertIn("Corrected major axis: 9.0", text)

    def test_manual_control_state_defaults_to_neutral_pwm(self) -> None:
        state = ManualControlState()
        self.assertEqual((state.left_cmd, state.right_cmd), (1500, 1500))

    def test_finalize_initial_sync_requests_pid_when_pending(self) -> None:
        import app.gui_app as gui_app_module

        app = object.__new__(VirtualControllerApp)
        sent: list[bytes] = []
        app.time_model = TimeModel()
        app._sync_points = []
        app._initial_sync_kbest = 5
        app._initial_sync_in_progress = True
        app._sync_period_ms = 10_000
        app._pending_initial_pid_read = True
        app._pid_request_seq = 0
        app.worker = types.SimpleNamespace(is_open=True, send=lambda payload: sent.append(payload))

        old_estimate_initial = gui_app_module.estimate_initial
        gui_app_module.estimate_initial = lambda points, k_best: TimeModel(a=1.0, b=2.0, have_lock=True)
        try:
            VirtualControllerApp._finalize_initial_sync(app, 500)
        finally:
            gui_app_module.estimate_initial = old_estimate_initial

        self.assertFalse(app._initial_sync_in_progress)
        self.assertFalse(app._pending_initial_pid_read)
        self.assertEqual(app._next_sync_due_ms, 10_500)
        self.assertEqual(len(sent), 1)
        self.assertEqual(sent[0][1], 0xB1)

    def test_poll_rx_reschedules_after_sensor_handler_exception(self) -> None:
        app = object.__new__(VirtualControllerApp)
        scheduled: list[tuple[int, object]] = []
        log_lines: list[tuple[str, str | None, str | None]] = []
        app._is_shutting_down = False
        app.worker = types.SimpleNamespace(rx_queue=queue.Queue())
        app.worker.rx_queue.put(
            RxEvent(
                pc_rx_ms=1010,
                parsed=ImuData(ts_ms=1000, accel=(0.0, 0.0, 1.0), magn=(0.0, 0.0, 0.0), gyro=(0.0, 0.0, 0.0)),
            )
        )
        app._send_control_if_due = lambda _now_ms: None
        app._kick_sync_if_due = lambda _now_ms: None
        app._poll_magnetometer_calibration_jobs = lambda: None
        app._poll_sensor_calibration_jobs = lambda _sensor: None
        app._rx_ok = collections.deque(maxlen=64)
        app.radio_var = types.SimpleNamespace(set=lambda _value: None)
        app._quality_0_10 = lambda: 10
        app.logger = types.SimpleNamespace(format_line=lambda _obj: "line", is_running=False)
        app._append_log_to_tabs = lambda line, msg_type, tag=None: log_lines.append((line, msg_type, tag))
        app.time_model = types.SimpleNamespace(a=1.0, b=0.0, have_lock=True)
        app._msg_to_dict = lambda _parsed: {"type": "imu"}
        app._update_mcu_time_label = lambda: None
        app._update_uart_status = lambda: None
        app._update_magnetometer_live_view = lambda _parsed, _pc_rx_ms: None
        app._update_accelerometer_live_view = lambda _parsed, _pc_rx_ms: (_ for _ in ()).throw(KeyError("raw_magnetometer"))
        app._update_gyroscope_live_view = lambda _parsed, _pc_rx_ms: None
        app.root = types.SimpleNamespace(after=lambda delay, callback: scheduled.append((delay, callback)))

        VirtualControllerApp._poll_rx(app)

        self.assertEqual(len(scheduled), 1)
        self.assertEqual(scheduled[0][0], 20)
        self.assertTrue(any("rx-handler:ImuData" in line for line, _msg_type, _tag in log_lines))

    def test_handle_motor_pid_response_updates_manual_fields(self) -> None:
        app = object.__new__(VirtualControllerApp)
        app.manual_tab = _DummyManualTab()
        app._motor_pid_values = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        VirtualControllerApp._handle_motor_pid_resp(
            app,
            MotorPidData(left_p=1.0, left_i=0.1, left_d=0.01, right_p=2.0, right_i=0.2, right_d=0.02),
        )

        self.assertEqual(app._motor_pid_values, (1.0, 0.1, 0.01, 2.0, 0.2, 0.02))
        assert app.manual_tab.pid_values is not None
        self.assertEqual(app.manual_tab.pid_values, (1.0, 0.1, 0.01, 2.0, 0.2, 0.02))

    def test_decode_tx_msg_decodes_pid_request_and_set_frames(self) -> None:
        app = object.__new__(VirtualControllerApp)

        pid_req = VirtualControllerApp._decode_tx_msg(app, build_pid_req(7, 1234))
        pid_set = VirtualControllerApp._decode_tx_msg(app, build_set_motor_pid(1.0, 0.1, 0.01, 2.0, 0.2, 0.02))

        self.assertEqual(pid_req, {"type": "B1", "seq": 7, "t1_pc_ms": 1234})
        self.assertEqual(pid_set["type"], "A2")
        self.assertAlmostEqual(pid_set["left_p"], 1.0)
        self.assertAlmostEqual(pid_set["right_d"], 0.02)

    def test_coordinate_tick_preserves_left_right_order_on_uart(self) -> None:
        app = object.__new__(VirtualControllerApp)
        sent: list[bytes] = []
        coord_tab = _DummyCoordTab()

        app._coord_running = True
        app._kill_active = False
        app._coord_tick_ms = 50
        app._coord_mission = None
        app._manual_neutral = 1500
        app.worker = types.SimpleNamespace(is_open=True, send=lambda payload: sent.append(payload))
        app.coord_tab = coord_tab
        app.root = types.SimpleNamespace(after=lambda *_args, **_kwargs: None)
        app._is_test_mode = lambda: False
        app._mission_uses_external_runtime = lambda mission=None: False
        app._control_timestamp_u32 = lambda: 123
        app._stop_external_runtime = lambda: None

        VirtualControllerApp._coord_send_tick(app)

        self.assertEqual(len(sent), 1)
        packet = sent[0]
        self.assertEqual(packet[1], TYPE_C0_CONTROL)
        ts_ms, left_cmd, right_cmd, duration_ms = struct.unpack_from("<IhhH", packet, 3)
        self.assertEqual(ts_ms, 123)
        self.assertEqual(left_cmd, 1600)
        self.assertEqual(right_cmd, 1400)
        self.assertEqual(duration_ms, 50)

    def test_build_telemetry_snapshot_includes_d3_sensor_tensor(self) -> None:
        sensor_tensor = parse_frame(
            Frame(
                msg_type=TYPE_D3_SENSOR_TENSOR,
                payload=struct.pack(
                    "<I12f",
                    77,
                    1.0, 2.0, 3.0,
                    4.0, 5.0, 6.0,
                    7.0, 8.0, 9.0,
                    10.0, 11.0, 12.0,
                ),
            )
        )

        app = object.__new__(VirtualControllerApp)
        app.time_model = TimeModel()
        app._last_mcu_ts_u32 = 77
        app._last_imu = None
        app._last_tacho = None
        app._last_motor = None
        app._last_d3 = sensor_tensor

        snapshot = VirtualControllerApp.build_telemetry_snapshot(app, now_ms=1234)

        self.assertIsNotNone(snapshot.sensor_tensor)
        assert snapshot.sensor_tensor is not None
        self.assertEqual(snapshot.sensor_tensor.ts_ms, 77)
        self.assertEqual(snapshot.sensor_tensor.linear_velocity, (1.0, 2.0, 3.0))
        self.assertEqual(snapshot.sensor_tensor.angular_velocity, (4.0, 5.0, 6.0))
        self.assertEqual(snapshot.sensor_tensor.linear_quality, (7.0, 8.0, 9.0))
        self.assertEqual(snapshot.sensor_tensor.angular_quality, (10.0, 11.0, 12.0))

    def test_estimate_pc_event_time_from_mcu_uses_inverse_time_model(self) -> None:
        app = object.__new__(VirtualControllerApp)
        app.time_model = TimeModel(a=2.0, b=100.0, have_lock=True)

        estimated = VirtualControllerApp._estimate_pc_event_ms_from_mcu(app, 500)

        self.assertEqual(estimated, 200)

    def test_estimate_pc_event_time_returns_none_without_lock(self) -> None:
        app = object.__new__(VirtualControllerApp)
        app.time_model = TimeModel()

        estimated = VirtualControllerApp._estimate_pc_event_ms_from_mcu(app, 500)

        self.assertIsNone(estimated)

    def test_magnetometer_recording_collects_dataset_rows(self) -> None:
        app = object.__new__(VirtualControllerApp)
        app._mag_datasets = []
        app._mag_dataset = None
        app._mag_recording_active = False
        app._mag_dataset_export_path = ""
        app._mag_sources = app._default_magnetometer_sources()
        app._mag_selected_source_id = "raw_magnetometer"
        app.magnetometer_tab = _DummyMagnetometerTab()
        app._make_magnetometer_dataset_name = lambda: "session_001"

        VirtualControllerApp._on_magnetometer_start_record(app)
        VirtualControllerApp._record_magnetometer_sample(
            app,
            timestamp_mcu=1000,
            timestamp_pc_rx=1010,
            timestamp_pc_est=1008,
            mx=1.0,
            my=0.0,
            mz=0.5,
        )
        VirtualControllerApp._on_magnetometer_stop_record(app)

        self.assertIsNotNone(app._mag_dataset)
        assert app._mag_dataset is not None
        self.assertEqual(app._mag_dataset.name, "session_001")
        self.assertEqual(len(app._mag_dataset.records), 1)
        self.assertEqual(app.magnetometer_tab.rows[0][0], 1)
        self.assertEqual(app.magnetometer_tab.summary, {
            "name": "session_001",
            "row_count": "1",
            "source_count": "1",
            "time_range": "1000..1000 MCU",
        })
        self.assertEqual(app.magnetometer_tab.recording_state, (False, True, True, False))

    def test_magnetometer_dataset_save_writes_valid_csv(self) -> None:
        app = object.__new__(VirtualControllerApp)
        app._mag_datasets = []
        app._mag_dataset = Dataset("session_002")
        app._mag_dataset.append(
            SampleRecord(
                stream_id="raw_magnetometer",
                stream_type="raw",
                producer_name="Raw Magnetometer",
                producer_version="builtin",
                timestamp_mcu=2000,
                timestamp_pc_rx=2015,
                timestamp_pc_est=2010,
                mag_x=0.25,
                mag_y=-0.5,
                mag_z=0.75,
                heading=153.4349488229,
                flags="",
            )
        )
        app._mag_recording_active = False
        app._mag_dataset_export_path = ""
        app._mag_sources = app._default_magnetometer_sources()
        app._mag_selected_source_id = "raw_magnetometer"
        app._mag_methods = {}
        app.magnetometer_tab = _DummyMagnetometerTab()

        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "mag_session.csv")

            saved = VirtualControllerApp._save_magnetometer_dataset_to_path(app, path)

            self.assertTrue(saved)
            self.assertEqual(app._mag_dataset_export_path, path)
            reopened = Dataset.from_csv(path)
            self.assertEqual(len(reopened.records), 1)
            self.assertEqual(reopened.records[0].stream_id, "raw_magnetometer")
            self.assertEqual(reopened.records[0].timestamp_pc_est, 2010)
            self.assertAlmostEqual(reopened.records[0].mag_z, 0.75)
            self.assertEqual(reopened.summary()["name"], "mag_session.csv")

    def test_magnetometer_dataset_save_writes_filter_metadata_header(self) -> None:
        app = object.__new__(VirtualControllerApp)
        app._mag_datasets = []
        app._mag_dataset = Dataset("session_003")
        app._mag_dataset.append(
            SampleRecord(
                stream_id="raw_magnetometer",
                stream_type="raw",
                producer_name="Raw Magnetometer",
                producer_version="builtin",
                timestamp_mcu=2000,
                timestamp_pc_rx=2015,
                timestamp_pc_est=2010,
                mag_x=0.25,
                mag_y=-0.5,
                mag_z=0.75,
                heading=153.4349488229,
                flags="",
            )
        )
        app._mag_recording_active = True
        app._mag_dataset_export_path = ""
        app._mag_sources = app._default_magnetometer_sources()
        app._mag_selected_source_id = "raw_magnetometer"
        app.magnetometer_tab = _DummyMagnetometerTab()
        plugin = LoadedMethodPlugin(
            method_id="method_1",
            name="Offset Method",
            version="1.0.0",
            file_path="/tmp/offset_method.py",
            info={
                "name": "Offset Method",
                "version": "1.0.0",
                "type": "method",
                "supports_calibrate": True,
                "supports_load_params": True,
                "supports_save_params": True,
                "supports_process": True,
                "input_schema": {},
                "output_schema": {},
            },
            module=types.SimpleNamespace(),
            status="ready",
        )
        plugin.derived_stream_id = "derived_method_1"
        plugin.record_enabled = True
        plugin.realtime_enabled = True
        plugin.calibration_params = {
            "algorithm_name": "Offset Method",
            "algorithm_version": "1.0.0",
            "schema_version": "1",
            "created_at": "2026-04-08T10:00:00",
            "params": {"offset_x": 1.0, "offset_y": 2.0, "offset_z": 3.0},
        }
        app._mag_methods = {"method_1": plugin}

        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "mag_session.csv")
            saved = VirtualControllerApp._save_magnetometer_dataset_to_path(app, path)

            self.assertTrue(saved)
            with open(path, "r", encoding="utf-8") as fh:
                header_line = fh.readline().strip()
            self.assertTrue(header_line.startswith("# magnetometer_metadata="))
            reopened = Dataset.from_csv(path)
            self.assertEqual(reopened.metadata["filters"][0]["name"], "Offset Method")
            self.assertEqual(reopened.metadata["filters"][0]["params"]["params"]["offset_x"], 1.0)
            self.assertEqual(reopened.metadata["recorded_method_ids"], ["method_1"])
            self.assertEqual(
                reopened.metadata["recorded_streams"],
                [
                    {
                        "stream_id": "raw_magnetometer",
                        "kind": "source",
                        "title": "Raw Magnetometer",
                        "stream_type": "raw",
                        "flags": "",
                    },
                    {
                        "stream_id": "derived_method_1",
                        "kind": "method",
                        "method_id": "method_1",
                        "name": "Offset Method",
                        "version": "1.0.0",
                        "file_path": "/tmp/offset_method.py",
                        "realtime_enabled": True,
                        "params": {
                            "algorithm_name": "Offset Method",
                            "algorithm_version": "1.0.0",
                            "schema_version": "1",
                            "created_at": "2026-04-08T10:00:00",
                            "params": {"offset_x": 1.0, "offset_y": 2.0, "offset_z": 3.0},
                        },
                    },
                ],
            )

    def test_load_multiple_datasets_sets_last_loaded_active(self) -> None:
        app = object.__new__(VirtualControllerApp)
        app._mag_datasets = []
        app._mag_dataset = None
        app._mag_recording_active = False
        app._mag_dataset_export_path = ""
        app._mag_sources = app._default_magnetometer_sources()
        app._mag_selected_source_id = "raw_magnetometer"
        app.magnetometer_tab = _DummyMagnetometerTab()

        with tempfile.TemporaryDirectory() as tmpdir:
            path_a = os.path.join(tmpdir, "a.csv")
            path_b = os.path.join(tmpdir, "b.csv")
            dataset_a = Dataset("a")
            dataset_a.append(
                SampleRecord("raw_a", "raw", "Raw A", "builtin", 10, 20, 21, 1.0, 2.0, 3.0, 45.0, "")
            )
            dataset_b = Dataset("b")
            dataset_b.append(
                SampleRecord("raw_b", "raw", "Raw B", "builtin", 30, 40, 41, 4.0, 5.0, 6.0, 90.0, "")
            )
            dataset_a.to_csv(path_a)
            dataset_b.to_csv(path_b)

            loaded_count = VirtualControllerApp._load_magnetometer_dataset_paths(app, (path_a, path_b))

            self.assertEqual(loaded_count, 2)
            self.assertEqual(len(app._mag_datasets), 2)
            self.assertIsNotNone(app._mag_dataset)
            assert app._mag_dataset is not None
            self.assertEqual(app._mag_dataset.source_path, path_b)
            self.assertEqual(app.magnetometer_tab.dataset_choices, (["a.csv", "b.csv"], 1))
            self.assertEqual(app.magnetometer_tab.summary, {
                "name": "b.csv",
                "row_count": "1",
                "source_count": "1",
                "time_range": "30..30 MCU",
            })
            self.assertEqual(app.magnetometer_tab.rows[0][0], 1)

    def test_trim_and_delete_edit_active_dataset_in_memory(self) -> None:
        app = object.__new__(VirtualControllerApp)
        app._mag_datasets = []
        app._mag_dataset = Dataset("edit_me")
        app._mag_dataset.extend([
            SampleRecord("s", "raw", "Raw", "builtin", 10, 20, 21, 1.0, 0.0, 0.0, 90.0, ""),
            SampleRecord("s", "raw", "Raw", "builtin", 11, 21, 22, 2.0, 0.0, 0.0, 90.0, ""),
            SampleRecord("s", "raw", "Raw", "builtin", 12, 22, 23, 3.0, 0.0, 0.0, 90.0, ""),
            SampleRecord("s", "raw", "Raw", "builtin", 13, 23, 24, 4.0, 0.0, 0.0, 90.0, ""),
        ])
        app._mag_datasets.append(app._mag_dataset)
        app._mag_recording_active = False
        app._mag_dataset_export_path = ""
        app._mag_sources = app._default_magnetometer_sources()
        app._mag_selected_source_id = "raw_magnetometer"
        app.magnetometer_tab = _DummyMagnetometerTab()

        trimmed = VirtualControllerApp._trim_active_magnetometer_dataset(app, [1, 2])

        self.assertTrue(trimmed)
        self.assertEqual([record.timestamp_mcu for record in app._mag_dataset.records], [11, 12])

        deleted = VirtualControllerApp._delete_active_magnetometer_rows(app, [0])

        self.assertTrue(deleted)
        self.assertEqual([record.timestamp_mcu for record in app._mag_dataset.records], [12])
        self.assertEqual(app.magnetometer_tab.summary, {
            "name": "edit_me",
            "row_count": "1",
            "source_count": "1",
            "time_range": "12..12 MCU",
        })

    def test_concatenate_selected_datasets_creates_new_active_dataset(self) -> None:
        app = object.__new__(VirtualControllerApp)
        app._mag_datasets = []
        app._mag_dataset = None
        app._mag_recording_active = False
        app._mag_dataset_export_path = ""
        app._mag_sources = app._default_magnetometer_sources()
        app._mag_selected_source_id = "raw_magnetometer"
        app.magnetometer_tab = _DummyMagnetometerTab()
        app._make_concatenated_magnetometer_dataset_name = lambda: "merged_001"

        dataset_a = Dataset("a")
        dataset_a.append(SampleRecord("a", "raw", "A", "builtin", 10, 20, 21, 1.0, 0.0, 0.0, 90.0, ""))
        dataset_b = Dataset("b")
        dataset_b.append(SampleRecord("b", "raw", "B", "builtin", 20, 30, 31, 0.0, 1.0, 0.0, 0.0, ""))
        app._mag_datasets.extend([dataset_a, dataset_b])

        merged = VirtualControllerApp._concatenate_selected_magnetometer_datasets(app, [0, 1])

        self.assertIsNotNone(merged)
        assert merged is not None
        self.assertEqual(merged.name, "merged_001")
        self.assertEqual(len(merged.records), 2)
        self.assertIs(app._mag_dataset, merged)
        self.assertEqual(app.magnetometer_tab.summary, {
            "name": "merged_001",
            "row_count": "2",
            "source_count": "2",
            "time_range": "10..20 MCU",
        })

    def test_select_active_dataset_by_index_switches_summary(self) -> None:
        app = object.__new__(VirtualControllerApp)
        app._mag_sources = app._default_magnetometer_sources()
        app._mag_selected_source_id = "raw_magnetometer"
        app._mag_datasets = []
        app._mag_recording_active = False
        app._mag_dataset_export_path = ""
        app.magnetometer_tab = _DummyMagnetometerTab()
        dataset_a = Dataset("a")
        dataset_a.append(SampleRecord("a", "raw", "A", "builtin", 10, 20, 21, 1.0, 0.0, 0.0, 90.0, ""))
        dataset_b = Dataset("b")
        dataset_b.append(SampleRecord("b", "raw", "B", "builtin", 20, 30, 31, 0.0, 1.0, 0.0, 0.0, ""))
        app._mag_datasets.extend([dataset_a, dataset_b])
        app._mag_dataset = dataset_b

        VirtualControllerApp._on_select_magnetometer_dataset(app, 0)

        self.assertIs(app._mag_dataset, dataset_a)
        self.assertEqual(app.magnetometer_tab.summary, {
            "name": "a",
            "row_count": "1",
            "source_count": "1",
            "time_range": "10..10 MCU",
        })

    def test_record_toggles_control_streams_written_to_dataset(self) -> None:
        app = object.__new__(VirtualControllerApp)
        app._mag_sources = app._default_magnetometer_sources()
        app._mag_sources["raw_magnetometer"]["record"] = False
        app._mag_sources["raw_heading"]["record"] = True
        app._mag_selected_source_id = "raw_heading"
        app._mag_datasets = []
        app._mag_dataset = Dataset("record_heading_only")
        app._mag_datasets.append(app._mag_dataset)
        app._mag_recording_active = True
        app._mag_dataset_export_path = ""
        app.magnetometer_tab = _DummyMagnetometerTab()

        VirtualControllerApp._record_magnetometer_sample(
            app,
            timestamp_mcu=100,
            timestamp_pc_rx=110,
            timestamp_pc_est=108,
            mx=1.0,
            my=0.0,
            mz=0.5,
        )

        self.assertEqual(len(app._mag_dataset.records), 1)
        self.assertEqual(app._mag_dataset.records[0].stream_id, "raw_heading")
        self.assertEqual(app._mag_dataset.records[0].flags, "heading_only")

    def test_source_toggle_updates_ui_state_snapshot(self) -> None:
        app = object.__new__(VirtualControllerApp)
        app._mag_sources = app._default_magnetometer_sources()
        app._mag_selected_source_id = "raw_magnetometer"
        app._mag_datasets = []
        app._mag_dataset = None
        app._mag_recording_active = False
        app._mag_dataset_export_path = ""
        app.magnetometer_tab = _DummyMagnetometerTab()

        VirtualControllerApp._on_toggle_magnetometer_source_show(app, "raw_heading", False)
        VirtualControllerApp._on_select_magnetometer_source(app, "raw_heading")

        assert app.magnetometer_tab.source_states is not None
        source_states, selected_source_id = app.magnetometer_tab.source_states
        self.assertFalse(source_states["raw_heading"]["show"])
        self.assertEqual(selected_source_id, "raw_heading")

    def test_register_magnetometer_method_updates_method_cards_and_selected_details(self) -> None:
        app = object.__new__(VirtualControllerApp)
        app._mag_methods = {}
        app._mag_selected_method_id = None
        app._mag_calibration_results = queue.Queue()
        app._mag_calibration_jobs = {}
        app.magnetometer_tab = _DummyMagnetometerTab()
        plugin = LoadedMethodPlugin(
            method_id="",
            name="Example Method",
            version="1.0.0",
            file_path="/tmp/example_method.py",
            info={
                "name": "Example Method",
                "version": "1.0.0",
                "type": "method",
                "supports_calibrate": True,
                "supports_load_params": True,
                "supports_save_params": True,
                "supports_process": True,
                "input_schema": {"mag": "xyz"},
                "output_schema": {"heading": "deg"},
            },
            module=types.SimpleNamespace(),
            status="ready",
        )

        registered = VirtualControllerApp._register_magnetometer_method(app, plugin)

        self.assertEqual(registered.method_id, "method_1")
        assert app.magnetometer_tab.method_states is not None
        method_states, selected_method_id = app.magnetometer_tab.method_states
        self.assertEqual(selected_method_id, "method_1")
        self.assertEqual(method_states["method_1"]["status"], "warning")
        self.assertTrue(method_states["method_1"]["can_calibrate"])
        self.assertEqual(app.magnetometer_tab.selected_method_details, {
            "name": "Example Method",
            "version": "1.0.0",
            "path": "/tmp/example_method.py",
            "status": "WARNING",
            "capabilities": "calibrate, load_params, save_params, process",
        })
        self.assertEqual(app.magnetometer_tab.selected_method_actions, {
            "can_calibrate": True,
            "can_load_params": True,
            "can_save_params": False,
            "can_clear_params": False,
            "can_enable_realtime": False,
            "can_disable_realtime": False,
        })

    def test_enable_magnetometer_method_realtime_defaults_record_on(self) -> None:
        app = object.__new__(VirtualControllerApp)
        app._mag_methods = {}
        app._mag_selected_method_id = "method_1"
        app._mag_latest_derived_streams = {}
        app._mag_primary_heading_stream_id = "raw_heading"
        app._mag_last_live_snapshot = None
        app._mag_offline_method_clouds = {}
        app._mag_recording_active = False
        app.magnetometer_tab = _DummyMagnetometerTab()

        plugin = LoadedMethodPlugin(
            method_id="method_1",
            name="Realtime Default Record",
            version="1.0.0",
            file_path="/tmp/realtime_default_record.py",
            info={
                "name": "Realtime Default Record",
                "version": "1.0.0",
                "type": "method",
                "supports_calibrate": True,
                "supports_load_params": True,
                "supports_save_params": True,
                "supports_process": True,
                "input_schema": {"mag": "xyz"},
                "output_schema": {"mag": "xyz"},
            },
            module=types.SimpleNamespace(),
            status="ready",
        )
        plugin.calibration_params = {
            "algorithm_name": "Realtime Default Record",
            "algorithm_version": "1.0.0",
            "schema_version": "1",
            "created_at": "2026-04-10T10:00:00Z",
            "params": {"offset": [0.0, 0.0, 0.0]},
        }
        app._mag_methods["method_1"] = plugin

        VirtualControllerApp._on_enable_magnetometer_method_realtime(app, "method_1")

        self.assertTrue(plugin.realtime_enabled)
        self.assertTrue(plugin.record_enabled)
        assert app.magnetometer_tab.method_states is not None
        method_states, _selected_method_id = app.magnetometer_tab.method_states
        self.assertTrue(method_states["method_1"]["record"])
        self.assertTrue(method_states["method_1"]["can_record"])

    def test_apply_magnetometer_calibration_result_stores_runtime_params(self) -> None:
        app = object.__new__(VirtualControllerApp)
        app._mag_methods = {}
        app._mag_selected_method_id = "method_1"
        app._mag_calibration_results = queue.Queue()
        app._mag_calibration_jobs = {"method_1": {"started_at": 0.0, "dataset_name": "session.csv"}}
        app.magnetometer_tab = _DummyMagnetometerTab()
        app._open_magnetometer_method_diagnostics = lambda _method_id: None

        plugin = LoadedMethodPlugin(
            method_id="method_1",
            name="Calibrating Method",
            version="1.0.0",
            file_path="/tmp/cal_method.py",
            info={
                "name": "Calibrating Method",
                "version": "1.0.0",
                "type": "method",
                "supports_calibrate": True,
                "supports_load_params": True,
                "supports_save_params": True,
                "supports_process": True,
                "input_schema": {"mag": "xyz"},
                "output_schema": {"heading": "deg"},
            },
            module=types.SimpleNamespace(),
            status="ready",
        )
        plugin.is_calibrating = True
        app._mag_methods["method_1"] = plugin

        VirtualControllerApp._apply_magnetometer_calibration_result(
            app,
            "method_1",
            CalibrationRunResult(
                ok=True,
                params={
                    "algorithm_name": "Calibrating Method",
                    "algorithm_version": "1.0.0",
                    "schema_version": "1",
                    "created_at": "2026-04-05T12:00:00Z",
                    "params": {"offset": [1.0, 2.0, 3.0]},
                },
                warnings=[],
                report="ok",
            ),
        )

        self.assertFalse(plugin.is_calibrating)
        self.assertEqual(plugin.calibration_dataset_name, "session.csv")
        self.assertEqual(plugin.calibration_report, "ok")
        self.assertEqual(plugin.calibration_params["params"]["offset"], [1.0, 2.0, 3.0])
        assert app.magnetometer_tab.selected_method_details is not None
        self.assertEqual(app.magnetometer_tab.selected_method_details["status"], "READY")
        assert app.magnetometer_tab.selected_method_actions is not None
        self.assertEqual(app.magnetometer_tab.selected_method_actions["can_save_params"], True)
        self.assertEqual(app.magnetometer_tab.selected_method_actions["can_clear_params"], True)
        self.assertEqual(app.magnetometer_tab.selected_method_actions["can_enable_realtime"], True)
        self.assertEqual(app.magnetometer_tab.selected_method_actions["can_disable_realtime"], False)

    def test_apply_magnetometer_calibration_result_builds_offline_cloud(self) -> None:
        app = object.__new__(VirtualControllerApp)
        app._mag_methods = {}
        app._mag_selected_method_id = "method_1"
        app._mag_calibration_results = queue.Queue()
        app._mag_calibration_jobs = {"method_1": {"started_at": 0.0, "dataset_name": "session.csv"}}
        app._mag_dataset = Dataset(
            "session.csv",
            records=[
                SampleRecord("raw_magnetometer", "raw", "Raw Magnetometer", "builtin", 100, 110, 108, 1.0, 2.0, 3.0, 26.0, ""),
                SampleRecord("raw_magnetometer", "raw", "Raw Magnetometer", "builtin", 200, 210, 208, 4.0, 5.0, 6.0, 32.0, ""),
            ],
            source_path="/tmp/session.csv",
        )
        app._mag_datasets = [app._mag_dataset]
        app.magnetometer_tab = _DummyMagnetometerTab()
        app._open_magnetometer_method_diagnostics = lambda _method_id: None

        def _process(sample: dict[str, object], params: dict[str, object]) -> dict[str, object]:
            inner = params["params"]
            return {
                "mag_x": float(sample["mag_x"]) - float(inner["offset_x"]),
                "mag_y": float(sample["mag_y"]) - float(inner["offset_y"]),
                "mag_z": float(sample["mag_z"]) - float(inner["offset_z"]),
            }

        plugin = LoadedMethodPlugin(
            method_id="method_1",
            name="Calibrating Method",
            version="1.0.0",
            file_path="/tmp/cal_method.py",
            info={
                "name": "Calibrating Method",
                "version": "1.0.0",
                "type": "method",
                "supports_calibrate": True,
                "supports_load_params": True,
                "supports_save_params": True,
                "supports_process": True,
                "input_schema": {"mag": "xyz"},
                "output_schema": {"heading": "deg"},
            },
            module=types.SimpleNamespace(process=_process),
            status="ready",
        )
        plugin.is_calibrating = True
        app._mag_methods["method_1"] = plugin

        VirtualControllerApp._apply_magnetometer_calibration_result(
            app,
            "method_1",
            CalibrationRunResult(
                ok=True,
                params={
                    "algorithm_name": "Calibrating Method",
                    "algorithm_version": "1.0.0",
                    "schema_version": "1",
                    "created_at": "2026-04-05T12:00:00Z",
                    "params": {"offset_x": 1.0, "offset_y": 2.0, "offset_z": 3.0},
                },
                warnings=[],
                report="ok",
            ),
        )

        assert app.magnetometer_tab.method_dataset_clouds is not None
        derived_records = app.magnetometer_tab.method_dataset_clouds["method_1"]
        self.assertEqual(len(derived_records), 2)
        self.assertEqual((derived_records[0].mag_x, derived_records[0].mag_y, derived_records[0].mag_z), (0.0, 0.0, 0.0))

    def test_apply_loaded_param_profile_sets_runtime_state_and_warning_status(self) -> None:
        app = object.__new__(VirtualControllerApp)
        app._mag_methods = {}
        app._mag_selected_method_id = "method_1"
        app.magnetometer_tab = _DummyMagnetometerTab()

        plugin = LoadedMethodPlugin(
            method_id="method_1",
            name="Param Method",
            version="1.0.0",
            file_path="/tmp/param_method.py",
            info={
                "name": "Param Method",
                "version": "1.0.0",
                "type": "method",
                "supports_calibrate": True,
                "supports_load_params": True,
                "supports_save_params": True,
                "supports_process": True,
                "input_schema": {"mag": "xyz"},
                "output_schema": {"heading": "deg"},
            },
            module=types.SimpleNamespace(),
            status="ready",
        )
        app._mag_methods["method_1"] = plugin

        warnings_seen: list[tuple[str, str]] = []
        import app.gui_app as gui_app_module
        old_showwarning = gui_app_module.messagebox.showwarning
        gui_app_module.messagebox.showwarning = lambda title, message: warnings_seen.append((title, message))
        try:
            VirtualControllerApp._apply_magnetometer_param_io_result(
                app,
                "method_1",
                ParamIoResult(
                    ok=True,
                    params={
                        "algorithm_name": "Other Method",
                        "algorithm_version": "2.0.0",
                        "schema_version": "1",
                        "created_at": "2026-04-05T12:00:00Z",
                        "params": {"offset": [0.0, 0.0, 0.0]},
                    },
                    warnings=[
                        "algorithm_name mismatch: expected 'Param Method', got 'Other Method'",
                        "algorithm_version mismatch: expected '1.0.0', got '2.0.0'",
                    ],
                ),
                action="load",
                path="/tmp/other_method.json",
            )
        finally:
            gui_app_module.messagebox.showwarning = old_showwarning

        self.assertEqual(plugin.params_profile_path, "/tmp/other_method.json")
        self.assertEqual(plugin.calibration_params["params"]["offset"], [0.0, 0.0, 0.0])
        self.assertEqual(len(plugin.params_warnings), 2)
        assert app.magnetometer_tab.selected_method_details is not None
        self.assertEqual(app.magnetometer_tab.selected_method_details["status"], "WARNING")
        self.assertEqual(len(warnings_seen), 1)

    def test_clear_magnetometer_method_params_resets_runtime_state(self) -> None:
        app = object.__new__(VirtualControllerApp)
        app._mag_methods = {}
        app._mag_selected_method_id = "method_1"
        app._mag_latest_derived_streams = {"method_1": {"heading": 42.0}}
        app._mag_offline_method_clouds = {"method_1": []}
        app._mag_primary_heading_stream_id = "method_1"
        app._mag_last_live_snapshot = None
        app._mag_recording_active = False
        app.magnetometer_tab = _DummyMagnetometerTab()

        plugin = LoadedMethodPlugin(
            method_id="method_1",
            name="Resettable Method",
            version="1.0.0",
            file_path="/tmp/resettable_method.py",
            info={
                "name": "Resettable Method",
                "version": "1.0.0",
                "type": "method",
                "supports_calibrate": True,
                "supports_load_params": True,
                "supports_save_params": True,
                "supports_process": True,
                "input_schema": {"mag_x": "float", "mag_y": "float", "mag_z": "float"},
                "output_schema": {"mag_x": "float", "mag_y": "float", "mag_z": "float"},
            },
            module=types.SimpleNamespace(),
            status="ready",
        )
        plugin.calibration_params = {
            "algorithm_name": "Resettable Method",
            "algorithm_version": "1.0.0",
            "schema_version": "1",
            "created_at": "2026-04-12T12:00:00Z",
            "params": {"offset": [1.0, 2.0, 3.0]},
        }
        plugin.params_profile_path = "/tmp/resettable_params.json"
        plugin.calibration_report = "ok"
        plugin.calibration_dataset_name = "session.csv"
        plugin.calibration_runtime_s = 0.42
        plugin.calibration_progress = 1.0
        plugin.realtime_enabled = True
        plugin.record_enabled = True
        plugin.last_output = {"mag_x": 1.0, "mag_y": 2.0, "mag_z": 3.0}
        plugin.derived_stream_id = "derived_method_1"
        app._mag_methods["method_1"] = plugin

        VirtualControllerApp._on_clear_magnetometer_method_params(app, "method_1")

        self.assertIsNone(plugin.calibration_params)
        self.assertIsNone(plugin.params_profile_path)
        self.assertEqual(plugin.calibration_report, "")
        self.assertIsNone(plugin.calibration_dataset_name)
        self.assertIsNone(plugin.calibration_runtime_s)
        self.assertEqual(plugin.calibration_progress, 0.0)
        self.assertFalse(plugin.realtime_enabled)
        self.assertFalse(plugin.record_enabled)
        self.assertIsNone(plugin.last_output)
        self.assertEqual(app._mag_latest_derived_streams, {})
        self.assertEqual(app._mag_offline_method_clouds, {})
        assert app.magnetometer_tab.selected_method_actions is not None
        self.assertFalse(app.magnetometer_tab.selected_method_actions["can_save_params"])
        self.assertFalse(app.magnetometer_tab.selected_method_actions["can_clear_params"])
        self.assertEqual(app.magnetometer_tab.selected_method_details["status"], "WARNING")

    def test_serialize_magnetometer_settings_includes_paths_and_view_state(self) -> None:
        app = object.__new__(VirtualControllerApp)
        app._mag_sources = app._default_magnetometer_sources()
        app._mag_sources["raw_heading"]["show"] = False
        app._mag_selected_source_id = "raw_heading"
        app._mag_dataset = Dataset("session.csv", source_path="/tmp/session.csv")
        app._mag_datasets = [app._mag_dataset]
        app._mag_file_dialog_dirs = {"dataset": "/tmp", "params": "/tmp", "plugin": "/tmp", "metrics": "/tmp"}
        app.magnetometer_tab = _DummyMagnetometerTab()
        app.magnetometer_tab.view_state = {
            "projection_mode": "3D",
            "auto_fit": False,
            "view_options": {"show raw points": True},
        }
        plugin = LoadedMethodPlugin(
            method_id="method_1",
            name="Persisted Method",
            version="1.0.0",
            file_path="/tmp/persisted_method.py",
            info={
                "name": "Persisted Method",
                "version": "1.0.0",
                "type": "method",
                "supports_calibrate": True,
                "supports_load_params": True,
                "supports_save_params": True,
                "supports_process": True,
                "input_schema": {},
                "output_schema": {},
            },
            module=types.SimpleNamespace(),
            status="ready",
        )
        plugin.show_enabled = True
        plugin.record_enabled = True
        plugin.realtime_enabled = True
        plugin.params_profile_path = "/tmp/persisted_params.json"
        plugin.stream_bindings[SCOPE_CALIBRATE] = {"mag_input": "builtin::raw_magnetometer"}
        app._mag_methods = {"method_1": plugin}
        app._mag_selected_method_id = "method_1"
        app._mag_primary_heading_stream_id = "method_1"

        snapshot = VirtualControllerApp._serialize_magnetometer_settings(app)

        self.assertEqual(snapshot["datasets"], ["/tmp/session.csv"])
        self.assertEqual(snapshot["active_dataset_path"], "/tmp/session.csv")
        self.assertEqual(snapshot["selected_source_id"], "raw_heading")
        self.assertEqual(snapshot["selected_method_path"], "/tmp/persisted_method.py")
        self.assertEqual(snapshot["primary_heading"], {"kind": "method", "file_path": "/tmp/persisted_method.py"})
        self.assertEqual(snapshot["view_state"]["projection_mode"], "3D")
        self.assertEqual(snapshot["methods"][0]["params_profile_path"], "/tmp/persisted_params.json")
        self.assertEqual(snapshot["methods"][0]["bindings"][SCOPE_CALIBRATE]["mag_input"], "builtin::raw_magnetometer")

    def test_clear_magnetometer_datasets_removes_all_loaded_datasets(self) -> None:
        app = object.__new__(VirtualControllerApp)
        app._mag_sources = app._default_magnetometer_sources()
        app._mag_selected_source_id = "raw_magnetometer"
        app._mag_recording_active = False
        app._mag_dataset_export_path = "/tmp/old.csv"
        app._mag_methods = {}
        app._mag_selected_method_id = None
        app._mag_latest_derived_streams = {}
        app._mag_offline_method_clouds = {}
        app.magnetometer_tab = _DummyMagnetometerTab()

        dataset_a = Dataset("a", records=[
            SampleRecord("raw_magnetometer", "raw", "Raw Magnetometer", "builtin", 10, 20, 21, 1.0, 0.0, 0.0, 90.0, "")
        ])
        dataset_b = Dataset("b", records=[
            SampleRecord("raw_magnetometer", "raw", "Raw Magnetometer", "builtin", 20, 30, 31, 0.0, 1.0, 0.0, 0.0, "")
        ])
        app._mag_datasets = [dataset_a, dataset_b]
        app._mag_dataset = dataset_b

        VirtualControllerApp._on_clear_magnetometer_datasets(app)

        self.assertEqual(app._mag_datasets, [])
        self.assertIsNone(app._mag_dataset)
        self.assertEqual(app._mag_dataset_export_path, "")
        self.assertEqual(app.magnetometer_tab.summary, {
            "name": "—",
            "row_count": "0",
            "source_count": "0",
            "time_range": "—",
        })
        self.assertEqual(app.magnetometer_tab.rows, [])
        self.assertFalse(app.magnetometer_tab.clear_dataset_enabled)

    def test_binding_change_rejects_cycle_and_keeps_previous_binding(self) -> None:
        app = object.__new__(VirtualControllerApp)
        app._mag_sources = app._default_magnetometer_sources()
        app._mag_selected_source_id = "raw_magnetometer"
        app._mag_dataset = Dataset(
            "session",
            records=[
                SampleRecord("raw_magnetometer", "raw", "Raw Magnetometer", "builtin", 10, 20, 21, 1.0, 0.0, 0.0, 90.0, "")
            ],
        )
        app._mag_datasets = [app._mag_dataset]
        app._mag_methods = {}
        app._mag_selected_method_id = "method_1"
        app._mag_calibration_results = queue.Queue()
        app._mag_calibration_jobs = {}
        app._mag_latest_derived_streams = {}
        app._mag_offline_method_clouds = {}
        app._mag_primary_heading_stream_id = "raw_heading"
        app.magnetometer_tab = _DummyMagnetometerTab()

        def _make_plugin(method_id: str, name: str, path: str) -> LoadedMethodPlugin:
            plugin = LoadedMethodPlugin(
                method_id=method_id,
                name=name,
                version="1.0.0",
                file_path=path,
                info={
                    "name": name,
                    "version": "1.0.0",
                    "type": "method",
                    "supports_calibrate": True,
                    "supports_load_params": True,
                    "supports_save_params": True,
                    "supports_process": True,
                    "input_schema": {"mag_x": "float", "mag_y": "float", "mag_z": "float"},
                    "output_schema": {"mag_x": "float", "mag_y": "float", "mag_z": "float"},
                    "stream_requirements": {
                        "calibrate": [
                            {
                                "slot": "mag_input",
                                "kind": "imu.magnetometer_vector",
                                "required": True,
                            }
                        ],
                        "process": [
                            {
                                "slot": "mag_input",
                                "kind": "imu.magnetometer_vector",
                                "required": True,
                            }
                        ],
                    },
                },
                module=types.SimpleNamespace(process=lambda sample, params: sample),
                status="ready",
            )
            plugin.calibration_params = {
                "algorithm_name": name,
                "algorithm_version": "1.0.0",
                "schema_version": "1",
                "created_at": "2026-04-12T12:00:00Z",
                "params": {},
            }
            return plugin

        plugin_a = _make_plugin("method_1", "Method A", "/tmp/method_a.py")
        plugin_b = _make_plugin("method_2", "Method B", "/tmp/method_b.py")
        plugin_a.derived_stream_id = "derived_method_1"
        plugin_b.derived_stream_id = "derived_method_2"
        plugin_a.stream_requirements = infer_stream_requirements(plugin_a.info)
        plugin_b.stream_requirements = infer_stream_requirements(plugin_b.info)
        plugin_a.stream_bindings[SCOPE_CALIBRATE] = {
            "mag_input": make_method_producer_id(plugin_b.file_path, "imu.magnetometer_vector")
        }
        app._mag_methods = {"method_1": plugin_a, "method_2": plugin_b}

        accepted = VirtualControllerApp._on_change_magnetometer_method_binding(
            app,
            "method_2",
            "mag_input",
            make_method_producer_id(plugin_a.file_path, "imu.magnetometer_vector"),
        )

        self.assertFalse(accepted)
        self.assertEqual(plugin_b.stream_bindings[SCOPE_CALIBRATE], {})
        validation = plugin_b.routing_validation[SCOPE_CALIBRATE].as_dict()
        disabled_candidates = validation["slot_candidates"]["mag_input"]
        matching = [
            candidate
            for candidate in disabled_candidates
            if candidate["producer_id"] == make_method_producer_id(plugin_a.file_path, "imu.magnetometer_vector")
        ]
        self.assertEqual(len(matching), 1)
        self.assertIn("cycle", matching[0]["disabled_reason"])

    def test_offline_method_clouds_can_chain_through_upstream_method_binding(self) -> None:
        app = object.__new__(VirtualControllerApp)
        app._mag_sources = app._default_magnetometer_sources()
        app._mag_selected_source_id = "raw_magnetometer"
        app._mag_dataset = Dataset(
            "session.csv",
            records=[
                SampleRecord("raw_magnetometer", "raw", "Raw Magnetometer", "builtin", 100, 110, 108, 5.0, 2.0, 1.0, 68.0, ""),
                SampleRecord("raw_magnetometer", "raw", "Raw Magnetometer", "builtin", 200, 210, 208, 8.0, 3.0, 1.0, 69.0, ""),
            ],
            source_path="/tmp/session.csv",
        )
        app._mag_datasets = [app._mag_dataset]
        app._mag_methods = {}
        app._mag_selected_method_id = "method_2"
        app._mag_calibration_results = queue.Queue()
        app._mag_calibration_jobs = {}
        app._mag_latest_derived_streams = {}
        app._mag_offline_method_clouds = {}
        app._mag_primary_heading_stream_id = "raw_heading"
        app.magnetometer_tab = _DummyMagnetometerTab()
        app._open_magnetometer_method_diagnostics = lambda _method_id: None

        plugin_a = LoadedMethodPlugin(
            method_id="method_1",
            name="Offset Method",
            version="1.0.0",
            file_path="/tmp/offset_method.py",
            info={
                "name": "Offset Method",
                "version": "1.0.0",
                "type": "method",
                "supports_calibrate": True,
                "supports_load_params": True,
                "supports_save_params": True,
                "supports_process": True,
                "input_schema": {"mag_x": "float", "mag_y": "float", "mag_z": "float"},
                "output_schema": {"mag_x": "float", "mag_y": "float", "mag_z": "float"},
            },
            module=types.SimpleNamespace(
                process=lambda sample, params: {
                    "mag_x": float(sample["mag_x"]) - float(params["params"]["offset_x"]),
                    "mag_y": float(sample["mag_y"]) - float(params["params"]["offset_y"]),
                    "mag_z": float(sample["mag_z"]) - float(params["params"]["offset_z"]),
                }
            ),
            status="ready",
        )
        plugin_a.calibration_params = {
            "algorithm_name": "Offset Method",
            "algorithm_version": "1.0.0",
            "schema_version": "1",
            "created_at": "2026-04-12T12:00:00Z",
            "params": {"offset_x": 1.0, "offset_y": 1.0, "offset_z": 0.0},
        }
        plugin_a.derived_stream_id = "derived_method_1"
        plugin_a.stream_requirements = infer_stream_requirements(plugin_a.info)

        plugin_b = LoadedMethodPlugin(
            method_id="method_2",
            name="Scale Method",
            version="1.0.0",
            file_path="/tmp/scale_method.py",
            info={
                "name": "Scale Method",
                "version": "1.0.0",
                "type": "method",
                "supports_calibrate": True,
                "supports_load_params": True,
                "supports_save_params": True,
                "supports_process": True,
                "input_schema": {"mag_x": "float", "mag_y": "float", "mag_z": "float"},
                "output_schema": {"mag_x": "float", "mag_y": "float", "mag_z": "float"},
                "stream_requirements": {
                    "calibrate": [
                        {
                            "slot": "mag_input",
                            "kind": "imu.magnetometer_vector",
                            "required": True,
                        }
                    ]
                },
            },
            module=types.SimpleNamespace(
                process=lambda sample, params: {
                    "mag_x": float(sample["mag_x"]) * float(params["params"]["scale"]),
                    "mag_y": float(sample["mag_y"]) * float(params["params"]["scale"]),
                    "mag_z": float(sample["mag_z"]) * float(params["params"]["scale"]),
                }
            ),
            status="ready",
        )
        plugin_b.calibration_params = {
            "algorithm_name": "Scale Method",
            "algorithm_version": "1.0.0",
            "schema_version": "1",
            "created_at": "2026-04-12T12:00:00Z",
            "params": {"scale": 2.0},
        }
        plugin_b.derived_stream_id = "derived_method_2"
        plugin_b.stream_requirements = infer_stream_requirements(plugin_b.info)
        plugin_b.stream_bindings[SCOPE_CALIBRATE] = {
            "mag_input": make_method_producer_id(plugin_a.file_path, "imu.magnetometer_vector")
        }

        app._mag_methods = {"method_1": plugin_a, "method_2": plugin_b}

        VirtualControllerApp._refresh_all_magnetometer_method_dataset_clouds(app)

        assert app.magnetometer_tab.method_dataset_clouds is not None
        derived_records = app.magnetometer_tab.method_dataset_clouds["method_2"]
        self.assertEqual(len(derived_records), 2)
        self.assertEqual((derived_records[0].mag_x, derived_records[0].mag_y, derived_records[0].mag_z), (8.0, 2.0, 2.0))
        self.assertEqual((derived_records[1].mag_x, derived_records[1].mag_y, derived_records[1].mag_z), (14.0, 4.0, 2.0))

    def test_apply_magnetometer_calibration_error_marks_method_red(self) -> None:
        app = object.__new__(VirtualControllerApp)
        app._mag_methods = {}
        app._mag_selected_method_id = "method_1"
        app._mag_calibration_results = queue.Queue()
        app._mag_calibration_jobs = {"method_1": {"started_at": 0.0, "dataset_name": "session.csv"}}
        app.magnetometer_tab = _DummyMagnetometerTab()
        opened: list[str] = []
        app._open_magnetometer_method_diagnostics = lambda method_id: opened.append(method_id)

        plugin = LoadedMethodPlugin(
            method_id="method_1",
            name="Broken Method",
            version="1.0.0",
            file_path="/tmp/broken_method.py",
            info={
                "name": "Broken Method",
                "version": "1.0.0",
                "type": "method",
                "supports_calibrate": True,
                "supports_load_params": True,
                "supports_save_params": True,
                "supports_process": True,
                "input_schema": {"mag": "xyz"},
                "output_schema": {"heading": "deg"},
            },
            module=None,
            status="ready",
        )
        plugin.is_calibrating = True
        app._mag_methods["method_1"] = plugin

        VirtualControllerApp._apply_magnetometer_calibration_result(
            app,
            "method_1",
            CalibrationRunResult(
                ok=False,
                warnings=["plugin failed"],
                diagnostics=PluginDiagnostics(
                    name="Broken Method",
                    version="1.0.0",
                    file_path="/tmp/broken_method.py",
                    last_action="calibrate",
                    error_text="boom",
                    traceback_text="traceback",
                    warnings=["plugin failed"],
                ),
            ),
        )

        self.assertEqual(plugin.status, "error")
        self.assertIsNotNone(plugin.diagnostics)
        self.assertEqual(opened, ["method_1"])

    def test_remove_magnetometer_method_clears_selection_and_primary_heading(self) -> None:
        app = object.__new__(VirtualControllerApp)
        app._mag_methods = {}
        app._mag_selected_method_id = "method_1"
        app._mag_latest_derived_streams = {"method_1": {"heading": 42.0}}
        app._mag_offline_method_clouds = {"method_1": []}
        app._mag_primary_heading_stream_id = "method_1"
        app.magnetometer_tab = _DummyMagnetometerTab()
        app._mag_dataset = None
        app._mag_datasets = []

        plugin = LoadedMethodPlugin(
            method_id="method_1",
            name="Removable Method",
            version="1.0.0",
            file_path="/tmp/removable_method.py",
            info={
                "name": "Removable Method",
                "version": "1.0.0",
                "type": "method",
                "supports_calibrate": True,
                "supports_load_params": True,
                "supports_save_params": True,
                "supports_process": True,
                "input_schema": {},
                "output_schema": {},
            },
            module=types.SimpleNamespace(),
            status="ready",
        )
        app._mag_methods["method_1"] = plugin

        VirtualControllerApp._on_remove_magnetometer_method(app, "method_1")

        self.assertEqual(app._mag_methods, {})
        self.assertEqual(app._mag_selected_method_id, None)
        self.assertEqual(app._mag_primary_heading_stream_id, "raw_heading")
        self.assertEqual(app._mag_latest_derived_streams, {})
        self.assertEqual(app.magnetometer_tab.method_dataset_clouds, {})

    def test_realtime_method_processing_keeps_raw_and_derived_streams_separate(self) -> None:
        app = object.__new__(VirtualControllerApp)
        app.time_model = TimeModel(a=2.0, b=100.0, have_lock=True)
        app._mag_sources = app._default_magnetometer_sources()
        app._mag_sources["raw_heading"]["record"] = False
        app._mag_selected_source_id = "raw_magnetometer"
        app._mag_methods = {}
        app._mag_selected_method_id = "method_1"
        app._mag_latest_derived_streams = {}
        app._mag_primary_heading_stream_id = "method_1"
        app._mag_last_live_snapshot = None
        app._mag_datasets = []
        app._mag_dataset = Dataset("live_session")
        app._mag_datasets.append(app._mag_dataset)
        app._mag_recording_active = True
        app._mag_dataset_export_path = ""
        app.magnetometer_tab = _DummyMagnetometerTab()
        app._mag_calibration_results = queue.Queue()
        app._mag_calibration_jobs = {}

        plugin = LoadedMethodPlugin(
            method_id="method_1",
            name="Offset Method",
            version="1.0.0",
            file_path="/tmp/offset_method.py",
            info={
                "name": "Offset Method",
                "version": "1.0.0",
                "type": "method",
                "supports_calibrate": True,
                "supports_load_params": True,
                "supports_save_params": True,
                "supports_process": True,
                "input_schema": {"mag": "xyz"},
                "output_schema": {"mag": "xyz", "heading": "deg"},
            },
            module=types.SimpleNamespace(
                process=lambda sample, params: {
                    "timestamp_mcu": sample["timestamp_mcu"],
                    "timestamp_pc_rx": sample["timestamp_pc_rx"],
                    "timestamp_pc_est": sample["timestamp_pc_est"],
                    "mag_x": float(sample["mag_x"]) - float(params["params"]["offset"][0]),
                    "mag_y": float(sample["mag_y"]) - float(params["params"]["offset"][1]),
                    "mag_z": float(sample["mag_z"]) - float(params["params"]["offset"][2]),
                    "heading": 12.5,
                    "flags": "derived_rt",
                }
            ),
            status="ready",
        )
        plugin.calibration_params = {
            "algorithm_name": "Offset Method",
            "algorithm_version": "1.0.0",
            "schema_version": "1",
            "created_at": "2026-04-06T12:00:00Z",
            "params": {"offset": [1.0, 0.0, 0.0]},
        }
        plugin.realtime_enabled = True
        plugin.show_enabled = True
        plugin.record_enabled = True
        plugin.derived_stream_id = "derived_method_1"
        app._mag_methods["method_1"] = plugin

        VirtualControllerApp._update_magnetometer_live_view(
            app,
            ImuData(ts_ms=1000, accel=(0.0, 0.0, 0.0), magn=(3.0, 2.0, 1.0), gyro=(0.0, 0.0, 0.0)),
            1110,
        )

        self.assertEqual(len(app._mag_dataset.records), 2)
        self.assertEqual(app._mag_dataset.records[0].stream_id, "raw_magnetometer")
        self.assertEqual(app._mag_dataset.records[1].stream_id, "derived_method_1")
        self.assertEqual(app._mag_dataset.records[1].stream_type, "derived")
        self.assertAlmostEqual(app._mag_dataset.records[1].mag_x, 2.0)
        self.assertEqual(app._mag_dataset.records[1].flags, "derived_rt")
        self.assertEqual(plugin.last_output["mag_x"], 2.0)
        self.assertEqual(plugin.last_output["heading"], 12.5)
        derived_streams = app.magnetometer_tab.live_updates[-1]["derived_streams"]
        self.assertIn("method_1", derived_streams)
        self.assertEqual(derived_streams["method_1"]["heading"], 12.5)
        self.assertEqual(app.magnetometer_tab.live_updates[-1]["selected_output_heading"], 12.5)
        self.assertEqual(app.magnetometer_tab.live_updates[-1]["selected_source_label"], "Offset Method")
        self.assertEqual(app.magnetometer_tab.heading_routing, {
            "stream_choices": [("raw_heading", "Raw Heading"), ("method_1", "Offset Method")],
            "primary_stream_id": "method_1",
        })

    def test_primary_heading_selection_updates_current_data_panel_without_new_sample(self) -> None:
        app = object.__new__(VirtualControllerApp)
        app.time_model = TimeModel(a=2.0, b=100.0, have_lock=True)
        app._mag_sources = app._default_magnetometer_sources()
        app._mag_selected_source_id = "raw_magnetometer"
        app._mag_methods = {}
        app._mag_selected_method_id = "method_1"
        app._mag_primary_heading_stream_id = "raw_heading"
        app._mag_latest_derived_streams = {}
        app._mag_last_live_snapshot = None
        app._mag_datasets = []
        app._mag_dataset = None
        app._mag_recording_active = False
        app._mag_dataset_export_path = ""
        app.magnetometer_tab = _DummyMagnetometerTab()
        app._mag_calibration_results = queue.Queue()
        app._mag_calibration_jobs = {}

        plugin = LoadedMethodPlugin(
            method_id="method_1",
            name="Half Scale",
            version="1.0.0",
            file_path="/tmp/half_scale.py",
            info={
                "name": "Half Scale",
                "version": "1.0.0",
                "type": "method",
                "supports_calibrate": True,
                "supports_load_params": True,
                "supports_save_params": True,
                "supports_process": True,
                "input_schema": {"mag": "xyz"},
                "output_schema": {"mag": "xyz", "heading": "deg"},
            },
            module=types.SimpleNamespace(
                process=lambda sample, params: {
                    "timestamp_mcu": sample["timestamp_mcu"],
                    "timestamp_pc_rx": sample["timestamp_pc_rx"],
                    "timestamp_pc_est": sample["timestamp_pc_est"],
                    "mag_x": sample["mag_x"] * 0.5,
                    "mag_y": sample["mag_y"] * 0.5,
                    "mag_z": sample["mag_z"] * 0.5,
                    "heading": 210.0,
                }
            ),
            status="ready",
        )
        plugin.calibration_params = {
            "algorithm_name": "Half Scale",
            "algorithm_version": "1.0.0",
            "schema_version": "1",
            "created_at": "2026-04-07T12:00:00Z",
            "params": {"scale": 0.5},
        }
        plugin.realtime_enabled = True
        plugin.show_enabled = True
        app._mag_methods["method_1"] = plugin

        VirtualControllerApp._update_magnetometer_live_view(
            app,
            ImuData(ts_ms=1500, accel=(0.0, 0.0, 0.0), magn=(1.0, 0.0, 0.0), gyro=(0.0, 0.0, 0.0)),
            1510,
        )
        self.assertEqual(app.magnetometer_tab.live_updates[-1]["selected_source_label"], "Raw Heading")
        self.assertEqual(app.magnetometer_tab.live_updates[-1]["selected_output_heading"], 90.0)

        VirtualControllerApp._on_select_magnetometer_primary_heading(app, "method_1")

        self.assertEqual(app.magnetometer_tab.primary_output_display, {
            "heading": 210.0,
            "source_label": "Half Scale",
        })
        self.assertEqual(app.magnetometer_tab.heading_routing, {
            "stream_choices": [("raw_heading", "Raw Heading"), ("method_1", "Half Scale")],
            "primary_stream_id": "method_1",
        })

    def test_metrics_refresh_populates_metrics_tab_snapshot(self) -> None:
        app = object.__new__(VirtualControllerApp)
        app._mag_methods = {}
        app._mag_selected_method_id = "method_1"
        app._mag_latest_derived_streams = {}
        app._mag_primary_heading_stream_id = "raw_heading"
        app._mag_last_live_snapshot = None
        app._mag_metrics_report = None
        app._mag_dataset = Dataset("metrics_case")
        app._mag_dataset.extend([
            SampleRecord("raw_magnetometer", "raw", "Raw Magnetometer", "builtin", 100, 110, 108, 1.0, 0.0, 0.0, 90.0, ""),
            SampleRecord("raw_magnetometer", "raw", "Raw Magnetometer", "builtin", 200, 210, 208, 0.0, 1.0, 0.0, 0.0, ""),
            SampleRecord("raw_magnetometer", "raw", "Raw Magnetometer", "builtin", 300, 310, 308, -1.0, 0.0, 0.0, 270.0, ""),
        ])
        app.magnetometer_tab = _DummyMagnetometerTab()

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
        plugin.calibration_runtime_s = 0.42
        app._mag_methods["method_1"] = plugin

        VirtualControllerApp._refresh_magnetometer_metrics_ui(app)

        assert app.magnetometer_tab.metrics_report is not None
        self.assertTrue(app.magnetometer_tab.metrics_report["can_export"])
        self.assertEqual(len(app.magnetometer_tab.metrics_report["rows"]), 6)
        self.assertIn("Metrics for metrics_case / Identity", app.magnetometer_tab.metrics_report["summary_text"])

    def test_realtime_process_error_disables_method_and_opens_diagnostics(self) -> None:
        app = object.__new__(VirtualControllerApp)
        app._mag_methods = {}
        app._mag_selected_method_id = "method_1"
        app._mag_latest_derived_streams = {}
        app._mag_datasets = []
        app._mag_dataset = None
        app._mag_recording_active = False
        app._mag_dataset_export_path = ""
        app._mag_sources = app._default_magnetometer_sources()
        app._mag_selected_source_id = "raw_magnetometer"
        app.magnetometer_tab = _DummyMagnetometerTab()
        opened: list[str] = []
        app._open_magnetometer_method_diagnostics = lambda method_id: opened.append(method_id)

        plugin = LoadedMethodPlugin(
            method_id="method_1",
            name="Exploding Realtime Method",
            version="1.0.0",
            file_path="/tmp/exploding_rt.py",
            info={
                "name": "Exploding Realtime Method",
                "version": "1.0.0",
                "type": "method",
                "supports_calibrate": True,
                "supports_load_params": True,
                "supports_save_params": True,
                "supports_process": True,
                "input_schema": {"mag": "xyz"},
                "output_schema": {"mag": "xyz"},
            },
            module=types.SimpleNamespace(process=lambda sample, params: (_ for _ in ()).throw(RuntimeError("boom"))),
            status="ready",
        )
        plugin.calibration_params = {
            "algorithm_name": "Exploding Realtime Method",
            "algorithm_version": "1.0.0",
            "schema_version": "1",
            "created_at": "2026-04-06T12:00:00Z",
            "params": {},
        }
        plugin.realtime_enabled = True
        plugin.show_enabled = True
        plugin.record_enabled = True
        app._mag_methods["method_1"] = plugin

        derived_streams, derived_records = VirtualControllerApp._process_magnetometer_methods_realtime(
            app,
            {
                "timestamp_mcu": 1000,
                "timestamp_pc_rx": 1010,
                "timestamp_pc_est": 1008,
                "mag_x": 1.0,
                "mag_y": 2.0,
                "mag_z": 3.0,
                "heading": 26.5,
                "flags": "",
            },
        )

        self.assertEqual(derived_streams, {})
        self.assertEqual(derived_records, [])
        self.assertEqual(plugin.status, "error")
        self.assertFalse(plugin.realtime_enabled)
        self.assertFalse(plugin.record_enabled)
        self.assertIsNotNone(plugin.diagnostics)
        self.assertEqual(opened, ["method_1"])


if __name__ == "__main__":
    unittest.main()
