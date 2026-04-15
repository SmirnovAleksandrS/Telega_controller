"""
Main GUI application:
- Menus: Files (start/stop log), Settings (COM settings)
- Tabs: Manual (implemented), Coordinate (placeholder)
- Right side panel with critical params, MCU time, radio quality, button deviation settings
- Serial worker + protocol handling + time sync
"""

from __future__ import annotations
import math
import queue
import threading
import time
import traceback
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from dataclasses import asdict
import struct
from typing import Optional, Any, Sequence
import collections
import json
import os
from datetime import datetime

from app.calibration_streams import (
    CalibrationStreamResolver,
    LatestStreamSnapshot,
    StreamProducerDescriptor,
    StreamRequirementDescriptor,
    SCOPE_CALIBRATE,
    SCOPE_PROCESS,
    infer_stream_requirements,
    infer_output_stream_kinds,
    make_builtin_producer_id,
    make_method_producer_id,
    select_default_binding,
    validate_method_bindings,
)
from app.accelerometer_dataset import AccelerometerDataset, AccelerometerSampleRecord
from app.accelerometer_tab import compute_accelerometer_tilt_deg
from app.gyroscope_dataset import GyroscopeDataset, GyroscopeSampleRecord
from app.gyroscope_tab import compute_gyro_rate_magnitude
from app.imu_dataset import ImuDataset, ImuSampleRecord
from app.manual_tab import ManualTab
from app.magnetometer_dataset import Dataset, SampleRecord
from app.magnetometer_metrics import MetricsReport, compute_metrics_report
from app.magnetometer_plugin_loader import (
    CalibrationRunResult,
    LoadedMethodPlugin,
    ParamIoResult,
    ProcessRunResult,
    PluginDiagnostics,
    load_method_params,
    load_method_plugin,
    run_method_calibration,
    run_method_process,
    save_method_params,
)
from app.magnetometer_tab import compute_raw_heading_deg
from app.coordinate_tab import CoordinateTab
from app.sensors_tab import SensorsTab
from app.dialogs import (
    AddPluginDialog,
    ComSettingsDialog,
    DeviationSettingsDialog,
    DeviationConfig,
    DatasetSelectionDialog,
    GeometrySettingsDialog,
    GeometryConfig,
    MethodDiagnosticsDialog,
    MethodInfoDialog,
    SpeedMapDialog,
    SpeedMapConfig,
    build_method_info_payload,
    format_method_info_text,
)
from app.styles import PANEL_BG, STATUS_GREEN, STATUS_RED, COLOR_GREEN, COLOR_YELLOW, COLOR_RED

from comm.serial_worker import SerialWorker, SerialMetrics, RxEvent, RxError
from comm.protocol import (
    build_sync_req, build_pid_req, build_control, build_set_motor_pid,
    TYPE_D0_IMU, TYPE_D1_TACHO,
    TYPE_B0_SYNC_REQ, TYPE_B1_PID_REQ, TYPE_C0_CONTROL, TYPE_A0_DISABLE_D, TYPE_A1_ENABLE_D, TYPE_A2_SET_PID,
    SOF, Frame, parse_frame,
    ImuData, TachoData, MotorData, SensorTensorData, SyncResp, MotorPidData
)
from comm.time_sync import TimeModel, compute_sync_point, control_timestamp_u32, estimate_initial, update_model, SyncPoint
from utils.timebase import now_ms_monotonic, u32
from runtime.contracts import (
    AutopilotMode,
    DriveCommand,
    ExternalRuntimeBridge,
    ExternalRuntimeState,
    ImuTelemetry,
    MissionConfig,
    MotorTelemetry,
    PoseSourceMode,
    SensorTensorTelemetry,
    TachoTelemetry,
    TelemetrySnapshot,
    TimeSyncState,
)
from runtime.socket_runtime import SocketExternalRuntime

from log.parsed_logger import ParsedLogger

class VirtualControllerApp:
    def __init__(self, *, fixed_route_canvases: bool = False) -> None:
        self.root = tk.Tk()
        self.root.title("Virtual controller")
        self.root.configure(bg=PANEL_BG)
        self.root.minsize(980, 560)
        self._is_shutting_down = False
        self._fixed_route_canvases = bool(fixed_route_canvases)

        self.worker = SerialWorker()
        self.worker.on_send = self._log_tx_raw
        self.logger = ParsedLogger()

        self.dev_cfg = DeviationConfig()
        self.geom_cfg = GeometryConfig()
        self.speed_cfg = SpeedMapConfig()
        self._test_mode_enabled = False
        self._manual_cfg = {"command_quantum_ms": 100}
        self._settings_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "app_settings.json"))
        self._mag_saved_state: dict[str, Any] = {}
        self._mag_file_dialog_dirs: dict[str, str] = {
            "dataset": "",
            "params": "",
            "plugin": "",
            "metrics": "",
        }
        self._load_settings()

        # Time sync state
        self.time_model = TimeModel()
        self._sync_seq = 0
        self._sync_pending: dict[int, int] = {}  # seq -> t1_pc_u32
        self._sync_points: list[SyncPoint] = []
        self._initial_sync_in_progress = False
        self._initial_sync_target = 20
        self._initial_sync_kbest = 5
        self._sync_beta = 0.05
        self._sync_period_ms = 10_000
        self._next_sync_due_ms = now_ms_monotonic() + 500  # shortly after start if connected
        self._pid_request_seq = 0
        self._pending_initial_pid_read = False
        self._motor_pid_values: tuple[float, float, float, float, float, float] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        # Control sending
        self._manual_neutral = 1500
        self._last_control = (self._manual_neutral, self._manual_neutral)
        self._control_period_ms = 100
        self._control_duration_ms = 100
        self._next_control_due_ms = now_ms_monotonic()
        self._active_tab = "Manual"
        self._kill_active = False
        self._kill_tick_ms = 200
        self._coord_running = False
        self._coord_tick_ms = 10
        self._mag_datasets: list[Dataset] = []
        self._mag_dataset: Optional[Dataset] = None
        self._mag_recording_active = False
        self._mag_dataset_export_path: str = ""
        self._mag_sources = self._default_magnetometer_sources()
        self._mag_selected_source_id = "raw_magnetometer"
        self._mag_methods: dict[str, LoadedMethodPlugin] = {}
        self._mag_selected_method_id: str | None = None
        self._mag_calibration_results: queue.Queue[tuple[str, CalibrationRunResult]] = queue.Queue()
        self._mag_calibration_jobs: dict[str, dict[str, Any]] = {}
        self._mag_latest_derived_streams: dict[str, dict[str, Any]] = {}
        self._mag_offline_method_clouds: dict[str, list[SampleRecord]] = {}
        self._calibration_stream_snapshots: dict[str, LatestStreamSnapshot] = {}
        self._mag_primary_heading_stream_id = "raw_heading"
        self._mag_last_live_snapshot: dict[str, Any] | None = None
        self._mag_metrics_report: MetricsReport | None = None
        self._mag_last_dataset_ui_refresh_s = 0.0
        self._mag_dataset_ui_refresh_interval_s = 0.2
        self._imu_datasets: list[ImuDataset] = []
        self._imu_dataset: Optional[ImuDataset] = None
        self._imu_recording_active = False
        self._imu_dataset_export_path: str = ""
        self._acc_datasets: list[AccelerometerDataset] = []
        self._acc_dataset: Optional[AccelerometerDataset] = None
        self._acc_recording_active = False
        self._acc_dataset_export_path: str = ""
        self._acc_sources = self._default_accelerometer_sources()
        self._acc_selected_source_id = "raw_accelerometer"
        self._acc_methods: dict[str, LoadedMethodPlugin] = {}
        self._acc_selected_method_id: str | None = None
        self._acc_calibration_results: queue.Queue[tuple[str, CalibrationRunResult]] = queue.Queue()
        self._acc_calibration_jobs: dict[str, dict[str, Any]] = {}
        self._acc_latest_derived_streams: dict[str, dict[str, Any]] = {}
        self._acc_offline_method_clouds: dict[str, list[AccelerometerSampleRecord]] = {}
        self._acc_calibration_stream_snapshots: dict[str, LatestStreamSnapshot] = {}
        self._acc_primary_output_stream_id = "raw_tilt"
        self._acc_last_live_snapshot: dict[str, Any] | None = None
        self._acc_saved_state: dict[str, Any] = {}
        self._acc_file_dialog_dirs: dict[str, str] = {"dataset": "", "params": "", "plugin": "", "metrics": ""}
        self._acc_last_dataset_ui_refresh_s = 0.0
        self._gyro_datasets: list[GyroscopeDataset] = []
        self._gyro_dataset: Optional[GyroscopeDataset] = None
        self._gyro_recording_active = False
        self._gyro_dataset_export_path: str = ""
        self._gyro_sources = self._default_gyroscope_sources()
        self._gyro_selected_source_id = "raw_gyroscope"
        self._gyro_methods: dict[str, LoadedMethodPlugin] = {}
        self._gyro_selected_method_id: str | None = None
        self._gyro_calibration_results: queue.Queue[tuple[str, CalibrationRunResult]] = queue.Queue()
        self._gyro_calibration_jobs: dict[str, dict[str, Any]] = {}
        self._gyro_latest_derived_streams: dict[str, dict[str, Any]] = {}
        self._gyro_calibration_stream_snapshots: dict[str, LatestStreamSnapshot] = {}
        self._gyro_primary_output_stream_id = "raw_gyroscope"
        self._gyro_last_live_snapshot: dict[str, Any] | None = None
        self._gyro_saved_state: dict[str, Any] = {}
        self._gyro_file_dialog_dirs: dict[str, str] = {"dataset": "", "params": "", "plugin": "", "metrics": ""}
        self._gyro_last_dataset_ui_refresh_s = 0.0
        self._set_manual_command_quantum_ms(self._manual_cfg["command_quantum_ms"], persist=False)

        # Radio quality (simple heuristic)
        self._rx_ok = collections.deque(maxlen=200)
        self._last_mcu_ts_u32: Optional[int] = None
        self._next_uart_status_due_ms = now_ms_monotonic()
        self._last_imu: Optional[ImuData] = None
        self._last_tacho: Optional[TachoData] = None
        self._last_motor: Optional[MotorData] = None
        self._last_d3: Optional[SensorTensorData] = None
        self._coord_mission: Optional[MissionConfig] = None
        self._external_runtime: Optional[ExternalRuntimeBridge] = None
        self._last_external_command: Optional[DriveCommand] = None
        self.set_external_runtime(self._build_default_external_runtime())

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self._on_exit)

        # periodic queue polling
        self.root.after(20, self._poll_rx)

    def run(self) -> None:
        try:
            self.root.mainloop()
        finally:
            self.shutdown()

    def set_external_runtime(self, runtime: Optional[ExternalRuntimeBridge]) -> None:
        previous_runtime = self._external_runtime
        if previous_runtime is not None and previous_runtime is not runtime:
            try:
                previous_runtime.stop()
            except Exception:
                pass
        self._external_runtime = runtime
        self._last_external_command = None
        if runtime is None and hasattr(self, "coord_tab"):
            self.coord_tab.set_external_pose(None)

    def _build_default_external_runtime(self) -> ExternalRuntimeBridge:
        host = (
            os.environ.get("TELEGA_CPP_RUNTIME_HOST")
            or os.environ.get("TELEGA_CPP_AUTOPILOT_HOST")
            or "127.0.0.1"
        ).strip() or "127.0.0.1"
        try:
            port = int(
                os.environ.get("TELEGA_CPP_RUNTIME_PORT")
                or os.environ.get("TELEGA_CPP_AUTOPILOT_PORT")
                or "8765"
            )
        except (TypeError, ValueError):
            port = 8765
        return SocketExternalRuntime(host=host, port=port)

    def _stop_external_runtime(self) -> None:
        self._last_external_command = None
        if self._external_runtime is None:
            return
        try:
            self._external_runtime.stop()
        except Exception:
            pass

    def _shutdown_external_runtime(self) -> None:
        self._last_external_command = None
        if self._external_runtime is None:
            return
        shutdown_server = getattr(self._external_runtime, "shutdown_server", None)
        try:
            if callable(shutdown_server):
                shutdown_server()
            else:
                self._external_runtime.stop()
        except Exception:
            pass

    # ---------------- UI ----------------

    def _build_ui(self) -> None:
        self._build_menu()

        # Top tabs + main area
        main = ttk.Frame(self.root, padding=6)
        main.pack(fill="both", expand=True)

        self.nb = ttk.Notebook(main)
        self.nb.pack(side="left", fill="both", expand=True)

        self.manual_tab = ManualTab(
            self.nb,
            on_control=self._on_manual_control,
            on_coeffs_change=self._on_manual_coeffs,
            on_command_quantum_change=self._on_manual_command_quantum_change,
            on_pid_change=self._on_manual_pid_change,
            on_pid_send=self._on_manual_pid_send,
            on_pid_read=self._on_manual_pid_read,
        )
        self.nb.add(self.manual_tab, text="Manual")
        self._apply_joystick_settings()
        self._apply_manual_settings()

        self.coord_tab = CoordinateTab(
            self.nb,
            on_start=self._on_coord_start,
            on_stop=self._on_coord_stop,
            on_state_change=self._save_settings,
            dynamic_route_views=not self._fixed_route_canvases,
        )
        self.nb.add(self.coord_tab, text="Coordinate")
        self.sensors_tab = SensorsTab(
            self.nb,
            on_start_record=self._on_magnetometer_start_record,
            on_stop_record=self._on_magnetometer_stop_record,
            on_load_csv=self._on_magnetometer_load_csv,
            on_load_multiple=self._on_magnetometer_load_multiple,
            on_save_current=self._on_magnetometer_save_current,
            on_save_as=self._on_magnetometer_save_as,
            on_clear_dataset=self._on_clear_magnetometer_datasets,
            on_concatenate=self._on_magnetometer_concatenate,
            on_trim_selection=self._on_magnetometer_trim_selection,
            on_delete_selection=self._on_magnetometer_delete_selection,
            on_select_dataset=self._on_select_magnetometer_dataset,
            on_select_source=self._on_select_magnetometer_source,
            on_source_show_change=self._on_toggle_magnetometer_source_show,
            on_source_record_change=self._on_toggle_magnetometer_source_record,
            on_add_plugin=self._on_magnetometer_add_plugin,
            on_select_method=self._on_select_magnetometer_method,
            on_open_method_info=self._on_open_magnetometer_method_info,
            on_calibrate_method=self._on_calibrate_magnetometer_method,
            on_load_method_params=self._on_load_magnetometer_method_params,
            on_save_method_params=self._on_save_magnetometer_method_params,
            on_clear_method_params=self._on_clear_magnetometer_method_params,
            on_method_show_change=self._on_toggle_magnetometer_method_show,
            on_remove_method=self._on_remove_magnetometer_method,
            on_enable_method_realtime=self._on_enable_magnetometer_method_realtime,
            on_disable_method_realtime=self._on_disable_magnetometer_method_realtime,
            on_method_record_change=self._on_toggle_magnetometer_method_record,
            on_select_primary_heading=self._on_select_magnetometer_primary_heading,
            on_export_metrics=self._on_magnetometer_export_metrics_csv,
            accelerometer_callbacks={
                "on_start_record": self._on_accelerometer_start_record,
                "on_stop_record": self._on_accelerometer_stop_record,
                "on_load_csv": self._on_accelerometer_load_csv,
                "on_load_multiple": self._on_accelerometer_load_multiple,
                "on_save_current": self._on_accelerometer_save_current,
                "on_save_as": self._on_accelerometer_save_as,
                "on_clear_dataset": self._on_clear_accelerometer_datasets,
                "on_concatenate": self._on_accelerometer_concatenate,
                "on_trim_selection": self._on_accelerometer_trim_selection,
                "on_delete_selection": self._on_accelerometer_delete_selection,
                "on_select_dataset": self._on_select_accelerometer_dataset,
                "on_select_source": self._on_select_accelerometer_source,
                "on_source_show_change": self._on_toggle_accelerometer_source_show,
                "on_source_record_change": self._on_toggle_accelerometer_source_record,
                "on_add_plugin": self._on_accelerometer_add_plugin,
                "on_select_method": self._on_select_accelerometer_method,
                "on_open_method_info": self._on_open_accelerometer_method_info,
                "on_calibrate_method": self._on_calibrate_accelerometer_method,
                "on_load_method_params": self._on_load_accelerometer_method_params,
                "on_save_method_params": self._on_save_accelerometer_method_params,
                "on_clear_method_params": self._on_clear_accelerometer_method_params,
                "on_method_show_change": self._on_toggle_accelerometer_method_show,
                "on_remove_method": self._on_remove_accelerometer_method,
                "on_enable_method_realtime": self._on_enable_accelerometer_method_realtime,
                "on_disable_method_realtime": self._on_disable_accelerometer_method_realtime,
                "on_method_record_change": self._on_toggle_accelerometer_method_record,
                "on_select_primary_output": self._on_select_accelerometer_primary_output,
            },
            gyroscope_callbacks={
                "on_start_record": self._on_gyroscope_start_record,
                "on_stop_record": self._on_gyroscope_stop_record,
                "on_load_csv": self._on_gyroscope_load_csv,
                "on_load_multiple": self._on_gyroscope_load_multiple,
                "on_save_current": self._on_gyroscope_save_current,
                "on_save_as": self._on_gyroscope_save_as,
                "on_clear_dataset": self._on_clear_gyroscope_datasets,
                "on_concatenate": self._on_gyroscope_concatenate,
                "on_trim_selection": self._on_gyroscope_trim_selection,
                "on_delete_selection": self._on_gyroscope_delete_selection,
                "on_select_dataset": self._on_select_gyroscope_dataset,
                "on_select_source": self._on_select_gyroscope_source,
                "on_source_show_change": self._on_toggle_gyroscope_source_show,
                "on_source_record_change": self._on_toggle_gyroscope_source_record,
                "on_add_plugin": self._on_gyroscope_add_plugin,
                "on_select_method": self._on_select_gyroscope_method,
                "on_open_method_info": self._on_open_gyroscope_method_info,
                "on_calibrate_method": self._on_calibrate_gyroscope_method,
                "on_load_method_params": self._on_load_gyroscope_method_params,
                "on_save_method_params": self._on_save_gyroscope_method_params,
                "on_clear_method_params": self._on_clear_gyroscope_method_params,
                "on_method_show_change": self._on_toggle_gyroscope_method_show,
                "on_remove_method": self._on_remove_gyroscope_method,
                "on_enable_method_realtime": self._on_enable_gyroscope_method_realtime,
                "on_disable_method_realtime": self._on_disable_gyroscope_method_realtime,
                "on_method_record_change": self._on_toggle_gyroscope_method_record,
                "on_select_primary_output": self._on_select_gyroscope_primary_output,
            },
        )
        self.magnetometer_tab = self.sensors_tab.magnetometer_tab
        self.accelerometer_tab = self.sensors_tab.accelerometer_tab
        self.gyroscope_tab = self.sensors_tab.gyroscope_tab
        self.nb.add(self.sensors_tab, text="Sensors")
        self._refresh_magnetometer_sources_ui()
        self._refresh_magnetometer_methods_ui()
        self._refresh_magnetometer_dataset_ui()
        self._refresh_magnetometer_heading_routing_ui()
        self._refresh_magnetometer_metrics_ui()
        self._refresh_accelerometer_sources_ui()
        self._refresh_accelerometer_methods_ui()
        self._refresh_accelerometer_dataset_ui()
        self._refresh_accelerometer_output_routing_ui()
        self._refresh_gyroscope_sources_ui()
        self._refresh_gyroscope_methods_ui()
        self._refresh_gyroscope_dataset_ui()
        self._refresh_gyroscope_output_routing_ui()
        self._restore_magnetometer_settings()
        self._restore_accelerometer_settings()
        self._restore_gyroscope_settings()
        self._apply_geometry_settings()
        self._apply_speed_map_settings()
        if hasattr(self, "_coord_state"):
            self.coord_tab.set_state(self._coord_state)
        self.nb.bind("<<NotebookTabChanged>>", self._on_tab_change)

        # Right panel
        right = ttk.Frame(main, width=280, padding=(10, 0, 0, 0))
        right.pack(side="right", fill="y")

        self._build_right_panel(right)

    def _build_menu(self) -> None:
        menubar = tk.Menu(self.root)

        menu_files = tk.Menu(menubar, tearoff=0)
        menu_files.add_command(label="Log path", command=self._choose_log_path)
        menu_files.add_command(label="Start logging", command=self._start_logging)
        menu_files.add_command(label="Stop logging", command=self._stop_logging)
        menu_files.add_separator()
        menu_files.add_command(label="Exit", command=self._on_exit)
        menubar.add_cascade(label="Files", menu=menu_files)

        menu_settings = tk.Menu(menubar, tearoff=0)
        menu_settings.add_command(label="COM Settings", command=self._open_com_settings)
        menubar.add_cascade(label="Settings", menu=menu_settings)

        menu_physics = tk.Menu(menubar, tearoff=0)
        menu_physics.add_command(label="Set geometry parameters", command=self._open_geometry_settings)
        menu_physics.add_command(label="Speed parameters", command=self._open_speed_settings)
        menubar.add_cascade(label="Physics", menu=menu_physics)

        self.root.config(menu=menubar)

        self._log_path: str = ""

    def _build_right_panel(self, parent: tk.Widget) -> None:
        # Top row: quick connect + log status
        top = ttk.Frame(parent)
        top.pack(fill="x")

        self.connect_btn = ttk.Button(top, text="Connect", command=self._toggle_connect)
        self.connect_btn.pack(side="left")
        self._test_mode_var = tk.BooleanVar(value=self._test_mode_enabled)
        self.test_mode_chk = ttk.Checkbutton(
            top,
            text="Test mode",
            variable=self._test_mode_var,
            command=self._on_test_mode_toggle,
        )
        self.test_mode_chk.pack(side="left", padx=(8, 0))

        self.log_status_var = tk.StringVar(value="Log Stopped")
        self.log_status_lbl = ttk.Label(top, textvariable=self.log_status_var)
        self.log_status_lbl.pack(side="right")
        self._apply_log_status_style()
        self._refresh_connect_btn()

        # Internal references to value labels (to recolor)
        self._value_labels: dict[str, tuple[ttk.Label, ttk.Label]] = {}

        # Critical parameters box
        crit = ttk.LabelFrame(parent, text="Critical Parameters", padding=8)
        crit.pack(fill="x", pady=(10, 10))

        # Grid header
        ttk.Label(crit, text="").grid(row=0, column=0, sticky="w")
        ttk.Label(crit, text="Left").grid(row=0, column=1, sticky="e", padx=(10, 10))
        ttk.Label(crit, text="Right").grid(row=0, column=2, sticky="e")

        self.volt_l = tk.StringVar(value="—")
        self.volt_r = tk.StringVar(value="—")
        self.cur_l = tk.StringVar(value="—")
        self.cur_r = tk.StringVar(value="—")
        self.temp_l = tk.StringVar(value="—")
        self.temp_r = tk.StringVar(value="—")
        self.speed_l = tk.StringVar(value="—")
        self.speed_r = tk.StringVar(value="—")

        self._make_param_row(crit, 1, "Voltage", self.volt_l, self.volt_r)
        self._make_param_row(crit, 2, "Current", self.cur_l, self.cur_r)
        self._make_param_row(crit, 3, "Temperature", self.temp_l, self.temp_r)
        self._make_param_row(crit, 4, "Speed", self.speed_l, self.speed_r)

        # MCU Time
        self.mcu_time_var = tk.StringVar(value="MCU Time:\nrx=—\nest=—")
        ttk.Label(parent, textvariable=self.mcu_time_var).pack(anchor="w", pady=(6, 0))

        # Radio Quality
        self.radio_var = tk.StringVar(value="Radio Quality: —/10")
        ttk.Label(parent, textvariable=self.radio_var).pack(anchor="w", pady=(6, 0))

        self.uart_status_var = tk.StringVar(value="UART:\nstate=CLOSED load=0%")
        self.uart_status_lbl = tk.Label(
            parent,
            textvariable=self.uart_status_var,
            justify="left",
            anchor="w",
            bg=PANEL_BG,
            fg=STATUS_RED,
        )
        self.uart_status_lbl.pack(anchor="w", pady=(6, 0), fill="x")

        # Deviation settings button
        ttk.Button(parent, text="Deviation Settings", command=self._open_deviation_settings).pack(fill="x", pady=(14, 0))
        ttk.Button(parent, text="Time Sync", command=self._begin_initial_sync).pack(fill="x", pady=(8, 0))
        self.kill_btn = ttk.Button(parent, text="KILL SWITCH", command=self._kill_switch)
        self.kill_btn.pack(fill="x", pady=(12, 0))
        self._apply_kill_style()
        self._update_uart_status(force=True)

    def _make_param_row(self, parent: tk.Widget, row: int, name: str, var_l: tk.StringVar, var_r: tk.StringVar) -> None:
        ttk.Label(parent, text=name).grid(row=row, column=0, sticky="w", pady=4)
        lbl_l = ttk.Label(parent, textvariable=var_l, font=("TkDefaultFont", 11, "bold"))
        lbl_r = ttk.Label(parent, textvariable=var_r, font=("TkDefaultFont", 11, "bold"))
        lbl_l.grid(row=row, column=1, sticky="e", padx=(10, 10))
        lbl_r.grid(row=row, column=2, sticky="e")
        self._value_labels[name] = (lbl_l, lbl_r)

    def _apply_log_status_style(self) -> None:
        # ttk doesn't directly support foreground change per-widget in a portable way without styles.
        # simplest: use a normal tk.Label for color
        # We'll replace once on first call.
        if isinstance(self.log_status_lbl, ttk.Label):
            parent = self.log_status_lbl.master
            self.log_status_lbl.destroy()
            self.log_status_lbl = tk.Label(parent, textvariable=self.log_status_var, bg=PANEL_BG)
            self.log_status_lbl.pack(anchor="ne")
        is_run = self.logger.is_running
        self.log_status_lbl.configure(fg=STATUS_GREEN if is_run else STATUS_RED)

    def _show_error(self, title: str, message: str) -> None:
        parent = getattr(self, "root", None)
        kwargs = {"parent": parent} if parent is not None else {}
        messagebox.showerror(title, message, **kwargs)

    def _show_warning(self, title: str, message: str) -> None:
        parent = getattr(self, "root", None)
        kwargs = {"parent": parent} if parent is not None else {}
        messagebox.showwarning(title, message, **kwargs)

    def _show_info(self, title: str, message: str) -> None:
        parent = getattr(self, "root", None)
        kwargs = {"parent": parent} if parent is not None else {}
        messagebox.showinfo(title, message, **kwargs)

    def _remember_dialog_dir(self, kind: str, path: str) -> None:
        if not path:
            return
        if not hasattr(self, "_mag_file_dialog_dirs"):
            self._mag_file_dialog_dirs = {"dataset": "", "params": "", "plugin": "", "metrics": ""}
        directory = path if os.path.isdir(path) else os.path.dirname(path)
        if directory:
            self._mag_file_dialog_dirs[kind] = directory

    def _dialog_initialdir(self, kind: str, *, fallback_path: str | None = None) -> str | None:
        if hasattr(self, "_mag_file_dialog_dirs"):
            candidate = self._mag_file_dialog_dirs.get(kind, "")
            if candidate and os.path.isdir(candidate):
                return candidate
        if fallback_path:
            candidate = fallback_path if os.path.isdir(fallback_path) else os.path.dirname(fallback_path)
            if candidate and os.path.isdir(candidate):
                return candidate
        return None

    # ---------------- Menu actions ----------------

    def _choose_log_path(self) -> None:
        path = filedialog.asksaveasfilename(
            title="Select log file",
            defaultextension=".log",
            filetypes=[("Log files", "*.log"), ("All files", "*.*")],
            parent=self.root,
        )
        if path:
            self._log_path = path

    def _start_logging(self) -> None:
        if self.logger.is_running:
            return
        if not self._log_path:
            self._choose_log_path()
            if not self._log_path:
                return
        try:
            self.logger.start(self._log_path)
        except Exception as e:
            self._show_error("Logging", f"Failed to start log: {e}")
            return
        self.log_status_var.set("Log Running")
        self._apply_log_status_style()

    def _stop_logging(self) -> None:
        self.logger.stop()
        self.log_status_var.set("Log Stopped")
        self._apply_log_status_style()

    def _open_com_settings(self) -> None:
        dlg = ComSettingsDialog(self.root, self.worker.port, self.worker.baud)
        self.root.wait_window(dlg)
        if dlg.result:
            port, baud = dlg.result
            self.worker.open(port, baud)
            if self.worker.is_open:
                self._save_settings()
                # Start initial sync when port opens
                self._pending_initial_pid_read = True
                self._begin_initial_sync()
            else:
                self._show_serial_error("COM Settings", "Failed to open selected port")
        self._update_uart_status(force=True)

    def _open_deviation_settings(self) -> None:
        dlg = DeviationSettingsDialog(self.root, self.dev_cfg)
        self.root.wait_window(dlg)
        if dlg.result:
            self.dev_cfg = dlg.result
            # Refresh coloring with last known values
            self._recolor_all()
            self._save_settings()

    def _open_geometry_settings(self) -> None:
        dlg = GeometrySettingsDialog(self.root, self.geom_cfg)
        self.root.wait_window(dlg)
        if dlg.result:
            self.geom_cfg = dlg.result
            self._apply_geometry_settings()
            self._save_settings()

    def _open_speed_settings(self) -> None:
        dlg = SpeedMapDialog(self.root, self.speed_cfg)
        self.root.wait_window(dlg)
        if dlg.result:
            self.speed_cfg = dlg.result
            self._apply_speed_map_settings()
            self._save_settings()

    def _on_exit(self) -> None:
        self.shutdown()

    def shutdown(self) -> None:
        if self._is_shutting_down:
            return
        self._is_shutting_down = True
        self._save_settings()
        self._shutdown_external_runtime()
        try:
            self.worker.close()
        finally:
            self.logger.stop()
            try:
                self.root.quit()
            except Exception:
                pass
            try:
                self.root.destroy()
            except Exception:
                pass

    def _control_timestamp_u32(self, now_ms: Optional[int] = None) -> int:
        pc_now_ms = now_ms_monotonic() if now_ms is None else int(now_ms)
        return control_timestamp_u32(self.time_model, pc_now_ms)

    def _normalize_manual_command_quantum_ms(self, value: int) -> int:
        return max(1, min(60_000, int(value)))

    def _set_manual_command_quantum_ms(self, value: int, *, persist: bool) -> None:
        normalized = self._normalize_manual_command_quantum_ms(value)
        self._manual_cfg["command_quantum_ms"] = normalized
        self._control_period_ms = normalized
        self._control_duration_ms = normalized
        self._next_control_due_ms = now_ms_monotonic()
        if hasattr(self, "manual_tab"):
            self.manual_tab.set_command_quantum_ms(normalized)
        if persist:
            self._save_settings()

    def build_telemetry_snapshot(self, now_ms: Optional[int] = None) -> TelemetrySnapshot:
        pc_now_ms = now_ms_monotonic() if now_ms is None else int(now_ms)
        mcu_est_ms = None
        if self.time_model.have_lock:
            mcu_est_ms = self._control_timestamp_u32(pc_now_ms)
        sync = TimeSyncState(
            pc_time_ms=pc_now_ms,
            have_lock=self.time_model.have_lock,
            scale_a=self.time_model.a,
            offset_b=self.time_model.b,
            mcu_rx_ms=self._last_mcu_ts_u32,
            mcu_est_ms=mcu_est_ms,
        )
        imu = None
        if self._last_imu is not None:
            imu = ImuTelemetry(
                ts_ms=self._last_imu.ts_ms,
                accel=self._last_imu.accel,
                magn=self._last_imu.magn,
                gyro=self._last_imu.gyro,
            )
        tacho = None
        if self._last_tacho is not None:
            tacho = TachoTelemetry(
                ts_ms=self._last_tacho.ts_ms,
                left_rpm=self._last_tacho.left_rpm,
                right_rpm=self._last_tacho.right_rpm,
            )
        motor = None
        if self._last_motor is not None:
            motor = MotorTelemetry(
                ts_ms=self._last_motor.ts_ms,
                current_l=self._last_motor.current_l,
                current_r=self._last_motor.current_r,
                voltage_l=self._last_motor.voltage_l,
                voltage_r=self._last_motor.voltage_r,
                temp_l=self._last_motor.temp_l,
                temp_r=self._last_motor.temp_r,
            )
        sensor_tensor = None
        if self._last_d3 is not None:
            sensor_tensor = SensorTensorTelemetry(
                ts_ms=self._last_d3.ts_ms,
                linear_velocity=self._last_d3.linear_velocity,
                angular_velocity=self._last_d3.angular_velocity,
                linear_quality=self._last_d3.linear_quality,
                angular_quality=self._last_d3.angular_quality,
            )
        return TelemetrySnapshot(sync=sync, imu=imu, tacho=tacho, motor=motor, sensor_tensor=sensor_tensor)

    def _mission_uses_external_runtime(self, mission: Optional[MissionConfig] = None) -> bool:
        active_mission = self._coord_mission if mission is None else mission
        return active_mission is not None and (
            active_mission.pose_source == PoseSourceMode.EXTERNAL
            or active_mission.autopilot == AutopilotMode.EXTERNAL
        )

    def _apply_external_runtime_state(self, state: ExternalRuntimeState) -> None:
        if state.pose is not None:
            self.coord_tab.set_external_pose(state.pose)
        if state.drive_command is not None:
            self._last_external_command = state.drive_command

    def _poll_external_runtime_state(self) -> ExternalRuntimeState:
        if self._external_runtime is None:
            return ExternalRuntimeState()
        try:
            state = self._external_runtime.poll_state()
        except Exception as exc:
            self._handle_external_runtime_failure(exc)
            return ExternalRuntimeState()
        self._apply_external_runtime_state(state)
        return state

    def _ingest_external_runtime_snapshot(self, snapshot: TelemetrySnapshot) -> None:
        if not self._coord_running or not self._mission_uses_external_runtime():
            return
        if self._external_runtime is None:
            return
        try:
            self._external_runtime.ingest_telemetry(snapshot)
            state = self._external_runtime.poll_state()
        except Exception as exc:
            self._handle_external_runtime_failure(exc)
            return
        self._apply_external_runtime_state(state)

    def _handle_external_runtime_failure(self, exc: Exception) -> None:
        self._stop_external_runtime()
        if not self._coord_running:
            return
        self._coord_running = False
        self._coord_mission = None
        self.coord_tab.set_running(False)
        self.coord_tab.stop_expected()
        self._send_coordinate_stop()
        self._show_error("Coordinate", f"External runtime failed:\n{exc}")

    def _toggle_connect(self) -> None:
        if self.worker.is_open:
            self.worker.close()
            self._pending_initial_pid_read = False
            if self._coord_running:
                self._coord_running = False
                self._coord_mission = None
                self._stop_external_runtime()
                self.coord_tab.set_running(False)
                self.coord_tab.stop_expected()
            self._update_uart_status(force=True)
            return

        if self.worker.port:
            ports = set(SerialWorker.list_ports())
            if self.worker.port not in ports:
                self._show_error(
                    "COM Settings",
                    f"Saved port '{self.worker.port}' is not available.\nSelect a port in COM Settings.",
                )
                self._open_com_settings()
                self._update_uart_status(force=True)
                return
            self.worker.open(self.worker.port, self.worker.baud)
            if self.worker.is_open:
                self._pending_initial_pid_read = True
                self._begin_initial_sync()
            else:
                self._show_serial_error("COM Settings", "Failed to open port")
        else:
            self._open_com_settings()
        self._update_uart_status(force=True)

    def _refresh_connect_btn(self, metrics: Optional[SerialMetrics] = None) -> None:
        if metrics is None:
            metrics = self.worker.get_metrics()
        state = metrics.state
        if state == "open":
            self.connect_btn.configure(text="Disconnect", state="normal")
        elif state == "opening":
            self.connect_btn.configure(text="Opening...", state="disabled")
        elif state == "closing":
            self.connect_btn.configure(text="Closing...", state="disabled")
        else:
            self.connect_btn.configure(text="Connect", state="normal")

    def _show_serial_error(self, title: str, fallback: str) -> None:
        metrics = self.worker.get_metrics()
        error = metrics.last_error.strip() or fallback
        self._show_error(title, error)

    def _update_uart_status(self, *, force: bool = False) -> None:
        if not hasattr(self, "uart_status_var"):
            return
        now_ms = now_ms_monotonic()
        if not force and now_ms < self._next_uart_status_due_ms:
            return
        self._next_uart_status_due_ms = now_ms + 250
        metrics = self.worker.get_metrics()

        tx_kib_s = metrics.tx_rate_Bps / 1024.0
        rx_kib_s = metrics.rx_rate_Bps / 1024.0
        text = (
            f"UART:\nstate={metrics.state.upper()} load={metrics.peak_load_pct:.0f}% "
            f"(tx={metrics.tx_load_pct:.0f}% rx={metrics.rx_load_pct:.0f}%)"
            f"\ntx={tx_kib_s:.1f} KiB/s q={metrics.tx_queue_depth} drop={metrics.tx_dropped}"
            f"\nrx={rx_kib_s:.1f} KiB/s q={metrics.rx_queue_depth} drop={metrics.rx_dropped}"
            f"\nerr={metrics.error_count}"
        )
        if metrics.last_error:
            compact_error = metrics.last_error.replace("\n", " ").strip()
            if len(compact_error) > 80:
                compact_error = compact_error[:77] + "..."
            text = f"{text}\nlast={compact_error}"
        self.uart_status_var.set(text)

        if metrics.state == "open":
            color = STATUS_GREEN
        elif metrics.state in {"opening", "closing"}:
            color = COLOR_YELLOW
        else:
            color = STATUS_RED
        self.uart_status_lbl.configure(fg=color)
        self._refresh_connect_btn(metrics)

    def _is_test_mode(self) -> bool:
        if hasattr(self, "_test_mode_var"):
            return bool(self._test_mode_var.get())
        return bool(self._test_mode_enabled)

    def _on_test_mode_toggle(self) -> None:
        self._test_mode_enabled = self._is_test_mode()
        self._save_settings()

    def _apply_kill_style(self) -> None:
        style = ttk.Style(self.root)
        style.configure("KillOff.TButton", foreground="#ffffff", background="#cc0000", padding=8)
        style.map("KillOff.TButton",
                  foreground=[("active", "#ffffff")],
                  background=[("active", "#aa0000")])
        style.configure("KillOn.TButton", foreground="#ffffff", background="#550000", padding=8)
        style.map("KillOn.TButton",
                  foreground=[("active", "#ffffff")],
                  background=[("active", "#770000")])
        if hasattr(self, "kill_btn"):
            self._update_kill_indicator()

    def _on_coord_start(self) -> None:
        if not self.worker.is_open and not self._is_test_mode():
            self._show_error("Coordinate", "Connect COM port before start")
            return
        if self._kill_active:
            self._show_error("Coordinate", "Kill switch is active")
            return
        if self._coord_running:
            return
        self.coord_tab.refresh_profile()
        mission = self.coord_tab.build_mission_config()
        if len(mission.track_points) < 2:
            self._show_error("Coordinate", "No valid trajectory for controller")
            return
        if self._mission_uses_external_runtime(mission) and self._external_runtime is None:
            self._show_error("Coordinate", "External runtime is selected, but no external runtime is attached")
            return
        if self.worker.is_open and not self._is_test_mode():
            self._begin_initial_sync()
        self._coord_tick_ms = max(1, self.coord_tab.get_time_quantum_ms())
        self.coord_tab.reset_actual_trace()
        self._last_external_command = None
        self.coord_tab.set_external_pose(None)
        if self._mission_uses_external_runtime(mission):
            try:
                assert self._external_runtime is not None
                self._external_runtime.apply_mission(mission)
                self._external_runtime.reset()
            except Exception as exc:
                self._show_error("Coordinate", f"Failed to initialize external runtime:\n{exc}")
                return
        if mission.autopilot == AutopilotMode.EXTERNAL:
            self.coord_tab.reset_expected_pose()
        else:
            if not self.coord_tab.start_controller():
                self._show_error("Coordinate", "No valid trajectory for controller")
                return
            self.coord_tab.start_expected(now_ms_monotonic())
        self._coord_mission = mission
        self._coord_running = True
        self.coord_tab.set_running(True)
        self._coord_send_tick()

    def _on_coord_stop(self) -> None:
        self._stop_external_runtime()
        if not self._coord_running:
            self.coord_tab.set_running(False)
            self.coord_tab.stop_expected()
            self._send_coordinate_stop()
            return
        self._coord_running = False
        self._coord_mission = None
        self.coord_tab.set_running(False)
        self.coord_tab.stop_expected()
        self._send_coordinate_stop()

    def _send_coordinate_stop(self) -> None:
        if not self.worker.is_open:
            return
        ts_u32 = self._control_timestamp_u32()
        pkt = build_control(ts_u32, self._manual_neutral, self._manual_neutral, self._control_duration_ms)
        self.worker.send(pkt)

    def _coord_send_tick(self) -> None:
        if not self._coord_running or self._kill_active:
            return
        test_mode = self._is_test_mode()
        if not self.worker.is_open and not test_mode:
            self._coord_running = False
            self._coord_mission = None
            self._stop_external_runtime()
            self.coord_tab.set_running(False)
            self.coord_tab.stop_expected()
            return
        dt_s = max(1, self._coord_tick_ms) / 1000.0
        autopilot_mode = self.coord_tab.get_autopilot_mode()
        command_duration_ms = self._coord_tick_ms
        runtime_state = self._poll_external_runtime_state() if self._mission_uses_external_runtime() else ExternalRuntimeState()
        if not self._coord_running:
            return
        if autopilot_mode == AutopilotMode.EXTERNAL:
            cmd_obj = self._last_external_command
            finished = runtime_state.finished
            if cmd_obj is None:
                cmd_obj = DriveCommand(
                    left_pwm=self._manual_neutral,
                    right_pwm=self._manual_neutral,
                    duration_ms=self._coord_tick_ms,
                    source="external-neutral",
                    created_pc_ms=now_ms_monotonic(),
                )
            self.coord_tab.apply_drive_command(cmd_obj, dt_s)
            left_cmd, right_cmd = cmd_obj.left_pwm, cmd_obj.right_pwm
            command_duration_ms = max(1, int(cmd_obj.duration_ms))
        else:
            cmd = self.coord_tab.step_controller(dt_s, force_internal_pose=test_mode)
            finished = self.coord_tab.controller_finished()
            if cmd is None:
                left_cmd, right_cmd = self._manual_neutral, self._manual_neutral
            else:
                left_cmd, right_cmd = cmd
        if finished:
            self._coord_running = False
            self._stop_external_runtime()
            self.coord_tab.set_running(False)
            self.coord_tab.stop_expected()
            self._send_coordinate_stop()
            return
        if self.worker.is_open and not test_mode:
            payload = build_control(self._control_timestamp_u32(), left_cmd, right_cmd, command_duration_ms)
            self.worker.send(payload)
        self.root.after(self._coord_tick_ms, self._coord_send_tick)

    # ---------------- Control logic ----------------

    def _on_manual_control(self, left_cmd: int, right_cmd: int) -> None:
        self._last_control = (left_cmd, right_cmd)

    def _on_manual_coeffs(self, left_shift: float, right_shift: float, left_linear: float, right_linear: float) -> None:
        self._joystick_cfg = {
            "left_shift": left_shift,
            "right_shift": right_shift,
            "left_linear": left_linear,
            "right_linear": right_linear,
        }
        self._save_settings()

    def _on_manual_command_quantum_change(self, value: int) -> None:
        self._set_manual_command_quantum_ms(value, persist=True)

    def _on_manual_pid_change(self, values: tuple[float, float, float, float, float, float]) -> None:
        self._motor_pid_values = tuple(float(value) for value in values)
        if self.worker.is_open:
            self.worker.send(build_set_motor_pid(*self._motor_pid_values))

    def _on_manual_pid_send(self, values: tuple[float, float, float, float, float, float]) -> None:
        self._motor_pid_values = tuple(float(value) for value in values)
        if self.worker.is_open:
            self.worker.send(build_set_motor_pid(*self._motor_pid_values))

    def _on_manual_pid_read(self) -> None:
        self._request_motor_pid_values()

    def _request_motor_pid_values(self) -> None:
        if not self.worker.is_open:
            return
        seq = (getattr(self, "_pid_request_seq", 0) + 1) & 0xFFFF
        self._pid_request_seq = seq
        self.worker.send(build_pid_req(seq, u32(now_ms_monotonic())))

    def _apply_motor_pid_values(self, values: tuple[float, float, float, float, float, float]) -> None:
        normalized = tuple(float(value) for value in values)
        self._motor_pid_values = normalized
        if hasattr(self, "manual_tab"):
            self.manual_tab.set_motor_pid_values(*normalized)

    def _handle_motor_pid_resp(self, resp: MotorPidData) -> None:
        self._apply_motor_pid_values(
            (
                resp.left_p,
                resp.left_i,
                resp.left_d,
                resp.right_p,
                resp.right_i,
                resp.right_d,
            )
        )

    def _send_control_if_due(self, now_ms: int) -> None:
        if not self.worker.is_open:
            return
        if self._kill_active:
            return
        if self._coord_running:
            return
        if self._active_tab != "Manual":
            return
        if now_ms < self._next_control_due_ms:
            return
        self._next_control_due_ms = now_ms + self._control_period_ms

        left_cmd, right_cmd = self._last_control

        pkt = build_control(self._control_timestamp_u32(now_ms), left_cmd, right_cmd, self._control_duration_ms)
        self.worker.send(pkt)

    def _on_tab_change(self, _event: tk.Event) -> None:
        try:
            tab_text = self.nb.tab(self.nb.select(), "text")
        except Exception:
            return
        self._active_tab = tab_text
        if tab_text != "Coordinate" and self._coord_running:
            self._coord_running = False
            self._stop_external_runtime()
            self.coord_tab.set_running(False)
            self.coord_tab.stop_expected()
        if tab_text != "Manual" and self.worker.is_open and not self._kill_active:
            ts_u32 = self._control_timestamp_u32()
            pkt = build_control(ts_u32, self._manual_neutral, self._manual_neutral, self._control_duration_ms)
            self.worker.send(pkt)

    def _kill_switch(self) -> None:
        self._kill_active = not self._kill_active
        self._update_kill_indicator()
        if self._kill_active:
            self._coord_running = False
            self._stop_external_runtime()
            self.coord_tab.set_running(False)
            self.coord_tab.stop_expected()
            self._last_control = (self._manual_neutral, self._manual_neutral)
            self._kill_send_tick()

    def _update_kill_indicator(self) -> None:
        if not hasattr(self, "kill_btn"):
            return
        if self._kill_active:
            self.kill_btn.configure(text="KILL SWITCH (ON)", style="KillOn.TButton")
        else:
            self.kill_btn.configure(text="KILL SWITCH", style="KillOff.TButton")

    def _kill_send_tick(self) -> None:
        if not self._kill_active:
            return
        if self.worker.is_open:
            ts_u32 = self._control_timestamp_u32()
            pkt = build_control(ts_u32, self._manual_neutral, self._manual_neutral, self._control_duration_ms)
            self.worker.send(pkt)
        self.root.after(self._kill_tick_ms, self._kill_send_tick)

    # ---------------- Sync logic ----------------

    def _begin_initial_sync(self) -> None:
        self._sync_points.clear()
        self._sync_pending.clear()
        self._initial_sync_in_progress = True
        self.time_model = TimeModel()
        self._sync_seq = 0
        self._next_sync_due_ms = now_ms_monotonic()

    def _finalize_initial_sync(self, now_ms: int) -> None:
        model = estimate_initial(self._sync_points, k_best=self._initial_sync_kbest)
        if model:
            self.time_model = model
        self._initial_sync_in_progress = False
        self._next_sync_due_ms = now_ms + self._sync_period_ms
        if self._pending_initial_pid_read:
            self._pending_initial_pid_read = False
            self._request_motor_pid_values()

    def _kick_sync_if_due(self, now_ms: int) -> None:
        if not self.worker.is_open:
            return
        if now_ms < self._next_sync_due_ms:
            return

        if self._initial_sync_in_progress:
            # fire rounds back-to-back (small spacing)
            self._next_sync_due_ms = now_ms + 80
            if len(self._sync_points) >= self._initial_sync_target:
                self._finalize_initial_sync(now_ms)
                return
        else:
            self._next_sync_due_ms = now_ms + self._sync_period_ms

        self._sync_seq = (self._sync_seq + 1) & 0xFFFF
        t1_u32 = u32(now_ms)
        self._sync_pending[self._sync_seq] = t1_u32
        self.worker.send(build_sync_req(self._sync_seq, t1_u32))

    def _handle_sync_resp(self, resp: SyncResp, pc_rx_ms: int) -> None:
        """
        We don't have seq in F0 in the PDF fragment, but the algorithm section says it should echo seq.
        In the "format messages" section, F0 payload is only t_rx and t_tx (8 bytes) :contentReference[oaicite:7]{index=7}.
        So we must match using "last pending" strategy.
        """
        if not self._sync_pending:
            return
        # Take the newest outstanding seq (best effort)
        seq = max(self._sync_pending.keys())
        t1_u32 = self._sync_pending.pop(seq)

        t4_u32 = u32(pc_rx_ms)
        sp = compute_sync_point(
            t1_pc_u32=t1_u32,
            t4_pc_u32=t4_u32,
            t2_mcu_u32=resp.t2_rx_ms,
            t3_mcu_u32=resp.t3_tx_ms,
            pc_now_ms_i64=pc_rx_ms
        )

        # Filter very bad deltas (optional). Keep simple: accept if delta within sane bound.
        if abs(sp.delta_ms) > 5000:
            return

        if self._initial_sync_in_progress:
            self._sync_points.append(sp)
        else:
            prev = self.time_model.last_good
            if prev is not None:
                update_model(self.time_model, prev, sp, beta=self._sync_beta)
            else:
                self.time_model.last_good = sp
                self.time_model.have_lock = True

    # ---------------- RX processing & UI updates ----------------

    def _poll_rx(self) -> None:
        if self._is_shutting_down:
            return
        now_ms = now_ms_monotonic()

        # periodic tasks
        self._send_control_if_due(now_ms)
        self._kick_sync_if_due(now_ms)
        self._poll_magnetometer_calibration_jobs()
        self._poll_sensor_calibration_jobs("acc")
        self._poll_sensor_calibration_jobs("gyro")

        # handle RX queue
        while True:
            try:
                ev = self.worker.rx_queue.get_nowait()
            except Exception:
                break

            if isinstance(ev, RxError):
                self._rx_ok.append(False)
                # show only rarely to avoid spam
                self.radio_var.set(f"Radio Quality: {self._quality_0_10()}/10")
                continue

            if isinstance(ev, RxEvent):
                self._rx_ok.append(True)
                self.radio_var.set(f"Radio Quality: {self._quality_0_10()}/10")

                parsed = ev.parsed

                # log parsed (console always, file when enabled)
                log_obj = {
                    "pc_rx_ms": ev.pc_rx_ms,
                    "msg": self._msg_to_dict(parsed),
                    "time_model": {"a": self.time_model.a, "b": self.time_model.b, "lock": self.time_model.have_lock}
                }
                if isinstance(parsed, (ImuData, TachoData, MotorData, SensorTensorData)):
                    log_obj["rx_time"] = parsed.ts_ms
                try:
                    line = self.logger.format_line(log_obj)
                    print(line, flush=True)
                    msg_type = None
                    msg = log_obj.get("msg")
                    if isinstance(msg, dict):
                        msg_type = msg.get("type")
                    self._append_log_to_tabs(line, msg_type)
                    if self.logger.is_running:
                        self.logger.write_line(line)
                except Exception:
                    pass

                # handle messages
                try:
                    if isinstance(parsed, MotorData):
                        self._last_motor = parsed
                        self._last_mcu_ts_u32 = parsed.ts_ms
                        self._publish_calibration_stream_snapshot(
                            make_builtin_producer_id("motor_telemetry"),
                            "motor.telemetry",
                            {
                                "timestamp_mcu": parsed.ts_ms,
                                "current_l": float(parsed.current_l),
                                "current_r": float(parsed.current_r),
                                "voltage_l": float(parsed.voltage_l),
                                "voltage_r": float(parsed.voltage_r),
                                "temp_l": float(parsed.temp_l),
                                "temp_r": float(parsed.temp_r),
                                "stream_id": "motor_telemetry",
                                "stream_type": "raw",
                            },
                        )
                        self._update_motor(parsed)
                    elif isinstance(parsed, SensorTensorData):
                        self._last_d3 = parsed
                        self._last_mcu_ts_u32 = parsed.ts_ms
                        self._publish_calibration_stream_snapshot(
                            make_builtin_producer_id("sensor_tensor_telemetry"),
                            "sensor_tensor.telemetry",
                            {
                                "timestamp_mcu": parsed.ts_ms,
                                "linear_velocity": list(parsed.linear_velocity),
                                "angular_velocity": list(parsed.angular_velocity),
                                "linear_quality": list(parsed.linear_quality),
                                "angular_quality": list(parsed.angular_quality),
                                "stream_id": "sensor_tensor_telemetry",
                                "stream_type": "raw",
                            },
                        )
                        self._update_mcu_time_label()
                    elif isinstance(parsed, SyncResp):
                        self._handle_sync_resp(parsed, ev.pc_rx_ms)
                    elif isinstance(parsed, MotorPidData):
                        self._handle_motor_pid_resp(parsed)
                    elif isinstance(parsed, (ImuData, TachoData)):
                        self._last_mcu_ts_u32 = parsed.ts_ms
                        # could be extended later
                        self._update_mcu_time_label()
                    if isinstance(parsed, ImuData):
                        self._last_imu = parsed
                        self._update_magnetometer_live_view(parsed, ev.pc_rx_ms)
                        self._update_accelerometer_live_view(parsed, ev.pc_rx_ms)
                        self._update_gyroscope_live_view(parsed, ev.pc_rx_ms)
                    if isinstance(parsed, TachoData):
                        self._last_tacho = parsed
                        self._publish_calibration_stream_snapshot(
                            make_builtin_producer_id("tacho_pair"),
                            "tacho.rpm_pair",
                            {
                                "timestamp_mcu": parsed.ts_ms,
                                "left_rpm": float(parsed.left_rpm),
                                "right_rpm": float(parsed.right_rpm),
                                "stream_id": "tacho_pair",
                                "stream_type": "raw",
                            },
                        )
                        self._update_track_speed_labels(parsed.left_rpm, parsed.right_rpm)
                        self.manual_tab.update_rpm(parsed.left_rpm, parsed.right_rpm)
                        self.coord_tab.add_actual_tacho(parsed.left_rpm, parsed.right_rpm, parsed.ts_ms)
                        # External runtime treats the incoming speed sample as the main processing trigger.
                        self._ingest_external_runtime_snapshot(self.build_telemetry_snapshot(ev.pc_rx_ms))
                    elif not isinstance(parsed, (MotorData, SensorTensorData, SyncResp, MotorPidData, ImuData)):
                        # unknown Frame - ignore
                        pass
                except Exception as exc:
                    error_line = f"[rx-handler:{type(parsed).__name__}] {type(exc).__name__}: {exc}"
                    try:
                        print(error_line, flush=True)
                        self._append_log_to_tabs(error_line, "error")
                    except Exception:
                        pass
                    traceback.print_exc()

        self._update_mcu_time_label()
        self._update_uart_status()
        self._poll_magnetometer_calibration_jobs()
        self._poll_sensor_calibration_jobs("acc")
        self._poll_sensor_calibration_jobs("gyro")
        if not self._is_shutting_down:
            self.root.after(20, self._poll_rx)

    # ---------------- Settings persistence ----------------

    def _load_settings(self) -> None:
        self._joystick_cfg = {
            "left_shift": 0.0,
            "right_shift": 0.0,
            "left_linear": 1.0,
            "right_linear": 1.0,
        }
        self._manual_cfg = {"command_quantum_ms": self._manual_cfg.get("command_quantum_ms", 100)}
        if not os.path.isfile(self._settings_path):
            return
        try:
            with open(self._settings_path, "r", encoding="utf-8") as fh:
                data = json.load(fh)
        except Exception:
            return

        com = data.get("com", {})
        self.worker.port = com.get("port", self.worker.port)
        self.worker.baud = int(com.get("baud", self.worker.baud))

        dev = data.get("deviation", {})
        try:
            self.dev_cfg = DeviationConfig(
                voltage_normal=float(dev.get("voltage_normal", self.dev_cfg.voltage_normal)),
                voltage_delta=float(dev.get("voltage_delta", self.dev_cfg.voltage_delta)),
                current_normal=float(dev.get("current_normal", self.dev_cfg.current_normal)),
                current_delta=float(dev.get("current_delta", self.dev_cfg.current_delta)),
                temp_normal=float(dev.get("temp_normal", self.dev_cfg.temp_normal)),
                temp_delta=float(dev.get("temp_delta", self.dev_cfg.temp_delta)),
            )
        except Exception:
            pass

        geom = data.get("geometry", {})
        speed = data.get("speed_map", {})
        try:
            drive_wheel_diameter_cm = geom.get("drive_wheel_diameter_cm")
            legacy_circumference_m = speed.get("track_circumference_m")
            if drive_wheel_diameter_cm is None and legacy_circumference_m is not None:
                drive_wheel_diameter_cm = float(legacy_circumference_m) * 100.0 / math.pi
            self.geom_cfg = GeometryConfig(
                a1_cm=float(geom.get("a1_cm", self.geom_cfg.a1_cm)),
                a2_cm=float(geom.get("a2_cm", self.geom_cfg.a2_cm)),
                drive_wheel_diameter_cm=float(
                    drive_wheel_diameter_cm
                    if drive_wheel_diameter_cm is not None
                    else self.geom_cfg.drive_wheel_diameter_cm
                ),
            )
        except Exception:
            pass

        try:
            self.speed_cfg = SpeedMapConfig(
                pwm_1=float(speed.get("pwm_1", self.speed_cfg.pwm_1)),
                speed_1=float(speed.get("speed_1", self.speed_cfg.speed_1)),
                pwm_2=float(speed.get("pwm_2", self.speed_cfg.pwm_2)),
                speed_2=float(speed.get("speed_2", self.speed_cfg.speed_2)),
                pwm_3=float(speed.get("pwm_3", self.speed_cfg.pwm_3)),
                speed_3=float(speed.get("speed_3", self.speed_cfg.speed_3)),
            )
        except Exception:
            pass

        joy = data.get("joystick", {})
        self._joystick_cfg = {
            "left_shift": float(joy.get("left_shift", self._joystick_cfg["left_shift"])),
            "right_shift": float(joy.get("right_shift", self._joystick_cfg["right_shift"])),
            "left_linear": float(joy.get("left_linear", self._joystick_cfg["left_linear"])),
            "right_linear": float(joy.get("right_linear", self._joystick_cfg["right_linear"])),
        }
        manual = data.get("manual", {})
        try:
            self._manual_cfg["command_quantum_ms"] = self._normalize_manual_command_quantum_ms(
                int(round(float(manual.get("command_quantum_ms", self._manual_cfg["command_quantum_ms"]))))
            )
        except Exception:
            pass
        self._test_mode_enabled = bool(data.get("test_mode", self._test_mode_enabled))

        coord = data.get("coordinate", {})
        if isinstance(coord, dict):
            self._coord_state = coord
        magnetometer = data.get("magnetometer", {})
        if isinstance(magnetometer, dict):
            self._mag_saved_state = magnetometer
            saved_dirs = magnetometer.get("file_dialog_dirs", {})
            if isinstance(saved_dirs, dict):
                for key in ("dataset", "params", "plugin", "metrics"):
                    value = str(saved_dirs.get(key, "")).strip()
                    if value:
                        self._mag_file_dialog_dirs[key] = value
        accelerometer = data.get("accelerometer", {})
        if isinstance(accelerometer, dict):
            self._acc_saved_state = accelerometer
            saved_dirs = accelerometer.get("file_dialog_dirs", {})
            if isinstance(saved_dirs, dict):
                for key in ("dataset", "params", "plugin", "metrics"):
                    value = str(saved_dirs.get(key, "")).strip()
                    if value:
                        self._acc_file_dialog_dirs[key] = value
        gyroscope = data.get("gyroscope", {})
        if isinstance(gyroscope, dict):
            self._gyro_saved_state = gyroscope
            saved_dirs = gyroscope.get("file_dialog_dirs", {})
            if isinstance(saved_dirs, dict):
                for key in ("dataset", "params", "plugin", "metrics"):
                    value = str(saved_dirs.get(key, "")).strip()
                    if value:
                        self._gyro_file_dialog_dirs[key] = value

    def _apply_joystick_settings(self) -> None:
        if not hasattr(self, "_joystick_cfg"):
            return
        self.manual_tab.set_coeffs(
            self._joystick_cfg["left_shift"],
            self._joystick_cfg["right_shift"],
            self._joystick_cfg["left_linear"],
            self._joystick_cfg["right_linear"],
        )

    def _apply_manual_settings(self) -> None:
        if not hasattr(self, "_manual_cfg"):
            return
        self.manual_tab.set_command_quantum_ms(self._manual_cfg["command_quantum_ms"])

    def _apply_geometry_settings(self) -> None:
        if not hasattr(self, "geom_cfg"):
            return
        self.coord_tab.set_geometry(
            self.geom_cfg.a1_cm,
            self.geom_cfg.a2_cm,
            self.geom_cfg.drive_wheel_diameter_cm,
        )
        self.manual_tab.set_drive_wheel_diameter_cm(self.geom_cfg.drive_wheel_diameter_cm)
        if self._last_tacho is not None:
            self._update_track_speed_labels(self._last_tacho.left_rpm, self._last_tacho.right_rpm)

    def _apply_speed_map_settings(self) -> None:
        if not hasattr(self, "speed_cfg"):
            return
        self.coord_tab.set_speed_map(self.speed_cfg)
        self.manual_tab.set_speed_map(self.speed_cfg)

    def _save_settings(self) -> None:
        required_attrs = (
            "_settings_path",
            "worker",
            "dev_cfg",
            "geom_cfg",
            "speed_cfg",
            "_joystick_cfg",
            "_manual_cfg",
        )
        if any(not hasattr(self, name) for name in required_attrs):
            return
        coord_state = None
        if hasattr(self, "coord_tab"):
            coord_state = self.coord_tab.get_state()
        magnetometer_state = self._serialize_magnetometer_settings()
        accelerometer_state = self._serialize_accelerometer_settings()
        gyroscope_state = self._serialize_gyroscope_settings()
        test_mode = False
        if hasattr(self, "_test_mode_var") or hasattr(self, "_test_mode_enabled"):
            test_mode = self._is_test_mode()
        data = {
            "com": {"port": self.worker.port, "baud": self.worker.baud},
            "deviation": {
                "voltage_normal": self.dev_cfg.voltage_normal,
                "voltage_delta": self.dev_cfg.voltage_delta,
                "current_normal": self.dev_cfg.current_normal,
                "current_delta": self.dev_cfg.current_delta,
                "temp_normal": self.dev_cfg.temp_normal,
                "temp_delta": self.dev_cfg.temp_delta,
            },
            "geometry": {
                "a1_cm": self.geom_cfg.a1_cm,
                "a2_cm": self.geom_cfg.a2_cm,
                "drive_wheel_diameter_cm": self.geom_cfg.drive_wheel_diameter_cm,
            },
            "speed_map": {
                "pwm_1": self.speed_cfg.pwm_1,
                "speed_1": self.speed_cfg.speed_1,
                "pwm_2": self.speed_cfg.pwm_2,
                "speed_2": self.speed_cfg.speed_2,
                "pwm_3": self.speed_cfg.pwm_3,
                "speed_3": self.speed_cfg.speed_3,
            },
            "joystick": {
                "left_shift": self._joystick_cfg["left_shift"],
                "right_shift": self._joystick_cfg["right_shift"],
                "left_linear": self._joystick_cfg["left_linear"],
                "right_linear": self._joystick_cfg["right_linear"],
            },
            "manual": {
                "command_quantum_ms": self._manual_cfg["command_quantum_ms"],
            },
            "test_mode": test_mode,
            "coordinate": coord_state,
            "magnetometer": magnetometer_state,
            "accelerometer": accelerometer_state,
            "gyroscope": gyroscope_state,
        }
        try:
            with open(self._settings_path, "w", encoding="utf-8") as fh:
                json.dump(data, fh, ensure_ascii=False, indent=2)
        except Exception:
            pass

    def _serialize_magnetometer_settings(self) -> dict[str, Any]:
        method_entries: list[dict[str, Any]] = []
        for plugin in getattr(self, "_mag_methods", {}).values():
            method_entries.append(
                {
                    "file_path": plugin.file_path,
                    "show": bool(plugin.show_enabled),
                    "record": bool(plugin.record_enabled),
                    "realtime_enabled": bool(plugin.realtime_enabled),
                    "params_profile_path": plugin.params_profile_path or "",
                    "bindings": self._stream_bindings_payload(plugin),
                }
            )

        dataset_paths = [
            dataset.source_path
            for dataset in getattr(self, "_mag_datasets", [])
            if dataset.source_path
        ]
        active_dataset_path = ""
        if getattr(self, "_mag_dataset", None) is not None and self._mag_dataset.source_path:
            active_dataset_path = self._mag_dataset.source_path

        selected_method_path = ""
        selected_plugin = self._selected_magnetometer_plugin() if hasattr(self, "_mag_selected_method_id") else None
        if selected_plugin is not None:
            selected_method_path = selected_plugin.file_path

        primary_heading = {"kind": "raw"}
        if getattr(self, "_mag_primary_heading_stream_id", "raw_heading") != "raw_heading":
            plugin = getattr(self, "_mag_methods", {}).get(self._mag_primary_heading_stream_id)
            if plugin is not None:
                primary_heading = {"kind": "method", "file_path": plugin.file_path}

        view_state = {}
        if hasattr(self, "magnetometer_tab"):
            view_state = self.magnetometer_tab.get_view_state()
        elif isinstance(getattr(self, "_mag_saved_state", None), dict):
            view_state = dict(self._mag_saved_state.get("view_state", {}))

        return {
            "datasets": dataset_paths,
            "active_dataset_path": active_dataset_path,
            "sources": {
                source_id: {
                    "show": bool(source.get("show", False)),
                    "record": bool(source.get("record", False)),
                }
                for source_id, source in getattr(self, "_mag_sources", {}).items()
            },
            "selected_source_id": getattr(self, "_mag_selected_source_id", "raw_magnetometer"),
            "methods": method_entries,
            "selected_method_path": selected_method_path,
            "primary_heading": primary_heading,
            "view_state": view_state,
            "file_dialog_dirs": dict(getattr(self, "_mag_file_dialog_dirs", {})),
        }

    def _log_tx_msg(self, msg: dict) -> None:
        log_obj = {
            "pc_tx_ms": now_ms_monotonic(),
            "direction": "tx",
            "msg": msg,
        }
        try:
            line = self.logger.format_line(log_obj)
            print(line, flush=True)
            msg_type = msg.get("type")
            self._append_log_to_tabs(line, msg_type, tag="tx")
            if self.logger.is_running:
                self.logger.write_line(line)
        except Exception:
            pass

    def _log_tx_raw(self, data: bytes) -> None:
        msg = self._decode_tx_msg(data)
        self._log_tx_msg(msg)

    def _decode_tx_msg(self, data: bytes) -> dict:
        if len(data) < 3 or data[0] != SOF:
            return {"type": "unknown", "raw_len": len(data)}
        msg_type = data[1]
        ln = data[2]
        if len(data) < 3 + ln:
            return {"type": "unknown", "raw_len": len(data)}
        payload = data[3:3+ln]

        if msg_type == TYPE_B0_SYNC_REQ and ln == 6:
            seq, t1 = struct.unpack_from("<HI", payload, 0)
            return {"type": "B0", "seq": seq, "t1_pc_ms": t1}
        if msg_type == TYPE_B1_PID_REQ and ln == 6:
            seq, t1 = struct.unpack_from("<HI", payload, 0)
            return {"type": "B1", "seq": seq, "t1_pc_ms": t1}
        if msg_type == TYPE_C0_CONTROL and ln == 10:
            ts, l, r, dur = struct.unpack_from("<IhhH", payload, 0)
            return {"type": "C0", "ts_ms": ts, "left_cmd": l, "right_cmd": r, "duration_ms": dur}
        if msg_type == TYPE_A0_DISABLE_D and ln == 5:
            ts, d_type = struct.unpack_from("<IB", payload, 0)
            return {"type": "A0", "ts_ms": ts, "d_type": d_type}
        if msg_type == TYPE_A1_ENABLE_D and ln == 7:
            ts, d_type, period = struct.unpack_from("<IBH", payload, 0)
            return {"type": "A1", "ts_ms": ts, "d_type": d_type, "period_ms": period}
        if msg_type == TYPE_A2_SET_PID and ln == 24:
            left_p, left_i, left_d, right_p, right_i, right_d = struct.unpack_from("<6f", payload, 0)
            return {
                "type": "A2",
                "left_p": left_p,
                "left_i": left_i,
                "left_d": left_d,
                "right_p": right_p,
                "right_i": right_i,
                "right_d": right_d,
            }

        parsed = parse_frame(Frame(msg_type=msg_type, payload=payload))
        return self._msg_to_dict(parsed)

    def _quality_0_10(self) -> int:
        if not self._rx_ok:
            return 0
        ok = sum(1 for x in self._rx_ok if x)
        q = int(round(10.0 * ok / len(self._rx_ok)))
        return max(0, min(10, q))

    def _iter_log_tabs(self) -> tuple[Any, ...]:
        tabs: list[Any] = []
        for attr_name in ("manual_tab", "coord_tab", "magnetometer_tab", "accelerometer_tab", "gyroscope_tab"):
            tab = getattr(self, attr_name, None)
            if tab is not None:
                tabs.append(tab)
        return tuple(tabs)

    def _append_log_to_tabs(self, line: str, msg_type: str | None, tag: str | None = None) -> None:
        for tab in self._iter_log_tabs():
            if tab.should_show_log(msg_type):
                tab.append_log_line(line, tag=tag)

    def _estimate_pc_event_ms_from_mcu(self, timestamp_mcu_ms: int) -> Optional[int]:
        if not self.time_model.have_lock:
            return None
        return int(round(self.time_model.pc_from_mcu(timestamp_mcu_ms)))

    def _update_magnetometer_live_view(self, imu: ImuData, pc_rx_ms: int) -> None:
        magnetometer_tab = getattr(self, "magnetometer_tab", None)
        if magnetometer_tab is None:
            return
        timestamp_pc_est = self._estimate_pc_event_ms_from_mcu(imu.ts_ms)
        mx = float(imu.magn[0])
        my = float(imu.magn[1])
        mz = float(imu.magn[2])
        raw_heading = compute_raw_heading_deg(mx, my)
        raw_sample = self._build_magnetometer_process_sample(
            timestamp_mcu=imu.ts_ms,
            timestamp_pc_rx=pc_rx_ms,
            timestamp_pc_est=timestamp_pc_est,
            mx=mx,
            my=my,
            mz=mz,
            heading=raw_heading,
        )
        self._publish_calibration_stream_snapshot(
            make_builtin_producer_id("raw_magnetometer"),
            "imu.magnetometer_vector",
            raw_sample,
        )
        self._publish_calibration_stream_snapshot(
            make_builtin_producer_id("raw_heading"),
            "heading.deg",
            {
                "timestamp_mcu": imu.ts_ms,
                "timestamp_pc_rx": pc_rx_ms,
                "timestamp_pc_est": timestamp_pc_est,
                "heading": raw_heading,
                "stream_id": "raw_heading",
                "stream_type": "raw",
            },
        )
        self._publish_calibration_stream_snapshot(
            make_builtin_producer_id("imu_accel"),
            "imu.accel_vector",
            {
                "timestamp_mcu": imu.ts_ms,
                "timestamp_pc_rx": pc_rx_ms,
                "timestamp_pc_est": timestamp_pc_est,
                "accel_x": float(imu.accel[0]),
                "accel_y": float(imu.accel[1]),
                "accel_z": float(imu.accel[2]),
                "stream_id": "imu_accel",
                "stream_type": "raw",
            },
        )
        self._publish_calibration_stream_snapshot(
            make_builtin_producer_id("imu_gyro"),
            "imu.gyro_vector",
            {
                "timestamp_mcu": imu.ts_ms,
                "timestamp_pc_rx": pc_rx_ms,
                "timestamp_pc_est": timestamp_pc_est,
                "gyro_x": float(imu.gyro[0]),
                "gyro_y": float(imu.gyro[1]),
                "gyro_z": float(imu.gyro[2]),
                "stream_id": "imu_gyro",
                "stream_type": "raw",
            },
        )
        derived_streams, derived_records = self._process_magnetometer_methods_realtime(raw_sample)
        self._record_magnetometer_sample(
            timestamp_mcu=imu.ts_ms,
            timestamp_pc_rx=pc_rx_ms,
            timestamp_pc_est=timestamp_pc_est,
            mx=mx,
            my=my,
            mz=mz,
            derived_records=derived_records,
        )
        self._mag_last_live_snapshot = {
            "timestamp_mcu": imu.ts_ms,
            "timestamp_pc_rx": pc_rx_ms,
            "timestamp_pc_est": timestamp_pc_est,
            "mx": mx,
            "my": my,
            "mz": mz,
            "raw_heading": raw_heading,
            "derived_streams": dict(derived_streams),
        }
        self._refresh_magnetometer_heading_routing_ui()
        selected_heading, selected_source_label = self._resolve_magnetometer_selected_output(raw_heading, derived_streams)
        magnetometer_tab.update_live_imu(
            timestamp_mcu=imu.ts_ms,
            timestamp_pc_rx=pc_rx_ms,
            timestamp_pc_est=timestamp_pc_est,
            mx=mx,
            my=my,
            mz=mz,
            selected_output_heading=selected_heading,
            selected_source_label=selected_source_label,
            derived_streams=derived_streams,
        )

    def _build_magnetometer_process_sample(
        self,
        *,
        timestamp_mcu: int,
        timestamp_pc_rx: int,
        timestamp_pc_est: int | None,
        mx: float,
        my: float,
        mz: float,
        heading: float | None,
    ) -> dict[str, Any]:
        return {
            "stream_id": "raw_magnetometer",
            "stream_type": "raw",
            "producer_name": "Raw Magnetometer",
            "producer_version": "builtin",
            "timestamp_mcu": timestamp_mcu,
            "timestamp_pc_rx": timestamp_pc_rx,
            "timestamp_pc_est": timestamp_pc_est,
            "mag_x": mx,
            "mag_y": my,
            "mag_z": mz,
            "heading": heading,
            "flags": "",
        }

    def _normalize_magnetometer_process_output(
        self,
        plugin: LoadedMethodPlugin,
        output: dict[str, Any],
    ) -> tuple[dict[str, Any] | None, PluginDiagnostics | None]:
        required = ("mag_x", "mag_y", "mag_z")
        missing = [key for key in required if key not in output]
        if missing:
            return (
                None,
                self._make_magnetometer_method_diagnostics(
                    plugin,
                    last_action="validate_process_output",
                    error_text=f"process() output missing required keys: {', '.join(missing)}",
                ),
            )

        try:
            mx = float(output["mag_x"])
            my = float(output["mag_y"])
            mz = float(output["mag_z"])
        except (TypeError, ValueError) as exc:
            return (
                None,
                self._make_magnetometer_method_diagnostics(
                    plugin,
                    last_action="validate_process_output",
                    error_text=f"process() output contains non-numeric magnetometer values: {exc}",
                ),
            )

        if not all(math.isfinite(value) for value in (mx, my, mz)):
            return (
                None,
                self._make_magnetometer_method_diagnostics(
                    plugin,
                    last_action="validate_process_output",
                    error_text="process() output contains non-finite magnetometer values.",
                ),
            )

        heading_raw = output.get("heading")
        heading: float | None
        if heading_raw is None or heading_raw == "":
            heading = compute_raw_heading_deg(mx, my)
        else:
            try:
                heading = float(heading_raw)
            except (TypeError, ValueError) as exc:
                return (
                    None,
                    self._make_magnetometer_method_diagnostics(
                        plugin,
                        last_action="validate_process_output",
                        error_text=f"process() output heading is invalid: {exc}",
                    ),
                )
            if not math.isfinite(heading):
                return (
                    None,
                    self._make_magnetometer_method_diagnostics(
                        plugin,
                        last_action="validate_process_output",
                        error_text="process() output heading must be finite or null.",
                    ),
                )

        normalized = dict(output)
        normalized["mag_x"] = mx
        normalized["mag_y"] = my
        normalized["mag_z"] = mz
        normalized["heading"] = heading
        return (normalized, None)

    def _build_magnetometer_derived_record(
        self,
        plugin: LoadedMethodPlugin,
        output: dict[str, Any],
    ) -> SampleRecord:
        return SampleRecord(
            stream_id=plugin.derived_stream_id or f"derived_{plugin.method_id}",
            stream_type="derived",
            producer_name=plugin.name,
            producer_version=plugin.version,
            timestamp_mcu=int(output["timestamp_mcu"]),
            timestamp_pc_rx=int(output["timestamp_pc_rx"]),
            timestamp_pc_est=output.get("timestamp_pc_est"),
            mag_x=float(output["mag_x"]),
            mag_y=float(output["mag_y"]),
            mag_z=float(output["mag_z"]),
            heading=output.get("heading"),
            flags=str(output.get("flags", "realtime_derived")),
        )

    def _apply_magnetometer_process_error(
        self,
        method_id: str,
        plugin: LoadedMethodPlugin,
        result: ProcessRunResult,
    ) -> None:
        plugin.status = "error"
        plugin.process_warnings = list(result.warnings)
        plugin.realtime_enabled = False
        plugin.record_enabled = False
        plugin.last_output = None
        self._mag_latest_derived_streams.pop(method_id, None)
        for kind in self._magnetometer_method_output_kinds(plugin):
            self._calibration_stream_snapshots.pop(make_method_producer_id(plugin.file_path, kind), None)
        if result.diagnostics is not None:
            plugin.diagnostics = result.diagnostics
        else:
            plugin.diagnostics = self._make_magnetometer_method_diagnostics(
                plugin,
                last_action="process",
                error_text="Realtime processing failed.",
                warnings=result.warnings,
            )
        self._refresh_magnetometer_methods_ui()
        self._open_magnetometer_method_diagnostics(method_id)
        self._save_settings()

    def _process_magnetometer_methods_realtime(
        self,
        raw_sample: dict[str, Any],
    ) -> tuple[dict[str, dict[str, Any]], list[SampleRecord]]:
        self._ensure_magnetometer_method_state()
        self._update_magnetometer_method_routing_state()
        derived_streams: dict[str, dict[str, Any]] = {}
        derived_records: list[SampleRecord] = []

        for method_id, plugin in self._mag_methods.items():
            if not plugin.realtime_enabled:
                plugin.last_output = None
                self._mag_latest_derived_streams.pop(method_id, None)
                continue

            process_requirements = self._magnetometer_method_requirements(plugin, scope=SCOPE_PROCESS)
            process_scope = SCOPE_PROCESS
            if not process_requirements:
                process_requirements = self._magnetometer_method_requirements(plugin, scope=SCOPE_CALIBRATE)
                process_scope = SCOPE_CALIBRATE
            process_sample = dict(raw_sample)
            if process_requirements:
                producers = self._magnetometer_calibration_producers()
                bindings = self._effective_magnetometer_stream_bindings(plugin, scope=process_scope, producers=producers)
                inputs: dict[str, Any] = {}
                for requirement in process_requirements:
                    producer_id = bindings.get(requirement.slot, "")
                    snapshot = self._calibration_stream_snapshots.get(producer_id)
                    if snapshot is not None:
                        inputs[requirement.slot] = dict(snapshot.payload)
                if inputs:
                    process_sample["inputs"] = inputs

            result = run_method_process(plugin, process_sample)
            if not result.ok:
                self._apply_magnetometer_process_error(method_id, plugin, result)
                continue

            assert result.output is not None
            merged_output = dict(process_sample)
            merged_output.update(result.output)
            normalized, diagnostics = self._normalize_magnetometer_process_output(plugin, merged_output)
            if diagnostics is not None or normalized is None:
                self._apply_magnetometer_process_error(
                    method_id,
                    plugin,
                    ProcessRunResult(ok=False, warnings=result.warnings, diagnostics=diagnostics),
                )
                continue

            plugin.status = plugin.load_status
            plugin.diagnostics = None
            plugin.process_warnings = list(result.warnings)
            plugin.last_output = dict(normalized)

            stream_state = {
                "title": plugin.name,
                "stream_id": plugin.derived_stream_id or f"derived_{plugin.method_id}",
                "mx": normalized["mag_x"],
                "my": normalized["mag_y"],
                "mz": normalized["mag_z"],
                "heading": normalized.get("heading"),
                "show": plugin.show_enabled,
            }
            derived_streams[method_id] = stream_state
            self._mag_latest_derived_streams[method_id] = dict(stream_state)
            for kind in self._magnetometer_method_output_kinds(plugin):
                payload = dict(normalized)
                if kind == "heading.deg":
                    payload = {
                        "timestamp_mcu": normalized.get("timestamp_mcu"),
                        "timestamp_pc_rx": normalized.get("timestamp_pc_rx"),
                        "timestamp_pc_est": normalized.get("timestamp_pc_est"),
                        "heading": normalized.get("heading"),
                        "stream_id": plugin.derived_stream_id or f"derived_{plugin.method_id}",
                        "stream_type": "derived",
                    }
                self._publish_calibration_stream_snapshot(
                    make_method_producer_id(plugin.file_path, kind),
                    kind,
                    payload,
                )

            if plugin.record_enabled:
                derived_records.append(self._build_magnetometer_derived_record(plugin, normalized))

        return (derived_streams, derived_records)

    def _resolve_magnetometer_selected_output(
        self,
        raw_heading: float | None,
        derived_streams: dict[str, dict[str, Any]],
    ) -> tuple[float | None, str]:
        self._ensure_magnetometer_method_state()
        primary_stream_id = self._mag_primary_heading_stream_id
        if primary_stream_id == "raw_heading":
            return (raw_heading, "Raw Heading")

        plugin = self._mag_methods.get(primary_stream_id)
        if plugin is None:
            return (raw_heading, "Raw Heading")

        selected = derived_streams.get(primary_stream_id)
        if selected is None:
            selected = self._mag_latest_derived_streams.get(primary_stream_id)
        if selected is None:
            return (None, plugin.name)
        return (selected.get("heading"), plugin.name)

    def _make_magnetometer_dataset_name(self) -> str:
        return f"mag_record_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

    def _make_concatenated_magnetometer_dataset_name(self) -> str:
        return f"mag_concat_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

    def _default_magnetometer_sources(self) -> dict[str, dict[str, object]]:
        return {
            "raw_magnetometer": {
                "title": "Raw Magnetometer",
                "show": True,
                "record": True,
            },
            "raw_heading": {
                "title": "Raw Heading",
                "show": True,
                "record": False,
            },
        }

    def _ensure_magnetometer_source_state(self) -> None:
        if not hasattr(self, "_mag_sources"):
            self._mag_sources = self._default_magnetometer_sources()
        if not hasattr(self, "_mag_selected_source_id") or self._mag_selected_source_id not in self._mag_sources:
            self._mag_selected_source_id = next(iter(self._mag_sources))

    def _ensure_magnetometer_method_state(self) -> None:
        if not hasattr(self, "_mag_methods"):
            self._mag_methods = {}
        if not hasattr(self, "_mag_selected_method_id"):
            self._mag_selected_method_id = None
        if not hasattr(self, "_mag_latest_derived_streams"):
            self._mag_latest_derived_streams = {}
        if not hasattr(self, "_mag_primary_heading_stream_id"):
            self._mag_primary_heading_stream_id = "raw_heading"
        if not hasattr(self, "_mag_last_live_snapshot"):
            self._mag_last_live_snapshot = None
        if not hasattr(self, "_mag_metrics_report"):
            self._mag_metrics_report = None
        if not hasattr(self, "_mag_offline_method_clouds"):
            self._mag_offline_method_clouds = {}
        if not hasattr(self, "_mag_file_dialog_dirs"):
            self._mag_file_dialog_dirs = {"dataset": "", "params": "", "plugin": "", "metrics": ""}
        if not hasattr(self, "_mag_saved_state"):
            self._mag_saved_state = {}
        if not hasattr(self, "_calibration_stream_snapshots"):
            self._calibration_stream_snapshots = {}
        for plugin in self._mag_methods.values():
            if not isinstance(getattr(plugin, "stream_bindings", None), dict):
                plugin.stream_bindings = {SCOPE_CALIBRATE: {}, SCOPE_PROCESS: {}}
            else:
                plugin.stream_bindings.setdefault(SCOPE_CALIBRATE, {})
                plugin.stream_bindings.setdefault(SCOPE_PROCESS, {})
            if not isinstance(getattr(plugin, "routing_validation", None), dict):
                plugin.routing_validation = {}
            if not isinstance(getattr(plugin, "routing_warnings", None), list):
                plugin.routing_warnings = []
            if not isinstance(getattr(plugin, "stream_requirements", None), dict):
                plugin.stream_requirements = {}
            if not plugin.stream_requirements:
                try:
                    plugin.stream_requirements = infer_stream_requirements(plugin.info)
                except Exception:
                    plugin.stream_requirements = {}

    def _publish_calibration_stream_snapshot(
        self,
        producer_id: str,
        kind: str,
        payload: dict[str, Any],
    ) -> None:
        self._ensure_magnetometer_method_state()
        self._calibration_stream_snapshots[producer_id] = LatestStreamSnapshot(
            producer_id=producer_id,
            kind=kind,
            payload=dict(payload),
            timestamp_mcu=payload.get("timestamp_mcu"),
            timestamp_pc_rx=payload.get("timestamp_pc_rx"),
            timestamp_pc_est=payload.get("timestamp_pc_est"),
        )

    def _magnetometer_method_requirements(
        self,
        plugin: LoadedMethodPlugin,
        *,
        scope: str,
    ) -> list[StreamRequirementDescriptor]:
        self._ensure_magnetometer_method_state()
        requirements = plugin.stream_requirements.get(scope, [])
        return [
            requirement
            for requirement in requirements
            if isinstance(requirement, StreamRequirementDescriptor)
        ]

    def _magnetometer_method_output_kinds(self, plugin: LoadedMethodPlugin) -> tuple[str, ...]:
        return infer_output_stream_kinds(plugin.info)

    def _dataset_has_stream(self, stream_id: str, dataset: Dataset | None = None) -> bool:
        active_dataset = getattr(self, "_mag_dataset", None) if dataset is None else dataset
        if active_dataset is None:
            return False
        return any(record.stream_id == stream_id for record in active_dataset.records)

    def _builtin_calibration_producers(self, dataset: Dataset | None = None) -> dict[str, StreamProducerDescriptor]:
        active_dataset = getattr(self, "_mag_dataset", None) if dataset is None else dataset
        raw_mag_active = True
        raw_heading_active = True
        accel_active = make_builtin_producer_id("imu_accel") in self._calibration_stream_snapshots
        gyro_active = make_builtin_producer_id("imu_gyro") in self._calibration_stream_snapshots
        tacho_active = make_builtin_producer_id("tacho_pair") in self._calibration_stream_snapshots
        motor_active = make_builtin_producer_id("motor_telemetry") in self._calibration_stream_snapshots
        tensor_active = make_builtin_producer_id("sensor_tensor_telemetry") in self._calibration_stream_snapshots
        return {
            make_builtin_producer_id("raw_magnetometer"): StreamProducerDescriptor(
                producer_id=make_builtin_producer_id("raw_magnetometer"),
                kind="imu.magnetometer_vector",
                title="Raw Magnetometer",
                stream_id="raw_magnetometer",
                stream_type="raw",
                origin="builtin",
                active=raw_mag_active,
                details={"source": "D0.magn"},
            ),
            make_builtin_producer_id("raw_heading"): StreamProducerDescriptor(
                producer_id=make_builtin_producer_id("raw_heading"),
                kind="heading.deg",
                title="Raw Heading",
                stream_id="raw_heading",
                stream_type="raw",
                origin="builtin",
                active=raw_heading_active,
                details={"source": "built-in atan2(mx, my)"},
            ),
            make_builtin_producer_id("imu_accel"): StreamProducerDescriptor(
                producer_id=make_builtin_producer_id("imu_accel"),
                kind="imu.accel_vector",
                title="Raw Accelerometer",
                stream_id="imu_accel",
                stream_type="raw",
                origin="builtin",
                active=accel_active,
                details={"source": "D0.accel"},
            ),
            make_builtin_producer_id("imu_gyro"): StreamProducerDescriptor(
                producer_id=make_builtin_producer_id("imu_gyro"),
                kind="imu.gyro_vector",
                title="Raw Gyroscope",
                stream_id="imu_gyro",
                stream_type="raw",
                origin="builtin",
                active=gyro_active,
                details={"source": "D0.gyro"},
            ),
            make_builtin_producer_id("tacho_pair"): StreamProducerDescriptor(
                producer_id=make_builtin_producer_id("tacho_pair"),
                kind="tacho.rpm_pair",
                title="Track Tacho Pair",
                stream_id="tacho_pair",
                stream_type="raw",
                origin="builtin",
                active=tacho_active,
                details={"source": "D1"},
            ),
            make_builtin_producer_id("motor_telemetry"): StreamProducerDescriptor(
                producer_id=make_builtin_producer_id("motor_telemetry"),
                kind="motor.telemetry",
                title="Motor Telemetry",
                stream_id="motor_telemetry",
                stream_type="raw",
                origin="builtin",
                active=motor_active,
                details={"source": "D2"},
            ),
            make_builtin_producer_id("sensor_tensor_telemetry"): StreamProducerDescriptor(
                producer_id=make_builtin_producer_id("sensor_tensor_telemetry"),
                kind="sensor_tensor.telemetry",
                title="Sensor Tensor Telemetry",
                stream_id="sensor_tensor_telemetry",
                stream_type="raw",
                origin="builtin",
                active=tensor_active,
                details={"source": "D3"},
            ),
        }

    def _method_calibration_producers(self) -> dict[str, StreamProducerDescriptor]:
        self._ensure_magnetometer_method_state()
        producers: dict[str, StreamProducerDescriptor] = {}
        for method_id, plugin in self._mag_methods.items():
            stream_id = plugin.derived_stream_id or f"derived_{plugin.method_id}"
            for kind in self._magnetometer_method_output_kinds(plugin):
                title = plugin.name if kind != "heading.deg" else f"{plugin.name} (Heading)"
                producer_id = make_method_producer_id(plugin.file_path, kind)
                producers[producer_id] = StreamProducerDescriptor(
                    producer_id=producer_id,
                    kind=kind,
                    title=title,
                    stream_id=stream_id,
                    stream_type="derived",
                    origin="method",
                    status=plugin.status,
                    active=(
                        plugin.status != "error"
                        and plugin.supports_process()
                        and plugin.calibration_params is not None
                    ),
                    file_path=plugin.file_path,
                    method_id=method_id,
                    requires_calibration=True,
                    details={
                        "method_name": plugin.name,
                        "method_version": plugin.version,
                    },
                )
        return producers

    def _magnetometer_calibration_producers(self, dataset: Dataset | None = None) -> dict[str, StreamProducerDescriptor]:
        producers = self._builtin_calibration_producers(dataset)
        producers.update(self._method_calibration_producers())
        return producers

    def _effective_magnetometer_stream_bindings(
        self,
        plugin: LoadedMethodPlugin,
        *,
        scope: str,
        producers: dict[str, StreamProducerDescriptor] | None = None,
    ) -> dict[str, str]:
        self._ensure_magnetometer_method_state()
        producer_map = self._magnetometer_calibration_producers() if producers is None else producers
        explicit = {
            str(slot): str(producer_id)
            for slot, producer_id in plugin.stream_bindings.get(scope, {}).items()
            if str(slot).strip() and str(producer_id).strip()
        }
        requirements = self._magnetometer_method_requirements(plugin, scope=scope)
        defaults = select_default_binding(requirements, producer_map)
        effective = dict(defaults)
        effective.update(explicit)
        return effective

    def _all_magnetometer_method_bindings_by_path(
        self,
        *,
        scope: str,
        producers: dict[str, StreamProducerDescriptor] | None = None,
    ) -> dict[str, dict[str, str]]:
        producer_map = self._magnetometer_calibration_producers() if producers is None else producers
        return {
            plugin.file_path: self._effective_magnetometer_stream_bindings(plugin, scope=scope, producers=producer_map)
            for plugin in self._mag_methods.values()
        }

    def _update_magnetometer_method_routing_state(self) -> None:
        self._ensure_magnetometer_method_state()
        producers = self._magnetometer_calibration_producers()
        bindings_by_path = self._all_magnetometer_method_bindings_by_path(scope=SCOPE_CALIBRATE, producers=producers)
        for plugin in self._mag_methods.values():
            requirements = self._magnetometer_method_requirements(plugin, scope=SCOPE_CALIBRATE)
            selected_bindings = dict(bindings_by_path.get(plugin.file_path, {}))
            validation = validate_method_bindings(
                method_key=plugin.file_path,
                requirements=requirements,
                selected_bindings=selected_bindings,
                producers=producers,
                bindings_by_method=bindings_by_path,
            )
            plugin.routing_validation[SCOPE_CALIBRATE] = validation
            plugin.routing_warnings = list(validation.issues)

    def _can_start_magnetometer_calibration(self, plugin: LoadedMethodPlugin) -> bool:
        validation = plugin.routing_validation.get(SCOPE_CALIBRATE)
        if validation is None:
            return plugin.can_start_calibration()
        return plugin.can_start_calibration() and validation.ok

    def _method_binding_candidates_for_ui(self, plugin: LoadedMethodPlugin) -> list[dict[str, Any]]:
        self._update_magnetometer_method_routing_state()
        validation = plugin.routing_validation.get(SCOPE_CALIBRATE)
        producers = self._magnetometer_calibration_producers()
        bindings = self._effective_magnetometer_stream_bindings(plugin, scope=SCOPE_CALIBRATE, producers=producers)
        rows: list[dict[str, Any]] = []
        for requirement in self._magnetometer_method_requirements(plugin, scope=SCOPE_CALIBRATE):
            selected_producer_id = bindings.get(requirement.slot, "")
            selected_label = "Not selected"
            if selected_producer_id in producers:
                selected_label = producers[selected_producer_id].title
            candidates: list[dict[str, Any]] = []
            if validation is not None:
                candidates = list(validation.slot_candidates.get(requirement.slot, []))
            rows.append(
                {
                    "slot": requirement.slot,
                    "kind": requirement.kind,
                    "label": requirement.label or requirement.slot,
                    "description": requirement.description,
                    "selected_producer_id": selected_producer_id,
                    "selected_label": selected_label,
                    "candidates": candidates,
                }
            )
        return rows

    def _method_routing_validation_payload(self, plugin: LoadedMethodPlugin) -> dict[str, Any]:
        validation = plugin.routing_validation.get(SCOPE_CALIBRATE)
        return {} if validation is None else validation.as_dict()

    def _stream_requirements_payload(self, plugin: LoadedMethodPlugin) -> dict[str, Any]:
        payload: dict[str, Any] = {}
        for scope, requirements in plugin.stream_requirements.items():
            payload[scope] = [
                requirement.as_dict()
                for requirement in requirements
                if isinstance(requirement, StreamRequirementDescriptor)
            ]
        return payload

    def _stream_bindings_payload(self, plugin: LoadedMethodPlugin) -> dict[str, Any]:
        return {
            scope: {
                str(slot): str(producer_id)
                for slot, producer_id in bindings.items()
                if str(slot).strip() and str(producer_id).strip()
            }
            for scope, bindings in plugin.stream_bindings.items()
            if isinstance(bindings, dict)
        }

    def _build_magnetometer_method_info_payload(self, method_id: str) -> dict[str, Any]:
        self._ensure_magnetometer_method_state()
        plugin = self._mag_methods.get(method_id)
        if plugin is None:
            return {}
        self._update_magnetometer_method_routing_state()
        diagnostics = None if plugin.diagnostics is None else plugin.diagnostics.as_dict()
        return build_method_info_payload(
            plugin.info,
            file_path=plugin.file_path,
            status_text=plugin.display_status_text(),
            warnings=plugin.effective_warnings(),
            calibration_dataset_name=plugin.calibration_dataset_name,
            calibration_runtime_s=plugin.calibration_runtime_s,
            params_profile_path=plugin.params_profile_path,
            calibration_params=plugin.calibration_params,
            calibration_report=plugin.calibration_report,
            stream_requirements=self._stream_requirements_payload(plugin),
            stream_bindings=self._stream_bindings_payload(plugin),
            routing_validation=self._method_routing_validation_payload(plugin),
            diagnostics=diagnostics,
            calibration_bindings=self._method_binding_candidates_for_ui(plugin),
        )

    def _on_change_magnetometer_method_binding(self, method_id: str, slot: str, producer_id: str) -> bool:
        self._ensure_magnetometer_method_state()
        plugin = self._mag_methods.get(method_id)
        if plugin is None:
            return False
        self._update_magnetometer_method_routing_state()
        current_validation = plugin.routing_validation.get(SCOPE_CALIBRATE)
        selectable = False
        if current_validation is not None:
            for candidate in current_validation.slot_candidates.get(slot, []):
                if candidate.get("producer_id") == producer_id and bool(candidate.get("selectable", False)):
                    selectable = True
                    break
        if not selectable:
            return False
        old_value = plugin.stream_bindings.setdefault(SCOPE_CALIBRATE, {}).get(slot, "")
        plugin.stream_bindings[SCOPE_CALIBRATE][slot] = producer_id
        self._update_magnetometer_method_routing_state()
        if plugin.routing_validation.get(SCOPE_CALIBRATE) is None:
            if old_value:
                plugin.stream_bindings[SCOPE_CALIBRATE][slot] = old_value
            else:
                plugin.stream_bindings[SCOPE_CALIBRATE].pop(slot, None)
            self._update_magnetometer_method_routing_state()
            return False
        self._refresh_magnetometer_methods_ui()
        self._refresh_all_magnetometer_method_dataset_clouds()
        self._save_settings()
        return True

    def _adapt_magnetometer_record_for_producer(
        self,
        producer: StreamProducerDescriptor,
        record: SampleRecord,
    ) -> dict[str, Any]:
        payload = self._build_magnetometer_sample_from_record(record)
        if producer.kind == "heading.deg" and payload.get("heading") is None:
            payload["heading"] = compute_raw_heading_deg(float(record.mag_x), float(record.mag_y))
        return payload

    def _records_for_builtin_producer(
        self,
        producer: StreamProducerDescriptor,
        dataset: Dataset,
    ) -> list[SampleRecord]:
        if producer.stream_id == "raw_magnetometer":
            return self._raw_magnetometer_dataset_records(dataset)
        if producer.stream_id == "raw_heading":
            records = [record for record in dataset.records if record.stream_id == "raw_heading"]
            return records if records else self._raw_magnetometer_dataset_records(dataset)
        return []

    def _build_process_sample_from_inputs(
        self,
        plugin: LoadedMethodPlugin,
        requirements: list[StreamRequirementDescriptor],
        inputs_payload: dict[str, dict[str, Any]],
        index: int,
    ) -> dict[str, Any] | None:
        primary_slot = ""
        for requirement in requirements:
            if requirement.kind == "imu.magnetometer_vector":
                primary_slot = requirement.slot
                break
        if not primary_slot and requirements:
            primary_slot = requirements[0].slot
        if not primary_slot:
            return None
        primary_records = list(inputs_payload.get(primary_slot, {}).get("records", []))
        if not primary_records:
            return None
        if index >= len(primary_records):
            index = len(primary_records) - 1
        sample = dict(primary_records[index])
        slot_inputs: dict[str, Any] = {}
        for requirement in requirements:
            records = list(inputs_payload.get(requirement.slot, {}).get("records", []))
            if records:
                selected_index = min(index, len(records) - 1)
                slot_inputs[requirement.slot] = dict(records[selected_index])
                continue
            latest = inputs_payload.get(requirement.slot, {}).get("latest")
            if latest is not None:
                slot_inputs[requirement.slot] = dict(latest)
        if slot_inputs:
            sample["inputs"] = slot_inputs
        return sample

    def _compute_offline_method_records(
        self,
        plugin: LoadedMethodPlugin,
        dataset: Dataset,
        *,
        cache: dict[str, list[SampleRecord]],
        stack: set[str],
    ) -> list[SampleRecord]:
        cache_key = plugin.file_path
        if cache_key in cache:
            return list(cache[cache_key])
        if cache_key in stack:
            return []
        if (
            plugin.module is None
            or not plugin.supports_process()
            or plugin.calibration_params is None
            or not callable(getattr(plugin.module, "process", None))
        ):
            return []
        stack.add(cache_key)
        resolver = self._build_magnetometer_calibration_resolver(
            plugin,
            dataset,
            cache=cache,
            stack=stack,
        )
        requirements = self._magnetometer_method_requirements(plugin, scope=SCOPE_CALIBRATE)
        if not requirements:
            requirements = self._magnetometer_method_requirements(plugin, scope=SCOPE_PROCESS)
        inputs_payload = resolver.export_inputs(requirements)
        if requirements:
            primary_records = list(inputs_payload.get(requirements[0].slot, {}).get("records", []))
        else:
            primary_records = [self._build_magnetometer_sample_from_record(record) for record in self._raw_magnetometer_dataset_records(dataset)]
        derived_records: list[SampleRecord] = []
        for index in range(len(primary_records)):
            if requirements:
                sample = self._build_process_sample_from_inputs(plugin, requirements, inputs_payload, index)
            else:
                sample = dict(primary_records[index])
            if sample is None:
                continue
            result = run_method_process(plugin, sample, plugin.calibration_params)
            if not result.ok or result.output is None:
                if result.diagnostics is not None:
                    plugin.diagnostics = result.diagnostics
                plugin.status = "error"
                plugin.realtime_enabled = False
                plugin.record_enabled = False
                stack.remove(cache_key)
                return []
            merged_output = dict(sample)
            merged_output.update(result.output)
            normalized, diagnostics = self._normalize_magnetometer_process_output(plugin, merged_output)
            if diagnostics is not None or normalized is None:
                plugin.diagnostics = diagnostics
                plugin.status = "error"
                plugin.realtime_enabled = False
                plugin.record_enabled = False
                stack.remove(cache_key)
                return []
            derived_records.append(self._build_magnetometer_derived_record(plugin, normalized))
        stack.remove(cache_key)
        cache[cache_key] = list(derived_records)
        return list(derived_records)

    def _dataset_records_for_stream_producer(
        self,
        producer: StreamProducerDescriptor,
        dataset: Dataset,
        *,
        cache: dict[str, list[SampleRecord]],
        stack: set[str],
    ) -> list[SampleRecord]:
        if producer.origin == "builtin":
            return self._records_for_builtin_producer(producer, dataset)
        if producer.producer_id in cache:
            return list(cache[producer.producer_id])
        records = [record for record in dataset.records if record.stream_id == producer.stream_id]
        if records:
            cache[producer.producer_id] = list(records)
            return list(records)
        upstream_plugin = None
        for plugin in self._mag_methods.values():
            if plugin.file_path == producer.file_path:
                upstream_plugin = plugin
                break
        if upstream_plugin is None:
            return []
        records = self._compute_offline_method_records(upstream_plugin, dataset, cache=cache, stack=stack)
        cache[producer.producer_id] = list(records)
        return list(records)

    def _build_magnetometer_calibration_resolver(
        self,
        plugin: LoadedMethodPlugin,
        dataset: Dataset,
        *,
        cache: dict[str, list[SampleRecord]] | None = None,
        stack: set[str] | None = None,
    ) -> CalibrationStreamResolver:
        producer_map = self._magnetometer_calibration_producers(dataset)
        bindings = self._effective_magnetometer_stream_bindings(plugin, scope=SCOPE_CALIBRATE, producers=producer_map)
        record_cache: dict[str, list[SampleRecord]] = {} if cache is None else cache
        active_stack: set[str] = set() if stack is None else stack

        return CalibrationStreamResolver(
            producers=producer_map,
            bindings=bindings,
            live_snapshots=self._calibration_stream_snapshots,
            dataset_records_provider=lambda producer: self._dataset_records_for_stream_producer(
                producer,
                dataset,
                cache=record_cache,
                stack=active_stack,
            ),
            record_adapter=self._adapt_magnetometer_record_for_producer,
            latest_adapter=lambda snapshot: dict(snapshot.payload),
        )

    def _build_magnetometer_calibration_config(
        self,
        plugin: LoadedMethodPlugin,
        dataset: Dataset,
    ) -> dict[str, Any]:
        self._update_magnetometer_method_routing_state()
        resolver = self._build_magnetometer_calibration_resolver(plugin, dataset)
        requirements = self._magnetometer_method_requirements(plugin, scope=SCOPE_CALIBRATE)
        return {
            "stream_bindings": self._effective_magnetometer_stream_bindings(plugin, scope=SCOPE_CALIBRATE),
            "stream_requirements": [requirement.as_dict() for requirement in requirements],
            "stream_resolver": resolver,
            "stream_inputs": resolver.export_inputs(requirements),
            "routing_validation": self._method_routing_validation_payload(plugin),
        }

    def _raw_magnetometer_dataset_records(self, dataset: Dataset | None = None) -> list[SampleRecord]:
        active_dataset = getattr(self, "_mag_dataset", None) if dataset is None else dataset
        if active_dataset is None:
            return []
        return [
            record
            for record in active_dataset.records
            if record.stream_id == "raw_magnetometer" and "heading_only" not in (record.flags or "")
        ]

    def _build_magnetometer_sample_from_record(self, record: SampleRecord) -> dict[str, Any]:
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

    def _refresh_magnetometer_method_clouds_ui(self) -> None:
        magnetometer_tab = getattr(self, "magnetometer_tab", None)
        if magnetometer_tab is None:
            return
        magnetometer_tab.set_method_dataset_clouds(getattr(self, "_mag_offline_method_clouds", {}))

    def _recompute_magnetometer_method_dataset_cloud(
        self,
        method_id: str,
        *,
        open_diagnostics: bool = False,
    ) -> None:
        self._ensure_magnetometer_method_state()
        plugin = self._mag_methods.get(method_id)
        if plugin is None:
            self._mag_offline_method_clouds.pop(method_id, None)
            return
        if (
            plugin.module is None
            or not plugin.supports_process()
            or plugin.calibration_params is None
            or not callable(getattr(plugin.module, "process", None))
        ):
            self._mag_offline_method_clouds.pop(method_id, None)
            return

        dataset = self._mag_dataset
        if dataset is None:
            self._mag_offline_method_clouds.pop(method_id, None)
            return

        derived_records = self._compute_offline_method_records(
            plugin,
            dataset,
            cache={},
            stack=set(),
        )
        if not derived_records:
            self._mag_offline_method_clouds.pop(method_id, None)
            return

        self._mag_offline_method_clouds[method_id] = derived_records

    def _refresh_all_magnetometer_method_dataset_clouds(self, *, open_diagnostics: bool = False) -> None:
        self._ensure_magnetometer_method_state()
        for method_id in list(self._mag_methods):
            self._recompute_magnetometer_method_dataset_cloud(method_id, open_diagnostics=open_diagnostics)
        self._refresh_magnetometer_methods_ui()
        self._refresh_magnetometer_method_clouds_ui()

    def _restore_magnetometer_settings(self) -> None:
        self._ensure_magnetometer_method_state()
        if not hasattr(self, "magnetometer_tab"):
            return
        saved_state = getattr(self, "_mag_saved_state", {})
        if not isinstance(saved_state, dict) or not saved_state:
            self._refresh_magnetometer_method_clouds_ui()
            return

        saved_sources = saved_state.get("sources", {})
        if isinstance(saved_sources, dict):
            for source_id, source_state in saved_sources.items():
                if source_id not in self._mag_sources or not isinstance(source_state, dict):
                    continue
                self._mag_sources[source_id]["show"] = bool(source_state.get("show", self._mag_sources[source_id]["show"]))
                self._mag_sources[source_id]["record"] = bool(source_state.get("record", self._mag_sources[source_id]["record"]))

        saved_selected_source = str(saved_state.get("selected_source_id", "")).strip()
        if saved_selected_source in self._mag_sources:
            self._mag_selected_source_id = saved_selected_source

        dataset_paths = [
            str(path).strip()
            for path in saved_state.get("datasets", [])
            if isinstance(path, str) and os.path.isfile(path)
        ]
        if dataset_paths:
            self._load_magnetometer_dataset_paths(dataset_paths, show_errors=False)

        active_dataset_path = str(saved_state.get("active_dataset_path", "")).strip()
        if active_dataset_path:
            for dataset in self._mag_datasets:
                if dataset.source_path == active_dataset_path:
                    self._set_active_magnetometer_dataset(dataset)
                    break

        plugins_by_path: dict[str, LoadedMethodPlugin] = {}
        for entry in saved_state.get("methods", []):
            if not isinstance(entry, dict):
                continue
            file_path = str(entry.get("file_path", "")).strip()
            if not file_path or not os.path.isfile(file_path):
                continue
            self._remember_dialog_dir("plugin", file_path)
            plugin = self._register_magnetometer_method(load_method_plugin(file_path))
            plugins_by_path[plugin.file_path] = plugin
            plugin.show_enabled = bool(entry.get("show", False))
            saved_bindings = entry.get("bindings", {})
            if isinstance(saved_bindings, dict):
                plugin.stream_bindings[SCOPE_CALIBRATE] = {
                    str(slot): str(producer_id)
                    for slot, producer_id in saved_bindings.get(SCOPE_CALIBRATE, {}).items()
                    if str(slot).strip() and str(producer_id).strip()
                }
                plugin.stream_bindings[SCOPE_PROCESS] = {
                    str(slot): str(producer_id)
                    for slot, producer_id in saved_bindings.get(SCOPE_PROCESS, {}).items()
                    if str(slot).strip() and str(producer_id).strip()
                }

            params_profile_path = str(entry.get("params_profile_path", "")).strip()
            if params_profile_path and os.path.isfile(params_profile_path) and plugin.can_load_params():
                result = load_method_params(plugin, params_profile_path)
                self._apply_magnetometer_param_io_result(
                    plugin.method_id,
                    result,
                    action="load",
                    path=params_profile_path,
                    show_warnings=False,
                    open_diagnostics=False,
                )
            if bool(entry.get("realtime_enabled", False)) and plugin.can_enable_realtime():
                plugin.realtime_enabled = True
            default_record_state = bool(entry.get("realtime_enabled", False))
            plugin.record_enabled = bool(entry.get("record", default_record_state) and plugin.realtime_enabled)

        selected_method_path = str(saved_state.get("selected_method_path", "")).strip()
        selected_plugin = plugins_by_path.get(selected_method_path)
        if selected_plugin is not None:
            self._mag_selected_method_id = selected_plugin.method_id

        primary_heading = saved_state.get("primary_heading", {})
        if isinstance(primary_heading, dict) and primary_heading.get("kind") == "method":
            file_path = str(primary_heading.get("file_path", "")).strip()
            plugin = plugins_by_path.get(file_path)
            if plugin is not None:
                self._mag_primary_heading_stream_id = plugin.method_id

        view_state = saved_state.get("view_state", {})
        if isinstance(view_state, dict):
            self.magnetometer_tab.apply_view_state(view_state)

        self._refresh_magnetometer_sources_ui()
        self._refresh_magnetometer_methods_ui()
        self._refresh_all_magnetometer_method_dataset_clouds()
        self._refresh_magnetometer_heading_routing_ui()
        self._refresh_magnetometer_metrics_ui()

    def _ensure_magnetometer_calibration_state(self) -> None:
        if not hasattr(self, "_mag_calibration_results"):
            self._mag_calibration_results = queue.Queue()
        if not hasattr(self, "_mag_calibration_jobs"):
            self._mag_calibration_jobs = {}

    def _refresh_magnetometer_sources_ui(self) -> None:
        self._ensure_magnetometer_source_state()
        magnetometer_tab = getattr(self, "magnetometer_tab", None)
        if magnetometer_tab is None:
            return
        magnetometer_tab.set_source_states(self._mag_sources, self._mag_selected_source_id)
        self._refresh_magnetometer_heading_routing_ui()

    def _refresh_magnetometer_methods_ui(self) -> None:
        self._ensure_magnetometer_method_state()
        self._ensure_magnetometer_calibration_state()
        self._update_magnetometer_method_routing_state()
        magnetometer_tab = getattr(self, "magnetometer_tab", None)
        if magnetometer_tab is None:
            return
        method_states = {
            method_id: {
                "name": plugin.name,
                "version": plugin.version,
                "status": plugin.display_status(),
                "status_text": plugin.display_status_text(),
                "progress": plugin.calibration_progress if (plugin.is_calibrating or plugin.calibration_params is not None) else 0.0,
                "show": plugin.show_enabled,
                "show_enabled": plugin.can_toggle_show(),
                "can_calibrate": self._can_start_magnetometer_calibration(plugin),
                "can_load_params": plugin.can_load_params(),
                "can_save_params": plugin.can_save_params(),
                "realtime_enabled": plugin.realtime_enabled,
                "can_enable_realtime": plugin.can_enable_realtime(),
                "can_disable_realtime": plugin.can_disable_realtime(),
                "record": plugin.record_enabled,
                "can_record": plugin.can_toggle_record(),
                "stream_id": plugin.derived_stream_id or f"derived_{plugin.method_id}",
                "live_output": dict(plugin.last_output) if plugin.last_output is not None else None,
            }
            for method_id, plugin in self._mag_methods.items()
        }
        magnetometer_tab.set_method_states(method_states, self._mag_selected_method_id)
        selected = self._mag_methods.get(self._mag_selected_method_id or "")
        if selected is None:
            magnetometer_tab.update_selected_method_details(
                name="-",
                version="-",
                path="-",
                status="-",
                capabilities="-",
            )
            magnetometer_tab.set_selected_method_actions(
                can_calibrate=False,
                can_load_params=False,
                can_save_params=False,
                can_clear_params=False,
                can_enable_realtime=False,
                can_disable_realtime=False,
            )
            return
        magnetometer_tab.update_selected_method_details(
            name=selected.name,
            version=selected.version,
            path=selected.file_path,
            status=selected.display_status_text(),
            capabilities=selected.capabilities_label(),
        )
        magnetometer_tab.set_selected_method_actions(
            can_calibrate=self._can_start_magnetometer_calibration(selected),
            can_load_params=selected.can_load_params(),
            can_save_params=selected.can_save_params(),
            can_clear_params=selected.can_clear_params(),
            can_enable_realtime=selected.can_enable_realtime(),
            can_disable_realtime=selected.can_disable_realtime(),
        )
        self._refresh_magnetometer_heading_routing_ui()

    def _available_magnetometer_heading_streams(self) -> list[tuple[str, str]]:
        self._ensure_magnetometer_method_state()
        streams: list[tuple[str, str]] = [("raw_heading", "Raw Heading")]
        for method_id, plugin in self._mag_methods.items():
            if plugin.realtime_enabled or plugin.last_output is not None:
                streams.append((method_id, plugin.name))
        return streams

    def _refresh_magnetometer_heading_routing_ui(self) -> None:
        self._ensure_magnetometer_method_state()
        magnetometer_tab = getattr(self, "magnetometer_tab", None)
        if magnetometer_tab is None:
            return
        stream_choices = self._available_magnetometer_heading_streams()
        available_ids = {stream_id for stream_id, _ in stream_choices}
        if self._mag_primary_heading_stream_id not in available_ids:
            self._mag_primary_heading_stream_id = "raw_heading"
        magnetometer_tab.set_heading_routing(
            stream_choices=stream_choices,
            primary_stream_id=self._mag_primary_heading_stream_id,
        )
        if self._mag_last_live_snapshot is not None:
            selected_heading, selected_source_label = self._resolve_magnetometer_selected_output(
                self._mag_last_live_snapshot.get("raw_heading"),
                self._mag_last_live_snapshot.get("derived_streams", {}),
            )
            magnetometer_tab.set_primary_output_display(
                heading=selected_heading,
                source_label=selected_source_label,
            )

    def _dataset_dialog_label(self, dataset: Dataset) -> str:
        summary = dataset.summary()
        return f"{summary['name']}  [{summary['row_count']} rows, {summary['time_range']}]"

    def _dataset_choice_labels(self) -> list[str]:
        seen: dict[str, int] = {}
        labels: list[str] = []
        for dataset in self._mag_datasets:
            base_name = dataset.summary()["name"]
            count = seen.get(base_name, 0) + 1
            seen[base_name] = count
            labels.append(base_name if count == 1 else f"{base_name} ({count})")
        return labels

    def _rebuild_magnetometer_dataset_table(self) -> None:
        magnetometer_tab = getattr(self, "magnetometer_tab", None)
        if magnetometer_tab is None:
            return
        if self._mag_dataset is None:
            magnetometer_tab.clear_dataset_rows()
            return
        magnetometer_tab.set_dataset_records(self._mag_dataset.records)

    def _set_active_magnetometer_dataset(self, dataset: Dataset | None) -> None:
        self._mag_dataset = dataset
        self._mag_dataset_export_path = "" if dataset is None or dataset.source_path is None else dataset.source_path
        shared_dataset = self._find_shared_imu_dataset(dataset)
        if shared_dataset is not None:
            self._imu_dataset = shared_dataset
            self._imu_dataset_export_path = shared_dataset.source_path or ""
        self._rebuild_magnetometer_dataset_table()
        self._refresh_magnetometer_dataset_ui()
        self._refresh_all_magnetometer_method_dataset_clouds()
        self._refresh_magnetometer_metrics_ui()

    def _refresh_magnetometer_dataset_ui(self) -> None:
        magnetometer_tab = getattr(self, "magnetometer_tab", None)
        if magnetometer_tab is None:
            return
        dataset_labels = self._dataset_choice_labels()
        active_index = None
        if self._mag_dataset is not None:
            try:
                active_index = self._mag_datasets.index(self._mag_dataset)
            except ValueError:
                active_index = None
        magnetometer_tab.set_dataset_choices(dataset_labels, active_index)
        if self._mag_dataset is None:
            magnetometer_tab.update_dataset_summary(name="—", row_count="0", source_count="0", time_range="—")
            magnetometer_tab.set_recording_state(
                is_recording=self._mag_recording_active,
                can_save=False,
                has_dataset=False,
                can_concatenate=len(self._mag_datasets) >= 2,
            )
            magnetometer_tab.set_clear_dataset_enabled(False)
            magnetometer_tab.clear_dataset_rows()
            return
        summary = self._mag_dataset.summary()
        active_name = summary["name"] if active_index is None else dataset_labels[active_index]
        magnetometer_tab.update_dataset_summary(
            name=active_name,
            row_count=summary["row_count"],
            source_count=summary["source_count"],
            time_range=summary["time_range"],
        )
        can_save = bool(self._mag_dataset.records) and not self._mag_recording_active
        magnetometer_tab.set_recording_state(
            is_recording=self._mag_recording_active,
            can_save=can_save,
            has_dataset=bool(self._mag_dataset.records),
            can_concatenate=len(self._mag_datasets) >= 2,
        )
        magnetometer_tab.set_clear_dataset_enabled(bool(self._mag_datasets) and not self._mag_recording_active)
        self._mag_last_dataset_ui_refresh_s = time.monotonic()

    def _refresh_magnetometer_dataset_ui_if_due(self, *, force: bool = False) -> None:
        if not hasattr(self, "_mag_last_dataset_ui_refresh_s"):
            self._mag_last_dataset_ui_refresh_s = 0.0
        if not hasattr(self, "_mag_dataset_ui_refresh_interval_s"):
            self._mag_dataset_ui_refresh_interval_s = 0.2
        now = time.monotonic()
        if force or (now - self._mag_last_dataset_ui_refresh_s) >= self._mag_dataset_ui_refresh_interval_s:
            self._refresh_magnetometer_dataset_ui()

    def _ensure_shared_imu_state(self) -> None:
        if not hasattr(self, "_imu_datasets"):
            self._imu_datasets = []
        if not hasattr(self, "_imu_dataset"):
            self._imu_dataset = None
        if not hasattr(self, "_imu_recording_active"):
            self._imu_recording_active = False
        if not hasattr(self, "_imu_dataset_export_path"):
            self._imu_dataset_export_path = ""
        if not hasattr(self, "_mag_datasets"):
            self._mag_datasets = []
        if not hasattr(self, "_mag_dataset"):
            self._mag_dataset = None
        if not hasattr(self, "_mag_recording_active"):
            self._mag_recording_active = False
        if not hasattr(self, "_mag_dataset_export_path"):
            self._mag_dataset_export_path = ""
        self._ensure_magnetometer_source_state()
        self._ensure_magnetometer_method_state()
        self._ensure_sensor_method_state("acc")
        self._ensure_sensor_method_state("gyro")

    def _make_imu_dataset_name(self) -> str:
        self._ensure_shared_imu_state()
        return f"imu_record_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

    def _shared_projection_key(self, dataset: Any | None) -> tuple[str, str]:
        if dataset is None:
            return ("", "")
        metadata = getattr(dataset, "metadata", {})
        if not isinstance(metadata, dict):
            return ("", "")
        if metadata.get("shared_dataset_format") != "imu_dataset":
            return ("", "")
        return (
            str(metadata.get("shared_source_path", "") or ""),
            str(metadata.get("shared_dataset_name", "") or ""),
        )

    def _is_shared_projection_dataset(self, dataset: Any | None) -> bool:
        return self._shared_projection_key(dataset) != ("", "")

    def _find_shared_imu_dataset(self, dataset: Any | None) -> ImuDataset | None:
        self._ensure_shared_imu_state()
        source_path, dataset_name = self._shared_projection_key(dataset)
        if self._imu_dataset is not None:
            current_summary = self._imu_dataset.summary()["name"]
            if (source_path and self._imu_dataset.source_path == source_path) or (
                not source_path and current_summary == dataset_name
            ):
                return self._imu_dataset
        for shared_dataset in self._imu_datasets:
            summary_name = shared_dataset.summary()["name"]
            if (source_path and shared_dataset.source_path == source_path) or (
                not source_path and summary_name == dataset_name
            ):
                return shared_dataset
        return None

    def _shared_projection_metadata(self, shared_dataset: ImuDataset) -> dict[str, Any]:
        metadata = dict(shared_dataset.metadata)
        metadata["shared_source_path"] = shared_dataset.source_path or ""
        metadata["shared_dataset_name"] = shared_dataset.summary()["name"]
        metadata["shared_dataset_format"] = "imu_dataset"
        return metadata

    def _update_projection_metadata_from_shared(self, shared_dataset: ImuDataset, dataset: Any | None) -> None:
        if dataset is None:
            return
        dataset.source_path = shared_dataset.source_path
        metadata = dict(getattr(dataset, "metadata", {}))
        metadata.update(self._shared_projection_metadata(shared_dataset))
        dataset.metadata = metadata

    def _bind_shared_imu_path(self, shared_dataset: ImuDataset, path: str) -> None:
        self._ensure_shared_imu_state()
        previous_source = shared_dataset.source_path or ""
        previous_name = shared_dataset.summary()["name"]
        shared_dataset.source_path = path
        self._imu_dataset_export_path = path
        for collection in (self._mag_datasets, self._acc_datasets, self._gyro_datasets):
            for dataset in collection:
                metadata = getattr(dataset, "metadata", {})
                if not isinstance(metadata, dict) or metadata.get("shared_dataset_format") != "imu_dataset":
                    continue
                if str(metadata.get("shared_dataset_name", "")) != previous_name:
                    continue
                if str(metadata.get("shared_source_path", "")) != previous_source:
                    continue
                self._update_projection_metadata_from_shared(shared_dataset, dataset)

    def _build_imu_recording_metadata(self) -> dict[str, Any]:
        return {
            "schema_version": "1",
            "saved_at": datetime.now().isoformat(timespec="seconds"),
            "magnetometer": self._build_magnetometer_recording_metadata(),
            "accelerometer": self._build_sensor_recording_metadata("acc"),
            "gyroscope": self._build_sensor_recording_metadata("gyro"),
        }

    def _update_imu_dataset_recording_metadata(self, dataset: ImuDataset | None = None) -> None:
        self._ensure_shared_imu_state()
        target_dataset = self._imu_dataset if dataset is None else dataset
        if target_dataset is None:
            return
        target_dataset.metadata = self._build_imu_recording_metadata()
        self._update_projection_metadata_from_shared(target_dataset, self._mag_dataset)
        self._update_projection_metadata_from_shared(target_dataset, self._acc_dataset)
        self._update_projection_metadata_from_shared(target_dataset, self._gyro_dataset)

    def _activate_shared_imu_bundle(self, shared_dataset: ImuDataset) -> None:
        self._ensure_shared_imu_state()
        if shared_dataset not in self._imu_datasets:
            self._imu_datasets.append(shared_dataset)
        self._imu_dataset = shared_dataset
        self._imu_dataset_export_path = shared_dataset.source_path or ""

        mag_dataset = shared_dataset.project_magnetometer_dataset()
        acc_dataset = shared_dataset.project_accelerometer_dataset()
        gyro_dataset = shared_dataset.project_gyroscope_dataset()
        self._mag_datasets.append(mag_dataset)
        self._acc_datasets.append(acc_dataset)
        self._gyro_datasets.append(gyro_dataset)
        self._set_active_magnetometer_dataset(mag_dataset)
        self._set_active_sensor_dataset("acc", acc_dataset)
        self._set_active_sensor_dataset("gyro", gyro_dataset)

    def _load_shared_imu_dataset_path(self, path: str) -> None:
        self._ensure_shared_imu_state()
        for shared_dataset in self._imu_datasets:
            if shared_dataset.source_path != path:
                continue
            self._imu_dataset = shared_dataset
            for dataset in self._mag_datasets:
                if self._shared_projection_key(dataset) == (path, shared_dataset.summary()["name"]):
                    self._set_active_magnetometer_dataset(dataset)
                    break
            for dataset in self._acc_datasets:
                if self._shared_projection_key(dataset) == (path, shared_dataset.summary()["name"]):
                    self._set_active_sensor_dataset("acc", dataset)
                    break
            for dataset in self._gyro_datasets:
                if self._shared_projection_key(dataset) == (path, shared_dataset.summary()["name"]):
                    self._set_active_sensor_dataset("gyro", dataset)
                    break
            self._imu_dataset_export_path = path
            return
        shared_dataset = ImuDataset.from_csv(path)
        self._activate_shared_imu_bundle(shared_dataset)
        self._remember_dialog_dir("dataset", path)
        self._remember_sensor_dialog_dir("acc", "dataset", path)
        self._remember_sensor_dialog_dir("gyro", "dataset", path)

    def _convert_magnetometer_record_to_imu(self, record: SampleRecord) -> ImuSampleRecord:
        return ImuSampleRecord(
            stream_id=record.stream_id,
            stream_type=record.stream_type,
            producer_name=record.producer_name,
            producer_version=record.producer_version,
            timestamp_mcu=record.timestamp_mcu,
            timestamp_pc_rx=record.timestamp_pc_rx,
            timestamp_pc_est=record.timestamp_pc_est,
            mag_x=record.mag_x,
            mag_y=record.mag_y,
            mag_z=record.mag_z,
            heading=record.heading,
            flags=record.flags,
        )

    def _convert_accelerometer_record_to_imu(self, record: AccelerometerSampleRecord) -> ImuSampleRecord:
        return ImuSampleRecord(
            stream_id=record.stream_id,
            stream_type=record.stream_type,
            producer_name=record.producer_name,
            producer_version=record.producer_version,
            timestamp_mcu=record.timestamp_mcu,
            timestamp_pc_rx=record.timestamp_pc_rx,
            timestamp_pc_est=record.timestamp_pc_est,
            acc_x=record.acc_x,
            acc_y=record.acc_y,
            acc_z=record.acc_z,
            roll_deg=record.roll_deg,
            pitch_deg=record.pitch_deg,
            flags=record.flags,
        )

    def _convert_gyroscope_record_to_imu(self, record: GyroscopeSampleRecord) -> ImuSampleRecord:
        return ImuSampleRecord(
            stream_id=record.stream_id,
            stream_type=record.stream_type,
            producer_name=record.producer_name,
            producer_version=record.producer_version,
            timestamp_mcu=record.timestamp_mcu,
            timestamp_pc_rx=record.timestamp_pc_rx,
            timestamp_pc_est=record.timestamp_pc_est,
            gyro_x=record.gyro_x,
            gyro_y=record.gyro_y,
            gyro_z=record.gyro_z,
            rate_mag=record.rate_mag,
            flags=record.flags,
        )

    def _append_imu_shared_record(self, record: ImuSampleRecord) -> None:
        self._ensure_shared_imu_state()
        if not self._imu_recording_active or self._imu_dataset is None:
            return
        self._imu_dataset.append(record)

    def _build_magnetometer_recording_metadata(self) -> dict[str, Any]:
        self._ensure_magnetometer_source_state()
        self._ensure_magnetometer_method_state()
        producers = self._magnetometer_calibration_producers()
        source_entries = [
            {
                "source_id": source_id,
                "title": str(source.get("title", source_id)),
                "show": bool(source.get("show", False)),
                "record": bool(source.get("record", False)),
            }
            for source_id, source in self._mag_sources.items()
        ]
        filter_entries = [
            {
                "method_id": method_id,
                "name": plugin.name,
                "version": plugin.version,
                "file_path": plugin.file_path,
                "stream_id": plugin.derived_stream_id or f"derived_{plugin.method_id}",
                "show": bool(plugin.show_enabled),
                "record": bool(plugin.record_enabled),
                "realtime_enabled": bool(plugin.realtime_enabled),
                "params": plugin.calibration_params,
                "stream_bindings": {
                    SCOPE_CALIBRATE: self._effective_magnetometer_stream_bindings(
                        plugin,
                        scope=SCOPE_CALIBRATE,
                        producers=producers,
                    ),
                    SCOPE_PROCESS: self._effective_magnetometer_stream_bindings(
                        plugin,
                        scope=SCOPE_PROCESS,
                        producers=producers,
                    ),
                },
            }
            for method_id, plugin in self._mag_methods.items()
        ]
        recorded_streams = [
            {
                "stream_id": source_id,
                "kind": "source",
                "title": str(source.get("title", source_id)),
                "stream_type": "raw",
                "flags": "heading_only" if source_id == "raw_heading" else "",
            }
            for source_id, source in self._mag_sources.items()
            if bool(source.get("record", False))
        ]
        recorded_streams.extend(
            {
                "stream_id": plugin.derived_stream_id or f"derived_{plugin.method_id}",
                "kind": "method",
                "method_id": method_id,
                "name": plugin.name,
                "version": plugin.version,
                "file_path": plugin.file_path,
                "realtime_enabled": bool(plugin.realtime_enabled),
                "params": plugin.calibration_params,
            }
            for method_id, plugin in self._mag_methods.items()
            if bool(plugin.record_enabled)
        )
        return {
            "schema_version": "1",
            "saved_at": datetime.now().isoformat(timespec="seconds"),
            "sources": source_entries,
            "filters": filter_entries,
            "recorded_streams": recorded_streams,
            "recorded_method_ids": [
                entry["method_id"]
                for entry in filter_entries
                if bool(entry.get("record", False))
            ],
            "method_bindings": [
                {
                    "file_path": plugin.file_path,
                    "name": plugin.name,
                    "bindings": {
                        SCOPE_CALIBRATE: self._effective_magnetometer_stream_bindings(
                            plugin,
                            scope=SCOPE_CALIBRATE,
                            producers=producers,
                        ),
                        SCOPE_PROCESS: self._effective_magnetometer_stream_bindings(
                            plugin,
                            scope=SCOPE_PROCESS,
                            producers=producers,
                        ),
                    },
                }
                for plugin in self._mag_methods.values()
            ],
        }

    def _update_magnetometer_dataset_recording_metadata(self, dataset: Dataset | None = None) -> None:
        target_dataset = self._mag_dataset if dataset is None else dataset
        if target_dataset is None:
            return
        target_dataset.metadata = self._build_magnetometer_recording_metadata()
        if self._imu_dataset is not None and (self._imu_recording_active or self._is_shared_projection_dataset(target_dataset)):
            self._update_imu_dataset_recording_metadata()

    def _start_shared_imu_recording(self, *, requested_name: str | None = None) -> None:
        self._ensure_shared_imu_state()
        if self._imu_recording_active:
            return
        shared_dataset = ImuDataset(requested_name or self._make_imu_dataset_name())
        self._imu_recording_active = True
        self._mag_recording_active = True
        self._acc_recording_active = True
        self._gyro_recording_active = True
        self._activate_shared_imu_bundle(shared_dataset)
        self._imu_dataset_export_path = ""
        self._update_imu_dataset_recording_metadata(shared_dataset)
        self._update_magnetometer_dataset_recording_metadata(self._mag_dataset)
        self._update_sensor_dataset_recording_metadata("acc", self._acc_dataset)
        self._update_sensor_dataset_recording_metadata("gyro", self._gyro_dataset)
        self._refresh_magnetometer_dataset_ui()
        self._refresh_sensor_dataset_ui("acc")
        self._refresh_sensor_dataset_ui("gyro")

    def _stop_shared_imu_recording(self) -> None:
        self._ensure_shared_imu_state()
        if not self._imu_recording_active:
            return
        self._imu_recording_active = False
        self._mag_recording_active = False
        self._acc_recording_active = False
        self._gyro_recording_active = False
        self._update_imu_dataset_recording_metadata()
        self._update_magnetometer_dataset_recording_metadata()
        self._update_sensor_dataset_recording_metadata("acc")
        self._update_sensor_dataset_recording_metadata("gyro")
        self._refresh_magnetometer_dataset_ui()
        self._refresh_sensor_dataset_ui("acc")
        self._refresh_sensor_dataset_ui("gyro")

    def _on_magnetometer_start_record(self) -> None:
        requested_name = self._make_magnetometer_dataset_name() if hasattr(self, "_make_magnetometer_dataset_name") else None
        self._start_shared_imu_recording(requested_name=requested_name)

    def _on_magnetometer_stop_record(self) -> None:
        self._stop_shared_imu_recording()

    def _record_magnetometer_sample(
        self,
        *,
        timestamp_mcu: int,
        timestamp_pc_rx: int,
        timestamp_pc_est: int | None,
        mx: float,
        my: float,
        mz: float,
        derived_records: Sequence[SampleRecord] | None = None,
    ) -> None:
        self._ensure_magnetometer_source_state()
        if not self._mag_recording_active or self._mag_dataset is None:
            return
        heading = compute_raw_heading_deg(mx, my)
        next_row_id = len(self._mag_dataset.records)
        magnetometer_tab = getattr(self, "magnetometer_tab", None)
        appended = False

        shared_recording_active = bool(getattr(self, "_imu_recording_active", False))
        record_raw_magnetometer_local = bool(self._mag_sources["raw_magnetometer"]["record"])
        record_raw_heading_local = bool(self._mag_sources["raw_heading"]["record"])
        if record_raw_magnetometer_local or shared_recording_active:
            record = SampleRecord(
                stream_id="raw_magnetometer",
                stream_type="raw",
                producer_name="Raw Magnetometer",
                producer_version="builtin",
                timestamp_mcu=timestamp_mcu,
                timestamp_pc_rx=timestamp_pc_rx,
                timestamp_pc_est=timestamp_pc_est,
                mag_x=mx,
                mag_y=my,
                mag_z=mz,
                heading=heading,
                flags="",
            )
            self._append_imu_shared_record(self._convert_magnetometer_record_to_imu(record))
            if record_raw_magnetometer_local:
                next_row_id += 1
                self._mag_dataset.append(record)
            if record_raw_magnetometer_local and magnetometer_tab is not None:
                magnetometer_tab.append_dataset_record(next_row_id, record)
                appended = True

        if record_raw_heading_local or shared_recording_active:
            record = SampleRecord(
                stream_id="raw_heading",
                stream_type="raw",
                producer_name="Raw Heading",
                producer_version="builtin",
                timestamp_mcu=timestamp_mcu,
                timestamp_pc_rx=timestamp_pc_rx,
                timestamp_pc_est=timestamp_pc_est,
                mag_x=mx,
                mag_y=my,
                mag_z=mz,
                heading=heading,
                flags="heading_only",
            )
            self._append_imu_shared_record(self._convert_magnetometer_record_to_imu(record))
            if record_raw_heading_local:
                next_row_id += 1
                self._mag_dataset.append(record)
            if record_raw_heading_local and magnetometer_tab is not None:
                magnetometer_tab.append_dataset_record(next_row_id, record)
                appended = True

        for record in derived_records or ():
            next_row_id += 1
            self._mag_dataset.append(record)
            self._append_imu_shared_record(self._convert_magnetometer_record_to_imu(record))
            if magnetometer_tab is not None:
                magnetometer_tab.append_dataset_record(next_row_id, record)
            appended = True

        if not appended:
            return
        self._refresh_magnetometer_dataset_ui_if_due(force=not self._mag_recording_active)

    def _choose_magnetometer_dataset_load_path(self) -> str:
        path = filedialog.askopenfilename(
            title="Load magnetometer dataset",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            initialdir=self._dialog_initialdir("dataset", fallback_path=self._mag_dataset_export_path),
            parent=self.root,
        )
        self._remember_dialog_dir("dataset", path)
        return path

    def _choose_magnetometer_dataset_load_paths(self) -> tuple[str, ...]:
        paths = filedialog.askopenfilenames(
            title="Load magnetometer datasets",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            initialdir=self._dialog_initialdir("dataset", fallback_path=self._mag_dataset_export_path),
            parent=self.root,
        )
        if paths:
            self._remember_dialog_dir("dataset", paths[0])
        return paths

    def _choose_magnetometer_dataset_path(self) -> str:
        path = filedialog.asksaveasfilename(
            title="Save magnetometer dataset",
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            initialdir=self._dialog_initialdir("dataset", fallback_path=self._mag_dataset_export_path),
            parent=self.root,
        )
        self._remember_dialog_dir("dataset", path)
        return path

    def _choose_magnetometer_params_load_path(self) -> str:
        path = filedialog.askopenfilename(
            title="Load magnetometer params",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            initialdir=self._dialog_initialdir("params"),
            parent=self.root,
        )
        self._remember_dialog_dir("params", path)
        return path

    def _choose_magnetometer_params_save_path(self) -> str:
        path = filedialog.asksaveasfilename(
            title="Save magnetometer params",
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            initialdir=self._dialog_initialdir("params"),
            parent=self.root,
        )
        self._remember_dialog_dir("params", path)
        return path

    def _choose_magnetometer_metrics_path(self) -> str:
        path = filedialog.asksaveasfilename(
            title="Export magnetometer metrics",
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            initialdir=self._dialog_initialdir("metrics"),
            parent=self.root,
        )
        self._remember_dialog_dir("metrics", path)
        return path

    def _selected_magnetometer_plugin(self) -> LoadedMethodPlugin | None:
        self._ensure_magnetometer_method_state()
        if self._mag_selected_method_id is None:
            return None
        return self._mag_methods.get(self._mag_selected_method_id)

    def _refresh_magnetometer_metrics_ui(self) -> None:
        magnetometer_tab = getattr(self, "magnetometer_tab", None)
        if magnetometer_tab is None:
            return
        report = compute_metrics_report(getattr(self, "_mag_dataset", None), self._selected_magnetometer_plugin())
        self._mag_metrics_report = report
        rows = [
            {
                "metric": row.label,
                "value": row.value_text(),
                "units": row.units,
                "status": row.status,
                "notes": row.notes,
            }
            for row in report.rows
        ]
        magnetometer_tab.set_metrics_report(
            rows=rows,
            summary_text=report.summary_text,
            can_export=bool(report.rows),
        )

    def _on_magnetometer_export_metrics_csv(self) -> None:
        report = self._mag_metrics_report
        if report is None or not report.rows:
            self._show_info("Magnetometer Metrics", "There are no metrics to export yet.")
            return
        path = self._choose_magnetometer_metrics_path()
        if not path:
            return
        try:
            report.export_csv(path)
        except Exception as exc:
            self._show_error("Magnetometer Metrics", f"Failed to export metrics: {exc}")

    def _load_magnetometer_dataset_paths(self, paths: Sequence[str], *, show_errors: bool = True) -> int:
        loaded: list[Dataset] = []
        errors: list[str] = []
        for path in paths:
            try:
                if ImuDataset.is_imu_csv(path):
                    self._load_shared_imu_dataset_path(path)
                    loaded.append(self._mag_dataset)
                    continue
                dataset = Dataset.from_csv(path)
            except Exception as exc:
                errors.append(f"{os.path.basename(path)}: {exc}")
                continue
            self._mag_datasets.append(dataset)
            loaded.append(dataset)

        if loaded:
            self._set_active_magnetometer_dataset(loaded[-1])
            self._remember_dialog_dir("dataset", loaded[-1].source_path or "")
            self._save_settings()
        if errors and show_errors:
            self._show_error("Magnetometer Dataset", "Failed to load:\n" + "\n".join(errors))
        return len(loaded)

    def _save_magnetometer_dataset_to_path(self, path: str) -> bool:
        if self._mag_dataset is None or not self._mag_dataset.records:
            self._show_info("Magnetometer Dataset", "There is no recorded dataset to save.")
            return False
        shared_dataset = self._find_shared_imu_dataset(self._mag_dataset)
        if shared_dataset is not None:
            self._update_imu_dataset_recording_metadata(shared_dataset)
            try:
                shared_dataset.to_csv(path)
            except Exception as exc:
                self._show_error("Magnetometer Dataset", f"Failed to save dataset: {exc}")
                return False
            self._bind_shared_imu_path(shared_dataset, path)
            self._mag_dataset_export_path = path
            self._remember_dialog_dir("dataset", path)
            self._refresh_magnetometer_dataset_ui()
            self._refresh_sensor_dataset_ui("acc")
            self._refresh_sensor_dataset_ui("gyro")
            self._save_settings()
            return True
        if self._mag_dataset.source_path is None or bool(getattr(self, "_mag_recording_active", False)):
            self._update_magnetometer_dataset_recording_metadata()
        try:
            self._mag_dataset.to_csv(path)
        except Exception as exc:
            self._show_error("Magnetometer Dataset", f"Failed to save dataset: {exc}")
            return False
        self._mag_dataset_export_path = path
        self._remember_dialog_dir("dataset", path)
        self._refresh_magnetometer_dataset_ui()
        self._save_settings()
        return True

    def _on_magnetometer_load_csv(self) -> None:
        if self._mag_recording_active:
            return
        path = self._choose_magnetometer_dataset_load_path()
        if not path:
            return
        self._load_magnetometer_dataset_paths((path,))

    def _on_magnetometer_load_multiple(self) -> None:
        if self._mag_recording_active:
            return
        paths = self._choose_magnetometer_dataset_load_paths()
        if not paths:
            return
        self._load_magnetometer_dataset_paths(paths)

    def _on_select_magnetometer_dataset(self, index: int) -> None:
        if self._mag_recording_active:
            return
        if 0 <= index < len(self._mag_datasets):
            self._set_active_magnetometer_dataset(self._mag_datasets[index])
            self._save_settings()

    def _on_select_magnetometer_source(self, source_id: str) -> None:
        self._ensure_magnetometer_source_state()
        if source_id not in self._mag_sources:
            return
        self._mag_selected_source_id = source_id
        self._refresh_magnetometer_sources_ui()
        if bool(getattr(self, "_mag_recording_active", False)):
            self._update_magnetometer_dataset_recording_metadata()
        self._save_settings()

    def _on_toggle_magnetometer_source_show(self, source_id: str, enabled: bool) -> None:
        self._ensure_magnetometer_source_state()
        if source_id not in self._mag_sources:
            return
        self._mag_sources[source_id]["show"] = bool(enabled)
        self._refresh_magnetometer_sources_ui()
        if bool(getattr(self, "_mag_recording_active", False)):
            self._update_magnetometer_dataset_recording_metadata()
        self._save_settings()

    def _on_toggle_magnetometer_source_record(self, source_id: str, enabled: bool) -> None:
        self._ensure_magnetometer_source_state()
        if source_id not in self._mag_sources:
            return
        self._mag_sources[source_id]["record"] = bool(enabled)
        self._refresh_magnetometer_sources_ui()
        if bool(getattr(self, "_mag_recording_active", False)):
            self._update_magnetometer_dataset_recording_metadata()
        self._save_settings()

    def _on_toggle_magnetometer_method_show(self, method_id: str, enabled: bool) -> None:
        self._ensure_magnetometer_method_state()
        plugin = self._mag_methods.get(method_id)
        if plugin is None:
            return
        plugin.show_enabled = bool(enabled)
        stream_state = self._mag_latest_derived_streams.get(method_id)
        if stream_state is not None:
            stream_state["show"] = plugin.show_enabled
        self._refresh_magnetometer_methods_ui()
        if bool(getattr(self, "_mag_recording_active", False)):
            self._update_magnetometer_dataset_recording_metadata()
        self._save_settings()

    def _on_enable_magnetometer_method_realtime(self, method_id: str) -> None:
        self._ensure_magnetometer_method_state()
        plugin = self._mag_methods.get(method_id)
        if plugin is None:
            return
        if not plugin.can_enable_realtime():
            if plugin.calibration_params is None:
                plugin.process_warnings = ["Load or create calibration params before enabling realtime."]
                self._refresh_magnetometer_methods_ui()
                self._show_warning(
                    "Magnetometer Realtime",
                    f"{plugin.name} requires calibration params before realtime can be enabled.",
                )
            return
        plugin.realtime_enabled = True
        plugin.show_enabled = True
        plugin.record_enabled = True
        plugin.process_warnings = []
        self._refresh_magnetometer_methods_ui()
        if bool(getattr(self, "_mag_recording_active", False)):
            self._update_magnetometer_dataset_recording_metadata()
        self._save_settings()

    def _on_disable_magnetometer_method_realtime(self, method_id: str) -> None:
        self._ensure_magnetometer_method_state()
        plugin = self._mag_methods.get(method_id)
        if plugin is None:
            return
        plugin.realtime_enabled = False
        plugin.record_enabled = False
        plugin.last_output = None
        self._mag_latest_derived_streams.pop(method_id, None)
        for kind in self._magnetometer_method_output_kinds(plugin):
            self._calibration_stream_snapshots.pop(make_method_producer_id(plugin.file_path, kind), None)
        self._refresh_magnetometer_methods_ui()
        if bool(getattr(self, "_mag_recording_active", False)):
            self._update_magnetometer_dataset_recording_metadata()
        self._save_settings()

    def _on_toggle_magnetometer_method_record(self, method_id: str, enabled: bool) -> None:
        self._ensure_magnetometer_method_state()
        plugin = self._mag_methods.get(method_id)
        if plugin is None:
            return
        if not plugin.can_toggle_record() and enabled:
            return
        plugin.record_enabled = bool(enabled and plugin.realtime_enabled)
        self._refresh_magnetometer_methods_ui()
        if bool(getattr(self, "_mag_recording_active", False)):
            self._update_magnetometer_dataset_recording_metadata()
        self._save_settings()

    def _on_select_magnetometer_primary_heading(self, stream_id: str) -> None:
        self._ensure_magnetometer_method_state()
        available_ids = {stream_id for stream_id, _ in self._available_magnetometer_heading_streams()}
        if stream_id not in available_ids:
            return
        self._mag_primary_heading_stream_id = stream_id
        self._refresh_magnetometer_heading_routing_ui()
        magnetometer_tab = getattr(self, "magnetometer_tab", None)
        if magnetometer_tab is None or self._mag_last_live_snapshot is None:
            return
        selected_heading, selected_source_label = self._resolve_magnetometer_selected_output(
            self._mag_last_live_snapshot.get("raw_heading"),
            self._mag_last_live_snapshot.get("derived_streams", {}),
        )
        magnetometer_tab.set_primary_output_display(
            heading=selected_heading,
            source_label=selected_source_label,
        )
        self._save_settings()

    def _make_magnetometer_method_diagnostics(
        self,
        plugin: LoadedMethodPlugin,
        *,
        last_action: str,
        error_text: str,
        traceback_text: str = "",
        warnings: Sequence[str] | None = None,
    ) -> PluginDiagnostics:
        return PluginDiagnostics(
            name=plugin.name,
            version=plugin.version,
            file_path=plugin.file_path,
            last_action=last_action,
            error_text=error_text,
            traceback_text=traceback_text,
            warnings=list(warnings or plugin.effective_warnings()),
        )

    def _snapshot_magnetometer_dataset(self, dataset: Dataset) -> Dataset:
        return Dataset(dataset.name, records=list(dataset.records), source_path=dataset.source_path)

    def _on_calibrate_magnetometer_method(self, method_id: str) -> None:
        self._ensure_magnetometer_method_state()
        self._ensure_magnetometer_calibration_state()
        plugin = self._mag_methods.get(method_id)
        if plugin is None:
            return
        if not plugin.supports_calibrate():
            self._show_warning("Magnetometer Calibration", f"{plugin.name} does not support calibration.")
            return
        if plugin.is_calibrating:
            return
        dataset = self._mag_dataset
        if dataset is None or not dataset.records:
            plugin.calibration_warnings = ["No active non-empty dataset loaded for calibration."]
            self._refresh_magnetometer_methods_ui()
            self._show_warning("Magnetometer Calibration", "Load or record a non-empty dataset first.")
            return
        self._update_magnetometer_method_routing_state()
        validation = plugin.routing_validation.get(SCOPE_CALIBRATE)
        if validation is not None and not validation.ok:
            plugin.calibration_warnings = list(validation.issues)
            self._refresh_magnetometer_methods_ui()
            self._show_warning(
                "Magnetometer Calibration",
                "Resolve method input bindings before calibration:\n" + "\n".join(validation.issues),
            )
            return

        dataset_snapshot = self._snapshot_magnetometer_dataset(dataset)
        calibration_config = self._build_magnetometer_calibration_config(plugin, dataset_snapshot)
        plugin.status = plugin.load_status
        plugin.diagnostics = None
        plugin.calibration_warnings = []
        plugin.calibration_report = ""
        plugin.params_warnings = []
        plugin.calibration_progress = 0.0
        plugin.calibration_runtime_s = None
        plugin.is_calibrating = True
        self._mag_calibration_jobs[method_id] = {
            "started_at": time.monotonic(),
            "dataset_name": dataset_snapshot.summary()["name"],
        }
        self._refresh_magnetometer_methods_ui()

        worker = threading.Thread(
            target=self._run_magnetometer_calibration_worker,
            args=(method_id, plugin, dataset_snapshot, calibration_config),
            name=f"mag-calibrate-{method_id}",
            daemon=True,
        )
        worker.start()

    def _run_magnetometer_calibration_worker(
        self,
        method_id: str,
        plugin: LoadedMethodPlugin,
        dataset: Dataset,
        calibration_config: dict[str, Any],
    ) -> None:
        result = run_method_calibration(plugin, dataset, config=calibration_config)
        self._mag_calibration_results.put((method_id, result))

    def _apply_magnetometer_calibration_result(self, method_id: str, result: CalibrationRunResult) -> None:
        self._ensure_magnetometer_method_state()
        self._ensure_magnetometer_calibration_state()
        plugin = self._mag_methods.get(method_id)
        if plugin is None:
            self._mag_calibration_jobs.pop(method_id, None)
            return

        job = self._mag_calibration_jobs.pop(method_id, None)
        plugin.is_calibrating = False
        plugin.calibration_progress = 1.0 if result.ok else 0.0
        if job is not None:
            plugin.calibration_dataset_name = str(job.get("dataset_name") or "")
            plugin.calibration_runtime_s = max(0.0, time.monotonic() - float(job.get("started_at", time.monotonic())))

        if result.ok:
            plugin.status = plugin.load_status
            plugin.diagnostics = None
            plugin.calibration_params = result.params
            plugin.calibration_warnings = list(result.warnings)
            plugin.params_warnings = []
            plugin.process_warnings = []
            plugin.calibration_report = result.report
            self._recompute_magnetometer_method_dataset_cloud(method_id, open_diagnostics=False)
            if bool(getattr(self, "_mag_recording_active", False)):
                self._update_magnetometer_dataset_recording_metadata()
        else:
            plugin.status = "error"
            plugin.calibration_warnings = list(result.warnings)
            plugin.calibration_report = result.report
            plugin.process_warnings = []
            plugin.realtime_enabled = False
            plugin.record_enabled = False
            plugin.last_output = None
            self._mag_latest_derived_streams.pop(method_id, None)
            for kind in self._magnetometer_method_output_kinds(plugin):
                self._calibration_stream_snapshots.pop(make_method_producer_id(plugin.file_path, kind), None)
            if result.diagnostics is not None:
                plugin.diagnostics = result.diagnostics
            else:
                plugin.diagnostics = self._make_magnetometer_method_diagnostics(
                    plugin,
                    last_action="calibrate",
                    error_text="Calibration failed.",
                    warnings=result.warnings,
                )
            self._mag_offline_method_clouds.pop(method_id, None)

        self._refresh_magnetometer_methods_ui()
        self._refresh_magnetometer_method_clouds_ui()
        if plugin.status == "error":
            self._open_magnetometer_method_diagnostics(method_id)
        self._refresh_magnetometer_metrics_ui()
        self._save_settings()

    def _poll_magnetometer_calibration_jobs(self) -> None:
        self._ensure_magnetometer_method_state()
        self._ensure_magnetometer_calibration_state()
        refresh_needed = False
        now = time.monotonic()
        for method_id, job in list(self._mag_calibration_jobs.items()):
            plugin = self._mag_methods.get(method_id)
            if plugin is None or not plugin.is_calibrating:
                continue
            elapsed = max(0.0, now - float(job.get("started_at", now)))
            progress = min(0.92, 0.08 + elapsed / 2.5)
            if abs(progress - plugin.calibration_progress) >= 0.02:
                plugin.calibration_progress = progress
                refresh_needed = True

        while True:
            try:
                method_id, result = self._mag_calibration_results.get_nowait()
            except queue.Empty:
                break
            self._apply_magnetometer_calibration_result(method_id, result)
            refresh_needed = False

        if refresh_needed:
            self._refresh_magnetometer_methods_ui()

    def _apply_magnetometer_param_io_result(
        self,
        method_id: str,
        result: ParamIoResult,
        *,
        action: str,
        path: str,
        show_warnings: bool = True,
        open_diagnostics: bool = True,
    ) -> None:
        self._ensure_magnetometer_method_state()
        plugin = self._mag_methods.get(method_id)
        if plugin is None:
            return

        if result.ok:
            plugin.status = plugin.load_status
            plugin.diagnostics = None
            plugin.params_warnings = list(result.warnings)
            plugin.process_warnings = []
            if action == "load":
                plugin.calibration_params = result.params
                plugin.params_profile_path = path
                plugin.calibration_report = ""
                plugin.calibration_progress = 1.0
                self._recompute_magnetometer_method_dataset_cloud(method_id, open_diagnostics=False)
                if bool(getattr(self, "_mag_recording_active", False)):
                    self._update_magnetometer_dataset_recording_metadata()
            elif action == "save" and result.params is not None:
                plugin.calibration_params = result.params
                plugin.params_profile_path = path
            self._refresh_magnetometer_methods_ui()
            self._refresh_magnetometer_method_clouds_ui()
            if result.warnings and show_warnings:
                self._show_warning(
                    "Magnetometer Params",
                    "Param profile loaded with warnings:\n" + "\n".join(result.warnings)
                    if action == "load"
                        else "Param profile saved with warnings:\n" + "\n".join(result.warnings),
                )
            self._refresh_magnetometer_metrics_ui()
            self._save_settings()
            return

        plugin.status = "error"
        if result.diagnostics is not None:
            plugin.diagnostics = result.diagnostics
        else:
            plugin.diagnostics = self._make_magnetometer_method_diagnostics(
                plugin,
                last_action=f"{action}_params",
                error_text=f"{action.title()} params failed.",
                warnings=result.warnings,
            )
        self._mag_offline_method_clouds.pop(method_id, None)
        self._refresh_magnetometer_methods_ui()
        self._refresh_magnetometer_method_clouds_ui()
        if open_diagnostics:
            self._open_magnetometer_method_diagnostics(method_id)
        self._refresh_magnetometer_metrics_ui()
        self._save_settings()

    def _on_load_magnetometer_method_params(self, method_id: str) -> None:
        self._ensure_magnetometer_method_state()
        plugin = self._mag_methods.get(method_id)
        if plugin is None:
            return
        if not plugin.can_load_params():
            self._show_warning("Magnetometer Params", f"{plugin.name} does not support param loading right now.")
            return
        path = self._choose_magnetometer_params_load_path()
        if not path:
            return
        result = load_method_params(plugin, path)
        self._apply_magnetometer_param_io_result(method_id, result, action="load", path=path)

    def _on_save_magnetometer_method_params(self, method_id: str) -> None:
        self._ensure_magnetometer_method_state()
        plugin = self._mag_methods.get(method_id)
        if plugin is None:
            return
        if not plugin.can_save_params():
            self._show_warning("Magnetometer Params", f"{plugin.name} has no runtime params available to save.")
            return
        path = self._choose_magnetometer_params_save_path()
        if not path:
            return
        result = save_method_params(plugin, path, plugin.calibration_params)
        self._apply_magnetometer_param_io_result(method_id, result, action="save", path=path)

    def _on_clear_magnetometer_method_params(self, method_id: str) -> None:
        self._ensure_magnetometer_method_state()
        plugin = self._mag_methods.get(method_id)
        if plugin is None:
            return
        if not plugin.can_clear_params():
            return
        plugin.status = plugin.load_status
        plugin.diagnostics = None
        plugin.calibration_params = None
        plugin.calibration_report = ""
        plugin.calibration_warnings = []
        plugin.calibration_dataset_name = None
        plugin.calibration_runtime_s = None
        plugin.calibration_progress = 0.0
        plugin.params_warnings = []
        plugin.params_profile_path = None
        plugin.process_warnings = []
        plugin.realtime_enabled = False
        plugin.record_enabled = False
        plugin.last_output = None
        self._mag_latest_derived_streams.pop(method_id, None)
        self._mag_offline_method_clouds.pop(method_id, None)
        for kind in self._magnetometer_method_output_kinds(plugin):
            self._calibration_stream_snapshots.pop(make_method_producer_id(plugin.file_path, kind), None)
        self._refresh_magnetometer_methods_ui()
        self._refresh_magnetometer_method_clouds_ui()
        self._refresh_magnetometer_heading_routing_ui()
        self._refresh_magnetometer_metrics_ui()
        if bool(getattr(self, "_mag_recording_active", False)):
            self._update_magnetometer_dataset_recording_metadata()
        self._save_settings()

    def _make_magnetometer_method_id(self) -> str:
        self._ensure_magnetometer_method_state()
        return f"method_{len(self._mag_methods) + 1}"

    def _register_magnetometer_method(self, plugin: LoadedMethodPlugin) -> LoadedMethodPlugin:
        self._ensure_magnetometer_method_state()
        plugin.method_id = self._make_magnetometer_method_id()
        plugin.derived_stream_id = f"derived_{plugin.method_id}"
        self._mag_methods[plugin.method_id] = plugin
        self._mag_selected_method_id = plugin.method_id
        self._refresh_magnetometer_methods_ui()
        self._refresh_magnetometer_metrics_ui()
        return plugin

    def _on_magnetometer_add_plugin(self) -> None:
        dlg = AddPluginDialog(self.root, initial_dir=self._dialog_initialdir("plugin"))
        self.root.wait_window(dlg)
        if not dlg.result:
            return
        self._remember_dialog_dir("plugin", dlg.result["path"])
        plugin = load_method_plugin(dlg.result["path"])
        plugin = self._register_magnetometer_method(plugin)
        if plugin.status == "error":
            self._open_magnetometer_method_diagnostics(plugin.method_id)
        if bool(getattr(self, "_mag_recording_active", False)):
            self._update_magnetometer_dataset_recording_metadata()
        self._save_settings()

    def _on_select_magnetometer_method(self, method_id: str) -> None:
        self._ensure_magnetometer_method_state()
        if method_id not in self._mag_methods:
            return
        self._mag_selected_method_id = method_id
        self._refresh_magnetometer_methods_ui()
        self._refresh_magnetometer_metrics_ui()
        if self._mag_methods[method_id].status == "error":
            self._open_magnetometer_method_diagnostics(method_id)
        self._save_settings()

    def _on_remove_magnetometer_method(self, method_id: str) -> None:
        self._ensure_magnetometer_method_state()
        plugin = self._mag_methods.pop(method_id, None)
        if plugin is None:
            return
        self._mag_latest_derived_streams.pop(method_id, None)
        self._mag_offline_method_clouds.pop(method_id, None)
        for kind in self._magnetometer_method_output_kinds(plugin):
            self._calibration_stream_snapshots.pop(make_method_producer_id(plugin.file_path, kind), None)
        if self._mag_selected_method_id == method_id:
            self._mag_selected_method_id = next(iter(self._mag_methods), None)
        if self._mag_primary_heading_stream_id == method_id:
            self._mag_primary_heading_stream_id = "raw_heading"
        self._refresh_magnetometer_methods_ui()
        self._refresh_magnetometer_method_clouds_ui()
        self._refresh_magnetometer_heading_routing_ui()
        self._refresh_magnetometer_metrics_ui()
        if bool(getattr(self, "_mag_recording_active", False)):
            self._update_magnetometer_dataset_recording_metadata()
        self._save_settings()

    def _on_open_magnetometer_method_info(self, method_id: str) -> None:
        self._ensure_magnetometer_method_state()
        plugin = self._mag_methods.get(method_id)
        if plugin is None:
            return
        dlg = MethodInfoDialog(
            self.root,
            title=f"Method Info — {plugin.name}",
            payload_provider=lambda method_id=method_id: self._build_magnetometer_method_info_payload(method_id),
            on_binding_change=lambda slot, producer_id, method_id=method_id: self._on_change_magnetometer_method_binding(
                method_id,
                slot,
                producer_id,
            ),
        )
        self.root.wait_window(dlg)

    def _open_magnetometer_method_diagnostics(self, method_id: str) -> None:
        self._ensure_magnetometer_method_state()
        plugin = self._mag_methods.get(method_id)
        if plugin is None:
            return
        diagnostics = plugin.diagnostics
        if diagnostics is None:
            return
        dlg = MethodDiagnosticsDialog(
            self.root,
            title=f"Method Diagnostics — {plugin.name}",
            diagnostics=diagnostics.as_dict(),
        )
        self.root.wait_window(dlg)

    def _on_magnetometer_save_current(self) -> None:
        path = self._mag_dataset_export_path
        if not path and self._mag_dataset is not None and self._mag_dataset.source_path:
            path = self._mag_dataset.source_path
        if not path:
            path = self._choose_magnetometer_dataset_path()
        if not path:
            return
        self._save_magnetometer_dataset_to_path(path)

    def _on_magnetometer_save_as(self) -> None:
        path = self._choose_magnetometer_dataset_path()
        if not path:
            return
        self._save_magnetometer_dataset_to_path(path)

    def _on_clear_magnetometer_datasets(self) -> None:
        if self._mag_recording_active:
            self._show_warning("Magnetometer Dataset", "Stop recording before clearing datasets from memory.")
            return
        if not self._mag_datasets and self._mag_dataset is None:
            return
        self._mag_datasets = []
        self._mag_dataset_export_path = ""
        self._set_active_magnetometer_dataset(None)
        self._save_settings()

    def _concatenate_selected_magnetometer_datasets(self, selected_indices: Sequence[int]) -> Dataset | None:
        datasets = [
            self._mag_datasets[index]
            for index in selected_indices
            if 0 <= index < len(self._mag_datasets)
        ]
        if len(datasets) < 2:
            return None

        merged = Dataset(self._make_concatenated_magnetometer_dataset_name())
        for dataset in datasets:
            merged.extend(list(dataset.records))
        self._mag_datasets.append(merged)
        self._set_active_magnetometer_dataset(merged)
        return merged

    def _detach_shared_magnetometer_dataset(self) -> None:
        if self._mag_dataset is None or not self._is_shared_projection_dataset(self._mag_dataset):
            return
        detached = self._snapshot_magnetometer_dataset(self._mag_dataset)
        detached.source_path = None
        metadata = dict(getattr(self._mag_dataset, "metadata", {}))
        metadata.pop("shared_source_path", None)
        metadata.pop("shared_dataset_name", None)
        metadata.pop("shared_dataset_format", None)
        detached.metadata = metadata
        self._mag_datasets.append(detached)
        self._set_active_magnetometer_dataset(detached)

    def _on_magnetometer_concatenate(self) -> None:
        if self._mag_recording_active:
            return
        if len(self._mag_datasets) < 2:
            self._show_info("Magnetometer Dataset", "Load at least two datasets before concatenation.")
            return
        items = [self._dataset_dialog_label(dataset) for dataset in self._mag_datasets]
        dlg = DatasetSelectionDialog(
            self.root,
            title="Concatenate Datasets",
            prompt="Select datasets to concatenate:",
            items=items,
            preselected=list(range(len(items))),
        )
        self.root.wait_window(dlg)
        if dlg.result is None:
            return
        if len(dlg.result) < 2:
            self._show_info("Magnetometer Dataset", "Select at least two datasets to concatenate.")
            return
        self._concatenate_selected_magnetometer_datasets(dlg.result)

    def _trim_active_magnetometer_dataset(self, selected_indices: Sequence[int]) -> bool:
        if self._mag_dataset is None or not selected_indices:
            return False
        self._detach_shared_magnetometer_dataset()
        self._mag_dataset.trim(min(selected_indices), max(selected_indices))
        self._rebuild_magnetometer_dataset_table()
        self._refresh_magnetometer_dataset_ui()
        self._refresh_magnetometer_metrics_ui()
        return True

    def _delete_active_magnetometer_rows(self, selected_indices: Sequence[int]) -> bool:
        if self._mag_dataset is None or not selected_indices:
            return False
        self._detach_shared_magnetometer_dataset()
        self._mag_dataset.delete_rows(list(selected_indices))
        self._rebuild_magnetometer_dataset_table()
        self._refresh_magnetometer_dataset_ui()
        self._refresh_magnetometer_metrics_ui()
        return True

    def _on_magnetometer_trim_selection(self) -> None:
        if self._mag_recording_active:
            return
        if self._mag_dataset is None:
            self._show_info("Magnetometer Dataset", "There is no active dataset.")
            return
        selected_indices = self.magnetometer_tab.get_selected_dataset_row_indices()
        if not selected_indices:
            self._show_info("Magnetometer Dataset", "Select one or more rows in the Data table.")
            return
        self._trim_active_magnetometer_dataset(selected_indices)

    def _on_magnetometer_delete_selection(self) -> None:
        if self._mag_recording_active:
            return
        if self._mag_dataset is None:
            self._show_info("Magnetometer Dataset", "There is no active dataset.")
            return
        selected_indices = self.magnetometer_tab.get_selected_dataset_row_indices()
        if not selected_indices:
            self._show_info("Magnetometer Dataset", "Select one or more rows in the Data table.")
            return
        self._delete_active_magnetometer_rows(selected_indices)

    def _sensor_title(self, sensor: str) -> str:
        return "Accelerometer" if sensor == "acc" else "Gyroscope"

    def _sensor_tab(self, sensor: str) -> Any:
        return getattr(self, "accelerometer_tab" if sensor == "acc" else "gyroscope_tab", None)

    def _sensor_raw_source_id(self, sensor: str) -> str:
        return "raw_accelerometer" if sensor == "acc" else "raw_gyroscope"

    def _sensor_primary_raw_stream_id(self, sensor: str) -> str:
        return "raw_tilt" if sensor == "acc" else "raw_gyroscope"

    def _sensor_primary_raw_label(self, sensor: str) -> str:
        return "Raw Tilt" if sensor == "acc" else "Raw Gyroscope"

    def _sensor_output_label(self, sensor: str) -> str:
        return "primary output" if sensor == "acc" else "output stream"

    def _default_accelerometer_sources(self) -> dict[str, dict[str, object]]:
        return {
            "raw_accelerometer": {
                "title": "Raw Accelerometer",
                "show": True,
                "record": True,
            },
            "raw_tilt": {
                "title": "Raw Tilt",
                "show": True,
                "record": False,
            },
        }

    def _default_gyroscope_sources(self) -> dict[str, dict[str, object]]:
        return {
            "raw_gyroscope": {
                "title": "Raw Gyroscope",
                "show": True,
                "record": True,
            },
        }

    def _remember_sensor_dialog_dir(self, sensor: str, kind: str, path: str) -> None:
        if not path:
            return
        attr_name = "_acc_file_dialog_dirs" if sensor == "acc" else "_gyro_file_dialog_dirs"
        if not hasattr(self, attr_name):
            setattr(self, attr_name, {"dataset": "", "params": "", "plugin": "", "metrics": ""})
        directory = path if os.path.isdir(path) else os.path.dirname(path)
        if directory:
            getattr(self, attr_name)[kind] = directory

    def _sensor_dialog_initialdir(self, sensor: str, kind: str, *, fallback_path: str | None = None) -> str | None:
        attr_name = "_acc_file_dialog_dirs" if sensor == "acc" else "_gyro_file_dialog_dirs"
        if hasattr(self, attr_name):
            candidate = getattr(self, attr_name).get(kind, "")
            if candidate and os.path.isdir(candidate):
                return candidate
        if fallback_path:
            candidate = fallback_path if os.path.isdir(fallback_path) else os.path.dirname(fallback_path)
            if candidate and os.path.isdir(candidate):
                return candidate
        return None

    def _ensure_sensor_source_state(self, sensor: str) -> None:
        if sensor == "acc":
            if not hasattr(self, "_acc_sources"):
                self._acc_sources = self._default_accelerometer_sources()
            if not hasattr(self, "_acc_selected_source_id"):
                self._acc_selected_source_id = "raw_accelerometer"
            return
        if not hasattr(self, "_gyro_sources"):
            self._gyro_sources = self._default_gyroscope_sources()
        if not hasattr(self, "_gyro_selected_source_id"):
            self._gyro_selected_source_id = "raw_gyroscope"

    def _ensure_sensor_method_state(self, sensor: str) -> None:
        prefix = "_acc" if sensor == "acc" else "_gyro"
        if not hasattr(self, f"{prefix}_methods"):
            setattr(self, f"{prefix}_methods", {})
        if not hasattr(self, f"{prefix}_selected_method_id"):
            setattr(self, f"{prefix}_selected_method_id", None)
        if not hasattr(self, f"{prefix}_latest_derived_streams"):
            setattr(self, f"{prefix}_latest_derived_streams", {})
        if not hasattr(self, f"{prefix}_primary_output_stream_id"):
            setattr(self, f"{prefix}_primary_output_stream_id", self._sensor_primary_raw_stream_id(sensor))
        if not hasattr(self, f"{prefix}_last_live_snapshot"):
            setattr(self, f"{prefix}_last_live_snapshot", None)
        if not hasattr(self, f"{prefix}_offline_method_clouds"):
            setattr(self, f"{prefix}_offline_method_clouds", {})
        if not hasattr(self, f"{prefix}_file_dialog_dirs"):
            setattr(self, f"{prefix}_file_dialog_dirs", {"dataset": "", "params": "", "plugin": "", "metrics": ""})
        if not hasattr(self, f"{prefix}_saved_state"):
            setattr(self, f"{prefix}_saved_state", {})
        if not hasattr(self, f"{prefix}_datasets"):
            setattr(self, f"{prefix}_datasets", [])
        if not hasattr(self, f"{prefix}_dataset"):
            setattr(self, f"{prefix}_dataset", None)
        if not hasattr(self, f"{prefix}_recording_active"):
            setattr(self, f"{prefix}_recording_active", False)
        if not hasattr(self, f"{prefix}_dataset_export_path"):
            setattr(self, f"{prefix}_dataset_export_path", "")
        if not hasattr(self, f"{prefix}_last_dataset_ui_refresh_s"):
            setattr(self, f"{prefix}_last_dataset_ui_refresh_s", 0.0)
        if not hasattr(self, "_calibration_stream_snapshots"):
            self._calibration_stream_snapshots = {}
        methods: dict[str, LoadedMethodPlugin] = getattr(self, f"{prefix}_methods")
        for plugin in methods.values():
            if not isinstance(getattr(plugin, "stream_bindings", None), dict):
                plugin.stream_bindings = {SCOPE_CALIBRATE: {}, SCOPE_PROCESS: {}}
            else:
                plugin.stream_bindings.setdefault(SCOPE_CALIBRATE, {})
                plugin.stream_bindings.setdefault(SCOPE_PROCESS, {})
            if not isinstance(getattr(plugin, "routing_validation", None), dict):
                plugin.routing_validation = {}
            if not isinstance(getattr(plugin, "routing_warnings", None), list):
                plugin.routing_warnings = []
            if not isinstance(getattr(plugin, "stream_requirements", None), dict):
                plugin.stream_requirements = {}
            if not plugin.stream_requirements:
                try:
                    plugin.stream_requirements = infer_stream_requirements(plugin.info)
                except Exception:
                    plugin.stream_requirements = {}

    def _ensure_sensor_calibration_state(self, sensor: str) -> None:
        prefix = "_acc" if sensor == "acc" else "_gyro"
        if not hasattr(self, f"{prefix}_calibration_results"):
            setattr(self, f"{prefix}_calibration_results", queue.Queue())
        if not hasattr(self, f"{prefix}_calibration_jobs"):
            setattr(self, f"{prefix}_calibration_jobs", {})

    def _selected_sensor_plugin(self, sensor: str) -> LoadedMethodPlugin | None:
        self._ensure_sensor_method_state(sensor)
        method_id = getattr(self, "_acc_selected_method_id" if sensor == "acc" else "_gyro_selected_method_id")
        methods = getattr(self, "_acc_methods" if sensor == "acc" else "_gyro_methods")
        return methods.get(method_id or "")

    def _serialize_sensor_settings(self, sensor: str) -> dict[str, Any]:
        self._ensure_sensor_source_state(sensor)
        self._ensure_sensor_method_state(sensor)
        prefix = "_acc" if sensor == "acc" else "_gyro"
        methods: dict[str, LoadedMethodPlugin] = getattr(self, f"{prefix}_methods")
        datasets = getattr(self, f"{prefix}_datasets")
        active_dataset = getattr(self, f"{prefix}_dataset")
        method_entries: list[dict[str, Any]] = []
        for plugin in methods.values():
            method_entries.append(
                {
                    "file_path": plugin.file_path,
                    "show": bool(plugin.show_enabled),
                    "record": bool(plugin.record_enabled),
                    "realtime_enabled": bool(plugin.realtime_enabled),
                    "params_profile_path": plugin.params_profile_path or "",
                    "bindings": self._stream_bindings_payload(plugin),
                }
            )

        dataset_paths = [
            dataset.source_path
            for dataset in datasets
            if getattr(dataset, "source_path", None)
        ]
        active_dataset_path = ""
        if active_dataset is not None and getattr(active_dataset, "source_path", None):
            active_dataset_path = active_dataset.source_path

        selected_method_path = ""
        selected_plugin = self._selected_sensor_plugin(sensor)
        if selected_plugin is not None:
            selected_method_path = selected_plugin.file_path

        primary_output = {"kind": "raw"}
        primary_stream_id = getattr(self, f"{prefix}_primary_output_stream_id")
        if primary_stream_id != self._sensor_primary_raw_stream_id(sensor):
            plugin = methods.get(primary_stream_id)
            if plugin is not None:
                primary_output = {"kind": "method", "file_path": plugin.file_path}

        tab = self._sensor_tab(sensor)
        view_state = {}
        if tab is not None:
            view_state = tab.get_view_state()
        else:
            saved_state = getattr(self, f"{prefix}_saved_state", {})
            if isinstance(saved_state, dict):
                view_state = dict(saved_state.get("view_state", {}))

        sources = getattr(self, f"{prefix}_sources")
        file_dialog_dirs = getattr(self, f"{prefix}_file_dialog_dirs")
        return {
            "datasets": dataset_paths,
            "active_dataset_path": active_dataset_path,
            "sources": {
                source_id: {
                    "show": bool(source.get("show", False)),
                    "record": bool(source.get("record", False)),
                }
                for source_id, source in sources.items()
            },
            "selected_source_id": getattr(self, f"{prefix}_selected_source_id"),
            "methods": method_entries,
            "selected_method_path": selected_method_path,
            "primary_output": primary_output,
            "view_state": view_state,
            "file_dialog_dirs": dict(file_dialog_dirs),
        }

    def _serialize_accelerometer_settings(self) -> dict[str, Any]:
        return self._serialize_sensor_settings("acc")

    def _serialize_gyroscope_settings(self) -> dict[str, Any]:
        return self._serialize_sensor_settings("gyro")

    def _sensor_method_requirements(
        self,
        sensor: str,
        plugin: LoadedMethodPlugin,
        *,
        scope: str,
    ) -> list[StreamRequirementDescriptor]:
        self._ensure_sensor_method_state(sensor)
        requirements = plugin.stream_requirements.get(scope, [])
        return [
            requirement
            for requirement in requirements
            if isinstance(requirement, StreamRequirementDescriptor)
        ]

    def _sensor_method_output_kinds(self, sensor: str, plugin: LoadedMethodPlugin) -> tuple[str, ...]:
        del sensor
        return infer_output_stream_kinds(plugin.info)

    def _sensor_dataset_has_stream(self, sensor: str, stream_id: str, dataset: Any | None = None) -> bool:
        prefix = "_acc" if sensor == "acc" else "_gyro"
        active_dataset = getattr(self, f"{prefix}_dataset", None) if dataset is None else dataset
        if active_dataset is None:
            return False
        return any(record.stream_id == stream_id for record in active_dataset.records)

    def _paired_sensor_dataset_for_calibration(self, sensor: str, target_sensor: str, dataset: Any | None) -> Any | None:
        self._ensure_shared_imu_state()
        self._ensure_sensor_method_state(target_sensor)
        if target_sensor == sensor:
            return dataset if dataset is not None else getattr(self, "_acc_dataset" if sensor == "acc" else "_gyro_dataset", None)
        active_target = getattr(self, "_acc_dataset" if target_sensor == "acc" else "_gyro_dataset", None)
        if dataset is None or not self._is_shared_projection_dataset(dataset):
            return active_target
        shared_dataset = self._find_shared_imu_dataset(dataset)
        if shared_dataset is None:
            return active_target
        target_collection = self._acc_datasets if target_sensor == "acc" else self._gyro_datasets
        key = (shared_dataset.source_path or "", shared_dataset.summary()["name"])
        for candidate in target_collection:
            if self._shared_projection_key(candidate) == key:
                return candidate
        return active_target

    def _sensor_builtin_calibration_producers(self, sensor: str, dataset: Any | None = None) -> dict[str, StreamProducerDescriptor]:
        self._ensure_sensor_method_state(sensor)
        active_dataset = getattr(self, "_acc_dataset" if sensor == "acc" else "_gyro_dataset", None) if dataset is None else dataset
        acc_dataset = self._paired_sensor_dataset_for_calibration(sensor, "acc", active_dataset)
        gyro_dataset = self._paired_sensor_dataset_for_calibration(sensor, "gyro", active_dataset)
        raw_acc_active = (
            self._sensor_dataset_has_stream("acc", "raw_accelerometer", acc_dataset)
            or make_builtin_producer_id("raw_accelerometer") in self._calibration_stream_snapshots
        )
        raw_tilt_active = (
            self._sensor_dataset_has_stream("acc", "raw_tilt", acc_dataset)
            or self._sensor_dataset_has_stream("acc", "raw_accelerometer", acc_dataset)
            or make_builtin_producer_id("raw_tilt") in self._calibration_stream_snapshots
            or make_builtin_producer_id("raw_accelerometer") in self._calibration_stream_snapshots
        )
        raw_gyro_active = (
            self._sensor_dataset_has_stream("gyro", "raw_gyroscope", gyro_dataset)
            or make_builtin_producer_id("raw_gyroscope") in self._calibration_stream_snapshots
        )
        tacho_active = make_builtin_producer_id("tacho_pair") in self._calibration_stream_snapshots
        motor_active = make_builtin_producer_id("motor_telemetry") in self._calibration_stream_snapshots
        tensor_active = make_builtin_producer_id("sensor_tensor_telemetry") in self._calibration_stream_snapshots
        producers = {
            make_builtin_producer_id("raw_accelerometer"): StreamProducerDescriptor(
                producer_id=make_builtin_producer_id("raw_accelerometer"),
                kind="imu.accel_vector",
                title="Raw Accelerometer",
                stream_id="raw_accelerometer",
                stream_type="raw",
                origin="builtin",
                active=raw_acc_active,
                details={"source": "D0.accel", "source_sensor": "acc"},
            ),
            make_builtin_producer_id("raw_tilt"): StreamProducerDescriptor(
                producer_id=make_builtin_producer_id("raw_tilt"),
                kind="imu.accel_vector",
                title="Raw Tilt",
                stream_id="raw_tilt",
                stream_type="raw",
                origin="builtin",
                active=raw_tilt_active,
                details={"source": "built-in tilt from D0.accel", "source_sensor": "acc"},
            ),
            make_builtin_producer_id("raw_gyroscope"): StreamProducerDescriptor(
                producer_id=make_builtin_producer_id("raw_gyroscope"),
                kind="imu.gyro_vector",
                title="Raw Gyroscope",
                stream_id="raw_gyroscope",
                stream_type="raw",
                origin="builtin",
                active=raw_gyro_active,
                details={"source": "D0.gyro", "source_sensor": "gyro"},
            ),
            make_builtin_producer_id("tacho_pair"): StreamProducerDescriptor(
                producer_id=make_builtin_producer_id("tacho_pair"),
                kind="tacho.rpm_pair",
                title="Track Tacho Pair",
                stream_id="tacho_pair",
                stream_type="raw",
                origin="builtin",
                active=tacho_active,
                details={"source": "D1"},
            ),
            make_builtin_producer_id("motor_telemetry"): StreamProducerDescriptor(
                producer_id=make_builtin_producer_id("motor_telemetry"),
                kind="motor.telemetry",
                title="Motor Telemetry",
                stream_id="motor_telemetry",
                stream_type="raw",
                origin="builtin",
                active=motor_active,
                details={"source": "D2"},
            ),
            make_builtin_producer_id("sensor_tensor_telemetry"): StreamProducerDescriptor(
                producer_id=make_builtin_producer_id("sensor_tensor_telemetry"),
                kind="sensor_tensor.telemetry",
                title="Sensor Tensor Telemetry",
                stream_id="sensor_tensor_telemetry",
                stream_type="raw",
                origin="builtin",
                active=tensor_active,
                details={"source": "D3"},
            ),
        }
        return producers

    def _sensor_method_calibration_producers(self, sensor: str, *, include_cross_sensor: bool = False) -> dict[str, StreamProducerDescriptor]:
        self._ensure_sensor_method_state(sensor)
        producers: dict[str, StreamProducerDescriptor] = {}
        sensor_groups = [(sensor, getattr(self, "_acc_methods" if sensor == "acc" else "_gyro_methods"))]
        if include_cross_sensor and sensor == "gyro":
            self._ensure_sensor_method_state("acc")
            sensor_groups.append(("acc", self._acc_methods))
        for source_sensor, methods in sensor_groups:
            for method_id, plugin in methods.items():
                stream_id = plugin.derived_stream_id or f"derived_{plugin.method_id}"
                for kind in self._sensor_method_output_kinds(source_sensor, plugin):
                    producer_id = make_method_producer_id(plugin.file_path, kind)
                    title = plugin.name if source_sensor == sensor else f"{plugin.name} (Accelerometer)"
                    producers[producer_id] = StreamProducerDescriptor(
                        producer_id=producer_id,
                        kind=kind,
                        title=title,
                        stream_id=stream_id,
                        stream_type="derived",
                        origin="method",
                        status=plugin.status,
                        active=(
                            plugin.status != "error"
                            and plugin.supports_process()
                            and plugin.calibration_params is not None
                        ),
                        file_path=plugin.file_path,
                        method_id=method_id,
                        requires_calibration=True,
                        details={
                            "method_name": plugin.name,
                            "method_version": plugin.version,
                            "source_sensor": source_sensor,
                        },
                    )
        return producers

    def _sensor_calibration_producers(self, sensor: str, dataset: Any | None = None) -> dict[str, StreamProducerDescriptor]:
        producers = self._sensor_builtin_calibration_producers(sensor, dataset)
        producers.update(self._sensor_method_calibration_producers(sensor, include_cross_sensor=(sensor == "gyro")))
        return producers

    def _effective_sensor_stream_bindings(
        self,
        sensor: str,
        plugin: LoadedMethodPlugin,
        *,
        scope: str,
        producers: dict[str, StreamProducerDescriptor] | None = None,
    ) -> dict[str, str]:
        self._ensure_sensor_method_state(sensor)
        producer_map = self._sensor_calibration_producers(sensor) if producers is None else producers
        explicit = {
            str(slot): str(producer_id)
            for slot, producer_id in plugin.stream_bindings.get(scope, {}).items()
            if str(slot).strip() and str(producer_id).strip()
        }
        requirements = self._sensor_method_requirements(sensor, plugin, scope=scope)
        defaults = select_default_binding(requirements, producer_map)
        effective = dict(defaults)
        effective.update(explicit)
        return effective

    def _all_sensor_method_bindings_by_path(
        self,
        sensor: str,
        *,
        scope: str,
        producers: dict[str, StreamProducerDescriptor] | None = None,
    ) -> dict[str, dict[str, str]]:
        methods: dict[str, LoadedMethodPlugin] = getattr(self, "_acc_methods" if sensor == "acc" else "_gyro_methods")
        producer_map = self._sensor_calibration_producers(sensor) if producers is None else producers
        return {
            plugin.file_path: self._effective_sensor_stream_bindings(sensor, plugin, scope=scope, producers=producer_map)
            for plugin in methods.values()
        }

    def _update_sensor_method_routing_state(self, sensor: str) -> None:
        self._ensure_sensor_method_state(sensor)
        producers = self._sensor_calibration_producers(sensor)
        bindings_by_path = self._all_sensor_method_bindings_by_path(sensor, scope=SCOPE_CALIBRATE, producers=producers)
        methods: dict[str, LoadedMethodPlugin] = getattr(self, "_acc_methods" if sensor == "acc" else "_gyro_methods")
        for plugin in methods.values():
            requirements = self._sensor_method_requirements(sensor, plugin, scope=SCOPE_CALIBRATE)
            selected_bindings = dict(bindings_by_path.get(plugin.file_path, {}))
            validation = validate_method_bindings(
                method_key=plugin.file_path,
                requirements=requirements,
                selected_bindings=selected_bindings,
                producers=producers,
                bindings_by_method=bindings_by_path,
            )
            if sensor == "gyro" and "acc_input" in validation.slot_candidates:
                filtered_candidates: list[dict[str, Any]] = []
                for candidate in validation.slot_candidates.get("acc_input", []):
                    details = candidate.get("details", {}) if isinstance(candidate, dict) else {}
                    is_acc_method = (
                        candidate.get("origin") == "method"
                        and str(details.get("source_sensor", "")) == "acc"
                    )
                    entry = dict(candidate)
                    if not is_acc_method:
                        entry["selectable"] = False
                        entry["disabled_reason"] = "requires calibrated accelerometer method output"
                    filtered_candidates.append(entry)
                validation.slot_candidates["acc_input"] = filtered_candidates
                selected_acc = validation.selected_bindings.get("acc_input", "")
                if selected_acc:
                    selected_producer = producers.get(selected_acc)
                    selected_is_acc_method = (
                        selected_producer is not None
                        and selected_producer.origin == "method"
                        and str(selected_producer.details.get("source_sensor", "")) == "acc"
                    )
                    if not selected_is_acc_method:
                        validation.issues = list(validation.issues) + [
                            "Accelerometer reference: requires calibrated accelerometer method output"
                        ]
                        validation.ok = False
            plugin.routing_validation[SCOPE_CALIBRATE] = validation
            plugin.routing_warnings = list(validation.issues)

    def _can_start_sensor_calibration(self, sensor: str, plugin: LoadedMethodPlugin) -> bool:
        del sensor
        validation = plugin.routing_validation.get(SCOPE_CALIBRATE)
        if validation is None:
            return plugin.can_start_calibration()
        return plugin.can_start_calibration() and validation.ok

    def _sensor_method_binding_candidates_for_ui(self, sensor: str, plugin: LoadedMethodPlugin) -> list[dict[str, Any]]:
        self._update_sensor_method_routing_state(sensor)
        validation = plugin.routing_validation.get(SCOPE_CALIBRATE)
        producers = self._sensor_calibration_producers(sensor)
        bindings = self._effective_sensor_stream_bindings(sensor, plugin, scope=SCOPE_CALIBRATE, producers=producers)
        rows: list[dict[str, Any]] = []
        for requirement in self._sensor_method_requirements(sensor, plugin, scope=SCOPE_CALIBRATE):
            selected_producer_id = bindings.get(requirement.slot, "")
            selected_label = "Not selected"
            if selected_producer_id in producers:
                selected_label = producers[selected_producer_id].title
            candidates = [] if validation is None else list(validation.slot_candidates.get(requirement.slot, []))
            rows.append(
                {
                    "slot": requirement.slot,
                    "kind": requirement.kind,
                    "label": requirement.label or requirement.slot,
                    "description": requirement.description,
                    "selected_producer_id": selected_producer_id,
                    "selected_label": selected_label,
                    "candidates": candidates,
                }
            )
        return rows

    def _build_sensor_method_info_payload(self, sensor: str, method_id: str) -> dict[str, Any]:
        self._ensure_sensor_method_state(sensor)
        methods: dict[str, LoadedMethodPlugin] = getattr(self, "_acc_methods" if sensor == "acc" else "_gyro_methods")
        plugin = methods.get(method_id)
        if plugin is None:
            return {}
        self._update_sensor_method_routing_state(sensor)
        diagnostics = None if plugin.diagnostics is None else plugin.diagnostics.as_dict()
        return build_method_info_payload(
            plugin.info,
            file_path=plugin.file_path,
            status_text=plugin.display_status_text(),
            warnings=plugin.effective_warnings(),
            calibration_dataset_name=plugin.calibration_dataset_name,
            calibration_runtime_s=plugin.calibration_runtime_s,
            params_profile_path=plugin.params_profile_path,
            calibration_params=plugin.calibration_params,
            calibration_report=plugin.calibration_report,
            stream_requirements=self._stream_requirements_payload(plugin),
            stream_bindings=self._stream_bindings_payload(plugin),
            routing_validation=self._method_routing_validation_payload(plugin),
            diagnostics=diagnostics,
            calibration_bindings=self._sensor_method_binding_candidates_for_ui(sensor, plugin),
        )

    def _on_change_sensor_method_binding(self, sensor: str, method_id: str, slot: str, producer_id: str) -> bool:
        self._ensure_sensor_method_state(sensor)
        methods: dict[str, LoadedMethodPlugin] = getattr(self, "_acc_methods" if sensor == "acc" else "_gyro_methods")
        plugin = methods.get(method_id)
        if plugin is None:
            return False
        self._update_sensor_method_routing_state(sensor)
        current_validation = plugin.routing_validation.get(SCOPE_CALIBRATE)
        selectable = False
        if current_validation is not None:
            for candidate in current_validation.slot_candidates.get(slot, []):
                if candidate.get("producer_id") == producer_id and bool(candidate.get("selectable", False)):
                    selectable = True
                    break
        if not selectable:
            return False
        old_value = plugin.stream_bindings.setdefault(SCOPE_CALIBRATE, {}).get(slot, "")
        plugin.stream_bindings[SCOPE_CALIBRATE][slot] = producer_id
        self._update_sensor_method_routing_state(sensor)
        if plugin.routing_validation.get(SCOPE_CALIBRATE) is None:
            if old_value:
                plugin.stream_bindings[SCOPE_CALIBRATE][slot] = old_value
            else:
                plugin.stream_bindings[SCOPE_CALIBRATE].pop(slot, None)
            self._update_sensor_method_routing_state(sensor)
            return False
        self._refresh_sensor_methods_ui(sensor)
        self._refresh_all_sensor_method_dataset_clouds(sensor)
        self._save_settings()
        return True

    def _sensor_raw_dataset_records(self, sensor: str, dataset: Any | None = None) -> list[Any]:
        prefix = "_acc" if sensor == "acc" else "_gyro"
        active_dataset = getattr(self, f"{prefix}_dataset", None) if dataset is None else dataset
        if active_dataset is None:
            return []
        if sensor == "acc":
            return [
                record
                for record in active_dataset.records
                if record.stream_id == "raw_accelerometer" and "tilt_only" not in (record.flags or "")
            ]
        return [record for record in active_dataset.records if record.stream_id == "raw_gyroscope"]

    def _build_sensor_sample_from_record(self, sensor: str, record: Any) -> dict[str, Any]:
        if sensor == "acc":
            return {
                "stream_id": record.stream_id,
                "stream_type": record.stream_type,
                "producer_name": record.producer_name,
                "producer_version": record.producer_version,
                "timestamp_mcu": record.timestamp_mcu,
                "timestamp_pc_rx": record.timestamp_pc_rx,
                "timestamp_pc_est": record.timestamp_pc_est,
                "acc_x": record.acc_x,
                "acc_y": record.acc_y,
                "acc_z": record.acc_z,
                "roll_deg": record.roll_deg,
                "pitch_deg": record.pitch_deg,
                "flags": record.flags,
            }
        return {
            "stream_id": record.stream_id,
            "stream_type": record.stream_type,
            "producer_name": record.producer_name,
            "producer_version": record.producer_version,
            "timestamp_mcu": record.timestamp_mcu,
            "timestamp_pc_rx": record.timestamp_pc_rx,
            "timestamp_pc_est": record.timestamp_pc_est,
            "gyro_x": record.gyro_x,
            "gyro_y": record.gyro_y,
            "gyro_z": record.gyro_z,
            "rate_mag": record.rate_mag,
            "flags": record.flags,
        }

    def _adapt_sensor_record_for_producer(
        self,
        sensor: str,
        producer: StreamProducerDescriptor,
        record: Any,
    ) -> dict[str, Any]:
        source_sensor = str(producer.details.get("source_sensor", sensor))
        payload = self._build_sensor_sample_from_record(source_sensor, record)
        if source_sensor == "acc" and producer.stream_id == "raw_tilt":
            if payload.get("roll_deg") is None or payload.get("pitch_deg") is None:
                roll_deg, pitch_deg = compute_accelerometer_tilt_deg(
                    float(payload["acc_x"]),
                    float(payload["acc_y"]),
                    float(payload["acc_z"]),
                )
                payload["roll_deg"] = roll_deg
                payload["pitch_deg"] = pitch_deg
        if source_sensor == "gyro" and payload.get("rate_mag") is None:
            payload["rate_mag"] = compute_gyro_rate_magnitude(
                float(payload["gyro_x"]),
                float(payload["gyro_y"]),
                float(payload["gyro_z"]),
            )
        return payload

    def _records_for_sensor_builtin_producer(
        self,
        sensor: str,
        producer: StreamProducerDescriptor,
        dataset: Any,
    ) -> list[Any]:
        source_sensor = str(producer.details.get("source_sensor", sensor))
        source_dataset = self._paired_sensor_dataset_for_calibration(sensor, source_sensor, dataset)
        if source_dataset is None:
            return []
        if producer.stream_id == "raw_accelerometer" and source_sensor == "acc":
            return self._sensor_raw_dataset_records("acc", source_dataset)
        if producer.stream_id == "raw_tilt" and source_sensor == "acc":
            records = [record for record in source_dataset.records if record.stream_id == "raw_tilt"]
            return records if records else self._sensor_raw_dataset_records("acc", source_dataset)
        if producer.stream_id == "raw_gyroscope" and source_sensor == "gyro":
            return self._sensor_raw_dataset_records("gyro", source_dataset)
        return []

    def _build_sensor_process_sample_from_inputs(
        self,
        sensor: str,
        requirements: list[StreamRequirementDescriptor],
        inputs_payload: dict[str, dict[str, Any]],
        index: int,
    ) -> dict[str, Any] | None:
        primary_kind = "imu.accel_vector" if sensor == "acc" else "imu.gyro_vector"
        primary_slot = ""
        for requirement in requirements:
            if requirement.kind == primary_kind:
                primary_slot = requirement.slot
                break
        if not primary_slot and requirements:
            primary_slot = requirements[0].slot
        if not primary_slot:
            return None
        primary_records = list(inputs_payload.get(primary_slot, {}).get("records", []))
        if not primary_records:
            return None
        if index >= len(primary_records):
            index = len(primary_records) - 1
        sample = dict(primary_records[index])
        slot_inputs: dict[str, Any] = {}
        for requirement in requirements:
            records = list(inputs_payload.get(requirement.slot, {}).get("records", []))
            if records:
                selected_index = min(index, len(records) - 1)
                slot_inputs[requirement.slot] = dict(records[selected_index])
                continue
            latest = inputs_payload.get(requirement.slot, {}).get("latest")
            if latest is not None:
                slot_inputs[requirement.slot] = dict(latest)
        if slot_inputs:
            sample["inputs"] = slot_inputs
        return sample

    def _normalize_sensor_process_output(
        self,
        sensor: str,
        plugin: LoadedMethodPlugin,
        output: dict[str, Any],
    ) -> tuple[dict[str, Any] | None, PluginDiagnostics | None]:
        if sensor == "acc":
            key_aliases = {
                "acc_x": ("acc_x", "accel_x"),
                "acc_y": ("acc_y", "accel_y"),
                "acc_z": ("acc_z", "accel_z"),
            }
            values: dict[str, float] = {}
            missing: list[str] = []
            for canonical, aliases in key_aliases.items():
                raw_value = None
                for alias in aliases:
                    if alias in output:
                        raw_value = output[alias]
                        break
                if raw_value is None:
                    missing.append(canonical)
                    continue
                try:
                    values[canonical] = float(raw_value)
                except (TypeError, ValueError) as exc:
                    return (
                        None,
                        self._make_magnetometer_method_diagnostics(
                            plugin,
                            last_action="validate_process_output",
                            error_text=f"process() output contains non-numeric accelerometer values: {exc}",
                        ),
                    )
            if missing:
                return (
                    None,
                    self._make_magnetometer_method_diagnostics(
                        plugin,
                        last_action="validate_process_output",
                        error_text=f"process() output missing required keys: {', '.join(missing)}",
                    ),
                )
            if not all(math.isfinite(value) for value in values.values()):
                return (
                    None,
                    self._make_magnetometer_method_diagnostics(
                        plugin,
                        last_action="validate_process_output",
                        error_text="process() output contains non-finite accelerometer values.",
                    ),
                )
            roll_deg = output.get("roll_deg")
            pitch_deg = output.get("pitch_deg")
            if roll_deg in (None, "") or pitch_deg in (None, ""):
                roll_deg, pitch_deg = compute_accelerometer_tilt_deg(values["acc_x"], values["acc_y"], values["acc_z"])
            else:
                try:
                    roll_deg = float(roll_deg)
                    pitch_deg = float(pitch_deg)
                except (TypeError, ValueError) as exc:
                    return (
                        None,
                        self._make_magnetometer_method_diagnostics(
                            plugin,
                            last_action="validate_process_output",
                            error_text=f"process() output tilt is invalid: {exc}",
                        ),
                    )
            normalized = dict(output)
            normalized.update(values)
            normalized["roll_deg"] = roll_deg
            normalized["pitch_deg"] = pitch_deg
            return (normalized, None)

        required = ("gyro_x", "gyro_y", "gyro_z")
        missing = [key for key in required if key not in output]
        if missing:
            return (
                None,
                self._make_magnetometer_method_diagnostics(
                    plugin,
                    last_action="validate_process_output",
                    error_text=f"process() output missing required keys: {', '.join(missing)}",
                ),
            )
        try:
            gx = float(output["gyro_x"])
            gy = float(output["gyro_y"])
            gz = float(output["gyro_z"])
        except (TypeError, ValueError) as exc:
            return (
                None,
                self._make_magnetometer_method_diagnostics(
                    plugin,
                    last_action="validate_process_output",
                    error_text=f"process() output contains non-numeric gyroscope values: {exc}",
                ),
            )
        if not all(math.isfinite(value) for value in (gx, gy, gz)):
            return (
                None,
                self._make_magnetometer_method_diagnostics(
                    plugin,
                    last_action="validate_process_output",
                    error_text="process() output contains non-finite gyroscope values.",
                ),
            )
        rate_mag = output.get("rate_mag")
        if rate_mag is None or rate_mag == "":
            rate_mag = compute_gyro_rate_magnitude(gx, gy, gz)
        else:
            try:
                rate_mag = float(rate_mag)
            except (TypeError, ValueError) as exc:
                return (
                    None,
                    self._make_magnetometer_method_diagnostics(
                        plugin,
                        last_action="validate_process_output",
                        error_text=f"process() output rate magnitude is invalid: {exc}",
                    ),
                )
        normalized = dict(output)
        normalized["gyro_x"] = gx
        normalized["gyro_y"] = gy
        normalized["gyro_z"] = gz
        normalized["rate_mag"] = rate_mag
        return (normalized, None)

    def _build_sensor_derived_record(
        self,
        sensor: str,
        plugin: LoadedMethodPlugin,
        output: dict[str, Any],
    ) -> Any:
        stream_id = plugin.derived_stream_id or f"derived_{plugin.method_id}"
        if sensor == "acc":
            return AccelerometerSampleRecord(
                stream_id=stream_id,
                stream_type="derived",
                producer_name=plugin.name,
                producer_version=plugin.version,
                timestamp_mcu=int(output["timestamp_mcu"]),
                timestamp_pc_rx=int(output["timestamp_pc_rx"]),
                timestamp_pc_est=output.get("timestamp_pc_est"),
                acc_x=float(output["acc_x"]),
                acc_y=float(output["acc_y"]),
                acc_z=float(output["acc_z"]),
                roll_deg=output.get("roll_deg"),
                pitch_deg=output.get("pitch_deg"),
                flags=str(output.get("flags", "realtime_derived")),
            )
        return GyroscopeSampleRecord(
            stream_id=stream_id,
            stream_type="derived",
            producer_name=plugin.name,
            producer_version=plugin.version,
            timestamp_mcu=int(output["timestamp_mcu"]),
            timestamp_pc_rx=int(output["timestamp_pc_rx"]),
            timestamp_pc_est=output.get("timestamp_pc_est"),
            gyro_x=float(output["gyro_x"]),
            gyro_y=float(output["gyro_y"]),
            gyro_z=float(output["gyro_z"]),
            rate_mag=output.get("rate_mag"),
            flags=str(output.get("flags", "realtime_derived")),
        )

    def _apply_sensor_process_error(
        self,
        sensor: str,
        method_id: str,
        plugin: LoadedMethodPlugin,
        result: ProcessRunResult,
    ) -> None:
        plugin.status = "error"
        plugin.process_warnings = list(result.warnings)
        plugin.realtime_enabled = False
        plugin.record_enabled = False
        plugin.last_output = None
        latest_derived = getattr(self, "_acc_latest_derived_streams" if sensor == "acc" else "_gyro_latest_derived_streams")
        latest_derived.pop(method_id, None)
        for kind in self._sensor_method_output_kinds(sensor, plugin):
            self._calibration_stream_snapshots.pop(make_method_producer_id(plugin.file_path, kind), None)
        if result.diagnostics is not None:
            plugin.diagnostics = result.diagnostics
        else:
            plugin.diagnostics = self._make_magnetometer_method_diagnostics(
                plugin,
                last_action="process",
                error_text="Realtime processing failed.",
                warnings=result.warnings,
            )
        self._refresh_sensor_methods_ui(sensor)
        self._open_sensor_method_diagnostics(sensor, method_id)
        self._save_settings()

    def _process_sensor_methods_realtime(
        self,
        sensor: str,
        raw_sample: dict[str, Any],
    ) -> tuple[dict[str, dict[str, Any]], list[Any]]:
        self._ensure_sensor_method_state(sensor)
        self._update_sensor_method_routing_state(sensor)
        methods: dict[str, LoadedMethodPlugin] = getattr(self, "_acc_methods" if sensor == "acc" else "_gyro_methods")
        latest_derived = getattr(self, "_acc_latest_derived_streams" if sensor == "acc" else "_gyro_latest_derived_streams")
        derived_streams: dict[str, dict[str, Any]] = {}
        derived_records: list[Any] = []

        for method_id, plugin in methods.items():
            if not plugin.realtime_enabled:
                plugin.last_output = None
                latest_derived.pop(method_id, None)
                continue

            process_requirements = self._sensor_method_requirements(sensor, plugin, scope=SCOPE_PROCESS)
            process_scope = SCOPE_PROCESS
            if not process_requirements:
                process_requirements = self._sensor_method_requirements(sensor, plugin, scope=SCOPE_CALIBRATE)
                process_scope = SCOPE_CALIBRATE
            process_sample = dict(raw_sample)
            if process_requirements:
                producers = self._sensor_calibration_producers(sensor)
                bindings = self._effective_sensor_stream_bindings(sensor, plugin, scope=process_scope, producers=producers)
                inputs: dict[str, Any] = {}
                for requirement in process_requirements:
                    producer_id = bindings.get(requirement.slot, "")
                    snapshot = self._calibration_stream_snapshots.get(producer_id)
                    if snapshot is not None:
                        inputs[requirement.slot] = dict(snapshot.payload)
                if inputs:
                    process_sample["inputs"] = inputs

            result = run_method_process(plugin, process_sample)
            if not result.ok:
                self._apply_sensor_process_error(sensor, method_id, plugin, result)
                continue

            assert result.output is not None
            merged_output = dict(process_sample)
            merged_output.update(result.output)
            normalized, diagnostics = self._normalize_sensor_process_output(sensor, plugin, merged_output)
            if diagnostics is not None or normalized is None:
                self._apply_sensor_process_error(
                    sensor,
                    method_id,
                    plugin,
                    ProcessRunResult(ok=False, warnings=result.warnings, diagnostics=diagnostics),
                )
                continue

            plugin.status = plugin.load_status
            plugin.diagnostics = None
            plugin.process_warnings = list(result.warnings)
            plugin.last_output = dict(normalized)
            stream_state = {
                "title": plugin.name,
                "stream_id": plugin.derived_stream_id or f"derived_{plugin.method_id}",
                "show": plugin.show_enabled,
            }
            if sensor == "acc":
                stream_state.update(
                    {
                        "acc_x": normalized["acc_x"],
                        "acc_y": normalized["acc_y"],
                        "acc_z": normalized["acc_z"],
                        "roll_deg": normalized.get("roll_deg"),
                        "pitch_deg": normalized.get("pitch_deg"),
                    }
                )
            else:
                stream_state.update(
                    {
                        "gyro_x": normalized["gyro_x"],
                        "gyro_y": normalized["gyro_y"],
                        "gyro_z": normalized["gyro_z"],
                        "rate_mag": normalized.get("rate_mag"),
                    }
                )
            derived_streams[method_id] = stream_state
            latest_derived[method_id] = dict(stream_state)
            for kind in self._sensor_method_output_kinds(sensor, plugin):
                self._publish_calibration_stream_snapshot(
                    make_method_producer_id(plugin.file_path, kind),
                    kind,
                    dict(normalized),
                )
            if plugin.record_enabled:
                derived_records.append(self._build_sensor_derived_record(sensor, plugin, normalized))

        return (derived_streams, derived_records)

    def _sensor_available_output_streams(self, sensor: str) -> list[tuple[str, str]]:
        self._ensure_sensor_method_state(sensor)
        methods: dict[str, LoadedMethodPlugin] = getattr(self, "_acc_methods" if sensor == "acc" else "_gyro_methods")
        streams: list[tuple[str, str]] = [(self._sensor_primary_raw_stream_id(sensor), self._sensor_primary_raw_label(sensor))]
        for method_id, plugin in methods.items():
            if plugin.realtime_enabled or plugin.last_output is not None:
                streams.append((method_id, plugin.name))
        return streams

    def _resolve_accelerometer_selected_output(
        self,
        raw_roll_deg: float | None,
        raw_pitch_deg: float | None,
        derived_streams: dict[str, dict[str, Any]],
    ) -> tuple[float | None, float | None, str]:
        self._ensure_sensor_method_state("acc")
        if self._acc_primary_output_stream_id == "raw_tilt":
            return (raw_roll_deg, raw_pitch_deg, "Raw Tilt")
        plugin = self._acc_methods.get(self._acc_primary_output_stream_id)
        if plugin is None:
            return (raw_roll_deg, raw_pitch_deg, "Raw Tilt")
        selected = derived_streams.get(self._acc_primary_output_stream_id)
        if selected is None:
            selected = self._acc_latest_derived_streams.get(self._acc_primary_output_stream_id)
        if selected is None:
            return (None, None, plugin.name)
        return (selected.get("roll_deg"), selected.get("pitch_deg"), plugin.name)

    def _resolve_gyroscope_selected_output(self, derived_streams: dict[str, dict[str, Any]]) -> str:
        self._ensure_sensor_method_state("gyro")
        if self._gyro_primary_output_stream_id == "raw_gyroscope":
            return "Raw Gyroscope"
        plugin = self._gyro_methods.get(self._gyro_primary_output_stream_id)
        if plugin is None:
            return "Raw Gyroscope"
        if self._gyro_primary_output_stream_id in derived_streams:
            return plugin.name
        if self._gyro_primary_output_stream_id in self._gyro_latest_derived_streams:
            return plugin.name
        return plugin.name

    def _refresh_sensor_sources_ui(self, sensor: str) -> None:
        self._ensure_sensor_source_state(sensor)
        tab = self._sensor_tab(sensor)
        if tab is None:
            return
        sources = getattr(self, "_acc_sources" if sensor == "acc" else "_gyro_sources")
        selected_source_id = getattr(self, "_acc_selected_source_id" if sensor == "acc" else "_gyro_selected_source_id")
        tab.set_source_states(sources, selected_source_id)
        self._refresh_sensor_output_routing_ui(sensor)

    def _refresh_sensor_methods_ui(self, sensor: str) -> None:
        self._ensure_sensor_method_state(sensor)
        self._ensure_sensor_calibration_state(sensor)
        self._update_sensor_method_routing_state(sensor)
        tab = self._sensor_tab(sensor)
        if tab is None:
            return
        methods: dict[str, LoadedMethodPlugin] = getattr(self, "_acc_methods" if sensor == "acc" else "_gyro_methods")
        selected_method_id = getattr(self, "_acc_selected_method_id" if sensor == "acc" else "_gyro_selected_method_id")
        method_states = {
            method_id: {
                "name": plugin.name,
                "version": plugin.version,
                "status": plugin.display_status(),
                "status_text": plugin.display_status_text(),
                "progress": plugin.calibration_progress if (plugin.is_calibrating or plugin.calibration_params is not None) else 0.0,
                "show": plugin.show_enabled,
                "show_enabled": plugin.can_toggle_show(),
                "can_calibrate": self._can_start_sensor_calibration(sensor, plugin),
                "can_load_params": plugin.can_load_params(),
                "can_save_params": plugin.can_save_params(),
                "realtime_enabled": plugin.realtime_enabled,
                "can_enable_realtime": plugin.can_enable_realtime(),
                "can_disable_realtime": plugin.can_disable_realtime(),
                "record": plugin.record_enabled,
                "can_record": plugin.can_toggle_record(),
                "stream_id": plugin.derived_stream_id or f"derived_{plugin.method_id}",
                "live_output": dict(plugin.last_output) if plugin.last_output is not None else None,
            }
            for method_id, plugin in methods.items()
        }
        tab.set_method_states(method_states, selected_method_id)
        selected = methods.get(selected_method_id or "")
        if selected is None:
            tab.update_selected_method_details(name="-", version="-", path="-", status="-", capabilities="-")
            tab.set_selected_method_actions(
                can_calibrate=False,
                can_load_params=False,
                can_save_params=False,
                can_clear_params=False,
                can_enable_realtime=False,
                can_disable_realtime=False,
            )
            self._refresh_sensor_output_routing_ui(sensor)
            return
        tab.update_selected_method_details(
            name=selected.name,
            version=selected.version,
            path=selected.file_path,
            status=selected.display_status_text(),
            capabilities=selected.capabilities_label(),
        )
        tab.set_selected_method_actions(
            can_calibrate=self._can_start_sensor_calibration(sensor, selected),
            can_load_params=selected.can_load_params(),
            can_save_params=selected.can_save_params(),
            can_clear_params=selected.can_clear_params(),
            can_enable_realtime=selected.can_enable_realtime(),
            can_disable_realtime=selected.can_disable_realtime(),
        )
        self._refresh_sensor_output_routing_ui(sensor)

    def _refresh_sensor_output_routing_ui(self, sensor: str) -> None:
        self._ensure_sensor_method_state(sensor)
        tab = self._sensor_tab(sensor)
        if tab is None:
            return
        prefix = "_acc" if sensor == "acc" else "_gyro"
        stream_choices = self._sensor_available_output_streams(sensor)
        available_ids = {stream_id for stream_id, _ in stream_choices}
        primary_attr = f"{prefix}_primary_output_stream_id"
        if getattr(self, primary_attr) not in available_ids:
            setattr(self, primary_attr, self._sensor_primary_raw_stream_id(sensor))
        tab.set_heading_routing(
            stream_choices=stream_choices,
            primary_stream_id=getattr(self, primary_attr),
        )
        snapshot = getattr(self, f"{prefix}_last_live_snapshot")
        if snapshot is None:
            return
        if sensor == "acc":
            roll_deg, pitch_deg, source_label = self._resolve_accelerometer_selected_output(
                snapshot.get("raw_roll_deg"),
                snapshot.get("raw_pitch_deg"),
                snapshot.get("derived_streams", {}),
            )
            tab.set_primary_output_display(
                source_label=source_label,
                roll_deg=roll_deg,
                pitch_deg=pitch_deg,
            )
            return
        tab.set_primary_output_display(
            source_label=self._resolve_gyroscope_selected_output(snapshot.get("derived_streams", {})),
        )

    def _sensor_dataset_choice_labels(self, sensor: str) -> list[str]:
        datasets = getattr(self, "_acc_datasets" if sensor == "acc" else "_gyro_datasets")
        seen: dict[str, int] = {}
        labels: list[str] = []
        for dataset in datasets:
            base_name = dataset.summary()["name"]
            count = seen.get(base_name, 0) + 1
            seen[base_name] = count
            labels.append(base_name if count == 1 else f"{base_name} ({count})")
        return labels

    def _sensor_dataset_dialog_label(self, dataset: Any) -> str:
        summary = dataset.summary()
        return f"{summary['name']}  [{summary['row_count']} rows, {summary['time_range']}]"

    def _rebuild_sensor_dataset_table(self, sensor: str) -> None:
        tab = self._sensor_tab(sensor)
        if tab is None:
            return
        dataset = getattr(self, "_acc_dataset" if sensor == "acc" else "_gyro_dataset")
        if dataset is None:
            tab.clear_dataset_rows()
            return
        tab.set_dataset_records(dataset.records)

    def _set_active_sensor_dataset(self, sensor: str, dataset: Any | None) -> None:
        self._ensure_sensor_method_state(sensor)
        prefix = "_acc" if sensor == "acc" else "_gyro"
        setattr(self, f"{prefix}_dataset", dataset)
        setattr(self, f"{prefix}_dataset_export_path", "" if dataset is None or dataset.source_path is None else dataset.source_path)
        shared_dataset = self._find_shared_imu_dataset(dataset)
        if shared_dataset is not None:
            self._imu_dataset = shared_dataset
            self._imu_dataset_export_path = shared_dataset.source_path or ""
        self._rebuild_sensor_dataset_table(sensor)
        self._refresh_sensor_dataset_ui(sensor)
        self._refresh_all_sensor_method_dataset_clouds(sensor)

    def _refresh_sensor_dataset_ui(self, sensor: str) -> None:
        self._ensure_sensor_method_state(sensor)
        tab = self._sensor_tab(sensor)
        if tab is None:
            return
        prefix = "_acc" if sensor == "acc" else "_gyro"
        datasets = getattr(self, f"{prefix}_datasets")
        dataset = getattr(self, f"{prefix}_dataset")
        recording_active = bool(getattr(self, f"{prefix}_recording_active"))
        dataset_labels = self._sensor_dataset_choice_labels(sensor)
        active_index = None
        if dataset is not None:
            try:
                active_index = datasets.index(dataset)
            except ValueError:
                active_index = None
        tab.set_dataset_choices(dataset_labels, active_index)
        if dataset is None:
            tab.update_dataset_summary(name="—", row_count="0", source_count="0", time_range="—")
            tab.set_recording_state(
                is_recording=recording_active,
                can_save=False,
                has_dataset=False,
                can_concatenate=len(datasets) >= 2,
            )
            tab.set_clear_dataset_enabled(False)
            tab.clear_dataset_rows()
            return
        summary = dataset.summary()
        active_name = summary["name"] if active_index is None else dataset_labels[active_index]
        tab.update_dataset_summary(
            name=active_name,
            row_count=summary["row_count"],
            source_count=summary["source_count"],
            time_range=summary["time_range"],
        )
        can_save = bool(dataset.records) and not recording_active
        tab.set_recording_state(
            is_recording=recording_active,
            can_save=can_save,
            has_dataset=bool(dataset.records),
            can_concatenate=len(datasets) >= 2,
        )
        tab.set_clear_dataset_enabled(bool(datasets) and not recording_active)
        setattr(self, f"{prefix}_last_dataset_ui_refresh_s", time.monotonic())

    def _refresh_sensor_dataset_ui_if_due(self, sensor: str, *, force: bool = False) -> None:
        self._ensure_sensor_method_state(sensor)
        prefix = "_acc" if sensor == "acc" else "_gyro"
        attr_name = f"{prefix}_last_dataset_ui_refresh_s"
        if not hasattr(self, attr_name):
            setattr(self, attr_name, 0.0)
        if not hasattr(self, "_sensor_dataset_ui_refresh_interval_s"):
            self._sensor_dataset_ui_refresh_interval_s = 0.2
        now = time.monotonic()
        if force or (now - getattr(self, attr_name)) >= self._sensor_dataset_ui_refresh_interval_s:
            self._refresh_sensor_dataset_ui(sensor)

    def _build_sensor_recording_metadata(self, sensor: str) -> dict[str, Any]:
        self._ensure_sensor_source_state(sensor)
        self._ensure_sensor_method_state(sensor)
        sources = getattr(self, "_acc_sources" if sensor == "acc" else "_gyro_sources")
        methods: dict[str, LoadedMethodPlugin] = getattr(self, "_acc_methods" if sensor == "acc" else "_gyro_methods")
        producers = self._sensor_calibration_producers(sensor)
        source_entries = [
            {
                "source_id": source_id,
                "title": str(source.get("title", source_id)),
                "show": bool(source.get("show", False)),
                "record": bool(source.get("record", False)),
            }
            for source_id, source in sources.items()
        ]
        filter_entries = [
            {
                "method_id": method_id,
                "name": plugin.name,
                "version": plugin.version,
                "file_path": plugin.file_path,
                "stream_id": plugin.derived_stream_id or f"derived_{plugin.method_id}",
                "show": bool(plugin.show_enabled),
                "record": bool(plugin.record_enabled),
                "realtime_enabled": bool(plugin.realtime_enabled),
                "params": plugin.calibration_params,
                "stream_bindings": {
                    SCOPE_CALIBRATE: self._effective_sensor_stream_bindings(sensor, plugin, scope=SCOPE_CALIBRATE, producers=producers),
                    SCOPE_PROCESS: self._effective_sensor_stream_bindings(sensor, plugin, scope=SCOPE_PROCESS, producers=producers),
                },
            }
            for method_id, plugin in methods.items()
        ]
        recorded_streams = [
            {
                "stream_id": source_id,
                "kind": "source",
                "title": str(source.get("title", source_id)),
                "stream_type": "raw",
                "flags": "tilt_only" if source_id == "raw_tilt" else "",
            }
            for source_id, source in sources.items()
            if bool(source.get("record", False))
        ]
        recorded_streams.extend(
            {
                "stream_id": plugin.derived_stream_id or f"derived_{plugin.method_id}",
                "kind": "method",
                "method_id": method_id,
                "name": plugin.name,
                "version": plugin.version,
                "file_path": plugin.file_path,
                "realtime_enabled": bool(plugin.realtime_enabled),
                "params": plugin.calibration_params,
            }
            for method_id, plugin in methods.items()
            if bool(plugin.record_enabled)
        )
        return {
            "schema_version": "1",
            "saved_at": datetime.now().isoformat(timespec="seconds"),
            "sources": source_entries,
            "filters": filter_entries,
            "recorded_streams": recorded_streams,
            "recorded_method_ids": [
                entry["method_id"]
                for entry in filter_entries
                if bool(entry.get("record", False))
            ],
            "method_bindings": [
                {
                    "file_path": plugin.file_path,
                    "name": plugin.name,
                    "bindings": {
                        SCOPE_CALIBRATE: self._effective_sensor_stream_bindings(sensor, plugin, scope=SCOPE_CALIBRATE, producers=producers),
                        SCOPE_PROCESS: self._effective_sensor_stream_bindings(sensor, plugin, scope=SCOPE_PROCESS, producers=producers),
                    },
                }
                for plugin in methods.values()
            ],
        }

    def _update_sensor_dataset_recording_metadata(self, sensor: str, dataset: Any | None = None) -> None:
        prefix = "_acc" if sensor == "acc" else "_gyro"
        target_dataset = getattr(self, f"{prefix}_dataset") if dataset is None else dataset
        if target_dataset is None:
            return
        target_dataset.metadata = self._build_sensor_recording_metadata(sensor)
        if self._imu_dataset is not None and (self._imu_recording_active or self._is_shared_projection_dataset(target_dataset)):
            self._update_imu_dataset_recording_metadata()

    def _sensor_make_dataset_name(self, sensor: str) -> str:
        base = "acc_record" if sensor == "acc" else "gyro_record"
        return f"{base}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

    def _sensor_make_concatenated_dataset_name(self, sensor: str) -> str:
        base = "acc_concat" if sensor == "acc" else "gyro_concat"
        return f"{base}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

    def _on_sensor_start_record(self, sensor: str) -> None:
        self._ensure_sensor_method_state(sensor)
        self._start_shared_imu_recording(requested_name=self._sensor_make_dataset_name(sensor))

    def _on_sensor_stop_record(self, sensor: str) -> None:
        self._ensure_sensor_method_state(sensor)
        self._stop_shared_imu_recording()

    def _choose_sensor_dataset_load_path(self, sensor: str) -> str:
        prefix = "_acc" if sensor == "acc" else "_gyro"
        path = filedialog.askopenfilename(
            title=f"Load {self._sensor_title(sensor).lower()} dataset",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            initialdir=self._sensor_dialog_initialdir(sensor, "dataset", fallback_path=getattr(self, f"{prefix}_dataset_export_path")),
            parent=self.root,
        )
        self._remember_sensor_dialog_dir(sensor, "dataset", path)
        return path

    def _choose_sensor_dataset_load_paths(self, sensor: str) -> tuple[str, ...]:
        prefix = "_acc" if sensor == "acc" else "_gyro"
        paths = filedialog.askopenfilenames(
            title=f"Load {self._sensor_title(sensor).lower()} datasets",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            initialdir=self._sensor_dialog_initialdir(sensor, "dataset", fallback_path=getattr(self, f"{prefix}_dataset_export_path")),
            parent=self.root,
        )
        if paths:
            self._remember_sensor_dialog_dir(sensor, "dataset", paths[0])
        return paths

    def _choose_sensor_dataset_path(self, sensor: str) -> str:
        prefix = "_acc" if sensor == "acc" else "_gyro"
        path = filedialog.asksaveasfilename(
            title=f"Save {self._sensor_title(sensor).lower()} dataset",
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            initialdir=self._sensor_dialog_initialdir(sensor, "dataset", fallback_path=getattr(self, f"{prefix}_dataset_export_path")),
            parent=self.root,
        )
        self._remember_sensor_dialog_dir(sensor, "dataset", path)
        return path

    def _choose_sensor_params_load_path(self, sensor: str) -> str:
        path = filedialog.askopenfilename(
            title=f"Load {self._sensor_title(sensor).lower()} params",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            initialdir=self._sensor_dialog_initialdir(sensor, "params"),
            parent=self.root,
        )
        self._remember_sensor_dialog_dir(sensor, "params", path)
        return path

    def _choose_sensor_params_save_path(self, sensor: str) -> str:
        path = filedialog.asksaveasfilename(
            title=f"Save {self._sensor_title(sensor).lower()} params",
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            initialdir=self._sensor_dialog_initialdir(sensor, "params"),
            parent=self.root,
        )
        self._remember_sensor_dialog_dir(sensor, "params", path)
        return path

    def _load_sensor_dataset_paths(self, sensor: str, paths: Sequence[str], *, show_errors: bool = True) -> int:
        self._ensure_sensor_method_state(sensor)
        prefix = "_acc" if sensor == "acc" else "_gyro"
        dataset_cls = AccelerometerDataset if sensor == "acc" else GyroscopeDataset
        loaded: list[Any] = []
        errors: list[str] = []
        for path in paths:
            try:
                if ImuDataset.is_imu_csv(path):
                    self._load_shared_imu_dataset_path(path)
                    loaded.append(getattr(self, f"{prefix}_dataset"))
                    continue
                dataset = dataset_cls.from_csv(path)
            except Exception as exc:
                errors.append(f"{os.path.basename(path)}: {exc}")
                continue
            getattr(self, f"{prefix}_datasets").append(dataset)
            loaded.append(dataset)

        if loaded:
            self._set_active_sensor_dataset(sensor, loaded[-1])
            self._remember_sensor_dialog_dir(sensor, "dataset", loaded[-1].source_path or "")
            self._save_settings()
        if errors and show_errors:
            self._show_error(f"{self._sensor_title(sensor)} Dataset", "Failed to load:\n" + "\n".join(errors))
        return len(loaded)

    def _save_sensor_dataset_to_path(self, sensor: str, path: str) -> bool:
        self._ensure_sensor_method_state(sensor)
        prefix = "_acc" if sensor == "acc" else "_gyro"
        dataset = getattr(self, f"{prefix}_dataset")
        if dataset is None or not dataset.records:
            self._show_info(f"{self._sensor_title(sensor)} Dataset", "There is no recorded dataset to save.")
            return False
        shared_dataset = self._find_shared_imu_dataset(dataset)
        if shared_dataset is not None:
            self._update_imu_dataset_recording_metadata(shared_dataset)
            try:
                shared_dataset.to_csv(path)
            except Exception as exc:
                self._show_error(f"{self._sensor_title(sensor)} Dataset", f"Failed to save dataset: {exc}")
                return False
            self._bind_shared_imu_path(shared_dataset, path)
            setattr(self, f"{prefix}_dataset_export_path", path)
            self._remember_sensor_dialog_dir(sensor, "dataset", path)
            self._refresh_sensor_dataset_ui(sensor)
            self._refresh_magnetometer_dataset_ui()
            self._save_settings()
            return True
        if dataset.source_path is None or bool(getattr(self, f"{prefix}_recording_active", False)):
            self._update_sensor_dataset_recording_metadata(sensor)
        try:
            dataset.to_csv(path)
        except Exception as exc:
            self._show_error(f"{self._sensor_title(sensor)} Dataset", f"Failed to save dataset: {exc}")
            return False
        setattr(self, f"{prefix}_dataset_export_path", path)
        self._remember_sensor_dialog_dir(sensor, "dataset", path)
        self._refresh_sensor_dataset_ui(sensor)
        self._save_settings()
        return True

    def _on_sensor_load_csv(self, sensor: str) -> None:
        prefix = "_acc" if sensor == "acc" else "_gyro"
        if getattr(self, f"{prefix}_recording_active", False):
            return
        path = self._choose_sensor_dataset_load_path(sensor)
        if not path:
            return
        self._load_sensor_dataset_paths(sensor, (path,))

    def _on_sensor_load_multiple(self, sensor: str) -> None:
        prefix = "_acc" if sensor == "acc" else "_gyro"
        if getattr(self, f"{prefix}_recording_active", False):
            return
        paths = self._choose_sensor_dataset_load_paths(sensor)
        if not paths:
            return
        self._load_sensor_dataset_paths(sensor, paths)

    def _on_select_sensor_dataset(self, sensor: str, index: int) -> None:
        prefix = "_acc" if sensor == "acc" else "_gyro"
        if getattr(self, f"{prefix}_recording_active", False):
            return
        datasets = getattr(self, f"{prefix}_datasets")
        if 0 <= index < len(datasets):
            self._set_active_sensor_dataset(sensor, datasets[index])
            self._save_settings()

    def _on_select_sensor_source(self, sensor: str, source_id: str) -> None:
        self._ensure_sensor_source_state(sensor)
        prefix = "_acc" if sensor == "acc" else "_gyro"
        sources = getattr(self, f"{prefix}_sources")
        if source_id not in sources:
            return
        setattr(self, f"{prefix}_selected_source_id", source_id)
        self._refresh_sensor_sources_ui(sensor)
        if bool(getattr(self, f"{prefix}_recording_active", False)):
            self._update_sensor_dataset_recording_metadata(sensor)
        self._save_settings()

    def _on_toggle_sensor_source_show(self, sensor: str, source_id: str, enabled: bool) -> None:
        self._ensure_sensor_source_state(sensor)
        prefix = "_acc" if sensor == "acc" else "_gyro"
        sources = getattr(self, f"{prefix}_sources")
        if source_id not in sources:
            return
        sources[source_id]["show"] = bool(enabled)
        self._refresh_sensor_sources_ui(sensor)
        if bool(getattr(self, f"{prefix}_recording_active", False)):
            self._update_sensor_dataset_recording_metadata(sensor)
        self._save_settings()

    def _on_toggle_sensor_source_record(self, sensor: str, source_id: str, enabled: bool) -> None:
        self._ensure_sensor_source_state(sensor)
        prefix = "_acc" if sensor == "acc" else "_gyro"
        sources = getattr(self, f"{prefix}_sources")
        if source_id not in sources:
            return
        sources[source_id]["record"] = bool(enabled)
        self._refresh_sensor_sources_ui(sensor)
        if bool(getattr(self, f"{prefix}_recording_active", False)):
            self._update_sensor_dataset_recording_metadata(sensor)
        self._save_settings()

    def _on_toggle_sensor_method_show(self, sensor: str, method_id: str, enabled: bool) -> None:
        self._ensure_sensor_method_state(sensor)
        prefix = "_acc" if sensor == "acc" else "_gyro"
        methods: dict[str, LoadedMethodPlugin] = getattr(self, f"{prefix}_methods")
        plugin = methods.get(method_id)
        if plugin is None:
            return
        plugin.show_enabled = bool(enabled)
        latest_derived = getattr(self, f"{prefix}_latest_derived_streams")
        stream_state = latest_derived.get(method_id)
        if stream_state is not None:
            stream_state["show"] = plugin.show_enabled
        self._refresh_sensor_methods_ui(sensor)
        if bool(getattr(self, f"{prefix}_recording_active", False)):
            self._update_sensor_dataset_recording_metadata(sensor)
        self._save_settings()

    def _on_enable_sensor_method_realtime(self, sensor: str, method_id: str) -> None:
        self._ensure_sensor_method_state(sensor)
        prefix = "_acc" if sensor == "acc" else "_gyro"
        methods: dict[str, LoadedMethodPlugin] = getattr(self, f"{prefix}_methods")
        plugin = methods.get(method_id)
        if plugin is None:
            return
        if not plugin.can_enable_realtime():
            if plugin.calibration_params is None:
                plugin.process_warnings = ["Load or create calibration params before enabling realtime."]
                self._refresh_sensor_methods_ui(sensor)
                self._show_warning(
                    f"{self._sensor_title(sensor)} Realtime",
                    f"{plugin.name} requires calibration params before realtime can be enabled.",
                )
            return
        plugin.realtime_enabled = True
        plugin.show_enabled = True
        plugin.record_enabled = True
        plugin.process_warnings = []
        self._refresh_sensor_methods_ui(sensor)
        if bool(getattr(self, f"{prefix}_recording_active", False)):
            self._update_sensor_dataset_recording_metadata(sensor)
        self._save_settings()

    def _on_disable_sensor_method_realtime(self, sensor: str, method_id: str) -> None:
        self._ensure_sensor_method_state(sensor)
        prefix = "_acc" if sensor == "acc" else "_gyro"
        methods: dict[str, LoadedMethodPlugin] = getattr(self, f"{prefix}_methods")
        plugin = methods.get(method_id)
        if plugin is None:
            return
        plugin.realtime_enabled = False
        plugin.record_enabled = False
        plugin.last_output = None
        getattr(self, f"{prefix}_latest_derived_streams").pop(method_id, None)
        for kind in self._sensor_method_output_kinds(sensor, plugin):
            self._calibration_stream_snapshots.pop(make_method_producer_id(plugin.file_path, kind), None)
        self._refresh_sensor_methods_ui(sensor)
        if bool(getattr(self, f"{prefix}_recording_active", False)):
            self._update_sensor_dataset_recording_metadata(sensor)
        self._save_settings()

    def _on_toggle_sensor_method_record(self, sensor: str, method_id: str, enabled: bool) -> None:
        self._ensure_sensor_method_state(sensor)
        prefix = "_acc" if sensor == "acc" else "_gyro"
        methods: dict[str, LoadedMethodPlugin] = getattr(self, f"{prefix}_methods")
        plugin = methods.get(method_id)
        if plugin is None:
            return
        if not plugin.can_toggle_record() and enabled:
            return
        plugin.record_enabled = bool(enabled and plugin.realtime_enabled)
        self._refresh_sensor_methods_ui(sensor)
        if bool(getattr(self, f"{prefix}_recording_active", False)):
            self._update_sensor_dataset_recording_metadata(sensor)
        self._save_settings()

    def _on_select_sensor_primary_output(self, sensor: str, stream_id: str) -> None:
        self._ensure_sensor_method_state(sensor)
        prefix = "_acc" if sensor == "acc" else "_gyro"
        available_ids = {stream_id for stream_id, _ in self._sensor_available_output_streams(sensor)}
        if stream_id not in available_ids:
            return
        setattr(self, f"{prefix}_primary_output_stream_id", stream_id)
        self._refresh_sensor_output_routing_ui(sensor)
        self._save_settings()

    def _make_sensor_method_id(self, sensor: str) -> str:
        methods: dict[str, LoadedMethodPlugin] = getattr(self, "_acc_methods" if sensor == "acc" else "_gyro_methods")
        return f"method_{len(methods) + 1}"

    def _register_sensor_method(self, sensor: str, plugin: LoadedMethodPlugin) -> LoadedMethodPlugin:
        self._ensure_sensor_method_state(sensor)
        prefix = "_acc" if sensor == "acc" else "_gyro"
        methods: dict[str, LoadedMethodPlugin] = getattr(self, f"{prefix}_methods")
        plugin.method_id = self._make_sensor_method_id(sensor)
        plugin.derived_stream_id = f"derived_{plugin.method_id}"
        methods[plugin.method_id] = plugin
        setattr(self, f"{prefix}_selected_method_id", plugin.method_id)
        self._refresh_sensor_methods_ui(sensor)
        return plugin

    def _snapshot_sensor_dataset(self, sensor: str, dataset: Any) -> Any:
        if sensor == "acc":
            return AccelerometerDataset(dataset.name, records=list(dataset.records), source_path=dataset.source_path, metadata=dict(getattr(dataset, "metadata", {})))
        return GyroscopeDataset(dataset.name, records=list(dataset.records), source_path=dataset.source_path, metadata=dict(getattr(dataset, "metadata", {})))

    def _compute_offline_sensor_records(
        self,
        sensor: str,
        plugin: LoadedMethodPlugin,
        dataset: Any,
        *,
        cache: dict[str, list[Any]],
        stack: set[str],
    ) -> list[Any]:
        cache_key = f"{sensor}::{plugin.file_path}"
        if cache_key in cache:
            return list(cache[cache_key])
        if cache_key in stack:
            return []
        if (
            plugin.module is None
            or not plugin.supports_process()
            or plugin.calibration_params is None
            or not callable(getattr(plugin.module, "process", None))
        ):
            return []
        stack.add(cache_key)
        resolver = self._build_sensor_calibration_resolver(sensor, plugin, dataset, cache=cache, stack=stack)
        requirements = self._sensor_method_requirements(sensor, plugin, scope=SCOPE_CALIBRATE)
        if not requirements:
            requirements = self._sensor_method_requirements(sensor, plugin, scope=SCOPE_PROCESS)
        inputs_payload = resolver.export_inputs(requirements)
        if requirements:
            primary_records = list(inputs_payload.get(requirements[0].slot, {}).get("records", []))
        else:
            primary_records = [self._build_sensor_sample_from_record(sensor, record) for record in self._sensor_raw_dataset_records(sensor, dataset)]
        derived_records: list[Any] = []
        for index in range(len(primary_records)):
            sample = self._build_sensor_process_sample_from_inputs(sensor, requirements, inputs_payload, index) if requirements else dict(primary_records[index])
            if sample is None:
                continue
            result = run_method_process(plugin, sample, plugin.calibration_params)
            if not result.ok or result.output is None:
                if result.diagnostics is not None:
                    plugin.diagnostics = result.diagnostics
                plugin.status = "error"
                plugin.realtime_enabled = False
                plugin.record_enabled = False
                stack.remove(cache_key)
                return []
            merged_output = dict(sample)
            merged_output.update(result.output)
            normalized, diagnostics = self._normalize_sensor_process_output(sensor, plugin, merged_output)
            if diagnostics is not None or normalized is None:
                plugin.diagnostics = diagnostics
                plugin.status = "error"
                plugin.realtime_enabled = False
                plugin.record_enabled = False
                stack.remove(cache_key)
                return []
            derived_records.append(self._build_sensor_derived_record(sensor, plugin, normalized))
        stack.remove(cache_key)
        cache[cache_key] = list(derived_records)
        return list(derived_records)

    def _dataset_records_for_sensor_stream_producer(
        self,
        sensor: str,
        producer: StreamProducerDescriptor,
        dataset: Any,
        *,
        cache: dict[str, list[Any]],
        stack: set[str],
    ) -> list[Any]:
        source_sensor = str(producer.details.get("source_sensor", sensor))
        source_dataset = self._paired_sensor_dataset_for_calibration(sensor, source_sensor, dataset)
        if source_dataset is None:
            return []
        if producer.origin == "builtin":
            return self._records_for_sensor_builtin_producer(sensor, producer, source_dataset)
        if producer.producer_id in cache:
            return list(cache[producer.producer_id])
        records = [record for record in source_dataset.records if record.stream_id == producer.stream_id]
        if records:
            cache[producer.producer_id] = list(records)
            return list(records)
        methods: dict[str, LoadedMethodPlugin] = getattr(self, "_acc_methods" if source_sensor == "acc" else "_gyro_methods")
        upstream_plugin = None
        for plugin in methods.values():
            if plugin.file_path == producer.file_path:
                upstream_plugin = plugin
                break
        if upstream_plugin is None:
            return []
        records = self._compute_offline_sensor_records(source_sensor, upstream_plugin, source_dataset, cache=cache, stack=stack)
        cache[producer.producer_id] = list(records)
        return list(records)

    def _build_sensor_calibration_resolver(
        self,
        sensor: str,
        plugin: LoadedMethodPlugin,
        dataset: Any,
        *,
        cache: dict[str, list[Any]] | None = None,
        stack: set[str] | None = None,
    ) -> CalibrationStreamResolver:
        producer_map = self._sensor_calibration_producers(sensor, dataset)
        bindings = self._effective_sensor_stream_bindings(sensor, plugin, scope=SCOPE_CALIBRATE, producers=producer_map)
        record_cache: dict[str, list[Any]] = {} if cache is None else cache
        active_stack: set[str] = set() if stack is None else stack
        return CalibrationStreamResolver(
            producers=producer_map,
            bindings=bindings,
            live_snapshots=self._calibration_stream_snapshots,
            dataset_records_provider=lambda producer: self._dataset_records_for_sensor_stream_producer(
                sensor,
                producer,
                dataset,
                cache=record_cache,
                stack=active_stack,
            ),
            record_adapter=lambda producer, record: self._adapt_sensor_record_for_producer(sensor, producer, record),
            latest_adapter=lambda snapshot: dict(snapshot.payload),
        )

    def _build_sensor_calibration_config(self, sensor: str, plugin: LoadedMethodPlugin, dataset: Any) -> dict[str, Any]:
        self._update_sensor_method_routing_state(sensor)
        producer_map = self._sensor_calibration_producers(sensor, dataset)
        bindings = self._effective_sensor_stream_bindings(sensor, plugin, scope=SCOPE_CALIBRATE, producers=producer_map)
        resolver = self._build_sensor_calibration_resolver(sensor, plugin, dataset)
        requirements = self._sensor_method_requirements(sensor, plugin, scope=SCOPE_CALIBRATE)
        return {
            "stream_bindings": bindings,
            "stream_requirements": [requirement.as_dict() for requirement in requirements],
            "stream_producers": {producer_id: descriptor.as_dict() for producer_id, descriptor in producer_map.items()},
            "selected_producers": {
                slot: producer_map[producer_id].as_dict()
                for slot, producer_id in bindings.items()
                if producer_id in producer_map
            },
            "stream_resolver": resolver,
            "stream_inputs": resolver.export_inputs(requirements),
            "routing_validation": self._method_routing_validation_payload(plugin),
        }

    def _refresh_sensor_method_clouds_ui(self, sensor: str) -> None:
        tab = self._sensor_tab(sensor)
        if tab is None:
            return
        method_clouds = getattr(self, "_acc_offline_method_clouds" if sensor == "acc" else "_gyro_offline_method_clouds")
        tab.set_method_dataset_clouds(method_clouds)

    def _recompute_sensor_method_dataset_cloud(self, sensor: str, method_id: str, *, open_diagnostics: bool = False) -> None:
        del open_diagnostics
        self._ensure_sensor_method_state(sensor)
        prefix = "_acc" if sensor == "acc" else "_gyro"
        methods: dict[str, LoadedMethodPlugin] = getattr(self, f"{prefix}_methods")
        offline_clouds = getattr(self, f"{prefix}_offline_method_clouds")
        plugin = methods.get(method_id)
        if plugin is None:
            offline_clouds.pop(method_id, None)
            return
        if (
            plugin.module is None
            or not plugin.supports_process()
            or plugin.calibration_params is None
            or not callable(getattr(plugin.module, "process", None))
        ):
            offline_clouds.pop(method_id, None)
            return
        dataset = getattr(self, f"{prefix}_dataset")
        if dataset is None:
            offline_clouds.pop(method_id, None)
            return
        derived_records = self._compute_offline_sensor_records(sensor, plugin, dataset, cache={}, stack=set())
        if not derived_records:
            offline_clouds.pop(method_id, None)
            return
        offline_clouds[method_id] = derived_records

    def _refresh_all_sensor_method_dataset_clouds(self, sensor: str, *, open_diagnostics: bool = False) -> None:
        self._ensure_sensor_method_state(sensor)
        methods: dict[str, LoadedMethodPlugin] = getattr(self, "_acc_methods" if sensor == "acc" else "_gyro_methods")
        for method_id in list(methods):
            self._recompute_sensor_method_dataset_cloud(sensor, method_id, open_diagnostics=open_diagnostics)
        self._refresh_sensor_methods_ui(sensor)
        self._refresh_sensor_method_clouds_ui(sensor)

    def _apply_sensor_param_io_result(
        self,
        sensor: str,
        method_id: str,
        result: ParamIoResult,
        *,
        action: str,
        path: str,
        show_warnings: bool = True,
        open_diagnostics: bool = True,
    ) -> None:
        self._ensure_sensor_method_state(sensor)
        methods: dict[str, LoadedMethodPlugin] = getattr(self, "_acc_methods" if sensor == "acc" else "_gyro_methods")
        offline_clouds = getattr(self, "_acc_offline_method_clouds" if sensor == "acc" else "_gyro_offline_method_clouds")
        plugin = methods.get(method_id)
        if plugin is None:
            return
        if result.ok:
            plugin.status = plugin.load_status
            plugin.diagnostics = None
            plugin.params_warnings = list(result.warnings)
            plugin.process_warnings = []
            if action == "load":
                plugin.calibration_params = result.params
                plugin.params_profile_path = path
                plugin.calibration_report = ""
                plugin.calibration_progress = 1.0
                self._recompute_sensor_method_dataset_cloud(sensor, method_id, open_diagnostics=False)
                if bool(getattr(self, "_acc_recording_active" if sensor == "acc" else "_gyro_recording_active", False)):
                    self._update_sensor_dataset_recording_metadata(sensor)
            elif action == "save" and result.params is not None:
                plugin.calibration_params = result.params
                plugin.params_profile_path = path
            self._refresh_sensor_methods_ui(sensor)
            self._refresh_sensor_method_clouds_ui(sensor)
            if result.warnings and show_warnings:
                verb = "loaded" if action == "load" else "saved"
                self._show_warning(
                    f"{self._sensor_title(sensor)} Params",
                    f"Param profile {verb} with warnings:\n" + "\n".join(result.warnings),
                )
            self._save_settings()
            return

        plugin.status = "error"
        if result.diagnostics is not None:
            plugin.diagnostics = result.diagnostics
        else:
            plugin.diagnostics = self._make_magnetometer_method_diagnostics(
                plugin,
                last_action=f"{action}_params",
                error_text=f"{action.title()} params failed.",
                warnings=result.warnings,
            )
        offline_clouds.pop(method_id, None)
        self._refresh_sensor_methods_ui(sensor)
        self._refresh_sensor_method_clouds_ui(sensor)
        if open_diagnostics:
            self._open_sensor_method_diagnostics(sensor, method_id)
        self._save_settings()

    def _on_calibrate_sensor_method(self, sensor: str, method_id: str) -> None:
        self._ensure_sensor_method_state(sensor)
        self._ensure_sensor_calibration_state(sensor)
        prefix = "_acc" if sensor == "acc" else "_gyro"
        methods: dict[str, LoadedMethodPlugin] = getattr(self, f"{prefix}_methods")
        plugin = methods.get(method_id)
        if plugin is None:
            return
        if not plugin.supports_calibrate():
            self._show_warning(f"{self._sensor_title(sensor)} Calibration", f"{plugin.name} does not support calibration.")
            return
        if plugin.is_calibrating:
            return
        dataset = getattr(self, f"{prefix}_dataset")
        if dataset is None or not dataset.records:
            plugin.calibration_warnings = ["No active non-empty dataset loaded for calibration."]
            self._refresh_sensor_methods_ui(sensor)
            self._show_warning(f"{self._sensor_title(sensor)} Calibration", "Load or record a non-empty dataset first.")
            return
        self._update_sensor_method_routing_state(sensor)
        validation = plugin.routing_validation.get(SCOPE_CALIBRATE)
        if validation is not None and not validation.ok:
            plugin.calibration_warnings = list(validation.issues)
            self._refresh_sensor_methods_ui(sensor)
            self._show_warning(
                f"{self._sensor_title(sensor)} Calibration",
                "Resolve method input bindings before calibration:\n" + "\n".join(validation.issues),
            )
            return
        dataset_snapshot = self._snapshot_sensor_dataset(sensor, dataset)
        calibration_config = self._build_sensor_calibration_config(sensor, plugin, dataset_snapshot)
        plugin.status = plugin.load_status
        plugin.diagnostics = None
        plugin.calibration_warnings = []
        plugin.calibration_report = ""
        plugin.params_warnings = []
        plugin.calibration_progress = 0.0
        plugin.calibration_runtime_s = None
        plugin.is_calibrating = True
        getattr(self, f"{prefix}_calibration_jobs")[method_id] = {
            "started_at": time.monotonic(),
            "dataset_name": dataset_snapshot.summary()["name"],
        }
        self._refresh_sensor_methods_ui(sensor)
        worker = threading.Thread(
            target=self._run_sensor_calibration_worker,
            args=(sensor, method_id, plugin, dataset_snapshot, calibration_config),
            name=f"{sensor}-calibrate-{method_id}",
            daemon=True,
        )
        worker.start()

    def _run_sensor_calibration_worker(
        self,
        sensor: str,
        method_id: str,
        plugin: LoadedMethodPlugin,
        dataset: Any,
        calibration_config: dict[str, Any],
    ) -> None:
        result = run_method_calibration(plugin, dataset, config=calibration_config)
        queue_attr = "_acc_calibration_results" if sensor == "acc" else "_gyro_calibration_results"
        getattr(self, queue_attr).put((method_id, result))

    def _apply_sensor_calibration_result(self, sensor: str, method_id: str, result: CalibrationRunResult) -> None:
        self._ensure_sensor_method_state(sensor)
        self._ensure_sensor_calibration_state(sensor)
        prefix = "_acc" if sensor == "acc" else "_gyro"
        methods: dict[str, LoadedMethodPlugin] = getattr(self, f"{prefix}_methods")
        latest_derived = getattr(self, f"{prefix}_latest_derived_streams")
        offline_clouds = getattr(self, f"{prefix}_offline_method_clouds")
        plugin = methods.get(method_id)
        if plugin is None:
            getattr(self, f"{prefix}_calibration_jobs").pop(method_id, None)
            return
        job = getattr(self, f"{prefix}_calibration_jobs").pop(method_id, None)
        plugin.is_calibrating = False
        plugin.calibration_progress = 1.0 if result.ok else 0.0
        if job is not None:
            plugin.calibration_dataset_name = str(job.get("dataset_name") or "")
            plugin.calibration_runtime_s = max(0.0, time.monotonic() - float(job.get("started_at", time.monotonic())))
        if result.ok:
            plugin.status = plugin.load_status
            plugin.diagnostics = None
            plugin.calibration_params = result.params
            plugin.calibration_warnings = list(result.warnings)
            plugin.params_warnings = []
            plugin.process_warnings = []
            plugin.calibration_report = result.report
            self._recompute_sensor_method_dataset_cloud(sensor, method_id, open_diagnostics=False)
            if bool(getattr(self, f"{prefix}_recording_active", False)):
                self._update_sensor_dataset_recording_metadata(sensor)
        else:
            plugin.status = "error"
            plugin.calibration_warnings = list(result.warnings)
            plugin.calibration_report = result.report
            plugin.process_warnings = []
            plugin.realtime_enabled = False
            plugin.record_enabled = False
            plugin.last_output = None
            latest_derived.pop(method_id, None)
            for kind in self._sensor_method_output_kinds(sensor, plugin):
                self._calibration_stream_snapshots.pop(make_method_producer_id(plugin.file_path, kind), None)
            if result.diagnostics is not None:
                plugin.diagnostics = result.diagnostics
            else:
                plugin.diagnostics = self._make_magnetometer_method_diagnostics(
                    plugin,
                    last_action="calibrate",
                    error_text="Calibration failed.",
                    warnings=result.warnings,
                )
            offline_clouds.pop(method_id, None)
        self._refresh_sensor_methods_ui(sensor)
        self._refresh_sensor_method_clouds_ui(sensor)
        if plugin.status == "error":
            self._open_sensor_method_diagnostics(sensor, method_id)
        self._save_settings()

    def _poll_sensor_calibration_jobs(self, sensor: str) -> None:
        self._ensure_sensor_method_state(sensor)
        self._ensure_sensor_calibration_state(sensor)
        prefix = "_acc" if sensor == "acc" else "_gyro"
        methods: dict[str, LoadedMethodPlugin] = getattr(self, f"{prefix}_methods")
        refresh_needed = False
        now = time.monotonic()
        for method_id, job in list(getattr(self, f"{prefix}_calibration_jobs").items()):
            plugin = methods.get(method_id)
            if plugin is None or not plugin.is_calibrating:
                continue
            elapsed = max(0.0, now - float(job.get("started_at", now)))
            progress = min(0.92, 0.08 + elapsed / 2.5)
            if abs(progress - plugin.calibration_progress) >= 0.02:
                plugin.calibration_progress = progress
                refresh_needed = True
        result_queue = getattr(self, "_acc_calibration_results" if sensor == "acc" else "_gyro_calibration_results")
        while True:
            try:
                method_id, result = result_queue.get_nowait()
            except queue.Empty:
                break
            self._apply_sensor_calibration_result(sensor, method_id, result)
            refresh_needed = False
        if refresh_needed:
            self._refresh_sensor_methods_ui(sensor)

    def _on_load_sensor_method_params(self, sensor: str, method_id: str) -> None:
        self._ensure_sensor_method_state(sensor)
        methods: dict[str, LoadedMethodPlugin] = getattr(self, "_acc_methods" if sensor == "acc" else "_gyro_methods")
        plugin = methods.get(method_id)
        if plugin is None:
            return
        if not plugin.can_load_params():
            self._show_warning(f"{self._sensor_title(sensor)} Params", f"{plugin.name} does not support param loading right now.")
            return
        path = self._choose_sensor_params_load_path(sensor)
        if not path:
            return
        result = load_method_params(plugin, path)
        self._apply_sensor_param_io_result(sensor, method_id, result, action="load", path=path)

    def _on_save_sensor_method_params(self, sensor: str, method_id: str) -> None:
        self._ensure_sensor_method_state(sensor)
        methods: dict[str, LoadedMethodPlugin] = getattr(self, "_acc_methods" if sensor == "acc" else "_gyro_methods")
        plugin = methods.get(method_id)
        if plugin is None:
            return
        if not plugin.can_save_params():
            self._show_warning(f"{self._sensor_title(sensor)} Params", f"{plugin.name} has no runtime params available to save.")
            return
        path = self._choose_sensor_params_save_path(sensor)
        if not path:
            return
        result = save_method_params(plugin, path, plugin.calibration_params)
        self._apply_sensor_param_io_result(sensor, method_id, result, action="save", path=path)

    def _on_clear_sensor_method_params(self, sensor: str, method_id: str) -> None:
        self._ensure_sensor_method_state(sensor)
        prefix = "_acc" if sensor == "acc" else "_gyro"
        methods: dict[str, LoadedMethodPlugin] = getattr(self, f"{prefix}_methods")
        latest_derived = getattr(self, f"{prefix}_latest_derived_streams")
        offline_clouds = getattr(self, f"{prefix}_offline_method_clouds")
        plugin = methods.get(method_id)
        if plugin is None or not plugin.can_clear_params():
            return
        plugin.status = plugin.load_status
        plugin.diagnostics = None
        plugin.calibration_params = None
        plugin.calibration_report = ""
        plugin.calibration_warnings = []
        plugin.calibration_dataset_name = None
        plugin.calibration_runtime_s = None
        plugin.calibration_progress = 0.0
        plugin.params_warnings = []
        plugin.params_profile_path = None
        plugin.process_warnings = []
        plugin.realtime_enabled = False
        plugin.record_enabled = False
        plugin.last_output = None
        latest_derived.pop(method_id, None)
        offline_clouds.pop(method_id, None)
        for kind in self._sensor_method_output_kinds(sensor, plugin):
            self._calibration_stream_snapshots.pop(make_method_producer_id(plugin.file_path, kind), None)
        self._refresh_sensor_methods_ui(sensor)
        self._refresh_sensor_method_clouds_ui(sensor)
        self._refresh_sensor_output_routing_ui(sensor)
        if bool(getattr(self, f"{prefix}_recording_active", False)):
            self._update_sensor_dataset_recording_metadata(sensor)
        self._save_settings()

    def _on_sensor_add_plugin(self, sensor: str) -> None:
        dlg = AddPluginDialog(self.root, initial_dir=self._sensor_dialog_initialdir(sensor, "plugin"))
        self.root.wait_window(dlg)
        if not dlg.result:
            return
        self._remember_sensor_dialog_dir(sensor, "plugin", dlg.result["path"])
        plugin = load_method_plugin(dlg.result["path"])
        plugin = self._register_sensor_method(sensor, plugin)
        if plugin.status == "error":
            self._open_sensor_method_diagnostics(sensor, plugin.method_id)
        prefix = "_acc" if sensor == "acc" else "_gyro"
        if bool(getattr(self, f"{prefix}_recording_active", False)):
            self._update_sensor_dataset_recording_metadata(sensor)
        self._save_settings()

    def _on_select_sensor_method(self, sensor: str, method_id: str) -> None:
        self._ensure_sensor_method_state(sensor)
        prefix = "_acc" if sensor == "acc" else "_gyro"
        methods: dict[str, LoadedMethodPlugin] = getattr(self, f"{prefix}_methods")
        if method_id not in methods:
            return
        setattr(self, f"{prefix}_selected_method_id", method_id)
        self._refresh_sensor_methods_ui(sensor)
        if methods[method_id].status == "error":
            self._open_sensor_method_diagnostics(sensor, method_id)
        self._save_settings()

    def _on_remove_sensor_method(self, sensor: str, method_id: str) -> None:
        self._ensure_sensor_method_state(sensor)
        prefix = "_acc" if sensor == "acc" else "_gyro"
        methods: dict[str, LoadedMethodPlugin] = getattr(self, f"{prefix}_methods")
        plugin = methods.pop(method_id, None)
        if plugin is None:
            return
        getattr(self, f"{prefix}_latest_derived_streams").pop(method_id, None)
        getattr(self, f"{prefix}_offline_method_clouds").pop(method_id, None)
        for kind in self._sensor_method_output_kinds(sensor, plugin):
            self._calibration_stream_snapshots.pop(make_method_producer_id(plugin.file_path, kind), None)
        if getattr(self, f"{prefix}_selected_method_id") == method_id:
            setattr(self, f"{prefix}_selected_method_id", next(iter(methods), None))
        if getattr(self, f"{prefix}_primary_output_stream_id") == method_id:
            setattr(self, f"{prefix}_primary_output_stream_id", self._sensor_primary_raw_stream_id(sensor))
        self._refresh_sensor_methods_ui(sensor)
        self._refresh_sensor_method_clouds_ui(sensor)
        self._refresh_sensor_output_routing_ui(sensor)
        if bool(getattr(self, f"{prefix}_recording_active", False)):
            self._update_sensor_dataset_recording_metadata(sensor)
        self._save_settings()

    def _on_open_sensor_method_info(self, sensor: str, method_id: str) -> None:
        self._ensure_sensor_method_state(sensor)
        methods: dict[str, LoadedMethodPlugin] = getattr(self, "_acc_methods" if sensor == "acc" else "_gyro_methods")
        plugin = methods.get(method_id)
        if plugin is None:
            return
        dlg = MethodInfoDialog(
            self.root,
            title=f"Method Info — {plugin.name}",
            payload_provider=lambda sensor=sensor, method_id=method_id: self._build_sensor_method_info_payload(sensor, method_id),
            on_binding_change=lambda slot, producer_id, sensor=sensor, method_id=method_id: self._on_change_sensor_method_binding(
                sensor,
                method_id,
                slot,
                producer_id,
            ),
        )
        self.root.wait_window(dlg)

    def _open_sensor_method_diagnostics(self, sensor: str, method_id: str) -> None:
        self._ensure_sensor_method_state(sensor)
        methods: dict[str, LoadedMethodPlugin] = getattr(self, "_acc_methods" if sensor == "acc" else "_gyro_methods")
        plugin = methods.get(method_id)
        if plugin is None or plugin.diagnostics is None:
            return
        dlg = MethodDiagnosticsDialog(
            self.root,
            title=f"Method Diagnostics — {plugin.name}",
            diagnostics=plugin.diagnostics.as_dict(),
        )
        self.root.wait_window(dlg)

    def _on_clear_sensor_datasets(self, sensor: str) -> None:
        self._ensure_sensor_method_state(sensor)
        prefix = "_acc" if sensor == "acc" else "_gyro"
        if getattr(self, f"{prefix}_recording_active", False):
            self._show_warning(f"{self._sensor_title(sensor)} Dataset", "Stop recording before clearing datasets from memory.")
            return
        if not getattr(self, f"{prefix}_datasets") and getattr(self, f"{prefix}_dataset") is None:
            return
        setattr(self, f"{prefix}_datasets", [])
        setattr(self, f"{prefix}_dataset_export_path", "")
        self._set_active_sensor_dataset(sensor, None)
        self._save_settings()

    def _concatenate_selected_sensor_datasets(self, sensor: str, selected_indices: Sequence[int]) -> Any | None:
        self._ensure_sensor_method_state(sensor)
        prefix = "_acc" if sensor == "acc" else "_gyro"
        datasets = [
            getattr(self, f"{prefix}_datasets")[index]
            for index in selected_indices
            if 0 <= index < len(getattr(self, f"{prefix}_datasets"))
        ]
        if len(datasets) < 2:
            return None
        dataset_cls = AccelerometerDataset if sensor == "acc" else GyroscopeDataset
        merged = dataset_cls(self._sensor_make_concatenated_dataset_name(sensor))
        for dataset in datasets:
            merged.extend(list(dataset.records))
        getattr(self, f"{prefix}_datasets").append(merged)
        self._set_active_sensor_dataset(sensor, merged)
        return merged

    def _detach_shared_sensor_dataset(self, sensor: str) -> None:
        self._ensure_sensor_method_state(sensor)
        prefix = "_acc" if sensor == "acc" else "_gyro"
        dataset = getattr(self, f"{prefix}_dataset")
        if dataset is None or not self._is_shared_projection_dataset(dataset):
            return
        detached = self._snapshot_sensor_dataset(sensor, dataset)
        detached.source_path = None
        metadata = dict(getattr(dataset, "metadata", {}))
        metadata.pop("shared_source_path", None)
        metadata.pop("shared_dataset_name", None)
        metadata.pop("shared_dataset_format", None)
        detached.metadata = metadata
        getattr(self, f"{prefix}_datasets").append(detached)
        self._set_active_sensor_dataset(sensor, detached)

    def _on_sensor_concatenate(self, sensor: str) -> None:
        prefix = "_acc" if sensor == "acc" else "_gyro"
        datasets = getattr(self, f"{prefix}_datasets")
        if getattr(self, f"{prefix}_recording_active", False):
            return
        if len(datasets) < 2:
            self._show_info(f"{self._sensor_title(sensor)} Dataset", "Load at least two datasets before concatenation.")
            return
        items = [self._sensor_dataset_dialog_label(dataset) for dataset in datasets]
        dlg = DatasetSelectionDialog(
            self.root,
            title="Concatenate Datasets",
            prompt="Select datasets to concatenate:",
            items=items,
            preselected=list(range(len(items))),
        )
        self.root.wait_window(dlg)
        if dlg.result is None:
            return
        if len(dlg.result) < 2:
            self._show_info(f"{self._sensor_title(sensor)} Dataset", "Select at least two datasets to concatenate.")
            return
        self._concatenate_selected_sensor_datasets(sensor, dlg.result)

    def _trim_active_sensor_dataset(self, sensor: str, selected_indices: Sequence[int]) -> bool:
        self._ensure_sensor_method_state(sensor)
        prefix = "_acc" if sensor == "acc" else "_gyro"
        dataset = getattr(self, f"{prefix}_dataset")
        if dataset is None or not selected_indices:
            return False
        self._detach_shared_sensor_dataset(sensor)
        dataset = getattr(self, f"{prefix}_dataset")
        if dataset is None:
            return False
        dataset.trim(min(selected_indices), max(selected_indices))
        self._rebuild_sensor_dataset_table(sensor)
        self._refresh_sensor_dataset_ui(sensor)
        return True

    def _delete_active_sensor_rows(self, sensor: str, selected_indices: Sequence[int]) -> bool:
        self._ensure_sensor_method_state(sensor)
        prefix = "_acc" if sensor == "acc" else "_gyro"
        dataset = getattr(self, f"{prefix}_dataset")
        if dataset is None or not selected_indices:
            return False
        self._detach_shared_sensor_dataset(sensor)
        dataset = getattr(self, f"{prefix}_dataset")
        if dataset is None:
            return False
        dataset.delete_rows(list(selected_indices))
        self._rebuild_sensor_dataset_table(sensor)
        self._refresh_sensor_dataset_ui(sensor)
        return True

    def _on_sensor_trim_selection(self, sensor: str) -> None:
        prefix = "_acc" if sensor == "acc" else "_gyro"
        if getattr(self, f"{prefix}_recording_active", False):
            return
        if getattr(self, f"{prefix}_dataset") is None:
            self._show_info(f"{self._sensor_title(sensor)} Dataset", "There is no active dataset.")
            return
        selected_indices = self._sensor_tab(sensor).get_selected_dataset_row_indices()
        if not selected_indices:
            self._show_info(f"{self._sensor_title(sensor)} Dataset", "Select one or more rows in the Data table.")
            return
        self._trim_active_sensor_dataset(sensor, selected_indices)

    def _on_sensor_delete_selection(self, sensor: str) -> None:
        prefix = "_acc" if sensor == "acc" else "_gyro"
        if getattr(self, f"{prefix}_recording_active", False):
            return
        if getattr(self, f"{prefix}_dataset") is None:
            self._show_info(f"{self._sensor_title(sensor)} Dataset", "There is no active dataset.")
            return
        selected_indices = self._sensor_tab(sensor).get_selected_dataset_row_indices()
        if not selected_indices:
            self._show_info(f"{self._sensor_title(sensor)} Dataset", "Select one or more rows in the Data table.")
            return
        self._delete_active_sensor_rows(sensor, selected_indices)

    def _restore_sensor_settings(self, sensor: str) -> None:
        self._ensure_sensor_source_state(sensor)
        self._ensure_sensor_method_state(sensor)
        tab = self._sensor_tab(sensor)
        if tab is None:
            return
        prefix = "_acc" if sensor == "acc" else "_gyro"
        saved_state = getattr(self, f"{prefix}_saved_state", {})
        if not isinstance(saved_state, dict) or not saved_state:
            self._refresh_sensor_method_clouds_ui(sensor)
            return
        sources = getattr(self, f"{prefix}_sources")
        saved_sources = saved_state.get("sources", {})
        if isinstance(saved_sources, dict):
            for source_id, source_state in saved_sources.items():
                if source_id not in sources or not isinstance(source_state, dict):
                    continue
                sources[source_id]["show"] = bool(source_state.get("show", sources[source_id]["show"]))
                sources[source_id]["record"] = bool(source_state.get("record", sources[source_id]["record"]))
        saved_selected_source = str(saved_state.get("selected_source_id", "")).strip()
        if saved_selected_source in sources:
            setattr(self, f"{prefix}_selected_source_id", saved_selected_source)
        dataset_paths = [
            str(path).strip()
            for path in saved_state.get("datasets", [])
            if isinstance(path, str) and os.path.isfile(path)
        ]
        if dataset_paths:
            self._load_sensor_dataset_paths(sensor, dataset_paths, show_errors=False)
        active_dataset_path = str(saved_state.get("active_dataset_path", "")).strip()
        if active_dataset_path:
            for dataset in getattr(self, f"{prefix}_datasets"):
                if dataset.source_path == active_dataset_path:
                    self._set_active_sensor_dataset(sensor, dataset)
                    break
        plugins_by_path: dict[str, LoadedMethodPlugin] = {}
        for entry in saved_state.get("methods", []):
            if not isinstance(entry, dict):
                continue
            file_path = str(entry.get("file_path", "")).strip()
            if not file_path or not os.path.isfile(file_path):
                continue
            self._remember_sensor_dialog_dir(sensor, "plugin", file_path)
            plugin = self._register_sensor_method(sensor, load_method_plugin(file_path))
            plugins_by_path[plugin.file_path] = plugin
            plugin.show_enabled = bool(entry.get("show", False))
            saved_bindings = entry.get("bindings", {})
            if isinstance(saved_bindings, dict):
                plugin.stream_bindings[SCOPE_CALIBRATE] = {
                    str(slot): str(producer_id)
                    for slot, producer_id in saved_bindings.get(SCOPE_CALIBRATE, {}).items()
                    if str(slot).strip() and str(producer_id).strip()
                }
                plugin.stream_bindings[SCOPE_PROCESS] = {
                    str(slot): str(producer_id)
                    for slot, producer_id in saved_bindings.get(SCOPE_PROCESS, {}).items()
                    if str(slot).strip() and str(producer_id).strip()
                }
            params_profile_path = str(entry.get("params_profile_path", "")).strip()
            if params_profile_path and os.path.isfile(params_profile_path) and plugin.can_load_params():
                result = load_method_params(plugin, params_profile_path)
                self._apply_sensor_param_io_result(
                    sensor,
                    plugin.method_id,
                    result,
                    action="load",
                    path=params_profile_path,
                    show_warnings=False,
                    open_diagnostics=False,
                )
            if bool(entry.get("realtime_enabled", False)) and plugin.can_enable_realtime():
                plugin.realtime_enabled = True
            default_record_state = bool(entry.get("realtime_enabled", False))
            plugin.record_enabled = bool(entry.get("record", default_record_state) and plugin.realtime_enabled)
        selected_method_path = str(saved_state.get("selected_method_path", "")).strip()
        selected_plugin = plugins_by_path.get(selected_method_path)
        if selected_plugin is not None:
            setattr(self, f"{prefix}_selected_method_id", selected_plugin.method_id)
        primary_output = saved_state.get("primary_output", {})
        if isinstance(primary_output, dict) and primary_output.get("kind") == "method":
            file_path = str(primary_output.get("file_path", "")).strip()
            plugin = plugins_by_path.get(file_path)
            if plugin is not None:
                setattr(self, f"{prefix}_primary_output_stream_id", plugin.method_id)
        view_state = saved_state.get("view_state", {})
        if isinstance(view_state, dict):
            tab.apply_view_state(view_state)
        self._refresh_sensor_sources_ui(sensor)
        self._refresh_sensor_methods_ui(sensor)
        self._refresh_all_sensor_method_dataset_clouds(sensor)
        self._refresh_sensor_output_routing_ui(sensor)

    def _record_accelerometer_sample(
        self,
        *,
        timestamp_mcu: int,
        timestamp_pc_rx: int,
        timestamp_pc_est: int | None,
        ax: float,
        ay: float,
        az: float,
        raw_roll_deg: float | None,
        raw_pitch_deg: float | None,
        derived_records: Sequence[AccelerometerSampleRecord] | None = None,
    ) -> None:
        self._ensure_sensor_source_state("acc")
        if not self._acc_recording_active or self._acc_dataset is None:
            return
        next_row_id = len(self._acc_dataset.records)
        appended = False
        shared_recording_active = bool(getattr(self, "_imu_recording_active", False))
        record_raw_accelerometer_local = bool(self._acc_sources["raw_accelerometer"]["record"])
        record_raw_tilt_local = bool(self._acc_sources["raw_tilt"]["record"])
        if record_raw_accelerometer_local or shared_recording_active:
            record = AccelerometerSampleRecord(
                stream_id="raw_accelerometer",
                stream_type="raw",
                producer_name="Raw Accelerometer",
                producer_version="builtin",
                timestamp_mcu=timestamp_mcu,
                timestamp_pc_rx=timestamp_pc_rx,
                timestamp_pc_est=timestamp_pc_est,
                acc_x=ax,
                acc_y=ay,
                acc_z=az,
                roll_deg=raw_roll_deg,
                pitch_deg=raw_pitch_deg,
                flags="",
            )
            self._append_imu_shared_record(self._convert_accelerometer_record_to_imu(record))
            if record_raw_accelerometer_local:
                next_row_id += 1
                self._acc_dataset.append(record)
            if record_raw_accelerometer_local and self.accelerometer_tab is not None:
                self.accelerometer_tab.append_dataset_record(next_row_id, record)
                appended = True
        if record_raw_tilt_local or shared_recording_active:
            record = AccelerometerSampleRecord(
                stream_id="raw_tilt",
                stream_type="raw",
                producer_name="Raw Tilt",
                producer_version="builtin",
                timestamp_mcu=timestamp_mcu,
                timestamp_pc_rx=timestamp_pc_rx,
                timestamp_pc_est=timestamp_pc_est,
                acc_x=ax,
                acc_y=ay,
                acc_z=az,
                roll_deg=raw_roll_deg,
                pitch_deg=raw_pitch_deg,
                flags="tilt_only",
            )
            self._append_imu_shared_record(self._convert_accelerometer_record_to_imu(record))
            if record_raw_tilt_local:
                next_row_id += 1
                self._acc_dataset.append(record)
            if record_raw_tilt_local and self.accelerometer_tab is not None:
                self.accelerometer_tab.append_dataset_record(next_row_id, record)
                appended = True
        for record in derived_records or ():
            next_row_id += 1
            self._acc_dataset.append(record)
            self._append_imu_shared_record(self._convert_accelerometer_record_to_imu(record))
            if self.accelerometer_tab is not None:
                self.accelerometer_tab.append_dataset_record(next_row_id, record)
            appended = True
        if appended:
            self._refresh_sensor_dataset_ui_if_due("acc", force=not self._acc_recording_active)

    def _record_gyroscope_sample(
        self,
        *,
        timestamp_mcu: int,
        timestamp_pc_rx: int,
        timestamp_pc_est: int | None,
        gx: float,
        gy: float,
        gz: float,
        derived_records: Sequence[GyroscopeSampleRecord] | None = None,
    ) -> None:
        self._ensure_sensor_source_state("gyro")
        if not self._gyro_recording_active or self._gyro_dataset is None:
            return
        next_row_id = len(self._gyro_dataset.records)
        appended = False
        if bool(self._gyro_sources["raw_gyroscope"]["record"]) or bool(getattr(self, "_imu_recording_active", False)):
            next_row_id += 1
            record = GyroscopeSampleRecord(
                stream_id="raw_gyroscope",
                stream_type="raw",
                producer_name="Raw Gyroscope",
                producer_version="builtin",
                timestamp_mcu=timestamp_mcu,
                timestamp_pc_rx=timestamp_pc_rx,
                timestamp_pc_est=timestamp_pc_est,
                gyro_x=gx,
                gyro_y=gy,
                gyro_z=gz,
                rate_mag=compute_gyro_rate_magnitude(gx, gy, gz),
                flags="",
            )
            self._gyro_dataset.append(record)
            self._append_imu_shared_record(self._convert_gyroscope_record_to_imu(record))
            if self.gyroscope_tab is not None:
                self.gyroscope_tab.append_dataset_record(next_row_id, record)
            appended = True
        for record in derived_records or ():
            next_row_id += 1
            self._gyro_dataset.append(record)
            self._append_imu_shared_record(self._convert_gyroscope_record_to_imu(record))
            if self.gyroscope_tab is not None:
                self.gyroscope_tab.append_dataset_record(next_row_id, record)
            appended = True
        if appended:
            self._refresh_sensor_dataset_ui_if_due("gyro", force=not self._gyro_recording_active)

    def _update_accelerometer_live_view(self, imu: ImuData, pc_rx_ms: int) -> None:
        accelerometer_tab = getattr(self, "accelerometer_tab", None)
        if accelerometer_tab is None:
            return
        timestamp_pc_est = self._estimate_pc_event_ms_from_mcu(imu.ts_ms)
        ax = float(imu.accel[0])
        ay = float(imu.accel[1])
        az = float(imu.accel[2])
        raw_roll_deg, raw_pitch_deg = compute_accelerometer_tilt_deg(ax, ay, az)
        raw_sample = {
            "stream_id": "raw_accelerometer",
            "stream_type": "raw",
            "producer_name": "Raw Accelerometer",
            "producer_version": "builtin",
            "timestamp_mcu": imu.ts_ms,
            "timestamp_pc_rx": pc_rx_ms,
            "timestamp_pc_est": timestamp_pc_est,
            "acc_x": ax,
            "acc_y": ay,
            "acc_z": az,
            "roll_deg": raw_roll_deg,
            "pitch_deg": raw_pitch_deg,
            "flags": "",
        }
        self._publish_calibration_stream_snapshot(make_builtin_producer_id("raw_accelerometer"), "imu.accel_vector", raw_sample)
        self._publish_calibration_stream_snapshot(
            make_builtin_producer_id("raw_tilt"),
            "imu.accel_vector",
            {
                **raw_sample,
                "stream_id": "raw_tilt",
                "flags": "tilt_only",
            },
        )
        derived_streams, derived_records = self._process_sensor_methods_realtime("acc", raw_sample)
        self._record_accelerometer_sample(
            timestamp_mcu=imu.ts_ms,
            timestamp_pc_rx=pc_rx_ms,
            timestamp_pc_est=timestamp_pc_est,
            ax=ax,
            ay=ay,
            az=az,
            raw_roll_deg=raw_roll_deg,
            raw_pitch_deg=raw_pitch_deg,
            derived_records=derived_records,
        )
        self._acc_last_live_snapshot = {
            "timestamp_mcu": imu.ts_ms,
            "timestamp_pc_rx": pc_rx_ms,
            "timestamp_pc_est": timestamp_pc_est,
            "ax": ax,
            "ay": ay,
            "az": az,
            "raw_roll_deg": raw_roll_deg,
            "raw_pitch_deg": raw_pitch_deg,
            "derived_streams": dict(derived_streams),
        }
        self._refresh_sensor_output_routing_ui("acc")
        selected_roll_deg, selected_pitch_deg, selected_source_label = self._resolve_accelerometer_selected_output(
            raw_roll_deg,
            raw_pitch_deg,
            derived_streams,
        )
        accelerometer_tab.update_live_imu(
            timestamp_mcu=imu.ts_ms,
            timestamp_pc_rx=pc_rx_ms,
            timestamp_pc_est=timestamp_pc_est,
            ax=ax,
            ay=ay,
            az=az,
            selected_roll_deg=selected_roll_deg,
            selected_pitch_deg=selected_pitch_deg,
            selected_source_label=selected_source_label,
            derived_streams=derived_streams,
        )

    def _update_gyroscope_live_view(self, imu: ImuData, pc_rx_ms: int) -> None:
        gyroscope_tab = getattr(self, "gyroscope_tab", None)
        if gyroscope_tab is None:
            return
        timestamp_pc_est = self._estimate_pc_event_ms_from_mcu(imu.ts_ms)
        gx = float(imu.gyro[0])
        gy = float(imu.gyro[1])
        gz = float(imu.gyro[2])
        raw_sample = {
            "stream_id": "raw_gyroscope",
            "stream_type": "raw",
            "producer_name": "Raw Gyroscope",
            "producer_version": "builtin",
            "timestamp_mcu": imu.ts_ms,
            "timestamp_pc_rx": pc_rx_ms,
            "timestamp_pc_est": timestamp_pc_est,
            "gyro_x": gx,
            "gyro_y": gy,
            "gyro_z": gz,
            "rate_mag": compute_gyro_rate_magnitude(gx, gy, gz),
            "flags": "",
        }
        self._publish_calibration_stream_snapshot(make_builtin_producer_id("raw_gyroscope"), "imu.gyro_vector", raw_sample)
        derived_streams, derived_records = self._process_sensor_methods_realtime("gyro", raw_sample)
        self._record_gyroscope_sample(
            timestamp_mcu=imu.ts_ms,
            timestamp_pc_rx=pc_rx_ms,
            timestamp_pc_est=timestamp_pc_est,
            gx=gx,
            gy=gy,
            gz=gz,
            derived_records=derived_records,
        )
        self._gyro_last_live_snapshot = {
            "timestamp_mcu": imu.ts_ms,
            "timestamp_pc_rx": pc_rx_ms,
            "timestamp_pc_est": timestamp_pc_est,
            "gx": gx,
            "gy": gy,
            "gz": gz,
            "derived_streams": dict(derived_streams),
        }
        self._refresh_sensor_output_routing_ui("gyro")
        gyroscope_tab.update_live_imu(
            timestamp_mcu=imu.ts_ms,
            timestamp_pc_rx=pc_rx_ms,
            timestamp_pc_est=timestamp_pc_est,
            gx=gx,
            gy=gy,
            gz=gz,
            selected_source_label=self._resolve_gyroscope_selected_output(derived_streams),
            derived_streams=derived_streams,
        )

    def _restore_accelerometer_settings(self) -> None:
        self._restore_sensor_settings("acc")

    def _restore_gyroscope_settings(self) -> None:
        self._restore_sensor_settings("gyro")

    def _refresh_accelerometer_sources_ui(self) -> None:
        self._refresh_sensor_sources_ui("acc")

    def _refresh_gyroscope_sources_ui(self) -> None:
        self._refresh_sensor_sources_ui("gyro")

    def _refresh_accelerometer_methods_ui(self) -> None:
        self._refresh_sensor_methods_ui("acc")

    def _refresh_gyroscope_methods_ui(self) -> None:
        self._refresh_sensor_methods_ui("gyro")

    def _refresh_accelerometer_output_routing_ui(self) -> None:
        self._refresh_sensor_output_routing_ui("acc")

    def _refresh_gyroscope_output_routing_ui(self) -> None:
        self._refresh_sensor_output_routing_ui("gyro")

    def _refresh_accelerometer_dataset_ui(self) -> None:
        self._refresh_sensor_dataset_ui("acc")

    def _refresh_gyroscope_dataset_ui(self) -> None:
        self._refresh_sensor_dataset_ui("gyro")

    def _on_accelerometer_start_record(self) -> None:
        self._on_sensor_start_record("acc")

    def _on_gyroscope_start_record(self) -> None:
        self._on_sensor_start_record("gyro")

    def _on_accelerometer_stop_record(self) -> None:
        self._on_sensor_stop_record("acc")

    def _on_gyroscope_stop_record(self) -> None:
        self._on_sensor_stop_record("gyro")

    def _on_accelerometer_load_csv(self) -> None:
        self._on_sensor_load_csv("acc")

    def _on_gyroscope_load_csv(self) -> None:
        self._on_sensor_load_csv("gyro")

    def _on_accelerometer_load_multiple(self) -> None:
        self._on_sensor_load_multiple("acc")

    def _on_gyroscope_load_multiple(self) -> None:
        self._on_sensor_load_multiple("gyro")

    def _on_accelerometer_save_current(self) -> None:
        path = self._acc_dataset_export_path
        if not path and self._acc_dataset is not None and self._acc_dataset.source_path:
            path = self._acc_dataset.source_path
        if not path:
            path = self._choose_sensor_dataset_path("acc")
        if path:
            self._save_sensor_dataset_to_path("acc", path)

    def _on_gyroscope_save_current(self) -> None:
        path = self._gyro_dataset_export_path
        if not path and self._gyro_dataset is not None and self._gyro_dataset.source_path:
            path = self._gyro_dataset.source_path
        if not path:
            path = self._choose_sensor_dataset_path("gyro")
        if path:
            self._save_sensor_dataset_to_path("gyro", path)

    def _on_accelerometer_save_as(self) -> None:
        path = self._choose_sensor_dataset_path("acc")
        if path:
            self._save_sensor_dataset_to_path("acc", path)

    def _on_gyroscope_save_as(self) -> None:
        path = self._choose_sensor_dataset_path("gyro")
        if path:
            self._save_sensor_dataset_to_path("gyro", path)

    def _on_clear_accelerometer_datasets(self) -> None:
        self._on_clear_sensor_datasets("acc")

    def _on_clear_gyroscope_datasets(self) -> None:
        self._on_clear_sensor_datasets("gyro")

    def _on_accelerometer_concatenate(self) -> None:
        self._on_sensor_concatenate("acc")

    def _on_gyroscope_concatenate(self) -> None:
        self._on_sensor_concatenate("gyro")

    def _on_accelerometer_trim_selection(self) -> None:
        self._on_sensor_trim_selection("acc")

    def _on_gyroscope_trim_selection(self) -> None:
        self._on_sensor_trim_selection("gyro")

    def _on_accelerometer_delete_selection(self) -> None:
        self._on_sensor_delete_selection("acc")

    def _on_gyroscope_delete_selection(self) -> None:
        self._on_sensor_delete_selection("gyro")

    def _on_select_accelerometer_dataset(self, index: int) -> None:
        self._on_select_sensor_dataset("acc", index)

    def _on_select_gyroscope_dataset(self, index: int) -> None:
        self._on_select_sensor_dataset("gyro", index)

    def _on_select_accelerometer_source(self, source_id: str) -> None:
        self._on_select_sensor_source("acc", source_id)

    def _on_select_gyroscope_source(self, source_id: str) -> None:
        self._on_select_sensor_source("gyro", source_id)

    def _on_toggle_accelerometer_source_show(self, source_id: str, enabled: bool) -> None:
        self._on_toggle_sensor_source_show("acc", source_id, enabled)

    def _on_toggle_gyroscope_source_show(self, source_id: str, enabled: bool) -> None:
        self._on_toggle_sensor_source_show("gyro", source_id, enabled)

    def _on_toggle_accelerometer_source_record(self, source_id: str, enabled: bool) -> None:
        self._on_toggle_sensor_source_record("acc", source_id, enabled)

    def _on_toggle_gyroscope_source_record(self, source_id: str, enabled: bool) -> None:
        self._on_toggle_sensor_source_record("gyro", source_id, enabled)

    def _on_accelerometer_add_plugin(self) -> None:
        self._on_sensor_add_plugin("acc")

    def _on_gyroscope_add_plugin(self) -> None:
        self._on_sensor_add_plugin("gyro")

    def _on_select_accelerometer_method(self, method_id: str) -> None:
        self._on_select_sensor_method("acc", method_id)

    def _on_select_gyroscope_method(self, method_id: str) -> None:
        self._on_select_sensor_method("gyro", method_id)

    def _on_open_accelerometer_method_info(self, method_id: str) -> None:
        self._on_open_sensor_method_info("acc", method_id)

    def _on_open_gyroscope_method_info(self, method_id: str) -> None:
        self._on_open_sensor_method_info("gyro", method_id)

    def _on_calibrate_accelerometer_method(self, method_id: str) -> None:
        self._on_calibrate_sensor_method("acc", method_id)

    def _on_calibrate_gyroscope_method(self, method_id: str) -> None:
        self._on_calibrate_sensor_method("gyro", method_id)

    def _on_load_accelerometer_method_params(self, method_id: str) -> None:
        self._on_load_sensor_method_params("acc", method_id)

    def _on_load_gyroscope_method_params(self, method_id: str) -> None:
        self._on_load_sensor_method_params("gyro", method_id)

    def _on_save_accelerometer_method_params(self, method_id: str) -> None:
        self._on_save_sensor_method_params("acc", method_id)

    def _on_save_gyroscope_method_params(self, method_id: str) -> None:
        self._on_save_sensor_method_params("gyro", method_id)

    def _on_clear_accelerometer_method_params(self, method_id: str) -> None:
        self._on_clear_sensor_method_params("acc", method_id)

    def _on_clear_gyroscope_method_params(self, method_id: str) -> None:
        self._on_clear_sensor_method_params("gyro", method_id)

    def _on_toggle_accelerometer_method_show(self, method_id: str, enabled: bool) -> None:
        self._on_toggle_sensor_method_show("acc", method_id, enabled)

    def _on_toggle_gyroscope_method_show(self, method_id: str, enabled: bool) -> None:
        self._on_toggle_sensor_method_show("gyro", method_id, enabled)

    def _on_remove_accelerometer_method(self, method_id: str) -> None:
        self._on_remove_sensor_method("acc", method_id)

    def _on_remove_gyroscope_method(self, method_id: str) -> None:
        self._on_remove_sensor_method("gyro", method_id)

    def _on_enable_accelerometer_method_realtime(self, method_id: str) -> None:
        self._on_enable_sensor_method_realtime("acc", method_id)

    def _on_enable_gyroscope_method_realtime(self, method_id: str) -> None:
        self._on_enable_sensor_method_realtime("gyro", method_id)

    def _on_disable_accelerometer_method_realtime(self, method_id: str) -> None:
        self._on_disable_sensor_method_realtime("acc", method_id)

    def _on_disable_gyroscope_method_realtime(self, method_id: str) -> None:
        self._on_disable_sensor_method_realtime("gyro", method_id)

    def _on_toggle_accelerometer_method_record(self, method_id: str, enabled: bool) -> None:
        self._on_toggle_sensor_method_record("acc", method_id, enabled)

    def _on_toggle_gyroscope_method_record(self, method_id: str, enabled: bool) -> None:
        self._on_toggle_sensor_method_record("gyro", method_id, enabled)

    def _on_select_accelerometer_primary_output(self, stream_id: str) -> None:
        self._on_select_sensor_primary_output("acc", stream_id)

    def _on_select_gyroscope_primary_output(self, stream_id: str) -> None:
        self._on_select_sensor_primary_output("gyro", stream_id)

    def _update_mcu_time_label(self) -> None:
        rx = "—" if self._last_mcu_ts_u32 is None else str(self._last_mcu_ts_u32)
        if self.time_model.have_lock:
            now_ms = now_ms_monotonic()
            est = str(u32(int(round(self.time_model.mcu_from_pc(now_ms)))))
        else:
            est = "—"
        self.mcu_time_var.set(f"MCU Time:\nrx={rx}\nest={est}")

    def _track_speed_from_rpm(self, rpm: int) -> float:
        return (float(rpm) / 60.0) * self.geom_cfg.track_circumference_m

    def _update_track_speed_labels(self, left_rpm: int, right_rpm: int) -> None:
        left_speed = self._track_speed_from_rpm(left_rpm)
        right_speed = self._track_speed_from_rpm(right_rpm)
        self.speed_l.set(f"{left_speed:.2f} m/s")
        self.speed_r.set(f"{right_speed:.2f} m/s")

    def _update_motor(self, m: MotorData) -> None:
        # Update numbers
        self.volt_l.set(str(m.voltage_l))
        self.volt_r.set(str(m.voltage_r))
        self.cur_l.set(str(m.current_l))
        self.cur_r.set(str(m.current_r))
        self.temp_l.set(str(m.temp_l))
        self.temp_r.set(str(m.temp_r))

        # Colorize
        self._set_param_color("Voltage", float(m.voltage_l), float(m.voltage_r),
                              self.dev_cfg.voltage_normal, self.dev_cfg.voltage_delta)
        self._set_param_color("Current", float(m.current_l), float(m.current_r),
                              self.dev_cfg.current_normal, self.dev_cfg.current_delta)
        self._set_param_color("Temperature", float(m.temp_l), float(m.temp_r),
                              self.dev_cfg.temp_normal, self.dev_cfg.temp_delta)

        self._update_mcu_time_label()

    def _set_param_color(self, name: str, left_val: float, right_val: float, normal: float, delta: float) -> None:
        """
        PDF coloring rule:
        green if within normal +- 0.5*delta
        yellow if within (0.5*delta .. 1.0*delta)
        red if > delta
        """
        half = 0.5 * delta

        def pick(v: float) -> str:
            d = abs(v - normal)
            if d <= half:
                return COLOR_GREEN
            if d <= delta:
                return COLOR_YELLOW
            return COLOR_RED

        lbl_l, lbl_r = self._value_labels[name]

        # ttk.Label doesn't reliably support fg changes cross-platform => replace with tk.Label lazily
        def ensure_tk(lbl: ttk.Label) -> tk.Label:
            if isinstance(lbl, tk.Label):
                return lbl
            # replace
            info = lbl.grid_info()
            parent = lbl.master
            var = lbl.cget("textvariable")
            font = lbl.cget("font")
            lbl.destroy()
            # textvariable expects a Tcl variable name string.
            new_lbl = tk.Label(parent, textvariable=var if var else None, bg=PANEL_BG, font=font)
            new_lbl.grid(**info)
            return new_lbl

        tk_l = ensure_tk(lbl_l)
        tk_r = ensure_tk(lbl_r)

        tk_l.configure(fg=pick(left_val))
        tk_r.configure(fg=pick(right_val))

        # Update dict references if replaced
        self._value_labels[name] = (tk_l, tk_r)

    def _recolor_all(self) -> None:
        try:
            vl = float(self.volt_l.get()) if self.volt_l.get() != "—" else None
            vr = float(self.volt_r.get()) if self.volt_r.get() != "—" else None
            cl = float(self.cur_l.get()) if self.cur_l.get() != "—" else None
            cr = float(self.cur_r.get()) if self.cur_r.get() != "—" else None
            tl = float(self.temp_l.get()) if self.temp_l.get() != "—" else None
            tr = float(self.temp_r.get()) if self.temp_r.get() != "—" else None
        except ValueError:
            return

        if vl is not None and vr is not None:
            self._set_param_color("Voltage", vl, vr, self.dev_cfg.voltage_normal, self.dev_cfg.voltage_delta)
        if cl is not None and cr is not None:
            self._set_param_color("Current", cl, cr, self.dev_cfg.current_normal, self.dev_cfg.current_delta)
        if tl is not None and tr is not None:
            self._set_param_color("Temperature", tl, tr, self.dev_cfg.temp_normal, self.dev_cfg.temp_delta)

    def _msg_to_dict(self, msg: Any) -> dict:
        # Keep logger stable and explicit
        if isinstance(msg, MotorData):
            return {"type": "D2", **asdict(msg)}
        if isinstance(msg, SensorTensorData):
            return {"type": "D3", **asdict(msg)}
        if isinstance(msg, ImuData):
            return {"type": "D0", **asdict(msg)}
        if isinstance(msg, TachoData):
            return {"type": "D1", **asdict(msg)}
        if isinstance(msg, SyncResp):
            return {"type": "F0", "t2_rx_ms": msg.t2_rx_ms, "t3_tx_ms": msg.t3_tx_ms}
        if isinstance(msg, MotorPidData):
            return {"type": "F1", **asdict(msg)}
        if isinstance(msg, Frame):
            return {"type": "unknown", "msg_type": msg.msg_type, "len": len(msg.payload)}
        return {"type": "unknown", "repr": repr(msg)}
