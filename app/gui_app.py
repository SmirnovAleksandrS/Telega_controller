"""
Main GUI application:
- Menus: Files (start/stop log), Settings (COM settings)
- Tabs: Manual (implemented), Coordinate (placeholder)
- Right side panel with critical params, MCU time, radio quality, button deviation settings
- Serial worker + protocol handling + time sync
"""

from __future__ import annotations
import math
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from dataclasses import asdict
import struct
from typing import Optional, Any
import collections
import json
import os

from app.manual_tab import ManualTab
from app.coordinate_tab import CoordinateTab
from app.dialogs import (
    ComSettingsDialog,
    DeviationSettingsDialog,
    DeviationConfig,
    GeometrySettingsDialog,
    GeometryConfig,
    SpeedMapDialog,
    SpeedMapConfig,
)
from app.styles import PANEL_BG, STATUS_GREEN, STATUS_RED, COLOR_GREEN, COLOR_YELLOW, COLOR_RED

from comm.serial_worker import SerialWorker, SerialMetrics, RxEvent, RxError
from comm.protocol import (
    build_sync_req, build_control,
    TYPE_D0_IMU, TYPE_D1_TACHO,
    TYPE_B0_SYNC_REQ, TYPE_C0_CONTROL, TYPE_A0_DISABLE_D, TYPE_A1_ENABLE_D,
    SOF, Frame, parse_frame,
    ImuData, TachoData, MotorData, SensorTensorData, SyncResp
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
    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.title("Virtual controller")
        self.root.configure(bg=PANEL_BG)
        self.root.minsize(980, 560)
        self._is_shutting_down = False

        self.worker = SerialWorker()
        self.worker.on_send = self._log_tx_raw
        self.logger = ParsedLogger()

        self.dev_cfg = DeviationConfig()
        self.geom_cfg = GeometryConfig()
        self.speed_cfg = SpeedMapConfig()
        self._test_mode_enabled = False
        self._manual_cfg = {"command_quantum_ms": 100}
        self._settings_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "app_settings.json"))
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
        )
        self.nb.add(self.manual_tab, text="Manual")
        self._apply_joystick_settings()
        self._apply_manual_settings()

        self.coord_tab = CoordinateTab(
            self.nb,
            on_start=self._on_coord_start,
            on_stop=self._on_coord_stop,
            on_state_change=self._save_settings,
        )
        self.nb.add(self.coord_tab, text="Coordinate")
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

    # ---------------- Menu actions ----------------

    def _choose_log_path(self) -> None:
        path = filedialog.asksaveasfilename(
            title="Select log file",
            defaultextension=".log",
            filetypes=[("Log files", "*.log"), ("All files", "*.*")]
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
            messagebox.showerror("Logging", f"Failed to start log: {e}")
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
        messagebox.showerror("Coordinate", f"External runtime failed:\n{exc}")

    def _toggle_connect(self) -> None:
        if self.worker.is_open:
            self.worker.close()
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
                messagebox.showerror(
                    "COM Settings",
                    f"Saved port '{self.worker.port}' is not available.\nSelect a port in COM Settings.",
                )
                self._open_com_settings()
                self._update_uart_status(force=True)
                return
            self.worker.open(self.worker.port, self.worker.baud)
            if self.worker.is_open:
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
        messagebox.showerror(title, error)

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
            messagebox.showerror("Coordinate", "Connect COM port before start")
            return
        if self._kill_active:
            messagebox.showerror("Coordinate", "Kill switch is active")
            return
        if self._coord_running:
            return
        self.coord_tab.refresh_profile()
        mission = self.coord_tab.build_mission_config()
        if len(mission.track_points) < 2:
            messagebox.showerror("Coordinate", "No valid trajectory for controller")
            return
        if self._mission_uses_external_runtime(mission) and self._external_runtime is None:
            messagebox.showerror("Coordinate", "External runtime is selected, but no external runtime is attached")
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
                messagebox.showerror("Coordinate", f"Failed to initialize external runtime:\n{exc}")
                return
        if mission.autopilot == AutopilotMode.EXTERNAL:
            self.coord_tab.reset_expected_pose()
        else:
            if not self.coord_tab.start_controller():
                messagebox.showerror("Coordinate", "No valid trajectory for controller")
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

    def _kick_sync_if_due(self, now_ms: int) -> None:
        if not self.worker.is_open:
            return
        if now_ms < self._next_sync_due_ms:
            return

        if self._initial_sync_in_progress:
            # fire rounds back-to-back (small spacing)
            self._next_sync_due_ms = now_ms + 80
            if len(self._sync_points) >= self._initial_sync_target:
                model = estimate_initial(self._sync_points, k_best=self._initial_sync_kbest)
                if model:
                    self.time_model = model
                self._initial_sync_in_progress = False
                self._next_sync_due_ms = now_ms + self._sync_period_ms
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
                    if self.manual_tab.should_show_log(msg_type):
                        self.manual_tab.append_log_line(line)
                    if self.coord_tab.should_show_log(msg_type):
                        self.coord_tab.append_log_line(line)
                    if self.logger.is_running:
                        self.logger.write_line(line)
                except Exception:
                    pass

                # handle messages
                if isinstance(parsed, MotorData):
                    self._last_motor = parsed
                    self._last_mcu_ts_u32 = parsed.ts_ms
                    self._update_motor(parsed)
                elif isinstance(parsed, SensorTensorData):
                    self._last_d3 = parsed
                    self._last_mcu_ts_u32 = parsed.ts_ms
                    self._update_mcu_time_label()
                elif isinstance(parsed, SyncResp):
                    self._handle_sync_resp(parsed, ev.pc_rx_ms)
                elif isinstance(parsed, (ImuData, TachoData)):
                    self._last_mcu_ts_u32 = parsed.ts_ms
                    # could be extended later
                    self._update_mcu_time_label()
                if isinstance(parsed, ImuData):
                    self._last_imu = parsed
                if isinstance(parsed, TachoData):
                    self._last_tacho = parsed
                    self._update_track_speed_labels(parsed.left_rpm, parsed.right_rpm)
                    self.manual_tab.update_rpm(parsed.left_rpm, parsed.right_rpm)
                    self.coord_tab.add_actual_tacho(parsed.left_rpm, parsed.right_rpm, parsed.ts_ms)
                    # External runtime treats the incoming speed sample as the main processing trigger.
                    self._ingest_external_runtime_snapshot(self.build_telemetry_snapshot(ev.pc_rx_ms))
                elif not isinstance(parsed, (MotorData, SensorTensorData, SyncResp, ImuData)):
                    # unknown Frame - ignore
                    pass

        self._update_mcu_time_label()
        self._update_uart_status()
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
        coord_state = None
        if hasattr(self, "coord_tab"):
            coord_state = self.coord_tab.get_state()
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
            "test_mode": self._is_test_mode(),
            "coordinate": coord_state,
        }
        try:
            with open(self._settings_path, "w", encoding="utf-8") as fh:
                json.dump(data, fh, ensure_ascii=False, indent=2)
        except Exception:
            pass

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
            if self.manual_tab.should_show_log(msg_type):
                self.manual_tab.append_log_line(line, tag="tx")
            if self.coord_tab.should_show_log(msg_type):
                self.coord_tab.append_log_line(line, tag="tx")
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
        if msg_type == TYPE_C0_CONTROL and ln == 10:
            ts, l, r, dur = struct.unpack_from("<IhhH", payload, 0)
            return {"type": "C0", "ts_ms": ts, "left_cmd": l, "right_cmd": r, "duration_ms": dur}
        if msg_type == TYPE_A0_DISABLE_D and ln == 5:
            ts, d_type = struct.unpack_from("<IB", payload, 0)
            return {"type": "A0", "ts_ms": ts, "d_type": d_type}
        if msg_type == TYPE_A1_ENABLE_D and ln == 7:
            ts, d_type, period = struct.unpack_from("<IBH", payload, 0)
            return {"type": "A1", "ts_ms": ts, "d_type": d_type, "period_ms": period}

        parsed = parse_frame(Frame(msg_type=msg_type, payload=payload))
        return self._msg_to_dict(parsed)

    def _quality_0_10(self) -> int:
        if not self._rx_ok:
            return 0
        ok = sum(1 for x in self._rx_ok if x)
        q = int(round(10.0 * ok / len(self._rx_ok)))
        return max(0, min(10, q))

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
        if isinstance(msg, Frame):
            return {"type": "unknown", "msg_type": msg.msg_type, "len": len(msg.payload)}
        return {"type": "unknown", "repr": repr(msg)}
