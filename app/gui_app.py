"""
Main GUI application:
- Menus: Files (start/stop log), Settings (COM settings)
- Tabs: Manual (implemented), Coordinate (placeholder)
- Right side panel with critical params, MCU time, radio quality, button deviation settings
- Serial worker + protocol handling + time sync
"""

from __future__ import annotations
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

from comm.serial_worker import SerialWorker, RxEvent, RxError
from comm.protocol import (
    build_sync_req, build_control,
    TYPE_D0_IMU, TYPE_D1_TACHO, TYPE_D2_MOTOR,
    TYPE_B0_SYNC_REQ, TYPE_C0_CONTROL, TYPE_A0_DISABLE_D, TYPE_A1_ENABLE_D,
    SOF, Frame, parse_frame,
    ImuData, TachoData, MotorData, SyncResp
)
from comm.time_sync import TimeModel, compute_sync_point, estimate_initial, update_model, SyncPoint
from utils.timebase import now_ms_monotonic, u32

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
        self._last_control = (0, 0)
        self._control_period_ms = 50
        self._control_duration_ms = 100
        self._next_control_due_ms = now_ms_monotonic()
        self._manual_neutral = 1500
        self._active_tab = "Manual"
        self._kill_active = False
        self._kill_tick_ms = 200
        self._coord_running = False
        self._coord_tick_ms = 10

        # Radio quality (simple heuristic)
        self._rx_ok = collections.deque(maxlen=200)
        self._last_mcu_ts_u32: Optional[int] = None

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self._on_exit)

        # periodic queue polling
        self.root.after(20, self._poll_rx)

    def run(self) -> None:
        try:
            self.root.mainloop()
        finally:
            self.shutdown()

    # ---------------- UI ----------------

    def _build_ui(self) -> None:
        self._build_menu()

        # Top tabs + main area
        main = ttk.Frame(self.root, padding=6)
        main.pack(fill="both", expand=True)

        self.nb = ttk.Notebook(main)
        self.nb.pack(side="left", fill="both", expand=True)

        self.manual_tab = ManualTab(self.nb, on_control=self._on_manual_control, on_coeffs_change=self._on_manual_coeffs)
        self.nb.add(self.manual_tab, text="Manual")
        self._apply_joystick_settings()

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

        self._make_param_row(crit, 1, "Voltage", self.volt_l, self.volt_r)
        self._make_param_row(crit, 2, "Current", self.cur_l, self.cur_r)
        self._make_param_row(crit, 3, "Temperature", self.temp_l, self.temp_r)

        # MCU Time
        self.mcu_time_var = tk.StringVar(value="MCU Time:\nrx=—\nest=—")
        ttk.Label(parent, textvariable=self.mcu_time_var).pack(anchor="w", pady=(6, 0))

        # Radio Quality
        self.radio_var = tk.StringVar(value="Radio Quality: —/10")
        ttk.Label(parent, textvariable=self.radio_var).pack(anchor="w", pady=(6, 0))

        # Deviation settings button
        ttk.Button(parent, text="Deviation Settings", command=self._open_deviation_settings).pack(fill="x", pady=(14, 0))
        ttk.Button(parent, text="Time Sync", command=self._begin_initial_sync).pack(fill="x", pady=(8, 0))
        self.kill_btn = ttk.Button(parent, text="KILL SWITCH", command=self._kill_switch)
        self.kill_btn.pack(fill="x", pady=(12, 0))
        self._apply_kill_style()

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
                messagebox.showerror("COM Settings", "Failed to open selected port")
        self._refresh_connect_btn()

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

    def _toggle_connect(self) -> None:
        if self.worker.is_open:
            self.worker.close()
            if self._coord_running:
                self._coord_running = False
                self.coord_tab.set_running(False)
                self.coord_tab.stop_expected()
            self._refresh_connect_btn()
            return

        if self.worker.port:
            ports = set(SerialWorker.list_ports())
            if self.worker.port not in ports:
                messagebox.showerror(
                    "COM Settings",
                    f"Saved port '{self.worker.port}' is not available.\nSelect a port in COM Settings.",
                )
                self._open_com_settings()
                self._refresh_connect_btn()
                return
            self.worker.open(self.worker.port, self.worker.baud)
            if self.worker.is_open:
                self._begin_initial_sync()
            else:
                messagebox.showerror("COM Settings", "Failed to open port")
        else:
            self._open_com_settings()
        self._refresh_connect_btn()

    def _refresh_connect_btn(self) -> None:
        if self.worker.is_open:
            self.connect_btn.configure(text="Disconnect")
        else:
            self.connect_btn.configure(text="Connect")

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
        if self.worker.is_open and not self._is_test_mode():
            self._begin_initial_sync()
        if not self.coord_tab.start_controller():
            messagebox.showerror("Coordinate", "No valid trajectory for controller")
            return
        self._coord_tick_ms = max(1, self.coord_tab.get_time_quantum_ms())
        self._coord_running = True
        self.coord_tab.set_running(True)
        self.coord_tab.reset_actual_trace()
        self.coord_tab.start_expected(now_ms_monotonic())
        self._coord_send_tick()

    def _on_coord_stop(self) -> None:
        if not self._coord_running:
            self.coord_tab.set_running(False)
            self.coord_tab.stop_expected()
            return
        self._coord_running = False
        self.coord_tab.set_running(False)
        self.coord_tab.stop_expected()

    def _coord_send_tick(self) -> None:
        if not self._coord_running or self._kill_active:
            return
        test_mode = self._is_test_mode()
        if not self.worker.is_open and not test_mode:
            self._coord_running = False
            self.coord_tab.set_running(False)
            self.coord_tab.stop_expected()
            return
        dt_s = max(1, self._coord_tick_ms) / 1000.0
        cmd = self.coord_tab.step_controller(dt_s, force_internal_pose=test_mode)
        if self.coord_tab.controller_finished():
            self._coord_running = False
            self.coord_tab.set_running(False)
            self.coord_tab.stop_expected()
            return
        if cmd is None:
            left_cmd, right_cmd = self._manual_neutral, self._manual_neutral
        else:
            left_cmd, right_cmd = cmd
        if self.worker.is_open and not test_mode:
            # Swap tracks for coordinate mode (per hardware wiring).
            payload = build_control(now_ms_monotonic(), right_cmd, left_cmd, self._coord_tick_ms)
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

        # Timestamp for C0 should be "MCU time in ms" per protocol.
        # If we have sync lock, translate PC->MCU estimate, else just send PC u32 ms.
        if self.time_model.have_lock:
            ts_mcu_est = int(round(self.time_model.mcu_from_pc(now_ms)))
            ts_u32 = u32(ts_mcu_est)
        else:
            ts_u32 = u32(now_ms)

        pkt = build_control(ts_u32, left_cmd, right_cmd, self._control_duration_ms)
        self.worker.send(pkt)

    def _on_tab_change(self, _event: tk.Event) -> None:
        try:
            tab_text = self.nb.tab(self.nb.select(), "text")
        except Exception:
            return
        self._active_tab = tab_text
        if tab_text != "Coordinate" and self._coord_running:
            self._coord_running = False
            self.coord_tab.set_running(False)
            self.coord_tab.stop_expected()
        if tab_text != "Manual" and self.worker.is_open and not self._kill_active:
            ts_u32 = u32(now_ms_monotonic())
            pkt = build_control(ts_u32, self._manual_neutral, self._manual_neutral, self._control_duration_ms)
            self.worker.send(pkt)

    def _kill_switch(self) -> None:
        self._kill_active = not self._kill_active
        self._update_kill_indicator()
        if self._kill_active:
            self._coord_running = False
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
            ts_u32 = u32(now_ms_monotonic())
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
                if isinstance(parsed, (ImuData, TachoData, MotorData)):
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
                    self._last_mcu_ts_u32 = parsed.ts_ms
                    self._update_motor(parsed)
                elif isinstance(parsed, SyncResp):
                    self._handle_sync_resp(parsed, ev.pc_rx_ms)
                elif isinstance(parsed, (ImuData, TachoData)):
                    self._last_mcu_ts_u32 = parsed.ts_ms
                    # could be extended later
                    self._update_mcu_time_label()
                if isinstance(parsed, TachoData):
                    self.manual_tab.update_rpm(parsed.left_rpm, parsed.right_rpm)
                    self.coord_tab.add_actual_tacho(parsed.left_rpm, parsed.right_rpm, parsed.ts_ms)
                elif not isinstance(parsed, (MotorData, SyncResp, ImuData)):
                    # unknown Frame - ignore
                    pass

        self._update_mcu_time_label()
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
        try:
            self.geom_cfg = GeometryConfig(
                a1_cm=float(geom.get("a1_cm", self.geom_cfg.a1_cm)),
                a2_cm=float(geom.get("a2_cm", self.geom_cfg.a2_cm)),
            )
        except Exception:
            pass

        speed = data.get("speed_map", {})
        try:
            self.speed_cfg = SpeedMapConfig(
                pwm_1=float(speed.get("pwm_1", self.speed_cfg.pwm_1)),
                speed_1=float(speed.get("speed_1", self.speed_cfg.speed_1)),
                pwm_2=float(speed.get("pwm_2", self.speed_cfg.pwm_2)),
                speed_2=float(speed.get("speed_2", self.speed_cfg.speed_2)),
                pwm_3=float(speed.get("pwm_3", self.speed_cfg.pwm_3)),
                speed_3=float(speed.get("speed_3", self.speed_cfg.speed_3)),
                track_circumference_m=float(speed.get("track_circumference_m", self.speed_cfg.track_circumference_m)),
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

    def _apply_geometry_settings(self) -> None:
        if not hasattr(self, "geom_cfg"):
            return
        self.coord_tab.set_geometry(self.geom_cfg.a1_cm, self.geom_cfg.a2_cm)

    def _apply_speed_map_settings(self) -> None:
        if not hasattr(self, "speed_cfg"):
            return
        self.coord_tab.set_speed_map(self.speed_cfg)

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
            },
            "speed_map": {
                "pwm_1": self.speed_cfg.pwm_1,
                "speed_1": self.speed_cfg.speed_1,
                "pwm_2": self.speed_cfg.pwm_2,
                "speed_2": self.speed_cfg.speed_2,
                "pwm_3": self.speed_cfg.pwm_3,
                "speed_3": self.speed_cfg.speed_3,
                "track_circumference_m": self.speed_cfg.track_circumference_m,
            },
            "joystick": {
                "left_shift": self._joystick_cfg["left_shift"],
                "right_shift": self._joystick_cfg["right_shift"],
                "left_linear": self._joystick_cfg["left_linear"],
                "right_linear": self._joystick_cfg["right_linear"],
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
        if isinstance(msg, ImuData):
            return {"type": "D0", **asdict(msg)}
        if isinstance(msg, TachoData):
            return {"type": "D1", **asdict(msg)}
        if isinstance(msg, SyncResp):
            return {"type": "F0", "t2_rx_ms": msg.t2_rx_ms, "t3_tx_ms": msg.t3_tx_ms}
        if isinstance(msg, Frame):
            return {"type": "unknown", "msg_type": msg.msg_type, "len": len(msg.payload)}
        return {"type": "unknown", "repr": repr(msg)}
