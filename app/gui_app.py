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
from typing import Optional, Any
import collections

from app.manual_tab import ManualTab
from app.dialogs import ComSettingsDialog, DeviationSettingsDialog, DeviationConfig
from app.styles import PANEL_BG, STATUS_GREEN, STATUS_RED, COLOR_GREEN, COLOR_YELLOW, COLOR_RED

from comm.serial_worker import SerialWorker, RxEvent, RxError
from comm.protocol import (
    build_sync_req, build_control,
    TYPE_D0_IMU, TYPE_D1_TACHO, TYPE_D2_MOTOR,
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

        self.worker = SerialWorker()
        self.logger = ParsedLogger()

        self.dev_cfg = DeviationConfig()

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

        # Radio quality (simple heuristic)
        self._rx_ok = collections.deque(maxlen=200)
        self._last_mcu_ts_u32: Optional[int] = None

        self._build_ui()

        # periodic queue polling
        self.root.after(20, self._poll_rx)

    def run(self) -> None:
        self.root.mainloop()

    # ---------------- UI ----------------

    def _build_ui(self) -> None:
        self._build_menu()

        # Top tabs + main area
        main = ttk.Frame(self.root, padding=6)
        main.pack(fill="both", expand=True)

        self.nb = ttk.Notebook(main)
        self.nb.pack(side="left", fill="both", expand=True)

        self.manual_tab = ManualTab(self.nb, on_control=self._on_manual_control)
        self.nb.add(self.manual_tab, text="Manual")

        coord_placeholder = ttk.Frame(self.nb)
        ttk.Label(coord_placeholder, text="Coordinate: not implemented yet (per TZ).").pack(padx=20, pady=20)
        self.nb.add(coord_placeholder, text="Coordinate")

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

        self.root.config(menu=menubar)

        self._log_path: str = ""

    def _build_right_panel(self, parent: tk.Widget) -> None:
        # Status at top right: Log Running / Stopped
        self.log_status_var = tk.StringVar(value="Log Stopped")
        self.log_status_lbl = ttk.Label(parent, textvariable=self.log_status_var)
        self.log_status_lbl.pack(anchor="ne")
        self._apply_log_status_style()

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
        self.mcu_time_var = tk.StringVar(value="MCU Time: —")
        ttk.Label(parent, textvariable=self.mcu_time_var).pack(anchor="w", pady=(6, 0))

        # Radio Quality
        self.radio_var = tk.StringVar(value="Radio Quality: —/10")
        ttk.Label(parent, textvariable=self.radio_var).pack(anchor="w", pady=(6, 0))

        # Deviation settings button
        ttk.Button(parent, text="Deviation Settings", command=self._open_deviation_settings).pack(fill="x", pady=(14, 0))

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
            # Start initial sync when port opens
            self._begin_initial_sync()

    def _open_deviation_settings(self) -> None:
        dlg = DeviationSettingsDialog(self.root, self.dev_cfg)
        self.root.wait_window(dlg)
        if dlg.result:
            self.dev_cfg = dlg.result
            # Refresh coloring with last known values
            self._recolor_all()

    def _on_exit(self) -> None:
        try:
            self.worker.close()
        finally:
            self.logger.stop()
            self.root.destroy()

    # ---------------- Control logic ----------------

    def _on_manual_control(self, left_cmd: int, right_cmd: int) -> None:
        self._last_control = (left_cmd, right_cmd)

    def _send_control_if_due(self, now_ms: int) -> None:
        if not self.worker.is_open:
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
                try:
                    line = self.logger.format_line(log_obj)
                    print(line, flush=True)
                    msg_type = None
                    msg = log_obj.get("msg")
                    if isinstance(msg, dict):
                        msg_type = msg.get("type")
                    if self.manual_tab.should_show_log(msg_type):
                        self.manual_tab.append_log_line(line)
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
                elif not isinstance(parsed, (MotorData, SyncResp, ImuData)):
                    # unknown Frame - ignore
                    pass

        self._update_mcu_time_label()
        self.root.after(20, self._poll_rx)

    def _quality_0_10(self) -> int:
        if not self._rx_ok:
            return 0
        ok = sum(1 for x in self._rx_ok if x)
        q = int(round(10.0 * ok / len(self._rx_ok)))
        return max(0, min(10, q))

    def _update_mcu_time_label(self) -> None:
        if self._last_mcu_ts_u32 is None:
            self.mcu_time_var.set("MCU Time: —")
            return

        # show MCU time as received u32
        self.mcu_time_var.set(f"MCU Time: {self._last_mcu_ts_u32}")

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
        return {"type": "unknown", "repr": repr(msg)}
