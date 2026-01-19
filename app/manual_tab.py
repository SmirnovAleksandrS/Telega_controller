"""
Manual tab:
- Virtual joystick on Canvas
- Outputs Left/Right values
- Shift & Linear coefficients per side (as in reference)
- Generates control commands (C0) through callback
"""

from __future__ import annotations
import tkinter as tk
from tkinter import ttk
from dataclasses import dataclass
import collections
import math
from typing import Callable

from app.styles import PANEL_BG

@dataclass
class ManualControlState:
    left_cmd: int = 0
    right_cmd: int = 0
    # user coefficients
    left_shift: float = 0.0
    right_shift: float = 0.0
    left_linear: float = 1.0
    right_linear: float = 1.0

class ManualTab(ttk.Frame):
    def __init__(self, master: tk.Widget, on_control: Callable[[int, int], None]) -> None:
        super().__init__(master)
        self.on_control = on_control
        self.state = ManualControlState()

        self._neutral = 1500
        self._range = 500
        self._min_cmd = 1000
        self._max_cmd = 2000
        self._deadzone = 0.02
        self._log_max_lines = 500
        self._log_lines = 0
        self._rpm_samples = collections.deque(maxlen=200)
        self._last_cmd = (self._neutral, self._neutral)
        self._last_rpm = (0, 0)
        self._rpm_tick_ms = 200
        self._log_filters: dict[str, tk.BooleanVar] = {}
        self._filter_desc: dict[str, str] = {}
        self._menu_tooltip: _MenuTooltip | None = None

        # Layout: big left area + right panel will be outside in main window
        # Here we only implement left manual control section.
        self.configure(padding=6)

        self._build()
        self.after(self._rpm_tick_ms, self._rpm_tick)

    def _build(self) -> None:
        # Canvas joystick
        self.canvas = tk.Canvas(self, width=360, height=360, bg=PANEL_BG, highlightthickness=0)
        self.canvas.grid(row=0, column=0, rowspan=4, sticky="nw", padx=(0, 10), pady=(0, 6))

        # Draw rings and knob
        self.center = (180, 180)
        self.radius = 135

        for r in [40, 80, 120, 160]:
            self.canvas.create_oval(
                self.center[0]-r, self.center[1]-r,
                self.center[0]+r, self.center[1]+r,
                outline="#808080"
            )

        self.knob_r = 40
        cx, cy = self.center
        self.knob = self.canvas.create_oval(cx-self.knob_r, cy-self.knob_r, cx+self.knob_r, cy+self.knob_r,
                                            fill="#4aa3ff", outline="#2b6aa3")

        self._dragging = False
        self.canvas.bind("<ButtonPress-1>", self._on_press)
        self.canvas.bind("<B1-Motion>", self._on_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_release)

        # Controls block to the right of joystick (as in reference)
        ttk.Label(self, text="Left").grid(row=0, column=1, sticky="w")
        self.left_val_var = tk.StringVar(value=str(self._neutral))
        ttk.Entry(self, textvariable=self.left_val_var, width=8, state="readonly").grid(row=0, column=2, sticky="w", padx=(6, 8))

        ttk.Label(self, text="Shift").grid(row=0, column=3, sticky="w")
        self.left_shift_var = tk.StringVar(value="0")
        ttk.Entry(self, textvariable=self.left_shift_var, width=6).grid(row=0, column=4, sticky="w", padx=(6, 8))

        ttk.Label(self, text="Linear").grid(row=0, column=5, sticky="w")
        self.left_linear_var = tk.StringVar(value="1")
        ttk.Entry(self, textvariable=self.left_linear_var, width=6).grid(row=0, column=6, sticky="w")

        ttk.Label(self, text="Right").grid(row=1, column=1, sticky="w", pady=(6, 0))
        self.right_val_var = tk.StringVar(value=str(self._neutral))
        ttk.Entry(self, textvariable=self.right_val_var, width=8, state="readonly").grid(row=1, column=2, sticky="w", padx=(6, 8), pady=(6, 0))

        ttk.Label(self, text="Shift").grid(row=1, column=3, sticky="w", pady=(6, 0))
        self.right_shift_var = tk.StringVar(value="0")
        ttk.Entry(self, textvariable=self.right_shift_var, width=6).grid(row=1, column=4, sticky="w", padx=(6, 8), pady=(6, 0))

        ttk.Label(self, text="Linear").grid(row=1, column=5, sticky="w", pady=(6, 0))
        self.right_linear_var = tk.StringVar(value="1")
        ttk.Entry(self, textvariable=self.right_linear_var, width=6).grid(row=1, column=6, sticky="w", pady=(6, 0))

        # Update coeffs on typing
        for var in [self.left_shift_var, self.right_shift_var, self.left_linear_var, self.right_linear_var]:
            var.trace_add("write", lambda *_: self._read_coeffs())

        # Stretch
        self.columnconfigure(0, weight=1)
        for c in range(1, 7):
            self.columnconfigure(c, weight=0)
        self.rowconfigure(3, weight=1)
        self.rowconfigure(4, weight=0)

        # Bottom notebook (Log / RPM)
        self.bottom_nb = ttk.Notebook(self)
        self.bottom_nb.grid(row=4, column=0, columnspan=7, sticky="nsew", pady=(6, 0))

        self._build_log_tab()
        self._build_rpm_tab()

    def _read_coeffs(self) -> None:
        def f(var: tk.StringVar, default: float) -> float:
            try:
                return float(var.get().strip())
            except ValueError:
                return default
        self.state.left_shift = f(self.left_shift_var, 0.0)
        self.state.right_shift = f(self.right_shift_var, 0.0)
        self.state.left_linear = f(self.left_linear_var, 1.0)
        self.state.right_linear = f(self.right_linear_var, 1.0)

    def _on_press(self, e: tk.Event) -> None:
        self._dragging = True
        self._move_knob(e.x, e.y)

    def _on_drag(self, e: tk.Event) -> None:
        if self._dragging:
            self._move_knob(e.x, e.y)

    def _on_release(self, e: tk.Event) -> None:
        self._dragging = False
        # Return to center
        cx, cy = self.center
        self._set_knob_pos(cx, cy)
        self._apply_joystick(0.0, 0.0)

    def _move_knob(self, x: int, y: int) -> None:
        cx, cy = self.center
        dx = x - cx
        dy = y - cy

        # Clamp to radius
        dist = math.hypot(dx, dy)
        if dist > self.radius:
            scale = self.radius / dist
            dx *= scale
            dy *= scale

        nx = cx + dx
        ny = cy + dy
        self._set_knob_pos(nx, ny)

        # Convert to normalized [-1..1]
        # forward is up on screen: dy negative => forward positive
        v = -dy / self.radius
        w = dx / self.radius

        # deadzone
        if abs(v) < self._deadzone: v = 0.0
        if abs(w) < self._deadzone: w = 0.0

        self._apply_joystick(v, w)

    def _set_knob_pos(self, x: float, y: float) -> None:
        self.canvas.coords(self.knob, x-self.knob_r, y-self.knob_r, x+self.knob_r, y+self.knob_r)

    def _apply_joystick(self, v: float, w: float) -> None:
        """
        Differential drive mixing:
          mixL = v + w
          mixR = v - w
        Then map to absolute throttle:
          cmd = neutral + mix * range
        Then apply per-side shift and linear:
          cmd = (cmd + shift) * linear
        Finally clamp to [1000..2000].
        """
        mix_l = max(-1.0, min(1.0, v + w))
        mix_r = max(-1.0, min(1.0, v - w))

        self._read_coeffs()
        raw_l = self._neutral + mix_l * self._range
        raw_r = self._neutral + mix_r * self._range

        raw_l = (raw_l + self.state.left_shift) * self.state.left_linear
        raw_r = (raw_r + self.state.right_shift) * self.state.right_linear

        left_cmd = int(round(max(self._min_cmd, min(self._max_cmd, raw_l))))
        right_cmd = int(round(max(self._min_cmd, min(self._max_cmd, raw_r))))

        self.state.left_cmd = left_cmd
        self.state.right_cmd = right_cmd
        self._last_cmd = (left_cmd, right_cmd)

        self.left_val_var.set(str(left_cmd))
        self.right_val_var.set(str(right_cmd))

        # Notify app to send command (or schedule)
        self.on_control(left_cmd, right_cmd)

    # ---------------- Log / RPM UI ----------------

    def _build_log_tab(self) -> None:
        frm = ttk.Frame(self.bottom_nb, padding=6)
        self.bottom_nb.add(frm, text="Log")

        topbar = ttk.Frame(frm)
        topbar.pack(fill="x", pady=(0, 6))

        self._filter_btn = ttk.Menubutton(topbar, text="Log Filters", direction="below")
        self._filter_btn.pack(side="left")
        self._build_log_filters(self._filter_btn)

        body = ttk.Frame(frm)
        body.pack(fill="both", expand=True)

        self.log_text = tk.Text(body, height=8, wrap="none")
        self.log_text.pack(side="left", fill="both", expand=True)
        self.log_text.tag_configure("tx", foreground="#001a66")
        self.log_text.configure(state="disabled")

        yscroll = ttk.Scrollbar(body, orient="vertical", command=self.log_text.yview)
        yscroll.pack(side="right", fill="y")
        self.log_text.configure(yscrollcommand=yscroll.set)

    def _build_rpm_tab(self) -> None:
        frm = ttk.Frame(self.bottom_nb, padding=6)
        self.bottom_nb.add(frm, text="RPM")
        self.rpm_canvas = tk.Canvas(frm, height=140, bg=PANEL_BG, highlightthickness=1, highlightbackground="#333333")
        self.rpm_canvas.pack(fill="both", expand=True)

    def append_log_line(self, line: str, tag: str | None = None) -> None:
        self.log_text.configure(state="normal")
        if tag:
            self.log_text.insert("end", line + "\n", (tag,))
        else:
            self.log_text.insert("end", line + "\n")
        self._log_lines += 1
        if self._log_lines > self._log_max_lines:
            self.log_text.delete("1.0", "2.0")
            self._log_lines -= 1
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def should_show_log(self, msg_type: str | None) -> bool:
        if not msg_type:
            return True
        var = self._log_filters.get(msg_type)
        return True if var is None else bool(var.get())

    def _build_log_filters(self, btn: ttk.Menubutton) -> None:
        menu = tk.Menu(btn, tearoff=0)
        btn["menu"] = menu

        def add_group(label: str, items: list[tuple[str, str]]) -> None:
            sub = tk.Menu(menu, tearoff=0)
            menu.add_cascade(label=label, menu=sub)
            for msg_type, desc in items:
                var = tk.BooleanVar(value=True)
                self._log_filters[msg_type] = var
                self._filter_desc[msg_type] = desc
                sub.add_checkbutton(label=msg_type, variable=var)
            sub.bind("<<MenuSelect>>", self._on_filter_menu_select)
            sub.bind("<Unmap>", self._on_filter_menu_unmap)

        add_group("A: Admin", [("A0", "A0 - disable D messages"), ("A1", "A1 - enable D messages")])
        add_group("B: Requests", [("B0", "B0 - sync request")])
        add_group("C: Control", [("C0", "C0 - control command")])
        add_group("D: Data", [
            ("D0", "D0 - IMU data"),
            ("D1", "D1 - tacho RPM"),
            ("D2", "D2 - motor currents/voltage/temp"),
        ])
        add_group("E: Errors", [("E0", "E0 - error code")])
        add_group("F: Responses", [("F0", "F0 - sync response")])
        add_group("Other", [
            ("unknown", "unknown - unparsed frame"),
        ])

        self._menu_tooltip = _MenuTooltip(btn)
        menu.bind("<<MenuSelect>>", self._on_filter_menu_select)
        menu.bind("<Unmap>", self._on_filter_menu_unmap)

    def _on_filter_menu_select(self, event: tk.Event) -> None:
        if self._menu_tooltip is None:
            return
        menu = event.widget
        idx = menu.index("active")
        if idx is None:
            self._menu_tooltip.hide()
            return
        label = menu.entrycget(idx, "label")
        desc = self._filter_desc.get(label)
        if not desc:
            self._menu_tooltip.hide()
            return
        x = self.winfo_pointerx() + 12
        y = self.winfo_pointery() + 12
        self._menu_tooltip.show(desc, x, y)

    def _on_filter_menu_unmap(self, _event: tk.Event) -> None:
        if self._menu_tooltip is not None:
            self._menu_tooltip.hide()

    def update_rpm(self, left_rpm: int, right_rpm: int) -> None:
        self._last_rpm = (left_rpm, right_rpm)
        left_cmd, right_cmd = self._last_cmd
        cmd_l_scaled = (left_cmd - self._neutral) / 10.0
        cmd_r_scaled = (right_cmd - self._neutral) / 10.0
        self._rpm_samples.append((left_rpm, right_rpm, cmd_l_scaled, cmd_r_scaled))
        self._redraw_rpm()

    def _redraw_rpm(self) -> None:
        c = self.rpm_canvas
        w = max(1, int(c.winfo_width()))
        h = max(1, int(c.winfo_height()))
        c.delete("all")

        pad = 20
        left = pad
        right = w - pad
        top = pad
        bottom = h - pad

        # Axes
        c.create_line(left, bottom, right, bottom, fill="#000000")
        c.create_line(left, top, left, bottom, fill="#000000")
        c.create_text(left + 4, top, text="RPM / cmd", anchor="nw", fill="#000000")

        if len(self._rpm_samples) < 2:
            return

        # Scale
        max_abs = 1.0
        for l_rpm, r_rpm, l_cmd, r_cmd in self._rpm_samples:
            max_abs = max(max_abs, abs(l_rpm), abs(r_rpm), abs(l_cmd), abs(r_cmd))
        max_abs = max(50.0, max_abs)

        def y_of(v: float) -> float:
            return top + (max_abs - v) * (bottom - top) / (2 * max_abs)

        n = len(self._rpm_samples)
        def x_of(i: int) -> float:
            return left + i * (right - left) / (n - 1)

        # Lines: left rpm (red), right rpm (blue), left cmd (green), right cmd (orange)
        colors = {
            "l_rpm": "#cc0000",
            "r_rpm": "#0044cc",
            "l_cmd": "#009900",
            "r_cmd": "#cc7700",
        }

        def draw_series(idx: int, color: str) -> None:
            pts = []
            for i, sample in enumerate(self._rpm_samples):
                v = sample[idx]
                pts.extend([x_of(i), y_of(v)])
            c.create_line(*pts, fill=color, width=2)

        draw_series(0, colors["l_rpm"])
        draw_series(1, colors["r_rpm"])
        draw_series(2, colors["l_cmd"])
        draw_series(3, colors["r_cmd"])

        # Legend
        c.create_text(right - 5, top, text="L RPM  R RPM  L cmd  R cmd", anchor="ne", fill="#000000")

    def _rpm_tick(self) -> None:
        # Always update graph from joystick commands even without MCU data.
        self.update_rpm(self._last_rpm[0], self._last_rpm[1])
        self.after(self._rpm_tick_ms, self._rpm_tick)


class _MenuTooltip:
    def __init__(self, master: tk.Widget) -> None:
        self.master = master
        self._tip: tk.Toplevel | None = None
        self._label: tk.Label | None = None

    def show(self, text: str, x: int, y: int) -> None:
        if self._tip is None:
            self._tip = tk.Toplevel(self.master)
            self._tip.wm_overrideredirect(True)
            self._tip.attributes("-topmost", True)
            self._label = tk.Label(self._tip, text="", bg="#ffffe0", relief="solid", borderwidth=1)
            self._label.pack(ipadx=4, ipady=2)
        assert self._label is not None
        self._label.configure(text=text)
        self._tip.geometry(f"+{x}+{y}")
        self._tip.deiconify()

    def hide(self) -> None:
        if self._tip is not None:
            self._tip.withdraw()
