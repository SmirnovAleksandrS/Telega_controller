from __future__ import annotations

import tkinter as tk
from tkinter import ttk
from typing import Callable

from app.styles import COLOR_GREEN, COLOR_RED, COLOR_YELLOW

COLOR_GRAY = "#808080"
COLOR_SELECTED = "#1f6aa5"
METHOD_STATUS_STYLES = {
    "ready": ("READY", COLOR_GREEN),
    "warning": ("WARNING", COLOR_YELLOW),
    "error": ("ERROR", COLOR_RED),
    "calibrating": ("CALIBRATING", COLOR_GREEN),
    "partial": ("PARTIAL", COLOR_GRAY),
}


class MethodCard(tk.Frame):
    def __init__(
        self,
        master: tk.Widget,
        *,
        method_id: str,
        title: str,
        version: str,
        on_select: Callable[[str], None] | None = None,
        on_info: Callable[[str], None] | None = None,
        on_calibrate: Callable[[str], None] | None = None,
        on_load_params: Callable[[str], None] | None = None,
        on_save_params: Callable[[str], None] | None = None,
        on_show_change: Callable[[str, bool], None] | None = None,
        on_record_change: Callable[[str, bool], None] | None = None,
        on_toggle_realtime: Callable[[str], None] | None = None,
        on_remove: Callable[[str], None] | None = None,
    ) -> None:
        super().__init__(
            master,
            bg="#ececec",
            bd=1,
            relief="solid",
            highlightthickness=2,
            highlightbackground=COLOR_GRAY,
            highlightcolor=COLOR_SELECTED,
            padx=0,
            pady=0,
        )
        self.method_id = method_id
        self._on_select = on_select
        self._on_info = on_info
        self._on_calibrate = on_calibrate
        self._on_load_params = on_load_params
        self._on_save_params = on_save_params
        self._on_show_change = on_show_change
        self._on_record_change = on_record_change
        self._on_toggle_realtime = on_toggle_realtime
        self._on_remove = on_remove
        self._title = title

        body = ttk.Frame(self, padding=8)
        body.pack(fill="both", expand=True)

        header = ttk.Frame(body)
        header.pack(fill="x")
        self.title_label = ttk.Label(header, text=title, font=("TkDefaultFont", 10, "bold"))
        self.title_label.pack(side="left")
        self.remove_btn = ttk.Button(header, text="×", width=2, command=self._handle_remove)
        self.remove_btn.pack(side="right", padx=(6, 0))
        self.status_label = tk.Label(header, text="PARTIAL", bg=COLOR_GRAY, fg="#000000", padx=6, pady=2)
        self.status_label.pack(side="right", padx=(0, 4))

        self.version_label = ttk.Label(body, text=f"Version: {version}")
        self.version_label.pack(anchor="w", pady=(6, 0))

        row1 = ttk.Frame(body)
        row1.pack(fill="x", pady=(8, 0))
        self.show_var = tk.BooleanVar(value=False)
        self.show_btn = ttk.Checkbutton(row1, text="Show", variable=self.show_var, command=self._handle_show_change)
        self.show_btn.pack(side="left")
        self.record_var = tk.BooleanVar(value=False)
        self.record_btn = ttk.Checkbutton(
            row1,
            text="Record",
            variable=self.record_var,
            command=self._handle_record_change,
            state="disabled",
        )
        self.record_btn.pack(side="left", padx=(8, 0))
        self.calibrate_btn = ttk.Button(row1, text="Calibrate", command=self._handle_calibrate, state="disabled")
        self.calibrate_btn.pack(side="left", padx=(8, 0))
        self.info_btn = ttk.Button(row1, text="Info", command=self._handle_info)
        self.info_btn.pack(side="right")

        row2 = ttk.Frame(body)
        row2.pack(fill="x", pady=(6, 0))
        self.load_params_btn = ttk.Button(row2, text="Load params", command=self._handle_load_params, state="disabled")
        self.load_params_btn.pack(side="left")
        self.save_params_btn = ttk.Button(row2, text="Save params", command=self._handle_save_params, state="disabled")
        self.save_params_btn.pack(side="left", padx=(8, 0))
        self.realtime_btn = ttk.Button(row2, text="Enable realtime", command=self._handle_toggle_realtime, state="disabled")
        self.realtime_btn.pack(side="right")

        self.progress_canvas = tk.Canvas(body, height=6, bg="#d5d5d5", highlightthickness=0)
        self.progress_canvas.pack(fill="x", pady=(8, 0))
        self.progress_fill = self.progress_canvas.create_rectangle(0, 0, 0, 6, fill=COLOR_GREEN, width=0)
        self.progress_canvas.bind("<Configure>", self._handle_progress_canvas_resize, add="+")
        self._progress_value = 0.0
        self._progress_color = COLOR_GREEN

        for widget in (self, body, header, self.title_label, self.version_label, self.status_label):
            widget.bind("<Button-1>", self._handle_select, add="+")

    @property
    def title(self) -> str:
        return self._title

    def set_selected(self, selected: bool) -> None:
        border_color = COLOR_SELECTED if selected else COLOR_GRAY
        self.configure(highlightbackground=border_color)

    def set_status(self, status: str, *, text: str | None = None) -> None:
        default_text, color = METHOD_STATUS_STYLES.get(status, METHOD_STATUS_STYLES["partial"])
        fg = "#ffffff" if color in {COLOR_GREEN, COLOR_RED} else "#000000"
        self.status_label.configure(text=text or default_text, bg=color, fg=fg)
        if status == "error":
            self.set_progress(0.0, color=COLOR_RED)
        elif status == "warning" and self._progress_value <= 0.0:
            self.set_progress(0.0, color=COLOR_YELLOW)
        elif status == "ready" and self._progress_value <= 0.0:
            self.set_progress(1.0, color=COLOR_GREEN)
        elif status == "calibrating":
            self.set_progress(max(self._progress_value, 0.05), color=COLOR_GREEN)

    def set_show_enabled(self, enabled: bool) -> None:
        self.show_btn.configure(state="normal" if enabled else "disabled")

    def set_show(self, enabled: bool) -> None:
        self.show_var.set(bool(enabled))

    def set_record(self, enabled: bool) -> None:
        self.record_var.set(bool(enabled))

    def set_record_enabled(self, enabled: bool) -> None:
        self.record_btn.configure(state="normal" if enabled else "disabled")

    def set_calibrate_enabled(self, enabled: bool) -> None:
        self.calibrate_btn.configure(state="normal" if enabled else "disabled")

    def set_load_params_enabled(self, enabled: bool) -> None:
        self.load_params_btn.configure(state="normal" if enabled else "disabled")

    def set_save_params_enabled(self, enabled: bool) -> None:
        self.save_params_btn.configure(state="normal" if enabled else "disabled")

    def set_realtime_state(self, *, enabled: bool, can_enable: bool, can_disable: bool) -> None:
        self.realtime_btn.configure(text="Disable realtime" if enabled else "Enable realtime")
        button_enabled = can_disable if enabled else can_enable
        self.realtime_btn.configure(state="normal" if button_enabled else "disabled")

    def set_progress(self, value: float, *, color: str | None = None) -> None:
        self._progress_value = max(0.0, min(1.0, float(value)))
        if color is not None:
            self._progress_color = color
        width = max(0, int(self.progress_canvas.winfo_width()))
        fill_width = width * self._progress_value
        self.progress_canvas.itemconfigure(self.progress_fill, fill=self._progress_color)
        self.progress_canvas.coords(self.progress_fill, 0, 0, fill_width, 6)

    def _handle_select(self, _event: tk.Event) -> None:
        if self._on_select is not None:
            self._on_select(self.method_id)

    def _handle_calibrate(self) -> None:
        if self._on_calibrate is not None:
            self._on_calibrate(self.method_id)

    def _handle_info(self) -> None:
        if self._on_info is not None:
            self._on_info(self.method_id)

    def _handle_load_params(self) -> None:
        if self._on_load_params is not None:
            self._on_load_params(self.method_id)

    def _handle_save_params(self) -> None:
        if self._on_save_params is not None:
            self._on_save_params(self.method_id)

    def _handle_show_change(self) -> None:
        if self._on_show_change is not None:
            self._on_show_change(self.method_id, bool(self.show_var.get()))

    def _handle_record_change(self) -> None:
        if self._on_record_change is not None:
            self._on_record_change(self.method_id, bool(self.record_var.get()))

    def _handle_toggle_realtime(self) -> None:
        if self._on_toggle_realtime is not None:
            self._on_toggle_realtime(self.method_id)

    def _handle_progress_canvas_resize(self, _event: tk.Event) -> None:
        self.set_progress(self._progress_value, color=self._progress_color)

    def _handle_remove(self) -> None:
        if self._on_remove is not None:
            self._on_remove(self.method_id)
