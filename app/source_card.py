from __future__ import annotations

import tkinter as tk
from tkinter import ttk
from typing import Callable

from app.styles import COLOR_GREEN, COLOR_YELLOW

COLOR_GRAY = "#808080"
COLOR_SELECTED = "#1f6aa5"


def resolve_source_card_status(show_enabled: bool, record_enabled: bool) -> tuple[str, str]:
    if show_enabled and record_enabled:
        return ("ACTIVE", COLOR_GREEN)
    if show_enabled or record_enabled:
        return ("PARTIAL", COLOR_YELLOW)
    return ("IDLE", COLOR_GRAY)


class SourceCard(tk.Frame):
    def __init__(
        self,
        master: tk.Widget,
        *,
        source_id: str,
        title: str,
        source_type: str,
        on_select: Callable[[str], None] | None = None,
        on_show_change: Callable[[str, bool], None] | None = None,
        on_record_change: Callable[[str, bool], None] | None = None,
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
        self.source_id = source_id
        self._on_select = on_select
        self._on_show_change = on_show_change
        self._on_record_change = on_record_change
        self._title = title

        body = ttk.Frame(self, padding=8)
        body.pack(fill="both", expand=True)
        self._body = body

        header = ttk.Frame(body)
        header.pack(fill="x")
        self._header = header

        self.title_label = ttk.Label(header, text=title, font=("TkDefaultFont", 10, "bold"))
        self.title_label.pack(side="left")
        self.status_label = tk.Label(header, text="IDLE", bg=COLOR_GRAY, fg="#000000", padx=6, pady=2)
        self.status_label.pack(side="right")

        self.type_label = ttk.Label(body, text=f"Type: {source_type}")
        self.type_label.pack(anchor="w", pady=(6, 0))

        controls = ttk.Frame(body)
        controls.pack(fill="x", pady=(8, 0))
        self.show_var = tk.BooleanVar(value=False)
        self.record_var = tk.BooleanVar(value=False)
        self.show_btn = ttk.Checkbutton(controls, text="Show", variable=self.show_var, command=self._handle_show)
        self.show_btn.pack(side="left")
        self.record_btn = ttk.Checkbutton(controls, text="Record", variable=self.record_var, command=self._handle_record)
        self.record_btn.pack(side="left", padx=(8, 0))
        ttk.Button(controls, text="Info", state="disabled").pack(side="right")

        self._bind_select(self)
        self._bind_select(body)
        self._bind_select(header)
        self._bind_select(self.title_label)
        self._bind_select(self.type_label)
        self._bind_select(self.status_label)

    @property
    def title(self) -> str:
        return self._title

    def set_selected(self, selected: bool) -> None:
        border_color = COLOR_SELECTED if selected else COLOR_GRAY
        self.configure(highlightbackground=border_color)

    def set_show(self, enabled: bool) -> None:
        self.show_var.set(bool(enabled))

    def set_record(self, enabled: bool) -> None:
        self.record_var.set(bool(enabled))

    def set_status(self, text: str, color: str) -> None:
        fg = "#ffffff" if color == COLOR_GREEN else "#000000"
        self.status_label.configure(text=text, bg=color, fg=fg)

    def _bind_select(self, widget: tk.Widget) -> None:
        widget.bind("<Button-1>", self._handle_select, add="+")

    def _handle_select(self, _event: tk.Event) -> None:
        if self._on_select is not None:
            self._on_select(self.source_id)

    def _handle_show(self) -> None:
        if self._on_show_change is not None:
            self._on_show_change(self.source_id, bool(self.show_var.get()))

    def _handle_record(self) -> None:
        if self._on_record_change is not None:
            self._on_record_change(self.source_id, bool(self.record_var.get()))
