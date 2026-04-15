from __future__ import annotations

import collections
import math
import time
from typing import Callable

import tkinter as tk
from tkinter import ttk

from app.gyroscope_dataset import GyroscopeSampleRecord
from app.magnetometer_tab import MagnetometerTab
from app.source_card import SourceCard
from app.styles import PANEL_BG
from app.tk_utils import bind_vertical_mousewheel, bind_vertical_mousewheel_tree

BUILTIN_SOURCE_DEFS = (
    ("raw_gyroscope", "Raw Gyroscope", "Built-in source"),
)
TRACE_COLOR_X = "#b91c1c"
TRACE_COLOR_Y = "#15803d"
TRACE_COLOR_Z = "#1d4ed8"
TRACE_COLOR_MAG = "#7c3aed"
LIVE_REDRAW_INTERVAL_S = 0.05


def compute_gyro_rate_magnitude(gx: float, gy: float, gz: float) -> float | None:
    if not all(math.isfinite(value) for value in (gx, gy, gz)):
        return None
    return math.sqrt(gx * gx + gy * gy + gz * gz)


class GyroscopeTab(MagnetometerTab):
    def __init__(
        self,
        master: tk.Widget,
        *,
        on_start_record: Callable[[], None] | None = None,
        on_stop_record: Callable[[], None] | None = None,
        on_load_csv: Callable[[], None] | None = None,
        on_load_multiple: Callable[[], None] | None = None,
        on_save_current: Callable[[], None] | None = None,
        on_save_as: Callable[[], None] | None = None,
        on_clear_dataset: Callable[[], None] | None = None,
        on_concatenate: Callable[[], None] | None = None,
        on_trim_selection: Callable[[], None] | None = None,
        on_delete_selection: Callable[[], None] | None = None,
        on_select_dataset: Callable[[int], None] | None = None,
        on_select_source: Callable[[str], None] | None = None,
        on_source_show_change: Callable[[str, bool], None] | None = None,
        on_source_record_change: Callable[[str, bool], None] | None = None,
        on_add_plugin: Callable[[], None] | None = None,
        on_select_method: Callable[[str], None] | None = None,
        on_open_method_info: Callable[[str], None] | None = None,
        on_calibrate_method: Callable[[str], None] | None = None,
        on_load_method_params: Callable[[str], None] | None = None,
        on_save_method_params: Callable[[str], None] | None = None,
        on_clear_method_params: Callable[[str], None] | None = None,
        on_method_show_change: Callable[[str, bool], None] | None = None,
        on_remove_method: Callable[[str], None] | None = None,
        on_enable_method_realtime: Callable[[str], None] | None = None,
        on_disable_method_realtime: Callable[[str], None] | None = None,
        on_method_record_change: Callable[[str, bool], None] | None = None,
        on_select_primary_output: Callable[[str], None] | None = None,
    ) -> None:
        self._raw_history: collections.deque[dict[str, float]] = collections.deque(maxlen=900)
        self._derived_history: dict[str, collections.deque[dict[str, float]]] = {}
        self._time_window_s = 3.0
        self._time_window_buttons: dict[float, ttk.Button] = {}
        self._last_live_redraw_s = 0.0
        super().__init__(
            master,
            on_start_record=on_start_record,
            on_stop_record=on_stop_record,
            on_load_csv=on_load_csv,
            on_load_multiple=on_load_multiple,
            on_save_current=on_save_current,
            on_save_as=on_save_as,
            on_clear_dataset=on_clear_dataset,
            on_concatenate=on_concatenate,
            on_trim_selection=on_trim_selection,
            on_delete_selection=on_delete_selection,
            on_select_dataset=on_select_dataset,
            on_select_source=on_select_source,
            on_source_show_change=on_source_show_change,
            on_source_record_change=on_source_record_change,
            on_add_plugin=on_add_plugin,
            on_select_method=on_select_method,
            on_open_method_info=on_open_method_info,
            on_calibrate_method=on_calibrate_method,
            on_load_method_params=on_load_method_params,
            on_save_method_params=on_save_method_params,
            on_clear_method_params=on_clear_method_params,
            on_method_show_change=on_method_show_change,
            on_remove_method=on_remove_method,
            on_enable_method_realtime=on_enable_method_realtime,
            on_disable_method_realtime=on_disable_method_realtime,
            on_method_record_change=on_method_record_change,
            on_select_primary_heading=on_select_primary_output,
            on_export_metrics=None,
        )

    def _can_redraw_live_view(self, *, first_sample: bool = False) -> bool:
        if not bool(self.winfo_viewable()):
            return False
        now = time.monotonic()
        min_interval = 0.0 if first_sample else LIVE_REDRAW_INTERVAL_S
        if (now - self._last_live_redraw_s) < min_interval:
            return False
        self._last_live_redraw_s = now
        return True

    def _build_sources_panel(self) -> None:
        left = ttk.LabelFrame(self, text="Sources & Methods", padding=8)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 8), pady=(0, 6))
        left.columnconfigure(0, weight=1)
        left.rowconfigure(1, weight=1)

        toolbar = ttk.Frame(left)
        toolbar.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        ttk.Button(toolbar, text="+ Add", command=self._handle_add_plugin).pack(side="left")

        cards_host = ttk.Frame(left)
        cards_host.grid(row=1, column=0, sticky="nsew")
        cards_host.columnconfigure(0, weight=1)
        cards_host.rowconfigure(0, weight=1)

        self.cards_canvas = tk.Canvas(
            cards_host,
            bg=PANEL_BG,
            highlightthickness=1,
            highlightbackground="#808080",
            height=self._top_panel_height,
        )
        self.cards_canvas.grid(row=0, column=0, sticky="nsew")
        bind_vertical_mousewheel(self.cards_canvas, target=self.cards_canvas)

        yscroll = ttk.Scrollbar(cards_host, orient="vertical", command=self.cards_canvas.yview)
        yscroll.grid(row=0, column=1, sticky="ns")
        self.cards_canvas.configure(yscrollcommand=yscroll.set)

        self.cards_frame = ttk.Frame(self.cards_canvas)
        self._cards_window = self.cards_canvas.create_window((0, 0), window=self.cards_frame, anchor="nw")
        bind_vertical_mousewheel(self.cards_frame, target=self.cards_canvas)
        self.cards_frame.bind("<Configure>", self._on_cards_frame_configure)
        self.cards_canvas.bind("<Configure>", self._on_cards_canvas_configure)

        self.source_cards_container = ttk.Frame(self.cards_frame)
        self.source_cards_container.pack(fill="x")
        self.method_cards_container = ttk.Frame(self.cards_frame)
        self.method_cards_container.pack(fill="x", pady=(8, 0))

        for source_id, title, source_type in BUILTIN_SOURCE_DEFS:
            self._add_gyroscope_source_card(source_id, title, source_type)

    def _add_gyroscope_source_card(self, source_id: str, title: str, source_type: str) -> None:
        card = SourceCard(
            self.source_cards_container,
            source_id=source_id,
            title=title,
            source_type=source_type,
            on_select=self._handle_source_selection,
            on_show_change=self._handle_source_show_change,
            on_record_change=self._handle_source_record_change,
        )
        card.pack(fill="x", pady=(0, 8))
        bind_vertical_mousewheel_tree(card, target=self.cards_canvas)
        self._source_cards[source_id] = card

    def _build_view_panel(self) -> None:
        center = ttk.LabelFrame(self, text="Gyroscope View", padding=8)
        center.grid(row=0, column=1, sticky="nsew", padx=8, pady=(0, 6))
        center.columnconfigure(0, weight=1)
        center.rowconfigure(1, weight=1)

        toolbar = ttk.Frame(center)
        toolbar.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        for seconds in (1.0, 3.0, 5.0):
            btn = ttk.Button(toolbar, text=f"{int(seconds)} s", command=lambda seconds=seconds: self._set_time_window(seconds))
            btn.pack(side="left", padx=(0, 6))
            self._time_window_buttons[seconds] = btn
        ttk.Button(toolbar, text="Fit", command=self._fit_trace_view).pack(side="left", padx=(0, 6))
        ttk.Button(toolbar, text="Reset", command=self._reset_trace_view).pack(side="left", padx=(0, 6))
        self.auto_fit_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(toolbar, text="Auto-fit", variable=self.auto_fit_var, command=self._handle_auto_fit_toggle).pack(side="right")

        self.view_canvas = tk.Canvas(
            center,
            bg=PANEL_BG,
            highlightthickness=1,
            highlightbackground="#606060",
            height=self._top_panel_height,
        )
        self.view_canvas.grid(row=1, column=0, sticky="nsew")
        self.view_canvas.bind("<Configure>", self._redraw_view)
        self._update_time_window_buttons()

    def _build_current_data_section(self, parent: tk.Widget) -> ttk.LabelFrame:
        frm = ttk.LabelFrame(parent, text="Current Data", padding=8)
        frm.columnconfigure(1, weight=1)
        labels = (
            "MCU ts",
            "PC rx ts",
            "PC est ts",
            "gx",
            "gy",
            "gz",
            "|ω|",
            "selected stream",
            "dataset status",
        )
        self.current_data_vars = {}
        for row, label in enumerate(labels):
            ttk.Label(frm, text=label).grid(row=row, column=0, sticky="w", pady=2)
            var = tk.StringVar(value="-")
            self.current_data_vars[label] = var
            ttk.Entry(frm, textvariable=var, state="readonly", width=18).grid(
                row=row,
                column=1,
                sticky="ew",
                padx=(8, 0),
                pady=2,
            )
        return frm

    def _build_view_options_section(self, parent: tk.Widget) -> ttk.LabelFrame:
        frm = ttk.LabelFrame(parent, text="View Options", padding=8)
        options = (
            ("show gx", True),
            ("show gy", True),
            ("show gz", True),
            ("show |ω|", True),
            ("show derived traces", True),
            ("auto fit on load", True),
        )
        self.view_option_vars = {}
        for row, (label, value) in enumerate(options):
            var = tk.BooleanVar(value=value)
            self.view_option_vars[label] = var
            ttk.Checkbutton(frm, text=label, variable=var, command=self._handle_view_option_change).grid(
                row=row,
                column=0,
                sticky="w",
                pady=2,
            )
        return frm

    def _build_output_routing_section(self, parent: tk.Widget) -> ttk.LabelFrame:
        frm = ttk.LabelFrame(parent, text="Output Routing", padding=8)
        frm.columnconfigure(1, weight=1)

        ttk.Label(frm, text="visible output streams").grid(row=0, column=0, sticky="w", pady=2)
        self.visible_heading_streams_var = tk.StringVar(value="Raw Gyroscope")
        ttk.Entry(frm, textvariable=self.visible_heading_streams_var, state="readonly", width=18).grid(
            row=0,
            column=1,
            sticky="ew",
            padx=(8, 0),
            pady=2,
        )

        ttk.Label(frm, text="primary output stream").grid(row=1, column=0, sticky="w", pady=2)
        self.primary_heading_var = tk.StringVar(value="Raw Gyroscope")
        self.primary_heading_combo = ttk.Combobox(frm, textvariable=self.primary_heading_var, state="disabled", width=18)
        self.primary_heading_combo.grid(row=1, column=1, sticky="ew", padx=(8, 0), pady=2)
        self.primary_heading_combo.bind("<<ComboboxSelected>>", self._handle_primary_heading_selection)

        ttk.Label(frm, text="streams in recording").grid(row=2, column=0, sticky="nw", pady=2)
        self.record_streams_frame = ttk.Frame(frm)
        self.record_streams_frame.grid(row=2, column=1, sticky="ew", padx=(8, 0), pady=2)
        self.record_streams_frame.columnconfigure(0, weight=1)
        ttk.Label(self.record_streams_frame, text="No derived streams").grid(row=0, column=0, sticky="w")

        ttk.Label(frm, text="autopilot routing").grid(row=3, column=0, sticky="w", pady=2)
        self.autopilot_routing_var = tk.StringVar(value="Coming later")
        ttk.Entry(frm, textvariable=self.autopilot_routing_var, state="readonly", width=18).grid(
            row=3,
            column=1,
            sticky="ew",
            padx=(8, 0),
            pady=2,
        )
        return frm

    def _build_data_tab(self) -> None:
        frm = ttk.Frame(self.bottom_nb, padding=6)
        self.data_tab_frame = frm
        self.bottom_nb.add(frm, text="Data")
        frm.columnconfigure(0, weight=1)
        frm.rowconfigure(0, weight=1)

        columns = (
            "row_id",
            "stream_id",
            "stream_type",
            "producer_name",
            "producer_version",
            "timestamp_mcu",
            "timestamp_pc_rx",
            "timestamp_pc_est",
            "gyro_x",
            "gyro_y",
            "gyro_z",
            "rate_mag",
            "flags",
        )

        tree = ttk.Treeview(frm, columns=columns, show="headings", height=8, selectmode="extended")
        self.data_tree = tree
        tree.grid(row=0, column=0, sticky="nsew")
        bind_vertical_mousewheel(tree)
        for column in columns:
            tree.heading(column, text=column)
            tree.column(column, width=110, stretch=True, anchor="w")

        yscroll = ttk.Scrollbar(frm, orient="vertical", command=tree.yview)
        yscroll.grid(row=0, column=1, sticky="ns")
        xscroll = ttk.Scrollbar(frm, orient="horizontal", command=tree.xview)
        xscroll.grid(row=1, column=0, sticky="ew")
        tree.configure(yscrollcommand=yscroll.set, xscrollcommand=xscroll.set)

    def _build_metrics_tab(self) -> None:
        frm = ttk.Frame(self.bottom_nb, padding=6)
        self.bottom_nb.add(frm, text="Metrics")
        frm.columnconfigure(0, weight=1)
        frm.rowconfigure(0, weight=1)
        ttk.Label(frm, text="Gyroscope metrics are not implemented yet.", anchor="center", justify="center").grid(
            row=0,
            column=0,
            sticky="nsew",
        )
        self.metrics_status_var = tk.StringVar(value="Gyroscope metrics are not available yet.")
        self.export_metrics_btn = ttk.Button(frm, text="Export CSV", state="disabled")
        self.metrics_tree = ttk.Treeview(frm, columns=("metric",), show="headings", height=1)

    def _set_time_window(self, seconds: float) -> None:
        self._time_window_s = seconds
        self._update_time_window_buttons()
        self._redraw_view()

    def _update_time_window_buttons(self) -> None:
        for seconds, button in self._time_window_buttons.items():
            if math.isclose(seconds, self._time_window_s):
                button.state(["disabled"])
            else:
                button.state(["!disabled"])

    def _fit_trace_view(self) -> None:
        self.auto_fit_var.set(True)
        self._redraw_view()

    def _reset_trace_view(self) -> None:
        self._raw_history.clear()
        self._derived_history.clear()
        self._redraw_view()

    def _handle_auto_fit_toggle(self) -> None:
        self._redraw_view()

    def _handle_view_option_change(self) -> None:
        self._update_heading_stream_summary()
        self._redraw_view()

    def _trace_window_records(self) -> tuple[list[dict[str, float]], dict[str, list[dict[str, float]]]]:
        if not self._raw_history:
            return ([], {})
        latest_ts = self._raw_history[-1]["time_s"]
        min_ts = latest_ts - self._time_window_s
        raw_records = [entry for entry in self._raw_history if entry["time_s"] >= min_ts]
        derived_records = {
            method_id: [entry for entry in series if entry["time_s"] >= min_ts]
            for method_id, series in self._derived_history.items()
        }
        return (raw_records, derived_records)

    def _visible_trace_series(self) -> list[tuple[str, str]]:
        series: list[tuple[str, str]] = []
        if self.view_option_vars["show gx"].get():
            series.append(("gx", TRACE_COLOR_X))
        if self.view_option_vars["show gy"].get():
            series.append(("gy", TRACE_COLOR_Y))
        if self.view_option_vars["show gz"].get():
            series.append(("gz", TRACE_COLOR_Z))
        if self.view_option_vars["show |ω|"].get():
            series.append(("rate_mag", TRACE_COLOR_MAG))
        return series

    def _trace_bounds(self) -> tuple[float, float, float, float]:
        raw_records, derived_records = self._trace_window_records()
        if not raw_records and not any(derived_records.values()):
            return (0.0, self._time_window_s, -1.0, 1.0)
        times = [entry["time_s"] for entry in raw_records]
        y_values: list[float] = []
        for entry in raw_records:
            for key, _color in self._visible_trace_series():
                value = entry.get(key)
                if isinstance(value, (int, float)) and math.isfinite(float(value)):
                    y_values.append(float(value))
        if self.view_option_vars["show derived traces"].get():
            for series in derived_records.values():
                for entry in series:
                    value = entry.get("rate_mag")
                    if isinstance(value, (int, float)) and math.isfinite(float(value)):
                        y_values.append(float(value))
                    times.append(entry["time_s"])
        min_t = min(times) if times else 0.0
        max_t = max(times) if times else self._time_window_s
        if math.isclose(min_t, max_t):
            max_t = min_t + self._time_window_s
        if not y_values:
            return (min_t, max_t, -1.0, 1.0)
        max_abs = max(abs(value) for value in y_values)
        max_abs = max(1.0, max_abs * 1.15)
        return (min_t, max_t, -max_abs, max_abs)

    def _trace_to_view(self, time_s: float, value: float, bounds: tuple[float, float, float, float]) -> tuple[float, float]:
        min_t, max_t, min_y, max_y = bounds
        width = max(1.0, float(max(self.view_canvas.winfo_width(), 1)))
        height = max(1.0, float(max(self.view_canvas.winfo_height(), 1)))
        pad_x = 42.0
        pad_y = 24.0
        usable_w = max(1.0, width - pad_x * 2.0)
        usable_h = max(1.0, height - pad_y * 2.0)
        x = pad_x + (time_s - min_t) / max(1e-9, max_t - min_t) * usable_w
        y = pad_y + (max_y - value) / max(1e-9, max_y - min_y) * usable_h
        return (x, y)

    def _draw_trace_series(self, entries: list[dict[str, float]], key: str, color: str, bounds: tuple[float, float, float, float]) -> None:
        points: list[float] = []
        for entry in entries:
            value = entry.get(key)
            if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
                continue
            points.extend(self._trace_to_view(float(entry["time_s"]), float(value), bounds))
        if len(points) >= 4:
            self.view_canvas.create_line(*points, fill=color, width=2)

    def _draw_trace_axes(self, bounds: tuple[float, float, float, float]) -> None:
        width = max(1.0, float(max(self.view_canvas.winfo_width(), 1)))
        height = max(1.0, float(max(self.view_canvas.winfo_height(), 1)))
        pad_x = 42.0
        pad_y = 24.0
        self.view_canvas.create_rectangle(pad_x, pad_y, width - pad_x, height - pad_y, outline="#808080")
        zero_y = self._trace_to_view(bounds[0], 0.0, bounds)[1]
        self.view_canvas.create_line(pad_x, zero_y, width - pad_x, zero_y, fill="#909090", dash=(4, 4))
        self.view_canvas.create_text(pad_x - 20, pad_y, text=f"{bounds[3]:.2f}", fill="#202020")
        self.view_canvas.create_text(pad_x - 20, zero_y, text="0", fill="#202020")
        self.view_canvas.create_text(pad_x - 20, height - pad_y, text=f"{bounds[2]:.2f}", fill="#202020")
        self.view_canvas.create_text(width * 0.5, height - 10, text=f"{self._time_window_s:.0f}s window", fill="#303030")

    def _draw_trace_legend(self) -> None:
        items = self._visible_trace_series()
        if self.view_option_vars["show derived traces"].get():
            for method_id, stream in self._derived_streams.items():
                if bool(stream.get("show", False)):
                    items.append((f"{self._method_titles.get(method_id, method_id)} |ω|", str(stream.get("color", self._stream_color(method_id)))))
        if not items:
            return
        x0 = 38.0
        y0 = 38.0
        row_h = 18.0
        box_h = 24.0 + row_h * len(items)
        self.view_canvas.create_rectangle(x0, y0, x0 + 220, y0 + box_h, fill="#f7f7f7", outline="#909090")
        self.view_canvas.create_text(x0 + 10, y0 + 12, text="Trace legend", anchor="w", fill="#202020")
        for idx, (label, color) in enumerate(items):
            y = y0 + 28.0 + idx * row_h
            self.view_canvas.create_line(x0 + 10, y, x0 + 34, y, fill=color, width=3)
            self.view_canvas.create_text(x0 + 42, y, text=label, anchor="w", fill="#202020")

    def _redraw_view(self, _event: tk.Event | None = None) -> None:
        self.view_canvas.delete("all")
        bounds = self._trace_bounds()
        self._draw_trace_axes(bounds)
        raw_records, derived_records = self._trace_window_records()
        for key, color in self._visible_trace_series():
            self._draw_trace_series(raw_records, key, color, bounds)
        if self.view_option_vars["show derived traces"].get():
            for method_id, series in derived_records.items():
                stream = self._derived_streams.get(method_id)
                if stream is None or not bool(stream.get("show", False)):
                    continue
                color = str(stream.get("color", self._stream_color(method_id)))
                self._draw_trace_series(series, "rate_mag", color, bounds)
        self._draw_trace_legend()

    def update_live_imu(
        self,
        *,
        timestamp_mcu: int,
        timestamp_pc_rx: int,
        timestamp_pc_est: int | None,
        gx: float,
        gy: float,
        gz: float,
        selected_source_label: str | None = None,
        derived_streams: dict[str, dict[str, object]] | None = None,
    ) -> None:
        first_sample = not self._raw_history
        rate_mag = compute_gyro_rate_magnitude(gx, gy, gz)
        self.current_data_vars["MCU ts"].set(str(timestamp_mcu))
        self.current_data_vars["PC rx ts"].set(str(timestamp_pc_rx))
        self.current_data_vars["PC est ts"].set("—" if timestamp_pc_est is None else str(timestamp_pc_est))
        self.current_data_vars["gx"].set(f"{gx:.3f}")
        self.current_data_vars["gy"].set(f"{gy:.3f}")
        self.current_data_vars["gz"].set(f"{gz:.3f}")
        self.current_data_vars["|ω|"].set("—" if rate_mag is None else f"{rate_mag:.3f}")
        self.current_data_vars["selected stream"].set(selected_source_label or "Raw Gyroscope")
        self._update_dataset_status()

        ts_ms = timestamp_pc_est if timestamp_pc_est is not None else timestamp_pc_rx
        ts_s = ts_ms / 1000.0
        self._raw_history.append({
            "time_s": ts_s,
            "gx": gx,
            "gy": gy,
            "gz": gz,
            "rate_mag": 0.0 if rate_mag is None else rate_mag,
        })
        self._derived_streams = {}
        for method_id, stream_state in (derived_streams or {}).items():
            stream = {
                "title": str(stream_state.get("title", method_id)),
                "stream_id": str(stream_state.get("stream_id", f"derived_{method_id}")),
                "mx": float(stream_state.get("gyro_x", stream_state.get("mx", 0.0))),
                "my": float(stream_state.get("gyro_y", stream_state.get("my", 0.0))),
                "mz": float(stream_state.get("gyro_z", stream_state.get("mz", 0.0))),
                "rate_mag": stream_state.get("rate_mag"),
                "show": bool(stream_state.get("show", False)),
                "color": stream_state.get("color", self._stream_color(method_id)),
            }
            self._derived_streams[method_id] = stream
            history = self._derived_history.setdefault(method_id, collections.deque(maxlen=900))
            rate_value = stream.get("rate_mag")
            if not isinstance(rate_value, (int, float)) or not math.isfinite(float(rate_value)):
                rate_value = compute_gyro_rate_magnitude(stream["mx"], stream["my"], stream["mz"]) or 0.0
            history.append({
                "time_s": ts_s,
                "gx": stream["mx"],
                "gy": stream["my"],
                "gz": stream["mz"],
                "rate_mag": float(rate_value),
            })
        stale_ids = set(self._derived_history) - set(self._derived_streams)
        for method_id in stale_ids:
            self._derived_history.pop(method_id, None)
        self._update_heading_stream_summary()
        if self._can_redraw_live_view(first_sample=first_sample):
            self._redraw_view()

    def set_primary_output_display(self, *, source_label: str) -> None:
        self.current_data_vars["selected stream"].set(source_label)

    def _update_heading_stream_summary(self) -> None:
        visible_names: list[str] = []
        if self._source_cards.get("raw_gyroscope") is not None and self._source_cards["raw_gyroscope"].show_var.get():
            visible_names.append("Raw Gyroscope")
        if self.view_option_vars["show derived traces"].get():
            for method_id, stream in self._derived_streams.items():
                if not bool(stream.get("show", False)):
                    continue
                visible_names.append(self._method_titles.get(method_id, str(stream.get("title", method_id))))
        self.visible_heading_streams_var.set(", ".join(visible_names) if visible_names else "None")

    def _record_radius(self, record: GyroscopeSampleRecord) -> float:
        gx = float(record.gyro_x)
        gy = float(record.gyro_y)
        gz = float(record.gyro_z)
        return max(abs(gx), abs(gy), abs(gz), math.sqrt(gx * gx + gy * gy + gz * gz))

    def _insert_dataset_row(self, row_id: int, record: GyroscopeSampleRecord) -> None:
        timestamp_pc_est = "—" if record.timestamp_pc_est is None else str(record.timestamp_pc_est)
        self.data_tree.insert(
            "",
            "end",
            values=(
                row_id,
                record.stream_id,
                record.stream_type,
                record.producer_name,
                record.producer_version,
                record.timestamp_mcu,
                record.timestamp_pc_rx,
                timestamp_pc_est,
                f"{record.gyro_x:.6g}",
                f"{record.gyro_y:.6g}",
                f"{record.gyro_z:.6g}",
                f"{record.rate_mag:.3f}" if record.rate_mag is not None else "—",
                record.flags,
            ),
        )

    def set_metrics_report(self, *, rows: list[dict[str, str]], summary_text: str, can_export: bool) -> None:
        del rows, summary_text, can_export
        self.metrics_status_var.set("Gyroscope metrics are not available yet.")
        self.export_metrics_btn.configure(state="disabled")

    def get_view_state(self) -> dict[str, object]:
        return {
            "time_window_s": self._time_window_s,
            "auto_fit": bool(self.auto_fit_var.get()),
            "view_options": {
                label: bool(var.get())
                for label, var in self.view_option_vars.items()
            },
        }

    def apply_view_state(self, state: dict[str, object]) -> None:
        try:
            time_window_s = float(state.get("time_window_s", self._time_window_s))
        except Exception:
            time_window_s = self._time_window_s
        if time_window_s in self._time_window_buttons:
            self._time_window_s = time_window_s
        self.auto_fit_var.set(bool(state.get("auto_fit", self.auto_fit_var.get())))
        view_options = state.get("view_options", {})
        if isinstance(view_options, dict):
            for label, var in self.view_option_vars.items():
                if label in view_options:
                    var.set(bool(view_options[label]))
        self._update_time_window_buttons()
        self._redraw_view()
