from __future__ import annotations

import math
import os
import tkinter as tk
from tkinter import filedialog, ttk
from typing import Callable

os.environ["MPLCONFIGDIR"] = "/tmp/telega_matplotlib"
os.makedirs(os.environ["MPLCONFIGDIR"], exist_ok=True)

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

from app.magnetometer_analysis import (
    AnalysisDataAvailability,
    AnalysisFigureSlot,
    AnalysisMethodState,
    AnalysisSnapshot,
    MagnetometerAnalysisResult,
    RAW_METHOD_ID,
    build_analysis_result,
    raw_method_state,
    unwrap_degrees,
)
from app.styles import COLOR_GREEN, COLOR_RED, COLOR_YELLOW, PANEL_BG
from app.tk_utils import bind_vertical_mousewheel, bind_vertical_mousewheel_tree, prepare_toplevel


STATUS_COLORS = {
    "ready": COLOR_GREEN,
    "warning": COLOR_YELLOW,
    "error": COLOR_RED,
    "calibrating": COLOR_GREEN,
    "partial": "#808080",
}


class MagnetometerExtendedAnalysisWindow(tk.Toplevel):
    def __init__(
        self,
        master: tk.Widget,
        *,
        snapshot_provider: Callable[[], AnalysisSnapshot],
        on_load_params: Callable[[str], None] | None = None,
        on_save_params: Callable[[str], None] | None = None,
    ) -> None:
        super().__init__(master)
        self.title("Magnetometer Extended Analysis")
        self.geometry("1280x820")
        self.minsize(980, 640)

        self._snapshot_provider = snapshot_provider
        self._on_load_params = on_load_params
        self._on_save_params = on_save_params
        self._snapshot = snapshot_provider()
        self._method_vars: dict[str, dict[str, tk.BooleanVar]] = {}
        self._selected_method_id = self._snapshot.selected_method_id
        self._method_detail_vars: dict[str, tk.StringVar] = {}
        self._availability_vars: dict[str, tk.StringVar] = {}
        self._dataset_vars: dict[str, tk.StringVar] = {}
        self._figures: dict[str, Figure] = {}
        self._figure_canvases: dict[str, FigureCanvasTkAgg] = {}
        self._figure_status_vars: dict[str, tk.StringVar] = {}
        self._figure_export_buttons: dict[str, ttk.Button] = {}
        self._figure_open_buttons: dict[str, ttk.Button] = {}
        self._last_result: MagnetometerAnalysisResult | None = None

        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)
        self._build()
        self.refresh()

        prepare_toplevel(self, master)
        self.protocol("WM_DELETE_WINDOW", self.destroy)

    def refresh(self) -> None:
        self._snapshot = self._snapshot_provider()
        if self._selected_method_id not in {method.method_id for method in self._display_methods()}:
            self._selected_method_id = self._snapshot.selected_method_id or (
                RAW_METHOD_ID if self._snapshot.raw_records else None
            )
        self._refresh_dataset_summary()
        self._refresh_availability()
        self._refresh_methods()
        self._refresh_selected_method_details()
        self._refresh_overview_tables()
        self._render_all_figures()

    def _build(self) -> None:
        root = ttk.Frame(self, padding=8)
        root.grid(row=0, column=0, sticky="nsew")
        root.columnconfigure(0, weight=1)
        root.rowconfigure(1, weight=1)

        toolbar = ttk.Frame(root)
        toolbar.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        self.updated_var = tk.StringVar(value="")
        ttk.Label(toolbar, textvariable=self.updated_var).pack(side="left")
        ttk.Button(toolbar, text="Refresh", command=self.refresh).pack(side="right")

        panes = ttk.Panedwindow(root, orient="horizontal")
        panes.grid(row=1, column=0, sticky="nsew")

        left = ttk.Frame(panes, padding=(0, 0, 8, 0))
        left.columnconfigure(0, weight=1)
        left.rowconfigure(2, weight=1)
        panes.add(left, weight=0)

        right = ttk.Frame(panes)
        right.columnconfigure(0, weight=1)
        right.rowconfigure(0, weight=1)
        panes.add(right, weight=1)

        self._build_left_panel(left)
        self._build_tabs(right)

    def _build_left_panel(self, parent: ttk.Frame) -> None:
        dataset = ttk.LabelFrame(parent, text="Dataset", padding=8)
        dataset.grid(row=0, column=0, sticky="ew", pady=(0, 8))
        dataset.columnconfigure(1, weight=1)
        for row, label in enumerate(("active dataset", "rows", "sources", "time range", "source path")):
            ttk.Label(dataset, text=label).grid(row=row, column=0, sticky="w", pady=2)
            var = tk.StringVar(value="-")
            self._dataset_vars[label] = var
            ttk.Entry(dataset, textvariable=var, state="readonly", width=26).grid(
                row=row,
                column=1,
                sticky="ew",
                padx=(8, 0),
                pady=2,
            )

        availability = ttk.LabelFrame(parent, text="Data availability", padding=8)
        availability.grid(row=1, column=0, sticky="ew", pady=(0, 8))
        availability.columnconfigure(1, weight=1)
        for row, label in enumerate(("raw", "GNSS", "tilt", "derived", "notes")):
            ttk.Label(availability, text=label).grid(row=row, column=0, sticky="w", pady=2)
            var = tk.StringVar(value="-")
            self._availability_vars[label] = var
            ttk.Entry(availability, textvariable=var, state="readonly", width=26).grid(
                row=row,
                column=1,
                sticky="ew",
                padx=(8, 0),
                pady=2,
            )

        methods = ttk.LabelFrame(parent, text="Methods", padding=8)
        methods.grid(row=2, column=0, sticky="nsew", pady=(0, 8))
        methods.columnconfigure(0, weight=1)
        methods.rowconfigure(0, weight=1)
        self.methods_canvas = tk.Canvas(methods, bg=PANEL_BG, highlightthickness=1, highlightbackground="#808080", width=330)
        self.methods_canvas.grid(row=0, column=0, sticky="nsew")
        bind_vertical_mousewheel(self.methods_canvas, target=self.methods_canvas)
        scrollbar = ttk.Scrollbar(methods, orient="vertical", command=self.methods_canvas.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.methods_canvas.configure(yscrollcommand=scrollbar.set)
        self.methods_frame = ttk.Frame(self.methods_canvas)
        self._methods_window = self.methods_canvas.create_window((0, 0), window=self.methods_frame, anchor="nw")
        self.methods_frame.bind("<Configure>", self._on_methods_frame_configure)
        self.methods_canvas.bind("<Configure>", self._on_methods_canvas_configure)

        details = ttk.LabelFrame(parent, text="Selected method", padding=8)
        details.grid(row=3, column=0, sticky="ew")
        details.columnconfigure(1, weight=1)
        for row, label in enumerate(("name", "version", "status", "calibrated", "offline records", "params profile")):
            ttk.Label(details, text=label).grid(row=row, column=0, sticky="w", pady=2)
            var = tk.StringVar(value="-")
            self._method_detail_vars[label] = var
            ttk.Entry(details, textvariable=var, state="readonly", width=26).grid(
                row=row,
                column=1,
                sticky="ew",
                padx=(8, 0),
                pady=2,
            )
        buttons = ttk.Frame(details)
        buttons.grid(row=6, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        buttons.columnconfigure(0, weight=1)
        buttons.columnconfigure(1, weight=1)
        self.load_params_btn = ttk.Button(buttons, text="Load params", command=self._handle_load_params, state="disabled")
        self.load_params_btn.grid(row=0, column=0, sticky="ew", padx=(0, 6))
        self.save_params_btn = ttk.Button(buttons, text="Save params", command=self._handle_save_params, state="disabled")
        self.save_params_btn.grid(row=0, column=1, sticky="ew")

    def _build_tabs(self, parent: ttk.Frame) -> None:
        self.tabs = ttk.Notebook(parent)
        self.tabs.grid(row=0, column=0, sticky="nsew")
        self._tab_frames: dict[str, ttk.Frame] = {}
        for tab_name in ("Overview", "Calibration cloud", "Heading vs GNSS", "Dynamics", "Tilt sensitivity", "Magnet/stress"):
            frame = ttk.Frame(self.tabs, padding=8)
            frame.columnconfigure(0, weight=1)
            frame.rowconfigure(0, weight=1)
            self.tabs.add(frame, text=tab_name)
            self._tab_frames[tab_name] = frame

        self._build_overview_tab(self._tab_frames["Overview"])
        for tab_name in ("Calibration cloud", "Heading vs GNSS", "Dynamics", "Tilt sensitivity", "Magnet/stress"):
            self._build_slot_tab(self._tab_frames[tab_name], tab_name)

    def _build_overview_tab(self, parent: ttk.Frame) -> None:
        body = ttk.Frame(parent)
        body.grid(row=0, column=0, sticky="nsew")
        body.columnconfigure(0, weight=1)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(1, weight=1)

        availability_frame = ttk.LabelFrame(body, text="Readiness", padding=8)
        availability_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 6), pady=(0, 8))
        self.overview_availability_text = tk.Text(availability_frame, height=7, wrap="word")
        self.overview_availability_text.pack(fill="both", expand=True)
        self.overview_availability_text.configure(state="disabled")

        slots_frame = ttk.LabelFrame(body, text="Figure slots", padding=8)
        slots_frame.grid(row=0, column=1, sticky="nsew", padx=(6, 0), pady=(0, 8))
        slot_columns = ("key", "tab", "required_data")
        self.figure_slots_tree = ttk.Treeview(slots_frame, columns=slot_columns, show="headings", height=7)
        self.figure_slots_tree.pack(fill="both", expand=True)
        for column in slot_columns:
            self.figure_slots_tree.heading(column, text=column)
            self.figure_slots_tree.column(column, width=140, stretch=True, anchor="w")

        methods_frame = ttk.LabelFrame(body, text="Method readiness", padding=8)
        methods_frame.grid(row=1, column=0, columnspan=2, sticky="nsew")
        method_columns = ("method", "status", "calibrated", "process", "offline_records", "analyze", "rmse", "p95", "offset")
        self.overview_methods_tree = ttk.Treeview(methods_frame, columns=method_columns, show="headings", height=10)
        self.overview_methods_tree.pack(fill="both", expand=True)
        for column in method_columns:
            self.overview_methods_tree.heading(column, text=column)
            width = 92 if column in {"rmse", "p95", "offset"} else 120
            self.overview_methods_tree.column(column, width=width, stretch=True, anchor="w")

    def _build_slot_tab(self, parent: ttk.Frame, tab_name: str) -> None:
        canvas = tk.Canvas(parent, bg=PANEL_BG, highlightthickness=0)
        canvas.grid(row=0, column=0, sticky="nsew")
        bind_vertical_mousewheel(canvas, target=canvas)
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        canvas.configure(yscrollcommand=scrollbar.set)
        content = ttk.Frame(canvas)
        window_id = canvas.create_window((0, 0), window=content, anchor="nw")

        def _content_configure(_event: tk.Event) -> None:
            canvas.configure(scrollregion=canvas.bbox("all"))

        def _canvas_configure(event: tk.Event) -> None:
            canvas.itemconfigure(window_id, width=event.width)

        content.bind("<Configure>", _content_configure)
        canvas.bind("<Configure>", _canvas_configure)

        tab_slots = [slot for slot in self._snapshot.figure_slots if slot.tab == tab_name]
        if not tab_slots:
            ttk.Label(content, text=f"No figure slots configured for {tab_name}.").pack(anchor="w")
            return
        for slot in tab_slots:
            self._build_figure_slot(content, slot)

    def _build_figure_slot(self, parent: ttk.Frame, slot: AnalysisFigureSlot) -> None:
        frame = ttk.LabelFrame(parent, text=slot.title, padding=8)
        frame.pack(fill="x", expand=True, pady=(0, 10))
        frame.columnconfigure(0, weight=1)
        top = ttk.Frame(frame)
        top.grid(row=0, column=0, sticky="ew", pady=(0, 6))
        ttk.Label(top, text=slot.description).pack(side="left")
        open_btn = ttk.Button(top, text="Open", command=lambda key=slot.key: self._handle_open_figure(key), state="disabled")
        open_btn.pack(side="right", padx=(6, 0))
        export_btn = ttk.Button(top, text="Export PNG", command=lambda key=slot.key: self._handle_export_png(key), state="disabled")
        export_btn.pack(side="right")

        figure = Figure(figsize=(7.4, 3.0), dpi=100)
        ax = figure.add_subplot(111)
        ax.set_axis_off()
        ax.text(
            0.5,
            0.55,
            "Figure placeholder",
            ha="center",
            va="center",
            fontsize=13,
            color="#555555",
        )
        ax.text(
            0.5,
            0.38,
            slot.key,
            ha="center",
            va="center",
            fontsize=9,
            color="#777777",
        )
        figure.tight_layout()
        canvas = FigureCanvasTkAgg(figure, master=frame)
        canvas.draw()
        canvas.get_tk_widget().grid(row=1, column=0, sticky="ew")
        self._figures[slot.key] = figure
        self._figure_canvases[slot.key] = canvas
        self._figure_export_buttons[slot.key] = export_btn
        self._figure_open_buttons[slot.key] = open_btn

        required = ", ".join(slot.required_data) if slot.required_data else "none"
        status_var = tk.StringVar(value=f"Required data: {required}")
        self._figure_status_vars[slot.key] = status_var
        ttk.Label(frame, textvariable=status_var).grid(row=2, column=0, sticky="w", pady=(6, 0))

    def _refresh_dataset_summary(self) -> None:
        active = self._snapshot.active_dataset
        self.updated_var.set(f"Snapshot: {self._snapshot.created_at}")
        self._dataset_vars["active dataset"].set(active.name if active is not None else "None")
        self._dataset_vars["rows"].set("0" if active is None else str(active.row_count))
        self._dataset_vars["sources"].set("0" if active is None else str(active.source_count))
        self._dataset_vars["time range"].set("-" if active is None else active.time_range)
        self._dataset_vars["source path"].set("-" if active is None or not active.source_path else active.source_path)

    def _refresh_availability(self) -> None:
        availability = self._snapshot.availability
        self._availability_vars["raw"].set(_available_text(availability.has_raw_magnetometer, availability.raw_sample_count))
        self._availability_vars["GNSS"].set(_available_text(availability.has_gnss_heading, availability.gnss_sample_count))
        self._availability_vars["tilt"].set(_available_text(availability.has_tilt, availability.tilt_sample_count))
        self._availability_vars["derived"].set(_available_text(availability.has_derived_outputs, availability.derived_stream_count))
        self._availability_vars["notes"].set("; ".join(availability.notes) if availability.notes else "Ready")

    def _refresh_methods(self) -> None:
        previous_values = {
            method_id: {name: var.get() for name, var in vars_by_name.items()}
            for method_id, vars_by_name in self._method_vars.items()
        }
        for child in self.methods_frame.winfo_children():
            child.destroy()
        self._method_vars.clear()
        if not self._snapshot.methods:
            if not self._snapshot.raw_records:
                ttk.Label(self.methods_frame, text="No methods loaded").pack(anchor="w", padx=8, pady=8)
                return
        raw_state = raw_method_state(len(self._snapshot.raw_records))
        self._build_method_row(raw_state, previous_values.get(raw_state.method_id, {}))
        for method in self._snapshot.methods:
            self._build_method_row(method, previous_values.get(method.method_id, {}))
        bind_vertical_mousewheel_tree(self.methods_frame, target=self.methods_canvas)

    def _build_method_row(self, method: AnalysisMethodState, previous_values: dict[str, bool] | None = None) -> None:
        row = tk.Frame(
            self.methods_frame,
            bg="#f4f4f4" if method.method_id != self._selected_method_id else "#e7f1fb",
            bd=1,
            relief="solid",
            padx=6,
            pady=6,
        )
        row.pack(fill="x", pady=(0, 8))
        row.bind("<Button-1>", lambda _event, method_id=method.method_id: self._select_method(method_id), add="+")

        header = ttk.Frame(row)
        header.pack(fill="x")
        ttk.Label(header, text=method.name, font=("TkDefaultFont", 10, "bold")).pack(side="left")
        color = STATUS_COLORS.get(method.status, "#808080")
        fg = "#ffffff" if color in {COLOR_GREEN, COLOR_RED} else "#000000"
        tk.Label(header, text=method.status_text, bg=color, fg=fg, padx=6, pady=2).pack(side="right")

        ttk.Label(row, text=f"Version: {method.version}").pack(anchor="w", pady=(4, 0))
        ttk.Label(row, text="calibrated" if method.calibrated else "not calibrated").pack(anchor="w", pady=(2, 0))
        checks = ttk.Frame(row)
        checks.pack(fill="x", pady=(6, 0))
        previous_values = previous_values or {}
        analyze_var = tk.BooleanVar(value=bool(method.analyze_enabled and previous_values.get("analyze", method.analyze_enabled)))
        view_var = tk.BooleanVar(value=bool(method.view_enabled and previous_values.get("view", method.view_enabled)))
        self._method_vars[method.method_id] = {"analyze": analyze_var, "view": view_var}
        ttk.Checkbutton(
            checks,
            text="Analyze",
            variable=analyze_var,
            state="normal" if method.analyze_enabled else "disabled",
            command=self._method_selection_changed,
        ).pack(side="left")
        ttk.Checkbutton(
            checks,
            text="View",
            variable=view_var,
            state="normal" if method.view_enabled else "disabled",
            command=self._method_selection_changed,
        ).pack(side="left", padx=(8, 0))
        if method.warning_count:
            ttk.Label(row, text=f"Warnings: {method.warning_count}").pack(anchor="w", pady=(4, 0))

    def _refresh_selected_method_details(self) -> None:
        method = self._selected_method()
        if method is None:
            for var in self._method_detail_vars.values():
                var.set("-")
            self.load_params_btn.configure(state="disabled")
            self.save_params_btn.configure(state="disabled")
            return
        self._method_detail_vars["name"].set(method.name)
        self._method_detail_vars["version"].set(method.version)
        self._method_detail_vars["status"].set(method.status_text)
        self._method_detail_vars["calibrated"].set("yes" if method.calibrated else "no")
        self._method_detail_vars["offline records"].set(str(method.offline_record_count))
        self._method_detail_vars["params profile"].set(method.params_profile_path or "-")
        if method.method_id == RAW_METHOD_ID:
            self.load_params_btn.configure(state="disabled")
            self.save_params_btn.configure(state="disabled")
        else:
            self.load_params_btn.configure(state="normal")
            self.save_params_btn.configure(state="normal" if method.calibrated else "disabled")

    def _refresh_overview_tables(self) -> None:
        _replace_text(self.overview_availability_text, _availability_summary(self._snapshot.availability))
        for item in self.figure_slots_tree.get_children():
            self.figure_slots_tree.delete(item)
        for slot in self._snapshot.figure_slots:
            self.figure_slots_tree.insert("", "end", values=(slot.key, slot.tab, ", ".join(slot.required_data)))

        for item in self.overview_methods_tree.get_children():
            self.overview_methods_tree.delete(item)
        for method in self._display_methods():
            self.overview_methods_tree.insert(
                "",
                "end",
                values=(
                    method.name,
                    method.status_text,
                    "yes" if method.calibrated else "no",
                    "yes" if method.supports_process else "no",
                    str(method.offline_record_count),
                    "yes" if method.analyze_enabled else "no",
                    "-",
                    "-",
                    "-",
                ),
            )

    def _selected_method(self) -> AnalysisMethodState | None:
        for method in self._display_methods():
            if method.method_id == self._selected_method_id:
                return method
        return None

    def _select_method(self, method_id: str) -> None:
        self._selected_method_id = method_id
        self._refresh_methods()
        self._refresh_selected_method_details()

    def _handle_load_params(self) -> None:
        if self._selected_method_id is None or self._selected_method_id == RAW_METHOD_ID or self._on_load_params is None:
            return
        self._on_load_params(self._selected_method_id)
        self.refresh()

    def _handle_save_params(self) -> None:
        if self._selected_method_id is None or self._selected_method_id == RAW_METHOD_ID or self._on_save_params is None:
            return
        self._on_save_params(self._selected_method_id)
        self.refresh()

    def _method_selection_changed(self) -> None:
        self._render_all_figures()

    def _selected_result_method_ids(self) -> tuple[str, ...]:
        selected: list[str] = []
        for method in self._display_methods():
            vars_by_name = self._method_vars.get(method.method_id)
            if vars_by_name is None:
                if method.analyze_enabled or method.view_enabled:
                    selected.append(method.method_id)
                continue
            if vars_by_name["analyze"].get() or vars_by_name["view"].get():
                selected.append(method.method_id)
        return tuple(selected)

    def _display_methods(self) -> tuple[AnalysisMethodState, ...]:
        return (raw_method_state(len(self._snapshot.raw_records)), *self._snapshot.methods)

    def _render_all_figures(self) -> None:
        try:
            result = build_analysis_result(self._snapshot, method_ids=self._selected_result_method_ids())
        except Exception as exc:
            self._last_result = None
            for key, figure in self._figures.items():
                self._plot_message(figure, "Analysis unavailable", str(exc))
                self._finish_slot(key, "Analysis failed", False)
            return
        self._last_result = result
        self._refresh_overview_method_metrics(result)
        for key, figure in self._figures.items():
            try:
                status, exportable = self._render_slot(key, figure, result)
            except Exception as exc:
                self._plot_message(figure, "Plot failed", str(exc))
                status, exportable = (f"Plot failed: {exc}", False)
            self._finish_slot(key, status, exportable)

    def _refresh_overview_method_metrics(self, result: MagnetometerAnalysisResult) -> None:
        result_by_method_id = {method.method_id: method for method in result.methods}
        for item in self.overview_methods_tree.get_children():
            self.overview_methods_tree.delete(item)
        for method in self._display_methods():
            computed = result_by_method_id.get(method.method_id)
            self.overview_methods_tree.insert(
                "",
                "end",
                values=(
                    method.name,
                    method.status_text,
                    "yes" if method.calibrated else "no",
                    "yes" if method.supports_process else "no",
                    str(method.offline_record_count),
                    "yes" if method.analyze_enabled else "no",
                    "-" if computed is None else _format_metric(computed.metrics.get("aligned_rmse_deg")),
                    "-" if computed is None else _format_metric(computed.metrics.get("aligned_p95_deg")),
                    "-" if computed is None else _format_metric(computed.metrics.get("alignment_offset_deg")),
                ),
            )

    def _finish_slot(self, key: str, status: str, exportable: bool) -> None:
        canvas = self._figure_canvases.get(key)
        if canvas is not None:
            canvas.draw()
        var = self._figure_status_vars.get(key)
        if var is not None:
            var.set(status)
        button = self._figure_export_buttons.get(key)
        if button is not None:
            button.configure(state="normal" if exportable else "disabled")
        open_button = self._figure_open_buttons.get(key)
        if open_button is not None:
            open_button.configure(state="normal" if exportable else "disabled")

    def _handle_export_png(self, key: str) -> None:
        figure = self._figures.get(key)
        if figure is None:
            return
        path = filedialog.asksaveasfilename(
            parent=self,
            title="Export figure",
            defaultextension=".png",
            initialfile=f"{key}.png",
            filetypes=(("PNG image", "*.png"), ("All files", "*.*")),
        )
        if not path:
            return
        figure.savefig(path, dpi=180, bbox_inches="tight")

    def _handle_open_figure(self, key: str) -> None:
        result = self._last_result
        if result is None:
            result = build_analysis_result(self._snapshot, method_ids=self._selected_result_method_ids())
        title = key
        source_figure = self._figures.get(key)
        if source_figure is not None and source_figure.axes:
            title = source_figure.axes[0].get_title() or key

        window = tk.Toplevel(self)
        window.title(title)
        window.geometry("1100x780")
        window.minsize(760, 520)
        window.columnconfigure(0, weight=1)
        window.rowconfigure(1, weight=1)

        toolbar_frame = ttk.Frame(window)
        toolbar_frame.grid(row=0, column=0, sticky="ew")
        figure = Figure(figsize=(10.5, 6.4), dpi=100)
        status, exportable = self._render_slot(key, figure, result)
        canvas = FigureCanvasTkAgg(figure, master=window)
        canvas.get_tk_widget().grid(row=1, column=0, sticky="nsew")
        toolbar = NavigationToolbar2Tk(canvas, toolbar_frame, pack_toolbar=False)
        toolbar.update()
        toolbar.pack(side="left", fill="x", expand=True)
        status_text = status if exportable else f"{status}; no exportable data"
        ttk.Label(window, text=status_text).grid(row=2, column=0, sticky="ew", padx=8, pady=(2, 6))
        canvas.draw()

        window._analysis_canvas = canvas  # type: ignore[attr-defined]
        window._analysis_toolbar = toolbar  # type: ignore[attr-defined]
        window._analysis_figure = figure  # type: ignore[attr-defined]
        prepare_toplevel(window, self)

    def _render_slot(self, key: str, figure: Figure, result: MagnetometerAnalysisResult) -> tuple[str, bool]:
        if key == "raw_xy":
            return self._plot_raw_xy(figure, result)
        if key == "calibrated_xy":
            return self._plot_calibrated_xy(figure, result)
        if key == "radius_time":
            return self._plot_radius_time(figure, result)
        if key == "heading_coverage":
            return self._plot_heading_coverage(figure, result)
        if key == "heading_time":
            return self._plot_heading_time(figure, result)
        if key == "aligned_error_time":
            return self._plot_aligned_error_time(figure, result)
        if key == "error_distribution":
            return self._plot_error_distribution(figure, result)
        if key == "error_vs_heading":
            return self._plot_error_vs_heading(figure, result)
        if key == "delta_heading_error":
            return self._plot_delta_heading_error(figure, result)
        if key == "closure_metrics":
            return self._plot_closure_metrics(figure, result)
        if key == "tilt_timeline":
            return self._plot_tilt_timeline(figure, result)
        if key == "error_vs_alpha":
            return self._plot_error_vs_alpha(figure, result)
        if key == "tilt_bins":
            return self._plot_tilt_bins(figure, result)
        if key == "magnet_cloud_compare":
            return self._plot_magnet_cloud_compare(figure, result)
        if key == "magnet_stress_modes":
            return self._plot_magnet_stress_modes(figure, result)
        self._plot_message(figure, "Not implemented", key)
        return ("No renderer for this figure slot", False)

    def _plot_raw_xy(self, figure: Figure, result: MagnetometerAnalysisResult) -> tuple[str, bool]:
        if not result.raw_x:
            self._plot_message(figure, "Raw XY cloud", "No raw magnetometer samples")
            return ("Raw data unavailable", False)
        figure.clear()
        ax = figure.add_subplot(111)
        xs, ys = _downsample_pair(result.raw_x, result.raw_y)
        ax.scatter(xs, ys, s=6, alpha=0.45, color="#1f77b4")
        ax.axhline(0.0, color="#999999", linewidth=0.8)
        ax.axvline(0.0, color="#999999", linewidth=0.8)
        ax.set_title("Raw magnetometer XY")
        ax.set_xlabel("mx")
        ax.set_ylabel("my")
        ax.set_aspect("equal", adjustable="datalim")
        figure.tight_layout()
        return (f"Raw samples: {len(result.raw_x)}", True)

    def _plot_calibrated_xy(self, figure: Figure, result: MagnetometerAnalysisResult) -> tuple[str, bool]:
        if not result.raw_x and not result.methods:
            self._plot_message(figure, "Calibrated XY cloud", "No raw or derived samples")
            return ("Derived data unavailable", False)
        figure.clear()
        ax = figure.add_subplot(111)
        if result.raw_x:
            xs, ys = _downsample_pair(result.raw_x, result.raw_y)
            ax.scatter(xs, ys, s=5, alpha=0.18, color="#888888", label="raw")
        for method in result.methods:
            xs, ys = _downsample_pair(method.mag_x, method.mag_y)
            ax.scatter(xs, ys, s=6, alpha=0.45, label=method.name)
        ax.axhline(0.0, color="#aaaaaa", linewidth=0.8)
        ax.axvline(0.0, color="#aaaaaa", linewidth=0.8)
        ax.set_title("Raw and calibrated XY overlay")
        ax.set_xlabel("mx")
        ax.set_ylabel("my")
        ax.set_aspect("equal", adjustable="datalim")
        ax.legend(loc="best", fontsize=8)
        figure.tight_layout()
        return (f"Rendered methods: {len(result.methods)}", bool(result.methods or result.raw_x))

    def _plot_radius_time(self, figure: Figure, result: MagnetometerAnalysisResult) -> tuple[str, bool]:
        methods = [method for method in result.methods if method.radius_xy]
        if not methods:
            self._plot_message(figure, "Radius over time", "No derived magnetometer radius data")
            return ("Derived radius data unavailable", False)
        figure.clear()
        ax = figure.add_subplot(111)
        for method in methods:
            times, radii = _downsample_pair(method.times_s, method.radius_xy)
            ax.plot(times, radii, linewidth=1.1, label=f"{method.name} CV={_format_metric(method.metrics.get('radius_cv'))}")
        ax.set_title("Horizontal radius over time")
        ax.set_xlabel("time, s")
        ax.set_ylabel("sqrt(mx^2 + my^2)")
        ax.legend(loc="best", fontsize=8)
        ax.grid(True, alpha=0.25)
        figure.tight_layout()
        return (f"Radius series: {len(methods)}", True)

    def _plot_heading_coverage(self, figure: Figure, result: MagnetometerAnalysisResult) -> tuple[str, bool]:
        if not result.raw_heading_deg:
            self._plot_message(figure, "Heading coverage", "No raw heading samples")
            return ("Raw heading unavailable", False)
        figure.clear()
        ax = figure.add_subplot(111)
        bins = [idx * 15.0 for idx in range(25)]
        ax.hist(result.raw_heading_deg, bins=bins, color="#4c78a8", alpha=0.75, edgecolor="#ffffff")
        ax.set_title("Raw heading coverage")
        ax.set_xlabel("heading, deg")
        ax.set_ylabel("samples")
        ax.set_xlim(0.0, 360.0)
        nonempty = _nonempty_heading_bins(result.raw_heading_deg, 24)
        ax.text(0.99, 0.96, f"non-empty bins: {nonempty}/24", transform=ax.transAxes, ha="right", va="top")
        figure.tight_layout()
        return (f"Heading coverage bins: {nonempty}/24", True)

    def _plot_heading_time(self, figure: Figure, result: MagnetometerAnalysisResult) -> tuple[str, bool]:
        methods = [method for method in result.methods if method.matched_times_s and method.aligned_heading_deg]
        if not result.reference_times_s or not methods:
            self._plot_message(figure, "Heading vs GNSS", "GNSS and derived method headings are required")
            return ("GNSS or derived headings unavailable", False)
        figure.clear()
        ax = figure.add_subplot(111)
        ax.plot(result.reference_times_s, unwrap_degrees(result.reference_heading_deg), color="#111111", linewidth=1.2, label="GNSS")
        for method in methods:
            aligned = _unwrap_like(method.aligned_heading_deg, method.gnss_heading_deg)
            times, values = _downsample_pair(method.matched_times_s, aligned)
            ax.plot(times, values, linewidth=1.1, label=method.name)
        ax.set_title("Aligned magnetometer heading vs GNSS")
        ax.set_xlabel("time, s")
        ax.set_ylabel("unwrapped heading, deg")
        ax.grid(True, alpha=0.25)
        ax.legend(loc="best", fontsize=8)
        figure.tight_layout()
        return (f"Heading methods: {len(methods)}", True)

    def _plot_aligned_error_time(self, figure: Figure, result: MagnetometerAnalysisResult) -> tuple[str, bool]:
        methods = [method for method in result.methods if method.aligned_error_deg]
        if not methods:
            self._plot_message(figure, "Aligned error over time", "No aligned GNSS error series")
            return ("Aligned error unavailable", False)
        figure.clear()
        ax = figure.add_subplot(111)
        for method in methods:
            times, errors = _downsample_pair(method.matched_times_s, method.aligned_error_deg)
            ax.plot(times, errors, linewidth=1.0, label=f"{method.name} RMSE={_format_metric(method.metrics.get('aligned_rmse_deg'))}")
        ax.axhline(0.0, color="#888888", linewidth=0.8)
        ax.set_title("Aligned heading error")
        ax.set_xlabel("time, s")
        ax.set_ylabel("error, deg")
        ax.grid(True, alpha=0.25)
        ax.legend(loc="best", fontsize=8)
        figure.tight_layout()
        return (f"Aligned error methods: {len(methods)}", True)

    def _plot_error_distribution(self, figure: Figure, result: MagnetometerAnalysisResult) -> tuple[str, bool]:
        methods = [method for method in result.methods if method.aligned_error_deg]
        if not methods:
            self._plot_message(figure, "Error distribution", "No aligned GNSS error samples")
            return ("Error distribution unavailable", False)
        figure.clear()
        left, right = figure.subplots(1, 2)
        for method in methods:
            abs_errors = [abs(value) for value in method.aligned_error_deg]
            left.hist(abs_errors, bins=24, alpha=0.45, label=method.name)
        left.set_title("Absolute aligned error")
        left.set_xlabel("abs(error), deg")
        left.set_ylabel("samples")
        left.legend(loc="best", fontsize=7)
        right.boxplot([[abs(value) for value in method.aligned_error_deg] for method in methods], labels=[method.name for method in methods], showfliers=False)
        right.set_title("Boxplot by method")
        right.set_ylabel("abs(error), deg")
        right.tick_params(axis="x", rotation=25, labelsize=7)
        figure.tight_layout()
        return (f"Distribution methods: {len(methods)}", True)

    def _plot_error_vs_heading(self, figure: Figure, result: MagnetometerAnalysisResult) -> tuple[str, bool]:
        methods = [method for method in result.methods if method.aligned_error_deg and method.gnss_heading_deg]
        if not methods:
            self._plot_message(figure, "Error vs GNSS heading", "No aligned heading/error pairs")
            return ("Error vs heading unavailable", False)
        figure.clear()
        ax = figure.add_subplot(111)
        for method in methods:
            xs, ys = _downsample_pair(method.gnss_heading_deg, method.aligned_error_deg)
            ax.scatter(xs, ys, s=7, alpha=0.42, label=method.name)
        ax.axhline(0.0, color="#888888", linewidth=0.8)
        ax.set_title("Aligned error vs GNSS heading")
        ax.set_xlabel("GNSS heading, deg")
        ax.set_ylabel("aligned error, deg")
        ax.set_xlim(0.0, 360.0)
        ax.grid(True, alpha=0.25)
        ax.legend(loc="best", fontsize=8)
        figure.tight_layout()
        return (f"Heading-error methods: {len(methods)}", True)

    def _plot_delta_heading_error(self, figure: Figure, result: MagnetometerAnalysisResult) -> tuple[str, bool]:
        methods = [method for method in result.methods if method.delta_error_deg]
        if not methods:
            self._plot_message(figure, "Delta-heading error", "No consecutive GNSS-aligned heading samples")
            return ("Delta-heading unavailable", False)
        figure.clear()
        ax = figure.add_subplot(111)
        for method in methods:
            times, errors = _downsample_pair(method.delta_times_s, method.delta_error_deg)
            ax.plot(times, errors, linewidth=1.0, label=f"{method.name} RMSE={_format_metric(method.metrics.get('delta_rmse_deg'))}")
        ax.axhline(0.0, color="#888888", linewidth=0.8)
        ax.set_title("Delta-heading error")
        ax.set_xlabel("time, s")
        ax.set_ylabel("delta error, deg")
        ax.grid(True, alpha=0.25)
        ax.legend(loc="best", fontsize=8)
        figure.tight_layout()
        return (f"Delta series: {len(methods)}", True)

    def _plot_closure_metrics(self, figure: Figure, result: MagnetometerAnalysisResult) -> tuple[str, bool]:
        methods = [method for method in result.methods if method.metrics.get("closure_relative_deg") is not None]
        if not methods:
            self._plot_message(figure, "Closure metrics", "No closure metrics; at least two matched samples are required")
            return ("Closure metrics unavailable", False)
        figure.clear()
        left, right = figure.subplots(1, 2)
        names = [method.name for method in methods]
        closure = [float(method.metrics.get("closure_relative_deg") or 0.0) for method in methods]
        delta_rmse = [float(method.metrics.get("delta_rmse_deg") or 0.0) for method in methods]
        positions = list(range(len(methods)))
        left.bar(positions, closure, color="#f58518")
        left.axhline(0.0, color="#777777", linewidth=0.8)
        left.set_title("Relative closure")
        left.set_ylabel("deg")
        left.set_xticks(positions, names, rotation=25, ha="right", fontsize=7)
        right.bar(positions, delta_rmse, color="#54a24b")
        right.set_title("Delta-heading RMSE")
        right.set_ylabel("deg")
        right.set_xticks(positions, names, rotation=25, ha="right", fontsize=7)
        figure.tight_layout()
        return (f"Closure methods: {len(methods)}", True)

    def _plot_tilt_timeline(self, figure: Figure, result: MagnetometerAnalysisResult) -> tuple[str, bool]:
        methods = [method for method in result.methods if method.alpha_deg and method.aligned_error_deg]
        if not result.tilt_times_s or not methods:
            self._plot_message(figure, "Tilt timeline", "Paired accelerometer tilt and aligned GNSS errors are required")
            return ("Tilt timeline unavailable", False)
        figure.clear()
        top, bottom = figure.subplots(2, 1, sharex=True)
        top.plot(result.tilt_times_s, result.tilt_roll_deg, linewidth=0.9, label="roll")
        top.plot(result.tilt_times_s, result.tilt_pitch_deg, linewidth=0.9, label="pitch")
        top.plot(result.tilt_times_s, result.tilt_alpha_deg, linewidth=1.0, label="alpha")
        top.set_ylabel("tilt, deg")
        top.grid(True, alpha=0.25)
        top.legend(loc="best", fontsize=8)
        for method in methods:
            times, errors = _downsample_pair(method.matched_times_s, method.aligned_error_deg)
            bottom.plot(times, errors, linewidth=0.9, label=method.name)
        bottom.axhline(0.0, color="#888888", linewidth=0.8)
        bottom.set_xlabel("time, s")
        bottom.set_ylabel("error, deg")
        bottom.grid(True, alpha=0.25)
        bottom.legend(loc="best", fontsize=8)
        figure.tight_layout()
        return (f"Tilt-aligned methods: {len(methods)}", True)

    def _plot_error_vs_alpha(self, figure: Figure, result: MagnetometerAnalysisResult) -> tuple[str, bool]:
        methods = [method for method in result.methods if method.alpha_deg and method.aligned_error_deg]
        if not methods:
            self._plot_message(figure, "Error vs tilt", "No tilt-aligned error samples")
            return ("Error vs tilt unavailable", False)
        figure.clear()
        ax = figure.add_subplot(111)
        for method in methods:
            pairs = [
                (alpha, abs(error))
                for alpha, error in zip(method.alpha_deg, method.aligned_error_deg)
                if _is_finite(alpha) and _is_finite(error)
            ]
            if not pairs:
                continue
            xs, ys = _downsample_pair(tuple(item[0] for item in pairs), tuple(item[1] for item in pairs))
            corr = _format_metric(method.metrics.get("tilt_alpha_corr_abs"))
            ax.scatter(xs, ys, s=7, alpha=0.42, label=f"{method.name} corr={corr}")
        ax.set_title("Absolute aligned error vs total tilt")
        ax.set_xlabel("alpha = sqrt(roll^2 + pitch^2), deg")
        ax.set_ylabel("abs(error), deg")
        ax.grid(True, alpha=0.25)
        ax.legend(loc="best", fontsize=8)
        figure.tight_layout()
        return (f"Tilt scatter methods: {len(methods)}", True)

    def _plot_tilt_bins(self, figure: Figure, result: MagnetometerAnalysisResult) -> tuple[str, bool]:
        methods = [method for method in result.methods if method.tilt_bins]
        if not methods:
            self._plot_message(figure, "Tilt-bin metrics", "No tilt bins; paired tilt and aligned errors are required")
            return ("Tilt-bin metrics unavailable", False)
        figure.clear()
        ax = figure.add_subplot(111)
        labels = [item.label for item in methods[0].tilt_bins]
        width = 0.8 / max(1, len(methods))
        x_positions = list(range(len(labels)))
        for index, method in enumerate(methods):
            values = [0.0 if item.rmse_deg is None else float(item.rmse_deg) for item in method.tilt_bins]
            offsets = [pos - 0.4 + width / 2.0 + index * width for pos in x_positions]
            ax.bar(offsets, values, width=width, label=method.name)
        ax.set_title("Aligned RMSE by tilt bin")
        ax.set_xlabel("alpha bin")
        ax.set_ylabel("RMSE, deg")
        ax.set_xticks(x_positions, labels)
        ax.legend(loc="best", fontsize=8)
        ax.grid(True, axis="y", alpha=0.25)
        figure.tight_layout()
        return (f"Tilt-bin methods: {len(methods)}", True)

    def _plot_magnet_cloud_compare(self, figure: Figure, result: MagnetometerAnalysisResult) -> tuple[str, bool]:
        clouds = list(result.dataset_clouds)
        if not clouds:
            self._plot_message(figure, "Magnet cloud comparison", "No loaded dataset clouds")
            return ("Dataset cloud comparison unavailable", False)
        figure.clear()
        ax = figure.add_subplot(111)
        for cloud in clouds[:10]:
            xs, ys = _downsample_pair(cloud.mag_x, cloud.mag_y)
            label = f"{cloud.name} [{cloud.magnet_label}]"
            if cloud.is_active:
                label = f"* {label}"
            ax.scatter(xs, ys, s=6, alpha=0.35, label=label)
        ax.axhline(0.0, color="#999999", linewidth=0.8)
        ax.axvline(0.0, color="#999999", linewidth=0.8)
        ax.set_title("Loaded raw clouds: magnet/no-magnet comparison")
        ax.set_xlabel("mx")
        ax.set_ylabel("my")
        ax.set_aspect("equal", adjustable="datalim")
        ax.legend(loc="best", fontsize=7)
        figure.tight_layout()
        with_count = sum(1 for cloud in clouds if cloud.magnet_label == "with magnet")
        without_count = sum(1 for cloud in clouds if cloud.magnet_label == "without magnet")
        return (f"Clouds: {len(clouds)}; with magnet: {with_count}; without: {without_count}", True)

    def _plot_magnet_stress_modes(self, figure: Figure, result: MagnetometerAnalysisResult) -> tuple[str, bool]:
        rows = list(result.stress_rows)
        if not rows:
            self._plot_message(figure, "Stress modes A-D", "No method metrics for stress mode table")
            return ("Stress modes unavailable", False)
        figure.clear()
        ax = figure.add_subplot(111)
        ax.set_axis_off()
        lines = ["method | mode | RMSE | P95 | N"]
        for row in rows[:16]:
            lines.append(
                " | ".join(
                    (
                        str(row.get("method", "-"))[:26],
                        str(row.get("mode", "-")),
                        _format_metric(row.get("rmse_deg")),
                        _format_metric(row.get("p95_deg")),
                        str(row.get("matched_count", "-")),
                    )
                )
            )
        ax.text(0.01, 0.98, "\n".join(lines), transform=ax.transAxes, va="top", ha="left", family="monospace", fontsize=8)
        ax.set_title("Magnet/stress modes from loaded calibration profiles")
        figure.tight_layout()
        return (f"Stress rows: {len(rows)}", True)

    def _plot_message(self, figure: Figure, title: str, message: str) -> None:
        figure.clear()
        ax = figure.add_subplot(111)
        ax.set_axis_off()
        ax.text(0.5, 0.62, title, ha="center", va="center", fontsize=13, color="#555555")
        ax.text(0.5, 0.42, message, ha="center", va="center", fontsize=9, color="#777777", wrap=True)
        figure.tight_layout()

    def _on_methods_frame_configure(self, _event: tk.Event) -> None:
        self.methods_canvas.configure(scrollregion=self.methods_canvas.bbox("all"))

    def _on_methods_canvas_configure(self, event: tk.Event) -> None:
        self.methods_canvas.itemconfigure(self._methods_window, width=event.width)


def _available_text(available: bool, count: int) -> str:
    return f"available ({count})" if available else "unavailable"


def _availability_summary(availability: AnalysisDataAvailability) -> str:
    lines = [
        f"Active dataset: {'yes' if availability.has_active_dataset else 'no'}",
        f"Raw magnetometer: {_available_text(availability.has_raw_magnetometer, availability.raw_sample_count)}",
        f"GNSS/reference heading: {_available_text(availability.has_gnss_heading, availability.gnss_sample_count)}",
        f"Tilt: {_available_text(availability.has_tilt, availability.tilt_sample_count)}",
        f"Derived outputs: {_available_text(availability.has_derived_outputs, availability.derived_stream_count)}",
    ]
    if availability.notes:
        lines.append("")
        lines.extend(availability.notes)
    return "\n".join(lines)


def _replace_text(widget: tk.Text, text: str) -> None:
    widget.configure(state="normal")
    widget.delete("1.0", "end")
    widget.insert("1.0", text)
    widget.configure(state="disabled")


def _downsample_pair(xs: tuple[float, ...] | list[float], ys: tuple[float, ...] | list[float], *, max_points: int = 3500) -> tuple[tuple[float, ...], tuple[float, ...]]:
    if len(xs) != len(ys):
        count = min(len(xs), len(ys))
        xs = xs[:count]
        ys = ys[:count]
    if len(xs) <= max_points:
        return (tuple(xs), tuple(ys))
    step = max(1, len(xs) // max_points)
    return (tuple(xs[::step]), tuple(ys[::step]))


def _nonempty_heading_bins(headings: tuple[float, ...], bin_count: int) -> int:
    if not headings or bin_count <= 0:
        return 0
    used: set[int] = set()
    for heading in headings:
        if not _is_finite(heading):
            continue
        index = int((float(heading) % 360.0) / 360.0 * bin_count)
        used.add(min(bin_count - 1, max(0, index)))
    return len(used)


def _unwrap_like(values: tuple[float, ...], reference_values: tuple[float, ...]) -> tuple[float, ...]:
    if not values:
        return ()
    unwrapped = unwrap_degrees(values)
    if not reference_values:
        return tuple(unwrapped)
    reference_unwrapped = unwrap_degrees(reference_values)
    if not reference_unwrapped:
        return tuple(unwrapped)
    shift = round((unwrapped[0] - reference_unwrapped[0]) / 360.0) * 360.0
    return tuple(value - shift for value in unwrapped)


def _format_metric(value: object) -> str:
    if value is None:
        return "-"
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return str(value)
    if not _is_finite(numeric):
        return "-"
    return f"{numeric:.3g}"


def _is_finite(value: object) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False
