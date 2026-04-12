from __future__ import annotations

import math
import time
import tkinter as tk
from tkinter import ttk
from typing import Callable

from app.magnetometer_dataset import SampleRecord
from app.method_card import MethodCard
from app.source_card import SourceCard, resolve_source_card_status
from app.styles import COLOR_GREEN, COLOR_RED, COLOR_YELLOW, PANEL_BG
from app.tk_utils import bind_vertical_mousewheel, bind_vertical_mousewheel_tree

PROJECTION_3D = "3D"
PROJECTION_XY = "XY"
PROJECTION_XZ = "XZ"
PROJECTION_YZ = "YZ"
PROJECTION_MODES = (PROJECTION_3D, PROJECTION_XY, PROJECTION_XZ, PROJECTION_YZ)
BUILTIN_SOURCE_DEFS = (
    ("raw_magnetometer", "Raw Magnetometer", "Built-in source"),
    ("raw_heading", "Raw Heading", "Built-in source"),
)

_ISO_X = 0.8660254037844386
_ISO_Y = 0.35
DEFAULT_3D_YAW_DEG = 45.0
DEFAULT_3D_PITCH_DEG = 35.0
MAX_CLOUD_POINTS = 900
MIN_CLOUD_POINTS_PER_STREAM = 120
LIVE_REDRAW_INTERVAL_S = 1.0 / 24.0
DATA_TABLE_FLUSH_DELAY_MS = 80
DATA_TABLE_FLUSH_BATCH = 240


def compute_raw_heading_deg(mx: float, my: float) -> float | None:
    if not math.isfinite(mx) or not math.isfinite(my):
        return None
    if math.isclose(mx, 0.0, abs_tol=1e-9) and math.isclose(my, 0.0, abs_tol=1e-9):
        return None
    return (math.degrees(math.atan2(mx, my)) + 360.0) % 360.0


def project_magnetometer_point(projection: str, mx: float, my: float, mz: float) -> tuple[float, float]:
    if projection == PROJECTION_XY:
        return (mx, my)
    if projection == PROJECTION_XZ:
        return (mx, mz)
    if projection == PROJECTION_YZ:
        return (my, mz)
    return ((mx - my) * _ISO_X, mz + (mx + my) * _ISO_Y)


def rotate_magnetometer_point(
    mx: float,
    my: float,
    mz: float,
    *,
    yaw_deg: float,
    pitch_deg: float,
) -> tuple[float, float, float]:
    yaw_rad = math.radians(yaw_deg)
    pitch_rad = math.radians(pitch_deg)

    x_yaw = mx * math.cos(yaw_rad) - my * math.sin(yaw_rad)
    y_yaw = mx * math.sin(yaw_rad) + my * math.cos(yaw_rad)

    y_pitch = y_yaw * math.cos(pitch_rad) - mz * math.sin(pitch_rad)
    z_pitch = y_yaw * math.sin(pitch_rad) + mz * math.cos(pitch_rad)
    return (x_yaw, y_pitch, z_pitch)


def project_rotated_magnetometer_point(
    mx: float,
    my: float,
    mz: float,
    *,
    yaw_deg: float,
    pitch_deg: float,
) -> tuple[float, float]:
    x_rot, _y_rot, z_rot = rotate_magnetometer_point(
        mx,
        my,
        mz,
        yaw_deg=yaw_deg,
        pitch_deg=pitch_deg,
    )
    return (x_rot, z_rot)


class MagnetometerTab(ttk.Frame):
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
        on_method_show_change: Callable[[str, bool], None] | None = None,
        on_remove_method: Callable[[str], None] | None = None,
        on_enable_method_realtime: Callable[[str], None] | None = None,
        on_disable_method_realtime: Callable[[str], None] | None = None,
        on_method_record_change: Callable[[str, bool], None] | None = None,
        on_select_primary_heading: Callable[[str], None] | None = None,
        on_export_metrics: Callable[[], None] | None = None,
    ) -> None:
        super().__init__(master, padding=6)

        self._on_start_record = on_start_record
        self._on_stop_record = on_stop_record
        self._on_load_csv = on_load_csv
        self._on_load_multiple = on_load_multiple
        self._on_save_current = on_save_current
        self._on_save_as = on_save_as
        self._on_concatenate = on_concatenate
        self._on_trim_selection = on_trim_selection
        self._on_delete_selection = on_delete_selection
        self._on_select_dataset = on_select_dataset
        self._on_select_source = on_select_source
        self._on_source_show_change = on_source_show_change
        self._on_source_record_change = on_source_record_change
        self._on_add_plugin = on_add_plugin
        self._on_select_method = on_select_method
        self._on_open_method_info = on_open_method_info
        self._on_calibrate_method = on_calibrate_method
        self._on_load_method_params = on_load_method_params
        self._on_save_method_params = on_save_method_params
        self._on_method_show_change = on_method_show_change
        self._on_remove_method = on_remove_method
        self._on_enable_method_realtime = on_enable_method_realtime
        self._on_disable_method_realtime = on_disable_method_realtime
        self._on_method_record_change = on_method_record_change
        self._on_select_primary_heading = on_select_primary_heading
        self._on_export_metrics = on_export_metrics
        self.columnconfigure(0, weight=22)
        self.columnconfigure(1, weight=56)
        self.columnconfigure(2, weight=22)
        self.rowconfigure(0, weight=3)
        self.rowconfigure(1, weight=2)
        self._top_panel_height = 420
        self._log_max_lines = 500
        self._log_lines = 0
        self._log_filters: dict[str, tk.BooleanVar] = {}
        self._filter_desc: dict[str, str] = {}
        self._menu_tooltip: _MenuTooltip | None = None
        self._current_heading_deg: float | None = None
        self._current_mag_vector: tuple[float, float, float] | None = None
        self._projection_mode = PROJECTION_XY
        self._projection_buttons: dict[str, ttk.Button] = {}
        self._view_scale = 1.0
        self._view_pan = (0.0, 0.0)
        self._pan_start: tuple[float, float] | None = None
        self._pan_origin: tuple[float, float] | None = None
        self._rotate_start: tuple[float, float] | None = None
        self._rotate_origin: tuple[float, float] | None = None
        self._fit_radius = 1.0
        self._view_rotation_yaw_deg = DEFAULT_3D_YAW_DEG
        self._view_rotation_pitch_deg = DEFAULT_3D_PITCH_DEG
        self._recording_active = False
        self._dataset_row_count = 0
        self._dataset_choice_labels: list[str] = []
        self._dataset_records: list[SampleRecord] = []
        self._dataset_records_by_stream: dict[str, list[SampleRecord]] = {}
        self._dataset_cloud_max_radius = 1.0
        self._method_dataset_clouds: dict[str, list[SampleRecord]] = {}
        self._method_dataset_cloud_max_radius = 1.0
        self._pending_dataset_rows: list[tuple[int, SampleRecord]] = []
        self._dataset_row_flush_after_id: str | None = None
        self._last_live_redraw_s = 0.0
        self._source_cards: dict[str, SourceCard] = {}
        self._method_cards: dict[str, MethodCard] = {}
        self._method_record_vars: dict[str, tk.BooleanVar] = {}
        self._method_record_checks: dict[str, ttk.Checkbutton] = {}
        self._method_titles: dict[str, str] = {}
        self._method_stream_ids: dict[str, str] = {}
        self._selected_method_id: str | None = None
        self._derived_streams: dict[str, dict[str, object]] = {}
        self._primary_heading_options: list[tuple[str, str]] = []

        self._build()

    def _build(self) -> None:
        self._build_sources_panel()
        self._build_view_panel()
        self._build_control_panel()
        self._build_bottom_notebook()

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
            self._add_source_card(source_id, title, source_type)

    def _build_view_panel(self) -> None:
        center = ttk.LabelFrame(self, text="Magnetometer View", padding=8)
        center.grid(row=0, column=1, sticky="nsew", padx=8, pady=(0, 6))
        center.columnconfigure(0, weight=1)
        center.rowconfigure(1, weight=1)

        toolbar = ttk.Frame(center)
        toolbar.grid(row=0, column=0, sticky="ew", pady=(0, 8))

        for label in PROJECTION_MODES:
            btn = ttk.Button(toolbar, text=label, command=lambda mode=label: self._set_projection(mode))
            btn.pack(side="left", padx=(0, 6))
            self._projection_buttons[label] = btn

        ttk.Button(toolbar, text="Fit", command=self._fit_view).pack(side="left", padx=(0, 6))
        ttk.Button(toolbar, text="Reset", command=self._reset_view).pack(side="left", padx=(0, 6))

        self.auto_fit_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            toolbar,
            text="Auto-fit",
            variable=self.auto_fit_var,
            command=self._handle_auto_fit_toggle,
        ).pack(side="right")

        self.view_canvas = tk.Canvas(
            center,
            bg=PANEL_BG,
            highlightthickness=1,
            highlightbackground="#606060",
            height=self._top_panel_height,
        )
        self.view_canvas.grid(row=1, column=0, sticky="nsew")
        self.view_canvas.bind("<Configure>", self._redraw_view)
        self.view_canvas.bind("<ButtonPress-3>", self._on_pan_start)
        self.view_canvas.bind("<B3-Motion>", self._on_pan_drag)
        self.view_canvas.bind("<ButtonRelease-3>", self._on_pan_end)
        self.view_canvas.bind("<ButtonPress-1>", self._on_rotate_start)
        self.view_canvas.bind("<B1-Motion>", self._on_rotate_drag)
        self.view_canvas.bind("<ButtonRelease-1>", self._on_rotate_end)
        self.view_canvas.bind("<MouseWheel>", self._on_zoom)
        self.view_canvas.bind("<Button-4>", self._on_zoom)
        self.view_canvas.bind("<Button-5>", self._on_zoom)

        self._update_projection_buttons()

    def _build_control_panel(self) -> None:
        right = ttk.LabelFrame(self, text="Magnetometer Control", padding=8)
        right.grid(row=0, column=2, sticky="nsew", padx=(8, 0), pady=(0, 6))
        right.columnconfigure(0, weight=1)
        right.rowconfigure(0, weight=1)

        scroll_host = ttk.Frame(right)
        scroll_host.grid(row=0, column=0, sticky="nsew")
        scroll_host.columnconfigure(0, weight=1)
        scroll_host.rowconfigure(0, weight=1)

        self.control_canvas = tk.Canvas(
            scroll_host,
            bg=PANEL_BG,
            highlightthickness=0,
            height=self._top_panel_height,
        )
        self.control_canvas.grid(row=0, column=0, sticky="nsew")
        bind_vertical_mousewheel(self.control_canvas, target=self.control_canvas)

        yscroll = ttk.Scrollbar(scroll_host, orient="vertical", command=self.control_canvas.yview)
        yscroll.grid(row=0, column=1, sticky="ns")
        self.control_canvas.configure(yscrollcommand=yscroll.set)

        self.control_frame = ttk.Frame(self.control_canvas)
        self._control_window = self.control_canvas.create_window((0, 0), window=self.control_frame, anchor="nw")
        bind_vertical_mousewheel(self.control_frame, target=self.control_canvas)
        self.control_frame.bind("<Configure>", self._on_control_frame_configure)
        self.control_canvas.bind("<Configure>", self._on_control_canvas_configure)

        self._build_current_data_section(self.control_frame).grid(row=0, column=0, sticky="ew")
        self._build_view_options_section(self.control_frame).grid(row=1, column=0, sticky="ew", pady=(8, 0))
        self._build_dataset_actions_section(self.control_frame).grid(row=2, column=0, sticky="ew", pady=(8, 0))
        self._build_selected_method_section(self.control_frame).grid(row=3, column=0, sticky="ew", pady=(8, 0))
        self._build_output_routing_section(self.control_frame).grid(row=4, column=0, sticky="ew", pady=(8, 0))
        bind_vertical_mousewheel_tree(self.control_frame, target=self.control_canvas)

    def _build_bottom_notebook(self) -> None:
        self.bottom_nb = ttk.Notebook(self)
        self.bottom_nb.grid(row=1, column=0, columnspan=3, sticky="nsew")
        self.bottom_nb.bind("<<NotebookTabChanged>>", self._handle_bottom_tab_change, add="+")

        self._build_log_tab()
        self._build_data_tab()
        self._build_metrics_tab()

    def _build_current_data_section(self, parent: tk.Widget) -> ttk.LabelFrame:
        frm = ttk.LabelFrame(parent, text="Current Data", padding=8)
        frm.columnconfigure(1, weight=1)

        labels = (
            "MCU ts",
            "PC rx ts",
            "PC est ts",
            "mx",
            "my",
            "mz",
            "|m|",
            "raw heading",
            "selected output heading",
            "selected source",
            "dataset status",
        )
        self.current_data_vars: dict[str, tk.StringVar] = {}
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
            ("show raw points", True),
            ("show corrected points", True),
            ("show current point", True),
            ("show raw heading", True),
            ("show GNSS heading", False),
            ("show derived headings", True),
            ("show trails", False),
            ("auto fit on load", True),
        )
        self.view_option_vars: dict[str, tk.BooleanVar] = {}
        for row, (label, value) in enumerate(options):
            var = tk.BooleanVar(value=value)
            self.view_option_vars[label] = var
            state = "normal" if label in {
                "show raw points",
                "show corrected points",
                "show current point",
                "show raw heading",
                "show derived headings",
                "auto fit on load",
            } else "disabled"
            ttk.Checkbutton(frm, text=label, variable=var, state=state, command=self._handle_view_option_change).grid(
                row=row,
                column=0,
                sticky="w",
                pady=2,
            )
        return frm

    def _build_dataset_actions_section(self, parent: tk.Widget) -> ttk.LabelFrame:
        frm = ttk.LabelFrame(parent, text="Dataset Actions", padding=8)
        frm.columnconfigure(0, weight=1)
        frm.columnconfigure(1, weight=1)

        self.start_record_btn = ttk.Button(frm, text="Start record", command=self._handle_start_record)
        self.start_record_btn.grid(row=0, column=0, sticky="ew", padx=(0, 6), pady=3)
        self.stop_record_btn = ttk.Button(frm, text="Stop record", command=self._handle_stop_record)
        self.stop_record_btn.grid(row=0, column=1, sticky="ew", pady=3)

        self.load_csv_btn = ttk.Button(frm, text="Load CSV", command=self._handle_load_csv)
        self.load_csv_btn.grid(row=1, column=0, sticky="ew", padx=(0, 6), pady=3)
        self.load_multiple_btn = ttk.Button(frm, text="Load multiple", command=self._handle_load_multiple)
        self.load_multiple_btn.grid(row=1, column=1, sticky="ew", pady=3)

        self.save_current_btn = ttk.Button(frm, text="Save current", command=self._handle_save_current)
        self.save_current_btn.grid(row=2, column=0, sticky="ew", padx=(0, 6), pady=3)
        self.save_as_btn = ttk.Button(frm, text="Save as", command=self._handle_save_as)
        self.save_as_btn.grid(row=2, column=1, sticky="ew", pady=3)

        self.concatenate_btn = ttk.Button(frm, text="Concatenate", command=self._handle_concatenate)
        self.concatenate_btn.grid(row=3, column=0, sticky="ew", padx=(0, 6), pady=3)
        self.trim_selection_btn = ttk.Button(frm, text="Trim selection", command=self._handle_trim_selection)
        self.trim_selection_btn.grid(row=3, column=1, sticky="ew", pady=3)
        self.delete_selection_btn = ttk.Button(frm, text="Delete selection", command=self._handle_delete_selection)
        self.delete_selection_btn.grid(row=4, column=0, sticky="ew", padx=(0, 6), pady=3)

        summary = ttk.Frame(frm)
        summary.grid(row=5, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        summary.columnconfigure(1, weight=1)

        self.dataset_summary_vars: dict[str, tk.StringVar] = {}
        for row, label in enumerate(("active dataset name", "number of rows", "source count", "time range")):
            ttk.Label(summary, text=label).grid(row=row, column=0, sticky="w", pady=2)
            default = "0" if label in {"number of rows", "source count"} else "—"
            var = tk.StringVar(value=default)
            self.dataset_summary_vars[label] = var
            if label == "active dataset name":
                self.dataset_name_combo = ttk.Combobox(summary, textvariable=var, state="disabled", width=18)
                self.dataset_name_combo.grid(
                    row=row,
                    column=1,
                    sticky="ew",
                    padx=(8, 0),
                    pady=2,
                )
                self.dataset_name_combo.bind("<<ComboboxSelected>>", self._handle_dataset_selection)
            else:
                ttk.Entry(summary, textvariable=var, state="readonly", width=18).grid(
                    row=row,
                    column=1,
                    sticky="ew",
                    padx=(8, 0),
                    pady=2,
                )
        return frm

    def _build_selected_method_section(self, parent: tk.Widget) -> ttk.LabelFrame:
        frm = ttk.LabelFrame(parent, text="Selected Method", padding=8)
        frm.columnconfigure(1, weight=1)

        self.selected_method_vars: dict[str, tk.StringVar] = {}
        for row, label in enumerate(("method name", "method version", "method path", "status", "supported capabilities")):
            ttk.Label(frm, text=label).grid(row=row, column=0, sticky="w", pady=2)
            var = tk.StringVar(value="-")
            self.selected_method_vars[label] = var
            ttk.Entry(frm, textvariable=var, state="readonly", width=18).grid(
                row=row,
                column=1,
                sticky="ew",
                padx=(8, 0),
                pady=2,
            )

        btns = ttk.Frame(frm)
        btns.grid(row=5, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        btns.columnconfigure(0, weight=1)
        btns.columnconfigure(1, weight=1)
        self.selected_method_calibrate_btn = ttk.Button(
            btns,
            text="Calibrate",
            command=self._handle_selected_method_calibrate,
            state="disabled",
        )
        self.selected_method_calibrate_btn.grid(row=0, column=0, sticky="ew", padx=(0, 6), pady=3)
        self.selected_method_load_btn = ttk.Button(
            btns,
            text="Load params",
            command=self._handle_selected_method_load_params,
            state="disabled",
        )
        self.selected_method_load_btn.grid(row=0, column=1, sticky="ew", pady=3)
        self.selected_method_save_btn = ttk.Button(
            btns,
            text="Save params",
            command=self._handle_selected_method_save_params,
            state="disabled",
        )
        self.selected_method_save_btn.grid(row=1, column=0, sticky="ew", padx=(0, 6), pady=3)
        self.selected_method_realtime_on_btn = ttk.Button(
            btns,
            text="Enable realtime",
            command=self._handle_selected_method_enable_realtime,
            state="disabled",
        )
        self.selected_method_realtime_on_btn.grid(row=1, column=1, sticky="ew", pady=3)
        self.selected_method_realtime_off_btn = ttk.Button(
            btns,
            text="Disable realtime",
            command=self._handle_selected_method_disable_realtime,
            state="disabled",
        )
        self.selected_method_realtime_off_btn.grid(row=2, column=0, sticky="ew", padx=(0, 6), pady=3)
        return frm

    def _build_output_routing_section(self, parent: tk.Widget) -> ttk.LabelFrame:
        frm = ttk.LabelFrame(parent, text="Output Routing", padding=8)
        frm.columnconfigure(1, weight=1)

        ttk.Label(frm, text="visible heading streams").grid(row=0, column=0, sticky="w", pady=2)
        self.visible_heading_streams_var = tk.StringVar(value="Raw Heading")
        ttk.Entry(frm, textvariable=self.visible_heading_streams_var, state="readonly", width=18).grid(
            row=0,
            column=1,
            sticky="ew",
            padx=(8, 0),
            pady=2,
        )

        ttk.Label(frm, text="primary heading stream").grid(row=1, column=0, sticky="w", pady=2)
        self.primary_heading_var = tk.StringVar(value="Raw Heading")
        self.primary_heading_combo = ttk.Combobox(frm, textvariable=self.primary_heading_var, state="disabled", width=18)
        self.primary_heading_combo.grid(
            row=1,
            column=1,
            sticky="ew",
            padx=(8, 0),
            pady=2,
        )
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
        bind_vertical_mousewheel(self.log_text)
        self.log_text.tag_configure("tx", foreground="#001a66")
        self.log_text.configure(state="disabled")

        yscroll = ttk.Scrollbar(body, orient="vertical", command=self.log_text.yview)
        yscroll.pack(side="right", fill="y")
        self.log_text.configure(yscrollcommand=yscroll.set)

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
            "mag_x",
            "mag_y",
            "mag_z",
            "heading",
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
        frm.rowconfigure(1, weight=1)

        topbar = ttk.Frame(frm)
        topbar.grid(row=0, column=0, sticky="ew", pady=(0, 6))
        topbar.columnconfigure(0, weight=1)

        self.metrics_status_var = tk.StringVar(value="Select a dataset and a method to compute metrics.")
        ttk.Label(topbar, textvariable=self.metrics_status_var).grid(row=0, column=0, sticky="w")
        self.export_metrics_btn = ttk.Button(topbar, text="Export CSV", command=self._handle_export_metrics, state="disabled")
        self.export_metrics_btn.grid(row=0, column=1, sticky="e")

        columns = ("metric", "value", "units", "status", "notes")
        tree = ttk.Treeview(frm, columns=columns, show="headings", height=8)
        self.metrics_tree = tree
        tree.grid(row=1, column=0, sticky="nsew")
        bind_vertical_mousewheel(tree)
        headings = {
            "metric": "metric",
            "value": "value",
            "units": "units",
            "status": "status",
            "notes": "notes",
        }
        widths = {
            "metric": 180,
            "value": 120,
            "units": 90,
            "status": 110,
            "notes": 360,
        }
        for column in columns:
            tree.heading(column, text=headings[column])
            tree.column(column, width=widths[column], stretch=True, anchor="w")

        yscroll = ttk.Scrollbar(frm, orient="vertical", command=tree.yview)
        yscroll.grid(row=1, column=1, sticky="ns")
        xscroll = ttk.Scrollbar(frm, orient="horizontal", command=tree.xview)
        xscroll.grid(row=2, column=0, sticky="ew")
        tree.configure(yscrollcommand=yscroll.set, xscrollcommand=xscroll.set)

    def _add_source_card(self, source_id: str, title: str, source_type: str) -> None:
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

    def set_method_states(self, method_states: dict[str, dict[str, object]], selected_method_id: str | None) -> None:
        self._selected_method_id = selected_method_id
        stale_ids = set(self._method_cards) - set(method_states)
        for method_id in stale_ids:
            self._method_cards[method_id].destroy()
            del self._method_cards[method_id]
            self._derived_streams.pop(method_id, None)
            self._method_dataset_clouds.pop(method_id, None)
            self._method_record_vars.pop(method_id, None)
            check = self._method_record_checks.pop(method_id, None)
            if check is not None:
                check.destroy()
            self._method_titles.pop(method_id, None)
            self._method_stream_ids.pop(method_id, None)
        if stale_ids:
            self._recompute_method_cloud_max_radius()

        for method_id, state in method_states.items():
            self._method_titles[method_id] = str(state.get("name", method_id))
            self._method_stream_ids[method_id] = str(state.get("stream_id", f"derived_{method_id}"))
            live_output = state.get("live_output")
            if isinstance(live_output, dict):
                self._derived_streams[method_id] = {
                    "title": self._method_titles[method_id],
                    "stream_id": self._method_stream_ids[method_id],
                    "mx": live_output.get("mag_x"),
                    "my": live_output.get("mag_y"),
                    "mz": live_output.get("mag_z"),
                    "heading": live_output.get("heading"),
                    "show": bool(state.get("show", False)),
                }
            else:
                self._derived_streams.pop(method_id, None)
            card = self._method_cards.get(method_id)
            if card is None:
                card = MethodCard(
                    self.method_cards_container,
                    method_id=method_id,
                    title=str(state.get("name", method_id)),
                    version=str(state.get("version", "-")),
                    on_select=self._handle_method_selection,
                    on_info=self._handle_open_method_info,
                    on_calibrate=self._handle_method_calibrate,
                    on_load_params=self._handle_method_load_params,
                    on_save_params=self._handle_method_save_params,
                    on_show_change=self._handle_method_show_change,
                    on_record_change=self._handle_method_record_change,
                    on_toggle_realtime=self._handle_method_toggle_realtime,
                    on_remove=self._handle_method_remove,
                )
                card.pack(fill="x", pady=(0, 8))
                bind_vertical_mousewheel_tree(card, target=self.cards_canvas)
                self._method_cards[method_id] = card
            card.set_status(
                str(state.get("status", "partial")),
                text=str(state.get("status_text", "")).strip() or None,
            )
            card.set_progress(float(state.get("progress", 0.0)))
            card.set_selected(method_id == selected_method_id)
            card.set_show(bool(state.get("show", False)))
            card.set_show_enabled(bool(state.get("show_enabled", False)))
            card.set_record(bool(state.get("record", False)))
            card.set_record_enabled(bool(state.get("can_record", False)))
            card.set_calibrate_enabled(bool(state.get("can_calibrate", False)))
            card.set_load_params_enabled(bool(state.get("can_load_params", False)))
            card.set_save_params_enabled(bool(state.get("can_save_params", False)))
            card.set_realtime_state(
                enabled=bool(state.get("realtime_enabled", False)),
                can_enable=bool(state.get("can_enable_realtime", False)),
                can_disable=bool(state.get("can_disable_realtime", False)),
            )

        self._rebuild_method_record_controls(method_states)
        self._update_heading_stream_summary()
        self._redraw_view()

    def set_method_dataset_clouds(self, method_clouds: dict[str, list[SampleRecord]]) -> None:
        self._method_dataset_clouds = {
            method_id: list(records)
            for method_id, records in method_clouds.items()
        }
        self._recompute_method_cloud_max_radius()
        if self.auto_fit_var.get():
            self._sync_auto_fit_radius()
        self._redraw_view()

    def update_selected_method_details(self, *, name: str, version: str, path: str, status: str, capabilities: str) -> None:
        self.selected_method_vars["method name"].set(name)
        self.selected_method_vars["method version"].set(version)
        self.selected_method_vars["method path"].set(path)
        self.selected_method_vars["status"].set(status)
        self.selected_method_vars["supported capabilities"].set(capabilities)

    def set_selected_method_actions(
        self,
        *,
        can_calibrate: bool,
        can_load_params: bool,
        can_save_params: bool,
        can_enable_realtime: bool,
        can_disable_realtime: bool,
    ) -> None:
        self.selected_method_calibrate_btn.configure(state="normal" if can_calibrate else "disabled")
        self.selected_method_load_btn.configure(state="normal" if can_load_params else "disabled")
        self.selected_method_save_btn.configure(state="normal" if can_save_params else "disabled")
        self.selected_method_realtime_on_btn.configure(state="normal" if can_enable_realtime else "disabled")
        self.selected_method_realtime_off_btn.configure(state="normal" if can_disable_realtime else "disabled")

    def set_heading_routing(self, *, stream_choices: list[tuple[str, str]], primary_stream_id: str) -> None:
        self._primary_heading_options = list(stream_choices)
        labels = [label for _, label in self._primary_heading_options]
        self.primary_heading_combo.configure(values=tuple(labels))
        if not labels:
            self.primary_heading_var.set("None")
            self.primary_heading_combo.configure(state="disabled")
            return

        selected_index = 0
        for idx, (stream_id, _label) in enumerate(self._primary_heading_options):
            if stream_id == primary_stream_id:
                selected_index = idx
                break
        self.primary_heading_var.set(labels[selected_index])
        self.primary_heading_combo.current(selected_index)
        self.primary_heading_combo.configure(state="readonly")

    def set_primary_output_display(self, *, heading: float | None, source_label: str) -> None:
        self.current_data_vars["selected output heading"].set("—" if heading is None else f"{heading:.1f}°")
        self.current_data_vars["selected source"].set(source_label)

    def _rebuild_method_record_controls(self, method_states: dict[str, dict[str, object]]) -> None:
        stale_ids = set(self._method_record_checks) - set(method_states)
        for method_id in stale_ids:
            self._method_record_checks[method_id].destroy()
            del self._method_record_checks[method_id]
            self._method_record_vars.pop(method_id, None)

        if not method_states:
            for child in self.record_streams_frame.winfo_children():
                child.destroy()
            ttk.Label(self.record_streams_frame, text="No derived streams").grid(row=0, column=0, sticky="w")
            return

        for child in self.record_streams_frame.winfo_children():
            child.destroy()

        row = 0
        for method_id, state in method_states.items():
            var = self._method_record_vars.get(method_id)
            if var is None:
                var = tk.BooleanVar(value=bool(state.get("record", False)))
                self._method_record_vars[method_id] = var
            else:
                var.set(bool(state.get("record", False)))
            check = ttk.Checkbutton(
                self.record_streams_frame,
                text=str(state.get("name", method_id)),
                variable=var,
                command=lambda method_id=method_id, var=var: self._handle_method_record_change(method_id, bool(var.get())),
                state="normal" if bool(state.get("can_record", False)) else "disabled",
            )
            check.grid(row=row, column=0, sticky="w", pady=1)
            bind_vertical_mousewheel_tree(check, target=self.control_canvas)
            self._method_record_checks[method_id] = check
            row += 1

    def _make_card(self, parent: tk.Widget) -> ttk.Frame:
        return ttk.Frame(parent, padding=8, borderwidth=1, relief="solid")

    def _make_status_pill(self, parent: tk.Widget, text: str, color: str) -> tk.Label:
        return tk.Label(
            parent,
            text=text,
            bg=color,
            fg="#ffffff" if color in {COLOR_GREEN, COLOR_RED} else "#000000",
            padx=6,
            pady=2,
        )

    def _on_cards_frame_configure(self, _event: tk.Event) -> None:
        self.cards_canvas.configure(scrollregion=self.cards_canvas.bbox("all"))

    def _on_cards_canvas_configure(self, event: tk.Event) -> None:
        self.cards_canvas.itemconfigure(self._cards_window, width=event.width)

    def _on_control_frame_configure(self, _event: tk.Event) -> None:
        self.control_canvas.configure(scrollregion=self.control_canvas.bbox("all"))

    def _on_control_canvas_configure(self, event: tk.Event) -> None:
        self.control_canvas.itemconfigure(self._control_window, width=event.width)

    def _canvas_dimensions(self) -> tuple[float, float]:
        width = int(self.view_canvas.winfo_width())
        height = int(self.view_canvas.winfo_height())
        if width <= 1:
            width = int(float(self.view_canvas.cget("width")))
        if height <= 1:
            height = int(float(self.view_canvas.cget("height")))
        return (float(max(width, 1)), float(max(height, 1)))

    def _canvas_center(self) -> tuple[float, float]:
        width, height = self._canvas_dimensions()
        return (width * 0.5, height * 0.5)

    def _reference_radius(self) -> float:
        radius = max(self._dataset_cloud_max_radius, self._method_dataset_cloud_max_radius)
        if self._current_mag_vector is not None and self._source_cards["raw_magnetometer"].show_var.get():
            mx, my, mz = self._current_mag_vector
            radius = max(radius, abs(mx), abs(my), abs(mz), math.sqrt(mx * mx + my * my + mz * mz))
        for stream in self._iter_visible_derived_streams():
            mx = float(stream["mx"])
            my = float(stream["my"])
            mz = float(stream["mz"])
            radius = max(radius, abs(mx), abs(my), abs(mz), math.sqrt(mx * mx + my * my + mz * mz))
        return radius

    def _projection_fit_points(self, projection: str | None = None) -> list[tuple[float, float]]:
        current_projection = self._projection_mode if projection is None else projection
        radius = self._reference_radius()
        points: list[tuple[float, float]]
        if current_projection == PROJECTION_3D:
            points = [
                self._project_scene_point(radius, 0.0, 0.0, projection=current_projection),
                self._project_scene_point(-radius, 0.0, 0.0, projection=current_projection),
                self._project_scene_point(0.0, radius, 0.0, projection=current_projection),
                self._project_scene_point(0.0, -radius, 0.0, projection=current_projection),
                self._project_scene_point(0.0, 0.0, radius, projection=current_projection),
                self._project_scene_point(0.0, 0.0, -radius, projection=current_projection),
            ]
        elif current_projection == PROJECTION_XY:
            points = [(radius, 0.0), (-radius, 0.0), (0.0, radius), (0.0, -radius)]
        elif current_projection == PROJECTION_XZ:
            points = [(radius, 0.0), (-radius, 0.0), (0.0, radius), (0.0, -radius)]
        else:
            points = [(radius, 0.0), (-radius, 0.0), (0.0, radius), (0.0, -radius)]
        if self._current_mag_vector is not None:
            points.append(self._project_scene_point(*self._current_mag_vector, projection=current_projection))
        for stream in self._iter_visible_derived_streams():
            points.append(self._project_scene_point(stream["mx"], stream["my"], stream["mz"], projection=current_projection))
        return points

    def _recommended_fit_radius(self, projection: str | None = None) -> float:
        current_projection = self._projection_mode if projection is None else projection
        max_extent = 1.0
        for x_val, y_val in self._projection_fit_points(current_projection):
            max_extent = max(max_extent, abs(x_val), abs(y_val))
        return max_extent * 1.15

    def _display_scale(self) -> float:
        width, height = self._canvas_dimensions()
        fit_pixels = 0.42 * min(width, height)
        return max(1e-9, fit_pixels / max(self._fit_radius, 1e-9) * self._view_scale)

    def _display_reference_radius(self) -> float:
        return min(self._reference_radius(), max(1.0, self._fit_radius / 1.15))

    def _scene_to_view(self, point: tuple[float, float]) -> tuple[float, float]:
        cx, cy = self._canvas_center()
        pan_x, pan_y = self._view_pan
        scale = self._display_scale()
        return (cx + (point[0] + pan_x) * scale, cy - (point[1] + pan_y) * scale)

    def _view_to_scene(self, point: tuple[float, float]) -> tuple[float, float]:
        cx, cy = self._canvas_center()
        pan_x, pan_y = self._view_pan
        scale = self._display_scale()
        return ((point[0] - cx) / scale - pan_x, -(point[1] - cy) / scale - pan_y)

    def _fit_view(self) -> None:
        self._fit_radius = self._recommended_fit_radius()
        self._view_scale = 1.0
        self._view_pan = (0.0, 0.0)
        self._redraw_view()

    def _reset_view(self) -> None:
        self._view_scale = 1.0
        self._view_pan = (0.0, 0.0)
        self._view_rotation_yaw_deg = DEFAULT_3D_YAW_DEG
        self._view_rotation_pitch_deg = DEFAULT_3D_PITCH_DEG
        if self.auto_fit_var.get():
            self._sync_auto_fit_radius()
        else:
            self._fit_radius = 1.0
        self._redraw_view()

    def _set_projection(self, projection: str) -> None:
        if projection == self._projection_mode:
            return
        self._projection_mode = projection
        self._view_scale = 1.0
        self._view_pan = (0.0, 0.0)
        if self.auto_fit_var.get():
            self._sync_auto_fit_radius(projection=projection)
        self._update_projection_buttons()
        self._redraw_view()

    def _sync_auto_fit_radius(self, *, projection: str | None = None) -> None:
        self._fit_radius = self._recommended_fit_radius(projection)

    def _update_projection_buttons(self) -> None:
        for projection, button in self._projection_buttons.items():
            if projection == self._projection_mode:
                button.state(["disabled"])
            else:
                button.state(["!disabled"])

    def _on_pan_start(self, event: tk.Event) -> None:
        self._pan_start = (event.x, event.y)
        self._pan_origin = self._view_pan

    def _on_pan_drag(self, event: tk.Event) -> None:
        if self._pan_start is None or self._pan_origin is None:
            return
        dx = event.x - self._pan_start[0]
        dy = event.y - self._pan_start[1]
        scale = self._display_scale()
        self._view_pan = (self._pan_origin[0] + dx / scale, self._pan_origin[1] - dy / scale)
        self._redraw_view()

    def _on_pan_end(self, _event: tk.Event) -> None:
        self._pan_start = None
        self._pan_origin = None

    def _on_rotate_start(self, event: tk.Event) -> None:
        if self._projection_mode != PROJECTION_3D:
            return
        self._rotate_start = (event.x, event.y)
        self._rotate_origin = (self._view_rotation_yaw_deg, self._view_rotation_pitch_deg)

    def _on_rotate_drag(self, event: tk.Event) -> None:
        if self._projection_mode != PROJECTION_3D:
            return
        if self._rotate_start is None or self._rotate_origin is None:
            return
        dx = event.x - self._rotate_start[0]
        dy = event.y - self._rotate_start[1]
        self._view_rotation_yaw_deg = self._rotate_origin[0] + dx * 0.45
        self._view_rotation_pitch_deg = max(-85.0, min(85.0, self._rotate_origin[1] - dy * 0.35))
        self._redraw_view()

    def _on_rotate_end(self, _event: tk.Event) -> None:
        self._rotate_start = None
        self._rotate_origin = None

    def _on_zoom(self, event: tk.Event) -> None:
        if getattr(event, "num", None) == 4:
            delta = 1
        elif getattr(event, "num", None) == 5:
            delta = -1
        else:
            delta = 1 if event.delta > 0 else -1
        factor = 1.1 if delta > 0 else 1 / 1.1
        old_scale = self._view_scale
        new_scale = max(0.25, min(6.0, old_scale * factor))
        if abs(new_scale - old_scale) <= 1e-9:
            return
        scene_x, scene_y = self._view_to_scene((event.x, event.y))
        view_cx, view_cy = self._canvas_center()
        self._view_scale = new_scale
        new_display_scale = self._display_scale()
        self._view_pan = (
            (event.x - view_cx) / new_display_scale - scene_x,
            -(event.y - view_cy) / new_display_scale - scene_y,
        )
        self._redraw_view()

    def _draw_circle(self, radius: float, outline: str = "#2f5f8f", dash: tuple[int, int] | None = None) -> None:
        left_top = self._scene_to_view((-radius, radius))
        right_bottom = self._scene_to_view((radius, -radius))
        self.view_canvas.create_oval(
            left_top[0],
            left_top[1],
            right_bottom[0],
            right_bottom[1],
            outline=outline,
            width=2,
            dash=dash,
        )

    def _draw_compass_labels(self, radius: float) -> None:
        label_offsets = {
            "N": (0.0, radius),
            "S": (0.0, -radius),
            "E": (radius, 0.0),
            "W": (-radius, 0.0),
        }
        for label, scene_point in label_offsets.items():
            vx, vy = self._scene_to_view(scene_point)
            self.view_canvas.create_text(vx, vy, text=label, fill="#000000")

    def _stream_color(self, stream_id: str) -> str:
        palette = ("#1f6aa5", "#16803c", "#a35d00", "#7a1fa2", "#0e7490", "#9f1239")
        index = sum(ord(ch) for ch in stream_id) % len(palette)
        return palette[index]

    def _project_scene_point(
        self,
        mx: float,
        my: float,
        mz: float,
        *,
        projection: str | None = None,
    ) -> tuple[float, float]:
        current_projection = self._projection_mode if projection is None else projection
        if current_projection != PROJECTION_3D:
            return project_magnetometer_point(current_projection, mx, my, mz)
        return project_rotated_magnetometer_point(
            mx,
            my,
            mz,
            yaw_deg=self._view_rotation_yaw_deg,
            pitch_deg=self._view_rotation_pitch_deg,
        )

    def _raw_cloud_records(self) -> list[SampleRecord]:
        return [
            record
            for record in self._dataset_records_by_stream.get("raw_magnetometer", [])
            if "heading_only" not in (record.flags or "")
        ]

    def _iter_visible_dataset_clouds(self) -> list[dict[str, object]]:
        clouds: list[dict[str, object]] = []
        raw_card = self._source_cards.get("raw_magnetometer")
        if raw_card is not None and raw_card.show_var.get() and self.view_option_vars["show raw points"].get():
            records = self._raw_cloud_records()
            if records:
                clouds.append({
                    "stream_id": "raw_magnetometer",
                    "title": "Raw Magnetometer",
                    "records": records,
                    "color": "#d14b4b",
                })
        if self.view_option_vars["show corrected points"].get():
            for method_id, stream_id in self._method_stream_ids.items():
                card = self._method_cards.get(method_id)
                if card is None or not card.show_var.get():
                    continue
                records = self._dataset_records_by_stream.get(stream_id, [])
                if not records:
                    records = self._method_dataset_clouds.get(method_id, [])
                if not records:
                    continue
                clouds.append({
                    "stream_id": stream_id,
                    "title": self._method_titles.get(method_id, stream_id),
                    "records": records,
                    "color": self._stream_color(method_id),
                })
        return clouds

    def _sample_cloud_records(self, records: list[SampleRecord], *, per_stream_budget: int) -> list[SampleRecord]:
        limit = max(MIN_CLOUD_POINTS_PER_STREAM, per_stream_budget)
        if len(records) <= limit:
            return records
        step = max(1, math.ceil(len(records) / limit))
        return records[::step]

    def _draw_cloud_point(self, point: tuple[float, float], *, color: str, size: float = 2.5) -> None:
        vx, vy = self._scene_to_view(point)
        self.view_canvas.create_oval(
            vx - size,
            vy - size,
            vx + size,
            vy + size,
            fill=color,
            outline="",
            stipple="gray50",
        )

    def _draw_dataset_clouds(self, projection: str) -> None:
        clouds = self._iter_visible_dataset_clouds()
        if not clouds:
            return
        width, height = self._canvas_dimensions()
        budget = max(320, min(MAX_CLOUD_POINTS, int(min(width, height) * 1.6)))
        per_stream_budget = max(MIN_CLOUD_POINTS_PER_STREAM, budget // max(1, len(clouds)))
        for cloud in clouds:
            for record in self._sample_cloud_records(cloud["records"], per_stream_budget=per_stream_budget):
                point = self._project_scene_point(record.mag_x, record.mag_y, record.mag_z, projection=projection)
                self._draw_cloud_point(point, color=str(cloud["color"]))

    def _iter_visible_derived_streams(self) -> list[dict[str, object]]:
        visible: list[dict[str, object]] = []
        for method_id, stream in self._derived_streams.items():
            if not bool(stream.get("show", False)):
                continue
            mx = stream.get("mx")
            my = stream.get("my")
            mz = stream.get("mz")
            if not all(isinstance(value, (int, float)) and math.isfinite(float(value)) for value in (mx, my, mz)):
                continue
            visible.append({
                "method_id": method_id,
                "title": str(stream.get("title", method_id)),
                "stream_id": str(stream.get("stream_id", self._method_stream_ids.get(method_id, f"derived_{method_id}"))),
                "mx": float(mx),
                "my": float(my),
                "mz": float(mz),
                "heading": stream.get("heading"),
                "color": str(stream.get("color", self._stream_color(method_id))),
            })
        return visible

    def _draw_heading_vector_xy(self, _radius: float) -> None:
        if (
            self._current_heading_deg is None
            or not self.view_option_vars["show raw heading"].get()
            or not self._source_cards["raw_heading"].show_var.get()
            or self._current_mag_vector is None
        ):
            return
        target = (self._current_mag_vector[0], self._current_mag_vector[1])
        vx0, vy0 = self._scene_to_view((0.0, 0.0))
        vx1, vy1 = self._scene_to_view(target)
        self.view_canvas.create_line(vx0, vy0, vx1, vy1, fill="#1f6aa5", width=3, arrow="last")

    def _draw_derived_heading_vectors_xy(self, _radius: float) -> None:
        if not self.view_option_vars["show derived headings"].get():
            return
        for stream in self._iter_visible_derived_streams():
            target = (float(stream["mx"]), float(stream["my"]))
            vx0, vy0 = self._scene_to_view((0.0, 0.0))
            vx1, vy1 = self._scene_to_view(target)
            self.view_canvas.create_line(vx0, vy0, vx1, vy1, fill=stream["color"], width=2, arrow="last")

    def _draw_current_point(self, point: tuple[float, float], *, color: str = COLOR_RED, size: int = 5) -> None:
        if not self.view_option_vars["show current point"].get():
            return
        vx, vy = self._scene_to_view(point)
        self.view_canvas.create_oval(vx - size, vy - size, vx + size, vy + size, fill=color, outline="")

    def _draw_derived_points(self, projection: str) -> None:
        if not self.view_option_vars["show corrected points"].get():
            return
        for stream in self._iter_visible_derived_streams():
            point = self._project_scene_point(stream["mx"], stream["my"], stream["mz"], projection=projection)
            self._draw_current_point(point, color=stream["color"], size=4)

    def _draw_side_projection_axes(self, horizontal_pos: str, horizontal_neg: str, radius: float) -> None:
        axis_color = "#606060"
        x0, y0 = self._scene_to_view((-radius, 0.0))
        x1, y1 = self._scene_to_view((radius, 0.0))
        self.view_canvas.create_line(x0, y0, x1, y1, fill=axis_color, dash=(3, 3))
        zx0, zy0 = self._scene_to_view((0.0, -radius))
        zx1, zy1 = self._scene_to_view((0.0, radius))
        self.view_canvas.create_line(zx0, zy0, zx1, zy1, fill=axis_color, dash=(3, 3))
        self.view_canvas.create_text(*self._scene_to_view((radius, 0.0)), text=horizontal_pos, fill="#000000")
        self.view_canvas.create_text(*self._scene_to_view((-radius, 0.0)), text=horizontal_neg, fill="#000000")
        self.view_canvas.create_text(*self._scene_to_view((0.0, radius)), text="+Z", fill="#000000")
        self.view_canvas.create_text(*self._scene_to_view((0.0, -radius)), text="-Z", fill="#000000")

    def _draw_heading_inset(self) -> None:
        if self._current_heading_deg is None or not self._source_cards["raw_heading"].show_var.get():
            return
        width, _height = self._canvas_dimensions()
        cx = width - 72.0
        cy = 72.0
        radius = 38.0
        canvas = self.view_canvas
        canvas.create_oval(cx - radius, cy - radius, cx + radius, cy + radius, outline="#2f5f8f", width=2)
        canvas.create_text(cx, cy - radius - 10, text="N", fill="#000000")
        canvas.create_text(cx, cy + radius + 10, text="S", fill="#000000")
        canvas.create_text(cx - radius - 10, cy, text="W", fill="#000000")
        canvas.create_text(cx + radius + 10, cy, text="E", fill="#000000")
        heading_rad = math.radians(self._current_heading_deg)
        dx = math.sin(heading_rad) * radius * 0.82
        dy = -math.cos(heading_rad) * radius * 0.82
        canvas.create_line(cx, cy, cx + dx, cy + dy, fill="#1f6aa5", width=3, arrow="last")
        if self.view_option_vars["show derived headings"].get():
            for stream in self._iter_visible_derived_streams():
                heading = stream.get("heading")
                if not isinstance(heading, (int, float)) or not math.isfinite(float(heading)):
                    continue
                heading_rad = math.radians(float(heading))
                dx = math.sin(heading_rad) * radius * 0.72
                dy = -math.cos(heading_rad) * radius * 0.72
                canvas.create_line(cx, cy, cx + dx, cy + dy, fill=stream["color"], width=2, arrow="last")

    def _draw_3d_projection(self) -> None:
        radius = self._display_reference_radius()
        compass_pts = [
            self._scene_to_view(self._project_scene_point(math.sin(angle) * radius, math.cos(angle) * radius, 0.0, projection=PROJECTION_3D))
            for angle in [math.radians(step) for step in range(0, 361, 8)]
        ]
        flat_pts = [coord for pt in compass_pts for coord in pt]
        self.view_canvas.create_line(*flat_pts, fill="#2f5f8f", width=2, smooth=True)
        self._draw_dataset_clouds(PROJECTION_3D)

        for label, coords in (
            ("N", (0.0, radius, 0.0)),
            ("S", (0.0, -radius, 0.0)),
            ("E", (radius, 0.0, 0.0)),
            ("W", (-radius, 0.0, 0.0)),
        ):
            self.view_canvas.create_text(*self._scene_to_view(self._project_scene_point(*coords, projection=PROJECTION_3D)), text=label, fill="#000000")

        z_top = self._scene_to_view(self._project_scene_point(0.0, 0.0, radius, projection=PROJECTION_3D))
        z_bottom = self._scene_to_view(self._project_scene_point(0.0, 0.0, -radius, projection=PROJECTION_3D))
        self.view_canvas.create_line(z_bottom[0], z_bottom[1], z_top[0], z_top[1], fill="#606060", dash=(3, 3))
        self.view_canvas.create_text(z_top[0], z_top[1] - 10, text="+Z", fill="#000000")
        self.view_canvas.create_text(z_bottom[0], z_bottom[1] + 10, text="-Z", fill="#000000")

        if self._current_mag_vector is not None and self.view_option_vars["show raw heading"].get():
            heading_end = (self._current_mag_vector[0], self._current_mag_vector[1])
            origin = self._scene_to_view((0.0, 0.0))
            vx1, vy1 = self._scene_to_view(self._project_scene_point(heading_end[0], heading_end[1], 0.0, projection=PROJECTION_3D))
            self.view_canvas.create_line(origin[0], origin[1], vx1, vy1, fill="#1f6aa5", width=3, arrow="last")
        if self.view_option_vars["show derived headings"].get():
            for stream in self._iter_visible_derived_streams():
                heading_end = (float(stream["mx"]), float(stream["my"]))
                origin = self._scene_to_view((0.0, 0.0))
                vx1, vy1 = self._scene_to_view(self._project_scene_point(heading_end[0], heading_end[1], 0.0, projection=PROJECTION_3D))
                self.view_canvas.create_line(origin[0], origin[1], vx1, vy1, fill=stream["color"], width=2, arrow="last")

        if (
            self._current_mag_vector is not None
            and self._source_cards["raw_magnetometer"].show_var.get()
        ):
            self._draw_current_point(self._project_scene_point(*self._current_mag_vector, projection=PROJECTION_3D))
        self._draw_derived_points(PROJECTION_3D)

    def _draw_xy_projection(self) -> None:
        radius = self._display_reference_radius()
        self._draw_circle(radius)
        self._draw_compass_labels(radius * 1.06)
        self.view_canvas.create_text(*self._scene_to_view((radius * 1.05, radius * 0.95)), text="+Z", fill="#000000")
        self.view_canvas.create_text(*self._scene_to_view((radius * 1.05, -radius * 0.95)), text="-Z", fill="#000000")
        self._draw_dataset_clouds(PROJECTION_XY)
        self._draw_heading_vector_xy(radius)
        self._draw_derived_heading_vectors_xy(radius)
        if (
            self._current_mag_vector is not None
            and self._source_cards["raw_magnetometer"].show_var.get()
        ):
            self._draw_current_point(self._project_scene_point(*self._current_mag_vector, projection=PROJECTION_XY))
        self._draw_derived_points(PROJECTION_XY)

    def _draw_xz_projection(self) -> None:
        radius = self._display_reference_radius()
        self._draw_side_projection_axes("E", "W", radius)
        self._draw_dataset_clouds(PROJECTION_XZ)
        if (
            self._current_mag_vector is not None
            and self._source_cards["raw_magnetometer"].show_var.get()
        ):
            self._draw_current_point(self._project_scene_point(*self._current_mag_vector, projection=PROJECTION_XZ))
        self._draw_derived_points(PROJECTION_XZ)
        self._draw_heading_inset()

    def _draw_yz_projection(self) -> None:
        radius = self._display_reference_radius()
        self._draw_side_projection_axes("N", "S", radius)
        self._draw_dataset_clouds(PROJECTION_YZ)
        if (
            self._current_mag_vector is not None
            and self._source_cards["raw_magnetometer"].show_var.get()
        ):
            self._draw_current_point(self._project_scene_point(*self._current_mag_vector, projection=PROJECTION_YZ))
        self._draw_derived_points(PROJECTION_YZ)
        self._draw_heading_inset()

    def _heading_legend_items(self) -> list[tuple[str, str]]:
        items: list[tuple[str, str]] = []
        if self.view_option_vars["show raw heading"].get() and self._source_cards["raw_heading"].show_var.get():
            items.append(("Raw Heading", "#1f6aa5"))
        if self.view_option_vars["show derived headings"].get():
            for stream in self._iter_visible_derived_streams():
                items.append((str(stream["title"]), str(stream["color"])))
        return items

    def _draw_heading_legend(self) -> None:
        items = self._heading_legend_items()
        if not items:
            return
        canvas = self.view_canvas
        x0 = 38.0
        y0 = 38.0
        row_h = 18.0
        box_w = 180.0
        box_h = 24.0 + row_h * len(items)
        canvas.create_rectangle(x0, y0, x0 + box_w, y0 + box_h, fill="#f7f7f7", outline="#909090")
        canvas.create_text(x0 + 10, y0 + 12, text="Heading legend", anchor="w", fill="#202020")
        primary_label = self.primary_heading_var.get()
        for idx, (label, color) in enumerate(items):
            y = y0 + 28.0 + idx * row_h
            canvas.create_line(x0 + 10, y, x0 + 34, y, fill=color, width=3)
            legend_label = f"{label} [primary]" if label == primary_label else label
            canvas.create_text(x0 + 42, y, text=legend_label, anchor="w", fill="#202020")

    def _redraw_view(self, _event: tk.Event | None = None) -> None:
        canvas = self.view_canvas
        width, height = self._canvas_dimensions()
        if self.auto_fit_var.get():
            self._sync_auto_fit_radius()
        canvas.delete("all")

        pad = 24
        canvas.create_rectangle(pad, pad, width - pad, height - pad, outline="#808080", dash=(4, 4))

        if self._projection_mode == PROJECTION_3D:
            self._draw_3d_projection()
        elif self._projection_mode == PROJECTION_XY:
            self._draw_xy_projection()
        elif self._projection_mode == PROJECTION_XZ:
            self._draw_xz_projection()
        else:
            self._draw_yz_projection()

        self._draw_heading_legend()
        projection_label = self._projection_mode
        hint = "LMB rotate | RMB pan | wheel zoom" if self._projection_mode == PROJECTION_3D else "RMB pan | wheel zoom"
        canvas.create_text(width * 0.5, height - 16, text=f"Projection: {projection_label} | {hint}", fill="#303030")

    def update_live_imu(
        self,
        *,
        timestamp_mcu: int,
        timestamp_pc_rx: int,
        timestamp_pc_est: int | None,
        mx: float,
        my: float,
        mz: float,
        selected_output_heading: float | None = None,
        selected_source_label: str | None = None,
        derived_streams: dict[str, dict[str, object]] | None = None,
    ) -> None:
        magnitude = math.sqrt(mx * mx + my * my + mz * mz)
        heading_deg = compute_raw_heading_deg(mx, my)

        self.current_data_vars["MCU ts"].set(str(timestamp_mcu))
        self.current_data_vars["PC rx ts"].set(str(timestamp_pc_rx))
        self.current_data_vars["PC est ts"].set("—" if timestamp_pc_est is None else str(timestamp_pc_est))
        self.current_data_vars["mx"].set(f"{mx:.3f}")
        self.current_data_vars["my"].set(f"{my:.3f}")
        self.current_data_vars["mz"].set(f"{mz:.3f}")
        self.current_data_vars["|m|"].set(f"{magnitude:.3f}")

        heading_text = "—" if heading_deg is None else f"{heading_deg:.1f}°"
        self.current_data_vars["raw heading"].set(heading_text)
        selected_heading = heading_deg if selected_output_heading is None else selected_output_heading
        self.set_primary_output_display(
            heading=selected_heading,
            source_label=selected_source_label or self.current_data_vars["selected source"].get(),
        )
        self._update_dataset_status()

        first_sample = self._current_mag_vector is None
        self._current_heading_deg = heading_deg
        self._current_mag_vector = (mx, my, mz)
        self._derived_streams = {
            method_id: dict(stream_state)
            for method_id, stream_state in (derived_streams or {}).items()
        }
        self._update_heading_stream_summary()
        if first_sample and self.auto_fit_var.get():
            self._sync_auto_fit_radius()
        elif self.auto_fit_var.get():
            self._sync_auto_fit_radius()

        now = time.monotonic()
        if first_sample or (now - self._last_live_redraw_s) >= LIVE_REDRAW_INTERVAL_S:
            self._last_live_redraw_s = now
            self._redraw_view()

    def set_recording_state(
        self,
        *,
        is_recording: bool,
        can_save: bool,
        has_dataset: bool = False,
        can_concatenate: bool = False,
    ) -> None:
        self._recording_active = is_recording
        if is_recording:
            self.start_record_btn.state(["disabled"])
            self.stop_record_btn.state(["!disabled"])
            self.load_csv_btn.state(["disabled"])
            self.load_multiple_btn.state(["disabled"])
            self.save_current_btn.state(["disabled"])
            self.save_as_btn.state(["disabled"])
            self.concatenate_btn.state(["disabled"])
            self.trim_selection_btn.state(["disabled"])
            self.delete_selection_btn.state(["disabled"])
            self.dataset_name_combo.configure(state="disabled")
        else:
            self.start_record_btn.state(["!disabled"])
            self.stop_record_btn.state(["disabled"])
            self._flush_pending_dataset_rows(force_all=True)
            self.load_csv_btn.state(["!disabled"])
            self.load_multiple_btn.state(["!disabled"])
            if can_save:
                self.save_current_btn.state(["!disabled"])
                self.save_as_btn.state(["!disabled"])
            else:
                self.save_current_btn.state(["disabled"])
                self.save_as_btn.state(["disabled"])
            if can_concatenate:
                self.concatenate_btn.state(["!disabled"])
            else:
                self.concatenate_btn.state(["disabled"])
            if has_dataset:
                self.trim_selection_btn.state(["!disabled"])
                self.delete_selection_btn.state(["!disabled"])
            else:
                self.trim_selection_btn.state(["disabled"])
                self.delete_selection_btn.state(["disabled"])
            if self._dataset_choice_labels:
                self.dataset_name_combo.configure(state="readonly")
            else:
                self.dataset_name_combo.configure(state="disabled")
        self._update_dataset_status()

    def update_dataset_summary(self, *, name: str, row_count: str, source_count: str, time_range: str) -> None:
        self.dataset_summary_vars["active dataset name"].set(name)
        self.dataset_summary_vars["number of rows"].set(row_count)
        self.dataset_summary_vars["source count"].set(source_count)
        self.dataset_summary_vars["time range"].set(time_range)
        self._dataset_row_count = int(row_count) if row_count.isdigit() else 0
        self._update_dataset_status()

    def set_dataset_choices(self, labels: list[str], active_index: int | None) -> None:
        self._dataset_choice_labels = list(labels)
        self.dataset_name_combo.configure(values=tuple(labels))
        if active_index is None or not labels:
            self.dataset_name_combo.configure(state="disabled")
            self.dataset_summary_vars["active dataset name"].set("—")
            return
        self.dataset_summary_vars["active dataset name"].set(labels[active_index])
        if self._recording_active:
            self.dataset_name_combo.configure(state="disabled")
        else:
            self.dataset_name_combo.configure(state="readonly")

    def set_source_states(self, source_states: dict[str, dict[str, object]], selected_source_id: str) -> None:
        for source_id, card in self._source_cards.items():
            state = source_states.get(source_id, {})
            show_enabled = bool(state.get("show", False))
            record_enabled = bool(state.get("record", False))
            status_text, status_color = resolve_source_card_status(show_enabled, record_enabled)
            card.set_show(show_enabled)
            card.set_record(record_enabled)
            card.set_status(status_text, status_color)
            card.set_selected(source_id == selected_source_id)
        self._update_heading_stream_summary()
        self._redraw_view()

    def _record_radius(self, record: SampleRecord) -> float:
        mx = float(record.mag_x)
        my = float(record.mag_y)
        mz = float(record.mag_z)
        return max(abs(mx), abs(my), abs(mz), math.sqrt(mx * mx + my * my + mz * mz))

    def _recompute_dataset_cloud_max_radius(self) -> None:
        radius = 1.0
        for record in self._dataset_records:
            radius = max(radius, self._record_radius(record))
        self._dataset_cloud_max_radius = radius

    def _recompute_method_cloud_max_radius(self) -> None:
        radius = 1.0
        for records in self._method_dataset_clouds.values():
            for record in records:
                radius = max(radius, self._record_radius(record))
        self._method_dataset_cloud_max_radius = radius

    def _set_dataset_record_cache(self, records: list[SampleRecord]) -> None:
        self._dataset_records = list(records)
        self._dataset_records_by_stream = {}
        for record in self._dataset_records:
            self._dataset_records_by_stream.setdefault(record.stream_id, []).append(record)
        self._recompute_dataset_cloud_max_radius()

    def _insert_dataset_row(self, row_id: int, record: SampleRecord) -> None:
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
                f"{record.mag_x:.6g}",
                f"{record.mag_y:.6g}",
                f"{record.mag_z:.6g}",
                f"{record.heading:.3f}" if record.heading is not None else "—",
                record.flags,
            ),
        )

    def _is_data_tab_visible(self) -> bool:
        return getattr(self, "data_tab_frame", None) is not None and self.bottom_nb.select() == str(self.data_tab_frame)

    def _schedule_dataset_row_flush(self) -> None:
        if self._dataset_row_flush_after_id is not None:
            return
        self._dataset_row_flush_after_id = self.after(DATA_TABLE_FLUSH_DELAY_MS, self._flush_pending_dataset_rows)

    def _flush_pending_dataset_rows(self, *, force_all: bool = False) -> None:
        self._dataset_row_flush_after_id = None
        if not self._pending_dataset_rows:
            return
        if self._recording_active and not force_all and not self._is_data_tab_visible():
            return
        batch_size = len(self._pending_dataset_rows) if force_all else min(DATA_TABLE_FLUSH_BATCH, len(self._pending_dataset_rows))
        batch = self._pending_dataset_rows[:batch_size]
        del self._pending_dataset_rows[:batch_size]
        for row_id, record in batch:
            self._insert_dataset_row(row_id, record)
        if self._pending_dataset_rows:
            self._schedule_dataset_row_flush()

    def clear_dataset_rows(self, *, clear_cache: bool = True) -> None:
        if self._dataset_row_flush_after_id is not None:
            self.after_cancel(self._dataset_row_flush_after_id)
            self._dataset_row_flush_after_id = None
        self._pending_dataset_rows.clear()
        for item in self.data_tree.get_children():
            self.data_tree.delete(item)
        if clear_cache:
            self._set_dataset_record_cache([])
            self._redraw_view()

    def set_dataset_records(self, records: list[SampleRecord]) -> None:
        self._set_dataset_record_cache(records)
        self.clear_dataset_rows(clear_cache=False)
        for row_id, record in enumerate(records, start=1):
            self._insert_dataset_row(row_id, record)
        if records and self.view_option_vars["auto fit on load"].get():
            self._fit_radius = self._recommended_fit_radius()
            self._view_scale = 1.0
            self._view_pan = (0.0, 0.0)
        self._redraw_view()

    def append_dataset_record(self, row_id: int, record: SampleRecord) -> None:
        self._dataset_records.append(record)
        self._dataset_records_by_stream.setdefault(record.stream_id, []).append(record)
        self._dataset_cloud_max_radius = max(self._dataset_cloud_max_radius, self._record_radius(record))
        self._pending_dataset_rows.append((row_id, record))
        if not self._recording_active or self._is_data_tab_visible():
            self._schedule_dataset_row_flush()

    def clear_metrics_rows(self) -> None:
        for item in self.metrics_tree.get_children():
            self.metrics_tree.delete(item)

    def set_metrics_report(
        self,
        *,
        rows: list[dict[str, str]],
        summary_text: str,
        can_export: bool,
    ) -> None:
        self.clear_metrics_rows()
        self.metrics_status_var.set(summary_text)
        for row in rows:
            self.metrics_tree.insert(
                "",
                "end",
                values=(
                    row.get("metric", ""),
                    row.get("value", ""),
                    row.get("units", ""),
                    row.get("status", ""),
                    row.get("notes", ""),
                ),
            )
        self.export_metrics_btn.configure(state="normal" if can_export else "disabled")

    def get_view_state(self) -> dict[str, object]:
        return {
            "projection_mode": self._projection_mode,
            "auto_fit": bool(self.auto_fit_var.get()),
            "view_options": {
                label: bool(var.get())
                for label, var in self.view_option_vars.items()
            },
        }

    def apply_view_state(self, state: dict[str, object]) -> None:
        projection_mode = str(state.get("projection_mode", self._projection_mode))
        if projection_mode in PROJECTION_MODES:
            self._projection_mode = projection_mode
        self.auto_fit_var.set(bool(state.get("auto_fit", self.auto_fit_var.get())))
        view_options = state.get("view_options", {})
        if isinstance(view_options, dict):
            for label, var in self.view_option_vars.items():
                if label in view_options:
                    var.set(bool(view_options[label]))
        self._update_projection_buttons()
        if self.auto_fit_var.get():
            self._sync_auto_fit_radius()
        self._redraw_view()

    def _update_dataset_status(self) -> None:
        if self._recording_active:
            self.current_data_vars["dataset status"].set(f"Recording ({self._dataset_row_count})")
        elif self._dataset_row_count > 0:
            self.current_data_vars["dataset status"].set(f"Dataset ready ({self._dataset_row_count})")
        else:
            self.current_data_vars["dataset status"].set("Live IMU")

    def _handle_dataset_selection(self, _event: tk.Event) -> None:
        if self._on_select_dataset is None:
            return
        index = self.dataset_name_combo.current()
        if index >= 0:
            self._on_select_dataset(index)

    def _handle_source_selection(self, source_id: str) -> None:
        if self._on_select_source is not None:
            self._on_select_source(source_id)

    def _handle_source_show_change(self, source_id: str, enabled: bool) -> None:
        if self._on_source_show_change is not None:
            self._on_source_show_change(source_id, enabled)

    def _handle_source_record_change(self, source_id: str, enabled: bool) -> None:
        if self._on_source_record_change is not None:
            self._on_source_record_change(source_id, enabled)

    def _handle_add_plugin(self) -> None:
        if self._on_add_plugin is not None:
            self._on_add_plugin()

    def _handle_method_selection(self, method_id: str) -> None:
        if self._on_select_method is not None:
            self._on_select_method(method_id)

    def _handle_open_method_info(self, method_id: str) -> None:
        if self._on_open_method_info is not None:
            self._on_open_method_info(method_id)

    def _handle_method_calibrate(self, method_id: str) -> None:
        if self._on_calibrate_method is not None:
            self._on_calibrate_method(method_id)

    def _handle_selected_method_calibrate(self) -> None:
        if self._selected_method_id is None:
            return
        self._handle_method_calibrate(self._selected_method_id)

    def _handle_method_load_params(self, method_id: str) -> None:
        if self._on_load_method_params is not None:
            self._on_load_method_params(method_id)

    def _handle_method_save_params(self, method_id: str) -> None:
        if self._on_save_method_params is not None:
            self._on_save_method_params(method_id)

    def _handle_method_show_change(self, method_id: str, enabled: bool) -> None:
        if self._on_method_show_change is not None:
            self._on_method_show_change(method_id, enabled)

    def _handle_method_remove(self, method_id: str) -> None:
        if self._on_remove_method is not None:
            self._on_remove_method(method_id)

    def _handle_method_toggle_realtime(self, method_id: str) -> None:
        card = self._method_cards.get(method_id)
        if card is None:
            return
        if card.realtime_btn.cget("text") == "Disable realtime":
            if self._on_disable_method_realtime is not None:
                self._on_disable_method_realtime(method_id)
            return
        if self._on_enable_method_realtime is not None:
            self._on_enable_method_realtime(method_id)

    def _handle_method_record_change(self, method_id: str, enabled: bool) -> None:
        if self._on_method_record_change is not None:
            self._on_method_record_change(method_id, enabled)

    def _handle_primary_heading_selection(self, _event: tk.Event) -> None:
        if self._on_select_primary_heading is None:
            return
        index = self.primary_heading_combo.current()
        if 0 <= index < len(self._primary_heading_options):
            self._on_select_primary_heading(self._primary_heading_options[index][0])

    def _handle_selected_method_load_params(self) -> None:
        if self._selected_method_id is None:
            return
        self._handle_method_load_params(self._selected_method_id)

    def _handle_selected_method_save_params(self) -> None:
        if self._selected_method_id is None:
            return
        self._handle_method_save_params(self._selected_method_id)

    def _handle_selected_method_enable_realtime(self) -> None:
        if self._selected_method_id is None:
            return
        if self._on_enable_method_realtime is not None:
            self._on_enable_method_realtime(self._selected_method_id)

    def _handle_selected_method_disable_realtime(self) -> None:
        if self._selected_method_id is None:
            return
        if self._on_disable_method_realtime is not None:
            self._on_disable_method_realtime(self._selected_method_id)

    def _handle_start_record(self) -> None:
        if self._on_start_record is not None:
            self._on_start_record()

    def _handle_stop_record(self) -> None:
        if self._on_stop_record is not None:
            self._on_stop_record()

    def _handle_save_current(self) -> None:
        if self._on_save_current is not None:
            self._on_save_current()

    def _handle_save_as(self) -> None:
        if self._on_save_as is not None:
            self._on_save_as()

    def _handle_load_csv(self) -> None:
        if self._on_load_csv is not None:
            self._on_load_csv()

    def _handle_load_multiple(self) -> None:
        if self._on_load_multiple is not None:
            self._on_load_multiple()

    def _handle_concatenate(self) -> None:
        if self._on_concatenate is not None:
            self._on_concatenate()

    def _handle_trim_selection(self) -> None:
        if self._on_trim_selection is not None:
            self._on_trim_selection()

    def _handle_delete_selection(self) -> None:
        if self._on_delete_selection is not None:
            self._on_delete_selection()

    def _handle_view_option_change(self) -> None:
        self._update_heading_stream_summary()
        self._redraw_view()

    def _handle_auto_fit_toggle(self) -> None:
        if self.auto_fit_var.get():
            self._sync_auto_fit_radius()
            self._view_scale = 1.0
            self._view_pan = (0.0, 0.0)
        self._redraw_view()

    def _handle_bottom_tab_change(self, _event: tk.Event) -> None:
        if self._is_data_tab_visible():
            self._flush_pending_dataset_rows(force_all=True)

    def _handle_export_metrics(self) -> None:
        if self._on_export_metrics is not None:
            self._on_export_metrics()

    def _update_heading_stream_summary(self) -> None:
        visible_names: list[str] = []
        if (
            self._source_cards.get("raw_heading") is not None
            and self._source_cards["raw_heading"].show_var.get()
            and self.view_option_vars["show raw heading"].get()
        ):
            visible_names.append("Raw Heading")
        if self.view_option_vars["show derived headings"].get():
            for method_id, stream in self._derived_streams.items():
                if not bool(stream.get("show", False)):
                    continue
                visible_names.append(self._method_titles.get(method_id, str(stream.get("title", method_id))))

        self.visible_heading_streams_var.set(", ".join(visible_names) if visible_names else "None")

    def get_selected_dataset_row_indices(self) -> list[int]:
        indices: list[int] = []
        for item_id in self.data_tree.selection():
            values = self.data_tree.item(item_id, "values")
            if not values:
                continue
            try:
                indices.append(int(values[0]) - 1)
            except (TypeError, ValueError):
                continue
        return sorted(set(indices))

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
            ("D3", "D3 - sensor tensor data"),
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


class _MenuTooltip:
    def __init__(self, master: tk.Widget) -> None:
        self.master = master
        self._win: tk.Toplevel | None = None

    def show(self, text: str, x: int, y: int) -> None:
        self.hide()
        win = tk.Toplevel(self.master)
        win.wm_overrideredirect(True)
        win.wm_geometry(f"+{x}+{y}")
        label = tk.Label(win, text=text, bg="#ffffe0", fg="#000000", relief="solid", borderwidth=1)
        label.pack()
        self._win = win

    def hide(self) -> None:
        if self._win is not None:
            self._win.destroy()
            self._win = None
