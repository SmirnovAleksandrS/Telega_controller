from __future__ import annotations

import math
import time
from typing import Callable

import tkinter as tk
from tkinter import ttk

from app.accelerometer_dataset import AccelerometerSampleRecord
from app.magnetometer_tab import (
    MAX_CLOUD_POINTS,
    MIN_CLOUD_POINTS_PER_STREAM,
    PROJECTION_3D,
    PROJECTION_MODES,
    PROJECTION_XY,
    PROJECTION_XZ,
    PROJECTION_YZ,
    MagnetometerTab,
    project_magnetometer_point,
    project_rotated_magnetometer_point,
)
from app.source_card import SourceCard
from app.styles import COLOR_RED, PANEL_BG
from app.tk_utils import bind_vertical_mousewheel, bind_vertical_mousewheel_tree

BUILTIN_SOURCE_DEFS = (
    ("raw_accelerometer", "Raw Accelerometer", "Built-in source"),
    ("raw_tilt", "Raw Tilt", "Built-in source"),
)
LIVE_REDRAW_INTERVAL_S = 0.05


def compute_accelerometer_tilt_deg(ax: float, ay: float, az: float) -> tuple[float | None, float | None]:
    if not all(math.isfinite(value) for value in (ax, ay, az)):
        return (None, None)
    magnitude = math.sqrt(ax * ax + ay * ay + az * az)
    if magnitude <= 1e-9:
        return (None, None)
    roll_deg = math.degrees(math.atan2(ay, az))
    pitch_deg = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
    return (roll_deg, pitch_deg)


def _rotation_matrix_from_tilt(roll_deg: float, pitch_deg: float) -> tuple[tuple[float, float, float], ...]:
    roll_rad = math.radians(roll_deg)
    pitch_rad = math.radians(pitch_deg)
    cr = math.cos(roll_rad)
    sr = math.sin(roll_rad)
    cp = math.cos(pitch_rad)
    sp = math.sin(pitch_rad)
    return (
        (cp, 0.0, -sp),
        (sr * sp, cr, sr * cp),
        (cr * sp, -sr, cr * cp),
    )


class AccelerometerTab(MagnetometerTab):
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
        self._current_tilt: tuple[float | None, float | None] = (None, None)
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
            self._add_accelerometer_source_card(source_id, title, source_type)

    def _add_accelerometer_source_card(self, source_id: str, title: str, source_type: str) -> None:
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

    def _build_current_data_section(self, parent: tk.Widget) -> ttk.LabelFrame:
        frm = ttk.LabelFrame(parent, text="Current Data", padding=8)
        frm.columnconfigure(1, weight=1)
        labels = (
            "MCU ts",
            "PC rx ts",
            "PC est ts",
            "ax",
            "ay",
            "az",
            "|a|",
            "roll",
            "pitch",
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
            ("show raw points", True),
            ("show corrected points", True),
            ("show current point", True),
            ("show sensor frame", True),
            ("show derived frames", True),
            ("show gravity vector", True),
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
        self.visible_heading_streams_var = tk.StringVar(value="Raw Tilt")
        ttk.Entry(frm, textvariable=self.visible_heading_streams_var, state="readonly", width=18).grid(
            row=0,
            column=1,
            sticky="ew",
            padx=(8, 0),
            pady=2,
        )

        ttk.Label(frm, text="primary output stream").grid(row=1, column=0, sticky="w", pady=2)
        self.primary_heading_var = tk.StringVar(value="Raw Tilt")
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
            "acc_x",
            "acc_y",
            "acc_z",
            "roll_deg",
            "pitch_deg",
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
        ttk.Label(
            frm,
            text="Accelerometer metrics are not implemented yet.",
            anchor="center",
            justify="center",
        ).grid(row=0, column=0, sticky="nsew")
        self.metrics_status_var = tk.StringVar(value="Accelerometer metrics are not available yet.")
        self.export_metrics_btn = ttk.Button(frm, text="Export CSV", state="disabled")
        self.metrics_tree = ttk.Treeview(frm, columns=("metric",), show="headings", height=1)

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

    def _raw_cloud_records(self) -> list[AccelerometerSampleRecord]:
        return [
            record
            for record in self._dataset_records_by_stream.get("raw_accelerometer", [])
            if "tilt_only" not in (record.flags or "")
        ]

    def _iter_visible_dataset_clouds(self) -> list[dict[str, object]]:
        clouds: list[dict[str, object]] = []
        raw_card = self._source_cards.get("raw_accelerometer")
        if raw_card is not None and raw_card.show_var.get() and self.view_option_vars["show raw points"].get():
            records = self._raw_cloud_records()
            if records:
                clouds.append({
                    "stream_id": "raw_accelerometer",
                    "title": "Raw Accelerometer",
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

    def _iter_visible_derived_streams(self) -> list[dict[str, object]]:
        visible: list[dict[str, object]] = []
        for method_id, stream in self._derived_streams.items():
            if not bool(stream.get("show", False)):
                continue
            values = (stream.get("mx"), stream.get("my"), stream.get("mz"))
            if not all(isinstance(value, (int, float)) and math.isfinite(float(value)) for value in values):
                continue
            visible.append({
                "method_id": method_id,
                "title": str(stream.get("title", method_id)),
                "stream_id": str(stream.get("stream_id", self._method_stream_ids.get(method_id, f"derived_{method_id}"))),
                "mx": float(values[0]),
                "my": float(values[1]),
                "mz": float(values[2]),
                "roll_deg": stream.get("roll_deg"),
                "pitch_deg": stream.get("pitch_deg"),
                "color": str(stream.get("color", self._stream_color(method_id))),
            })
        return visible

    def _frame_axes(self, roll_deg: float | None, pitch_deg: float | None, radius: float) -> tuple[tuple[str, tuple[float, float, float], str], ...]:
        if roll_deg is None or pitch_deg is None:
            return ()
        matrix = _rotation_matrix_from_tilt(roll_deg, pitch_deg)
        axis_scale = radius * 0.55
        return (
            ("X", tuple(value * axis_scale for value in matrix[0]), "#b91c1c"),
            ("Y", tuple(value * axis_scale for value in matrix[1]), "#15803d"),
            ("Z", tuple(value * axis_scale for value in matrix[2]), "#1d4ed8"),
        )

    def _draw_axis_arrow(self, vector: tuple[float, float, float], *, color: str, projection: str) -> None:
        origin = self._scene_to_view((0.0, 0.0))
        vx1, vy1 = self._scene_to_view(self._project_scene_point(*vector, projection=projection))
        self.view_canvas.create_line(origin[0], origin[1], vx1, vy1, fill=color, width=2, arrow="last")

    def _draw_current_frame(self, projection: str, radius: float) -> None:
        if not self.view_option_vars["show sensor frame"].get():
            return
        if not self._source_cards.get("raw_tilt") or not self._source_cards["raw_tilt"].show_var.get():
            return
        roll_deg, pitch_deg = self._current_tilt
        for _label, vector, color in self._frame_axes(roll_deg, pitch_deg, radius):
            self._draw_axis_arrow(vector, color=color, projection=projection)

    def _draw_derived_frames(self, projection: str, radius: float) -> None:
        if not self.view_option_vars.get("show derived frames", tk.BooleanVar(value=True)).get():
            return
        for stream in self._iter_visible_derived_streams():
            for _label, vector, _color in self._frame_axes(
                stream.get("roll_deg"),
                stream.get("pitch_deg"),
                radius * 0.9,
            ):
                self._draw_axis_arrow(vector, color=stream["color"], projection=projection)

    def _draw_gravity_vector(self, projection: str, radius: float) -> None:
        if not self.view_option_vars["show gravity vector"].get():
            return
        if self._current_mag_vector is None:
            return
        ax, ay, az = self._current_mag_vector
        magnitude = math.sqrt(ax * ax + ay * ay + az * az)
        if magnitude <= 1e-9:
            return
        scale = radius * 0.85 / magnitude
        gravity = (-ax * scale, -ay * scale, -az * scale)
        self._draw_axis_arrow(gravity, color="#111111", projection=projection)

    def _draw_plane_axes(self, projection: str, radius: float) -> None:
        if projection == PROJECTION_XY:
            labels = (("+X", (radius, 0.0)), ("-X", (-radius, 0.0)), ("+Y", (0.0, radius)), ("-Y", (0.0, -radius)))
            for label, point in labels:
                self.view_canvas.create_text(*self._scene_to_view(point), text=label, fill="#000000")
            return
        if projection == PROJECTION_XZ:
            self._draw_side_projection_axes("+X", "-X", radius)
            return
        if projection == PROJECTION_YZ:
            self._draw_side_projection_axes("+Y", "-Y", radius)

    def _draw_dataset_clouds(self, projection: str) -> None:
        clouds = self._iter_visible_dataset_clouds()
        if not clouds:
            return
        width, height = self._canvas_dimensions()
        budget = max(320, min(MAX_CLOUD_POINTS, int(min(width, height) * 1.6)))
        per_stream_budget = max(MIN_CLOUD_POINTS_PER_STREAM, budget // max(1, len(clouds)))
        for cloud in clouds:
            for record in self._sample_cloud_records(cloud["records"], per_stream_budget=per_stream_budget):
                point = self._project_scene_point(record.acc_x, record.acc_y, record.acc_z, projection=projection)
                self._draw_cloud_point(point, color=str(cloud["color"]))

    def _draw_3d_projection(self) -> None:
        radius = self._display_reference_radius()
        self._draw_dataset_clouds(PROJECTION_3D)
        for label, coords in (
            ("+X", (radius, 0.0, 0.0)),
            ("-X", (-radius, 0.0, 0.0)),
            ("+Y", (0.0, radius, 0.0)),
            ("-Y", (0.0, -radius, 0.0)),
        ):
            self.view_canvas.create_text(*self._scene_to_view(self._project_scene_point(*coords, projection=PROJECTION_3D)), text=label, fill="#000000")
        z_top = self._scene_to_view(self._project_scene_point(0.0, 0.0, radius, projection=PROJECTION_3D))
        z_bottom = self._scene_to_view(self._project_scene_point(0.0, 0.0, -radius, projection=PROJECTION_3D))
        self.view_canvas.create_line(z_bottom[0], z_bottom[1], z_top[0], z_top[1], fill="#606060", dash=(3, 3))
        self.view_canvas.create_text(z_top[0], z_top[1] - 10, text="+Z", fill="#000000")
        self.view_canvas.create_text(z_bottom[0], z_bottom[1] + 10, text="-Z", fill="#000000")
        self._draw_gravity_vector(PROJECTION_3D, radius)
        self._draw_current_frame(PROJECTION_3D, radius)
        self._draw_derived_frames(PROJECTION_3D, radius)
        if self._current_mag_vector is not None and self._source_cards["raw_accelerometer"].show_var.get():
            self._draw_current_point(self._project_scene_point(*self._current_mag_vector, projection=PROJECTION_3D), color=COLOR_RED, size=5)
        self._draw_derived_points(PROJECTION_3D)

    def _draw_xy_projection(self) -> None:
        radius = self._display_reference_radius()
        self._draw_plane_axes(PROJECTION_XY, radius * 1.05)
        self._draw_dataset_clouds(PROJECTION_XY)
        self._draw_gravity_vector(PROJECTION_XY, radius)
        self._draw_current_frame(PROJECTION_XY, radius)
        self._draw_derived_frames(PROJECTION_XY, radius)
        if self._current_mag_vector is not None and self._source_cards["raw_accelerometer"].show_var.get():
            self._draw_current_point(self._project_scene_point(*self._current_mag_vector, projection=PROJECTION_XY), color=COLOR_RED, size=5)
        self._draw_derived_points(PROJECTION_XY)

    def _draw_xz_projection(self) -> None:
        radius = self._display_reference_radius()
        self._draw_plane_axes(PROJECTION_XZ, radius * 1.05)
        self._draw_dataset_clouds(PROJECTION_XZ)
        self._draw_gravity_vector(PROJECTION_XZ, radius)
        self._draw_current_frame(PROJECTION_XZ, radius)
        self._draw_derived_frames(PROJECTION_XZ, radius)
        if self._current_mag_vector is not None and self._source_cards["raw_accelerometer"].show_var.get():
            self._draw_current_point(self._project_scene_point(*self._current_mag_vector, projection=PROJECTION_XZ), color=COLOR_RED, size=5)
        self._draw_derived_points(PROJECTION_XZ)

    def _draw_yz_projection(self) -> None:
        radius = self._display_reference_radius()
        self._draw_plane_axes(PROJECTION_YZ, radius * 1.05)
        self._draw_dataset_clouds(PROJECTION_YZ)
        self._draw_gravity_vector(PROJECTION_YZ, radius)
        self._draw_current_frame(PROJECTION_YZ, radius)
        self._draw_derived_frames(PROJECTION_YZ, radius)
        if self._current_mag_vector is not None and self._source_cards["raw_accelerometer"].show_var.get():
            self._draw_current_point(self._project_scene_point(*self._current_mag_vector, projection=PROJECTION_YZ), color=COLOR_RED, size=5)
        self._draw_derived_points(PROJECTION_YZ)

    def _heading_legend_items(self) -> list[tuple[str, str]]:
        items: list[tuple[str, str]] = []
        if self._source_cards.get("raw_tilt") is not None and self._source_cards["raw_tilt"].show_var.get():
            items.append(("Raw Tilt", "#111111"))
        if self.view_option_vars["show derived frames"].get():
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
        box_w = 200.0
        box_h = 24.0 + row_h * len(items)
        canvas.create_rectangle(x0, y0, x0 + box_w, y0 + box_h, fill="#f7f7f7", outline="#909090")
        canvas.create_text(x0 + 10, y0 + 12, text="Output legend", anchor="w", fill="#202020")
        primary_label = self.primary_heading_var.get()
        for idx, (label, color) in enumerate(items):
            y = y0 + 28.0 + idx * row_h
            canvas.create_line(x0 + 10, y, x0 + 34, y, fill=color, width=3)
            legend_label = f"{label} [primary]" if label == primary_label else label
            canvas.create_text(x0 + 42, y, text=legend_label, anchor="w", fill="#202020")

    def update_live_imu(
        self,
        *,
        timestamp_mcu: int,
        timestamp_pc_rx: int,
        timestamp_pc_est: int | None,
        ax: float,
        ay: float,
        az: float,
        selected_roll_deg: float | None = None,
        selected_pitch_deg: float | None = None,
        selected_source_label: str | None = None,
        derived_streams: dict[str, dict[str, object]] | None = None,
    ) -> None:
        magnitude = math.sqrt(ax * ax + ay * ay + az * az)
        raw_roll_deg, raw_pitch_deg = compute_accelerometer_tilt_deg(ax, ay, az)

        self.current_data_vars["MCU ts"].set(str(timestamp_mcu))
        self.current_data_vars["PC rx ts"].set(str(timestamp_pc_rx))
        self.current_data_vars["PC est ts"].set("—" if timestamp_pc_est is None else str(timestamp_pc_est))
        self.current_data_vars["ax"].set(f"{ax:.3f}")
        self.current_data_vars["ay"].set(f"{ay:.3f}")
        self.current_data_vars["az"].set(f"{az:.3f}")
        self.current_data_vars["|a|"].set(f"{magnitude:.3f}")
        self.current_data_vars["roll"].set("—" if selected_roll_deg is None else f"{selected_roll_deg:.1f}°")
        self.current_data_vars["pitch"].set("—" if selected_pitch_deg is None else f"{selected_pitch_deg:.1f}°")
        self.current_data_vars["selected stream"].set(selected_source_label or "Raw Tilt")
        self._update_dataset_status()

        first_sample = self._current_mag_vector is None
        self._current_mag_vector = (ax, ay, az)
        self._current_tilt = (raw_roll_deg, raw_pitch_deg)
        self._derived_streams = {
            method_id: {
                "title": str(stream_state.get("title", method_id)),
                "stream_id": str(stream_state.get("stream_id", f"derived_{method_id}")),
                "mx": float(stream_state.get("acc_x", stream_state.get("mx", 0.0))),
                "my": float(stream_state.get("acc_y", stream_state.get("my", 0.0))),
                "mz": float(stream_state.get("acc_z", stream_state.get("mz", 0.0))),
                "roll_deg": stream_state.get("roll_deg"),
                "pitch_deg": stream_state.get("pitch_deg"),
                "show": bool(stream_state.get("show", False)),
                "color": stream_state.get("color", self._stream_color(method_id)),
            }
            for method_id, stream_state in (derived_streams or {}).items()
        }
        self._update_heading_stream_summary()
        should_redraw = self._can_redraw_live_view(first_sample=first_sample)
        if should_redraw and self.auto_fit_var.get():
            self._sync_auto_fit_radius()
        if should_redraw:
            self._redraw_view()

    def set_primary_output_display(
        self,
        *,
        source_label: str,
        roll_deg: float | None = None,
        pitch_deg: float | None = None,
    ) -> None:
        self.current_data_vars["selected stream"].set(source_label)
        if roll_deg is not None:
            self.current_data_vars["roll"].set(f"{roll_deg:.1f}°")
        if pitch_deg is not None:
            self.current_data_vars["pitch"].set(f"{pitch_deg:.1f}°")

    def _update_heading_stream_summary(self) -> None:
        visible_names: list[str] = []
        if self._source_cards.get("raw_tilt") is not None and self._source_cards["raw_tilt"].show_var.get():
            visible_names.append("Raw Tilt")
        if self.view_option_vars["show derived frames"].get():
            for method_id, stream in self._derived_streams.items():
                if not bool(stream.get("show", False)):
                    continue
                visible_names.append(self._method_titles.get(method_id, str(stream.get("title", method_id))))
        self.visible_heading_streams_var.set(", ".join(visible_names) if visible_names else "None")

    def _record_radius(self, record: AccelerometerSampleRecord) -> float:
        ax = float(record.acc_x)
        ay = float(record.acc_y)
        az = float(record.acc_z)
        return max(abs(ax), abs(ay), abs(az), math.sqrt(ax * ax + ay * ay + az * az))

    def _reference_radius(self) -> float:
        radius = max(self._dataset_cloud_max_radius, self._method_dataset_cloud_max_radius)
        raw_card = self._source_cards.get("raw_accelerometer")
        if self._current_mag_vector is not None and raw_card is not None and raw_card.show_var.get():
            ax, ay, az = self._current_mag_vector
            radius = max(radius, abs(ax), abs(ay), abs(az), math.sqrt(ax * ax + ay * ay + az * az))
        for stream in self._iter_visible_derived_streams():
            ax = float(stream["mx"])
            ay = float(stream["my"])
            az = float(stream["mz"])
            radius = max(radius, abs(ax), abs(ay), abs(az), math.sqrt(ax * ax + ay * ay + az * az))
        return radius

    def _insert_dataset_row(self, row_id: int, record: AccelerometerSampleRecord) -> None:
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
                f"{record.acc_x:.6g}",
                f"{record.acc_y:.6g}",
                f"{record.acc_z:.6g}",
                f"{record.roll_deg:.3f}" if record.roll_deg is not None else "—",
                f"{record.pitch_deg:.3f}" if record.pitch_deg is not None else "—",
                record.flags,
            ),
        )

    def set_metrics_report(self, *, rows: list[dict[str, str]], summary_text: str, can_export: bool) -> None:
        del rows, summary_text, can_export
        self.metrics_status_var.set("Accelerometer metrics are not available yet.")
        self.export_metrics_btn.configure(state="disabled")
