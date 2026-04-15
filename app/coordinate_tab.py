"""
Coordinate tab:
- Scaled grid for trajectory planning
- Bezier curve editor with anchors/handles
- Minimum turning radius validation
"""

from __future__ import annotations

import os
import math
from dataclasses import dataclass
import queue
import threading
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from typing import Optional, Callable, Any

try:
    from PIL import Image, ImageTk, UnidentifiedImageError
except Exception:  # pragma: no cover - optional dependency guard
    Image = None
    ImageTk = None
    UnidentifiedImageError = OSError

from app.styles import PANEL_BG, COLOR_GREEN, COLOR_RED
from utils.timebase import now_ms_monotonic, u32_diff_mod
from utils.bezier_math import (
    cubic_point,
    cubic_derivative,
    curvature_signed,
    min_curve_radius,
)
from utils.motion_math import MotionParams, build_path_samples, plan_profile
from utils.speed_map import SpeedMapPoint, speed_to_pwm, pwm_to_speed
from utils.pure_pursuit_controller import (
    ControllerConfig,
    Pose2D as ControllerPose2D,
    PurePursuitController,
    SimplePolylineTrack,
)
from app.dialogs import SpeedMapConfig
from runtime.contracts import (
    AutopilotMode,
    BuiltinAutopilotTuning,
    DriveCommand,
    GeometrySettings,
    MissionConfig,
    MotionSettings,
    PoseEstimate,
    PoseSourceMode,
    PwmCorrection,
    SpeedMapEntry,
    TelemetrySubscription,
    TrackPoint,
)


@dataclass
class BezierNode:
    anchor: tuple[float, float]
    handle_in: Optional[tuple[float, float]] = None   # offset from anchor
    handle_out: Optional[tuple[float, float]] = None  # offset from anchor


@dataclass(frozen=True)
class _MapRenderSpec:
    canvas_key: str
    draw_w: int
    draw_h: int
    angle_deg: float
    angle_key: int

    @property
    def cache_key(self) -> tuple[str, int, int, int]:
        return (self.canvas_key, self.draw_w, self.draw_h, self.angle_key)


POSE_SOURCE_LABELS: dict[PoseSourceMode, str] = {
    PoseSourceMode.INTERNAL: "Internal integrator",
    PoseSourceMode.TROLLEY: "Trolley data",
    PoseSourceMode.EXTERNAL: "External estimator",
}

POSE_SOURCE_ALIASES: dict[str, PoseSourceMode] = {
    PoseSourceMode.INTERNAL.value: PoseSourceMode.INTERNAL,
    PoseSourceMode.TROLLEY.value: PoseSourceMode.TROLLEY,
    PoseSourceMode.EXTERNAL.value: PoseSourceMode.EXTERNAL,
    "Internal integrator": PoseSourceMode.INTERNAL,
    "Trolley data": PoseSourceMode.TROLLEY,
    "External estimator": PoseSourceMode.EXTERNAL,
}

AUTOPILOT_LABELS: dict[AutopilotMode, str] = {
    AutopilotMode.BUILTIN_PURE_PURSUIT: "Built-in Pure Pursuit",
    AutopilotMode.EXTERNAL: "External autopilot",
}

AUTOPILOT_ALIASES: dict[str, AutopilotMode] = {
    AutopilotMode.BUILTIN_PURE_PURSUIT.value: AutopilotMode.BUILTIN_PURE_PURSUIT,
    AutopilotMode.EXTERNAL.value: AutopilotMode.EXTERNAL,
    "Built-in Pure Pursuit": AutopilotMode.BUILTIN_PURE_PURSUIT,
    "External autopilot": AutopilotMode.EXTERNAL,
}


class CoordinateTab(ttk.Frame):
    def __init__(
        self,
        master: tk.Widget,
        on_start: Optional[Callable[[], None]] = None,
        on_stop: Optional[Callable[[], None]] = None,
        on_state_change: Optional[Callable[[], None]] = None,
        dynamic_route_views: bool = True,
    ) -> None:
        super().__init__(master, padding=6)

        self.on_start = on_start
        self.on_stop = on_stop
        self.on_state_change = on_state_change
        self._dynamic_route_views = bool(dynamic_route_views)
        self._grid_cells = 14
        self._canvas_size = 420
        self._magnifier_zoom = 4.0
        self._hit_radius = 8
        self._handle_len = 60
        self._samples_per_seg = 60
        self._hover_px = 8
        self._view_scale = 1.0
        self._view_pan = (0.0, 0.0)
        self._pan_start: Optional[tuple[float, float]] = None
        self._pan_origin: Optional[tuple[float, float]] = None
        self._pan_canvas: Optional[tk.Canvas] = None
        self._map_path = ""
        self._map_image = None
        self._map_image_cache: dict[tuple[str, int, int, int], ImageTk.PhotoImage] = {}
        self._map_render_generation = 0
        self._map_render_preview = False
        self._map_render_idle_ms = 180
        self._map_render_after_id: str | None = None
        self._map_render_lock = threading.Lock()
        self._map_render_request: Optional[tuple[int, Any, tuple[_MapRenderSpec, ...]]] = None
        self._map_render_results: "queue.Queue[tuple[int, _MapRenderSpec, str, Any]]" = queue.Queue()
        self._map_render_wake = threading.Event()
        self._map_render_stop = threading.Event()
        self._map_render_thread: Optional[threading.Thread] = None
        self._map_render_stage_order: tuple[tuple[str, float], ...] = (
            ("low", 0.25),
            ("medium", 0.5),
            ("full", 1.0),
        )
        self._map_center = self._center()
        self._map_drag_start: Optional[tuple[float, float]] = None
        self._map_drag_origin: Optional[tuple[float, float]] = None
        self._map_drag_canvas: Optional[tk.Canvas] = None
        self._map_drag_moved = False
        self._map_drag_threshold_px = 3.0
        self._map_click_add_point: Optional[tuple[float, float]] = None

        self._a1_cm = 20.0
        self._a2_cm = 20.0
        self._drive_wheel_diameter_cm = 10.0
        self._v_cm = 0.5
        self._accel = 0.5
        self._decel = 0.5
        self._dt_ms = 10.0
        self._speed_map = SpeedMapConfig()
        self._pwm_left = 1500.0
        self._pwm_right = 1500.0
        self._track_circumference_m = math.pi * self._drive_wheel_diameter_cm / 100.0
        self._editable = True
        self._lookahead_min_m = 0.25
        self._lookahead_max_m = 1.50
        self._lookahead_k = 0.60
        self._kappa_slowdown = 1.50

        self._log_max_lines = 500
        self._log_lines = 0
        self._log_filters: dict[str, tk.BooleanVar] = {}
        self._filter_desc: dict[str, str] = {}
        self._menu_tooltip: _MenuTooltip | None = None

        self._scale_var = tk.StringVar(value="1 m")
        self._min_radius_var = tk.StringVar(value="50")
        self._v_cm_var = tk.StringVar(value=str(self._v_cm))
        self._accel_var = tk.StringVar(value=str(self._accel))
        self._decel_var = tk.StringVar(value=str(self._decel))
        self._dt_var = tk.StringVar(value=str(self._dt_ms))
        self._pwm_left_var = tk.StringVar(value=str(int(self._pwm_left)))
        self._pwm_right_var = tk.StringVar(value=str(int(self._pwm_right)))
        self._left_shift_var = tk.StringVar(value="0")
        self._right_shift_var = tk.StringVar(value="0")
        self._left_linear_var = tk.StringVar(value="1")
        self._right_linear_var = tk.StringVar(value="1")
        self._map_file_var = tk.StringVar(value="")
        self._map_image_scale_var = tk.DoubleVar(value=1.0)
        self._map_image_scale_text_var = tk.StringVar(value="1.00")
        self._map_rotation_var = tk.DoubleVar(value=0.0)
        self._map_rotation_text_var = tk.StringVar(value="0.0")
        self._pose_source_var = tk.StringVar(value=POSE_SOURCE_LABELS[PoseSourceMode.INTERNAL])
        self._autopilot_var = tk.StringVar(value=AUTOPILOT_LABELS[AutopilotMode.BUILTIN_PURE_PURSUIT])

        self._drag_target: Optional[tuple[str, int]] = None
        self._state_notify_after_id: str | None = None

        self._build()
        self.bind("<Destroy>", self._on_destroy, add="+")
        self.after(40, self._poll_map_render_results)
        self._init_nodes()
        self._redraw()

    # ---------------- UI ----------------

    def _build(self) -> None:
        self.columnconfigure(0, weight=1 if self._dynamic_route_views else 0)
        self.columnconfigure(1, weight=0 if self._dynamic_route_views else 1)
        self.rowconfigure(0, weight=1)
        self.rowconfigure(1, weight=0)

        left = ttk.Frame(self)
        left.grid(row=0, column=0, sticky="nsew" if self._dynamic_route_views else "nw")
        if self._dynamic_route_views:
            left.columnconfigure(0, weight=1)
            left.rowconfigure(0, weight=1)

        route_views = ttk.Frame(left)
        route_views.grid(row=0, column=0, sticky="nsew" if self._dynamic_route_views else "nw")
        if self._dynamic_route_views:
            route_views.columnconfigure(0, weight=1)
            route_views.rowconfigure(0, weight=1)

        self.canvas = tk.Canvas(
            route_views,
            width=self._canvas_size,
            height=self._canvas_size,
            bg=PANEL_BG,
            highlightthickness=1,
            highlightbackground="#606060",
        )
        self.canvas.grid(row=0, column=0, sticky="nsew" if self._dynamic_route_views else "nw")

        self.magnifier_canvas: Optional[tk.Canvas] = None
        self._view_canvas_zoom: dict[tk.Canvas, float] = {
            self.canvas: 1.0,
        }
        if self._dynamic_route_views:
            magnifier_row = ttk.Frame(route_views)
            magnifier_row.grid(row=1, column=0, sticky="w", pady=(8, 0))
            self.magnifier_canvas = tk.Canvas(
                magnifier_row,
                width=self._canvas_size // 2,
                height=self._canvas_size // 2,
                bg=PANEL_BG,
                highlightthickness=1,
                highlightbackground="#606060",
            )
            self.magnifier_canvas.grid(row=0, column=0, sticky="nw")
            self._view_canvas_zoom[self.magnifier_canvas] = self._magnifier_zoom
            self._bind_route_canvas(self.canvas, self._on_main_canvas_configure)
            self._bind_route_canvas(self.magnifier_canvas, self._on_magnifier_canvas_configure)
        else:
            self._bind_route_canvas(self.canvas)

        scale_row = ttk.Frame(left)
        scale_row.grid(row=1, column=0, sticky="w", pady=(6, 0))
        ttk.Label(scale_row, text="Scale").pack(side="left")
        self.scale_box = ttk.Combobox(
            scale_row,
            textvariable=self._scale_var,
            values=["10 cm", "20 cm", "50 cm", "1 m", "2 m", "5 m", "10 m", "15 m", "25 m", "50 m"],
            width=6,
            state="readonly",
        )
        self.scale_box.pack(side="left", padx=(6, 0))
        self.scale_box.bind("<<ComboboxSelected>>", lambda _e: self._redraw())

        ttk.Label(scale_row, text="Map").pack(side="left", padx=(12, 0))
        self.map_file_entry = ttk.Entry(
            scale_row,
            textvariable=self._map_file_var,
            width=26,
            state="readonly",
        )
        self.map_file_entry.pack(side="left", padx=(6, 0))
        ttk.Button(scale_row, text="...", width=3, command=self._choose_map_file).pack(side="left", padx=(4, 0))

        map_scale_row = ttk.Frame(left)
        map_scale_row.grid(row=2, column=0, sticky="w", pady=(4, 0))
        ttk.Label(map_scale_row, text="Map scale").pack(side="left")
        self.map_scale_slider = ttk.Scale(
            map_scale_row,
            from_=0.1,
            to=6.0,
            variable=self._map_image_scale_var,
            orient="horizontal",
            length=190,
            command=self._on_map_scale_change,
        )
        self.map_scale_slider.pack(side="left", padx=(8, 6))
        self.map_scale_entry = ttk.Entry(map_scale_row, textvariable=self._map_image_scale_text_var, width=7)
        self.map_scale_entry.pack(side="left")
        self.map_scale_entry.bind("<Return>", self._commit_map_scale_text)
        self.map_scale_entry.bind("<FocusOut>", self._commit_map_scale_text)
        ttk.Label(map_scale_row, text="x").pack(side="left", padx=(4, 0))
        self.map_scale_slider.state(["disabled"])
        self.map_scale_entry.state(["disabled"])

        map_rot_row = ttk.Frame(left)
        map_rot_row.grid(row=3, column=0, sticky="w", pady=(4, 0))
        ttk.Label(map_rot_row, text="Map rotation").pack(side="left")
        self.map_rot_slider = ttk.Scale(
            map_rot_row,
            from_=0.0,
            to=360.0,
            variable=self._map_rotation_var,
            orient="horizontal",
            length=190,
            command=self._on_map_rotation_change,
        )
        self.map_rot_slider.pack(side="left", padx=(8, 6))
        self.map_rot_entry = ttk.Entry(map_rot_row, textvariable=self._map_rotation_text_var, width=7)
        self.map_rot_entry.pack(side="left")
        self.map_rot_entry.bind("<Return>", self._commit_map_rotation_text)
        self.map_rot_entry.bind("<FocusOut>", self._commit_map_rotation_text)
        ttk.Label(map_rot_row, text="deg").pack(side="left", padx=(4, 0))
        self.map_rot_slider.state(["disabled"])
        self.map_rot_entry.state(["disabled"])

        right = ttk.Frame(self)
        right.grid(row=0, column=1, sticky="nw", padx=(12, 0))
        right.columnconfigure(0, weight=1)

        min_row = ttk.Frame(right)
        min_row.grid(row=0, column=0, sticky="w", pady=(10, 0))
        ttk.Label(min_row, text="Minimal rotation radius").pack(side="left")
        min_entry = ttk.Entry(min_row, textvariable=self._min_radius_var, width=6)
        min_entry.pack(side="left", padx=(8, 4))
        ttk.Label(min_row, text="cm").pack(side="left")
        self._min_radius_var.trace_add("write", lambda *_: self._redraw())

        speed_row = ttk.Frame(right)
        speed_row.grid(row=1, column=0, sticky="w", pady=(10, 0))
        ttk.Label(speed_row, text="Speed (CM)").pack(side="left")
        ttk.Entry(speed_row, textvariable=self._v_cm_var, width=6).pack(side="left", padx=(8, 4))
        ttk.Label(speed_row, text="m/s").pack(side="left")

        accel_row = ttk.Frame(right)
        accel_row.grid(row=2, column=0, sticky="w", pady=(6, 0))
        ttk.Label(accel_row, text="Accel").pack(side="left")
        ttk.Entry(accel_row, textvariable=self._accel_var, width=6).pack(side="left", padx=(8, 4))
        ttk.Label(accel_row, text="m/s^2").pack(side="left")

        decel_row = ttk.Frame(right)
        decel_row.grid(row=3, column=0, sticky="w", pady=(6, 0))
        ttk.Label(decel_row, text="Brake").pack(side="left")
        ttk.Entry(decel_row, textvariable=self._decel_var, width=6).pack(side="left", padx=(8, 4))
        ttk.Label(decel_row, text="m/s^2").pack(side="left")

        dt_row = ttk.Frame(right)
        dt_row.grid(row=4, column=0, sticky="w", pady=(6, 0))
        ttk.Label(dt_row, text="Time quantum").pack(side="left")
        ttk.Entry(dt_row, textvariable=self._dt_var, width=6).pack(side="left", padx=(8, 4))
        ttk.Label(dt_row, text="ms").pack(side="left")

        for var in [self._v_cm_var, self._accel_var, self._decel_var, self._dt_var]:
            var.trace_add("write", lambda *_: self._redraw())

        pwm_block = ttk.Frame(right)
        pwm_block.grid(row=5, column=0, sticky="w", pady=(12, 0))

        ttk.Label(pwm_block, text="Left").grid(row=0, column=0, sticky="w")
        ttk.Entry(pwm_block, textvariable=self._pwm_left_var, width=8, state="readonly").grid(row=0, column=1, sticky="w", padx=(6, 8))
        ttk.Label(pwm_block, text="Shift").grid(row=0, column=2, sticky="w")
        ttk.Entry(pwm_block, textvariable=self._left_shift_var, width=6).grid(row=0, column=3, sticky="w", padx=(6, 8))
        ttk.Label(pwm_block, text="Linear").grid(row=0, column=4, sticky="w")
        ttk.Entry(pwm_block, textvariable=self._left_linear_var, width=6).grid(row=0, column=5, sticky="w")

        ttk.Label(pwm_block, text="Right").grid(row=1, column=0, sticky="w", pady=(6, 0))
        ttk.Entry(pwm_block, textvariable=self._pwm_right_var, width=8, state="readonly").grid(row=1, column=1, sticky="w", padx=(6, 8), pady=(6, 0))
        ttk.Label(pwm_block, text="Shift").grid(row=1, column=2, sticky="w", pady=(6, 0))
        ttk.Entry(pwm_block, textvariable=self._right_shift_var, width=6).grid(row=1, column=3, sticky="w", padx=(6, 8), pady=(6, 0))
        ttk.Label(pwm_block, text="Linear").grid(row=1, column=4, sticky="w", pady=(6, 0))
        ttk.Entry(pwm_block, textvariable=self._right_linear_var, width=6).grid(row=1, column=5, sticky="w", pady=(6, 0))

        for var in [self._left_shift_var, self._right_shift_var, self._left_linear_var, self._right_linear_var]:
            var.trace_add("write", lambda *_: self._update_pwm_display())

        btn_row = ttk.Frame(right)
        btn_row.grid(row=6, column=0, sticky="w", pady=(14, 0))
        self.start_btn = ttk.Button(btn_row, text="Start", width=14, command=self._on_start_click)
        self.stop_btn = ttk.Button(btn_row, text="Stop", width=14, command=self._on_stop_click, state="disabled")
        self.start_btn.pack(side="left", padx=(0, 10))
        self.stop_btn.pack(side="left")

        pose_row = ttk.Frame(right)
        pose_row.grid(row=7, column=0, sticky="w", pady=(8, 0))
        ttk.Label(pose_row, text="Pose input").pack(side="left")
        self.pose_source_box = ttk.Combobox(
            pose_row,
            textvariable=self._pose_source_var,
            values=[POSE_SOURCE_LABELS[item] for item in PoseSourceMode],
            width=20,
            state="readonly",
        )
        self.pose_source_box.pack(side="left", padx=(8, 0))
        self.pose_source_box.bind("<<ComboboxSelected>>", lambda _e: self._notify_state_change())

        autopilot_row = ttk.Frame(right)
        autopilot_row.grid(row=8, column=0, sticky="w", pady=(6, 0))
        ttk.Label(autopilot_row, text="Autopilot").pack(side="left")
        self.autopilot_box = ttk.Combobox(
            autopilot_row,
            textvariable=self._autopilot_var,
            values=[AUTOPILOT_LABELS[item] for item in AutopilotMode],
            width=20,
            state="readonly",
        )
        self.autopilot_box.pack(side="left", padx=(12, 0))
        self.autopilot_box.bind("<<ComboboxSelected>>", lambda _e: self._notify_state_change())

        self.bottom_nb = ttk.Notebook(self)
        self.bottom_nb.grid(row=1, column=0, columnspan=2, sticky="nsew", pady=(6, 0))
        self._build_log_tab()
        self._build_rpm_tab()

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
        self.rpm_canvas.bind("<Motion>", self._on_rpm_motion)
        self.rpm_canvas.bind("<Leave>", self._on_rpm_leave)

    # ---------------- Model ----------------

    def _init_nodes(self) -> None:
        cx, cy = self._center()
        self.nodes: list[BezierNode] = [
            BezierNode(anchor=(cx, cy), handle_out=(0.0, -self._handle_len))
        ]
        self._samples: list[dict[str, float]] = []
        self._profile_s: list[float] = []
        self._profile_v_left: list[float] = []
        self._profile_v_right: list[float] = []
        self._hover_s: Optional[float] = None
        self._rpm_view: dict[str, float] = {}
        self._expected_poses: list[tuple[float, float, float]] = []
        self._expected_world: Optional[tuple[float, float, float]] = None
        self._expected_start_ms: Optional[int] = None
        self._expected_dt_s: float = 0.0
        self._expected_running = False
        self._expected_cmds: list[tuple[int, int]] = []
        self._controller: Optional[PurePursuitController] = None
        self._controller_track: Optional[SimplePolylineTrack] = None
        self._actual_s: list[float] = []
        self._actual_v_left: list[float] = []
        self._actual_v_right: list[float] = []
        self._last_actual_ts_ms: Optional[int] = None
        self._actual_pose: Optional[tuple[float, float, float]] = None
        self._actual_world: Optional[tuple[float, float, float]] = None
        self._external_pose: Optional[PoseEstimate] = None
        self._external_world: Optional[tuple[float, float, float]] = None
        self._suspend_state_notify = False

    def _center(self) -> tuple[float, float]:
        half = self._canvas_size / 2.0
        return (half, half)

    def _bind_route_canvas(
        self,
        canvas: tk.Canvas,
        on_configure: Optional[Callable[[tk.Event], None]] = None,
    ) -> None:
        canvas.bind("<ButtonPress-1>", self._on_press)
        canvas.bind("<B1-Motion>", self._on_drag)
        canvas.bind("<ButtonRelease-1>", self._on_release)
        canvas.bind("<Double-Button-1>", self._on_double_click)
        canvas.bind("<Motion>", self._on_motion)
        canvas.bind("<ButtonPress-3>", self._on_pan_start)
        canvas.bind("<B3-Motion>", self._on_pan_drag)
        canvas.bind("<ButtonRelease-3>", self._on_pan_end)
        canvas.bind("<MouseWheel>", self._on_zoom)
        canvas.bind("<Button-4>", self._on_zoom)
        canvas.bind("<Button-5>", self._on_zoom)
        if on_configure is not None:
            canvas.bind("<Configure>", on_configure)

    def _route_canvases(self) -> tuple[tk.Canvas, ...]:
        if self.magnifier_canvas is None:
            return (self.canvas,)
        return (self.canvas, self.magnifier_canvas)

    def _resolve_canvas(self, canvas: Optional[tk.Canvas]) -> tk.Canvas:
        return self.canvas if canvas is None else canvas

    def _canvas_dimensions(self, canvas: Optional[tk.Canvas] = None) -> tuple[float, float]:
        current = self._resolve_canvas(canvas)
        width = int(current.winfo_width())
        height = int(current.winfo_height())
        if width <= 1:
            width = int(float(current.cget("width")))
        if height <= 1:
            height = int(float(current.cget("height")))
        return (float(max(width, 1)), float(max(height, 1)))

    def _canvas_zoom(self, canvas: Optional[tk.Canvas] = None) -> float:
        current = self._resolve_canvas(canvas)
        return float(self._view_canvas_zoom.get(current, 1.0))

    def _canvas_fit_scale(self, canvas: Optional[tk.Canvas] = None) -> float:
        width, height = self._canvas_dimensions(canvas)
        return min(width, height) / self._canvas_size

    def _canvas_display_scale(self, canvas: Optional[tk.Canvas] = None) -> float:
        return max(1e-9, self._view_scale * self._canvas_fit_scale(canvas) * self._canvas_zoom(canvas))

    def _canvas_screen_center(self, canvas: Optional[tk.Canvas] = None) -> tuple[float, float]:
        width, height = self._canvas_dimensions(canvas)
        return (width * 0.5, height * 0.5)

    def _canvas_key(self, canvas: tk.Canvas) -> str:
        return "main" if canvas is self.canvas else "magnifier"

    def _canvas_for_key(self, canvas_key: str) -> Optional[tk.Canvas]:
        if canvas_key == "main":
            return self.canvas
        if canvas_key == "magnifier":
            return self.magnifier_canvas
        return None

    def _map_angle_key(self) -> int:
        return int(round((self._map_rotation_deg() % 360.0) * 10.0))

    def _map_render_spec(self, canvas: tk.Canvas) -> Optional[_MapRenderSpec]:
        model_size = self._map_model_size()
        if model_size is None:
            return None
        draw_scale = self._canvas_display_scale(canvas)
        draw_w = max(1, int(round(model_size[0] * draw_scale)))
        draw_h = max(1, int(round(model_size[1] * draw_scale)))
        return _MapRenderSpec(
            canvas_key=self._canvas_key(canvas),
            draw_w=draw_w,
            draw_h=draw_h,
            angle_deg=self._map_rotation_deg(),
            angle_key=self._map_angle_key(),
        )

    def _to_view(self, pt: tuple[float, float], canvas: Optional[tk.Canvas] = None) -> tuple[float, float]:
        model_cx, model_cy = self._center()
        view_cx, view_cy = self._canvas_screen_center(canvas)
        pan_x, pan_y = self._view_pan
        scale = self._canvas_display_scale(canvas)
        return (
            view_cx + (pt[0] - model_cx + pan_x) * scale,
            view_cy + (pt[1] - model_cy + pan_y) * scale,
        )

    def _from_view(self, pt: tuple[float, float], canvas: Optional[tk.Canvas] = None) -> tuple[float, float]:
        model_cx, model_cy = self._center()
        view_cx, view_cy = self._canvas_screen_center(canvas)
        pan_x, pan_y = self._view_pan
        scale = self._canvas_display_scale(canvas)
        return (
            (pt[0] - view_cx) / scale - pan_x + model_cx,
            (pt[1] - view_cy) / scale - pan_y + model_cy,
        )

    def _cell_px(self) -> float:
        return self._canvas_size / self._grid_cells

    def _meters_per_px(self) -> float:
        return self._scale_m() / self._cell_px()

    def _scale_m(self) -> float:
        raw = self._scale_var.get().strip().lower()
        if raw.endswith("cm"):
            return float(raw.replace("cm", "").strip()) / 100.0
        return float(raw.replace("m", "").strip())

    def _min_radius_m(self) -> Optional[float]:
        try:
            return float(self._min_radius_var.get().strip()) / 100.0
        except ValueError:
            return None

    def _canvas_to_world(self, pt: tuple[float, float]) -> tuple[float, float]:
        cx, cy = self._center()
        meter_per_px = self._meters_per_px()
        return ((pt[0] - cx) * meter_per_px, (cy - pt[1]) * meter_per_px)

    def _meters_to_px(self, meters: float) -> float:
        return meters / self._meters_per_px()

    def _map_scale_factor(self) -> float:
        try:
            value = float(self._map_image_scale_var.get())
        except Exception:
            value = 1.0
        return max(0.1, min(6.0, value))

    def _map_rotation_deg(self) -> float:
        try:
            value = float(self._map_rotation_var.get())
        except Exception:
            value = 0.0
        return max(0.0, min(360.0, value))

    def _sync_map_scale_text(self) -> None:
        self._map_image_scale_text_var.set(f"{self._map_scale_factor():.2f}")

    def _sync_map_rotation_text(self) -> None:
        self._map_rotation_text_var.set(f"{self._map_rotation_deg():.1f}")

    def _clear_map_render_schedule(self) -> None:
        if self._map_render_after_id is None:
            return
        try:
            self.after_cancel(self._map_render_after_id)
        except Exception:
            pass
        self._map_render_after_id = None

    def _invalidate_map_render(self, *, preview: bool) -> None:
        self._map_render_generation += 1
        self._map_render_preview = preview
        self._map_image_cache.clear()
        self._clear_map_render_schedule()
        with self._map_render_lock:
            self._map_render_request = None

    def _schedule_map_render(
        self,
        *,
        preview: bool,
        delay_ms: int,
        redraw: bool,
        notify_state: bool,
    ) -> None:
        if self._map_image is None or Image is None or ImageTk is None:
            return
        self._invalidate_map_render(preview=preview)
        if delay_ms <= 0:
            self._dispatch_map_render_request()
        else:
            self._map_render_after_id = self.after(delay_ms, self._dispatch_map_render_request)
        if redraw:
            self._redraw(recompute_profile=False, notify_state=notify_state)

    def _dispatch_map_render_request(self) -> None:
        self._map_render_after_id = None
        if self._map_image is None or Image is None or ImageTk is None:
            return
        specs = tuple(
            spec
            for spec in (self._map_render_spec(canvas) for canvas in self._route_canvases())
            if spec is not None
        )
        if not specs:
            return
        generation = self._map_render_generation
        self._map_render_preview = False
        self._ensure_map_render_worker()
        with self._map_render_lock:
            self._map_render_request = (generation, self._map_image, specs)
        self._map_render_wake.set()

    def _ensure_map_render_worker(self) -> None:
        if self._map_render_thread is not None and self._map_render_thread.is_alive():
            return
        self._map_render_stop.clear()
        self._map_render_thread = threading.Thread(
            target=self._map_render_worker_loop,
            name="coord-map-render",
            daemon=True,
        )
        self._map_render_thread.start()

    def _map_render_worker_loop(self) -> None:
        while not self._map_render_stop.is_set():
            self._map_render_wake.wait()
            self._map_render_wake.clear()
            if self._map_render_stop.is_set():
                return
            with self._map_render_lock:
                request = self._map_render_request
            if request is None:
                continue
            generation, source_image, specs = request
            for stage_name, stage_scale in self._map_render_stage_order:
                for spec in specs:
                    if self._map_render_is_stale(generation) or self._map_render_stop.is_set():
                        break
                    rendered = self._render_map_stage(source_image, spec, stage_scale)
                    if rendered is None:
                        continue
                    if self._map_render_is_stale(generation) or self._map_render_stop.is_set():
                        break
                    self._map_render_results.put((generation, spec, stage_name, rendered))
                else:
                    continue
                break

    def _map_render_is_stale(self, generation: int) -> bool:
        return generation != self._map_render_generation

    def _render_map_stage(self, source_image: Any, spec: _MapRenderSpec, stage_scale: float) -> Any:
        if Image is None:
            return None
        draw_w = max(1, int(spec.draw_w))
        draw_h = max(1, int(spec.draw_h))
        work_w = max(1, int(round(draw_w * stage_scale)))
        work_h = max(1, int(round(draw_h * stage_scale)))
        angle_deg = float(spec.angle_deg)
        try:
            resized = source_image.resize((work_w, work_h), Image.Resampling.BILINEAR)
            if spec.angle_key != 0:
                rotated = resized.rotate(-angle_deg, resample=Image.Resampling.BILINEAR, expand=True)
            else:
                rotated = resized
            if stage_scale >= 0.999:
                return rotated
            angle_rad = math.radians(angle_deg)
            cos_a = abs(math.cos(angle_rad))
            sin_a = abs(math.sin(angle_rad))
            final_w = max(1, int(round(draw_w * cos_a + draw_h * sin_a)))
            final_h = max(1, int(round(draw_w * sin_a + draw_h * cos_a)))
            return rotated.resize((final_w, final_h), Image.Resampling.BILINEAR)
        except Exception:
            return None

    def _poll_map_render_results(self) -> None:
        if self._map_render_stop.is_set():
            return
        changed = False
        while True:
            try:
                generation, spec, _stage_name, rendered = self._map_render_results.get_nowait()
            except queue.Empty:
                break
            if generation != self._map_render_generation:
                continue
            if ImageTk is None:
                continue
            try:
                photo = ImageTk.PhotoImage(rendered)
            except Exception:
                continue
            self._map_image_cache[spec.cache_key] = photo
            changed = True
        if changed:
            self._redraw(recompute_profile=False, notify_state=False)
        if self.winfo_exists():
            self.after(40, self._poll_map_render_results)

    def _draw_map_placeholder(self, canvas: tk.Canvas, spec: _MapRenderSpec) -> None:
        cx, cy = self._to_view(self._map_center, canvas)
        half_w = spec.draw_w * 0.5
        half_h = spec.draw_h * 0.5
        angle_rad = math.radians(spec.angle_deg)
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        pts: list[float] = []
        for lx, ly in ((-half_w, -half_h), (half_w, -half_h), (half_w, half_h), (-half_w, half_h)):
            dx = lx * cos_a - ly * sin_a
            dy = lx * sin_a + ly * cos_a
            pts.extend([cx + dx, cy + dy])
        canvas.create_polygon(
            pts,
            outline="#6a6a6a",
            fill="#d9d9d9",
            width=2,
            dash=(4, 3),
            tags="map",
        )
        canvas.create_line(pts[0], pts[1], pts[4], pts[5], fill="#909090", dash=(3, 3), tags="map")
        canvas.create_line(pts[2], pts[3], pts[6], pts[7], fill="#909090", dash=(3, 3), tags="map")
        canvas.create_text(cx, cy, text="Map preview", fill="#505050", tags="map")

    def _on_destroy(self, event: tk.Event) -> None:
        if event.widget is not self:
            return
        self._clear_map_render_schedule()
        self._map_render_stop.set()
        self._map_render_wake.set()

    def _map_model_size(self) -> Optional[tuple[float, float]]:
        if self._map_image is None:
            return None
        scale = self._map_scale_factor()
        return (float(self._map_image.width) * scale, float(self._map_image.height) * scale)

    def _map_model_rect(self) -> Optional[tuple[float, float, float, float]]:
        size = self._map_model_size()
        if size is None:
            return None
        cx, cy = self._map_center
        half_w = size[0] * 0.5
        half_h = size[1] * 0.5
        return (cx - half_w, cy - half_h, cx + half_w, cy + half_h)

    def _map_view_rect(self) -> Optional[tuple[float, float, float, float]]:
        model_size = self._map_model_size()
        if model_size is None:
            return None
        w = model_size[0] * self._canvas_display_scale(self.canvas)
        h = model_size[1] * self._canvas_display_scale(self.canvas)
        if w <= 0.0 or h <= 0.0:
            return None
        angle_rad = math.radians(self._map_rotation_deg())
        cos_a = abs(math.cos(angle_rad))
        sin_a = abs(math.sin(angle_rad))
        box_w = w * cos_a + h * sin_a
        box_h = w * sin_a + h * cos_a
        cx, cy = self._to_view(self._map_center)
        return (cx - box_w * 0.5, cy - box_h * 0.5, cx + box_w * 0.5, cy + box_h * 0.5)

    def _hit_map(self, x: float, y: float, canvas: Optional[tk.Canvas] = None) -> bool:
        model_size = self._map_model_size()
        if model_size is None:
            return False
        w = model_size[0] * self._canvas_display_scale(canvas)
        h = model_size[1] * self._canvas_display_scale(canvas)
        if w <= 0.0 or h <= 0.0:
            return False
        cx, cy = self._to_view(self._map_center, canvas)
        dx = x - cx
        dy = y - cy
        angle_rad = math.radians(self._map_rotation_deg())
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        # Rotate into image-local coordinates and check against unrotated bounds.
        lx = dx * cos_a + dy * sin_a
        ly = -dx * sin_a + dy * cos_a
        return abs(lx) <= w * 0.5 and abs(ly) <= h * 0.5

    def _world_to_model(self, pt: tuple[float, float]) -> tuple[float, float]:
        cx, cy = self._center()
        px_per_m = 1.0 / self._meters_per_px()
        return (cx + pt[0] * px_per_m, cy - pt[1] * px_per_m)

    def _desired_magnifier_size(self, width: float, height: float) -> tuple[int, int]:
        return (
            max(self._canvas_size // 2, int(round(width * 0.5))),
            max(self._canvas_size // 2, int(round(height * 0.5))),
        )

    def _sync_magnifier_size(self, width: float, height: float) -> None:
        if self.magnifier_canvas is None:
            return
        desired_w, desired_h = self._desired_magnifier_size(width, height)
        current_w = int(float(self.magnifier_canvas.cget("width")))
        current_h = int(float(self.magnifier_canvas.cget("height")))
        if current_w == desired_w and current_h == desired_h:
            return
        self.magnifier_canvas.configure(width=desired_w, height=desired_h)

    def _on_main_canvas_configure(self, event: tk.Event) -> None:
        if self.magnifier_canvas is None or not hasattr(self, "rpm_canvas") or not hasattr(self, "nodes"):
            return
        self._sync_magnifier_size(float(event.width), float(event.height))
        self._redraw(recompute_profile=False, notify_state=False)
        if self._map_image is not None:
            self._schedule_map_render(preview=True, delay_ms=self._map_render_idle_ms, redraw=False, notify_state=False)

    def _on_magnifier_canvas_configure(self, _event: tk.Event) -> None:
        if self.magnifier_canvas is None or not hasattr(self, "rpm_canvas") or not hasattr(self, "nodes"):
            return
        self._redraw(recompute_profile=False, notify_state=False)
        if self._map_image is not None:
            self._schedule_map_render(preview=True, delay_ms=self._map_render_idle_ms, redraw=False, notify_state=False)

    def set_geometry(self, a1_cm: float, a2_cm: float, drive_wheel_diameter_cm: float) -> None:
        self._a1_cm = max(a1_cm, 0.0)
        self._a2_cm = max(a2_cm, 0.0)
        self._drive_wheel_diameter_cm = max(drive_wheel_diameter_cm, 0.0)
        self._track_circumference_m = math.pi * max(self._drive_wheel_diameter_cm, 0.0) / 100.0
        self._redraw()

    def set_speed_map(self, cfg: SpeedMapConfig) -> None:
        self._speed_map = cfg
        self._update_pwm_display()
        self._refresh_expected_from_commands()

    def get_pose_source_mode(self) -> PoseSourceMode:
        return POSE_SOURCE_ALIASES.get(
            self._pose_source_var.get().strip(),
            PoseSourceMode.INTERNAL,
        )

    def get_autopilot_mode(self) -> AutopilotMode:
        return AUTOPILOT_ALIASES.get(
            self._autopilot_var.get().strip(),
            AutopilotMode.BUILTIN_PURE_PURSUIT,
        )

    def get_state(self) -> dict:
        nodes = []
        for node in self.nodes:
            item = {
                "anchor": [node.anchor[0], node.anchor[1]],
                "handle_in": None,
                "handle_out": None,
            }
            if node.handle_in is not None:
                item["handle_in"] = [node.handle_in[0], node.handle_in[1]]
            if node.handle_out is not None:
                item["handle_out"] = [node.handle_out[0], node.handle_out[1]]
            nodes.append(item)
        return {
            "nodes": nodes,
            "scale": self._scale_var.get(),
            "map_path": self._map_path,
            "map_scale_factor": self._map_scale_factor(),
            "map_rotation_deg": self._map_rotation_deg(),
            "map_center": [self._map_center[0], self._map_center[1]],
            "min_radius_cm": self._min_radius_var.get(),
            "v_cm": self._v_cm_var.get(),
            "accel": self._accel_var.get(),
            "decel": self._decel_var.get(),
            "dt_ms": self._dt_var.get(),
            "left_shift": self._left_shift_var.get(),
            "right_shift": self._right_shift_var.get(),
            "left_linear": self._left_linear_var.get(),
            "right_linear": self._right_linear_var.get(),
            "pose_source": self.get_pose_source_mode().value,
            "autopilot_mode": self.get_autopilot_mode().value,
            "view_scale": self._view_scale,
            "view_pan": [self._view_pan[0], self._view_pan[1]],
        }

    def set_state(self, data: dict) -> None:
        self._suspend_state_notify = True
        nodes = data.get("nodes", [])
        if isinstance(nodes, list) and nodes:
            parsed = []
            for item in nodes:
                try:
                    ax, ay = item.get("anchor", [0.0, 0.0])
                    hin = item.get("handle_in")
                    hout = item.get("handle_out")
                    parsed.append(
                        BezierNode(
                            anchor=(float(ax), float(ay)),
                            handle_in=(float(hin[0]), float(hin[1])) if hin else None,
                            handle_out=(float(hout[0]), float(hout[1])) if hout else None,
                        )
                    )
                except Exception:
                    continue
            if parsed:
                cx, cy = self._center()
                parsed[0].anchor = (cx, cy)
                if parsed[0].handle_out:
                    parsed[0].handle_out = (0.0, min(parsed[0].handle_out[1], -5.0))
                self.nodes = parsed
        self._scale_var.set(data.get("scale", self._scale_var.get()))
        self._map_image_scale_var.set(data.get("map_scale_factor", self._map_scale_factor()))
        self._map_rotation_var.set(data.get("map_rotation_deg", self._map_rotation_deg()))
        self._min_radius_var.set(str(data.get("min_radius_cm", self._min_radius_var.get())))
        self._v_cm_var.set(str(data.get("v_cm", self._v_cm_var.get())))
        self._accel_var.set(str(data.get("accel", self._accel_var.get())))
        self._decel_var.set(str(data.get("decel", self._decel_var.get())))
        self._dt_var.set(str(data.get("dt_ms", self._dt_var.get())))
        self._left_shift_var.set(str(data.get("left_shift", self._left_shift_var.get())))
        self._right_shift_var.set(str(data.get("right_shift", self._right_shift_var.get())))
        self._left_linear_var.set(str(data.get("left_linear", self._left_linear_var.get())))
        self._right_linear_var.set(str(data.get("right_linear", self._right_linear_var.get())))
        pose_source = POSE_SOURCE_ALIASES.get(
            str(data.get("pose_source", self.get_pose_source_mode().value)),
            PoseSourceMode.INTERNAL,
        )
        self._pose_source_var.set(POSE_SOURCE_LABELS[pose_source])
        autopilot_mode = AUTOPILOT_ALIASES.get(
            str(data.get("autopilot_mode", self.get_autopilot_mode().value)),
            AutopilotMode.BUILTIN_PURE_PURSUIT,
        )
        self._autopilot_var.set(AUTOPILOT_LABELS[autopilot_mode])
        try:
            self._view_scale = float(data.get("view_scale", self._view_scale))
        except Exception:
            pass
        try:
            pan = data.get("view_pan", [self._view_pan[0], self._view_pan[1]])
            self._view_pan = (float(pan[0]), float(pan[1]))
        except Exception:
            pass
        map_path = data.get("map_path")
        map_loaded = False
        if isinstance(map_path, str) and map_path.strip():
            map_loaded = self._load_map_image(map_path.strip(), recenter=False, show_errors=False)
            if map_loaded:
                try:
                    center = data.get("map_center", [self._map_center[0], self._map_center[1]])
                    self._map_center = (float(center[0]), float(center[1]))
                except Exception:
                    self._map_center = self._center()
        if not map_loaded:
            self._map_path = ""
            self._map_image = None
            self._invalidate_map_render(preview=False)
            self._map_image_cache.clear()
            self._map_file_var.set("")
            self.map_scale_slider.state(["disabled"])
            self.map_scale_entry.state(["disabled"])
            self.map_rot_slider.state(["disabled"])
            self.map_rot_entry.state(["disabled"])
        else:
            self.map_scale_entry.state(["!disabled"])
            self.map_rot_entry.state(["!disabled"])
        self._sync_map_scale_text()
        self._sync_map_rotation_text()
        self._suspend_state_notify = False
        self._redraw()
        if map_loaded:
            self._schedule_map_render(preview=False, delay_ms=0, redraw=False, notify_state=False)

    def set_running(self, running: bool) -> None:
        self._editable = not running
        if running:
            self.start_btn.configure(state="disabled")
            self.stop_btn.configure(state="normal")
            self.pose_source_box.configure(state="disabled")
            self.autopilot_box.configure(state="disabled")
        else:
            self.start_btn.configure(state="normal")
            self.stop_btn.configure(state="disabled")
            self.pose_source_box.configure(state="readonly")
            self.autopilot_box.configure(state="readonly")

    def get_command_sequence(self) -> list[tuple[int, int]]:
        if not self._profile_s:
            return []
        points = [
            SpeedMapPoint(self._speed_map.pwm_1, self._speed_map.speed_1),
            SpeedMapPoint(self._speed_map.pwm_2, self._speed_map.speed_2),
            SpeedMapPoint(self._speed_map.pwm_3, self._speed_map.speed_3),
        ]
        cmds = []
        for v_left, v_right in zip(self._profile_v_left, self._profile_v_right):
            pwm_left = speed_to_pwm(v_left, points)
            pwm_right = speed_to_pwm(v_right, points)
            pwm_left = self._apply_pwm_correction(pwm_left, self._left_shift_var, self._left_linear_var)
            pwm_right = self._apply_pwm_correction(pwm_right, self._right_shift_var, self._right_linear_var)
            cmds.append((int(round(pwm_left)), int(round(pwm_right))))
        return cmds

    def get_time_quantum_ms(self) -> int:
        return int(round(self._read_float(self._dt_var, self._dt_ms)))

    def _controller_cfg(self) -> ControllerConfig:
        track_width = max((self._a1_cm + self._a2_cm) / 100.0, 1e-6)
        v_max = max(abs(self._read_float(self._v_cm_var, self._v_cm)), 1e-3)
        accel = max(self._read_float(self._accel_var, self._accel), 1e-3)
        decel = max(self._read_float(self._decel_var, self._decel), 1e-3)
        return ControllerConfig(
            track_width=track_width,
            v_cm_max=v_max,
            a_cm_max=accel,
            a_track_up=accel,
            a_track_down=decel,
            L_min=self._lookahead_min_m,
            L_max=self._lookahead_max_m,
            K_L=self._lookahead_k,
            K_kappa=self._kappa_slowdown,
            enable_stop_profile=True,
            a_stop=decel,
        )

    def _track_world_polyline(self) -> list[tuple[float, float]]:
        pts: list[tuple[float, float]] = []
        last: Optional[tuple[float, float]] = None
        for sample in self._samples:
            world = self._canvas_to_world((sample["x"], sample["y"]))
            if last is None or math.hypot(world[0] - last[0], world[1] - last[1]) >= 1e-4:
                pts.append(world)
                last = world
        return pts

    def build_mission_config(self) -> MissionConfig:
        points = tuple(
            TrackPoint(x_m=world[0], y_m=world[1])
            for world in self._track_world_polyline()
        )
        geometry = GeometrySettings(
            a1_m=self._a1_cm / 100.0,
            a2_m=self._a2_cm / 100.0,
            track_width_m=max((self._a1_cm + self._a2_cm) / 100.0, 1e-6),
            track_circumference_m=self._track_circumference_m,
        )
        motion = MotionSettings(
            target_center_speed_m_s=self._read_float(self._v_cm_var, self._v_cm),
            accel_m_s2=self._read_float(self._accel_var, self._accel),
            decel_m_s2=self._read_float(self._decel_var, self._decel),
            controller_dt_s=self._read_float(self._dt_var, self._dt_ms) / 1000.0,
            minimal_turn_radius_m=self._min_radius_m(),
        )
        builtin_tuning = BuiltinAutopilotTuning(
            lookahead_min_m=self._lookahead_min_m,
            lookahead_max_m=self._lookahead_max_m,
            lookahead_gain=self._lookahead_k,
            curvature_slowdown=self._kappa_slowdown,
        )
        speed_map = (
            SpeedMapEntry(pwm=self._speed_map.pwm_1, speed_m_s=self._speed_map.speed_1),
            SpeedMapEntry(pwm=self._speed_map.pwm_2, speed_m_s=self._speed_map.speed_2),
            SpeedMapEntry(pwm=self._speed_map.pwm_3, speed_m_s=self._speed_map.speed_3),
        )
        pwm_correction = PwmCorrection(
            left_shift=self._read_float(self._left_shift_var, 0.0),
            right_shift=self._read_float(self._right_shift_var, 0.0),
            left_linear=self._read_float(self._left_linear_var, 1.0),
            right_linear=self._read_float(self._right_linear_var, 1.0),
        )
        return MissionConfig(
            track_points=points,
            geometry=geometry,
            motion=motion,
            builtin_tuning=builtin_tuning,
            speed_map=speed_map,
            pwm_correction=pwm_correction,
            pose_source=self.get_pose_source_mode(),
            autopilot=self.get_autopilot_mode(),
            telemetry_subscription=TelemetrySubscription(),
        )

    def get_pose_source(self) -> str:
        return self.get_pose_source_mode().value

    def start_controller(self) -> bool:
        pts = self._track_world_polyline()
        if len(pts) < 2:
            return False
        try:
            self._controller_track = SimplePolylineTrack(pts)
        except ValueError:
            return False
        self._controller = PurePursuitController(self._controller_cfg(), self._controller_track)
        self._controller.reset()
        self._expected_world = (0.0, 0.0, 0.0)
        self._expected_running = True
        self._draw_markers()
        return True

    def controller_finished(self) -> bool:
        if self._controller is None:
            return True
        return self._controller.finished

    def _controller_input_pose_world(self) -> Optional[tuple[float, float, float]]:
        source = self.get_pose_source_mode()
        if source == PoseSourceMode.TROLLEY:
            return self._actual_world
        if source == PoseSourceMode.EXTERNAL:
            return self._external_world
        return self._expected_world

    def step_controller(self, dt_s: float, force_internal_pose: bool = False) -> Optional[tuple[int, int]]:
        if self._controller is None or dt_s <= 0.0:
            return None
        if force_internal_pose:
            src_pose = self._expected_world
        else:
            src_pose = self._controller_input_pose_world()
        if src_pose is None:
            return None
        # Internal kinematics uses theta=0 along +Y. Controller uses theta=0 along +X.
        pose = ControllerPose2D(x=src_pose[0], y=src_pose[1], theta=(math.pi * 0.5 - src_pose[2]))
        speeds, _dbg = self._controller.update(pose, dt_s, debug=False)
        if self._expected_world is None:
            self._expected_world = (0.0, 0.0, 0.0)
        self._expected_world = self._integrate_pose(self._expected_world, speeds.v_left, speeds.v_right, dt_s)
        self._set_pwm_from_speeds(speeds.v_left, speeds.v_right)
        self._draw_markers()
        return (int(round(self._pwm_left)), int(round(self._pwm_right)))

    def reset_actual_trace(self) -> None:
        self._actual_s = []
        self._actual_v_left = []
        self._actual_v_right = []
        self._last_actual_ts_ms = None
        self._actual_pose = None
        self._actual_world = None
        self._external_pose = None
        self._external_world = None
        self._draw_rpm()
        self._draw_markers()

    def add_actual_tacho(self, left_rpm: int, right_rpm: int, ts_ms: int) -> None:
        if self._actual_pose is None:
            self._actual_pose = (0.0, 0.0, 0.0)
        if self._last_actual_ts_ms is None:
            dt = self._expected_dt_s if self._expected_dt_s > 0.0 else 0.1
        else:
            dt_ms = u32_diff_mod(int(ts_ms), int(self._last_actual_ts_ms))
            if dt_ms <= 0:
                dt = self._expected_dt_s if self._expected_dt_s > 0.0 else 0.1
            else:
                dt = dt_ms / 1000.0
        # Clamp dt to reject long UI stalls / outlier packets in odometry integration.
        dt = max(0.005, min(dt, 0.5))
        self._last_actual_ts_ms = ts_ms
        v_left = self._track_speed_from_rpm(left_rpm)
        v_right = self._track_speed_from_rpm(right_rpm)
        last_s = self._actual_s[-1] if self._actual_s else 0.0
        s_next = last_s + (abs(v_left) + abs(v_right)) * 0.5 * dt
        self._actual_s.append(s_next)
        self._actual_v_left.append(v_left)
        self._actual_v_right.append(v_right)
        self._actual_pose = self._integrate_pose(self._actual_pose, v_left, v_right, dt)
        self._actual_world = self._actual_pose
        self._draw_rpm()
        self._draw_markers()

    def set_external_pose(self, pose: Optional[PoseEstimate]) -> None:
        self._external_pose = pose
        if pose is None:
            self._external_world = None
        else:
            self._external_world = (pose.x_m, pose.y_m, pose.theta_rad)
        self._draw_markers()

    def get_selected_pose_estimate(self) -> Optional[PoseEstimate]:
        source = self.get_pose_source_mode()
        if source == PoseSourceMode.TROLLEY:
            return self._pose_estimate_from_world(self._actual_world, source, self._last_actual_ts_ms)
        if source == PoseSourceMode.EXTERNAL:
            return self._external_pose
        return self._pose_estimate_from_world(self._expected_world, source, None)

    def apply_drive_command(self, command: DriveCommand, dt_s: float) -> None:
        self._expected_running = True
        self._pwm_left = float(command.left_pwm)
        self._pwm_right = float(command.right_pwm)
        self._pwm_left_var.set(str(int(round(self._pwm_left))))
        self._pwm_right_var.set(str(int(round(self._pwm_right))))
        if self._expected_world is None:
            self._expected_world = (0.0, 0.0, 0.0)
        if dt_s > 0.0:
            v_left, v_right = self._speeds_from_pwm(command.left_pwm, command.right_pwm)
            self._expected_world = self._integrate_pose(self._expected_world, v_left, v_right, dt_s)
        self._draw_markers()

    def _pose_estimate_from_world(
        self,
        pose: Optional[tuple[float, float, float]],
        source: PoseSourceMode,
        mcu_time_ms: Optional[int],
    ) -> Optional[PoseEstimate]:
        if pose is None:
            return None
        return PoseEstimate(
            x_m=pose[0],
            y_m=pose[1],
            theta_rad=pose[2],
            source=source,
            pc_time_ms=now_ms_monotonic(),
            mcu_time_ms=mcu_time_ms,
        )

    def _speeds_from_pwm(self, left_pwm: float, right_pwm: float) -> tuple[float, float]:
        points = [
            SpeedMapPoint(self._speed_map.pwm_1, self._speed_map.speed_1),
            SpeedMapPoint(self._speed_map.pwm_2, self._speed_map.speed_2),
            SpeedMapPoint(self._speed_map.pwm_3, self._speed_map.speed_3),
        ]
        return (
            pwm_to_speed(left_pwm, points),
            pwm_to_speed(right_pwm, points),
        )

    def _track_speed_from_rpm(self, rpm: float) -> float:
        return (float(rpm) / 60.0) * self._track_circumference_m

    # ---------------- Drawing ----------------

    def _redraw(self, *, recompute_profile: bool = True, notify_state: bool = True) -> None:
        for canvas in self._route_canvases():
            canvas.delete("map")
            canvas.delete("grid")
            canvas.delete("curve")
            canvas.delete("nodes")
            canvas.delete("hover")
            canvas.delete("marker")
        self.rpm_canvas.delete("hover")
        for canvas in self._route_canvases():
            self._draw_map(canvas)
            self._draw_grid(canvas)
            self._draw_curve(canvas)
            self._draw_nodes(canvas)
        if recompute_profile:
            self._recompute_profile()
        if notify_state:
            self._notify_state_change()
        self._draw_markers()

    def _schedule_state_notify(self, delay_ms: int = 120) -> None:
        if self._suspend_state_notify:
            return
        if self._state_notify_after_id is not None:
            try:
                self.after_cancel(self._state_notify_after_id)
            except Exception:
                pass
            self._state_notify_after_id = None
        self._state_notify_after_id = self.after(delay_ms, self._flush_scheduled_state_notify)

    def _flush_scheduled_state_notify(self) -> None:
        self._state_notify_after_id = None
        self._notify_state_change()

    def _draw_map(self, canvas: tk.Canvas) -> None:
        if self._map_image is None:
            return
        spec = self._map_render_spec(canvas)
        if spec is None:
            return
        image = None if self._map_render_preview else self._map_image_cache.get(spec.cache_key)
        if image is None:
            self._draw_map_placeholder(canvas, spec)
            return
        vx, vy = self._to_view(self._map_center, canvas)
        canvas.create_image(vx, vy, image=image, anchor="center", tags="map")

    def _draw_grid(self, canvas: tk.Canvas) -> None:
        step = self._cell_px()
        # Compute visible model bounds to draw an infinite grid.
        canvas_w, canvas_h = self._canvas_dimensions(canvas)
        min_model = self._from_view((0.0, 0.0), canvas)
        max_model = self._from_view((canvas_w, canvas_h), canvas)
        min_x = min(min_model[0], max_model[0])
        max_x = max(min_model[0], max_model[0])
        min_y = min(min_model[1], max_model[1])
        max_y = max(min_model[1], max_model[1])

        start_x = int(math.floor(min_x / step)) - 1
        end_x = int(math.ceil(max_x / step)) + 1
        start_y = int(math.floor(min_y / step)) - 1
        end_y = int(math.ceil(max_y / step)) + 1

        for i in range(start_y, end_y + 1):
            pos = i * step
            y = self._to_view((0.0, pos), canvas)[1]
            canvas.create_line(0, y, canvas_w, y, fill="#8a8a8a", tags="grid")
        for i in range(start_x, end_x + 1):
            pos = i * step
            x = self._to_view((pos, 0.0), canvas)[0]
            canvas.create_line(x, 0, x, canvas_h, fill="#8a8a8a", tags="grid")

    def _draw_curve(self, canvas: tk.Canvas) -> None:
        segments = self._segments()
        if not segments:
            return
        points: list[float] = []
        left_pts: list[float] = []
        right_pts: list[float] = []
        self._samples = []
        prev_world: Optional[tuple[float, float]] = None
        s_world = 0.0
        for seg_idx, (p0, p1, p2, p3) in enumerate(segments):
            for i in range(self._samples_per_seg + 1):
                t = i / self._samples_per_seg
                x, y = cubic_point(p0, p1, p2, p3, t)
                vx, vy = self._to_view((x, y), canvas)
                points.extend([vx, vy])
                w_pt = self._canvas_to_world((x, y))
                if prev_world is not None:
                    s_world += math.hypot(w_pt[0] - prev_world[0], w_pt[1] - prev_world[1])
                prev_world = w_pt
                dx, dy = cubic_derivative(p0, p1, p2, p3, t)
                speed = math.hypot(dx, dy)
                if speed > 1e-8:
                    tx, ty = dx / speed, dy / speed
                    nx, ny = -ty, tx
                else:
                    nx, ny = 0.0, 0.0
                self._samples.append({"x": x, "y": y, "nx": nx, "ny": ny, "t": t, "seg": seg_idx, "s": s_world})
                a1_px = self._meters_to_px(self._a1_cm / 100.0)
                a2_px = self._meters_to_px(self._a2_cm / 100.0)
                left_v = self._to_view((x + nx * a1_px, y + ny * a1_px), canvas)
                right_v = self._to_view((x - nx * a2_px, y - ny * a2_px), canvas)
                left_pts.extend([left_v[0], left_v[1]])
                right_pts.extend([right_v[0], right_v[1]])
        color = self._curve_color(segments)
        if left_pts:
            canvas.create_line(left_pts, fill="#9aa6e8", width=1, smooth=True, tags="curve")
        if right_pts:
            canvas.create_line(right_pts, fill="#9aa6e8", width=1, smooth=True, tags="curve")
        canvas.create_line(points, fill=color, width=2, smooth=True, tags="curve")

    def _recompute_profile(self) -> None:
        segments = self._segments()
        world_segments = self._world_segments(segments)
        # Build curvature-vs-distance samples and plan track speeds over time quanta.
        samples = build_path_samples(world_segments, self._samples_per_seg)
        params = MotionParams(
            v_cm=self._read_float(self._v_cm_var, self._v_cm),
            accel=self._read_float(self._accel_var, self._accel),
            decel=self._read_float(self._decel_var, self._decel),
            dt=self._read_float(self._dt_var, self._dt_ms) / 1000.0,
            a1=self._a1_cm / 100.0,
            a2=self._a2_cm / 100.0,
        )
        profile = plan_profile(samples, params)
        self._profile_s = profile.s
        self._profile_v_left = profile.v_left
        self._profile_v_right = profile.v_right
        self._expected_dt_s = params.dt
        if not self._expected_running:
            self.prepare_expected()
        self._draw_rpm()

    def _read_float(self, var: tk.StringVar, default: float) -> float:
        try:
            return float(var.get().strip())
        except ValueError:
            return default

    def _draw_rpm(self) -> None:
        c = self.rpm_canvas
        w = max(1, int(c.winfo_width()))
        h = max(1, int(c.winfo_height()))
        c.delete("all")

        pad = 20
        left = pad
        right = w - pad
        top = pad
        bottom = h - pad

        c.create_line(left, bottom, right, bottom, fill="#000000")
        c.create_line(left, top, left, bottom, fill="#000000")
        c.create_text(left + 4, top, text="Speed (m/s)", anchor="nw", fill="#000000")

        if not self._profile_s:
            self._rpm_view = {}
            return

        max_abs = 0.1
        for v in self._profile_v_left + self._profile_v_right + self._actual_v_left + self._actual_v_right:
            max_abs = max(max_abs, abs(v))

        def y_of(v: float) -> float:
            return top + (max_abs - v) * (bottom - top) / (2 * max_abs)

        total = max(self._profile_s[-1], self._actual_s[-1] if self._actual_s else 0.0, 1e-6)
        def x_of(s_val: float) -> float:
            return left + s_val * (right - left) / total

        # Grid + labels
        grid_color = "#cccccc"
        grid_n = 5
        for i in range(grid_n + 1):
            gx = left + i * (right - left) / grid_n
            c.create_line(gx, top, gx, bottom, fill=grid_color)
            s_val = total * i / grid_n
            c.create_text(gx, bottom + 2, text=f"{s_val:.1f}", anchor="n", fill="#000000")
        for i in range(grid_n + 1):
            gy = top + i * (bottom - top) / grid_n
            c.create_line(left, gy, right, gy, fill=grid_color)
            v_val = max_abs - (2 * max_abs) * i / grid_n
            c.create_text(left - 2, gy, text=f"{v_val:.2f}", anchor="e", fill="#000000")
        c.create_text(right, bottom + 2, text="Distance (m)", anchor="ne", fill="#000000")

        def draw_series(s_axis: list[float], values: list[float], color: str) -> None:
            pts = []
            for s_val, v in zip(s_axis, values):
                pts.extend([x_of(s_val), y_of(v)])
            if len(pts) >= 4:
                c.create_line(*pts, fill=color, width=2)

        draw_series(self._profile_s, self._profile_v_left, "#cc0000")
        draw_series(self._profile_s, self._profile_v_right, "#0044cc")
        if self._actual_s:
            draw_series(self._actual_s, self._actual_v_left, "#cc6666")
            draw_series(self._actual_s, self._actual_v_right, "#6688cc")
        c.create_text(right - 5, top, text="L speed  R speed", anchor="ne", fill="#000000")

        self._rpm_view = {
            "left": float(left),
            "right": float(right),
            "top": float(top),
            "bottom": float(bottom),
            "max_abs": float(max_abs),
            "total": float(total),
        }
        if self._hover_s is not None:
            self._draw_rpm_hover(self._hover_s)

    def refresh_profile(self) -> None:
        self._recompute_profile()

    def prepare_expected(self) -> list[tuple[int, int]]:
        self._expected_cmds = self.get_command_sequence()
        self._expected_dt_s = self._read_float(self._dt_var, self._dt_ms) / 1000.0
        self._build_expected_poses()
        return self._expected_cmds

    def reset_expected_pose(self) -> None:
        self._expected_start_ms = None
        self._expected_running = True
        self._expected_world = (0.0, 0.0, 0.0)
        self._draw_markers()

    def start_expected(self, start_ms: int) -> None:
        self._expected_start_ms = start_ms
        self._expected_running = True
        if self._expected_world is None:
            self._expected_world = (0.0, 0.0, 0.0)
        self._draw_markers()

    def stop_expected(self) -> None:
        self._expected_running = False
        self._expected_start_ms = None
        self._expected_world = None
        self._controller = None
        self._controller_track = None
        self._draw_markers()

    def _expected_tick(self) -> None:
        if not self._expected_running:
            return
        if not self._expected_poses or self._expected_start_ms is None:
            return
        if self._expected_dt_s <= 0.0:
            return
        elapsed = max(0.0, (now_ms_monotonic() - self._expected_start_ms) / 1000.0)
        idx = min(int(elapsed / self._expected_dt_s), len(self._expected_poses) - 1)
        self._expected_world = self._expected_poses[idx]
        self._draw_markers()
        self.after(50, self._expected_tick)

    def _build_expected_poses(self) -> None:
        self._expected_poses = []
        if not self._expected_cmds or self._expected_dt_s <= 0.0:
            return
        pose = (0.0, 0.0, 0.0)
        self._expected_poses.append(pose)
        points = [
            SpeedMapPoint(self._speed_map.pwm_1, self._speed_map.speed_1),
            SpeedMapPoint(self._speed_map.pwm_2, self._speed_map.speed_2),
            SpeedMapPoint(self._speed_map.pwm_3, self._speed_map.speed_3),
        ]
        for left_pwm, right_pwm in self._expected_cmds:
            v_left = pwm_to_speed(left_pwm, points)
            v_right = pwm_to_speed(right_pwm, points)
            pose = self._integrate_pose(pose, v_left, v_right, self._expected_dt_s)
            self._expected_poses.append(pose)

    def _integrate_pose(
        self,
        pose: tuple[float, float, float],
        v_left: float,
        v_right: float,
        dt: float,
    ) -> tuple[float, float, float]:
        x, y, theta = pose
        L = (self._a1_cm + self._a2_cm) / 100.0
        if L <= 1e-9:
            return (x, y, theta)
        v = 0.5 * (v_right + v_left)
        # Internal heading uses theta=0 along +Y and grows clockwise.
        # For this convention, angular rate sign is opposite to the standard diff-drive equation.
        w = (v_left - v_right) / L
        if abs(w) <= 1e-6:
            x += v * dt * math.sin(theta)
            y += v * dt * math.cos(theta)
        else:
            dtheta = w * dt
            R = v / w
            x += R * (math.cos(theta) - math.cos(theta + dtheta))
            y += R * (math.sin(theta + dtheta) - math.sin(theta))
            theta += dtheta
        return (x, y, theta)

    def _track_centers(self, pose: tuple[float, float, float]) -> tuple[tuple[float, float], tuple[float, float], tuple[float, float]]:
        x, y, theta = pose
        a1 = self._a1_cm / 100.0
        a2 = self._a2_cm / 100.0
        left = (x - math.cos(theta) * a1, y + math.sin(theta) * a1)
        right = (x + math.cos(theta) * a2, y - math.sin(theta) * a2)
        return (x, y), left, right

    def _draw_markers(self) -> None:
        for canvas in self._route_canvases():
            canvas.delete("marker")
            if self._expected_world is not None:
                self._draw_pose_marker(canvas, self._expected_world, "#00aa00")
            if self._actual_world is not None:
                self._draw_pose_marker(canvas, self._actual_world, "#ff8800")
            if self._external_world is not None and self.get_pose_source_mode() == PoseSourceMode.EXTERNAL:
                self._draw_pose_marker(canvas, self._external_world, "#0088aa")

    def _draw_pose_marker(self, canvas: tk.Canvas, pose: tuple[float, float, float], color: str) -> None:
        center, left, right = self._track_centers(pose)
        for pt in (center, left, right):
            mx, my = self._world_to_model(pt)
            vx, vy = self._to_view((mx, my), canvas)
            canvas.create_oval(vx - 4, vy - 4, vx + 4, vy + 4, fill=color, outline="#000000", tags="marker")

    def _draw_nodes(self, canvas: tk.Canvas) -> None:
        for idx, node in enumerate(self.nodes):
            ax, ay = node.anchor
            vx, vy = self._to_view((ax, ay), canvas)
            self._draw_point(canvas, vx, vy, 5, "#000000", "nodes")
            if node.handle_in:
                hx, hy = self._handle_pos(node.anchor, node.handle_in)
                hvx, hvy = self._to_view((hx, hy), canvas)
                canvas.create_line(vx, vy, hvx, hvy, fill="#5a5a5a", tags="nodes")
                self._draw_point(canvas, hvx, hvy, 4, "#3a6bbf", "nodes")
            if node.handle_out:
                hx, hy = self._handle_pos(node.anchor, node.handle_out)
                hvx, hvy = self._to_view((hx, hy), canvas)
                canvas.create_line(vx, vy, hvx, hvy, fill="#5a5a5a", tags="nodes")
                self._draw_point(canvas, hvx, hvy, 4, "#3a6bbf", "nodes")

    def _draw_point(self, canvas: tk.Canvas, x: float, y: float, r: int, fill: str, tag: str) -> None:
        canvas.create_oval(x - r, y - r, x + r, y + r, fill=fill, outline="#000000", tags=tag)

    def _curve_color(self, segments: list[tuple[tuple[float, float], ...]]) -> str:
        min_allowed = self._min_radius_m()
        if min_allowed is None:
            return "#3b6eff"
        world_segments = self._world_segments(segments)
        min_radius = min_curve_radius(world_segments, self._samples_per_seg)
        if min_radius is None:
            return "#3b6eff"
        return COLOR_GREEN if min_radius >= min_allowed else COLOR_RED

    # ---------------- Bezier math ----------------

    def _segments(self) -> list[tuple[tuple[float, float], ...]]:
        if len(self.nodes) < 2:
            return []
        segs = []
        for a, b in zip(self.nodes[:-1], self.nodes[1:]):
            p0 = a.anchor
            p1 = self._handle_pos(a.anchor, a.handle_out) if a.handle_out else a.anchor
            p2 = self._handle_pos(b.anchor, b.handle_in) if b.handle_in else b.anchor
            p3 = b.anchor
            segs.append((p0, p1, p2, p3))
        return segs

    def _world_segments(self, segments: list[tuple[tuple[float, float], ...]]) -> list[tuple[tuple[float, float], ...]]:
        world_segments = []
        for p0, p1, p2, p3 in segments:
            world_segments.append(
                (
                    self._canvas_to_world(p0),
                    self._canvas_to_world(p1),
                    self._canvas_to_world(p2),
                    self._canvas_to_world(p3),
                )
            )
        return world_segments

    def _handle_pos(self, anchor: tuple[float, float], offset: tuple[float, float]) -> tuple[float, float]:
        return (anchor[0] + offset[0], anchor[1] + offset[1])

    # ---------------- Interaction ----------------

    def _choose_map_file(self) -> None:
        if Image is None or ImageTk is None:
            messagebox.showerror("Coordinate", "Map image support requires Pillow (PIL).")
            return
        path = filedialog.askopenfilename(
            title="Select map image",
            filetypes=[
                ("Map images", "*.png *.jpg *.jpeg"),
                ("PNG", "*.png"),
                ("JPEG", "*.jpg *.jpeg"),
                ("All files", "*.*"),
            ],
        )
        if not path:
            return
        if not self._load_map_image(path, recenter=True, show_errors=True, redraw=False):
            return
        self._map_image_scale_var.set(1.0)
        self._map_rotation_var.set(0.0)
        self._sync_map_scale_text()
        self._sync_map_rotation_text()
        self._redraw()
        self._schedule_map_render(preview=False, delay_ms=0, redraw=False, notify_state=False)

    def _on_map_scale_change(self, _value: str | None = None) -> None:
        self._sync_map_scale_text()
        if self._map_image is not None:
            self._schedule_map_render(
                preview=True,
                delay_ms=self._map_render_idle_ms,
                redraw=True,
                notify_state=True,
            )

    def _on_map_rotation_change(self, _value: str | None = None) -> None:
        self._sync_map_rotation_text()
        if self._map_image is not None:
            self._schedule_map_render(
                preview=True,
                delay_ms=self._map_render_idle_ms,
                redraw=True,
                notify_state=True,
            )

    def _commit_map_scale_text(self, _event: tk.Event | None = None) -> str | None:
        try:
            value = float(self._map_image_scale_text_var.get().strip())
        except ValueError:
            self._sync_map_scale_text()
            return "break"
        self._map_image_scale_var.set(max(0.1, min(6.0, value)))
        self._sync_map_scale_text()
        if self._map_image is not None:
            self._schedule_map_render(
                preview=True,
                delay_ms=self._map_render_idle_ms,
                redraw=True,
                notify_state=True,
            )
        return "break"

    def _commit_map_rotation_text(self, _event: tk.Event | None = None) -> str | None:
        try:
            value = float(self._map_rotation_text_var.get().strip())
        except ValueError:
            self._sync_map_rotation_text()
            return "break"
        self._map_rotation_var.set(max(0.0, min(360.0, value)))
        self._sync_map_rotation_text()
        if self._map_image is not None:
            self._schedule_map_render(
                preview=True,
                delay_ms=self._map_render_idle_ms,
                redraw=True,
                notify_state=True,
            )
        return "break"

    def _load_map_image(
        self,
        path: str,
        *,
        recenter: bool,
        show_errors: bool,
        redraw: bool = False,
    ) -> bool:
        if Image is None or ImageTk is None:
            if show_errors:
                messagebox.showerror("Coordinate", "Map image support requires Pillow (PIL).")
            return False
        try:
            with Image.open(path) as src:
                loaded = src.convert("RGBA")
        except (FileNotFoundError, PermissionError, OSError, UnidentifiedImageError) as exc:
            if show_errors:
                messagebox.showerror("Coordinate", f"Failed to load map image:\n{exc}")
            return False
        self._map_path = path
        self._map_image = loaded
        self._invalidate_map_render(preview=False)
        self._map_file_var.set(os.path.basename(path))
        if recenter:
            self._map_center = self._center()
        self.map_scale_slider.state(["!disabled"])
        self.map_scale_entry.state(["!disabled"])
        self.map_rot_slider.state(["!disabled"])
        self.map_rot_entry.state(["!disabled"])
        if redraw:
            self._redraw()
        return True

    def _clear_map_drag_state(self) -> None:
        self._map_drag_start = None
        self._map_drag_origin = None
        self._map_drag_canvas = None
        self._map_drag_moved = False
        self._map_click_add_point = None

    def _on_press(self, e: tk.Event) -> None:
        if not self._editable:
            return
        canvas = self._resolve_canvas(e.widget if isinstance(e.widget, tk.Canvas) else None)
        self._clear_map_drag_state()
        hit = self._hit_test(e.x, e.y, canvas)
        if hit:
            self._drag_target = hit
            return
        if self._hit_map(e.x, e.y, canvas):
            self._map_drag_start = (e.x, e.y)
            self._map_drag_origin = self._map_center
            self._map_drag_canvas = canvas
            self._map_drag_moved = False
            self._map_click_add_point = self._from_view((e.x, e.y), canvas)
            return
        mx, my = self._from_view((e.x, e.y), canvas)
        self._add_node(mx, my)
        self._drag_target = ("anchor", len(self.nodes) - 1)

    def _on_drag(self, e: tk.Event) -> None:
        canvas = self._resolve_canvas(e.widget if isinstance(e.widget, tk.Canvas) else None)
        if self._drag_target:
            kind, idx = self._drag_target
            node = self.nodes[idx]
            if kind == "anchor":
                if idx == 0:
                    return
                if not self._editable:
                    return
                mx, my = self._from_view((e.x, e.y), canvas)
                node.anchor = (mx, my)
            elif kind == "in":
                if not self._editable:
                    return
                mx, my = self._from_view((e.x, e.y), canvas)
                offset = self._handle_offset(node.anchor, mx, my)
                self._set_symmetric_handle(node, idx, "in", offset)
            elif kind == "out":
                if idx == 0:
                    _, my = self._from_view((e.x, e.y), canvas)
                    dy = my - node.anchor[1]
                    offset = (0.0, min(dy, -5.0))
                    node.handle_out = offset
                else:
                    if not self._editable:
                        return
                    mx, my = self._from_view((e.x, e.y), canvas)
                    offset = self._handle_offset(node.anchor, mx, my)
                    self._set_symmetric_handle(node, idx, "out", offset)
            self._redraw()
            return
        if self._map_drag_start is None or self._map_drag_origin is None or self._map_drag_canvas is None:
            return
        dx = e.x - self._map_drag_start[0]
        dy = e.y - self._map_drag_start[1]
        if not self._map_drag_moved and math.hypot(dx, dy) >= self._map_drag_threshold_px:
            self._map_drag_moved = True
            self._map_click_add_point = None
        if not self._map_drag_moved:
            return
        inv_scale = self._canvas_display_scale(self._map_drag_canvas)
        self._map_center = (
            self._map_drag_origin[0] + dx / inv_scale,
            self._map_drag_origin[1] + dy / inv_scale,
        )
        self._redraw(recompute_profile=False, notify_state=False)
        self._schedule_state_notify()

    def _on_release(self, _e: tk.Event) -> None:
        if self._drag_target is not None:
            self._drag_target = None
            return
        if self._map_drag_start is None:
            return
        add_point = self._map_click_add_point
        moved = self._map_drag_moved
        self._clear_map_drag_state()
        if moved:
            self._schedule_state_notify(0)
            return
        if add_point is None:
            return
        self._add_node(add_point[0], add_point[1])
        self._drag_target = None

    def _handle_offset(self, anchor: tuple[float, float], x: float, y: float) -> tuple[float, float]:
        return (x - anchor[0], y - anchor[1])

    def _set_symmetric_handle(self, node: BezierNode, idx: int, kind: str, offset: tuple[float, float]) -> None:
        is_end = idx == 0 or idx == len(self.nodes) - 1
        if kind == "in":
            node.handle_in = offset
            if not is_end:
                node.handle_out = (-offset[0], -offset[1])
        elif kind == "out":
            node.handle_out = offset
            if not is_end:
                node.handle_in = (-offset[0], -offset[1])

    def _add_node(self, x: float, y: float) -> None:
        if not self.nodes:
            return
        prev = self.nodes[-1]
        dx = x - prev.anchor[0]
        dy = y - prev.anchor[1]
        length = math.hypot(dx, dy) or 1.0
        ux, uy = dx / length, dy / length

        if prev.handle_out is None:
            prev.handle_out = (ux * self._handle_len, uy * self._handle_len)
        if prev != self.nodes[0]:
            prev.handle_in = (-prev.handle_out[0], -prev.handle_out[1])
        handle_in = (-ux * self._handle_len, -uy * self._handle_len)
        handle_out = (ux * self._handle_len, uy * self._handle_len)
        self.nodes.append(BezierNode(anchor=(x, y), handle_in=handle_in, handle_out=handle_out))
        self._redraw()

    def _on_double_click(self, e: tk.Event) -> None:
        if not self._editable:
            return
        canvas = self._resolve_canvas(e.widget if isinstance(e.widget, tk.Canvas) else None)
        hit = self._hit_test(e.x, e.y, canvas)
        if not hit:
            return
        kind, idx = hit
        if kind != "anchor" or idx == 0:
            return
        self.nodes.pop(idx)
        self._redraw()

    def _on_motion(self, e: tk.Event) -> None:
        canvas = self._resolve_canvas(e.widget if isinstance(e.widget, tk.Canvas) else None)
        self._clear_hover()
        if not self._samples:
            return
        nearest = None
        min_dist = None
        for sample in self._samples:
            sx, sy = self._to_view((sample["x"], sample["y"]), canvas)
            dx = sx - e.x
            dy = sy - e.y
            dist = math.hypot(dx, dy)
            if min_dist is None or dist < min_dist:
                min_dist = dist
                nearest = sample
        if nearest is None or min_dist is None or min_dist > self._hover_px:
            self._hover_s = None
            return
        self._hover_s = nearest.get("s", 0.0)
        self._draw_hover(nearest)
        self._draw_rpm_hover(self._hover_s)

    def _on_rpm_motion(self, e: tk.Event) -> None:
        if not self._profile_s:
            return
        view = self._rpm_view
        if not view:
            return
        left = view["left"]
        right = view["right"]
        total = view["total"]
        if right <= left:
            return
        s = (e.x - left) * total / (right - left)
        s = min(max(s, 0.0), total)
        self._hover_s = s
        sample = self._nearest_sample_by_s(s)
        if sample is None:
            return
        self._clear_hover()
        self._draw_hover(sample)
        self._draw_rpm_hover(s)

    def _on_rpm_leave(self, _e: tk.Event) -> None:
        self._clear_hover()
        self._hover_s = None

    def _draw_hover(self, sample: dict[str, float]) -> None:
        x = sample["x"]
        y = sample["y"]
        nx = sample["nx"]
        ny = sample["ny"]
        s = sample.get("s", 0.0)
        a1_px = self._meters_to_px(self._a1_cm / 100.0)
        a2_px = self._meters_to_px(self._a2_cm / 100.0)
        left_m = (x + nx * a1_px, y + ny * a1_px)
        right_m = (x - nx * a2_px, y - ny * a2_px)
        radii = self._curvature_radii(sample)
        speeds = self._speed_at_s(s)
        for canvas in self._route_canvases():
            vx, vy = self._to_view((x, y), canvas)
            left = self._to_view(left_m, canvas)
            right = self._to_view(right_m, canvas)
            self._draw_point(canvas, vx, vy, 4, "#000000", "hover")
            self._draw_point(canvas, left[0], left[1], 4, "#3a6bbf", "hover")
            self._draw_point(canvas, right[0], right[1], 4, "#3a6bbf", "hover")
            self._draw_radius_label(canvas, (vx + 8, vy - 8), radii["center"], speeds[0], "hover")
            self._draw_radius_label(canvas, (left[0] + 8, left[1] - 8), radii["left"], speeds[1], "hover")
            self._draw_radius_label(canvas, (right[0] + 8, right[1] - 8), radii["right"], speeds[2], "hover")
        if speeds[1] is not None and speeds[2] is not None:
            self._set_pwm_from_speeds(speeds[1], speeds[2])

    def _on_start_click(self) -> None:
        if self.on_start:
            self.on_start()

    def _on_stop_click(self) -> None:
        if self.on_stop:
            self.on_stop()

    def _draw_radius_label(
        self,
        canvas: tk.Canvas,
        pos: tuple[float, float],
        radius_m: Optional[float],
        speed_m_s: Optional[float],
        tag: str,
    ) -> None:
        label = "R=inf"
        if radius_m is not None:
            radius_cm = abs(radius_m) * 100.0
            label = f"R={radius_cm:.1f} cm"
        if speed_m_s is not None:
            label = f"{label}\nV={speed_m_s:.2f} m/s"
        canvas.create_text(pos[0], pos[1], text=label, fill="#202020", anchor="w", tags=tag)

    def _speed_at_s(self, s: float) -> tuple[Optional[float], Optional[float], Optional[float]]:
        if not self._profile_s:
            return None, None, None
        if s <= self._profile_s[0]:
            v_left = self._profile_v_left[0]
            v_right = self._profile_v_right[0]
        elif s >= self._profile_s[-1]:
            v_left = self._profile_v_left[-1]
            v_right = self._profile_v_right[-1]
        else:
            lo = 0
            hi = len(self._profile_s) - 1
            while hi - lo > 1:
                mid = (lo + hi) // 2
                if self._profile_s[mid] < s:
                    lo = mid
                else:
                    hi = mid
            s0 = self._profile_s[lo]
            s1 = self._profile_s[hi]
            if s1 <= s0:
                v_left = self._profile_v_left[lo]
                v_right = self._profile_v_right[lo]
            else:
                t = (s - s0) / (s1 - s0)
                v_left = self._profile_v_left[lo] + (self._profile_v_left[hi] - self._profile_v_left[lo]) * t
                v_right = self._profile_v_right[lo] + (self._profile_v_right[hi] - self._profile_v_right[lo]) * t
        return (0.5 * (v_left + v_right), v_left, v_right)

    def _set_pwm_from_speeds(self, v_left: float, v_right: float) -> None:
        points = [
            SpeedMapPoint(self._speed_map.pwm_1, self._speed_map.speed_1),
            SpeedMapPoint(self._speed_map.pwm_2, self._speed_map.speed_2),
            SpeedMapPoint(self._speed_map.pwm_3, self._speed_map.speed_3),
        ]
        pwm_left = speed_to_pwm(v_left, points)
        pwm_right = speed_to_pwm(v_right, points)
        pwm_left = self._apply_pwm_correction(pwm_left, self._left_shift_var, self._left_linear_var)
        pwm_right = self._apply_pwm_correction(pwm_right, self._right_shift_var, self._right_linear_var)
        self._pwm_left = pwm_left
        self._pwm_right = pwm_right
        self._pwm_left_var.set(str(int(round(pwm_left))))
        self._pwm_right_var.set(str(int(round(pwm_right))))

    def _apply_pwm_correction(self, value: float, shift_var: tk.StringVar, linear_var: tk.StringVar) -> float:
        try:
            shift = float(shift_var.get().strip())
        except ValueError:
            shift = 0.0
        try:
            linear = float(linear_var.get().strip())
        except ValueError:
            linear = 1.0
        return (value + shift) * linear

    def _update_pwm_display(self) -> None:
        if not self._profile_s:
            return
        if self._hover_s is None:
            s = self._profile_s[-1]
        else:
            s = self._hover_s
        speeds = self._speed_at_s(s)
        if speeds[1] is None or speeds[2] is None:
            return
        self._set_pwm_from_speeds(speeds[1], speeds[2])
        self._notify_state_change()
        self._refresh_expected_from_commands()

    def _nearest_sample_by_s(self, s: float) -> Optional[dict[str, float]]:
        if not self._samples:
            return None
        nearest = None
        min_dist = None
        for sample in self._samples:
            ds = abs(sample.get("s", 0.0) - s)
            if min_dist is None or ds < min_dist:
                min_dist = ds
                nearest = sample
        return nearest

    def _clear_hover(self) -> None:
        for canvas in self._route_canvases():
            canvas.delete("hover")
        self.rpm_canvas.delete("hover")

    def _draw_rpm_hover(self, s: float) -> None:
        view = self._rpm_view
        if not view or not self._profile_s:
            return
        left = view["left"]
        right = view["right"]
        top = view["top"]
        bottom = view["bottom"]
        max_abs = view["max_abs"]
        total = view["total"]
        if right <= left or total <= 0.0:
            return
        speeds = self._speed_at_s(s)
        if speeds[1] is None or speeds[2] is None:
            return

        def y_of(v: float) -> float:
            return top + (max_abs - v) * (bottom - top) / (2 * max_abs)

        x = left + s * (right - left) / total
        y_left = y_of(speeds[1])
        y_right = y_of(speeds[2])

        self.rpm_canvas.create_line(x, top, x, bottom, fill="#888888", dash=(3, 3), tags="hover")
        self.rpm_canvas.create_oval(x - 3, y_left - 3, x + 3, y_left + 3, fill="#cc0000", outline="#000000", tags="hover")
        self.rpm_canvas.create_oval(x - 3, y_right - 3, x + 3, y_right + 3, fill="#0044cc", outline="#000000", tags="hover")
        label = f"V_L={speeds[1]:.2f} m/s\nV_R={speeds[2]:.2f} m/s"
        self.rpm_canvas.create_text(x + 8, top + 8, text=label, anchor="nw", fill="#202020", tags="hover")
        self._set_pwm_from_speeds(speeds[1], speeds[2])

    def _notify_state_change(self) -> None:
        if self._suspend_state_notify:
            return
        if self.on_state_change:
            self.on_state_change()

    def _refresh_expected_from_commands(self) -> None:
        if self._expected_running:
            return
        self.prepare_expected()

    def _curvature_radii(self, sample: dict[str, float]) -> dict[str, Optional[float]]:
        segments = self._segments()
        if not segments:
            return {"center": None, "left": None, "right": None}
        meter_per_px = self._meters_per_px()
        t = sample["t"]
        seg_idx = int(sample["seg"])
        p0, p1, p2, p3 = segments[seg_idx]
        k_signed = curvature_signed(p0, p1, p2, p3, t)
        if k_signed is None or abs(k_signed) <= 1e-9:
            return {"center": None, "left": None, "right": None}
        r_signed_px = 1.0 / k_signed
        r_signed_m = r_signed_px * meter_per_px
        a1_m = self._a1_cm / 100.0
        a2_m = self._a2_cm / 100.0
        r_center = r_signed_m
        r_left = r_signed_m - a1_m
        r_right = r_signed_m + a2_m
        return {"center": r_center, "left": r_left, "right": r_right}

    def _on_pan_start(self, e: tk.Event) -> None:
        self._pan_canvas = self._resolve_canvas(e.widget if isinstance(e.widget, tk.Canvas) else None)
        self._pan_start = (e.x, e.y)
        self._pan_origin = self._view_pan

    def _on_pan_drag(self, e: tk.Event) -> None:
        if self._pan_start is None or self._pan_origin is None or self._pan_canvas is None:
            return
        dx = e.x - self._pan_start[0]
        dy = e.y - self._pan_start[1]
        scale = self._canvas_display_scale(self._pan_canvas)
        self._view_pan = (self._pan_origin[0] + dx / scale, self._pan_origin[1] + dy / scale)
        self._redraw(recompute_profile=False, notify_state=False)
        self._schedule_state_notify()

    def _on_pan_end(self, _e: tk.Event) -> None:
        self._pan_start = None
        self._pan_origin = None
        self._pan_canvas = None
        self._schedule_state_notify(0)

    def _on_zoom(self, e: tk.Event) -> None:
        canvas = self._resolve_canvas(e.widget if isinstance(e.widget, tk.Canvas) else None)
        if getattr(e, "num", None) == 4:
            delta = 1
        elif getattr(e, "num", None) == 5:
            delta = -1
        else:
            delta = 1 if e.delta > 0 else -1
        factor = 1.1 if delta > 0 else 1 / 1.1
        old_scale = self._view_scale
        new_scale = max(0.2, min(4.0, old_scale * factor))
        if abs(new_scale - old_scale) <= 1e-6:
            return
        # Keep the point under cursor fixed while zooming.
        mx, my = self._from_view((e.x, e.y), canvas)
        view_cx, view_cy = self._canvas_screen_center(canvas)
        self._view_scale = new_scale
        model_cx, model_cy = self._center()
        new_display_scale = self._canvas_display_scale(canvas)
        new_pan_x = (e.x - view_cx) / new_display_scale - (mx - model_cx)
        new_pan_y = (e.y - view_cy) / new_display_scale - (my - model_cy)
        self._view_pan = (new_pan_x, new_pan_y)
        self._redraw(recompute_profile=False, notify_state=False)
        self._schedule_state_notify()

    def _hit_test(self, x: float, y: float, canvas: Optional[tk.Canvas] = None) -> Optional[tuple[str, int]]:
        for idx, node in enumerate(self.nodes):
            ax, ay = node.anchor
            vx, vy = self._to_view((ax, ay), canvas)
            if self._dist(vx, vy, x, y) <= self._hit_radius:
                return ("anchor", idx)
            if node.handle_in:
                hx, hy = self._handle_pos(node.anchor, node.handle_in)
                hvx, hvy = self._to_view((hx, hy), canvas)
                if self._dist(hvx, hvy, x, y) <= self._hit_radius:
                    return ("in", idx)
            if node.handle_out:
                hx, hy = self._handle_pos(node.anchor, node.handle_out)
                hvx, hvy = self._to_view((hx, hy), canvas)
                if self._dist(hvx, hvy, x, y) <= self._hit_radius:
                    return ("out", idx)
        return None

    def _dist(self, ax: float, ay: float, bx: float, by: float) -> float:
        return math.hypot(ax - bx, ay - by)

    # ---------------- Log helpers ----------------

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

        add_group("A: Admin", [("A0", "A0 - disable D messages"), ("A1", "A1 - enable D messages"), ("A2", "A2 - set motor PID")])
        add_group("B: Requests", [("B0", "B0 - sync request"), ("B1", "B1 - read motor PID request")])
        add_group("C: Control", [("C0", "C0 - control command")])
        add_group("D: Data", [
            ("D0", "D0 - IMU data"),
            ("D1", "D1 - tacho RPM"),
            ("D2", "D2 - motor currents/voltage/temp"),
            ("D3", "D3 - sensor tensor data"),
        ])
        add_group("E: Errors", [("E0", "E0 - error code")])
        add_group("F: Responses", [("F0", "F0 - sync response"), ("F1", "F1 - motor PID response")])
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
