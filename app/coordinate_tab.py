"""
Coordinate tab:
- Scaled grid for trajectory planning
- Bezier curve editor with anchors/handles
- Minimum turning radius validation
"""

from __future__ import annotations

import math
from dataclasses import dataclass
import tkinter as tk
from tkinter import ttk
from typing import Optional

from app.styles import PANEL_BG, COLOR_GREEN, COLOR_RED
from utils.bezier_math import (
    cubic_point,
    cubic_derivative,
    curvature_signed,
    min_curve_radius,
)
from utils.motion_math import MotionParams, build_path_samples, plan_profile
from utils.speed_map import SpeedMapPoint, speed_to_pwm
from app.dialogs import SpeedMapConfig


@dataclass
class BezierNode:
    anchor: tuple[float, float]
    handle_in: Optional[tuple[float, float]] = None   # offset from anchor
    handle_out: Optional[tuple[float, float]] = None  # offset from anchor


class CoordinateTab(ttk.Frame):
    def __init__(self, master: tk.Widget) -> None:
        super().__init__(master, padding=6)

        self._grid_cells = 14
        self._canvas_size = 420
        self._hit_radius = 8
        self._handle_len = 60
        self._samples_per_seg = 60
        self._hover_px = 8
        self._view_scale = 1.0
        self._view_pan = (0.0, 0.0)
        self._pan_start: Optional[tuple[float, float]] = None
        self._pan_origin: Optional[tuple[float, float]] = None

        self._a1_cm = 20.0
        self._a2_cm = 20.0
        self._v_cm = 0.5
        self._accel = 0.5
        self._decel = 0.5
        self._dt_ms = 10.0
        self._speed_map = SpeedMapConfig()
        self._pwm_left = 1500.0
        self._pwm_right = 1500.0

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

        self._drag_target: Optional[tuple[str, int]] = None

        self._build()
        self._init_nodes()
        self._redraw()

    # ---------------- UI ----------------

    def _build(self) -> None:
        self.columnconfigure(0, weight=0)
        self.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)
        self.rowconfigure(1, weight=0)

        left = ttk.Frame(self)
        left.grid(row=0, column=0, sticky="nw")

        self.canvas = tk.Canvas(
            left,
            width=self._canvas_size,
            height=self._canvas_size,
            bg=PANEL_BG,
            highlightthickness=1,
            highlightbackground="#606060",
        )
        self.canvas.grid(row=0, column=0, sticky="nw")
        self.canvas.bind("<ButtonPress-1>", self._on_press)
        self.canvas.bind("<B1-Motion>", self._on_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_release)
        self.canvas.bind("<Double-Button-1>", self._on_double_click)
        self.canvas.bind("<Motion>", self._on_motion)
        self.canvas.bind("<ButtonPress-3>", self._on_pan_start)
        self.canvas.bind("<B3-Motion>", self._on_pan_drag)
        self.canvas.bind("<ButtonRelease-3>", self._on_pan_end)
        self.canvas.bind("<MouseWheel>", self._on_zoom)
        self.canvas.bind("<Button-4>", self._on_zoom)
        self.canvas.bind("<Button-5>", self._on_zoom)

        scale_row = ttk.Frame(left)
        scale_row.grid(row=1, column=0, sticky="w", pady=(6, 0))
        ttk.Label(scale_row, text="Scale").pack(side="left")
        self.scale_box = ttk.Combobox(
            scale_row,
            textvariable=self._scale_var,
            values=["10 cm", "20 cm", "50 cm", "1 m", "2 m"],
            width=6,
            state="readonly",
        )
        self.scale_box.pack(side="left", padx=(6, 0))
        self.scale_box.bind("<<ComboboxSelected>>", lambda _e: self._redraw())

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

    def _center(self) -> tuple[float, float]:
        half = self._canvas_size / 2.0
        return (half, half)

    def _to_view(self, pt: tuple[float, float]) -> tuple[float, float]:
        # View transform: scale around center, then pan.
        cx, cy = self._center()
        px, py = self._view_pan
        return ((pt[0] - cx) * self._view_scale + cx + px,
                (pt[1] - cy) * self._view_scale + cy + py)

    def _from_view(self, pt: tuple[float, float]) -> tuple[float, float]:
        # Inverse view transform for input handling.
        cx, cy = self._center()
        px, py = self._view_pan
        return ((pt[0] - cx - px) / self._view_scale + cx,
                (pt[1] - cy - py) / self._view_scale + cy)

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

    def set_geometry(self, a1_cm: float, a2_cm: float) -> None:
        self._a1_cm = max(a1_cm, 0.0)
        self._a2_cm = max(a2_cm, 0.0)
        self._redraw()

    def set_speed_map(self, cfg: SpeedMapConfig) -> None:
        self._speed_map = cfg
        self._update_pwm_display()

    # ---------------- Drawing ----------------

    def _redraw(self) -> None:
        self.canvas.delete("grid")
        self.canvas.delete("curve")
        self.canvas.delete("nodes")
        self.canvas.delete("hover")
        self.rpm_canvas.delete("hover")
        self._draw_grid()
        self._draw_curve()
        self._draw_nodes()
        self._recompute_profile()

    def _draw_grid(self) -> None:
        step = self._cell_px()
        # Compute visible model bounds to draw an infinite grid.
        min_model = self._from_view((0.0, 0.0))
        max_model = self._from_view((self._canvas_size, self._canvas_size))
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
            y = self._to_view((0.0, pos))[1]
            self.canvas.create_line(0, y, self._canvas_size, y, fill="#8a8a8a", tags="grid")
        for i in range(start_x, end_x + 1):
            pos = i * step
            x = self._to_view((pos, 0.0))[0]
            self.canvas.create_line(x, 0, x, self._canvas_size, fill="#8a8a8a", tags="grid")

    def _draw_curve(self) -> None:
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
                vx, vy = self._to_view((x, y))
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
                left_v = self._to_view((x + nx * a1_px, y + ny * a1_px))
                right_v = self._to_view((x - nx * a2_px, y - ny * a2_px))
                left_pts.extend([left_v[0], left_v[1]])
                right_pts.extend([right_v[0], right_v[1]])
        color = self._curve_color(segments)
        if left_pts:
            self.canvas.create_line(left_pts, fill="#9aa6e8", width=1, smooth=True, tags="curve")
        if right_pts:
            self.canvas.create_line(right_pts, fill="#9aa6e8", width=1, smooth=True, tags="curve")
        self.canvas.create_line(points, fill=color, width=2, smooth=True, tags="curve")

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
        for v in self._profile_v_left + self._profile_v_right:
            max_abs = max(max_abs, abs(v))

        def y_of(v: float) -> float:
            return top + (max_abs - v) * (bottom - top) / (2 * max_abs)

        total = max(self._profile_s[-1], 1e-6)
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

        def draw_series(values: list[float], color: str) -> None:
            pts = []
            for s_val, v in zip(self._profile_s, values):
                pts.extend([x_of(s_val), y_of(v)])
            if pts:
                c.create_line(*pts, fill=color, width=2)

        draw_series(self._profile_v_left, "#cc0000")
        draw_series(self._profile_v_right, "#0044cc")
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

    def _draw_nodes(self) -> None:
        for idx, node in enumerate(self.nodes):
            ax, ay = node.anchor
            vx, vy = self._to_view((ax, ay))
            self._draw_point(vx, vy, 5, "#000000", "nodes")
            if node.handle_in:
                hx, hy = self._handle_pos(node.anchor, node.handle_in)
                hvx, hvy = self._to_view((hx, hy))
                self.canvas.create_line(vx, vy, hvx, hvy, fill="#5a5a5a", tags="nodes")
                self._draw_point(hvx, hvy, 4, "#3a6bbf", "nodes")
            if node.handle_out:
                hx, hy = self._handle_pos(node.anchor, node.handle_out)
                hvx, hvy = self._to_view((hx, hy))
                self.canvas.create_line(vx, vy, hvx, hvy, fill="#5a5a5a", tags="nodes")
                self._draw_point(hvx, hvy, 4, "#3a6bbf", "nodes")

    def _draw_point(self, x: float, y: float, r: int, fill: str, tag: str) -> None:
        self.canvas.create_oval(x - r, y - r, x + r, y + r, fill=fill, outline="#000000", tags=tag)

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

    def _on_press(self, e: tk.Event) -> None:
        hit = self._hit_test(e.x, e.y)
        if hit:
            self._drag_target = hit
            return
        mx, my = self._from_view((e.x, e.y))
        self._add_node(mx, my)
        self._drag_target = ("anchor", len(self.nodes) - 1)

    def _on_drag(self, e: tk.Event) -> None:
        if not self._drag_target:
            return
        kind, idx = self._drag_target
        node = self.nodes[idx]
        if kind == "anchor":
            if idx == 0:
                return
            mx, my = self._from_view((e.x, e.y))
            node.anchor = (mx, my)
        elif kind == "in":
            mx, my = self._from_view((e.x, e.y))
            offset = self._handle_offset(node.anchor, mx, my)
            self._set_symmetric_handle(node, idx, "in", offset)
        elif kind == "out":
            if idx == 0:
                _, my = self._from_view((e.x, e.y))
                dy = my - node.anchor[1]
                offset = (0.0, min(dy, -5.0))
                node.handle_out = offset
            else:
                mx, my = self._from_view((e.x, e.y))
                offset = self._handle_offset(node.anchor, mx, my)
                self._set_symmetric_handle(node, idx, "out", offset)
        self._redraw()

    def _on_release(self, _e: tk.Event) -> None:
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
        hit = self._hit_test(e.x, e.y)
        if not hit:
            return
        kind, idx = hit
        if kind != "anchor" or idx == 0:
            return
        self.nodes.pop(idx)
        self._redraw()

    def _on_motion(self, e: tk.Event) -> None:
        self._clear_hover()
        if not self._samples:
            return
        nearest = None
        min_dist = None
        for sample in self._samples:
            sx, sy = self._to_view((sample["x"], sample["y"]))
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
        vx, vy = self._to_view((x, y))
        nx = sample["nx"]
        ny = sample["ny"]
        s = sample.get("s", 0.0)
        a1_px = self._meters_to_px(self._a1_cm / 100.0)
        a2_px = self._meters_to_px(self._a2_cm / 100.0)
        left_m = (x + nx * a1_px, y + ny * a1_px)
        right_m = (x - nx * a2_px, y - ny * a2_px)
        left = self._to_view(left_m)
        right = self._to_view(right_m)

        self._draw_point(vx, vy, 4, "#000000", "hover")
        self._draw_point(left[0], left[1], 4, "#3a6bbf", "hover")
        self._draw_point(right[0], right[1], 4, "#3a6bbf", "hover")

        radii = self._curvature_radii(sample)
        speeds = self._speed_at_s(s)
        self._draw_radius_label((vx + 8, vy - 8), radii["center"], speeds[0], "hover")
        self._draw_radius_label((left[0] + 8, left[1] - 8), radii["left"], speeds[1], "hover")
        self._draw_radius_label((right[0] + 8, right[1] - 8), radii["right"], speeds[2], "hover")
        if speeds[1] is not None and speeds[2] is not None:
            self._set_pwm_from_speeds(speeds[1], speeds[2])

    def _draw_radius_label(self, pos: tuple[float, float], radius_m: Optional[float], speed_m_s: Optional[float], tag: str) -> None:
        label = "R=inf"
        if radius_m is not None:
            radius_cm = abs(radius_m) * 100.0
            label = f"R={radius_cm:.1f} cm"
        if speed_m_s is not None:
            label = f"{label}\nV={speed_m_s:.2f} m/s"
        self.canvas.create_text(pos[0], pos[1], text=label, fill="#202020", anchor="w", tags=tag)

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
        self.canvas.delete("hover")
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
        self._pan_start = (e.x, e.y)
        self._pan_origin = self._view_pan

    def _on_pan_drag(self, e: tk.Event) -> None:
        if self._pan_start is None or self._pan_origin is None:
            return
        dx = e.x - self._pan_start[0]
        dy = e.y - self._pan_start[1]
        self._view_pan = (self._pan_origin[0] + dx, self._pan_origin[1] + dy)
        self._redraw()

    def _on_pan_end(self, _e: tk.Event) -> None:
        self._pan_start = None
        self._pan_origin = None

    def _on_zoom(self, e: tk.Event) -> None:
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
        mx, my = self._from_view((e.x, e.y))
        cx, cy = self._center()
        px, py = self._view_pan
        self._view_scale = new_scale
        new_px = e.x - cx - (mx - cx) * new_scale
        new_py = e.y - cy - (my - cy) * new_scale
        self._view_pan = (new_px, new_py)
        self._redraw()

    def _hit_test(self, x: float, y: float) -> Optional[tuple[str, int]]:
        for idx, node in enumerate(self.nodes):
            ax, ay = node.anchor
            vx, vy = self._to_view((ax, ay))
            if self._dist(vx, vy, x, y) <= self._hit_radius:
                return ("anchor", idx)
            if node.handle_in:
                hx, hy = self._handle_pos(node.anchor, node.handle_in)
                hvx, hvy = self._to_view((hx, hy))
                if self._dist(hvx, hvy, x, y) <= self._hit_radius:
                    return ("in", idx)
            if node.handle_out:
                hx, hy = self._handle_pos(node.anchor, node.handle_out)
                hvx, hvy = self._to_view((hx, hy))
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
