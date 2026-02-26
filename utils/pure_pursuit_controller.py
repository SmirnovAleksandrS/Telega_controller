"""
Pure Pursuit controller for a differential/tracked robot.

The module expects external pose integration/odometry and returns
track speeds in m/s on each control tick.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, sin, sqrt
from typing import List, Optional, Protocol, Tuple


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    theta: float


@dataclass(frozen=True)
class WheelSpeeds:
    v_left: float
    v_right: float


@dataclass(frozen=True)
class DebugInfo:
    s_near: float
    s_progress: float
    lookahead_distance: float
    lookahead_point: Tuple[float, float]
    x_r: float
    y_r: float
    kappa: float
    v_curve_max: float
    v_stop_max: Optional[float]
    v_target: float
    v_cm_cmd_before_tracks: float
    omega_cmd_before_tracks: float
    v_left_raw: float
    v_right_raw: float
    v_left_cmd: float
    v_right_cmd: float
    v_cm_after_tracks: float
    omega_after_tracks: float


class Track(Protocol):
    def length(self) -> float:
        ...

    def position(self, s: float) -> Tuple[float, float]:
        ...

    def nearest_s(self, x: float, y: float) -> float:
        ...


@dataclass
class ControllerConfig:
    track_width: float
    v_cm_max: float
    a_cm_max: float
    a_track_up: float
    a_track_down: float
    L_min: float
    L_max: float
    K_L: float
    K_kappa: float
    enable_stop_profile: bool = True
    a_stop: float = 0.5
    v_eps: float = 1e-6
    allow_progress_backtrack: bool = False


def clamp(x: float, lo: float, hi: float) -> float:
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


def slew_limit(prev: float, target: float, a_max: float, dt: float) -> float:
    if dt <= 0.0:
        return target
    max_step = a_max * dt
    delta = target - prev
    if delta > max_step:
        return prev + max_step
    if delta < -max_step:
        return prev - max_step
    return target


def slew_limit_asym(prev: float, target: float, a_up: float, a_down: float, dt: float) -> float:
    if dt <= 0.0:
        return target
    accelerating = abs(target) > abs(prev)
    a_max = a_up if accelerating else a_down
    max_step = a_max * dt
    delta = target - prev
    if delta > max_step:
        return prev + max_step
    if delta < -max_step:
        return prev - max_step
    return target


def world_to_robot(pose: Pose2D, point: Tuple[float, float]) -> Tuple[float, float]:
    dx = point[0] - pose.x
    dy = point[1] - pose.y
    c = cos(pose.theta)
    s = sin(pose.theta)
    x_r = c * dx + s * dy
    y_r = -s * dx + c * dy
    return x_r, y_r


class PurePursuitController:
    def __init__(self, config: ControllerConfig, track: Track):
        self.cfg = config
        self.track = track
        self.s_progress = 0.0
        self.v_cm_prev = 0.0
        self.v_left_prev = 0.0
        self.v_right_prev = 0.0
        self.finished = False

    def reset(self) -> None:
        self.s_progress = 0.0
        self.v_cm_prev = 0.0
        self.v_left_prev = 0.0
        self.v_right_prev = 0.0
        self.finished = False

    def update(self, pose: Pose2D, dt: float, debug: bool = False) -> Tuple[WheelSpeeds, Optional[DebugInfo]]:
        if self.finished:
            speeds = WheelSpeeds(0.0, 0.0)
            dbg = None
            if debug:
                dbg = DebugInfo(
                    s_near=self.s_progress,
                    s_progress=self.s_progress,
                    lookahead_distance=0.0,
                    lookahead_point=(pose.x, pose.y),
                    x_r=0.0,
                    y_r=0.0,
                    kappa=0.0,
                    v_curve_max=0.0,
                    v_stop_max=0.0,
                    v_target=0.0,
                    v_cm_cmd_before_tracks=0.0,
                    omega_cmd_before_tracks=0.0,
                    v_left_raw=0.0,
                    v_right_raw=0.0,
                    v_left_cmd=0.0,
                    v_right_cmd=0.0,
                    v_cm_after_tracks=0.0,
                    omega_after_tracks=0.0,
                )
            return speeds, dbg

        s_near = clamp(self.track.nearest_s(pose.x, pose.y), 0.0, self.track.length())
        if self.cfg.allow_progress_backtrack:
            self.s_progress = s_near
        elif s_near > self.s_progress:
            self.s_progress = s_near

        L_d = clamp(self.cfg.L_min + self.cfg.K_L * abs(self.v_cm_prev), self.cfg.L_min, self.cfg.L_max)
        s_look = clamp(self.s_progress + L_d, 0.0, self.track.length())
        p_look = self.track.position(s_look)
        x_r, y_r = world_to_robot(pose, p_look)

        # Use geometric distance to lookahead point for better off-track behavior.
        look_dist2 = x_r * x_r + y_r * y_r
        if look_dist2 < 1e-12:
            kappa = 0.0
        else:
            kappa = 2.0 * y_r / look_dist2
        # Keep curvature bounded to avoid unstable spin commands.
        kappa_max = 2.0 / max(self.cfg.L_min, 1e-6)
        kappa = clamp(kappa, -kappa_max, kappa_max)

        v_curve_max = self.cfg.v_cm_max / (1.0 + self.cfg.K_kappa * abs(kappa))
        if x_r < 0.0:
            # If lookahead ended up behind robot, slow down to let heading recover.
            v_curve_max = min(v_curve_max, 0.35 * self.cfg.v_cm_max)

        v_stop_max: Optional[float] = None
        if self.cfg.enable_stop_profile:
            s_remain = max(self.track.length() - self.s_progress, 0.0)
            v_stop_max = sqrt(max(2.0 * self.cfg.a_stop * s_remain, 0.0))

        v_target = min(v_curve_max, self.cfg.v_cm_max)
        if v_stop_max is not None:
            v_target = min(v_target, v_stop_max)

        v_cm_cmd = slew_limit(self.v_cm_prev, v_target, self.cfg.a_cm_max, dt)
        omega_cmd = v_cm_cmd * kappa
        B = max(self.cfg.track_width, 1e-9)

        v_left_raw = v_cm_cmd - (omega_cmd * B * 0.5)
        v_right_raw = v_cm_cmd + (omega_cmd * B * 0.5)

        v_left_cmd = slew_limit_asym(
            self.v_left_prev,
            v_left_raw,
            self.cfg.a_track_up,
            self.cfg.a_track_down,
            dt,
        )
        v_right_cmd = slew_limit_asym(
            self.v_right_prev,
            v_right_raw,
            self.cfg.a_track_up,
            self.cfg.a_track_down,
            dt,
        )

        v_cm_after = 0.5 * (v_left_cmd + v_right_cmd)
        omega_after = (v_right_cmd - v_left_cmd) / B
        self.v_cm_prev = v_cm_after
        self.v_left_prev = v_left_cmd
        self.v_right_prev = v_right_cmd

        if self.cfg.enable_stop_profile:
            near_end = (self.track.length() - self.s_progress) < max(self.cfg.L_min, 0.02)
            slow = abs(v_cm_after) < 1e-3 and abs(omega_after) < 1e-3
            if near_end and slow:
                self.finished = True

        speeds = WheelSpeeds(v_left=v_left_cmd, v_right=v_right_cmd)
        dbg = None
        if debug:
            dbg = DebugInfo(
                s_near=s_near,
                s_progress=self.s_progress,
                lookahead_distance=L_d,
                lookahead_point=p_look,
                x_r=x_r,
                y_r=y_r,
                kappa=kappa,
                v_curve_max=v_curve_max,
                v_stop_max=v_stop_max,
                v_target=v_target,
                v_cm_cmd_before_tracks=v_cm_cmd,
                omega_cmd_before_tracks=omega_cmd,
                v_left_raw=v_left_raw,
                v_right_raw=v_right_raw,
                v_left_cmd=v_left_cmd,
                v_right_cmd=v_right_cmd,
                v_cm_after_tracks=v_cm_after,
                omega_after_tracks=omega_after,
            )
        return speeds, dbg


class SimplePolylineTrack:
    def __init__(self, points: List[Tuple[float, float]]):
        if len(points) < 2:
            raise ValueError("Polyline track requires at least 2 points.")
        self.points = points
        self.seg_len: List[float] = []
        self.s_prefix: List[float] = [0.0]
        total = 0.0
        for i in range(len(points) - 1):
            x0, y0 = points[i]
            x1, y1 = points[i + 1]
            L = sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0))
            self.seg_len.append(L)
            total += L
            self.s_prefix.append(total)
        self._length = total

    def length(self) -> float:
        return self._length

    def position(self, s: float) -> Tuple[float, float]:
        s = clamp(s, 0.0, self._length)
        for i in range(len(self.seg_len)):
            s0 = self.s_prefix[i]
            s1 = self.s_prefix[i + 1]
            if s <= s1 or i == len(self.seg_len) - 1:
                L = max(self.seg_len[i], 1e-12)
                t = (s - s0) / L
                x0, y0 = self.points[i]
                x1, y1 = self.points[i + 1]
                return (x0 + t * (x1 - x0), y0 + t * (y1 - y0))
        return self.points[-1]

    def nearest_s(self, x: float, y: float) -> float:
        best_dist2 = float("inf")
        best_s = 0.0
        for i in range(len(self.seg_len)):
            x0, y0 = self.points[i]
            x1, y1 = self.points[i + 1]
            vx = x1 - x0
            vy = y1 - y0
            seg2 = vx * vx + vy * vy
            if seg2 < 1e-12:
                px, py = x0, y0
                t = 0.0
            else:
                t = ((x - x0) * vx + (y - y0) * vy) / seg2
                t = clamp(t, 0.0, 1.0)
                px = x0 + t * vx
                py = y0 + t * vy
            dx = x - px
            dy = y - py
            dist2 = dx * dx + dy * dy
            if dist2 < best_dist2:
                best_dist2 = dist2
                best_s = self.s_prefix[i] + t * self.seg_len[i]
        return best_s
