"""
Motion planning math for track speeds along a path.
All units are SI (meters, seconds) unless noted.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional

from utils.bezier_math import cubic_point, curvature_signed


@dataclass
class PathSample:
    s: float  # distance from start (m)
    k: float  # signed curvature (1/m)


@dataclass
class MotionParams:
    v_cm: float   # desired center speed (m/s), can be negative
    accel: float  # max acceleration (m/s^2)
    decel: float  # max braking (m/s^2)
    dt: float     # time quantum (s)
    a1: float     # CM to left track center (m)
    a2: float     # CM to right track center (m)


@dataclass
class MotionProfile:
    s: list[float]       # distance samples (m)
    v_cm: list[float]    # center speed per quantum (m/s)
    v_left: list[float]  # left track speed per quantum (m/s)
    v_right: list[float] # right track speed per quantum (m/s)


def build_path_samples(
    segments: list[tuple[tuple[float, float], ...]],
    samples_per_segment: int,
) -> list[PathSample]:
    samples: list[PathSample] = []
    if not segments:
        return samples
    prev: Optional[tuple[float, float]] = None
    s = 0.0
    for p0, p1, p2, p3 in segments:
        for i in range(samples_per_segment + 1):
            t = i / samples_per_segment
            x, y = cubic_point(p0, p1, p2, p3, t)
            if prev is not None:
                s += math.hypot(x - prev[0], y - prev[1])
            prev = (x, y)
            k = curvature_signed(p0, p1, p2, p3, t)
            # Use 0 curvature when derivatives are degenerate.
            samples.append(PathSample(s=s, k=0.0 if k is None else k))
    return samples


def track_speeds_from_center(v_cm: float, k: float, a1: float, a2: float) -> tuple[float, float]:
    # Isolated formula for easy future edits of the kinematics.
    v_left = v_cm * (1.0 - a1 * k)
    v_right = v_cm * (1.0 + a2 * k)
    return v_left, v_right


def center_speed_from_tracks(v_left: float, v_right: float, k: float, a1: float, a2: float) -> float:
    # Inverse mapping (averaged) to recover CM speed from track speeds.
    eps = 1e-6
    vals = []
    denom_l = 1.0 - a1 * k
    denom_r = 1.0 + a2 * k
    if abs(denom_l) > eps:
        vals.append(v_left / denom_l)
    if abs(denom_r) > eps:
        vals.append(v_right / denom_r)
    if vals:
        return sum(vals) / len(vals)
    return 0.5 * (v_left + v_right)


def plan_profile(samples: list[PathSample], params: MotionParams) -> MotionProfile:
    if not samples or params.dt <= 0.0:
        return MotionProfile(s=[], v_cm=[], v_left=[], v_right=[])

    total_len = samples[-1].s
    if total_len <= 1e-9 or abs(params.v_cm) <= 1e-9:
        return MotionProfile(s=[0.0], v_cm=[0.0], v_left=[0.0], v_right=[0.0])

    def k_at_s(s_val: float) -> float:
        # Interpolate curvature by distance for smooth planning.
        if s_val <= samples[0].s:
            return samples[0].k
        if s_val >= samples[-1].s:
            return samples[-1].k
        # Linear interpolation
        lo = 0
        hi = len(samples) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if samples[mid].s < s_val:
                lo = mid
            else:
                hi = mid
        s0 = samples[lo].s
        s1 = samples[hi].s
        if s1 <= s0:
            return samples[lo].k
        t = (s_val - s0) / (s1 - s0)
        return samples[lo].k + (samples[hi].k - samples[lo].k) * t

    def limited_step(cur: float, target: float) -> float:
        # Apply acceleration/braking limits per time quantum.
        if abs(cur - target) <= 1e-12:
            return target
        delta = target - cur
        # Choose accel or brake based on magnitude change or direction flip.
        if cur == 0.0:
            max_delta = params.accel * params.dt
        elif (cur > 0 and target > 0) or (cur < 0 and target < 0):
            max_delta = (params.accel if abs(target) > abs(cur) else params.decel) * params.dt
        else:
            max_delta = params.decel * params.dt
        if max_delta <= 0.0:
            return cur
        if abs(delta) > max_delta:
            delta = math.copysign(max_delta, delta)
        return cur + delta

    s_list: list[float] = []
    vcm_list: list[float] = []
    vl_list: list[float] = []
    vr_list: list[float] = []

    s = 0.0
    v_left = 0.0
    v_right = 0.0
    max_iters = int((total_len / max(abs(params.v_cm), 1e-3)) / params.dt) + 10_000

    for _ in range(max_iters):
        remaining = max(0.0, total_len - s)
        vcm_eff = center_speed_from_tracks(v_left, v_right, k_at_s(s), params.a1, params.a2)
        vcm_abs = abs(vcm_eff)
        # Stop planning when we reached the end and fully stopped.
        if remaining <= 1e-6 and vcm_abs <= 1e-6 and abs(v_left) <= 1e-6 and abs(v_right) <= 1e-6:
            break

        # Simple braking distance for CM to stop.
        if params.decel > 0.0:
            d_stop = (vcm_abs * vcm_abs) / (2.0 * params.decel)
        else:
            d_stop = 0.0

        target_v_cm = params.v_cm
        if remaining <= d_stop:
            target_v_cm = 0.0

        k = k_at_s(s)
        v_left_ref, v_right_ref = track_speeds_from_center(target_v_cm, k, params.a1, params.a2)

        v_left = limited_step(v_left, v_left_ref)
        v_right = limited_step(v_right, v_right_ref)

        vcm_eff = center_speed_from_tracks(v_left, v_right, k, params.a1, params.a2)
        s += abs(vcm_eff) * params.dt

        s_list.append(min(s, total_len))
        vcm_list.append(vcm_eff)
        vl_list.append(v_left)
        vr_list.append(v_right)

        if s >= total_len and target_v_cm == 0.0 and vcm_abs <= 1e-6:
            # Allow a few iterations to settle to zero.
            if abs(v_left) <= 1e-6 and abs(v_right) <= 1e-6:
                break

    return MotionProfile(s=s_list, v_cm=vcm_list, v_left=vl_list, v_right=vr_list)
