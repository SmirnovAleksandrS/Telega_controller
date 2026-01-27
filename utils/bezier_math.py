"""
Bezier curve math helpers.
"""

from __future__ import annotations

import math
from typing import Iterable, Optional


def cubic_point(
    p0: tuple[float, float],
    p1: tuple[float, float],
    p2: tuple[float, float],
    p3: tuple[float, float],
    t: float,
) -> tuple[float, float]:
    u = 1.0 - t
    tt = t * t
    uu = u * u
    uuu = uu * u
    ttt = tt * t
    x = (uuu * p0[0]) + (3 * uu * t * p1[0]) + (3 * u * tt * p2[0]) + (ttt * p3[0])
    y = (uuu * p0[1]) + (3 * uu * t * p1[1]) + (3 * u * tt * p2[1]) + (ttt * p3[1])
    return (x, y)


def cubic_derivative(
    p0: tuple[float, float],
    p1: tuple[float, float],
    p2: tuple[float, float],
    p3: tuple[float, float],
    t: float,
) -> tuple[float, float]:
    u = 1.0 - t
    x = (3 * u * u * (p1[0] - p0[0])) + (6 * u * t * (p2[0] - p1[0])) + (3 * t * t * (p3[0] - p2[0]))
    y = (3 * u * u * (p1[1] - p0[1])) + (6 * u * t * (p2[1] - p1[1])) + (3 * t * t * (p3[1] - p2[1]))
    return (x, y)


def cubic_second_derivative(
    p0: tuple[float, float],
    p1: tuple[float, float],
    p2: tuple[float, float],
    p3: tuple[float, float],
    t: float,
) -> tuple[float, float]:
    u = 1.0 - t
    x = (6 * u * (p2[0] - 2 * p1[0] + p0[0])) + (6 * t * (p3[0] - 2 * p2[0] + p1[0]))
    y = (6 * u * (p2[1] - 2 * p1[1] + p0[1])) + (6 * t * (p3[1] - 2 * p2[1] + p1[1]))
    return (x, y)


def curvature_radius(
    p0: tuple[float, float],
    p1: tuple[float, float],
    p2: tuple[float, float],
    p3: tuple[float, float],
    t: float,
) -> Optional[float]:
    dx, dy = cubic_derivative(p0, p1, p2, p3, t)
    ddx, ddy = cubic_second_derivative(p0, p1, p2, p3, t)
    denom = (dx * dx + dy * dy) ** 1.5
    if denom <= 1e-9:
        return None
    num = abs(dx * ddy - dy * ddx)
    if num <= 1e-12:
        return None
    return denom / num


def curvature_signed(
    p0: tuple[float, float],
    p1: tuple[float, float],
    p2: tuple[float, float],
    p3: tuple[float, float],
    t: float,
) -> Optional[float]:
    dx, dy = cubic_derivative(p0, p1, p2, p3, t)
    ddx, ddy = cubic_second_derivative(p0, p1, p2, p3, t)
    denom = (dx * dx + dy * dy) ** 1.5
    if denom <= 1e-9:
        return None
    return (dx * ddy - dy * ddx) / denom


def min_curve_radius(
    segments: Iterable[tuple[tuple[float, float], ...]],
    samples_per_segment: int,
) -> Optional[float]:
    min_radius: Optional[float] = None
    for p0, p1, p2, p3 in segments:
        for i in range(samples_per_segment + 1):
            t = i / samples_per_segment
            radius = curvature_radius(p0, p1, p2, p3, t)
            if radius is None:
                continue
            if min_radius is None or radius < min_radius:
                min_radius = radius
    return min_radius
