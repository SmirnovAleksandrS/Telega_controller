"""
Speed to PWM mapping helpers.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable


@dataclass
class SpeedMapPoint:
    pwm: float
    speed: float  # m/s


def speed_to_pwm(speed: float, points: Iterable[SpeedMapPoint]) -> float:
    pts = sorted(points, key=lambda p: p.speed)
    if not pts:
        return 0.0
    if speed <= pts[0].speed:
        return pts[0].pwm
    if speed >= pts[-1].speed:
        return pts[-1].pwm
    for a, b in zip(pts[:-1], pts[1:]):
        if a.speed <= speed <= b.speed:
            if b.speed == a.speed:
                return a.pwm
            t = (speed - a.speed) / (b.speed - a.speed)
            return a.pwm + (b.pwm - a.pwm) * t
    return pts[-1].pwm
