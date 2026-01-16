"""
NTP-like time synchronization (PC estimates a and b):
  t_mcu ≈ a * t_pc + b

PDF describes:
- initial N rounds, compute (delta_i, tmid_pc_i, tmid_mcu_i), take K best by delta
- estimate a,b from earliest and latest among K
- later periodic single round, compute raw a,b from prev and cur, apply EMA with beta

We implement it in milliseconds (int64 on PC),
while sync packets use u32 ms (wrap possible).
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Optional

from utils.timebase import u32_diff_mod

@dataclass
class SyncPoint:
    delta_ms: int
    tmid_pc_ms: int
    tmid_mcu_ms: int

@dataclass
class TimeModel:
    a: float = 1.0
    b: float = 0.0
    have_lock: bool = False
    last_good: Optional[SyncPoint] = None

    def mcu_from_pc(self, t_pc_ms: int) -> float:
        return self.a * float(t_pc_ms) + self.b

    def pc_from_mcu(self, t_mcu_ms: int) -> float:
        if self.a == 0:
            return 0.0
        return (float(t_mcu_ms) - self.b) / self.a


def compute_sync_point(t1_pc_u32: int, t4_pc_u32: int, t2_mcu_u32: int, t3_mcu_u32: int, pc_now_ms_i64: int) -> SyncPoint:
    """
    All inputs are u32(ms) from packets/measurements, but we need stable ms.

    We treat differences via modular arithmetic for u32 wrap.
    For tmid_pc_ms we anchor to pc_now_ms_i64:
      t4_pc_u32 corresponds to pc_now_ms_i64 (measured at receive).
      Then any other u32 timestamp is mapped to int64 by applying modular diff.
    """
    # Map u32 to i64 using t4 as anchor
    # pc_i64(x) = pc_now + (x - t4_u32)mod
    def pc_u32_to_i64(x_u32: int) -> int:
        return int(pc_now_ms_i64 + u32_diff_mod(x_u32, t4_pc_u32))

    t1_pc_i64 = pc_u32_to_i64(t1_pc_u32)
    t4_pc_i64 = pc_u32_to_i64(t4_pc_u32)  # equals pc_now_ms_i64

    # Compute delta per PDF:
    # delta = (t4 - t1) - (t3 - t2)
    rtt_pc = t4_pc_i64 - t1_pc_i64
    proc_mcu = u32_diff_mod(t3_mcu_u32, t2_mcu_u32)
    delta = int(rtt_pc - proc_mcu)

    tmid_pc = int((t1_pc_i64 + t4_pc_i64) // 2)

    # MCU midpoint is still u32-ish; for our model we can keep it as int in same ms scale
    # Without a current MCU->PC mapping yet, we keep mcu midpoint as "u32 expanded" around t2.
    # For stable differences between points, it's enough to unwrap relative to previous later.
    # Here: use midpoint in u32 domain, but stored in int.
    # We'll unwrap in estimator using modular diffs as needed.
    # But simplest: store raw u32 midpoint as int, and when computing between points use u32_diff_mod.
    tmid_mcu_u32 = ( (t2_mcu_u32 + t3_mcu_u32) // 2 ) & 0xFFFFFFFF

    return SyncPoint(delta_ms=delta, tmid_pc_ms=tmid_pc, tmid_mcu_ms=int(tmid_mcu_u32))


def estimate_initial(points: list[SyncPoint], k_best: int = 5) -> Optional[TimeModel]:
    if len(points) < 2:
        return None

    pts = sorted(points, key=lambda p: p.delta_ms)
    pts = pts[:max(2, min(k_best, len(pts)))]

    # Choose earliest and latest by PC time
    pts_sorted_pc = sorted(pts, key=lambda p: p.tmid_pc_ms)
    first = pts_sorted_pc[0]
    last = pts_sorted_pc[-1]

    dt_pc = last.tmid_pc_ms - first.tmid_pc_ms
    if dt_pc == 0:
        return None

    # MCU dt uses modular unwrap
    dt_mcu = u32_diff_mod(last.tmid_mcu_ms, first.tmid_mcu_ms)

    a = float(dt_mcu) / float(dt_pc)
    b = float(first.tmid_mcu_ms) - a * float(first.tmid_pc_ms)

    m = TimeModel(a=a, b=b, have_lock=True, last_good=last)
    return m


def update_model(model: TimeModel, prev: SyncPoint, cur: SyncPoint, beta: float = 0.05) -> None:
    """
    Update a,b from two consecutive good points and EMA smoothing.
    """
    dt_pc = cur.tmid_pc_ms - prev.tmid_pc_ms
    if dt_pc == 0:
        return
    dt_mcu = u32_diff_mod(cur.tmid_mcu_ms, prev.tmid_mcu_ms)

    a_raw = float(dt_mcu) / float(dt_pc)
    b_raw = float(cur.tmid_mcu_ms) - a_raw * float(cur.tmid_pc_ms)

    model.a = (1.0 - beta) * model.a + beta * a_raw
    model.b = (1.0 - beta) * model.b + beta * b_raw
    model.have_lock = True
    model.last_good = cur
