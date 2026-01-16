"""
Time helpers.

We use monotonic time in milliseconds for all PC-side computations.
"""

from __future__ import annotations
import time

U32_MOD = 2**32

def now_ms_monotonic() -> int:
    """Monotonic time in milliseconds as int64."""
    return int(time.monotonic() * 1000)

def u32(x: int) -> int:
    """Cast to u32 range."""
    return x & 0xFFFFFFFF

def u32_diff_mod(a: int, b: int) -> int:
    """
    Compute (a - b) in modular u32 arithmetic as signed-ish minimal distance.
    For our sync deltas we need forward difference where wrap can occur.
    We'll return a value in range [-2^31, 2^31-1].
    """
    d = (a - b) & 0xFFFFFFFF
    if d & 0x80000000:
        d = d - 0x100000000
    return d
