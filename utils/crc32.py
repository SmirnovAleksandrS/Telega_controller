"""
CRC32 helper.

We use standard IEEE CRC-32 (same as zlib.crc32).
On-wire crc32 is u32 little-endian unless your MCU uses big-endian.
Here we assume LITTLE-endian (most STM32 code writes uint32_t as LE on UART).
If your MCU sends BE, switch pack/unpack in protocol.py.
"""

from __future__ import annotations
import zlib

def crc32_ieee(data: bytes) -> int:
    # zlib.crc32 returns signed on some pythons, so mask to u32
    return zlib.crc32(data) & 0xFFFFFFFF
