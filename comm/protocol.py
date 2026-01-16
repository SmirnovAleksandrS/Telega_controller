"""
Protocol framing/parsing.

Frame format (from PDF):
  0x7E | type(u8) | len(u8) | payload(len) | crc32(u32)

CRC32 is computed over: type + len + payload
(You can change to include 0x7E if your MCU does so, but PDF doesn't state that.)

Message payload formats (little-endian):
D0: u32 ts_ms + 9 * float32
D1: u32 ts_ms + i32 left_rpm + i32 right_rpm
D2: u32 ts_ms + i16 cur_l + i16 cur_r + i16 volt_l + i16 volt_r + i16 temp_l + i16 temp_r
B0: u16 seq + u32 pc_time_ms (u32)
F0: u32 t2_mcu_rx_ms + u32 t3_mcu_tx_ms
C0: u32 ts_ms + i16 left_cmd + i16 right_cmd + u16 duration_ms
A0/A1: see PDF (we implement builders).
"""

from __future__ import annotations
from dataclasses import dataclass
import struct
from typing import Optional, Union

from utils.crc32 import crc32_ieee

SOF = 0x7E

# Types
TYPE_D0_IMU   = 0xD0
TYPE_D1_TACHO = 0xD1
TYPE_D2_MOTOR = 0xD2
TYPE_B0_SYNC_REQ  = 0xB0
TYPE_F0_SYNC_RESP = 0xF0
TYPE_C0_CONTROL   = 0xC0
TYPE_A0_DISABLE_D = 0xA0
TYPE_A1_ENABLE_D  = 0xA1

@dataclass
class Frame:
    msg_type: int
    payload: bytes

@dataclass
class ImuData:
    ts_ms: int
    accel: tuple[float, float, float]
    magn: tuple[float, float, float]
    gyro: tuple[float, float, float]

@dataclass
class TachoData:
    ts_ms: int
    left_rpm: int
    right_rpm: int

@dataclass
class MotorData:
    ts_ms: int
    current_l: int
    current_r: int
    voltage_l: int
    voltage_r: int
    temp_l: int
    temp_r: int

@dataclass
class SyncResp:
    t2_rx_ms: int
    t3_tx_ms: int

ParsedMsg = Union[ImuData, TachoData, MotorData, SyncResp, Frame]

class StreamParser:
    """
    Incremental stream parser with resync on errors.
    """
    def __init__(self) -> None:
        self._buf = bytearray()

    def push(self, data: bytes) -> list[Frame]:
        self._buf.extend(data)
        out: list[Frame] = []

        while True:
            # Find SOF
            sof_idx = self._buf.find(bytes([SOF]))
            if sof_idx < 0:
                self._buf.clear()
                return out
            if sof_idx > 0:
                del self._buf[:sof_idx]

            # Need at least SOF + type + len
            if len(self._buf) < 3:
                return out

            msg_type = self._buf[1]
            ln = self._buf[2]

            full_len = 1 + 1 + 1 + ln + 4  # sof + type + len + payload + crc32
            if len(self._buf) < full_len:
                return out

            payload = bytes(self._buf[3:3+ln])
            crc_rx = struct.unpack_from("<I", self._buf, 3+ln)[0]
            crc_calc = crc32_ieee(bytes([msg_type, ln]) + payload)

            if crc_rx != crc_calc:
                # Drop SOF and resync
                del self._buf[0:1]
                continue

            out.append(Frame(msg_type=msg_type, payload=payload))
            del self._buf[:full_len]

        return out


def parse_frame(frame: Frame) -> ParsedMsg:
    """
    Convert raw frame to strongly-typed message when we know the type.
    Unknown types are returned as Frame.
    """
    t = frame.msg_type
    p = frame.payload

    if t == TYPE_D0_IMU and len(p) == 40:
        ts_ms = struct.unpack_from("<I", p, 0)[0]
        vals = struct.unpack_from("<9f", p, 4)
        accel = vals[0:3]
        magn  = vals[3:6]
        gyro  = vals[6:9]
        return ImuData(ts_ms=ts_ms, accel=accel, magn=magn, gyro=gyro)

    if t == TYPE_D1_TACHO and len(p) == 12:
        ts_ms, l, r = struct.unpack_from("<Iii", p, 0)
        return TachoData(ts_ms=ts_ms, left_rpm=l, right_rpm=r)

    if t == TYPE_D2_MOTOR and len(p) == 16:
        ts_ms = struct.unpack_from("<I", p, 0)[0]
        cur_l, cur_r, volt_l, volt_r, temp_l, temp_r = struct.unpack_from("<hhhhhh", p, 4)
        return MotorData(
            ts_ms=ts_ms,
            current_l=cur_l, current_r=cur_r,
            voltage_l=volt_l, voltage_r=volt_r,
            temp_l=temp_l, temp_r=temp_r
        )

    if t == TYPE_F0_SYNC_RESP and len(p) == 8:
        t2, t3 = struct.unpack_from("<II", p, 0)
        return SyncResp(t2_rx_ms=t2, t3_tx_ms=t3)

    return frame


def build_frame(msg_type: int, payload: bytes) -> bytes:
    """
    Build on-wire frame.
    """
    if len(payload) > 255:
        raise ValueError("payload too long")
    ln = len(payload)
    crc = crc32_ieee(bytes([msg_type, ln]) + payload)
    return struct.pack("<BBB", SOF, msg_type, ln) + payload + struct.pack("<I", crc)


def build_sync_req(seq: int, t1_pc_ms_u32: int) -> bytes:
    payload = struct.pack("<HI", seq & 0xFFFF, t1_pc_ms_u32 & 0xFFFFFFFF)
    return build_frame(TYPE_B0_SYNC_REQ, payload)


def build_control(ts_ms_u32: int, left_cmd: int, right_cmd: int, duration_ms: int) -> bytes:
    payload = struct.pack(
        "<IhhH",
        ts_ms_u32 & 0xFFFFFFFF,
        int(left_cmd),
        int(right_cmd),
        duration_ms & 0xFFFF
    )
    return build_frame(TYPE_C0_CONTROL, payload)


def build_disable_d(ts_ms_u32: int, d_type: int) -> bytes:
    payload = struct.pack("<IB", ts_ms_u32 & 0xFFFFFFFF, d_type & 0xFF)
    return build_frame(TYPE_A0_DISABLE_D, payload)


def build_enable_d(ts_ms_u32: int, d_type: int, period_ms: int) -> bytes:
    payload = struct.pack("<IBH", ts_ms_u32 & 0xFFFFFFFF, d_type & 0xFF, period_ms & 0xFFFF)
    return build_frame(TYPE_A1_ENABLE_D, payload)
