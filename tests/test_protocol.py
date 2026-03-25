from __future__ import annotations

import struct
import unittest

from comm.protocol import (
    TYPE_D1_TACHO,
    TYPE_D2_MOTOR,
    TYPE_D3_SENSOR_TENSOR,
    Frame,
    MotorData,
    SensorTensorData,
    StreamParser,
    TachoData,
    build_frame,
    build_control,
    parse_frame,
)
from utils.crc32 import crc32_ieee


class ProtocolTests(unittest.TestCase):
    def test_build_frame_uses_type_len_and_payload_crc(self) -> None:
        packet = build_control(0x11223344, 1500, 1500, 100)

        msg_type = packet[1]
        ln = packet[2]
        payload = packet[3:3 + ln]
        crc = struct.unpack_from("<I", packet, 3 + ln)[0]

        self.assertEqual(crc, crc32_ieee(bytes([msg_type, ln]) + payload))

    def test_stream_parser_accepts_live_d1_frame(self) -> None:
        raw = build_frame(TYPE_D1_TACHO, struct.pack("<Iii", 525249, 0, 0))

        parser = StreamParser()
        frames = parser.push(raw)

        self.assertEqual(len(frames), 1)
        parsed = parse_frame(frames[0])
        self.assertIsInstance(parsed, TachoData)
        assert isinstance(parsed, TachoData)
        self.assertEqual(parsed.ts_ms, 525249)
        self.assertEqual(parsed.left_rpm, 0)
        self.assertEqual(parsed.right_rpm, 0)

    def test_motor_data_parses_d2_payload(self) -> None:
        payload = struct.pack("<Ihhhhhh", 100, 1, 2, 3, 4, 5, 6)

        parsed = parse_frame(Frame(msg_type=TYPE_D2_MOTOR, payload=payload))

        self.assertIsInstance(parsed, MotorData)
        assert isinstance(parsed, MotorData)
        self.assertEqual(parsed.ts_ms, 100)
        self.assertEqual(parsed.current_l, 1)
        self.assertEqual(parsed.temp_r, 6)

    def test_sensor_tensor_data_parses_d3_payload(self) -> None:
        payload = struct.pack(
            "<I12f",
            77,
            1.0, 2.0, 3.0,
            4.0, 5.0, 6.0,
            7.0, 8.0, 9.0,
            10.0, 11.0, 12.0,
        )

        parsed = parse_frame(Frame(msg_type=TYPE_D3_SENSOR_TENSOR, payload=payload))

        self.assertIsInstance(parsed, SensorTensorData)
        assert isinstance(parsed, SensorTensorData)
        self.assertEqual(parsed.ts_ms, 77)
        self.assertEqual(parsed.linear_velocity, (1.0, 2.0, 3.0))
        self.assertEqual(parsed.angular_velocity, (4.0, 5.0, 6.0))
        self.assertEqual(parsed.linear_quality, (7.0, 8.0, 9.0))
        self.assertEqual(parsed.angular_quality, (10.0, 11.0, 12.0))


if __name__ == "__main__":
    unittest.main()
