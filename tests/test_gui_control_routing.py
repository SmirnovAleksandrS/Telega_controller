from __future__ import annotations

import struct
import types
import unittest

from app.gui_app import VirtualControllerApp
from app.manual_tab import ManualControlState
from comm.protocol import Frame, TYPE_C0_CONTROL, TYPE_D3_SENSOR_TENSOR, parse_frame
from comm.time_sync import TimeModel
from runtime.contracts import AutopilotMode


class _DummyCoordTab:
    def __init__(self) -> None:
        self.running = None
        self.stop_expected_calls = 0

    def get_autopilot_mode(self) -> AutopilotMode:
        return AutopilotMode.BUILTIN_PURE_PURSUIT

    def step_controller(self, _dt_s: float, force_internal_pose: bool = False) -> tuple[int, int]:
        self.force_internal_pose = force_internal_pose
        return (1600, 1400)

    def controller_finished(self) -> bool:
        return False

    def set_running(self, value: bool) -> None:
        self.running = value

    def stop_expected(self) -> None:
        self.stop_expected_calls += 1


class GuiControlRoutingTests(unittest.TestCase):
    def test_manual_control_state_defaults_to_neutral_pwm(self) -> None:
        state = ManualControlState()
        self.assertEqual((state.left_cmd, state.right_cmd), (1500, 1500))

    def test_coordinate_tick_preserves_left_right_order_on_uart(self) -> None:
        app = object.__new__(VirtualControllerApp)
        sent: list[bytes] = []
        coord_tab = _DummyCoordTab()

        app._coord_running = True
        app._kill_active = False
        app._coord_tick_ms = 50
        app._coord_mission = None
        app._manual_neutral = 1500
        app.worker = types.SimpleNamespace(is_open=True, send=lambda payload: sent.append(payload))
        app.coord_tab = coord_tab
        app.root = types.SimpleNamespace(after=lambda *_args, **_kwargs: None)
        app._is_test_mode = lambda: False
        app._mission_uses_external_runtime = lambda mission=None: False
        app._control_timestamp_u32 = lambda: 123
        app._stop_external_runtime = lambda: None

        VirtualControllerApp._coord_send_tick(app)

        self.assertEqual(len(sent), 1)
        packet = sent[0]
        self.assertEqual(packet[1], TYPE_C0_CONTROL)
        ts_ms, left_cmd, right_cmd, duration_ms = struct.unpack_from("<IhhH", packet, 3)
        self.assertEqual(ts_ms, 123)
        self.assertEqual(left_cmd, 1600)
        self.assertEqual(right_cmd, 1400)
        self.assertEqual(duration_ms, 50)

    def test_build_telemetry_snapshot_includes_d3_sensor_tensor(self) -> None:
        sensor_tensor = parse_frame(
            Frame(
                msg_type=TYPE_D3_SENSOR_TENSOR,
                payload=struct.pack(
                    "<I12f",
                    77,
                    1.0, 2.0, 3.0,
                    4.0, 5.0, 6.0,
                    7.0, 8.0, 9.0,
                    10.0, 11.0, 12.0,
                ),
            )
        )

        app = object.__new__(VirtualControllerApp)
        app.time_model = TimeModel()
        app._last_mcu_ts_u32 = 77
        app._last_imu = None
        app._last_tacho = None
        app._last_motor = None
        app._last_d3 = sensor_tensor

        snapshot = VirtualControllerApp.build_telemetry_snapshot(app, now_ms=1234)

        self.assertIsNotNone(snapshot.sensor_tensor)
        assert snapshot.sensor_tensor is not None
        self.assertEqual(snapshot.sensor_tensor.ts_ms, 77)
        self.assertEqual(snapshot.sensor_tensor.linear_velocity, (1.0, 2.0, 3.0))
        self.assertEqual(snapshot.sensor_tensor.angular_velocity, (4.0, 5.0, 6.0))
        self.assertEqual(snapshot.sensor_tensor.linear_quality, (7.0, 8.0, 9.0))
        self.assertEqual(snapshot.sensor_tensor.angular_quality, (10.0, 11.0, 12.0))


if __name__ == "__main__":
    unittest.main()
