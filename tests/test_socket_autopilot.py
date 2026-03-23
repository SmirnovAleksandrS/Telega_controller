from __future__ import annotations

import json
import socketserver
import threading
import time
import unittest

from runtime.contracts import (
    AutopilotMode,
    BuiltinAutopilotTuning,
    ExternalRuntimeState,
    GeometrySettings,
    MissionConfig,
    MotionSettings,
    PoseSourceMode,
    PwmCorrection,
    SpeedMapEntry,
    TelemetrySnapshot,
    TelemetrySubscription,
    TimeSyncState,
    TrackPoint,
)
from runtime.socket_autopilot import SocketAutopilotClient


def wait_until(predicate, timeout_s: float = 1.0, step_s: float = 0.01) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(step_s)
    return predicate()


class _AutopilotStubServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True

    def __init__(self, server_address: tuple[str, int]) -> None:
        super().__init__(server_address, _AutopilotStubHandler)
        self.received: list[dict] = []


class _AutopilotStubHandler(socketserver.StreamRequestHandler):
    def handle(self) -> None:
        server = self.server
        if not isinstance(server, _AutopilotStubServer):
            return
        while True:
            raw = self.rfile.readline()
            if not raw:
                return
            payload = json.loads(raw.decode("utf-8"))
            server.received.append(payload)
            if payload.get("type") != "telemetry_event":
                continue
            pc_time_ms = payload.get("telemetry", {}).get("sync", {}).get("pc_time_ms")
            pose_reply = {
                "type": "pose_update",
                "pose": {
                    "x_m": 1.25,
                    "y_m": -0.5,
                    "theta_rad": 0.75,
                    "source": "external",
                    "pc_time_ms": pc_time_ms,
                    "mcu_time_ms": 777,
                },
                "source": "test_stub_logger",
            }
            command_reply = {
                "type": "control_command",
                "left_speed_m_s": 0.0,
                "right_speed_m_s": 0.0,
                "duration_ms": 50,
                "finished": False,
                "source": "test_stub_controller",
            }
            self.wfile.write(json.dumps(pose_reply).encode("utf-8") + b"\n")
            self.wfile.write(json.dumps(command_reply).encode("utf-8") + b"\n")
            self.wfile.flush()


class SocketAutopilotTests(unittest.TestCase):
    def setUp(self) -> None:
        self.server = _AutopilotStubServer(("127.0.0.1", 0))
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def tearDown(self) -> None:
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=1.0)

    def _build_mission(self) -> MissionConfig:
        return MissionConfig(
            track_points=(TrackPoint(0.0, 0.0), TrackPoint(1.0, 0.0)),
            geometry=GeometrySettings(
                a1_m=0.2,
                a2_m=0.2,
                track_width_m=0.4,
                track_circumference_m=1.0,
            ),
            motion=MotionSettings(
                target_center_speed_m_s=0.5,
                accel_m_s2=0.5,
                decel_m_s2=0.5,
                controller_dt_s=0.05,
                minimal_turn_radius_m=0.3,
            ),
            builtin_tuning=BuiltinAutopilotTuning(
                lookahead_min_m=0.2,
                lookahead_max_m=1.0,
                lookahead_gain=0.5,
                curvature_slowdown=1.2,
            ),
            speed_map=(
                SpeedMapEntry(pwm=1000.0, speed_m_s=-1.0),
                SpeedMapEntry(pwm=1500.0, speed_m_s=0.0),
                SpeedMapEntry(pwm=2000.0, speed_m_s=1.0),
            ),
            pwm_correction=PwmCorrection(
                left_shift=0.0,
                right_shift=0.0,
                left_linear=1.0,
                right_linear=1.0,
            ),
            pose_source=PoseSourceMode.TROLLEY,
            autopilot=AutopilotMode.EXTERNAL,
            telemetry_subscription=TelemetrySubscription(),
        )

    def test_client_sends_runtime_events_and_receives_pose_and_zero_speed_command(self) -> None:
        client = SocketAutopilotClient(port=self.server.server_address[1])
        mission = self._build_mission()
        snapshot = TelemetrySnapshot(
            sync=TimeSyncState(
                pc_time_ms=1234,
                have_lock=False,
                scale_a=1.0,
                offset_b=0.0,
                mcu_rx_ms=None,
                mcu_est_ms=None,
            )
        )

        client.apply_mission(mission)
        client.reset()
        client.ingest_telemetry(snapshot)
        state = ExternalRuntimeState()
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            state = client.poll_state()
            if state.pose is not None and state.drive_command is not None:
                break
            time.sleep(0.01)
        client.stop()
        self.assertTrue(wait_until(lambda: any(item.get("type") == "stop" for item in self.server.received)))

        self.assertIsNotNone(state.pose)
        self.assertIsNotNone(state.drive_command)
        assert state.pose is not None
        assert state.drive_command is not None
        self.assertEqual(state.pose.x_m, 1.25)
        self.assertEqual(state.pose.y_m, -0.5)
        self.assertEqual(state.pose.theta_rad, 0.75)
        self.assertEqual(state.pose.source, PoseSourceMode.EXTERNAL)
        self.assertEqual(state.pose.pc_time_ms, 1234)
        self.assertEqual(state.pose.mcu_time_ms, 777)
        self.assertEqual(state.drive_command.left_pwm, 1500)
        self.assertEqual(state.drive_command.right_pwm, 1500)
        self.assertEqual(state.drive_command.duration_ms, 50)
        self.assertFalse(state.finished)

        msg_types = [item.get("type") for item in self.server.received]
        self.assertEqual(msg_types[:4], ["hello", "mission", "reset", "telemetry_event"])
        self.assertEqual(self.server.received[1]["mission"]["autopilot"], "external")
        self.assertEqual(self.server.received[3]["telemetry"]["sync"]["pc_time_ms"], 1234)

    def test_client_can_request_server_shutdown(self) -> None:
        client = SocketAutopilotClient(port=self.server.server_address[1])

        client.apply_mission(self._build_mission())
        client.shutdown_server()

        self.assertTrue(wait_until(lambda: any(item.get("type") == "shutdown" for item in self.server.received)))


if __name__ == "__main__":
    unittest.main()
