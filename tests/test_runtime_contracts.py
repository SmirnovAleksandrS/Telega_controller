from __future__ import annotations

import unittest

from comm.time_sync import TimeModel, control_timestamp_u32
from runtime.contracts import (
    AutopilotMode,
    BuiltinAutopilotTuning,
    GeometrySettings,
    MissionConfig,
    MotionSettings,
    PoseSourceMode,
    PwmCorrection,
    SpeedMapEntry,
    TelemetrySubscription,
    TrackPoint,
)


class ControlTimestampTests(unittest.TestCase):
    def test_without_sync_lock_uses_pc_time(self) -> None:
        model = TimeModel()
        self.assertEqual(control_timestamp_u32(model, 123456), 123456)

    def test_with_sync_lock_uses_mcu_model(self) -> None:
        model = TimeModel(a=1.0, b=250.0, have_lock=True)
        self.assertEqual(control_timestamp_u32(model, 1000), 1250)

    def test_result_is_wrapped_to_u32(self) -> None:
        model = TimeModel()
        self.assertEqual(control_timestamp_u32(model, (1 << 32) + 7), 7)


class RuntimeContractsTests(unittest.TestCase):
    def test_mission_to_dict_serializes_enums(self) -> None:
        mission = MissionConfig(
            track_points=(TrackPoint(0.0, 0.0), TrackPoint(1.0, 2.0)),
            geometry=GeometrySettings(
                a1_m=0.2,
                a2_m=0.2,
                track_width_m=0.4,
                track_circumference_m=1.0,
            ),
            motion=MotionSettings(
                target_center_speed_m_s=1.0,
                accel_m_s2=0.5,
                decel_m_s2=0.5,
                controller_dt_s=0.1,
                minimal_turn_radius_m=0.3,
            ),
            builtin_tuning=BuiltinAutopilotTuning(
                lookahead_min_m=0.25,
                lookahead_max_m=1.5,
                lookahead_gain=0.6,
                curvature_slowdown=1.5,
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
            pose_source=PoseSourceMode.EXTERNAL,
            autopilot=AutopilotMode.EXTERNAL,
            telemetry_subscription=TelemetrySubscription(),
        )

        data = mission.to_dict()

        self.assertEqual(data["pose_source"], "external")
        self.assertEqual(data["autopilot"], "external")
        self.assertEqual(data["telemetry_subscription"]["channels"], ["imu", "tacho", "motor"])


if __name__ == "__main__":
    unittest.main()
