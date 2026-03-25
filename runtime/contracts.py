"""
Stable runtime contracts between GUI, telemetry, pose estimation, and the external runtime.
"""

from __future__ import annotations

from dataclasses import dataclass, fields, is_dataclass
from enum import Enum
from typing import Any, Optional, Protocol


class PoseSourceMode(str, Enum):
    INTERNAL = "internal"
    TROLLEY = "trolley"
    EXTERNAL = "external"


class AutopilotMode(str, Enum):
    BUILTIN_PURE_PURSUIT = "builtin_pure_pursuit"
    EXTERNAL = "external"


class TelemetryChannel(str, Enum):
    IMU = "imu"
    TACHO = "tacho"
    MOTOR = "motor"
    SENSOR_TENSOR = "sensor_tensor"


@dataclass(frozen=True)
class TrackPoint:
    x_m: float
    y_m: float


@dataclass(frozen=True)
class GeometrySettings:
    a1_m: float
    a2_m: float
    track_width_m: float
    track_circumference_m: float


@dataclass(frozen=True)
class MotionSettings:
    target_center_speed_m_s: float
    accel_m_s2: float
    decel_m_s2: float
    controller_dt_s: float
    minimal_turn_radius_m: Optional[float]


@dataclass(frozen=True)
class BuiltinAutopilotTuning:
    lookahead_min_m: float
    lookahead_max_m: float
    lookahead_gain: float
    curvature_slowdown: float


@dataclass(frozen=True)
class SpeedMapEntry:
    pwm: float
    speed_m_s: float


@dataclass(frozen=True)
class PwmCorrection:
    left_shift: float
    right_shift: float
    left_linear: float
    right_linear: float


@dataclass(frozen=True)
class TelemetrySubscription:
    channels: tuple[TelemetryChannel, ...] = (
        TelemetryChannel.IMU,
        TelemetryChannel.TACHO,
        TelemetryChannel.MOTOR,
    )
    imu_period_ms: Optional[int] = None
    tacho_period_ms: Optional[int] = None
    motor_period_ms: Optional[int] = None


@dataclass(frozen=True)
class MissionConfig:
    track_points: tuple[TrackPoint, ...]
    geometry: GeometrySettings
    motion: MotionSettings
    builtin_tuning: BuiltinAutopilotTuning
    speed_map: tuple[SpeedMapEntry, ...]
    pwm_correction: PwmCorrection
    pose_source: PoseSourceMode
    autopilot: AutopilotMode
    telemetry_subscription: TelemetrySubscription

    def to_dict(self) -> dict[str, Any]:
        return _serialize(self)


@dataclass(frozen=True)
class ImuTelemetry:
    ts_ms: int
    accel: tuple[float, float, float]
    magn: tuple[float, float, float]
    gyro: tuple[float, float, float]


@dataclass(frozen=True)
class TachoTelemetry:
    ts_ms: int
    left_rpm: int
    right_rpm: int


@dataclass(frozen=True)
class MotorTelemetry:
    ts_ms: int
    current_l: int
    current_r: int
    voltage_l: int
    voltage_r: int
    temp_l: int
    temp_r: int


@dataclass(frozen=True)
class SensorTensorTelemetry:
    ts_ms: int
    linear_velocity: tuple[float, float, float]
    angular_velocity: tuple[float, float, float]
    linear_quality: tuple[float, float, float]
    angular_quality: tuple[float, float, float]


@dataclass(frozen=True)
class TimeSyncState:
    pc_time_ms: int
    have_lock: bool
    scale_a: float
    offset_b: float
    mcu_rx_ms: Optional[int]
    mcu_est_ms: Optional[int]


@dataclass(frozen=True)
class TelemetrySnapshot:
    sync: TimeSyncState
    imu: Optional[ImuTelemetry] = None
    tacho: Optional[TachoTelemetry] = None
    motor: Optional[MotorTelemetry] = None
    sensor_tensor: Optional[SensorTensorTelemetry] = None

    def to_dict(self) -> dict[str, Any]:
        return _serialize(self)


@dataclass(frozen=True)
class PoseEstimate:
    x_m: float
    y_m: float
    theta_rad: float
    source: PoseSourceMode
    pc_time_ms: Optional[int] = None
    mcu_time_ms: Optional[int] = None

    def to_dict(self) -> dict[str, Any]:
        return _serialize(self)


@dataclass(frozen=True)
class DriveCommand:
    left_pwm: int
    right_pwm: int
    duration_ms: int
    source: str
    created_pc_ms: Optional[int] = None

    def to_dict(self) -> dict[str, Any]:
        return _serialize(self)


@dataclass(frozen=True)
class ExternalRuntimeState:
    pose: Optional[PoseEstimate] = None
    drive_command: Optional[DriveCommand] = None
    finished: bool = False

    def to_dict(self) -> dict[str, Any]:
        return _serialize(self)


class PoseEstimator(Protocol):
    def apply_mission(self, mission: MissionConfig) -> None:
        ...

    def reset(self) -> None:
        ...

    def update_telemetry(self, snapshot: TelemetrySnapshot) -> Optional[PoseEstimate]:
        ...


class AutopilotController(Protocol):
    def apply_mission(self, mission: MissionConfig) -> None:
        ...

    def reset(self) -> None:
        ...

    def update(
        self,
        snapshot: TelemetrySnapshot,
        pose: Optional[PoseEstimate],
        dt_s: float,
    ) -> Optional[DriveCommand]:
        ...

    def is_finished(self) -> bool:
        ...

    def stop(self) -> None:
        ...


class ExternalRuntimeBridge(Protocol):
    def apply_mission(self, mission: MissionConfig) -> None:
        ...

    def reset(self) -> None:
        ...

    def ingest_telemetry(self, snapshot: TelemetrySnapshot) -> None:
        ...

    def poll_state(self) -> ExternalRuntimeState:
        ...

    def stop(self) -> None:
        ...


def _serialize(value: Any) -> Any:
    if isinstance(value, Enum):
        return value.value
    if is_dataclass(value):
        return {field.name: _serialize(getattr(value, field.name)) for field in fields(value)}
    if isinstance(value, tuple):
        return [_serialize(item) for item in value]
    if isinstance(value, list):
        return [_serialize(item) for item in value]
    if isinstance(value, dict):
        return {str(key): _serialize(item) for key, item in value.items()}
    return value
