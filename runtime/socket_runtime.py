"""
Socket-based external runtime bridge.

The external process is expected to run standalone and speak newline-delimited JSON over TCP.
GUI publishes mission and telemetry events, while pose/control outputs arrive asynchronously.
"""

from __future__ import annotations

import json
import socket
import threading
from typing import Optional

from runtime.contracts import (
    DriveCommand,
    ExternalRuntimeState,
    MissionConfig,
    PoseEstimate,
    PoseSourceMode,
    TelemetrySnapshot,
)
from utils.speed_map import SpeedMapPoint, speed_to_pwm


class SocketExternalRuntime:
    def __init__(
        self,
        *,
        host: str = "127.0.0.1",
        port: int = 8765,
        connect_timeout_s: float = 2.0,
        io_timeout_s: float = 0.25,
    ) -> None:
        self.host = host
        self.port = int(port)
        self.connect_timeout_s = max(0.1, float(connect_timeout_s))
        self.io_timeout_s = max(0.05, float(io_timeout_s))
        self._sock: Optional[socket.socket] = None
        self._mission: Optional[MissionConfig] = None
        self._hello_sent = False
        self._finished = False
        self._latest_pose: Optional[PoseEstimate] = None
        self._latest_command: Optional[DriveCommand] = None
        self._reader_error: Optional[RuntimeError] = None
        self._reader_stop = threading.Event()
        self._reader_thread: Optional[threading.Thread] = None
        self._send_lock = threading.Lock()
        self._state_lock = threading.Lock()

    def apply_mission(self, mission: MissionConfig) -> None:
        self._mission = mission
        self._reset_state()
        self._ensure_connected()
        if not self._hello_sent:
            self._send_message(
                {
                    "type": "hello",
                    "protocol": "telega_cpp_runtime",
                    "version": 2,
                }
            )
            self._hello_sent = True
        self._send_message({"type": "mission", "mission": mission.to_dict()})

    def reset(self) -> None:
        self._reset_state()
        if self._sock is not None:
            self._send_message({"type": "reset"})

    def ingest_telemetry(self, snapshot: TelemetrySnapshot) -> None:
        if self._mission is None:
            raise RuntimeError("Socket external runtime has no mission")
        self._ensure_connected()
        self._raise_if_reader_failed()
        self._send_message(
            {
                "type": "telemetry_event",
                "trigger": "tacho",
                "telemetry": snapshot.to_dict(),
            }
        )

    def poll_state(self) -> ExternalRuntimeState:
        self._raise_if_reader_failed()
        with self._state_lock:
            return ExternalRuntimeState(
                pose=self._latest_pose,
                drive_command=self._latest_command,
                finished=self._finished,
            )

    def stop(self) -> None:
        if self._sock is not None:
            try:
                self._send_message({"type": "stop"})
            except Exception:
                pass
        self.close()

    def shutdown_server(self) -> None:
        if self._sock is not None:
            try:
                self._send_message({"type": "shutdown"})
            except Exception:
                pass
        self.close()

    def close(self) -> None:
        self._reader_stop.set()
        sock = self._sock
        self._sock = None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass
        thread = self._reader_thread
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        self._reader_thread = None
        self._hello_sent = False

    def _reset_state(self) -> None:
        with self._state_lock:
            self._finished = False
            self._latest_pose = None
            self._latest_command = None
        self._reader_error = None

    def _ensure_connected(self) -> None:
        if self._sock is not None:
            return
        self.close()
        sock = socket.create_connection((self.host, self.port), timeout=self.connect_timeout_s)
        sock.settimeout(self.io_timeout_s)
        self._sock = sock
        self._reader_stop.clear()
        self._reader_thread = threading.Thread(
            target=self._reader_loop,
            name="socket-external-runtime-reader",
            daemon=True,
        )
        self._reader_thread.start()

    def _send_message(self, payload: dict) -> None:
        sock = self._sock
        if sock is None:
            raise RuntimeError("Socket external runtime is not connected")
        raw = json.dumps(payload, ensure_ascii=False).encode("utf-8") + b"\n"
        with self._send_lock:
            sock.sendall(raw)

    def _reader_loop(self) -> None:
        buffer = b""
        while not self._reader_stop.is_set():
            sock = self._sock
            if sock is None:
                return
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                continue
            except OSError as exc:
                if self._reader_stop.is_set():
                    return
                self._reader_error = RuntimeError(f"Socket external runtime recv failed: {exc}")
                return
            if not chunk:
                if self._reader_stop.is_set():
                    return
                self._reader_error = RuntimeError("Socket external runtime connection closed")
                return
            buffer += chunk
            while b"\n" in buffer:
                raw_line, buffer = buffer.split(b"\n", 1)
                raw_line = raw_line.strip()
                if not raw_line:
                    continue
                try:
                    payload = json.loads(raw_line.decode("utf-8"))
                except json.JSONDecodeError as exc:
                    self._reader_error = RuntimeError(f"Socket external runtime returned invalid JSON: {exc}")
                    return
                if not isinstance(payload, dict):
                    self._reader_error = RuntimeError("Socket external runtime returned non-object JSON")
                    return
                self._handle_message(payload)

    def _handle_message(self, payload: dict) -> None:
        msg_type = str(payload.get("type", "")).strip()
        if msg_type == "pose_update":
            pose_payload = payload.get("pose", payload)
            if isinstance(pose_payload, dict):
                with self._state_lock:
                    self._latest_pose = self._decode_pose(pose_payload)
            return
        if msg_type == "control_command":
            command_payload = payload.get("command", payload)
            if isinstance(command_payload, dict):
                command = self._decode_command(command_payload)
                finished = bool(payload.get("finished", command_payload.get("finished", False)))
                with self._state_lock:
                    self._latest_command = command
                    self._finished = finished
            return
        if msg_type == "runtime_status":
            with self._state_lock:
                self._finished = bool(payload.get("finished", self._finished))
            return
        if msg_type in {"hello_ack", "ack", "log"}:
            return
        if msg_type == "error":
            self._reader_error = RuntimeError(str(payload.get("message", "External runtime returned error")))

    def _decode_pose(self, payload: dict) -> PoseEstimate:
        source_value = str(payload.get("source", PoseSourceMode.EXTERNAL.value)).strip()
        try:
            source = PoseSourceMode(source_value)
        except ValueError:
            source = PoseSourceMode.EXTERNAL
        return PoseEstimate(
            x_m=float(payload.get("x_m", 0.0)),
            y_m=float(payload.get("y_m", 0.0)),
            theta_rad=float(payload.get("theta_rad", 0.0)),
            source=source,
            pc_time_ms=self._optional_int(payload.get("pc_time_ms")),
            mcu_time_ms=self._optional_int(payload.get("mcu_time_ms")),
        )

    def _decode_command(self, payload: dict) -> DriveCommand:
        duration_ms = self._command_duration_ms(payload)
        source = str(payload.get("source", "cpp_socket_runtime"))
        if "left_pwm" in payload and "right_pwm" in payload:
            return DriveCommand(
                left_pwm=int(round(float(payload.get("left_pwm", 0.0)))),
                right_pwm=int(round(float(payload.get("right_pwm", 0.0)))),
                duration_ms=duration_ms,
                source=source,
            )
        left_speed = float(payload.get("left_speed_m_s", 0.0))
        right_speed = float(payload.get("right_speed_m_s", 0.0))
        return DriveCommand(
            left_pwm=self._speed_to_pwm(left_speed, is_left=True),
            right_pwm=self._speed_to_pwm(right_speed, is_left=False),
            duration_ms=duration_ms,
            source=source,
        )

    def _command_duration_ms(self, payload: dict) -> int:
        if "duration_ms" in payload:
            return max(1, int(round(float(payload.get("duration_ms", 0.0)))))
        if self._mission is not None:
            return max(1, int(round(self._mission.motion.controller_dt_s * 1000.0)))
        return 100

    def _speed_to_pwm(self, speed_m_s: float, *, is_left: bool) -> int:
        if self._mission is None:
            raise RuntimeError("Socket external runtime has no mission")
        points = [SpeedMapPoint(item.pwm, item.speed_m_s) for item in self._mission.speed_map]
        pwm = speed_to_pwm(speed_m_s, points)
        corr = self._mission.pwm_correction
        if is_left:
            pwm = (pwm + corr.left_shift) * corr.left_linear
        else:
            pwm = (pwm + corr.right_shift) * corr.right_linear
        return int(round(pwm))

    def _raise_if_reader_failed(self) -> None:
        if self._reader_error is not None:
            raise self._reader_error

    @staticmethod
    def _optional_int(value: object) -> Optional[int]:
        if value is None:
            return None
        try:
            return int(round(float(value)))
        except (TypeError, ValueError):
            return None
