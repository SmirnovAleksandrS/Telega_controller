"""
Serial worker thread:
- opens COM port
- reads bytes
- parses frames
- pushes parsed events to a thread-safe queue
- provides send() for outgoing frames

We keep all UI interactions in main thread: worker only communicates via Queue.
"""

from __future__ import annotations
import threading
import queue
from dataclasses import dataclass
from typing import Optional, Any
import os

import serial
import serial.tools.list_ports

from utils.timebase import now_ms_monotonic, u32
from comm.protocol import StreamParser, parse_frame, Frame

@dataclass
class RxEvent:
    pc_rx_ms: int
    parsed: Any   # typed msg or Frame

@dataclass
class RxError:
    pc_rx_ms: int
    error: str

class SerialWorker:
    def __init__(self) -> None:
        self._ser: Optional[serial.Serial] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_evt: Optional[threading.Event] = None
        self._state_lock = threading.RLock()

        self.rx_queue: "queue.Queue[object]" = queue.Queue()
        self.tx_queue: "queue.Queue[bytes]" = queue.Queue()

        self.is_open = False
        self.port = ""
        self.baud = 115200
        self.on_send = None  # type: Optional[callable[[bytes], None]]
        self.tx_packets = 0
        self.tx_bytes = 0
        self.last_tx_ms: Optional[int] = None
        self.last_tx_error: str = ""

    @staticmethod
    def list_ports() -> list[str]:
        return [p.device for p in serial.tools.list_ports.comports()]

    def open(self, port: str, baud: int) -> None:
        self.close()
        with self._state_lock:
            if self._thread is not None and self._thread.is_alive():
                self.rx_queue.put(
                    RxError(
                        pc_rx_ms=now_ms_monotonic(),
                        error="Serial close is still in progress; refusing to reopen port",
                    )
                )
                self.is_open = False
                return
        self.port = port
        self.baud = baud

        stop_evt = threading.Event()
        open_kwargs: dict[str, Any] = {
            "port": port,
            "baudrate": baud,
            "timeout": 0.05,
            "write_timeout": 0.20,
            "rtscts": False,
            "dsrdtr": False,
            "xonxoff": False,
        }
        if os.name == "posix":
            open_kwargs["exclusive"] = True

        try:
            try:
                ser = serial.Serial(**open_kwargs)
            except TypeError:
                # Older pyserial may not support "exclusive"; retry without it.
                open_kwargs.pop("exclusive", None)
                ser = serial.Serial(**open_kwargs)
            try:
                ser.reset_input_buffer()
                ser.reset_output_buffer()
            except Exception:
                pass
        except Exception as e:
            self.rx_queue.put(RxError(pc_rx_ms=now_ms_monotonic(), error=f"Serial open failed: {e}"))
            with self._state_lock:
                if self._ser is None:
                    self._thread = None
                    self._stop_evt = None
                self.is_open = False
            return

        with self._state_lock:
            if self._thread is not None and self._thread.is_alive():
                try:
                    ser.close()
                except Exception:
                    pass
                self.rx_queue.put(
                    RxError(
                        pc_rx_ms=now_ms_monotonic(),
                        error="Serial worker is busy; try connect again in a moment",
                    )
                )
                self.is_open = False
                return
            self._ser = ser
            self._stop_evt = stop_evt
            self.is_open = True
            self._clear_tx_queue()
            self._thread = threading.Thread(
                target=self._run,
                args=(ser, stop_evt),
                daemon=True,
                name="serial-worker",
            )
            self._thread.start()

    def close(self) -> None:
        with self._state_lock:
            ser = self._ser
            th = self._thread
            stop_evt = self._stop_evt
            self.is_open = False

        if stop_evt is not None:
            stop_evt.set()
        if ser is not None:
            for method_name in ("cancel_read", "cancel_write"):
                try:
                    getattr(ser, method_name)()
                except Exception:
                    pass
            try:
                ser.close()
            except Exception:
                pass
        if th is not None and th.is_alive() and th is not threading.current_thread():
            th.join(timeout=2.0)
        if th is not None and th.is_alive():
            self.rx_queue.put(
                RxError(pc_rx_ms=now_ms_monotonic(), error="Serial worker did not stop cleanly")
            )
        with self._state_lock:
            if self._thread is th and (th is None or not th.is_alive()):
                self._ser = None
                self._thread = None
                self._stop_evt = None
        self._clear_tx_queue()

    def send(self, data: bytes) -> None:
        """Thread-safe send request."""
        if self.on_send:
            try:
                self.on_send(data)
            except Exception:
                pass
        self.tx_queue.put(data)

    def _clear_tx_queue(self) -> None:
        while True:
            try:
                self.tx_queue.get_nowait()
            except queue.Empty:
                return

    def _run(self, ser: serial.Serial, stop_evt: threading.Event) -> None:
        parser = StreamParser()
        while not stop_evt.is_set():
            try:
                # TX first (low latency control)
                try:
                    while True:
                        pkt = self.tx_queue.get_nowait()
                        n = ser.write(pkt)
                        self.tx_packets += 1
                        self.tx_bytes += n
                        self.last_tx_ms = now_ms_monotonic()
                        if n != len(pkt):
                            self.last_tx_error = f"short write {n}/{len(pkt)}"
                except queue.Empty:
                    pass

                # RX
                chunk = ser.read(4096)
                if chunk:
                    frames = parser.push(chunk)
                    pc_rx_ms = now_ms_monotonic()
                    for fr in frames:
                        parsed = parse_frame(fr)
                        self.rx_queue.put(RxEvent(pc_rx_ms=pc_rx_ms, parsed=parsed))

            except Exception as e:
                self.rx_queue.put(RxError(pc_rx_ms=now_ms_monotonic(), error=f"Serial worker error: {e}"))
                break

        try:
            ser.close()
        except Exception:
            pass
        with self._state_lock:
            if self._ser is ser:
                self._ser = None
                self._thread = None
                self._stop_evt = None
                self.is_open = False
