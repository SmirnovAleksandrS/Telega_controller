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
        self._stop = threading.Event()
        self._parser = StreamParser()

        self.rx_queue: "queue.Queue[object]" = queue.Queue()
        self.tx_queue: "queue.Queue[bytes]" = queue.Queue()

        self.is_open = False
        self.port = ""
        self.baud = 115200

    @staticmethod
    def list_ports() -> list[str]:
        return [p.device for p in serial.tools.list_ports.comports()]

    def open(self, port: str, baud: int) -> None:
        self.close()
        self.port = port
        self.baud = baud
        self._stop.clear()

        try:
            self._ser = serial.Serial(port=port, baudrate=baud, timeout=0.05)
        except Exception as e:
            self.rx_queue.put(RxError(pc_rx_ms=now_ms_monotonic(), error=f"Serial open failed: {e}"))
            self._ser = None
            self.is_open = False
            return

        self.is_open = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        self.is_open = False
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
        self._ser = None

    def send(self, data: bytes) -> None:
        """Thread-safe send request."""
        self.tx_queue.put(data)

    def _run(self) -> None:
        assert self._ser is not None
        while not self._stop.is_set():
            try:
                # TX first (low latency control)
                try:
                    while True:
                        pkt = self.tx_queue.get_nowait()
                        self._ser.write(pkt)
                except queue.Empty:
                    pass

                # RX
                chunk = self._ser.read(4096)
                if chunk:
                    frames = self._parser.push(chunk)
                    pc_rx_ms = now_ms_monotonic()
                    for fr in frames:
                        parsed = parse_frame(fr)
                        self.rx_queue.put(RxEvent(pc_rx_ms=pc_rx_ms, parsed=parsed))

            except Exception as e:
                self.rx_queue.put(RxError(pc_rx_ms=now_ms_monotonic(), error=f"Serial worker error: {e}"))
                break

        self.is_open = False
