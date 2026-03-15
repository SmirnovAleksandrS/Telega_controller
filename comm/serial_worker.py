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
import collections
import os
import queue
import threading
from dataclasses import dataclass
from typing import Any, Callable, Optional

import serial
import serial.tools.list_ports

from utils.timebase import now_ms_monotonic
from comm.protocol import StreamParser, parse_frame, Frame


UART_BITS_PER_BYTE = 10.0
DEFAULT_RX_QUEUE_MAXSIZE = 4096
DEFAULT_TX_QUEUE_MAXSIZE = 512
DEFAULT_RATE_WINDOW_MS = 1000
DEFAULT_CLOSE_TIMEOUT_S = 2.0


@dataclass
class RxEvent:
    pc_rx_ms: int
    parsed: Any   # typed msg or Frame


@dataclass
class RxError:
    pc_rx_ms: int
    error: str


@dataclass(frozen=True)
class SerialMetrics:
    state: str
    is_open: bool
    worker_alive: bool
    port: str
    baud: int
    session_id: int
    open_count: int
    close_count: int
    error_count: int
    tx_packets: int
    tx_bytes: int
    rx_packets: int
    rx_bytes: int
    tx_queue_depth: int
    rx_queue_depth: int
    tx_dropped: int
    rx_dropped: int
    tx_rate_Bps: float
    rx_rate_Bps: float
    tx_load_pct: float
    rx_load_pct: float
    peak_load_pct: float
    combined_load_pct: float
    last_open_ms: Optional[int]
    last_close_ms: Optional[int]
    last_tx_ms: Optional[int]
    last_rx_ms: Optional[int]
    last_error: str
    last_tx_error: str


class SerialWorker:
    def __init__(
        self,
        *,
        rx_queue_maxsize: int = DEFAULT_RX_QUEUE_MAXSIZE,
        tx_queue_maxsize: int = DEFAULT_TX_QUEUE_MAXSIZE,
        rate_window_ms: int = DEFAULT_RATE_WINDOW_MS,
        close_timeout_s: float = DEFAULT_CLOSE_TIMEOUT_S,
        prefer_exclusive: Optional[bool] = None,
    ) -> None:
        self._ser: Optional[serial.Serial] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_evt: Optional[threading.Event] = None
        self._state_lock = threading.RLock()
        self._state = "closed"
        self._run_id = 0
        self._session_id = 0
        self._rate_window_ms = max(200, int(rate_window_ms))
        self._close_timeout_s = max(0.1, float(close_timeout_s))
        self._prefer_exclusive = os.name == "posix" if prefer_exclusive is None else bool(prefer_exclusive)
        self._io_samples: collections.deque[tuple[int, int, int]] = collections.deque()

        self.rx_queue: "queue.Queue[object]" = queue.Queue(maxsize=max(1, int(rx_queue_maxsize)))
        self.tx_queue: "queue.Queue[bytes]" = queue.Queue(maxsize=max(1, int(tx_queue_maxsize)))

        self.is_open = False
        self.port = ""
        self.baud = 115200
        self.on_send: Optional[Callable[[bytes], None]] = None
        self.open_count = 0
        self.close_count = 0
        self.error_count = 0
        self.tx_packets = 0
        self.tx_bytes = 0
        self.rx_packets = 0
        self.rx_bytes = 0
        self.tx_dropped = 0
        self.rx_dropped = 0
        self.last_open_ms: Optional[int] = None
        self.last_close_ms: Optional[int] = None
        self.last_tx_ms: Optional[int] = None
        self.last_rx_ms: Optional[int] = None
        self.last_error: str = ""
        self.last_tx_error: str = ""
        self._record_io_sample_locked(now_ms_monotonic())

    @staticmethod
    def list_ports() -> list[str]:
        return sorted({p.device for p in serial.tools.list_ports.comports()})

    @property
    def state(self) -> str:
        with self._state_lock:
            return self._state

    def get_metrics(self) -> SerialMetrics:
        with self._state_lock:
            now_ms = now_ms_monotonic()
            self._record_io_sample_locked(now_ms)
            tx_rate_Bps, rx_rate_Bps = self._compute_rates_locked()
            baud = max(1, int(self.baud))
            tx_load_pct = (tx_rate_Bps * UART_BITS_PER_BYTE * 100.0) / float(baud)
            rx_load_pct = (rx_rate_Bps * UART_BITS_PER_BYTE * 100.0) / float(baud)
            worker_alive = self._thread is not None and self._thread.is_alive()
            return SerialMetrics(
                state=self._state,
                is_open=self.is_open,
                worker_alive=worker_alive,
                port=self.port,
                baud=self.baud,
                session_id=self._session_id,
                open_count=self.open_count,
                close_count=self.close_count,
                error_count=self.error_count,
                tx_packets=self.tx_packets,
                tx_bytes=self.tx_bytes,
                rx_packets=self.rx_packets,
                rx_bytes=self.rx_bytes,
                tx_queue_depth=self._queue_depth(self.tx_queue),
                rx_queue_depth=self._queue_depth(self.rx_queue),
                tx_dropped=self.tx_dropped,
                rx_dropped=self.rx_dropped,
                tx_rate_Bps=tx_rate_Bps,
                rx_rate_Bps=rx_rate_Bps,
                tx_load_pct=tx_load_pct,
                rx_load_pct=rx_load_pct,
                peak_load_pct=max(tx_load_pct, rx_load_pct),
                combined_load_pct=tx_load_pct + rx_load_pct,
                last_open_ms=self.last_open_ms,
                last_close_ms=self.last_close_ms,
                last_tx_ms=self.last_tx_ms,
                last_rx_ms=self.last_rx_ms,
                last_error=self.last_error,
                last_tx_error=self.last_tx_error,
            )

    def open(self, port: str, baud: int) -> None:
        self.close()
        port = str(port).strip()
        try:
            baud = int(baud)
        except (TypeError, ValueError):
            self._publish_error(f"Serial open failed: invalid baudrate {baud!r}")
            return
        if not port:
            self._publish_error("Serial open failed: empty port")
            return

        with self._state_lock:
            self.port = port
            self.baud = baud
            self._state = "opening"
            self.last_error = ""
            self.last_tx_error = ""

        stop_evt = threading.Event()
        try:
            ser = self._open_serial(port, baud)
            try:
                ser.reset_input_buffer()
                ser.reset_output_buffer()
            except Exception:
                pass
        except Exception as exc:
            self._publish_error(f"Serial open failed: {exc}")
            return

        now_ms = now_ms_monotonic()
        self._clear_rx_queue()
        self._clear_tx_queue()

        with self._state_lock:
            self._run_id += 1
            run_id = self._run_id
            worker = threading.Thread(
                target=self._run,
                args=(ser, stop_evt, run_id),
                daemon=True,
                name=f"serial-worker-{run_id}",
            )
            self._ser = ser
            self._thread = worker
            self._stop_evt = stop_evt
            self.is_open = True
            self._state = "open"
            self._session_id += 1
            self.open_count += 1
            self.last_open_ms = now_ms
            self.last_error = ""
            self.last_tx_error = ""
            self._record_io_sample_locked(now_ms)

        try:
            worker.start()
        except Exception as exc:
            try:
                ser.close()
            except Exception:
                pass
            with self._state_lock:
                self._ser = None
                self._thread = None
                self._stop_evt = None
                self.is_open = False
                self._state = "fault"
            self._publish_error(f"Serial worker failed to start: {exc}")

    def close(self) -> None:
        now_ms = now_ms_monotonic()
        with self._state_lock:
            ser = self._ser
            th = self._thread
            stop_evt = self._stop_evt
            had_session = ser is not None or th is not None or self.is_open
            if had_session:
                self._run_id += 1
                self._ser = None
                self._thread = None
                self._stop_evt = None
                self.is_open = False
                self._state = "closing"
                self.close_count += 1
                self.last_close_ms = now_ms
                self._record_io_sample_locked(now_ms)
            elif self._state != "fault":
                self.is_open = False
                self._state = "closed"

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

        timed_out = False
        if th is not None and th.is_alive() and th is not threading.current_thread():
            th.join(timeout=self._close_timeout_s)
            timed_out = th.is_alive()
        elif th is not None and th.is_alive():
            timed_out = True

        if timed_out:
            self._publish_error("Serial worker did not stop cleanly before timeout")
        with self._state_lock:
            if self._state == "closing":
                self._state = "closed"
                self.last_error = ""
        self._clear_tx_queue()

    def send(self, data: bytes) -> bool:
        """Thread-safe send request."""
        if not data:
            return False
        with self._state_lock:
            can_send = self._state == "open" and self._ser is not None and self._stop_evt is not None
        if not can_send:
            with self._state_lock:
                self.tx_dropped += 1
                self.last_tx_error = "send dropped while UART is not open"
            return False
        if not self._enqueue_tx(data):
            return False
        if self.on_send:
            try:
                self.on_send(data)
            except Exception:
                pass
        return True

    def _clear_tx_queue(self) -> None:
        while True:
            try:
                self.tx_queue.get_nowait()
            except queue.Empty:
                return

    def _clear_rx_queue(self) -> None:
        while True:
            try:
                self.rx_queue.get_nowait()
            except queue.Empty:
                return

    def _open_serial(self, port: str, baud: int) -> serial.Serial:
        open_kwargs: dict[str, Any] = {
            "port": port,
            "baudrate": baud,
            "timeout": 0.05,
            "write_timeout": 0.20,
            "rtscts": False,
            "dsrdtr": False,
            "xonxoff": False,
        }
        if self._prefer_exclusive:
            open_kwargs["exclusive"] = True

        try:
            return serial.Serial(**open_kwargs)
        except TypeError:
            open_kwargs.pop("exclusive", None)
            return serial.Serial(**open_kwargs)
        except Exception:
            if open_kwargs.pop("exclusive", None) is not None:
                return serial.Serial(**open_kwargs)
            raise

    def _publish_error(self, error: str, *, run_id: Optional[int] = None) -> None:
        now_ms = now_ms_monotonic()
        with self._state_lock:
            if run_id is not None and run_id != self._run_id:
                return
            self.error_count += 1
            self.last_error = error
            self.is_open = False
            self._state = "fault"
            self._record_io_sample_locked(now_ms)
        self._enqueue_rx(RxError(pc_rx_ms=now_ms, error=error), run_id=run_id)

    def _enqueue_tx(self, data: bytes) -> bool:
        while True:
            try:
                self.tx_queue.put_nowait(data)
                return True
            except queue.Full:
                try:
                    self.tx_queue.get_nowait()
                except queue.Empty:
                    continue
                with self._state_lock:
                    self.tx_dropped += 1
                    self.last_tx_error = "tx queue overflow: dropped oldest packet"

    def _enqueue_rx(self, item: object, *, run_id: Optional[int] = None) -> None:
        while True:
            with self._state_lock:
                if run_id is not None and run_id != self._run_id:
                    return
            try:
                self.rx_queue.put_nowait(item)
                return
            except queue.Full:
                try:
                    self.rx_queue.get_nowait()
                except queue.Empty:
                    continue
                with self._state_lock:
                    self.rx_dropped += 1

    def _record_io_sample_locked(self, now_ms: int) -> None:
        sample = (now_ms, self.rx_bytes, self.tx_bytes)
        if self._io_samples and self._io_samples[-1][0] == now_ms:
            self._io_samples[-1] = sample
        else:
            self._io_samples.append(sample)
        cutoff = now_ms - self._rate_window_ms
        while len(self._io_samples) > 2 and self._io_samples[1][0] <= cutoff:
            self._io_samples.popleft()

    def _compute_rates_locked(self) -> tuple[float, float]:
        if len(self._io_samples) < 2:
            return (0.0, 0.0)
        first_ms, first_rx, first_tx = self._io_samples[0]
        last_ms, last_rx, last_tx = self._io_samples[-1]
        dt_ms = max(1, last_ms - first_ms)
        tx_rate = max(0, last_tx - first_tx) * 1000.0 / float(dt_ms)
        rx_rate = max(0, last_rx - first_rx) * 1000.0 / float(dt_ms)
        return (tx_rate, rx_rate)

    @staticmethod
    def _queue_depth(q: queue.Queue[object]) -> int:
        try:
            return q.qsize()
        except NotImplementedError:
            return 0

    def _write_packet(self, ser: serial.Serial, pkt: bytes, stop_evt: threading.Event) -> int:
        total = 0
        while total < len(pkt) and not stop_evt.is_set():
            written = ser.write(pkt[total:])
            if not written:
                raise serial.SerialTimeoutException(
                    f"write timeout after {total}/{len(pkt)} bytes"
                )
            total += written
        if total != len(pkt):
            raise serial.SerialTimeoutException(
                f"write interrupted after {total}/{len(pkt)} bytes"
            )
        return total

    def _run(self, ser: serial.Serial, stop_evt: threading.Event, run_id: int) -> None:
        parser = StreamParser()
        try:
            while not stop_evt.is_set():
                with self._state_lock:
                    if run_id != self._run_id:
                        return
                try:
                    while True:
                        with self._state_lock:
                            if run_id != self._run_id:
                                return
                        pkt = self.tx_queue.get_nowait()
                        written = self._write_packet(ser, pkt, stop_evt)
                        now_ms = now_ms_monotonic()
                        with self._state_lock:
                            if run_id != self._run_id:
                                return
                            self.tx_packets += 1
                            self.tx_bytes += written
                            self.last_tx_ms = now_ms
                            self.last_tx_error = ""
                            self._record_io_sample_locked(now_ms)
                except queue.Empty:
                    pass

                chunk = ser.read(4096)
                if not chunk:
                    continue

                frames = parser.push(chunk)
                now_ms = now_ms_monotonic()
                with self._state_lock:
                    if run_id != self._run_id:
                        return
                    self.rx_bytes += len(chunk)
                    self.rx_packets += len(frames)
                    self.last_rx_ms = now_ms
                    self._record_io_sample_locked(now_ms)

                for frame in frames:
                    parsed = parse_frame(frame)
                    self._enqueue_rx(RxEvent(pc_rx_ms=now_ms, parsed=parsed), run_id=run_id)

        except Exception as exc:
            if not stop_evt.is_set():
                self._publish_error(f"Serial worker error: {exc}", run_id=run_id)
        finally:
            try:
                ser.close()
            except Exception:
                pass
            now_ms = now_ms_monotonic()
            with self._state_lock:
                if run_id == self._run_id:
                    self._ser = None
                    self._thread = None
                    self._stop_evt = None
                    self.is_open = False
                    if self._state == "open":
                        self._state = "closed" if stop_evt.is_set() else "fault"
                    self.last_close_ms = now_ms
                    self._record_io_sample_locked(now_ms)
