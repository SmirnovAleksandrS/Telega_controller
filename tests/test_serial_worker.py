from __future__ import annotations

import os
import pty
import struct
import time
import unittest
from unittest import mock

import serial

from comm.protocol import TYPE_D1_TACHO, build_control, build_frame
from comm.serial_worker import RxEvent, SerialWorker


def wait_until(predicate, timeout_s: float = 1.0, step_s: float = 0.02) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(step_s)
    return predicate()


@unittest.skipUnless(hasattr(pty, "openpty"), "pty support required")
class SerialWorkerTests(unittest.TestCase):
    def open_pty(self) -> tuple[int, int, str]:
        master_fd, slave_fd = pty.openpty()
        port = os.ttyname(slave_fd)
        self.addCleanup(os.close, master_fd)
        self.addCleanup(os.close, slave_fd)
        return master_fd, slave_fd, port

    def test_open_close_reopen_on_pty(self) -> None:
        _master_fd, _slave_fd, port = self.open_pty()
        worker = SerialWorker()
        self.addCleanup(worker.close)

        worker.open(port, 115200)
        self.assertTrue(worker.is_open)
        self.assertEqual(worker.state, "open")

        worker.close()
        self.assertFalse(worker.is_open)
        self.assertEqual(worker.state, "closed")

        worker.open(port, 115200)
        self.assertTrue(worker.is_open)
        self.assertEqual(worker.open_count, 2)

        worker.close()
        metrics = worker.get_metrics()
        self.assertEqual(metrics.state, "closed")
        self.assertEqual(metrics.close_count, 2)

    def test_send_while_closed_is_dropped(self) -> None:
        worker = SerialWorker()
        self.assertFalse(worker.send(b"abc"))
        metrics = worker.get_metrics()
        self.assertEqual(metrics.tx_dropped, 1)
        self.assertEqual(metrics.last_tx_error, "send dropped while UART is not open")

    def test_open_retries_without_exclusive(self) -> None:
        calls: list[dict[str, object]] = []

        class FakeSerial:
            def __init__(self) -> None:
                self.closed = False

            def reset_input_buffer(self) -> None:
                return None

            def reset_output_buffer(self) -> None:
                return None

            def cancel_read(self) -> None:
                return None

            def cancel_write(self) -> None:
                return None

            def write(self, data: bytes) -> int:
                return len(data)

            def read(self, _size: int) -> bytes:
                time.sleep(0.01)
                return b""

            def close(self) -> None:
                self.closed = True

        def fake_serial_factory(**kwargs):
            calls.append(dict(kwargs))
            if kwargs.get("exclusive"):
                raise serial.SerialException("exclusive unsupported")
            return FakeSerial()

        worker = SerialWorker(prefer_exclusive=True)
        self.addCleanup(worker.close)

        with mock.patch("comm.serial_worker.serial.Serial", side_effect=fake_serial_factory):
            worker.open("/dev/ttyFAKE0", 115200)
            self.assertTrue(worker.is_open)
            self.assertEqual(len(calls), 2)
            self.assertTrue(calls[0].get("exclusive"))
            self.assertNotIn("exclusive", calls[1])

    def test_metrics_capture_rx_tx_activity(self) -> None:
        master_fd, _slave_fd, port = self.open_pty()
        worker = SerialWorker(rate_window_ms=500)
        self.addCleanup(worker.close)

        worker.open(port, 115200)
        self.assertTrue(worker.is_open)

        tx_packet = build_control(1234, 1500, 1500, 50)
        self.assertTrue(worker.send(tx_packet))

        rx_payload = struct.pack("<Iii", 42, 120, 130)
        rx_packet = build_frame(TYPE_D1_TACHO, rx_payload)
        os.write(master_fd, rx_packet)

        self.assertTrue(wait_until(lambda: worker.get_metrics().rx_packets >= 1))
        metrics = worker.get_metrics()

        self.assertGreaterEqual(metrics.tx_packets, 1)
        self.assertGreaterEqual(metrics.tx_bytes, len(tx_packet))
        self.assertGreaterEqual(metrics.rx_packets, 1)
        self.assertGreaterEqual(metrics.rx_bytes, len(rx_packet))
        self.assertGreater(metrics.tx_rate_Bps + metrics.rx_rate_Bps, 0.0)
        self.assertGreater(metrics.peak_load_pct, 0.0)

        received = []
        while True:
            try:
                received.append(worker.rx_queue.get_nowait())
            except Exception:
                break
        self.assertTrue(any(isinstance(item, RxEvent) for item in received))


if __name__ == "__main__":
    unittest.main()
