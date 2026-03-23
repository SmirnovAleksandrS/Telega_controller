from __future__ import annotations

import json
import os
import pathlib
import socket
import subprocess
import time
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
CPP_STUB_BINARY = REPO_ROOT / "cpp_autopilot" / "build" / "telega_cpp_autopilot"


def _free_tcp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def _wait_until(predicate, timeout_s: float = 2.0, step_s: float = 0.02) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(step_s)
    return predicate()


class CppRuntimeStubTests(unittest.TestCase):
    @unittest.skipUnless(CPP_STUB_BINARY.is_file(), "C++ runtime stub binary is not built")
    def test_stop_closes_session_and_shutdown_releases_port(self) -> None:
        port = _free_tcp_port()
        proc = subprocess.Popen(
            [os.fspath(CPP_STUB_BINARY), "--host", "127.0.0.1", "--port", str(port)],
            cwd=REPO_ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        self.addCleanup(self._cleanup_process, proc)

        self.assertTrue(self._wait_for_listen(port, proc))

        with socket.create_connection(("127.0.0.1", port), timeout=1.0) as sock:
            sock.settimeout(1.0)
            sock.sendall(json.dumps({"type": "stop"}).encode("utf-8") + b"\n")
            self.assertEqual(sock.recv(1), b"")

        with socket.create_connection(("127.0.0.1", port), timeout=1.0) as sock:
            sock.sendall(json.dumps({"type": "shutdown"}).encode("utf-8") + b"\n")

        self.assertTrue(_wait_until(lambda: proc.poll() is not None, timeout_s=2.0))
        self.assertEqual(proc.returncode, 0)

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("127.0.0.1", port))

    def _wait_for_listen(self, port: int, proc: subprocess.Popen[str]) -> bool:
        def can_connect() -> bool:
            if proc.poll() is not None:
                return False
            try:
                with socket.create_connection(("127.0.0.1", port), timeout=0.1):
                    return True
            except OSError:
                return False

        return _wait_until(can_connect, timeout_s=2.0)

    def _cleanup_process(self, proc: subprocess.Popen[str]) -> None:
        try:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait(timeout=2.0)
        finally:
            if proc.stdout is not None:
                proc.stdout.close()


if __name__ == "__main__":
    unittest.main()
