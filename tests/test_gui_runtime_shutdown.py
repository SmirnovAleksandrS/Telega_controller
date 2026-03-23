from __future__ import annotations

import types
import unittest

from app.gui_app import VirtualControllerApp


class _DummyCoordTab:
    def __init__(self) -> None:
        self.running = None
        self.stop_expected_calls = 0

    def set_running(self, value: bool) -> None:
        self.running = value

    def stop_expected(self) -> None:
        self.stop_expected_calls += 1


class _DummyNotebook:
    def __init__(self, tab_text: str) -> None:
        self._tab_text = tab_text

    def select(self) -> str:
        return "current"

    def tab(self, _item: str, _option: str) -> str:
        return self._tab_text


class _DummyRuntime:
    def __init__(self) -> None:
        self.stop_calls = 0
        self.shutdown_calls = 0

    def stop(self) -> None:
        self.stop_calls += 1

    def shutdown_server(self) -> None:
        self.shutdown_calls += 1


class GuiRuntimeShutdownTests(unittest.TestCase):
    def test_shutdown_prefers_explicit_server_shutdown(self) -> None:
        app = object.__new__(VirtualControllerApp)
        runtime = _DummyRuntime()
        app._external_runtime = runtime
        app._last_external_command = object()

        VirtualControllerApp._shutdown_external_runtime(app)

        self.assertEqual(runtime.shutdown_calls, 1)
        self.assertEqual(runtime.stop_calls, 0)
        self.assertIsNone(app._last_external_command)

    def test_kill_switch_uses_runtime_stop_path(self) -> None:
        app = object.__new__(VirtualControllerApp)
        app._kill_active = False
        app._coord_running = True
        app._manual_neutral = 1500
        app._last_control = (1600, 1400)
        app.coord_tab = _DummyCoordTab()
        calls: list[str] = []
        app._update_kill_indicator = lambda: calls.append("indicator")
        app._stop_external_runtime = lambda: calls.append("stop")
        app._kill_send_tick = lambda: calls.append("tick")

        VirtualControllerApp._kill_switch(app)

        self.assertTrue(app._kill_active)
        self.assertFalse(app._coord_running)
        self.assertEqual(app._last_control, (1500, 1500))
        self.assertEqual(app.coord_tab.running, False)
        self.assertEqual(app.coord_tab.stop_expected_calls, 1)
        self.assertEqual(calls, ["indicator", "stop", "tick"])

    def test_tab_change_stops_runtime_when_leaving_coordinate_tab(self) -> None:
        app = object.__new__(VirtualControllerApp)
        app.nb = _DummyNotebook("Manual")
        app._coord_running = True
        app._kill_active = False
        app._manual_neutral = 1500
        app._control_duration_ms = 100
        app.worker = types.SimpleNamespace(is_open=False, send=lambda _payload: None)
        app.coord_tab = _DummyCoordTab()
        calls: list[str] = []
        app._stop_external_runtime = lambda: calls.append("stop")
        app._control_timestamp_u32 = lambda: 123

        VirtualControllerApp._on_tab_change(app, None)

        self.assertEqual(app._active_tab, "Manual")
        self.assertFalse(app._coord_running)
        self.assertEqual(app.coord_tab.running, False)
        self.assertEqual(app.coord_tab.stop_expected_calls, 1)
        self.assertEqual(calls, ["stop"])


if __name__ == "__main__":
    unittest.main()
