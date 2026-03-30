from __future__ import annotations

import unittest

from app.coordinate_tab import CoordinateTab


class _DummyVar:
    def __init__(self, value: str) -> None:
        self._value = value

    def get(self) -> str:
        return self._value

    def set(self, value: str) -> None:
        self._value = value


class _DummyNumericVar:
    def __init__(self, value: float) -> None:
        self._value = value

    def get(self) -> float:
        return self._value

    def set(self, value: float) -> None:
        self._value = value


class _DummyCanvas:
    def __init__(self, width: int, height: int) -> None:
        self._width = width
        self._height = height

    def winfo_width(self) -> int:
        return self._width

    def winfo_height(self) -> int:
        return self._height

    def cget(self, key: str) -> str:
        if key == "width":
            return str(self._width)
        if key == "height":
            return str(self._height)
        raise KeyError(key)


class CoordinateTabViewTests(unittest.TestCase):
    def _make_tab(self) -> CoordinateTab:
        tab = object.__new__(CoordinateTab)
        tab._canvas_size = 420
        tab._grid_cells = 14
        tab._view_scale = 1.0
        tab._view_pan = (12.5, -7.5)
        tab._magnifier_zoom = 4.0
        tab._scale_var = _DummyVar("1 m")
        tab._map_image_scale_var = _DummyNumericVar(1.0)
        tab._map_rotation_var = _DummyNumericVar(0.0)
        tab._map_image_scale_text_var = _DummyVar("1.00")
        tab._map_rotation_text_var = _DummyVar("0.0")
        tab._map_image = None
        tab.canvas = _DummyCanvas(420, 420)
        tab.magnifier_canvas = _DummyCanvas(210, 210)
        tab._view_canvas_zoom = {
            tab.canvas: 1.0,
            tab.magnifier_canvas: tab._magnifier_zoom,
        }
        return tab

    def test_scale_m_accepts_extended_meter_ranges(self) -> None:
        tab = self._make_tab()

        for label, expected in [
            ("5 m", 5.0),
            ("10 m", 10.0),
            ("15 m", 15.0),
            ("25 m", 25.0),
            ("50 m", 50.0),
        ]:
            tab._scale_var.set(label)
            self.assertEqual(tab._scale_m(), expected)

    def test_magnifier_size_is_half_of_main_canvas(self) -> None:
        tab = self._make_tab()

        self.assertEqual(tab._desired_magnifier_size(900.0, 700.0), (450, 350))

    def test_magnifier_visible_span_is_four_times_tighter(self) -> None:
        tab = self._make_tab()

        main_span = tab._canvas_dimensions(tab.canvas)[0] / tab._canvas_display_scale(tab.canvas)
        magnifier_span = tab._canvas_dimensions(tab.magnifier_canvas)[0] / tab._canvas_display_scale(tab.magnifier_canvas)

        self.assertAlmostEqual(main_span, 420.0)
        self.assertAlmostEqual(magnifier_span, 105.0)
        self.assertAlmostEqual(main_span / magnifier_span, 4.0)

    def test_view_transform_roundtrip_matches_on_both_canvases(self) -> None:
        tab = self._make_tab()
        point = (263.0, 171.0)

        for canvas in (tab.canvas, tab.magnifier_canvas):
            view = tab._to_view(point, canvas)
            model = tab._from_view(view, canvas)
            self.assertAlmostEqual(model[0], point[0])
            self.assertAlmostEqual(model[1], point[1])

    def test_route_canvases_support_legacy_single_canvas_mode(self) -> None:
        tab = self._make_tab()
        tab.magnifier_canvas = None

        self.assertEqual(tab._route_canvases(), (tab.canvas,))

    def test_manual_map_scale_entry_clamps_to_allowed_range(self) -> None:
        tab = self._make_tab()
        tab._map_image_scale_text_var.set("7.50")

        tab._commit_map_scale_text()

        self.assertEqual(tab._map_image_scale_var.get(), 6.0)
        self.assertEqual(tab._map_image_scale_text_var.get(), "6.00")

    def test_manual_map_rotation_entry_clamps_to_allowed_range(self) -> None:
        tab = self._make_tab()
        tab._map_rotation_text_var.set("-15")

        tab._commit_map_rotation_text()

        self.assertEqual(tab._map_rotation_var.get(), 0.0)
        self.assertEqual(tab._map_rotation_text_var.get(), "0.0")


if __name__ == "__main__":
    unittest.main()
