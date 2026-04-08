from __future__ import annotations

import tkinter as tk
from tkinter import ttk
import unittest

from app.magnetometer_tab import (
    MagnetometerTab,
    PROJECTION_3D,
    PROJECTION_XY,
    PROJECTION_XZ,
    PROJECTION_YZ,
    compute_raw_heading_deg,
    project_magnetometer_point,
    project_rotated_magnetometer_point,
    rotate_magnetometer_point,
)
from app.method_card import MethodCard
from app.source_card import SourceCard, resolve_source_card_status
from app.sensors_tab import SensorsTab


class SensorsUiShellImportTests(unittest.TestCase):
    def test_sensors_tab_is_a_ttk_frame(self) -> None:
        self.assertTrue(issubclass(SensorsTab, ttk.Frame))

    def test_magnetometer_tab_is_a_ttk_frame(self) -> None:
        self.assertTrue(issubclass(MagnetometerTab, ttk.Frame))

    def test_source_card_is_a_tk_widget(self) -> None:
        self.assertTrue(issubclass(SourceCard, tk.Widget))

    def test_method_card_is_a_tk_widget(self) -> None:
        self.assertTrue(issubclass(MethodCard, tk.Widget))

    def test_new_tabs_keep_tk_widget_contract(self) -> None:
        self.assertTrue(issubclass(SensorsTab, tk.Widget))
        self.assertTrue(issubclass(MagnetometerTab, tk.Widget))

    def test_raw_heading_uses_north_zero_east_ninety(self) -> None:
        self.assertAlmostEqual(compute_raw_heading_deg(0.0, 1.0), 0.0)
        self.assertAlmostEqual(compute_raw_heading_deg(1.0, 0.0), 90.0)
        self.assertAlmostEqual(compute_raw_heading_deg(0.0, -1.0), 180.0)
        self.assertAlmostEqual(compute_raw_heading_deg(-1.0, 0.0), 270.0)

    def test_raw_heading_returns_none_for_zero_vector(self) -> None:
        self.assertIsNone(compute_raw_heading_deg(0.0, 0.0))

    def test_projection_xy_maps_horizontal_plane_directly(self) -> None:
        self.assertEqual(project_magnetometer_point(PROJECTION_XY, 1.5, -2.0, 3.25), (1.5, -2.0))

    def test_projection_xz_maps_vertical_slice(self) -> None:
        self.assertEqual(project_magnetometer_point(PROJECTION_XZ, 1.5, -2.0, 3.25), (1.5, 3.25))

    def test_projection_yz_maps_vertical_slice(self) -> None:
        self.assertEqual(project_magnetometer_point(PROJECTION_YZ, 1.5, -2.0, 3.25), (-2.0, 3.25))

    def test_projection_3d_returns_stable_isometric_projection(self) -> None:
        x_val, y_val = project_magnetometer_point(PROJECTION_3D, 1.0, -1.0, 2.0)
        self.assertAlmostEqual(x_val, 1.7320508075688772)
        self.assertAlmostEqual(y_val, 2.0)

    def test_rotate_magnetometer_point_is_identity_for_zero_angles(self) -> None:
        self.assertEqual(
            rotate_magnetometer_point(1.0, -2.0, 3.0, yaw_deg=0.0, pitch_deg=0.0),
            (1.0, -2.0, 3.0),
        )

    def test_project_rotated_magnetometer_point_uses_rotated_xz_plane(self) -> None:
        x_val, y_val = project_rotated_magnetometer_point(1.0, 0.0, 0.0, yaw_deg=90.0, pitch_deg=0.0)
        self.assertAlmostEqual(x_val, 0.0, places=6)
        self.assertAlmostEqual(y_val, 0.0, places=6)

    def test_source_status_is_active_when_show_and_record_enabled(self) -> None:
        self.assertEqual(resolve_source_card_status(True, True)[0], "ACTIVE")

    def test_source_status_is_partial_when_only_one_toggle_enabled(self) -> None:
        self.assertEqual(resolve_source_card_status(True, False)[0], "PARTIAL")
        self.assertEqual(resolve_source_card_status(False, True)[0], "PARTIAL")

    def test_source_status_is_idle_when_all_toggles_disabled(self) -> None:
        self.assertEqual(resolve_source_card_status(False, False)[0], "IDLE")


if __name__ == "__main__":
    unittest.main()
