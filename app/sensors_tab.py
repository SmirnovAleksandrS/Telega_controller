from __future__ import annotations

import tkinter as tk
from tkinter import ttk
from typing import Callable

from app.magnetometer_tab import MagnetometerTab


class SensorsTab(ttk.Frame):
    def __init__(
        self,
        master: tk.Widget,
        *,
        on_start_record: Callable[[], None] | None = None,
        on_stop_record: Callable[[], None] | None = None,
        on_load_csv: Callable[[], None] | None = None,
        on_load_multiple: Callable[[], None] | None = None,
        on_save_current: Callable[[], None] | None = None,
        on_save_as: Callable[[], None] | None = None,
        on_concatenate: Callable[[], None] | None = None,
        on_trim_selection: Callable[[], None] | None = None,
        on_delete_selection: Callable[[], None] | None = None,
        on_select_dataset: Callable[[int], None] | None = None,
        on_select_source: Callable[[str], None] | None = None,
        on_source_show_change: Callable[[str, bool], None] | None = None,
        on_source_record_change: Callable[[str, bool], None] | None = None,
        on_add_plugin: Callable[[], None] | None = None,
        on_select_method: Callable[[str], None] | None = None,
        on_open_method_info: Callable[[str], None] | None = None,
        on_calibrate_method: Callable[[str], None] | None = None,
        on_load_method_params: Callable[[str], None] | None = None,
        on_save_method_params: Callable[[str], None] | None = None,
        on_method_show_change: Callable[[str, bool], None] | None = None,
        on_remove_method: Callable[[str], None] | None = None,
        on_enable_method_realtime: Callable[[str], None] | None = None,
        on_disable_method_realtime: Callable[[str], None] | None = None,
        on_method_record_change: Callable[[str, bool], None] | None = None,
        on_select_primary_heading: Callable[[str], None] | None = None,
        on_export_metrics: Callable[[], None] | None = None,
    ) -> None:
        super().__init__(master)

        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)

        self.nb = ttk.Notebook(self)
        self.nb.grid(row=0, column=0, sticky="nsew")

        self.magnetometer_tab = MagnetometerTab(
            self.nb,
            on_start_record=on_start_record,
            on_stop_record=on_stop_record,
            on_load_csv=on_load_csv,
            on_load_multiple=on_load_multiple,
            on_save_current=on_save_current,
            on_save_as=on_save_as,
            on_concatenate=on_concatenate,
            on_trim_selection=on_trim_selection,
            on_delete_selection=on_delete_selection,
            on_select_dataset=on_select_dataset,
            on_select_source=on_select_source,
            on_source_show_change=on_source_show_change,
            on_source_record_change=on_source_record_change,
            on_add_plugin=on_add_plugin,
            on_select_method=on_select_method,
            on_open_method_info=on_open_method_info,
            on_calibrate_method=on_calibrate_method,
            on_load_method_params=on_load_method_params,
            on_save_method_params=on_save_method_params,
            on_method_show_change=on_method_show_change,
            on_remove_method=on_remove_method,
            on_enable_method_realtime=on_enable_method_realtime,
            on_disable_method_realtime=on_disable_method_realtime,
            on_method_record_change=on_method_record_change,
            on_select_primary_heading=on_select_primary_heading,
            on_export_metrics=on_export_metrics,
        )
        self.nb.add(self.magnetometer_tab, text="Magnetometer")
