"""
Dialogs: COM settings, Deviation settings.
"""

from __future__ import annotations
import json
import math
import os
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from dataclasses import dataclass

from app.tk_utils import bind_vertical_mousewheel, prepare_toplevel
from comm.serial_worker import SerialWorker

@dataclass
class DeviationConfig:
    # Per parameter: normal, delta
    voltage_normal: float = 43.0
    voltage_delta: float = 2.0
    current_normal: float = 40.0
    current_delta: float = 80.0
    temp_normal: float = 40.0
    temp_delta: float = 20.0


@dataclass
class GeometryConfig:
    a1_cm: float = 20.0
    a2_cm: float = 20.0
    drive_wheel_diameter_cm: float = 10.0

    @property
    def track_circumference_m(self) -> float:
        return math.pi * self.drive_wheel_diameter_cm / 100.0


@dataclass
class SpeedMapConfig:
    pwm_1: float = 2000.0
    speed_1: float = 10.0
    pwm_2: float = 1500.0
    speed_2: float = 0.0
    pwm_3: float = 1000.0
    speed_3: float = -10.0

class ComSettingsDialog(tk.Toplevel):
    def __init__(self, master: tk.Widget, current_port: str, current_baud: int) -> None:
        super().__init__(master)
        self.title("COM Settings")
        self.resizable(False, False)
        self.result = None  # (port, baud) or None

        frm = ttk.Frame(self, padding=10)
        frm.grid(row=0, column=0, sticky="nsew")

        ttk.Label(frm, text="Port:").grid(row=0, column=0, sticky="w")
        self.port_var = tk.StringVar(value=current_port)
        ports = SerialWorker.list_ports()
        self.port_cb = ttk.Combobox(frm, textvariable=self.port_var, values=ports, width=25)
        self.port_cb.grid(row=0, column=1, sticky="ew", padx=(8, 0))

        ttk.Label(frm, text="Baud:").grid(row=1, column=0, sticky="w", pady=(8, 0))
        self.baud_var = tk.StringVar(value=str(current_baud))
        self.baud_cb = ttk.Combobox(frm, textvariable=self.baud_var,
                                    values=["9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600"],
                                    width=25)
        self.baud_cb.grid(row=1, column=1, sticky="ew", padx=(8, 0), pady=(8, 0))

        btns = ttk.Frame(frm)
        btns.grid(row=2, column=0, columnspan=2, sticky="e", pady=(10, 0))

        ttk.Button(btns, text="Cancel", command=self._cancel).grid(row=0, column=0, padx=(0, 8))
        ttk.Button(btns, text="OK", command=self._ok).grid(row=0, column=1)

        prepare_toplevel(self, master)
        self.grab_set()
        self.protocol("WM_DELETE_WINDOW", self._cancel)

    def _ok(self) -> None:
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("COM Settings", "Select port", parent=self)
            return
        try:
            baud = int(self.baud_var.get().strip())
        except ValueError:
            messagebox.showerror("COM Settings", "Invalid baudrate", parent=self)
            return
        self.result = (port, baud)
        self.destroy()

    def _cancel(self) -> None:
        self.result = None
        self.destroy()


class DeviationSettingsDialog(tk.Toplevel):
    def __init__(self, master: tk.Widget, cfg: DeviationConfig) -> None:
        super().__init__(master)
        self.title("Deviation Settings")
        self.resizable(False, False)

        self.cfg = cfg
        self.result: DeviationConfig | None = None

        frm = ttk.Frame(self, padding=10)
        frm.grid(row=0, column=0, sticky="nsew")

        ttk.Label(frm, text="Parameter").grid(row=0, column=0, sticky="w")
        ttk.Label(frm, text="Normal").grid(row=0, column=1, sticky="w", padx=(8, 0))
        ttk.Label(frm, text="Delta").grid(row=0, column=2, sticky="w", padx=(8, 0))

        # Helper to add row
        self._vars = {}
        def add_row(r: int, name: str, normal: float, delta: float) -> None:
            ttk.Label(frm, text=name).grid(row=r, column=0, sticky="w", pady=4)
            v_n = tk.StringVar(value=str(normal))
            v_d = tk.StringVar(value=str(delta))
            ttk.Entry(frm, textvariable=v_n, width=10).grid(row=r, column=1, sticky="w", padx=(8, 0))
            ttk.Entry(frm, textvariable=v_d, width=10).grid(row=r, column=2, sticky="w", padx=(8, 0))
            self._vars[name] = (v_n, v_d)

        add_row(1, "Voltage", self.cfg.voltage_normal, self.cfg.voltage_delta)
        add_row(2, "Current", self.cfg.current_normal, self.cfg.current_delta)
        add_row(3, "Temperature", self.cfg.temp_normal, self.cfg.temp_delta)

        btns = ttk.Frame(frm)
        btns.grid(row=4, column=0, columnspan=3, sticky="e", pady=(10, 0))
        ttk.Button(btns, text="Cancel", command=self._cancel).grid(row=0, column=0, padx=(0, 8))
        ttk.Button(btns, text="OK", command=self._ok).grid(row=0, column=1)

        prepare_toplevel(self, master)
        self.grab_set()
        self.protocol("WM_DELETE_WINDOW", self._cancel)

    def _ok(self) -> None:
        try:
            vn, vd = self._vars["Voltage"]
            cn, cd = self._vars["Current"]
            tn, td = self._vars["Temperature"]

            new = DeviationConfig(
                voltage_normal=float(vn.get()), voltage_delta=float(vd.get()),
                current_normal=float(cn.get()), current_delta=float(cd.get()),
                temp_normal=float(tn.get()), temp_delta=float(td.get()),
            )
        except ValueError:
            messagebox.showerror("Deviation Settings", "Invalid number format", parent=self)
            return

        self.result = new
        self.destroy()

    def _cancel(self) -> None:
        self.result = None
        self.destroy()


class GeometrySettingsDialog(tk.Toplevel):
    def __init__(self, master: tk.Widget, cfg: GeometryConfig) -> None:
        super().__init__(master)
        self.title("Geometry Parameters")
        self.resizable(False, False)

        self.cfg = cfg
        self.result: GeometryConfig | None = None

        frm = ttk.Frame(self, padding=10)
        frm.grid(row=0, column=0, sticky="nsew")

        ttk.Label(frm, text="A1 (cm)").grid(row=0, column=0, sticky="w")
        self.a1_var = tk.StringVar(value=str(self.cfg.a1_cm))
        ttk.Entry(frm, textvariable=self.a1_var, width=10).grid(row=0, column=1, sticky="w", padx=(8, 0))

        ttk.Label(frm, text="A2 (cm)").grid(row=1, column=0, sticky="w", pady=(8, 0))
        self.a2_var = tk.StringVar(value=str(self.cfg.a2_cm))
        ttk.Entry(frm, textvariable=self.a2_var, width=10).grid(row=1, column=1, sticky="w", padx=(8, 0), pady=(8, 0))

        ttk.Label(frm, text="Drive wheel diameter (cm)").grid(row=2, column=0, sticky="w", pady=(8, 0))
        self.drive_wheel_diameter_var = tk.StringVar(value=str(self.cfg.drive_wheel_diameter_cm))
        ttk.Entry(frm, textvariable=self.drive_wheel_diameter_var, width=10).grid(row=2, column=1, sticky="w", padx=(8, 0), pady=(8, 0))

        btns = ttk.Frame(frm)
        btns.grid(row=3, column=0, columnspan=2, sticky="e", pady=(10, 0))
        ttk.Button(btns, text="Cancel", command=self._cancel).grid(row=0, column=0, padx=(0, 8))
        ttk.Button(btns, text="OK", command=self._ok).grid(row=0, column=1)

        prepare_toplevel(self, master)
        self.grab_set()
        self.protocol("WM_DELETE_WINDOW", self._cancel)

    def _ok(self) -> None:
        try:
            a1 = float(self.a1_var.get())
            a2 = float(self.a2_var.get())
            drive_wheel_diameter_cm = float(self.drive_wheel_diameter_var.get())
        except ValueError:
            messagebox.showerror("Geometry Parameters", "Invalid number format", parent=self)
            return
        if a1 < 0 or a2 < 0:
            messagebox.showerror("Geometry Parameters", "A1/A2 must be >= 0", parent=self)
            return
        if drive_wheel_diameter_cm <= 0:
            messagebox.showerror("Geometry Parameters", "Drive wheel diameter must be > 0", parent=self)
            return
        self.result = GeometryConfig(
            a1_cm=a1,
            a2_cm=a2,
            drive_wheel_diameter_cm=drive_wheel_diameter_cm,
        )
        self.destroy()

    def _cancel(self) -> None:
        self.result = None
        self.destroy()


class SpeedMapDialog(tk.Toplevel):
    def __init__(self, master: tk.Widget, cfg: SpeedMapConfig) -> None:
        super().__init__(master)
        self.title("Speed Parameters")
        self.resizable(False, False)

        self.cfg = cfg
        self.result: SpeedMapConfig | None = None

        frm = ttk.Frame(self, padding=10)
        frm.grid(row=0, column=0, sticky="nsew")

        ttk.Label(frm, text="PWM").grid(row=0, column=0, sticky="w")
        ttk.Label(frm, text="Speed (m/s)").grid(row=0, column=1, sticky="w", padx=(8, 0))

        self._vars = []
        def add_row(r: int, pwm: float, speed: float) -> None:
            v_pwm = tk.StringVar(value=str(pwm))
            v_spd = tk.StringVar(value=str(speed))
            ttk.Entry(frm, textvariable=v_pwm, width=10).grid(row=r, column=0, sticky="w", pady=4)
            ttk.Entry(frm, textvariable=v_spd, width=10).grid(row=r, column=1, sticky="w", padx=(8, 0), pady=4)
            self._vars.append((v_pwm, v_spd))

        add_row(1, self.cfg.pwm_1, self.cfg.speed_1)
        add_row(2, self.cfg.pwm_2, self.cfg.speed_2)
        add_row(3, self.cfg.pwm_3, self.cfg.speed_3)

        btns = ttk.Frame(frm)
        btns.grid(row=4, column=0, columnspan=2, sticky="e", pady=(10, 0))
        ttk.Button(btns, text="Cancel", command=self._cancel).grid(row=0, column=0, padx=(0, 8))
        ttk.Button(btns, text="OK", command=self._ok).grid(row=0, column=1)

        prepare_toplevel(self, master)
        self.grab_set()
        self.protocol("WM_DELETE_WINDOW", self._cancel)

    def _ok(self) -> None:
        try:
            (p1, s1), (p2, s2), (p3, s3) = self._vars
            cfg = SpeedMapConfig(
                pwm_1=float(p1.get()), speed_1=float(s1.get()),
                pwm_2=float(p2.get()), speed_2=float(s2.get()),
                pwm_3=float(p3.get()), speed_3=float(s3.get()),
            )
        except ValueError:
            messagebox.showerror("Speed Parameters", "Invalid number format", parent=self)
            return
        self.result = cfg
        self.destroy()

    def _cancel(self) -> None:
        self.result = None
        self.destroy()


class DatasetSelectionDialog(tk.Toplevel):
    def __init__(
        self,
        master: tk.Widget,
        *,
        title: str,
        prompt: str,
        items: list[str],
        preselected: list[int] | None = None,
    ) -> None:
        super().__init__(master)
        self.title(title)
        self.resizable(False, False)
        self.result: list[int] | None = None
        self._items = list(items)

        frm = ttk.Frame(self, padding=10)
        frm.grid(row=0, column=0, sticky="nsew")
        frm.columnconfigure(0, weight=1)
        frm.rowconfigure(1, weight=1)

        ttk.Label(frm, text=prompt).grid(row=0, column=0, sticky="w")

        list_host = ttk.Frame(frm)
        list_host.grid(row=1, column=0, sticky="nsew", pady=(8, 0))
        list_host.columnconfigure(0, weight=1)
        list_host.rowconfigure(0, weight=1)

        self.listbox = tk.Listbox(list_host, selectmode="extended", height=min(8, max(4, len(items))))
        self.listbox.grid(row=0, column=0, sticky="nsew")
        bind_vertical_mousewheel(self.listbox)
        for item in items:
            self.listbox.insert("end", item)
        if preselected:
            for index in preselected:
                if 0 <= index < len(items):
                    self.listbox.selection_set(index)

        yscroll = ttk.Scrollbar(list_host, orient="vertical", command=self.listbox.yview)
        yscroll.grid(row=0, column=1, sticky="ns")
        self.listbox.configure(yscrollcommand=yscroll.set)

        btns = ttk.Frame(frm)
        btns.grid(row=2, column=0, sticky="e", pady=(10, 0))
        ttk.Button(btns, text="Cancel", command=self._cancel).grid(row=0, column=0, padx=(0, 8))
        ttk.Button(btns, text="OK", command=self._ok).grid(row=0, column=1)

        prepare_toplevel(self, master)
        self.grab_set()
        self.protocol("WM_DELETE_WINDOW", self._cancel)

    def _ok(self) -> None:
        self.result = [int(index) for index in self.listbox.curselection()]
        self.destroy()

    def _cancel(self) -> None:
        self.result = None
        self.destroy()


class AddPluginDialog(tk.Toplevel):
    def __init__(self, master: tk.Widget, *, initial_dir: str | None = None) -> None:
        super().__init__(master)
        self.title("Add Source / Method")
        self.resizable(False, False)
        self.result: dict[str, str] | None = None
        self._initial_dir = initial_dir

        frm = ttk.Frame(self, padding=10)
        frm.grid(row=0, column=0, sticky="nsew")
        frm.columnconfigure(1, weight=1)

        ttk.Label(frm, text="Kind").grid(row=0, column=0, sticky="w")
        self.kind_var = tk.StringVar(value="Method plugin (.py)")
        kind_cb = ttk.Combobox(frm, textvariable=self.kind_var, values=("Method plugin (.py)",), state="readonly", width=32)
        kind_cb.grid(row=0, column=1, sticky="ew", padx=(8, 0))

        ttk.Label(frm, text="Python file").grid(row=1, column=0, sticky="w", pady=(8, 0))
        self.path_var = tk.StringVar()
        ttk.Entry(frm, textvariable=self.path_var, width=42).grid(row=1, column=1, sticky="ew", padx=(8, 0), pady=(8, 0))

        browse_btn = ttk.Button(frm, text="Browse...", command=self._browse)
        browse_btn.grid(row=1, column=2, sticky="w", padx=(8, 0), pady=(8, 0))

        hint = ttk.Label(
            frm,
            text="Source plugins can be added later. This phase loads method plugins only.",
            foreground="#444444",
        )
        hint.grid(row=2, column=0, columnspan=3, sticky="w", pady=(8, 0))

        btns = ttk.Frame(frm)
        btns.grid(row=3, column=0, columnspan=3, sticky="e", pady=(10, 0))
        ttk.Button(btns, text="Cancel", command=self._cancel).grid(row=0, column=0, padx=(0, 8))
        ttk.Button(btns, text="Add", command=self._ok).grid(row=0, column=1)

        prepare_toplevel(self, master)
        self.grab_set()
        self.protocol("WM_DELETE_WINDOW", self._cancel)

    def _browse(self) -> None:
        default_dir = self._initial_dir or os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "externModules", "magnetometer")
        )
        path = filedialog.askopenfilename(
            title="Select Python plugin",
            initialdir=default_dir if os.path.isdir(default_dir) else None,
            filetypes=[("Python files", "*.py"), ("All files", "*.*")],
            parent=self,
        )
        if path:
            self.path_var.set(path)

    def _ok(self) -> None:
        path = self.path_var.get().strip()
        if not path:
            messagebox.showerror("Add Source / Method", "Select a Python file.", parent=self)
            return
        self.result = {"kind": "method", "path": path}
        self.destroy()

    def _cancel(self) -> None:
        self.result = None
        self.destroy()


class MethodInfoDialog(tk.Toplevel):
    def __init__(self, master: tk.Widget, *, title: str, info_text: str) -> None:
        super().__init__(master)
        self.title(title)
        self.geometry("720x520")

        frm = ttk.Frame(self, padding=10)
        frm.pack(fill="both", expand=True)

        text = tk.Text(frm, wrap="word")
        text.pack(side="left", fill="both", expand=True)
        bind_vertical_mousewheel(text)
        text.insert("1.0", info_text)
        text.configure(state="disabled")

        yscroll = ttk.Scrollbar(frm, orient="vertical", command=text.yview)
        yscroll.pack(side="right", fill="y")
        text.configure(yscrollcommand=yscroll.set)

        prepare_toplevel(self, master)
        self.grab_set()


class MethodDiagnosticsDialog(tk.Toplevel):
    def __init__(self, master: tk.Widget, *, title: str, diagnostics: dict[str, object]) -> None:
        super().__init__(master)
        self.title(title)
        self.geometry("820x620")

        frm = ttk.Frame(self, padding=10)
        frm.pack(fill="both", expand=True)

        sections = [
            f"Method: {diagnostics.get('name', '-')}",
            f"Version: {diagnostics.get('version', '-')}",
            f"File path: {diagnostics.get('file_path', '-')}",
            f"Last action: {diagnostics.get('last_action', '-')}",
            "",
            "Error:",
            str(diagnostics.get("error_text", "-")),
            "",
            "Warnings:",
            "\n".join(diagnostics.get("warnings", []) or ["-"]),
            "",
            "Traceback:",
            str(diagnostics.get("traceback_text", "-")),
        ]

        text = tk.Text(frm, wrap="word")
        text.pack(side="left", fill="both", expand=True)
        bind_vertical_mousewheel(text)
        text.insert("1.0", "\n".join(sections))
        text.configure(state="disabled")

        yscroll = ttk.Scrollbar(frm, orient="vertical", command=text.yview)
        yscroll.pack(side="right", fill="y")
        text.configure(yscrollcommand=yscroll.set)

        prepare_toplevel(self, master)
        self.grab_set()


def format_method_info_text(
    info: dict[str, object],
    *,
    file_path: str,
    status_text: str = "-",
    warnings: list[str] | None = None,
    calibration_dataset_name: str | None = None,
    calibration_runtime_s: float | None = None,
    params_profile_path: str | None = None,
    calibration_params: object = None,
    calibration_report: str = "",
) -> str:
    capabilities = [
        f"supports_calibrate={bool(info.get('supports_calibrate', False))}",
        f"supports_load_params={bool(info.get('supports_load_params', False))}",
        f"supports_save_params={bool(info.get('supports_save_params', False))}",
        f"supports_process={bool(info.get('supports_process', False))}",
    ]
    payload = {
        "name": info.get("name", "-"),
        "version": info.get("version", "-"),
        "type": info.get("type", "-"),
        "file_path": file_path,
        "capabilities": capabilities,
        "input_schema": info.get("input_schema"),
        "output_schema": info.get("output_schema"),
        "runtime": {
            "status": status_text,
            "warnings": list(warnings or []),
            "calibration_dataset_name": calibration_dataset_name or "",
            "calibration_runtime_s": calibration_runtime_s,
            "params_profile_path": params_profile_path or "",
            "has_calibration_params": calibration_params is not None,
            "calibration_params": calibration_params,
        },
    }
    text = json.dumps(payload, ensure_ascii=False, indent=2)
    if calibration_report:
        text = f"{text}\n\nCalibration report:\n{calibration_report}"
    return text
