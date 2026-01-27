"""
Dialogs: COM settings, Deviation settings.
"""

from __future__ import annotations
import tkinter as tk
from tkinter import ttk, messagebox
from dataclasses import dataclass

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

        self.grab_set()
        self.protocol("WM_DELETE_WINDOW", self._cancel)

    def _ok(self) -> None:
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("COM Settings", "Select port")
            return
        try:
            baud = int(self.baud_var.get().strip())
        except ValueError:
            messagebox.showerror("COM Settings", "Invalid baudrate")
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
            messagebox.showerror("Deviation Settings", "Invalid number format")
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

        btns = ttk.Frame(frm)
        btns.grid(row=2, column=0, columnspan=2, sticky="e", pady=(10, 0))
        ttk.Button(btns, text="Cancel", command=self._cancel).grid(row=0, column=0, padx=(0, 8))
        ttk.Button(btns, text="OK", command=self._ok).grid(row=0, column=1)

        self.grab_set()
        self.protocol("WM_DELETE_WINDOW", self._cancel)

    def _ok(self) -> None:
        try:
            a1 = float(self.a1_var.get())
            a2 = float(self.a2_var.get())
        except ValueError:
            messagebox.showerror("Geometry Parameters", "Invalid number format")
            return
        if a1 < 0 or a2 < 0:
            messagebox.showerror("Geometry Parameters", "Values must be >= 0")
            return
        self.result = GeometryConfig(a1_cm=a1, a2_cm=a2)
        self.destroy()

    def _cancel(self) -> None:
        self.result = None
        self.destroy()
