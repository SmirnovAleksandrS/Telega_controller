from __future__ import annotations

import importlib.util
import json
import os
import traceback
from dataclasses import dataclass, field
from types import ModuleType
from typing import Any

REQUIRED_FUNCTIONS = ("get_info", "calibrate", "load_params", "save_params", "process")
CALIBRATION_HEADER_KEYS = ("algorithm_name", "algorithm_version", "schema_version", "created_at")
REQUIRED_INFO_KEYS = (
    "name",
    "version",
    "type",
    "supports_calibrate",
    "supports_load_params",
    "supports_save_params",
    "supports_process",
    "input_schema",
    "output_schema",
)


@dataclass
class PluginDiagnostics:
    name: str
    version: str
    file_path: str
    last_action: str
    error_text: str
    traceback_text: str
    warnings: list[str] = field(default_factory=list)

    def as_dict(self) -> dict[str, object]:
        return {
            "name": self.name,
            "version": self.version,
            "file_path": self.file_path,
            "last_action": self.last_action,
            "error_text": self.error_text,
            "traceback_text": self.traceback_text,
            "warnings": list(self.warnings),
        }


@dataclass
class LoadedMethodPlugin:
    method_id: str
    name: str
    version: str
    file_path: str
    info: dict[str, Any]
    module: ModuleType | None
    status: str
    warnings: list[str] = field(default_factory=list)
    diagnostics: PluginDiagnostics | None = None
    load_status: str = ""
    load_warnings: list[str] = field(default_factory=list)
    calibration_params: Any = None
    calibration_report: str = ""
    calibration_warnings: list[str] = field(default_factory=list)
    calibration_dataset_name: str | None = None
    calibration_runtime_s: float | None = None
    calibration_progress: float = 0.0
    is_calibrating: bool = False
    params_warnings: list[str] = field(default_factory=list)
    params_profile_path: str | None = None
    process_warnings: list[str] = field(default_factory=list)
    realtime_enabled: bool = False
    show_enabled: bool = False
    record_enabled: bool = False
    derived_stream_id: str | None = None
    last_output: dict[str, Any] | None = None

    def __post_init__(self) -> None:
        if not self.load_status:
            self.load_status = self.status
        if not self.load_warnings:
            self.load_warnings = list(self.warnings)

    def capabilities_label(self) -> str:
        parts: list[str] = []
        for key in ("supports_calibrate", "supports_load_params", "supports_save_params", "supports_process"):
            if bool(self.info.get(key, False)):
                parts.append(key.removeprefix("supports_"))
        return ", ".join(parts) if parts else "none"

    def supports(self, capability: str) -> bool:
        return bool(self.info.get(capability, False))

    def supports_calibrate(self) -> bool:
        return self.supports("supports_calibrate")

    def supports_process(self) -> bool:
        return self.supports("supports_process")

    def effective_warnings(self) -> list[str]:
        warnings = list(self.load_warnings)
        warnings.extend(self.calibration_warnings)
        warnings.extend(self.params_warnings)
        warnings.extend(self.process_warnings)
        if self.supports_calibrate() and self.calibration_params is None and self.status != "error":
            warnings.append("calibration not completed")
        deduped: list[str] = []
        seen: set[str] = set()
        for warning in warnings:
            key = str(warning)
            if key in seen:
                continue
            seen.add(key)
            deduped.append(key)
        return deduped

    def display_status(self) -> str:
        if self.status == "error":
            return "error"
        if self.is_calibrating:
            return "calibrating"
        if self.effective_warnings():
            return "warning"
        return "ready" if self.load_status != "partial" else "partial"

    def display_status_text(self) -> str:
        status = self.display_status()
        if status == "calibrating":
            return f"CALIBRATING {int(max(0.0, min(1.0, self.calibration_progress)) * 100):d}%"
        if status == "ready":
            return "READY"
        if status == "warning":
            return "WARNING"
        if status == "error":
            return "ERROR"
        return "PARTIAL"

    def can_start_calibration(self) -> bool:
        return self.module is not None and self.supports_calibrate() and not self.is_calibrating

    def can_load_params(self) -> bool:
        return self.module is not None and self.supports("supports_load_params") and not self.is_calibrating

    def can_save_params(self) -> bool:
        return (
            self.module is not None
            and self.supports("supports_save_params")
            and self.calibration_params is not None
            and not self.is_calibrating
        )

    def can_toggle_show(self) -> bool:
        return self.module is not None and self.supports_process()

    def can_enable_realtime(self) -> bool:
        return (
            self.module is not None
            and self.supports_process()
            and not self.is_calibrating
            and not self.realtime_enabled
            and self.calibration_params is not None
            and self.status != "error"
        )

    def can_disable_realtime(self) -> bool:
        return self.module is not None and self.realtime_enabled

    def can_toggle_record(self) -> bool:
        return self.module is not None and self.supports_process() and self.realtime_enabled


@dataclass
class CalibrationRunResult:
    ok: bool
    params: Any = None
    warnings: list[str] = field(default_factory=list)
    report: str = ""
    diagnostics: PluginDiagnostics | None = None


@dataclass
class ParamIoResult:
    ok: bool
    params: Any = None
    warnings: list[str] = field(default_factory=list)
    diagnostics: PluginDiagnostics | None = None


@dataclass
class ProcessRunResult:
    ok: bool
    output: dict[str, Any] | None = None
    warnings: list[str] = field(default_factory=list)
    diagnostics: PluginDiagnostics | None = None


def _make_failure(file_path: str, *, last_action: str, error_text: str, traceback_text: str, warnings: list[str] | None = None) -> LoadedMethodPlugin:
    base_name = os.path.splitext(os.path.basename(file_path))[0] or "unknown_plugin"
    diagnostics = PluginDiagnostics(
        name=base_name,
        version="unknown",
        file_path=file_path,
        last_action=last_action,
        error_text=error_text,
        traceback_text=traceback_text,
        warnings=list(warnings or []),
    )
    return LoadedMethodPlugin(
        method_id="",
        name=base_name,
        version="unknown",
        file_path=file_path,
        info={},
        module=None,
        status="error",
        warnings=list(warnings or []),
        diagnostics=diagnostics,
    )


def load_method_plugin(path: str) -> LoadedMethodPlugin:
    abs_path = os.path.abspath(path)
    module_name = f"mag_plugin_{abs(hash((abs_path, os.path.getmtime(abs_path) if os.path.exists(abs_path) else 0))):x}"

    try:
        spec = importlib.util.spec_from_file_location(module_name, abs_path)
        if spec is None or spec.loader is None:
            return _make_failure(
                abs_path,
                last_action="import",
                error_text="Unable to build import specification.",
                traceback_text="",
            )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    except Exception as exc:
        return _make_failure(
            abs_path,
            last_action="import",
            error_text=str(exc),
            traceback_text=traceback.format_exc(),
        )

    missing = [name for name in REQUIRED_FUNCTIONS if not callable(getattr(module, name, None))]
    if missing:
        return _make_failure(
            abs_path,
            last_action="validate_api",
            error_text=f"Missing required callable(s): {', '.join(missing)}",
            traceback_text="",
        )

    try:
        info = module.get_info()
    except Exception as exc:
        return _make_failure(
            abs_path,
            last_action="get_info",
            error_text=str(exc),
            traceback_text=traceback.format_exc(),
        )

    if not isinstance(info, dict):
        return _make_failure(
            abs_path,
            last_action="validate_info",
            error_text="get_info() must return a dict.",
            traceback_text="",
        )

    missing_keys = [key for key in REQUIRED_INFO_KEYS if key not in info]
    if missing_keys:
        return _make_failure(
            abs_path,
            last_action="validate_info",
            error_text=f"get_info() missing required keys: {', '.join(missing_keys)}",
            traceback_text="",
        )

    warnings: list[str] = []
    capability_keys = ("supports_calibrate", "supports_load_params", "supports_save_params", "supports_process")
    for key in capability_keys:
        if not isinstance(info.get(key), bool):
            return _make_failure(
                abs_path,
                last_action="validate_info",
                error_text=f"{key} must be bool.",
                traceback_text="",
            )
        if not info.get(key):
            warnings.append(f"{key} is disabled")

    status = "warning" if warnings else "ready"
    return LoadedMethodPlugin(
        method_id="",
        name=str(info["name"]),
        version=str(info["version"]),
        file_path=abs_path,
        info=dict(info),
        module=module,
        status=status,
        warnings=warnings,
        diagnostics=None,
    )


def _coerce_validation_warnings(result: Any) -> tuple[list[str], bool]:
    if result is None or result is True:
        return ([], False)
    if result is False:
        return (["validate_dataset() rejected the active dataset."], True)
    if isinstance(result, str):
        text = result.strip()
        return ([text] if text else [], False)
    if isinstance(result, (list, tuple, set)):
        warnings = [str(item).strip() for item in result if str(item).strip()]
        return (warnings, False)
    if isinstance(result, dict):
        warnings = [str(item).strip() for item in result.get("warnings", []) if str(item).strip()]
        rejected = bool(result.get("ok") is False or result.get("reject") is True)
        return (warnings, rejected)
    return ([f"validate_dataset() returned unsupported type: {type(result).__name__}"], False)


def run_method_calibration(plugin: LoadedMethodPlugin, dataset: Any, config: Any = None) -> CalibrationRunResult:
    if plugin.module is None:
        diagnostics = PluginDiagnostics(
            name=plugin.name,
            version=plugin.version,
            file_path=plugin.file_path,
            last_action="calibrate",
            error_text="Plugin module is not loaded.",
            traceback_text="",
            warnings=list(plugin.effective_warnings()),
        )
        return CalibrationRunResult(ok=False, diagnostics=diagnostics)

    warnings = list(plugin.load_warnings)
    validate_dataset = getattr(plugin.module, "validate_dataset", None)
    if callable(validate_dataset):
        try:
            validation_result = validate_dataset(dataset)
        except Exception as exc:
            diagnostics = PluginDiagnostics(
                name=plugin.name,
                version=plugin.version,
                file_path=plugin.file_path,
                last_action="validate_dataset",
                error_text=str(exc),
                traceback_text=traceback.format_exc(),
                warnings=warnings,
            )
            return CalibrationRunResult(ok=False, warnings=warnings, diagnostics=diagnostics)
        validation_warnings, rejected = _coerce_validation_warnings(validation_result)
        warnings.extend(validation_warnings)
        if rejected:
            diagnostics = PluginDiagnostics(
                name=plugin.name,
                version=plugin.version,
                file_path=plugin.file_path,
                last_action="validate_dataset",
                error_text="validate_dataset() rejected the active dataset.",
                traceback_text="",
                warnings=warnings,
            )
            return CalibrationRunResult(ok=False, warnings=warnings, diagnostics=diagnostics)

    try:
        params = plugin.module.calibrate(dataset, config=config)
    except Exception as exc:
        diagnostics = PluginDiagnostics(
            name=plugin.name,
            version=plugin.version,
            file_path=plugin.file_path,
            last_action="calibrate",
            error_text=str(exc),
            traceback_text=traceback.format_exc(),
            warnings=warnings,
        )
        return CalibrationRunResult(ok=False, warnings=warnings, diagnostics=diagnostics)

    report = ""
    get_last_report = getattr(plugin.module, "get_last_report", None)
    if callable(get_last_report):
        try:
            report_value = get_last_report()
        except Exception as exc:
            warnings.append(f"get_last_report() failed: {exc}")
        else:
            if report_value is not None:
                report = str(report_value)

    if params is None:
        warnings.append("calibrate() returned None.")
    elif isinstance(params, dict):
        missing = [key for key in CALIBRATION_HEADER_KEYS if key not in params]
        if missing:
            warnings.append(f"calibration result missing header keys: {', '.join(missing)}")
    else:
        warnings.append(f"calibrate() returned {type(params).__name__}; dict with JSON header is preferred.")

    return CalibrationRunResult(ok=True, params=params, warnings=warnings, report=report, diagnostics=None)


def _make_action_diagnostics(
    plugin: LoadedMethodPlugin,
    *,
    last_action: str,
    error_text: str,
    traceback_text: str = "",
    warnings: list[str] | None = None,
) -> PluginDiagnostics:
    return PluginDiagnostics(
        name=plugin.name,
        version=plugin.version,
        file_path=plugin.file_path,
        last_action=last_action,
        error_text=error_text,
        traceback_text=traceback_text,
        warnings=list(warnings or plugin.effective_warnings()),
    )


def _validate_params_header(plugin: LoadedMethodPlugin, params: Any) -> tuple[list[str], PluginDiagnostics | None]:
    if not isinstance(params, dict):
        return (
            [],
            _make_action_diagnostics(
                plugin,
                last_action="validate_params_header",
                error_text=f"load_params() must return dict, got {type(params).__name__}.",
            ),
        )

    missing = [key for key in CALIBRATION_HEADER_KEYS if key not in params]
    if missing:
        return (
            [],
            _make_action_diagnostics(
                plugin,
                last_action="validate_params_header",
                error_text=f"Param profile missing required header keys: {', '.join(missing)}",
            ),
        )

    warnings: list[str] = []
    algorithm_name = str(params.get("algorithm_name", "")).strip()
    algorithm_version = str(params.get("algorithm_version", "")).strip()
    schema_version = str(params.get("schema_version", "")).strip()
    created_at = str(params.get("created_at", "")).strip()

    if not algorithm_name or not algorithm_version or not schema_version or not created_at:
        return (
            [],
            _make_action_diagnostics(
                plugin,
                last_action="validate_params_header",
                error_text="Param profile header fields must be non-empty.",
            ),
        )

    if algorithm_name != plugin.name:
        warnings.append(
            f"algorithm_name mismatch: expected '{plugin.name}', got '{algorithm_name}'"
        )
    if algorithm_version != plugin.version:
        warnings.append(
            f"algorithm_version mismatch: expected '{plugin.version}', got '{algorithm_version}'"
        )

    return (warnings, None)


def load_method_params(plugin: LoadedMethodPlugin, path: str) -> ParamIoResult:
    if plugin.module is None:
        return ParamIoResult(
            ok=False,
            diagnostics=_make_action_diagnostics(
                plugin,
                last_action="load_params",
                error_text="Plugin module is not loaded.",
            ),
        )
    if not plugin.supports("supports_load_params"):
        return ParamIoResult(
            ok=False,
            diagnostics=_make_action_diagnostics(
                plugin,
                last_action="load_params",
                error_text="Method does not support load_params.",
            ),
        )

    try:
        params = plugin.module.load_params(path)
    except Exception as exc:
        return ParamIoResult(
            ok=False,
            diagnostics=_make_action_diagnostics(
                plugin,
                last_action="load_params",
                error_text=str(exc),
                traceback_text=traceback.format_exc(),
            ),
        )

    warnings, diagnostics = _validate_params_header(plugin, params)
    if diagnostics is not None:
        return ParamIoResult(ok=False, warnings=warnings, diagnostics=diagnostics)

    return ParamIoResult(ok=True, params=params, warnings=warnings)


def save_method_params(plugin: LoadedMethodPlugin, path: str, params: Any) -> ParamIoResult:
    if plugin.module is None:
        return ParamIoResult(
            ok=False,
            diagnostics=_make_action_diagnostics(
                plugin,
                last_action="save_params",
                error_text="Plugin module is not loaded.",
            ),
        )
    if not plugin.supports("supports_save_params"):
        return ParamIoResult(
            ok=False,
            diagnostics=_make_action_diagnostics(
                plugin,
                last_action="save_params",
                error_text="Method does not support save_params.",
            ),
        )
    if params is None:
        return ParamIoResult(
            ok=False,
            diagnostics=_make_action_diagnostics(
                plugin,
                last_action="save_params",
                error_text="No calibration params are available to save.",
            ),
        )

    try:
        plugin.module.save_params(path, params)
    except Exception as exc:
        return ParamIoResult(
            ok=False,
            diagnostics=_make_action_diagnostics(
                plugin,
                last_action="save_params",
                error_text=str(exc),
                traceback_text=traceback.format_exc(),
            ),
        )

    try:
        with open(path, "r", encoding="utf-8") as fh:
            saved_payload = json.load(fh)
    except Exception as exc:
        return ParamIoResult(
            ok=False,
            diagnostics=_make_action_diagnostics(
                plugin,
                last_action="validate_saved_json",
                error_text=str(exc),
                traceback_text=traceback.format_exc(),
            ),
        )

    warnings, diagnostics = _validate_params_header(plugin, saved_payload)
    if diagnostics is not None:
        return ParamIoResult(ok=False, warnings=warnings, diagnostics=diagnostics)

    return ParamIoResult(ok=True, params=saved_payload, warnings=warnings)


def run_method_process(
    plugin: LoadedMethodPlugin,
    sample: dict[str, Any],
    params: Any | None = None,
) -> ProcessRunResult:
    runtime_params = plugin.calibration_params if params is None else params
    if plugin.module is None or not plugin.supports_process():
        return ProcessRunResult(
            ok=False,
            diagnostics=_make_action_diagnostics(
                plugin,
                last_action="process",
                error_text="Method does not support realtime processing.",
            ),
        )

    try:
        output = plugin.module.process(dict(sample), runtime_params)
    except Exception as exc:
        return ProcessRunResult(
            ok=False,
            diagnostics=_make_action_diagnostics(
                plugin,
                last_action="process",
                error_text=str(exc),
                traceback_text=traceback.format_exc(),
            ),
        )

    if not isinstance(output, dict):
        return ProcessRunResult(
            ok=False,
            diagnostics=_make_action_diagnostics(
                plugin,
                last_action="validate_process_output",
                error_text="process() must return a dict.",
            ),
        )

    return ProcessRunResult(ok=True, output=dict(output))
