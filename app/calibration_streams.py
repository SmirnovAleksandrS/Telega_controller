from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Mapping, Sequence

CANONICAL_STREAM_KINDS = (
    "imu.accel_vector",
    "imu.gyro_vector",
    "imu.magnetometer_vector",
    "heading.deg",
    "tacho.rpm_pair",
    "motor.telemetry",
    "sensor_tensor.telemetry",
)

SCOPE_CALIBRATE = "calibrate"
SCOPE_PROCESS = "process"
SUPPORTED_SCOPES = (SCOPE_CALIBRATE, SCOPE_PROCESS)


@dataclass(slots=True)
class StreamRequirementDescriptor:
    slot: str
    kind: str
    required: bool = True
    label: str = ""
    description: str = ""
    scope: str = SCOPE_CALIBRATE
    source: str = "explicit"

    def as_dict(self) -> dict[str, Any]:
        return {
            "slot": self.slot,
            "kind": self.kind,
            "required": self.required,
            "label": self.label,
            "description": self.description,
            "scope": self.scope,
            "source": self.source,
        }


@dataclass(slots=True)
class StreamProducerDescriptor:
    producer_id: str
    kind: str
    title: str
    stream_id: str
    stream_type: str
    origin: str
    status: str = "ready"
    active: bool = True
    file_path: str | None = None
    method_id: str | None = None
    requires_calibration: bool = False
    disabled_reason: str = ""
    details: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "producer_id": self.producer_id,
            "kind": self.kind,
            "title": self.title,
            "stream_id": self.stream_id,
            "stream_type": self.stream_type,
            "origin": self.origin,
            "status": self.status,
            "active": self.active,
            "file_path": self.file_path,
            "method_id": self.method_id,
            "requires_calibration": self.requires_calibration,
            "disabled_reason": self.disabled_reason,
            "details": dict(self.details),
        }


@dataclass(slots=True)
class StreamBinding:
    scope: str
    slot: str
    producer_id: str
    requirement_kind: str

    def as_dict(self) -> dict[str, Any]:
        return {
            "scope": self.scope,
            "slot": self.slot,
            "producer_id": self.producer_id,
            "requirement_kind": self.requirement_kind,
        }


@dataclass(slots=True)
class LatestStreamSnapshot:
    producer_id: str
    kind: str
    payload: dict[str, Any]
    timestamp_mcu: int | None = None
    timestamp_pc_rx: int | None = None
    timestamp_pc_est: int | None = None

    def as_dict(self) -> dict[str, Any]:
        return {
            "producer_id": self.producer_id,
            "kind": self.kind,
            "payload": dict(self.payload),
            "timestamp_mcu": self.timestamp_mcu,
            "timestamp_pc_rx": self.timestamp_pc_rx,
            "timestamp_pc_est": self.timestamp_pc_est,
        }


@dataclass(slots=True)
class RoutingValidationResult:
    ok: bool
    issues: list[str] = field(default_factory=list)
    selected_bindings: dict[str, str] = field(default_factory=dict)
    slot_candidates: dict[str, list[dict[str, Any]]] = field(default_factory=dict)
    cycle_paths: list[str] = field(default_factory=list)

    def as_dict(self) -> dict[str, Any]:
        return {
            "ok": self.ok,
            "issues": list(self.issues),
            "selected_bindings": dict(self.selected_bindings),
            "slot_candidates": {
                slot: [dict(candidate) for candidate in candidates]
                for slot, candidates in self.slot_candidates.items()
            },
            "cycle_paths": list(self.cycle_paths),
        }


def make_builtin_producer_id(name: str) -> str:
    return f"builtin::{name}"


def make_method_producer_id(file_path: str, kind: str) -> str:
    return f"method::{file_path}::{kind}"


def producer_method_file_path(producer_id: str) -> str | None:
    parts = producer_id.split("::", 2)
    if len(parts) != 3 or parts[0] != "method":
        return None
    return parts[1]


def infer_stream_requirements(info: Mapping[str, Any]) -> dict[str, list[StreamRequirementDescriptor]]:
    explicit = info.get("stream_requirements")
    if explicit is not None:
        return _parse_explicit_stream_requirements(explicit)
    inferred = _infer_stream_requirements_from_schema(info)
    return {
        scope: list(requirements)
        for scope, requirements in inferred.items()
    }


def infer_output_stream_kinds(info: Mapping[str, Any]) -> tuple[str, ...]:
    output_schema = info.get("output_schema")
    input_schema = info.get("input_schema")
    kinds: list[str] = []
    if _schema_has_vector_kind(output_schema, "mag") or _schema_has_vector_kind(input_schema, "mag"):
        kinds.append("imu.magnetometer_vector")
    if _schema_has_vector_kind(output_schema, "acc") or _schema_has_vector_kind(input_schema, "acc"):
        kinds.append("imu.accel_vector")
    if _schema_has_vector_kind(output_schema, "accel") or _schema_has_vector_kind(input_schema, "accel"):
        kinds.append("imu.accel_vector")
    if _schema_has_vector_kind(output_schema, "gyro") or _schema_has_vector_kind(input_schema, "gyro"):
        kinds.append("imu.gyro_vector")
    if _schema_has_heading(output_schema):
        kinds.append("heading.deg")
    deduped: list[str] = []
    seen: set[str] = set()
    for kind in kinds:
        if kind in seen:
            continue
        seen.add(kind)
        deduped.append(kind)
    return tuple(deduped)


def kind_matches(requirement_kind: str, producer_kind: str) -> bool:
    return requirement_kind == producer_kind


def select_default_binding(
    requirements: Sequence[StreamRequirementDescriptor],
    producers: Mapping[str, StreamProducerDescriptor],
) -> dict[str, str]:
    selected: dict[str, str] = {}
    for requirement in requirements:
        for producer in producers.values():
            if not kind_matches(requirement.kind, producer.kind):
                continue
            if producer.origin == "builtin" and producer.active:
                selected[requirement.slot] = producer.producer_id
                break
    return selected


def build_method_dependency_graph(
    bindings_by_method: Mapping[str, Mapping[str, str]],
) -> dict[str, set[str]]:
    graph: dict[str, set[str]] = {}
    for method_key, slot_bindings in bindings_by_method.items():
        graph.setdefault(method_key, set())
        for producer_id in slot_bindings.values():
            dep = producer_method_file_path(producer_id)
            if dep:
                graph[method_key].add(dep)
    return graph


def detect_cycle_path(
    *,
    method_key: str,
    candidate_producer_id: str,
    bindings_by_method: Mapping[str, Mapping[str, str]],
) -> list[str] | None:
    upstream = producer_method_file_path(candidate_producer_id)
    if upstream is None:
        return None
    graph = build_method_dependency_graph(bindings_by_method)
    graph.setdefault(method_key, set()).add(upstream)
    visited: set[str] = set()

    def _dfs(node: str, trail: list[str]) -> list[str] | None:
        if node == method_key and trail:
            return trail + [node]
        if node in visited:
            return None
        visited.add(node)
        for nxt in graph.get(node, ()):
            result = _dfs(nxt, trail + [node])
            if result is not None:
                return result
        return None

    return _dfs(method_key, [])


def validate_binding_candidate(
    *,
    requirement: StreamRequirementDescriptor,
    candidate: StreamProducerDescriptor,
    method_key: str,
    bindings_by_method: Mapping[str, Mapping[str, str]],
) -> tuple[bool, str]:
    if not kind_matches(requirement.kind, candidate.kind):
        return (False, f"incompatible kind: expected {requirement.kind}")
    if candidate.status == "error":
        return (False, "producer is in error state")
    if candidate.requires_calibration and not candidate.active:
        return (False, "producer requires valid calibration params")
    if not candidate.active:
        return (False, "producer is inactive")
    cycle = detect_cycle_path(
        method_key=method_key,
        candidate_producer_id=candidate.producer_id,
        bindings_by_method=bindings_by_method,
    )
    if cycle:
        return (False, "cycle detected: " + " -> ".join(cycle))
    return (True, "")


def validate_method_bindings(
    *,
    method_key: str,
    requirements: Sequence[StreamRequirementDescriptor],
    selected_bindings: Mapping[str, str],
    producers: Mapping[str, StreamProducerDescriptor],
    bindings_by_method: Mapping[str, Mapping[str, str]],
) -> RoutingValidationResult:
    issues: list[str] = []
    cycle_paths: list[str] = []
    slot_candidates: dict[str, list[dict[str, Any]]] = {}
    effective = dict(selected_bindings)

    for requirement in requirements:
        candidates: list[dict[str, Any]] = []
        for producer in producers.values():
            if not kind_matches(requirement.kind, producer.kind):
                continue
            selectable, reason = validate_binding_candidate(
                requirement=requirement,
                candidate=producer,
                method_key=method_key,
                bindings_by_method=bindings_by_method,
            )
            entry = producer.as_dict()
            entry["selectable"] = selectable
            entry["disabled_reason"] = reason
            if reason.startswith("cycle detected: "):
                cycle_paths.append(reason.removeprefix("cycle detected: "))
            candidates.append(entry)
        slot_candidates[requirement.slot] = candidates

        selected = effective.get(requirement.slot, "")
        if selected and selected in producers:
            chosen = producers[selected]
            selectable, reason = validate_binding_candidate(
                requirement=requirement,
                candidate=chosen,
                method_key=method_key,
                bindings_by_method=bindings_by_method,
            )
            if not selectable:
                issues.append(f"{requirement.label or requirement.slot}: {reason}")
        elif requirement.required:
            issues.append(f"{requirement.label or requirement.slot}: no stream selected")

    deduped_issues = _dedupe_preserve_order(issues)
    deduped_cycles = _dedupe_preserve_order(cycle_paths)
    return RoutingValidationResult(
        ok=not deduped_issues,
        issues=deduped_issues,
        selected_bindings=effective,
        slot_candidates=slot_candidates,
        cycle_paths=deduped_cycles,
    )


class CalibrationStreamResolver:
    def __init__(
        self,
        *,
        producers: Mapping[str, StreamProducerDescriptor],
        bindings: Mapping[str, str],
        live_snapshots: Mapping[str, LatestStreamSnapshot] | None = None,
        dataset_records_provider: Callable[[StreamProducerDescriptor], Sequence[Any]] | None = None,
        record_adapter: Callable[[StreamProducerDescriptor, Any], dict[str, Any]] | None = None,
        latest_adapter: Callable[[LatestStreamSnapshot], dict[str, Any]] | None = None,
    ) -> None:
        self._producers = dict(producers)
        self._bindings = dict(bindings)
        self._live_snapshots = dict(live_snapshots or {})
        self._dataset_records_provider = dataset_records_provider
        self._record_adapter = record_adapter
        self._latest_adapter = latest_adapter or (lambda snapshot: dict(snapshot.payload))
        self._series_cache: dict[str, list[dict[str, Any]]] = {}

    def binding_producer(self, slot: str) -> StreamProducerDescriptor | None:
        producer_id = self._bindings.get(slot)
        if not producer_id:
            return None
        return self._producers.get(producer_id)

    def resolve_series(self, slot: str) -> list[dict[str, Any]]:
        producer = self.binding_producer(slot)
        if producer is None:
            return []
        if producer.producer_id in self._series_cache:
            return list(self._series_cache[producer.producer_id])
        if self._dataset_records_provider is None or self._record_adapter is None:
            return []
        records = self._dataset_records_provider(producer)
        series = [self._record_adapter(producer, record) for record in records]
        self._series_cache[producer.producer_id] = list(series)
        return list(series)

    def resolve_latest(self, slot: str) -> dict[str, Any] | None:
        producer = self.binding_producer(slot)
        if producer is None:
            return None
        snapshot = self._live_snapshots.get(producer.producer_id)
        if snapshot is not None:
            return self._latest_adapter(snapshot)
        series = self.resolve_series(slot)
        return dict(series[-1]) if series else None

    def export_inputs(
        self,
        requirements: Sequence[StreamRequirementDescriptor],
    ) -> dict[str, dict[str, Any]]:
        payload: dict[str, dict[str, Any]] = {}
        for requirement in requirements:
            producer = self.binding_producer(requirement.slot)
            payload[requirement.slot] = {
                "slot": requirement.slot,
                "kind": requirement.kind,
                "label": requirement.label or requirement.slot,
                "description": requirement.description,
                "producer": None if producer is None else producer.as_dict(),
                "records": self.resolve_series(requirement.slot),
                "latest": self.resolve_latest(requirement.slot),
            }
        return payload


def _parse_explicit_stream_requirements(raw: Any) -> dict[str, list[StreamRequirementDescriptor]]:
    if not isinstance(raw, Mapping):
        raise ValueError("stream_requirements must be a dict keyed by scope")
    parsed: dict[str, list[StreamRequirementDescriptor]] = {}
    for scope, items in raw.items():
        if scope not in SUPPORTED_SCOPES:
            raise ValueError(f"unsupported stream requirement scope: {scope}")
        if not isinstance(items, Sequence) or isinstance(items, (str, bytes)):
            raise ValueError(f"stream_requirements[{scope!r}] must be a list")
        parsed_scope: list[StreamRequirementDescriptor] = []
        seen_slots: set[str] = set()
        for item in items:
            if not isinstance(item, Mapping):
                raise ValueError(f"stream_requirements[{scope!r}] entries must be dicts")
            slot = str(item.get("slot", "")).strip()
            kind = str(item.get("kind", "")).strip()
            if not slot:
                raise ValueError(f"stream_requirements[{scope!r}] entry missing slot")
            if not kind:
                raise ValueError(f"stream_requirements[{scope!r}] entry {slot!r} missing kind")
            if kind not in CANONICAL_STREAM_KINDS:
                raise ValueError(f"stream_requirements[{scope!r}] entry {slot!r} uses unsupported kind {kind!r}")
            if slot in seen_slots:
                raise ValueError(f"stream_requirements[{scope!r}] has duplicate slot {slot!r}")
            seen_slots.add(slot)
            parsed_scope.append(
                StreamRequirementDescriptor(
                    slot=slot,
                    kind=kind,
                    required=bool(item.get("required", True)),
                    label=str(item.get("label", slot)).strip() or slot,
                    description=str(item.get("description", "")).strip(),
                    scope=scope,
                    source="explicit",
                )
            )
        parsed[scope] = parsed_scope
    for scope in SUPPORTED_SCOPES:
        parsed.setdefault(scope, [])
    return parsed


def _infer_stream_requirements_from_schema(info: Mapping[str, Any]) -> dict[str, list[StreamRequirementDescriptor]]:
    input_schema = info.get("input_schema")
    requirements: list[StreamRequirementDescriptor] = []
    if _schema_has_vector_kind(input_schema, "mag"):
        requirements.append(
            StreamRequirementDescriptor(
                slot="mag_input",
                kind="imu.magnetometer_vector",
                required=True,
                label="Magnetometer input",
                description="Vector used during calibration and processing",
                source="inferred",
            )
        )
    if _schema_has_vector_kind(input_schema, "accel"):
        requirements.append(
            StreamRequirementDescriptor(
                slot="accel_input",
                kind="imu.accel_vector",
                required=True,
                label="Accelerometer input",
                description="Accelerometer vector required by the method",
                source="inferred",
            )
        )
    if _schema_has_vector_kind(input_schema, "gyro"):
        requirements.append(
            StreamRequirementDescriptor(
                slot="gyro_input",
                kind="imu.gyro_vector",
                required=True,
                label="Gyroscope input",
                description="Gyroscope vector required by the method",
                source="inferred",
            )
        )
    if _schema_has_heading(input_schema):
        requirements.append(
            StreamRequirementDescriptor(
                slot="heading_input",
                kind="heading.deg",
                required=True,
                label="Heading input",
                description="Heading stream required by the method",
                source="inferred",
            )
        )
    if _schema_has_tacho(input_schema):
        requirements.append(
            StreamRequirementDescriptor(
                slot="tacho_input",
                kind="tacho.rpm_pair",
                required=True,
                label="Tacho input",
                description="Track speed pair required by the method",
                source="inferred",
            )
        )
    if _schema_has_motor(input_schema):
        requirements.append(
            StreamRequirementDescriptor(
                slot="motor_input",
                kind="motor.telemetry",
                required=True,
                label="Motor telemetry",
                description="Motor telemetry required by the method",
                source="inferred",
            )
        )
    if _schema_has_sensor_tensor(input_schema):
        requirements.append(
            StreamRequirementDescriptor(
                slot="sensor_tensor_input",
                kind="sensor_tensor.telemetry",
                required=True,
                label="Sensor tensor telemetry",
                description="Sensor tensor telemetry required by the method",
                source="inferred",
            )
        )
    calibrate_requirements = [
        StreamRequirementDescriptor(
            slot=req.slot,
            kind=req.kind,
            required=req.required,
            label=req.label,
            description=req.description,
            scope=SCOPE_CALIBRATE,
            source=req.source,
        )
        for req in requirements
    ]
    process_requirements = [
        StreamRequirementDescriptor(
            slot=req.slot,
            kind=req.kind,
            required=req.required,
            label=req.label,
            description=req.description,
            scope=SCOPE_PROCESS,
            source=req.source,
        )
        for req in requirements
    ]
    return {
        SCOPE_CALIBRATE: calibrate_requirements,
        SCOPE_PROCESS: process_requirements,
    }


def _schema_has_vector_kind(schema: Any, prefix: str) -> bool:
    if not isinstance(schema, Mapping):
        return False
    keys = {str(key).strip().lower() for key in schema}
    if prefix in keys:
        return True
    expected = {f"{prefix}_x", f"{prefix}_y", f"{prefix}_z"}
    return expected.issubset(keys)


def _schema_has_heading(schema: Any) -> bool:
    if not isinstance(schema, Mapping):
        return False
    keys = {str(key).strip().lower() for key in schema}
    return "heading" in keys or "heading_deg" in keys


def _schema_has_tacho(schema: Any) -> bool:
    if not isinstance(schema, Mapping):
        return False
    keys = {str(key).strip().lower() for key in schema}
    return "tacho" in keys or {"left_rpm", "right_rpm"}.issubset(keys)


def _schema_has_motor(schema: Any) -> bool:
    if not isinstance(schema, Mapping):
        return False
    keys = {str(key).strip().lower() for key in schema}
    motor_keys = {"current_l", "current_r", "voltage_l", "voltage_r", "temp_l", "temp_r"}
    return "motor" in keys or motor_keys.issubset(keys)


def _schema_has_sensor_tensor(schema: Any) -> bool:
    if not isinstance(schema, Mapping):
        return False
    keys = {str(key).strip().lower() for key in schema}
    tensor_keys = {
        "linear_velocity",
        "angular_velocity",
        "linear_quality",
        "angular_quality",
    }
    return "sensor_tensor" in keys or tensor_keys.issubset(keys)


def _dedupe_preserve_order(items: Sequence[str]) -> list[str]:
    deduped: list[str] = []
    seen: set[str] = set()
    for item in items:
        value = str(item).strip()
        if not value or value in seen:
            continue
        seen.add(value)
        deduped.append(value)
    return deduped
