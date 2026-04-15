from __future__ import annotations

import csv
import json
import os
from dataclasses import dataclass, field
from typing import Any

from app.imu_dataset import ImuDataset

CSV_COLUMNS = (
    "stream_id",
    "stream_type",
    "producer_name",
    "producer_version",
    "timestamp_mcu",
    "timestamp_pc_rx",
    "timestamp_pc_est",
    "acc_x",
    "acc_y",
    "acc_z",
    "roll_deg",
    "pitch_deg",
    "flags",
)
METADATA_PREFIX = "# accelerometer_metadata="


def _format_scalar(value: object) -> str:
    if value is None:
        return ""
    if isinstance(value, float):
        return format(value, ".12g")
    return str(value)


def _parse_int(value: str) -> int | None:
    if value == "":
        return None
    return int(value)


def _parse_float(value: str) -> float | None:
    if value == "":
        return None
    return float(value)


@dataclass(slots=True)
class AccelerometerSampleRecord:
    stream_id: str
    stream_type: str
    producer_name: str
    producer_version: str
    timestamp_mcu: int
    timestamp_pc_rx: int
    timestamp_pc_est: int | None
    acc_x: float
    acc_y: float
    acc_z: float
    roll_deg: float | None
    pitch_deg: float | None
    flags: str = ""
    extra: dict[str, Any] = field(default_factory=dict)

    def to_csv_row(self) -> dict[str, str]:
        return {
            "stream_id": self.stream_id,
            "stream_type": self.stream_type,
            "producer_name": self.producer_name,
            "producer_version": self.producer_version,
            "timestamp_mcu": _format_scalar(self.timestamp_mcu),
            "timestamp_pc_rx": _format_scalar(self.timestamp_pc_rx),
            "timestamp_pc_est": _format_scalar(self.timestamp_pc_est),
            "acc_x": _format_scalar(self.acc_x),
            "acc_y": _format_scalar(self.acc_y),
            "acc_z": _format_scalar(self.acc_z),
            "roll_deg": _format_scalar(self.roll_deg),
            "pitch_deg": _format_scalar(self.pitch_deg),
            "flags": self.flags,
        }

    @classmethod
    def from_csv_row(cls, row: dict[str, str]) -> AccelerometerSampleRecord:
        missing = [column for column in CSV_COLUMNS if column not in row]
        if missing:
            raise ValueError(f"CSV row missing required columns: {', '.join(missing)}")

        return cls(
            stream_id=row["stream_id"],
            stream_type=row["stream_type"],
            producer_name=row["producer_name"],
            producer_version=row["producer_version"],
            timestamp_mcu=int(row["timestamp_mcu"]),
            timestamp_pc_rx=int(row["timestamp_pc_rx"]),
            timestamp_pc_est=_parse_int(row["timestamp_pc_est"]),
            acc_x=float(row["acc_x"]),
            acc_y=float(row["acc_y"]),
            acc_z=float(row["acc_z"]),
            roll_deg=_parse_float(row["roll_deg"]),
            pitch_deg=_parse_float(row["pitch_deg"]),
            flags=row["flags"],
        )


class AccelerometerDataset:
    def __init__(
        self,
        name: str,
        *,
        records: list[AccelerometerSampleRecord] | None = None,
        source_path: str | None = None,
        metadata: dict[str, Any] | None = None,
    ) -> None:
        self.name = name
        self.source_path = source_path
        self.records: list[AccelerometerSampleRecord] = list(records or [])
        self.metadata: dict[str, Any] = dict(metadata or {})

    def append(self, record: AccelerometerSampleRecord) -> None:
        self.records.append(record)

    def extend(self, records: list[AccelerometerSampleRecord]) -> None:
        self.records.extend(records)

    def trim(self, start_idx: int, end_idx: int) -> None:
        if not self.records:
            return
        first = max(0, min(start_idx, end_idx))
        last = min(len(self.records) - 1, max(start_idx, end_idx))
        self.records = self.records[first:last + 1]

    def delete_rows(self, indices: list[int]) -> None:
        if not indices:
            return
        index_set = {index for index in indices if 0 <= index < len(self.records)}
        self.records = [record for idx, record in enumerate(self.records) if idx not in index_set]

    def concatenate(self, other_dataset: AccelerometerDataset, *, name: str | None = None) -> AccelerometerDataset:
        merged_name = name or f"{self.name}+{other_dataset.name}"
        merged_records = list(self.records)
        merged_records.extend(other_dataset.records)
        return AccelerometerDataset(merged_name, records=merged_records)

    def to_csv(self, path: str) -> None:
        with open(path, "w", encoding="utf-8", newline="") as fh:
            if self.metadata:
                fh.write(f"{METADATA_PREFIX}{json.dumps(self.metadata, ensure_ascii=False, separators=(',', ':'))}\n")
            writer = csv.DictWriter(fh, fieldnames=CSV_COLUMNS)
            writer.writeheader()
            for record in self.records:
                writer.writerow(record.to_csv_row())
        self.source_path = path

    @classmethod
    def from_csv(cls, path: str) -> AccelerometerDataset:
        if ImuDataset.is_imu_csv(path):
            return ImuDataset.from_csv(path).project_accelerometer_dataset()
        metadata: dict[str, Any] = {}
        with open(path, "r", encoding="utf-8", newline="") as fh:
            data_lines: list[str] = []
            for raw_line in fh:
                if raw_line.startswith("#"):
                    stripped = raw_line.strip()
                    if stripped.startswith(METADATA_PREFIX):
                        payload = stripped[len(METADATA_PREFIX):].strip()
                        try:
                            loaded_metadata = json.loads(payload)
                        except Exception:
                            loaded_metadata = {}
                        if isinstance(loaded_metadata, dict):
                            metadata.update(loaded_metadata)
                    continue
                data_lines.append(raw_line)
            reader = csv.DictReader(data_lines)
            fieldnames = reader.fieldnames or []
            missing = [column for column in CSV_COLUMNS if column not in fieldnames]
            if missing:
                raise ValueError(f"CSV missing required columns: {', '.join(missing)}")
            records = [AccelerometerSampleRecord.from_csv_row(row) for row in reader]
        return cls(os.path.basename(path), records=records, source_path=path, metadata=metadata)

    def summary(self) -> dict[str, str]:
        if not self.records:
            return {
                "name": os.path.basename(self.source_path) if self.source_path else self.name,
                "row_count": "0",
                "source_count": "0",
                "time_range": "—",
            }

        time_start = min(record.timestamp_mcu for record in self.records)
        time_end = max(record.timestamp_mcu for record in self.records)
        source_ids = {record.stream_id for record in self.records}
        return {
            "name": os.path.basename(self.source_path) if self.source_path else self.name,
            "row_count": str(len(self.records)),
            "source_count": str(len(source_ids)),
            "time_range": f"{time_start}..{time_end} MCU",
        }
