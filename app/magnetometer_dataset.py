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
    "mag_x",
    "mag_y",
    "mag_z",
    "heading",
    "flags",
)
METADATA_PREFIX = "# magnetometer_metadata="


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
class SampleRecord:
    stream_id: str
    stream_type: str
    producer_name: str
    producer_version: str
    timestamp_mcu: int
    timestamp_pc_rx: int
    timestamp_pc_est: int | None
    mag_x: float
    mag_y: float
    mag_z: float
    heading: float | None
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
            "mag_x": _format_scalar(self.mag_x),
            "mag_y": _format_scalar(self.mag_y),
            "mag_z": _format_scalar(self.mag_z),
            "heading": _format_scalar(self.heading),
            "flags": self.flags,
        }

    @classmethod
    def from_csv_row(cls, row: dict[str, str]) -> SampleRecord:
        missing = [column for column in CSV_COLUMNS if column not in row]
        if missing:
            raise ValueError(f"CSV row missing required columns: {', '.join(missing)}")

        timestamp_pc_est = _parse_int(row["timestamp_pc_est"])
        heading = _parse_float(row["heading"])
        return cls(
            stream_id=row["stream_id"],
            stream_type=row["stream_type"],
            producer_name=row["producer_name"],
            producer_version=row["producer_version"],
            timestamp_mcu=int(row["timestamp_mcu"]),
            timestamp_pc_rx=int(row["timestamp_pc_rx"]),
            timestamp_pc_est=timestamp_pc_est,
            mag_x=float(row["mag_x"]),
            mag_y=float(row["mag_y"]),
            mag_z=float(row["mag_z"]),
            heading=heading,
            flags=row["flags"],
        )


class Dataset:
    def __init__(
        self,
        name: str,
        *,
        records: list[SampleRecord] | None = None,
        source_path: str | None = None,
        metadata: dict[str, Any] | None = None,
    ) -> None:
        self.name = name
        self.source_path = source_path
        self.records: list[SampleRecord] = list(records or [])
        self.metadata: dict[str, Any] = dict(metadata or {})

    def append(self, record: SampleRecord) -> None:
        self.records.append(record)

    def extend(self, records: list[SampleRecord]) -> None:
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

    def concatenate(self, other_dataset: Dataset, *, name: str | None = None) -> Dataset:
        merged_name = name or f"{self.name}+{other_dataset.name}"
        merged_records = list(self.records)
        merged_records.extend(other_dataset.records)
        return Dataset(merged_name, records=merged_records)

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
    def from_csv(cls, path: str) -> Dataset:
        if ImuDataset.is_imu_csv(path):
            return ImuDataset.from_csv(path).project_magnetometer_dataset()
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
            records = [SampleRecord.from_csv_row(row) for row in reader]
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
