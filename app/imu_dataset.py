from __future__ import annotations

import csv
import json
import os
import queue
import threading
import time
from dataclasses import dataclass, field
from typing import Any, TextIO

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
    "gyro_x",
    "gyro_y",
    "gyro_z",
    "mag_x",
    "mag_y",
    "mag_z",
    "roll_deg",
    "pitch_deg",
    "rate_mag",
    "heading",
    "flags",
)
METADATA_PREFIX = "# imu_dataset_metadata="


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
class ImuSampleRecord:
    stream_id: str
    stream_type: str
    producer_name: str
    producer_version: str
    timestamp_mcu: int
    timestamp_pc_rx: int
    timestamp_pc_est: int | None
    acc_x: float | None = None
    acc_y: float | None = None
    acc_z: float | None = None
    gyro_x: float | None = None
    gyro_y: float | None = None
    gyro_z: float | None = None
    mag_x: float | None = None
    mag_y: float | None = None
    mag_z: float | None = None
    roll_deg: float | None = None
    pitch_deg: float | None = None
    rate_mag: float | None = None
    heading: float | None = None
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
            "gyro_x": _format_scalar(self.gyro_x),
            "gyro_y": _format_scalar(self.gyro_y),
            "gyro_z": _format_scalar(self.gyro_z),
            "mag_x": _format_scalar(self.mag_x),
            "mag_y": _format_scalar(self.mag_y),
            "mag_z": _format_scalar(self.mag_z),
            "roll_deg": _format_scalar(self.roll_deg),
            "pitch_deg": _format_scalar(self.pitch_deg),
            "rate_mag": _format_scalar(self.rate_mag),
            "heading": _format_scalar(self.heading),
            "flags": self.flags,
        }

    @classmethod
    def from_csv_row(cls, row: dict[str, str]) -> ImuSampleRecord:
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
            acc_x=_parse_float(row["acc_x"]),
            acc_y=_parse_float(row["acc_y"]),
            acc_z=_parse_float(row["acc_z"]),
            gyro_x=_parse_float(row["gyro_x"]),
            gyro_y=_parse_float(row["gyro_y"]),
            gyro_z=_parse_float(row["gyro_z"]),
            mag_x=_parse_float(row["mag_x"]),
            mag_y=_parse_float(row["mag_y"]),
            mag_z=_parse_float(row["mag_z"]),
            roll_deg=_parse_float(row["roll_deg"]),
            pitch_deg=_parse_float(row["pitch_deg"]),
            rate_mag=_parse_float(row["rate_mag"]),
            heading=_parse_float(row["heading"]),
            flags=row["flags"],
        )


class ImuCsvStreamWriter:
    def __init__(
        self,
        path: str,
        *,
        metadata: dict[str, Any] | None = None,
        flush_every_rows: int = 250,
        flush_interval_s: float = 1.0,
        max_queue_rows: int = 50000,
    ) -> None:
        self.path = path
        self._flush_every_rows = max(1, int(flush_every_rows))
        self._flush_interval_s = max(0.1, float(flush_interval_s))
        self._queue: queue.Queue[ImuSampleRecord | None] = queue.Queue(maxsize=max(1, int(max_queue_rows)))
        self._pending_flush_rows = 0
        self._last_flush_s = time.monotonic()
        self._closed = False
        self._error: BaseException | None = None
        self._fh: TextIO = open(path, "w", encoding="utf-8", newline="")
        if metadata:
            self._fh.write(f"{METADATA_PREFIX}{json.dumps(metadata, ensure_ascii=False, separators=(',', ':'))}\n")
        self._writer = csv.DictWriter(self._fh, fieldnames=CSV_COLUMNS)
        self._writer.writeheader()
        self.flush()
        self._thread = threading.Thread(target=self._run, name="imu-csv-stream-writer", daemon=True)
        self._thread.start()

    def _raise_if_error(self) -> None:
        if self._error is not None:
            raise RuntimeError(f"IMU CSV stream writer failed: {self._error}") from self._error

    def append(self, record: ImuSampleRecord) -> None:
        if self._closed:
            raise RuntimeError("IMU CSV stream writer is closed")
        self._raise_if_error()
        try:
            self._queue.put(record, timeout=0.2)
        except queue.Full as exc:
            raise RuntimeError("IMU CSV stream writer queue is full") from exc

    def _run(self) -> None:
        try:
            while True:
                record = self._queue.get()
                try:
                    if record is None:
                        return
                    self._writer.writerow(record.to_csv_row())
                    self._pending_flush_rows += 1
                    now_s = time.monotonic()
                    if (
                        self._pending_flush_rows >= self._flush_every_rows
                        or (now_s - self._last_flush_s) >= self._flush_interval_s
                    ):
                        self.flush()
                finally:
                    self._queue.task_done()
        except BaseException as exc:
            self._error = exc
            self._drain_queue()

    def _drain_queue(self) -> None:
        while True:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                return
            else:
                self._queue.task_done()

    def flush(self) -> None:
        self._fh.flush()
        self._pending_flush_rows = 0
        self._last_flush_s = time.monotonic()

    def close(self) -> None:
        if self._closed:
            self._raise_if_error()
            return
        self._closed = True
        try:
            if self._thread.is_alive():
                self._queue.put(None)
                self._thread.join()
            self._raise_if_error()
            self.flush()
        finally:
            self._fh.close()


class ImuDataset:
    def __init__(
        self,
        name: str,
        *,
        records: list[ImuSampleRecord] | None = None,
        source_path: str | None = None,
        metadata: dict[str, Any] | None = None,
    ) -> None:
        self.name = name
        self.source_path = source_path
        self.records: list[ImuSampleRecord] = list(records or [])
        self.metadata: dict[str, Any] = dict(metadata or {})
        self._recompute_summary_cache()

    def _recompute_summary_cache(self) -> None:
        self._summary_source_ids: set[str] = set()
        self._summary_time_start: int | None = None
        self._summary_time_end: int | None = None
        for record in self.records:
            self._update_summary_cache_for_record(record)

    def _update_summary_cache_for_record(self, record: ImuSampleRecord) -> None:
        self._summary_source_ids.add(record.stream_id)
        timestamp_mcu = record.timestamp_mcu
        if self._summary_time_start is None or timestamp_mcu < self._summary_time_start:
            self._summary_time_start = timestamp_mcu
        if self._summary_time_end is None or timestamp_mcu > self._summary_time_end:
            self._summary_time_end = timestamp_mcu

    def has_stream(self, stream_id: str) -> bool:
        return stream_id in self._summary_source_ids

    def append(self, record: ImuSampleRecord) -> None:
        self.records.append(record)
        self._update_summary_cache_for_record(record)

    def extend(self, records: list[ImuSampleRecord]) -> None:
        self.records.extend(records)
        for record in records:
            self._update_summary_cache_for_record(record)

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
    def is_imu_csv(cls, path: str) -> bool:
        try:
            with open(path, "r", encoding="utf-8", newline="") as fh:
                for raw_line in fh:
                    if raw_line.startswith("#"):
                        if raw_line.strip().startswith(METADATA_PREFIX):
                            return True
                        continue
                    header = next(csv.reader([raw_line]), [])
                    return all(column in header for column in CSV_COLUMNS)
        except Exception:
            return False
        return False

    @classmethod
    def from_csv(cls, path: str) -> ImuDataset:
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
            records = [ImuSampleRecord.from_csv_row(row) for row in reader]
        return cls(os.path.basename(path), records=records, source_path=path, metadata=metadata)

    def summary(self) -> dict[str, str]:
        if not self.records:
            return {
                "name": os.path.basename(self.source_path) if self.source_path else self.name,
                "row_count": "0",
                "source_count": "0",
                "time_range": "—",
            }
        return {
            "name": os.path.basename(self.source_path) if self.source_path else self.name,
            "row_count": str(len(self.records)),
            "source_count": str(len(self._summary_source_ids)),
            "time_range": f"{self._summary_time_start}..{self._summary_time_end} MCU",
        }

    def _shared_projection_metadata(self) -> dict[str, Any]:
        metadata = dict(self.metadata)
        metadata["shared_source_path"] = self.source_path or ""
        metadata["shared_dataset_name"] = self.summary()["name"]
        metadata["shared_dataset_format"] = "imu_dataset"
        return metadata

    def project_magnetometer_dataset(self):
        from app.magnetometer_dataset import Dataset, SampleRecord

        records: list[SampleRecord] = []
        for record in self.records:
            has_mag = any(value is not None for value in (record.mag_x, record.mag_y, record.mag_z))
            if not has_mag and record.heading is None:
                continue
            records.append(
                SampleRecord(
                    stream_id=record.stream_id,
                    stream_type=record.stream_type,
                    producer_name=record.producer_name,
                    producer_version=record.producer_version,
                    timestamp_mcu=record.timestamp_mcu,
                    timestamp_pc_rx=record.timestamp_pc_rx,
                    timestamp_pc_est=record.timestamp_pc_est,
                    mag_x=float(record.mag_x or 0.0),
                    mag_y=float(record.mag_y or 0.0),
                    mag_z=float(record.mag_z or 0.0),
                    heading=record.heading,
                    flags=record.flags,
                )
            )
        return Dataset(
            self.name,
            records=records,
            source_path=self.source_path,
            metadata=self._shared_projection_metadata(),
        )

    def project_accelerometer_dataset(self):
        from app.accelerometer_dataset import AccelerometerDataset, AccelerometerSampleRecord

        records: list[AccelerometerSampleRecord] = []
        for record in self.records:
            has_acc = any(value is not None for value in (record.acc_x, record.acc_y, record.acc_z))
            if not has_acc and record.roll_deg is None and record.pitch_deg is None:
                continue
            records.append(
                AccelerometerSampleRecord(
                    stream_id=record.stream_id,
                    stream_type=record.stream_type,
                    producer_name=record.producer_name,
                    producer_version=record.producer_version,
                    timestamp_mcu=record.timestamp_mcu,
                    timestamp_pc_rx=record.timestamp_pc_rx,
                    timestamp_pc_est=record.timestamp_pc_est,
                    acc_x=float(record.acc_x or 0.0),
                    acc_y=float(record.acc_y or 0.0),
                    acc_z=float(record.acc_z or 0.0),
                    roll_deg=record.roll_deg,
                    pitch_deg=record.pitch_deg,
                    flags=record.flags,
                )
            )
        return AccelerometerDataset(
            self.name,
            records=records,
            source_path=self.source_path,
            metadata=self._shared_projection_metadata(),
        )

    def project_gyroscope_dataset(self):
        from app.gyroscope_dataset import GyroscopeDataset, GyroscopeSampleRecord

        records: list[GyroscopeSampleRecord] = []
        for record in self.records:
            has_gyro = any(value is not None for value in (record.gyro_x, record.gyro_y, record.gyro_z))
            if not has_gyro and record.rate_mag is None:
                continue
            records.append(
                GyroscopeSampleRecord(
                    stream_id=record.stream_id,
                    stream_type=record.stream_type,
                    producer_name=record.producer_name,
                    producer_version=record.producer_version,
                    timestamp_mcu=record.timestamp_mcu,
                    timestamp_pc_rx=record.timestamp_pc_rx,
                    timestamp_pc_est=record.timestamp_pc_est,
                    gyro_x=float(record.gyro_x or 0.0),
                    gyro_y=float(record.gyro_y or 0.0),
                    gyro_z=float(record.gyro_z or 0.0),
                    rate_mag=record.rate_mag,
                    flags=record.flags,
                )
            )
        return GyroscopeDataset(
            self.name,
            records=records,
            source_path=self.source_path,
            metadata=self._shared_projection_metadata(),
        )
