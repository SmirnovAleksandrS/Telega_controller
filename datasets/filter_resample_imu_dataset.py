#!/usr/bin/env python3
"""
Filter and synchronize IMU / magnetometer / GNSS-heading CSV datasets.

Designed for the long CSV format used in the project:
    stream_id, stream_type, timestamp_mcu, timestamp_pc_rx, ...,
    acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z,
    mag_x, mag_y, mag_z, roll_deg, pitch_deg, rate_mag, heading, flags

Main idea:
1. Read all streams from one CSV.
2. Estimate the real GNSS-compass frequency from `gnss_heading` timestamps.
3. For higher-rate streams, apply:
   - short median filter, usually 3 or 5 samples;
   - centered moving-average low-pass filter with window approximately fs_src / fs_target.
4. Resample/interpolate all selected streams to the GNSS time grid or to a uniform grid
   with the estimated GNSS frequency.
5. Save a synchronized wide CSV convenient for further calibration/heading comparison.

Example:
    python filter_resample_imu_dataset.py input.csv -o synced.csv \
        --time-col timestamp_mcu --median-window 5 --grid gnss

    python filter_resample_imu_dataset.py input.csv -o synced_uniform.csv \
        --time-col timestamp_mcu --median-window 3 --grid uniform
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import pandas as pd


DEFAULT_VALUE_COLS = [
    "acc_x", "acc_y", "acc_z",
    "gyro_x", "gyro_y", "gyro_z",
    "mag_x", "mag_y", "mag_z",
    "roll_deg", "pitch_deg", "rate_mag", "heading",
]

ANGLE_COLS_DEG = {"heading"}


@dataclass
class StreamInfo:
    stream_id: str
    n_samples: int
    n_unique_timestamps: int
    median_dt_raw: float
    median_dt_s: float
    fs_hz: float


def read_project_csv(path: str | Path) -> tuple[Optional[str], pd.DataFrame]:
    """Read project CSV and preserve the first metadata line if present."""
    path = Path(path)
    metadata_line: Optional[str] = None
    with path.open("r", encoding="utf-8", errors="replace") as f:
        first = f.readline()
        if first.startswith("#"):
            metadata_line = first.rstrip("\n")

    df = pd.read_csv(path, comment="#")
    return metadata_line, df


def infer_time_scale_to_seconds(t: np.ndarray, forced_unit: str = "auto") -> float:
    """
    Return multiplier converting timestamp values to seconds.

    Heuristic for auto:
    - unix-ish nanoseconds/microseconds/milliseconds are detected by value magnitude;
    - small MCU timestamps in this project usually are milliseconds, because dt~25 means 40 Hz.
    """
    forced_unit = forced_unit.lower()
    units = {
        "s": 1.0,
        "sec": 1.0,
        "second": 1.0,
        "seconds": 1.0,
        "ms": 1e-3,
        "millisecond": 1e-3,
        "milliseconds": 1e-3,
        "us": 1e-6,
        "microsecond": 1e-6,
        "microseconds": 1e-6,
        "ns": 1e-9,
        "nanosecond": 1e-9,
        "nanoseconds": 1e-9,
    }
    if forced_unit != "auto":
        if forced_unit not in units:
            raise ValueError(f"Unknown time unit: {forced_unit}")
        return units[forced_unit]

    t = np.asarray(t, dtype=float)
    t = t[np.isfinite(t)]
    if t.size < 2:
        return 1e-3

    typical_abs = np.nanmedian(np.abs(t))
    dt = np.diff(np.sort(np.unique(t)))
    dt = dt[dt > 0]
    typical_dt = np.nanmedian(dt) if dt.size else np.nan

    # Absolute Unix-like timestamp scale.
    if typical_abs > 1e17:
        return 1e-9
    if typical_abs > 1e14:
        return 1e-6
    if typical_abs > 1e11:
        return 1e-3

    # Relative MCU-like timestamp scale. In the current logger timestamp_mcu is ms.
    # dt around 20-30 should mean 20-30 ms, not 20-30 seconds.
    if np.isfinite(typical_dt):
        if typical_dt >= 1.0:
            return 1e-3
        return 1.0

    return 1e-3


def estimate_stream_frequency(
    df: pd.DataFrame,
    stream_id: str,
    time_col: str,
    time_scale: float,
) -> StreamInfo:
    sdf = df[df["stream_id"] == stream_id].copy()
    t_raw = pd.to_numeric(sdf[time_col], errors="coerce").dropna().to_numpy(dtype=float)
    t_unique = np.sort(np.unique(t_raw))
    dt_raw = np.diff(t_unique)
    dt_raw = dt_raw[dt_raw > 0]
    median_dt_raw = float(np.nanmedian(dt_raw)) if dt_raw.size else float("nan")
    median_dt_s = median_dt_raw * time_scale if np.isfinite(median_dt_raw) else float("nan")
    fs_hz = 1.0 / median_dt_s if median_dt_s > 0 else float("nan")
    return StreamInfo(
        stream_id=stream_id,
        n_samples=len(sdf),
        n_unique_timestamps=len(t_unique),
        median_dt_raw=median_dt_raw,
        median_dt_s=median_dt_s,
        fs_hz=fs_hz,
    )


def print_frequency_report(infos: Sequence[StreamInfo]) -> None:
    print("\nEstimated stream frequencies")
    print("-" * 86)
    print(f"{'stream_id':24s} {'samples':>9s} {'unique_t':>9s} {'median_dt':>12s} {'fs_hz':>12s}")
    for info in infos:
        print(
            f"{info.stream_id:24s} {info.n_samples:9d} {info.n_unique_timestamps:9d} "
            f"{info.median_dt_raw:12.6g} {info.fs_hz:12.6g}"
        )
    print("-" * 86)


def normalize_time_column(df: pd.DataFrame, time_col: str, time_scale: float) -> pd.Series:
    t = pd.to_numeric(df[time_col], errors="coerce").astype(float) * time_scale
    return t


def angle_to_unwrapped_deg(series: pd.Series) -> pd.Series:
    values = pd.to_numeric(series, errors="coerce").to_numpy(dtype=float)
    mask = np.isfinite(values)
    out = np.full_like(values, np.nan, dtype=float)
    if mask.sum() >= 2:
        out[mask] = np.rad2deg(np.unwrap(np.deg2rad(values[mask])))
    elif mask.sum() == 1:
        out[mask] = values[mask]
    return pd.Series(out, index=series.index)


def wrap_deg(angle: np.ndarray | pd.Series) -> np.ndarray:
    a = np.asarray(angle, dtype=float)
    return (a + 180.0) % 360.0 - 180.0


def prepare_stream_table(
    df: pd.DataFrame,
    stream_id: str,
    time_col: str,
    time_scale: float,
    value_cols: Sequence[str],
) -> pd.DataFrame:
    """Return one row per timestamp for one stream, duplicate timestamps averaged."""
    sdf = df[df["stream_id"] == stream_id].copy()
    if sdf.empty:
        return pd.DataFrame(columns=["t_s", *value_cols])

    sdf["t_s"] = normalize_time_column(sdf, time_col, time_scale)
    sdf = sdf.dropna(subset=["t_s"]).sort_values("t_s")

    present_cols = [c for c in value_cols if c in sdf.columns and sdf[c].notna().any()]
    if not present_cols:
        return pd.DataFrame(columns=["t_s"])

    for c in present_cols:
        if c in ANGLE_COLS_DEG:
            sdf[c] = angle_to_unwrapped_deg(sdf[c])
        else:
            sdf[c] = pd.to_numeric(sdf[c], errors="coerce")

    # Duplicates can exist when several packets have the same PC receive time.
    # timestamp_mcu usually does not duplicate, but grouping is harmless.
    sdf = sdf[["t_s", *present_cols]].groupby("t_s", as_index=False).mean(numeric_only=True)
    return sdf


def rolling_filter_stream(
    sdf: pd.DataFrame,
    fs_src: float,
    fs_target: float,
    median_window: int = 5,
    mean_window: Optional[int] = None,
    disable_mean: bool = False,
) -> tuple[pd.DataFrame, int]:
    """Apply median + moving-average low-pass filters to stream columns."""
    if sdf.empty or len(sdf) < 2:
        return sdf, 1

    filtered = sdf.copy()
    cols = [c for c in filtered.columns if c != "t_s"]

    if median_window and median_window > 1:
        # rolling median should have an odd window. If even, make it odd upwards.
        if median_window % 2 == 0:
            median_window += 1
        filtered[cols] = (
            filtered[cols]
            .rolling(window=median_window, center=True, min_periods=1)
            .median()
        )

    if disable_mean:
        return filtered, 1

    if mean_window is None or mean_window <= 0:
        if np.isfinite(fs_src) and np.isfinite(fs_target) and fs_target > 0:
            mean_window = max(1, int(round(fs_src / fs_target)))
        else:
            mean_window = 1

    # Keep at least 1; too large windows smear dynamics heavily.
    mean_window = int(max(1, mean_window))
    if mean_window > 1:
        filtered[cols] = (
            filtered[cols]
            .rolling(window=mean_window, center=True, min_periods=1)
            .mean()
        )

    return filtered, mean_window


def interpolate_stream_to_grid(
    sdf: pd.DataFrame,
    grid_t: np.ndarray,
    prefix: str,
) -> pd.DataFrame:
    """Interpolate filtered stream values to target time grid."""
    result = pd.DataFrame({"t_s": grid_t})
    if sdf.empty or len(sdf) < 1:
        return result

    t = sdf["t_s"].to_numpy(dtype=float)
    order = np.argsort(t)
    t = t[order]
    for col in [c for c in sdf.columns if c != "t_s"]:
        y = sdf[col].to_numpy(dtype=float)[order]
        mask = np.isfinite(t) & np.isfinite(y)
        if mask.sum() == 0:
            result[f"{prefix}_{col}"] = np.nan
        elif mask.sum() == 1:
            arr = np.full_like(grid_t, np.nan, dtype=float)
            arr[np.argmin(np.abs(grid_t - t[mask][0]))] = y[mask][0]
            result[f"{prefix}_{col}"] = arr
        else:
            result[f"{prefix}_{col}"] = np.interp(grid_t, t[mask], y[mask], left=np.nan, right=np.nan)

    # Wrap heading-like angle after interpolation.
    if f"{prefix}_heading" in result.columns:
        result[f"{prefix}_heading_wrapped_deg"] = wrap_deg(result[f"{prefix}_heading"].to_numpy())

    return result


def make_target_grid(
    target_sdf: pd.DataFrame,
    target_fs_hz: float,
    grid_mode: str = "gnss",
) -> np.ndarray:
    t = target_sdf["t_s"].dropna().to_numpy(dtype=float)
    t = np.sort(np.unique(t))
    if len(t) < 2:
        raise ValueError("Target stream has fewer than 2 unique timestamps")

    if grid_mode == "gnss":
        return t
    if grid_mode == "uniform":
        dt = 1.0 / target_fs_hz
        return np.arange(t[0], t[-1] + 0.5 * dt, dt)
    raise ValueError(f"Unknown grid mode: {grid_mode}")


def synchronize_dataset(
    df: pd.DataFrame,
    time_col: str = "timestamp_mcu",
    time_unit: str = "auto",
    target_stream: str = "gnss_heading",
    streams: Sequence[str] = ("raw_magnetometer", "raw_accelerometer", "raw_gyroscope", "gnss_heading"),
    value_cols: Sequence[str] = DEFAULT_VALUE_COLS,
    median_window: int = 5,
    mean_window: Optional[int] = None,
    grid_mode: str = "gnss",
    target_fs_hz: Optional[float] = None,
    no_mean_filter: bool = False,
) -> tuple[pd.DataFrame, Dict]:
    if time_col not in df.columns:
        raise ValueError(f"No time column {time_col!r}. Available columns: {list(df.columns)}")

    all_t = pd.to_numeric(df[time_col], errors="coerce").dropna().to_numpy(dtype=float)
    time_scale = infer_time_scale_to_seconds(all_t, forced_unit=time_unit)

    existing_streams = [s for s in streams if s in set(df["stream_id"].dropna().astype(str))]
    if target_stream not in existing_streams:
        raise ValueError(f"Target stream {target_stream!r} not found in dataset")

    infos = [estimate_stream_frequency(df, s, time_col, time_scale) for s in existing_streams]
    info_by_stream = {info.stream_id: info for info in infos}
    target_info = info_by_stream[target_stream]
    if target_fs_hz is None:
        target_fs_hz = target_info.fs_hz
    if not np.isfinite(target_fs_hz) or target_fs_hz <= 0:
        raise ValueError("Could not estimate target frequency. Pass --target-fs manually.")

    prepared: Dict[str, pd.DataFrame] = {}
    filtered: Dict[str, pd.DataFrame] = {}
    used_mean_windows: Dict[str, int] = {}

    for stream in existing_streams:
        sdf = prepare_stream_table(df, stream, time_col, time_scale, value_cols)
        prepared[stream] = sdf
        fs_src = info_by_stream[stream].fs_hz

        # Do not low-pass target stream by default as a high-rate source. It may still get median-filtered.
        filt, used_w = rolling_filter_stream(
            sdf,
            fs_src=fs_src,
            fs_target=target_fs_hz,
            median_window=median_window,
            mean_window=(1 if stream == target_stream and mean_window is None else mean_window),
            disable_mean=no_mean_filter,
        )
        filtered[stream] = filt
        used_mean_windows[stream] = used_w

    grid_t = make_target_grid(filtered[target_stream], target_fs_hz=target_fs_hz, grid_mode=grid_mode)
    out = pd.DataFrame({"t_s": grid_t})

    # Add relative time for plotting.
    out["t_rel_s"] = out["t_s"] - out["t_s"].iloc[0]

    for stream in existing_streams:
        interp = interpolate_stream_to_grid(filtered[stream], grid_t, prefix=stream)
        cols_to_add = [c for c in interp.columns if c != "t_s"]
        out = pd.concat([out, interp[cols_to_add]], axis=1)

    report = {
        "time_col": time_col,
        "time_unit_multiplier_to_seconds": time_scale,
        "target_stream": target_stream,
        "target_fs_hz": target_fs_hz,
        "grid_mode": grid_mode,
        "median_window": median_window,
        "requested_mean_window": mean_window,
        "used_mean_windows": used_mean_windows,
        "streams": [info.__dict__ for info in infos],
        "n_output_rows": int(len(out)),
    }
    return out, report


def main() -> None:
    parser = argparse.ArgumentParser(description="Filter and synchronize IMU / GNSS heading dataset")
    parser.add_argument("input_csv", type=str, help="Input project CSV")
    parser.add_argument("-o", "--output", type=str, default=None, help="Output synchronized wide CSV")
    parser.add_argument("--report", type=str, default=None, help="Optional JSON report path")
    parser.add_argument("--time-col", type=str, default="timestamp_mcu", help="Timestamp column")
    parser.add_argument("--time-unit", type=str, default="auto", help="auto, s, ms, us, ns")
    parser.add_argument("--target-stream", type=str, default="gnss_heading", help="Stream whose frequency/grid is target")
    parser.add_argument("--target-fs", type=float, default=None, help="Manual target frequency, Hz")
    parser.add_argument("--grid", choices=["gnss", "uniform"], default="gnss", help="Use real GNSS timestamps or uniform target grid")
    parser.add_argument("--median-window", type=int, default=5, help="Median filter window in samples; use 3 or 5")
    parser.add_argument("--mean-window", type=int, default=None, help="Moving average window in source samples; default auto=fs_src/fs_target")
    parser.add_argument("--no-mean-filter", action="store_true", help="Disable moving average low-pass filter")
    parser.add_argument(
        "--streams",
        type=str,
        default="raw_magnetometer,raw_accelerometer,raw_gyroscope,gnss_heading",
        help="Comma-separated stream ids to synchronize",
    )
    args = parser.parse_args()

    metadata, df = read_project_csv(args.input_csv)
    streams = [s.strip() for s in args.streams.split(",") if s.strip()]

    # Frequency report before processing.
    all_t = pd.to_numeric(df[args.time_col], errors="coerce").dropna().to_numpy(dtype=float)
    time_scale = infer_time_scale_to_seconds(all_t, forced_unit=args.time_unit)
    present_streams = [s for s in streams if s in set(df["stream_id"].dropna().astype(str))]
    infos = [estimate_stream_frequency(df, s, args.time_col, time_scale) for s in present_streams]
    print_frequency_report(infos)

    out, report = synchronize_dataset(
        df,
        time_col=args.time_col,
        time_unit=args.time_unit,
        target_stream=args.target_stream,
        streams=streams,
        median_window=args.median_window,
        mean_window=args.mean_window,
        grid_mode=args.grid,
        target_fs_hz=args.target_fs,
        no_mean_filter=args.no_mean_filter,
    )

    input_path = Path(args.input_csv)
    output_path = Path(args.output) if args.output else input_path.with_name(input_path.stem + "_filtered_synced.csv")
    report_path = Path(args.report) if args.report else output_path.with_suffix(".report.json")

    out.to_csv(output_path, index=False)
    with report_path.open("w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=2)

    print(f"\nSaved synchronized CSV: {output_path}")
    print(f"Saved report:           {report_path}")
    print(f"Output rows:            {len(out)}")
    print(f"Target fs, Hz:          {report['target_fs_hz']:.6g}")
    print(f"Used mean windows:      {report['used_mean_windows']}")


if __name__ == "__main__":
    main()
