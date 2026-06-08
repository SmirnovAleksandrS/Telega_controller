# Dataset CSV format

This document describes the CSV files that the virtual controller can save and load for magnetometer, accelerometer, gyroscope, and unified IMU datasets. Use it when generating datasets in an external program so they can be loaded, displayed, and processed by this application.

## General rules

- Files are UTF-8 CSV files.
- Optional metadata can be written as comment lines before the CSV header.
- Comment lines start with `#` and are skipped by the CSV reader.
- The actual CSV header is the first non-comment line.
- All required column names must be present.
- Extra columns are tolerated by the reader, but the current dataset classes ignore them.
- Empty string means `None` only for nullable numeric fields.
- Non-empty numeric values are parsed by Python `int()` or `float()`.
- Floats are saved by the app with up to 12 significant digits, but generated files may use any Python-compatible float text such as `1`, `1.23`, `-4.5e-3`.
- Timestamps are integer milliseconds.

Recommended line order:

1. Optional metadata line.
2. CSV header.
3. Data rows.

Example:

```csv
# imu_dataset_metadata={"schema_version":"1","source":"external_generator"}
stream_id,stream_type,producer_name,producer_version,timestamp_mcu,timestamp_pc_rx,timestamp_pc_est,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z,roll_deg,pitch_deg,rate_mag,heading,flags
raw_accelerometer,raw,Raw Accelerometer,builtin,1000,1710000000000,,0.01,-0.02,0.99,,,,,,,0.58,1.16,,,
```

## Metadata

Metadata is optional. If present, it must be a single JSON object after the exact prefix for the file type.

| Dataset type | Metadata prefix |
| --- | --- |
| Unified IMU | `# imu_dataset_metadata=` |
| Magnetometer | `# magnetometer_metadata=` |
| Accelerometer | `# accelerometer_metadata=` |
| Gyroscope | `# gyroscope_metadata=` |

Example:

```csv
# magnetometer_metadata={"schema_version":"1","generator":"my_tool","notes":"lab run"}
stream_id,stream_type,producer_name,producer_version,timestamp_mcu,timestamp_pc_rx,timestamp_pc_est,mag_x,mag_y,mag_z,heading,flags
...
```

If metadata JSON is malformed, the loader ignores it and continues reading the CSV rows. Metadata is useful but not required for external generators.

## Common columns

These columns appear in all dataset formats.

| Column | Type | Required | Meaning |
| --- | --- | --- | --- |
| `stream_id` | string | yes | Logical stream identifier, for example `raw_magnetometer` or `raw_accelerometer`. |
| `stream_type` | string | yes | Stream category. Built-in raw streams use `raw`; GNSS heading uses `reference`; method outputs often use `derived`. |
| `producer_name` | string | yes | Human-readable source name shown in UI/metadata. |
| `producer_version` | string | yes | Source version. Built-in streams use `builtin`. |
| `timestamp_mcu` | int | yes | Sensor/device timestamp in MCU milliseconds. |
| `timestamp_pc_rx` | int | yes | PC receive timestamp in milliseconds. For generated offline data, use host time, simulated time, or repeat `timestamp_mcu` if no better value exists. |
| `timestamp_pc_est` | int or empty | yes | Estimated PC timestamp for the MCU event. Leave empty if unknown. |
| `flags` | string | yes | Optional flags. Use empty string for normal vector samples. |

## Unified IMU CSV

Unified IMU CSV is the most flexible format. A single file can contain magnetometer, accelerometer, gyroscope, heading, tilt, and derived streams. When loaded through any sensor tab, the app detects the unified format and projects relevant rows into that tab.

The app recognizes a file as unified IMU if either:

- it contains a metadata line starting with `# imu_dataset_metadata=`, or
- its first non-comment header contains all unified IMU columns.

### Unified columns

Header, in the app's save order:

```csv
stream_id,stream_type,producer_name,producer_version,timestamp_mcu,timestamp_pc_rx,timestamp_pc_est,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z,roll_deg,pitch_deg,rate_mag,heading,flags
```

| Column | Type | Nullable | Meaning |
| --- | --- | --- | --- |
| `acc_x`, `acc_y`, `acc_z` | float | yes | Accelerometer vector components. |
| `gyro_x`, `gyro_y`, `gyro_z` | float | yes | Gyroscope vector components. |
| `mag_x`, `mag_y`, `mag_z` | float | yes | Magnetometer vector components. |
| `roll_deg`, `pitch_deg` | float | yes | Tilt angles in degrees. |
| `rate_mag` | float | yes | Gyroscope magnitude. |
| `heading` | float | yes | Heading angle in degrees. |

For columns unrelated to a row's stream, leave cells empty.

### Projection rules

When a unified IMU CSV is loaded:

- Magnetometer projection includes rows where any `mag_x/mag_y/mag_z` is present or `heading` is present.
- Accelerometer projection includes rows where any `acc_x/acc_y/acc_z` is present or `roll_deg`/`pitch_deg` is present.
- Gyroscope projection includes rows where any `gyro_x/gyro_y/gyro_z` is present or `rate_mag` is present.
- If a projected vector component is empty, the projection fills it with `0.0`.

Because of these rules, avoid partially empty vector rows unless that is intentional. For example, a heading-only row can leave `mag_x/mag_y/mag_z` empty and provide only `heading`; the magnetometer projection will show it as a heading stream with zero vector components.

### Recommended unified stream IDs

| `stream_id` | `stream_type` | Required value columns | Recommended flags |
| --- | --- | --- | --- |
| `raw_magnetometer` | `raw` | `mag_x`, `mag_y`, `mag_z`; optionally `heading` | empty |
| `raw_heading` | `raw` | `heading`; optionally `mag_x`, `mag_y`, `mag_z` | `heading_only` |
| `gnss_heading` | `reference` | `heading` | `heading_only` |
| `raw_accelerometer` | `raw` | `acc_x`, `acc_y`, `acc_z`; optionally `roll_deg`, `pitch_deg` | empty |
| `raw_tilt` | `raw` | `roll_deg`, `pitch_deg`; optionally `acc_x`, `acc_y`, `acc_z` | `tilt_only` |
| `raw_gyroscope` | `raw` | `gyro_x`, `gyro_y`, `gyro_z`; optionally `rate_mag` | empty |
| custom derived stream | `derived` | columns matching the sensor output | usually `realtime_derived` or empty |

The program does not hard-reject other stream IDs. Custom IDs can be useful for calibrated data, for example `calibrated_magnetometer` or `derived_soft_iron`.

### Unified IMU example

This example represents one D0-like sample as five logical rows.

```csv
# imu_dataset_metadata={"schema_version":"1","generator":"external_tool"}
stream_id,stream_type,producer_name,producer_version,timestamp_mcu,timestamp_pc_rx,timestamp_pc_est,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z,roll_deg,pitch_deg,rate_mag,heading,flags
raw_magnetometer,raw,Raw Magnetometer,builtin,1000,1005,1004,,,,,,,12.3,-4.5,33.1,,,,159.9,
raw_heading,raw,Raw Heading,builtin,1000,1005,1004,,,,,,,12.3,-4.5,33.1,,,,159.9,heading_only
raw_accelerometer,raw,Raw Accelerometer,builtin,1000,1005,1004,0.01,-0.02,0.99,,,,,,,1.16,-0.58,,,
raw_tilt,raw,Raw Tilt,builtin,1000,1005,1004,0.01,-0.02,0.99,,,,,,,1.16,-0.58,,,tilt_only
raw_gyroscope,raw,Raw Gyroscope,builtin,1000,1005,1004,,,,-0.1,0.2,0.3,,,,,,0.374,, 
```

Note: the last row above has an accidental trailing space in `flags` if copied exactly. Prefer an actually empty final cell in generated files.

Cleaner gyroscope row:

```csv
raw_gyroscope,raw,Raw Gyroscope,builtin,1000,1005,1004,,,,-0.1,0.2,0.3,,,,,,0.374,,
```

## Magnetometer CSV

Magnetometer-specific files use this header:

```csv
stream_id,stream_type,producer_name,producer_version,timestamp_mcu,timestamp_pc_rx,timestamp_pc_est,mag_x,mag_y,mag_z,heading,flags
```

| Column | Type | Nullable | Meaning |
| --- | --- | --- | --- |
| `mag_x`, `mag_y`, `mag_z` | float | no | Magnetometer vector components. |
| `heading` | float | yes | Heading in degrees. |

Because `mag_x/mag_y/mag_z` are parsed as required floats in this format, heading-only rows must still provide numeric vector values. The app uses `0.0,0.0,0.0` for GNSS heading rows.

Example:

```csv
# magnetometer_metadata={"schema_version":"1","generator":"external_tool"}
stream_id,stream_type,producer_name,producer_version,timestamp_mcu,timestamp_pc_rx,timestamp_pc_est,mag_x,mag_y,mag_z,heading,flags
raw_magnetometer,raw,Raw Magnetometer,builtin,1000,1005,1004,12.3,-4.5,33.1,159.9,
raw_heading,raw,Raw Heading,builtin,1000,1005,1004,12.3,-4.5,33.1,159.9,heading_only
gnss_heading,reference,GNSS Heading,builtin,1000,1005,1004,0,0,0,160.2,heading_only
```

## Accelerometer CSV

Accelerometer-specific files use this header:

```csv
stream_id,stream_type,producer_name,producer_version,timestamp_mcu,timestamp_pc_rx,timestamp_pc_est,acc_x,acc_y,acc_z,roll_deg,pitch_deg,flags
```

| Column | Type | Nullable | Meaning |
| --- | --- | --- | --- |
| `acc_x`, `acc_y`, `acc_z` | float | no | Accelerometer vector components. |
| `roll_deg`, `pitch_deg` | float | yes | Tilt angles in degrees. |

Example:

```csv
# accelerometer_metadata={"schema_version":"1","generator":"external_tool"}
stream_id,stream_type,producer_name,producer_version,timestamp_mcu,timestamp_pc_rx,timestamp_pc_est,acc_x,acc_y,acc_z,roll_deg,pitch_deg,flags
raw_accelerometer,raw,Raw Accelerometer,builtin,1000,1005,1004,0.01,-0.02,0.99,1.16,-0.58,
raw_tilt,raw,Raw Tilt,builtin,1000,1005,1004,0.01,-0.02,0.99,1.16,-0.58,tilt_only
```

## Gyroscope CSV

Gyroscope-specific files use this header:

```csv
stream_id,stream_type,producer_name,producer_version,timestamp_mcu,timestamp_pc_rx,timestamp_pc_est,gyro_x,gyro_y,gyro_z,rate_mag,flags
```

| Column | Type | Nullable | Meaning |
| --- | --- | --- | --- |
| `gyro_x`, `gyro_y`, `gyro_z` | float | no | Gyroscope vector components. |
| `rate_mag` | float | yes | Vector magnitude. |

Example:

```csv
# gyroscope_metadata={"schema_version":"1","generator":"external_tool"}
stream_id,stream_type,producer_name,producer_version,timestamp_mcu,timestamp_pc_rx,timestamp_pc_est,gyro_x,gyro_y,gyro_z,rate_mag,flags
raw_gyroscope,raw,Raw Gyroscope,builtin,1000,1005,1004,-0.1,0.2,0.3,0.374,
```

## Handling calibrated or derived data

For externally calibrated data, use a distinct `stream_id` and `stream_type=derived`.

Examples:

- `calibrated_magnetometer`
- `hard_iron_corrected`
- `soft_iron_corrected`
- `calibrated_accelerometer`
- `calibrated_gyroscope`

For magnetometer derived vector output, fill `mag_x/mag_y/mag_z` and optionally `heading`.

For accelerometer derived output, fill `acc_x/acc_y/acc_z` and optionally `roll_deg/pitch_deg`.

For gyroscope derived output, fill `gyro_x/gyro_y/gyro_z` and optionally `rate_mag`.

Derived rows can be in sensor-specific CSV files or unified IMU CSV files. Unified IMU is recommended when a single generated file should feed all three sensor tabs.

## Practical generation checklist

1. Choose unified IMU CSV if you want one file for all sensors.
2. Use exact header names from this document.
3. Write optional metadata as one JSON comment line before the header.
4. Use integer millisecond timestamps.
5. Leave nullable numeric fields empty, not `None`, `null`, or `NaN`.
6. For sensor-specific CSV files, do not leave required vector columns empty.
7. Use stable `stream_id` values; the UI groups and routes data by `stream_id`.
8. Use `flags=heading_only` for heading-only magnetometer/reference rows.
9. Use `flags=tilt_only` for accelerometer tilt-only rows.
10. Keep rows sorted by `timestamp_mcu` unless you intentionally want a non-monotonic dataset.

## Minimal Python generator example

```python
import csv
import json
import math

path = "generated_imu.csv"
columns = [
    "stream_id", "stream_type", "producer_name", "producer_version",
    "timestamp_mcu", "timestamp_pc_rx", "timestamp_pc_est",
    "acc_x", "acc_y", "acc_z",
    "gyro_x", "gyro_y", "gyro_z",
    "mag_x", "mag_y", "mag_z",
    "roll_deg", "pitch_deg", "rate_mag", "heading", "flags",
]

def empty_row():
    return {column: "" for column in columns}

with open(path, "w", encoding="utf-8", newline="") as fh:
    metadata = {"schema_version": "1", "generator": "external_tool"}
    fh.write("# imu_dataset_metadata=" + json.dumps(metadata, ensure_ascii=False, separators=(",", ":")) + "\n")
    writer = csv.DictWriter(fh, fieldnames=columns)
    writer.writeheader()

    for i in range(1000):
        ts = i * 5
        pc = 1_700_000_000_000 + ts

        ax, ay, az = 0.01, -0.02, 0.99
        gx, gy, gz = -0.1, 0.2, 0.3
        mx, my, mz = 12.3, -4.5, 33.1
        heading = math.degrees(math.atan2(my, mx)) % 360.0
        rate_mag = math.sqrt(gx * gx + gy * gy + gz * gz)

        row = empty_row()
        row.update({
            "stream_id": "raw_magnetometer",
            "stream_type": "raw",
            "producer_name": "Raw Magnetometer",
            "producer_version": "builtin",
            "timestamp_mcu": ts,
            "timestamp_pc_rx": pc,
            "mag_x": mx,
            "mag_y": my,
            "mag_z": mz,
            "heading": heading,
        })
        writer.writerow(row)

        row = empty_row()
        row.update({
            "stream_id": "raw_accelerometer",
            "stream_type": "raw",
            "producer_name": "Raw Accelerometer",
            "producer_version": "builtin",
            "timestamp_mcu": ts,
            "timestamp_pc_rx": pc,
            "acc_x": ax,
            "acc_y": ay,
            "acc_z": az,
        })
        writer.writerow(row)

        row = empty_row()
        row.update({
            "stream_id": "raw_gyroscope",
            "stream_type": "raw",
            "producer_name": "Raw Gyroscope",
            "producer_version": "builtin",
            "timestamp_mcu": ts,
            "timestamp_pc_rx": pc,
            "gyro_x": gx,
            "gyro_y": gy,
            "gyro_z": gz,
            "rate_mag": rate_mag,
        })
        writer.writerow(row)
```

## Common errors

- Missing a required header column causes a load error.
- Writing `None`, `null`, or `NaN` into numeric cells can cause parse errors or invalid downstream math. Use empty cells for nullable values.
- Leaving vector columns empty in sensor-specific CSV formats causes parse errors because those columns are required floats.
- Using a unified IMU file without all unified columns prevents unified detection by header unless the `# imu_dataset_metadata=` line is present.
- Heading-only rows without `heading_only` still load, but they may be treated as normal vector rows in some visual previews.
- If a unified row has only `heading` and empty `mag_x/mag_y/mag_z`, magnetometer projection fills vector components with `0.0`.
