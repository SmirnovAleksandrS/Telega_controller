from __future__ import annotations

import json
import math
from datetime import datetime, timezone


METHOD_NAME = "Simple Hard Ironing Method"
METHOD_VERSION = "1.0.0"
SCHEMA_VERSION = "1"

_last_report = ""


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")


def get_info() -> dict:
    return {
        "name": METHOD_NAME,
        "version": METHOD_VERSION,
        "type": "method",
        "supports_calibrate": True,
        "supports_load_params": True,
        "supports_save_params": True,
        "supports_process": True,
        "input_schema": {
            "timestamp_mcu": "int",
            "timestamp_pc_rx": "int",
            "timestamp_pc_est": "int|None",
            "mag_x": "float",
            "mag_y": "float",
            "mag_z": "float",
            "heading": "float|None",
            "flags": "str",
        },
        "output_schema": {
            "timestamp_mcu": "int",
            "timestamp_pc_rx": "int",
            "timestamp_pc_est": "int|None",
            "mag_x": "float",
            "mag_y": "float",
            "mag_z": "float",
            "heading": "float|None",
            "flags": "str",
        },
    }


def validate_dataset(dataset):
    if dataset is None or not hasattr(dataset, "records"):
        return {"ok": False, "warnings": ["dataset object is missing records"]}

    raw_records = [record for record in dataset.records if getattr(record, "stream_id", "") == "raw_magnetometer"]
    if len(raw_records) < 8:
        return {"ok": False, "warnings": ["need at least 8 raw_magnetometer samples"]}

    return None


def _vector_add(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _vector_sub(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _vector_scale(a: tuple[float, float, float], scalar: float) -> tuple[float, float, float]:
    return (a[0] * scalar, a[1] * scalar, a[2] * scalar)


def _dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _norm(a: tuple[float, float, float]) -> float:
    return math.sqrt(_dot(a, a))


def _normalize(a: tuple[float, float, float]) -> tuple[float, float, float]:
    length = _norm(a)
    if length <= 1e-12:
        raise ValueError("cannot normalize zero-length vector")
    return (a[0] / length, a[1] / length, a[2] / length)


def _mean_point(points: list[tuple[float, float, float]]) -> tuple[float, float, float]:
    inv = 1.0 / float(len(points))
    sx = 0.0
    sy = 0.0
    sz = 0.0
    for px, py, pz in points:
        sx += px
        sy += py
        sz += pz
    return (sx * inv, sy * inv, sz * inv)


def _covariance_matrix(points: list[tuple[float, float, float]], center: tuple[float, float, float]) -> list[list[float]]:
    xx = xy = xz = yy = yz = zz = 0.0
    for point in points:
        dx, dy, dz = _vector_sub(point, center)
        xx += dx * dx
        xy += dx * dy
        xz += dx * dz
        yy += dy * dy
        yz += dy * dz
        zz += dz * dz
    inv = 1.0 / float(len(points))
    return [
        [xx * inv, xy * inv, xz * inv],
        [xy * inv, yy * inv, yz * inv],
        [xz * inv, yz * inv, zz * inv],
    ]


def _identity_matrix(n: int) -> list[list[float]]:
    return [[1.0 if row == col else 0.0 for col in range(n)] for row in range(n)]


def _jacobi_eigen_decomposition(matrix: list[list[float]]) -> tuple[list[float], list[list[float]]]:
    n = len(matrix)
    a = [[float(matrix[row][col]) for col in range(n)] for row in range(n)]
    v = _identity_matrix(n)

    for _ in range(40):
        p = 0
        q = 1
        max_off = 0.0
        for row in range(n):
            for col in range(row + 1, n):
                value = abs(a[row][col])
                if value > max_off:
                    max_off = value
                    p = row
                    q = col
        if max_off <= 1e-12:
            break

        app = a[p][p]
        aqq = a[q][q]
        apq = a[p][q]
        tau = (aqq - app) / (2.0 * apq)
        t = math.copysign(1.0 / (abs(tau) + math.sqrt(1.0 + tau * tau)), tau)
        c = 1.0 / math.sqrt(1.0 + t * t)
        s = t * c

        for idx in range(n):
            if idx == p or idx == q:
                continue
            aip = a[idx][p]
            aiq = a[idx][q]
            a[idx][p] = c * aip - s * aiq
            a[p][idx] = a[idx][p]
            a[idx][q] = s * aip + c * aiq
            a[q][idx] = a[idx][q]

        a[p][p] = c * c * app - 2.0 * s * c * apq + s * s * aqq
        a[q][q] = s * s * app + 2.0 * s * c * apq + c * c * aqq
        a[p][q] = 0.0
        a[q][p] = 0.0

        for idx in range(n):
            vip = v[idx][p]
            viq = v[idx][q]
            v[idx][p] = c * vip - s * viq
            v[idx][q] = s * vip + c * viq

    eigenvalues = [a[idx][idx] for idx in range(n)]
    eigenvectors = [[v[row][col] for row in range(n)] for col in range(n)]
    return eigenvalues, eigenvectors


def _plane_basis(points: list[tuple[float, float, float]]) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    center = _mean_point(points)
    covariance = _covariance_matrix(points, center)
    eigenvalues, eigenvectors = _jacobi_eigen_decomposition(covariance)
    min_index = min(range(3), key=lambda idx: eigenvalues[idx])
    normal = _normalize(tuple(eigenvectors[min_index]))

    ref = (1.0, 0.0, 0.0) if abs(normal[0]) < 0.9 else (0.0, 1.0, 0.0)
    axis_u = _cross(normal, ref)
    if _norm(axis_u) <= 1e-9:
        ref = (0.0, 0.0, 1.0)
        axis_u = _cross(normal, ref)
    axis_u = _normalize(axis_u)
    axis_v = _normalize(_cross(normal, axis_u))
    return center, axis_u, axis_v


def _solve_linear_system(matrix: list[list[float]], rhs: list[float]) -> list[float]:
    n = len(rhs)
    aug = [list(matrix[row]) + [rhs[row]] for row in range(n)]

    for col in range(n):
        pivot_row = max(range(col, n), key=lambda row: abs(aug[row][col]))
        pivot = aug[pivot_row][col]
        if abs(pivot) <= 1e-12:
            raise ValueError("singular linear system")
        if pivot_row != col:
            aug[col], aug[pivot_row] = aug[pivot_row], aug[col]

        pivot = aug[col][col]
        for idx in range(col, n + 1):
            aug[col][idx] /= pivot

        for row in range(n):
            if row == col:
                continue
            factor = aug[row][col]
            if abs(factor) <= 1e-12:
                continue
            for idx in range(col, n + 1):
                aug[row][idx] -= factor * aug[col][idx]

    return [aug[row][n] for row in range(n)]


def _project_points_to_plane(
    points: list[tuple[float, float, float]],
    center: tuple[float, float, float],
    axis_u: tuple[float, float, float],
    axis_v: tuple[float, float, float],
) -> list[tuple[float, float]]:
    projected: list[tuple[float, float]] = []
    for point in points:
        delta = _vector_sub(point, center)
        projected.append((_dot(delta, axis_u), _dot(delta, axis_v)))
    return projected


def _fit_ellipse_center(projected_points: list[tuple[float, float]]) -> tuple[float, float]:
    normal = [[0.0 for _ in range(5)] for _ in range(5)]
    rhs = [0.0 for _ in range(5)]

    for x_val, y_val in projected_points:
        row = [x_val * x_val, x_val * y_val, y_val * y_val, x_val, y_val]
        for row_idx in range(5):
            rhs[row_idx] += row[row_idx]
            for col_idx in range(5):
                normal[row_idx][col_idx] += row[row_idx] * row[col_idx]

    coeff_a, coeff_b, coeff_c, coeff_d, coeff_e = _solve_linear_system(normal, rhs)
    det = 4.0 * coeff_a * coeff_c - coeff_b * coeff_b
    if abs(det) <= 1e-12:
        raise ValueError("ellipse fit is degenerate")
    if det < 0.0:
        raise ValueError("fitted conic is not an ellipse")

    center_x = (coeff_b * coeff_e - 2.0 * coeff_c * coeff_d) / det
    center_y = (coeff_b * coeff_d - 2.0 * coeff_a * coeff_e) / det
    return (center_x, center_y)


def _extract_raw_points(dataset) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for record in dataset.records:
        if getattr(record, "stream_id", "") != "raw_magnetometer":
            continue
        points.append((float(record.mag_x), float(record.mag_y), float(record.mag_z)))
    return points


def calibrate(dataset, config=None):
    global _last_report
    _ = config

    raw_points = _extract_raw_points(dataset)
    if len(raw_points) < 8:
        raise ValueError("need at least 8 raw_magnetometer samples for calibration")

    plane_center, axis_u, axis_v = _plane_basis(raw_points)
    projected_points = _project_points_to_plane(raw_points, plane_center, axis_u, axis_v)
    ellipse_center_2d = _fit_ellipse_center(projected_points)
    center_3d = _vector_add(
        plane_center,
        _vector_add(
            _vector_scale(axis_u, ellipse_center_2d[0]),
            _vector_scale(axis_v, ellipse_center_2d[1]),
        ),
    )

    _last_report = (
        f"Raw samples: {len(raw_points)}\n"
        f"Plane center: ({plane_center[0]:.6f}, {plane_center[1]:.6f}, {plane_center[2]:.6f})\n"
        f"Ellipse center 2D: ({ellipse_center_2d[0]:.6f}, {ellipse_center_2d[1]:.6f})\n"
        f"Hard-iron center 3D: ({center_3d[0]:.6f}, {center_3d[1]:.6f}, {center_3d[2]:.6f})"
    )

    return {
        "algorithm_name": METHOD_NAME,
        "algorithm_version": METHOD_VERSION,
        "schema_version": SCHEMA_VERSION,
        "created_at": _utc_now(),
        "params": {
            "offset_x": center_3d[0],
            "offset_y": center_3d[1],
            "offset_z": center_3d[2],
        },
    }


def get_last_report() -> str:
    return _last_report


def load_params(path):
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def save_params(path, params):
    payload = dict(params or {})
    payload.setdefault("algorithm_name", METHOD_NAME)
    payload.setdefault("algorithm_version", METHOD_VERSION)
    payload.setdefault("schema_version", SCHEMA_VERSION)
    payload.setdefault("created_at", _utc_now())
    payload.setdefault(
        "params",
        {
            "offset_x": 0.0,
            "offset_y": 0.0,
            "offset_z": 0.0,
        },
    )
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)


def process(sample, params):
    inner = params.get("params", {}) if isinstance(params, dict) else {}
    offset_x = float(inner.get("offset_x", 0.0))
    offset_y = float(inner.get("offset_y", 0.0))
    offset_z = float(inner.get("offset_z", 0.0))

    mag_x = float(sample["mag_x"]) - offset_x
    mag_y = float(sample["mag_y"]) - offset_y
    mag_z = float(sample["mag_z"]) - offset_z

    heading = None
    if not (math.isclose(mag_x, 0.0, abs_tol=1e-9) and math.isclose(mag_y, 0.0, abs_tol=1e-9)):
        heading = (math.degrees(math.atan2(mag_x, mag_y)) + 360.0) % 360.0

    return {
        "timestamp_mcu": sample["timestamp_mcu"],
        "timestamp_pc_rx": sample["timestamp_pc_rx"],
        "timestamp_pc_est": sample.get("timestamp_pc_est"),
        "mag_x": mag_x,
        "mag_y": mag_y,
        "mag_z": mag_z,
        "heading": heading,
        "flags": "simple_hard_iron_rt",
    }
