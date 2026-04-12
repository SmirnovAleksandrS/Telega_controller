from __future__ import annotations

import json
import math
import statistics
from datetime import datetime, timezone


METHOD_NAME = "Improved Affine Magnetometer Method"
METHOD_VERSION = "1.3.0"
SCHEMA_VERSION = "1"

# Tuning parameters.
MIN_RAW_SAMPLES = 12
ELLIPSE_FIT_EPS = 1e-12
JACOBI_MAX_SWEEPS = 40
ANGULAR_SECTOR_COUNT = 90
MIN_EFFECTIVE_SECTOR_POPULATION = 4
ROBUST_PHASE_RESIDUAL_FLOOR = 0.02
ROBUST_PHASE_RESIDUAL_SIGMA = 3.0
ROBUST_RADIUS_PERCENTILE_LOW = 0.10
ROBUST_RADIUS_PERCENTILE_HIGH = 0.90
PRINT_DEBUG_REPORT = False

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
    if len(raw_records) < MIN_RAW_SAMPLES:
        return {"ok": False, "warnings": [f"need at least {MIN_RAW_SAMPLES} raw_magnetometer samples"]}
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
    if length <= ELLIPSE_FIT_EPS:
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

    for _ in range(JACOBI_MAX_SWEEPS):
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
        if max_off <= ELLIPSE_FIT_EPS:
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


def _plane_basis(
    points: list[tuple[float, float, float]]
) -> tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]:
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
    return center, axis_u, axis_v, normal


def _solve_linear_system(matrix: list[list[float]], rhs: list[float]) -> list[float]:
    n = len(rhs)
    aug = [list(matrix[row]) + [rhs[row]] for row in range(n)]

    for col in range(n):
        pivot_row = max(range(col, n), key=lambda row: abs(aug[row][col]))
        pivot = aug[pivot_row][col]
        if abs(pivot) <= ELLIPSE_FIT_EPS:
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
            if abs(factor) <= ELLIPSE_FIT_EPS:
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


def _fit_ellipse_parameters(
    projected_points: list[tuple[float, float]],
    *,
    weights: list[float] | None = None,
) -> dict[str, float]:
    normal = [[0.0 for _ in range(5)] for _ in range(5)]
    rhs = [0.0 for _ in range(5)]

    if weights is None:
        weights = [1.0] * len(projected_points)
    if len(weights) != len(projected_points):
        raise ValueError("weights length must match projected_points length")

    for (x_val, y_val), weight in zip(projected_points, weights):
        row = [x_val * x_val, x_val * y_val, y_val * y_val, x_val, y_val]
        for row_idx in range(5):
            rhs[row_idx] += weight * row[row_idx]
            for col_idx in range(5):
                normal[row_idx][col_idx] += weight * row[row_idx] * row[col_idx]

    coeff_a, coeff_b, coeff_c, coeff_d, coeff_e = _solve_linear_system(normal, rhs)
    det = 4.0 * coeff_a * coeff_c - coeff_b * coeff_b
    if abs(det) <= ELLIPSE_FIT_EPS:
        raise ValueError("ellipse fit is degenerate")
    if det < 0.0:
        raise ValueError("fitted conic is not an ellipse")

    center_x = (coeff_b * coeff_e - 2.0 * coeff_c * coeff_d) / det
    center_y = (coeff_b * coeff_d - 2.0 * coeff_a * coeff_e) / det

    q_matrix = [
        [coeff_a, 0.5 * coeff_b],
        [0.5 * coeff_b, coeff_c],
    ]
    linear = [coeff_d, coeff_e]
    center_q_center = (
        center_x * (q_matrix[0][0] * center_x + q_matrix[0][1] * center_y)
        + center_y * (q_matrix[1][0] * center_x + q_matrix[1][1] * center_y)
    )
    scale = center_q_center + 1.0
    if scale <= ELLIPSE_FIT_EPS:
        raise ValueError("ellipse normalization is degenerate")

    eigenvalues, eigenvectors = _jacobi_eigen_decomposition(q_matrix)
    if eigenvalues[0] <= ELLIPSE_FIT_EPS or eigenvalues[1] <= ELLIPSE_FIT_EPS:
        raise ValueError("ellipse quadratic form is not positive definite")

    radii: list[tuple[float, tuple[float, float]]] = []
    for idx in range(2):
        radius = math.sqrt(scale / eigenvalues[idx])
        vector = tuple(eigenvectors[idx])
        length = math.hypot(vector[0], vector[1])
        if length <= ELLIPSE_FIT_EPS:
            raise ValueError("ellipse eigenvector is degenerate")
        radii.append((radius, (vector[0] / length, vector[1] / length)))

    radii.sort(key=lambda item: item[0], reverse=True)
    major_radius, major_axis = radii[0]
    minor_radius, _minor_axis = radii[1]
    if minor_radius <= ELLIPSE_FIT_EPS:
        raise ValueError("minor ellipse radius is degenerate")

    theta = math.atan2(major_axis[1], major_axis[0])
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)

    return {
        "center_x": center_x,
        "center_y": center_y,
        "cos_theta": cos_theta,
        "sin_theta": sin_theta,
        "major_radius": major_radius,
        "minor_radius": minor_radius,
        "major_scale": 1.0,
        "minor_scale": major_radius / minor_radius,
        "axis_ratio": major_radius / minor_radius,
        "ellipse_a": coeff_a,
        "ellipse_b": coeff_b,
        "ellipse_c": coeff_c,
        "ellipse_d": coeff_d,
        "ellipse_e": coeff_e,
    }


def _rotate_points_to_ellipse_frame(
    projected_points: list[tuple[float, float]],
    *,
    center_x: float,
    center_y: float,
    cos_theta: float,
    sin_theta: float,
) -> list[tuple[float, float]]:
    rotated_points: list[tuple[float, float]] = []
    for x_val, y_val in projected_points:
        centered_x = x_val - center_x
        centered_y = y_val - center_y
        rotated_points.append(_rotate_to_ellipse_axes(centered_x, centered_y, cos_theta, sin_theta))
    return rotated_points


def _median(values: list[float]) -> float:
    if not values:
        raise ValueError("median requires non-empty values")
    return float(statistics.median(values))


def _quantile(values: list[float], q: float) -> float:
    if not values:
        raise ValueError("quantile requires non-empty values")
    if q <= 0.0:
        return min(values)
    if q >= 1.0:
        return max(values)
    sorted_values = sorted(values)
    position = (len(sorted_values) - 1) * q
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return sorted_values[lower]
    frac = position - lower
    return sorted_values[lower] * (1.0 - frac) + sorted_values[upper] * frac


def _compute_sector_weights(
    projected_points: list[tuple[float, float]],
    *,
    rough_ellipse: dict[str, float],
    sector_count: int = ANGULAR_SECTOR_COUNT,
) -> tuple[list[float], list[int]]:
    if sector_count <= 0:
        raise ValueError("sector_count must be positive")

    rotated_points = _rotate_points_to_ellipse_frame(
        projected_points,
        center_x=rough_ellipse["center_x"],
        center_y=rough_ellipse["center_y"],
        cos_theta=rough_ellipse["cos_theta"],
        sin_theta=rough_ellipse["sin_theta"],
    )
    major_radius = max(rough_ellipse["major_radius"], ELLIPSE_FIT_EPS)
    minor_radius = max(rough_ellipse["minor_radius"], ELLIPSE_FIT_EPS)
    residuals = [
        abs(math.hypot(rotated_x / major_radius, rotated_y / minor_radius) - 1.0)
        for rotated_x, rotated_y in rotated_points
    ]
    median_residual = _median(residuals)
    residual_mad = _median([abs(value - median_residual) for value in residuals])
    robust_scale = max(ROBUST_PHASE_RESIDUAL_FLOOR, 1.4826 * residual_mad)

    buckets: list[list[int]] = [[] for _ in range(sector_count)]
    for idx, (rotated_x, rotated_y) in enumerate(rotated_points):
        angle = math.atan2(rotated_y / minor_radius, rotated_x / major_radius)
        normalized = (angle + 2.0 * math.pi) % (2.0 * math.pi)
        bucket_idx = min(sector_count - 1, int(normalized / (2.0 * math.pi) * sector_count))
        buckets[bucket_idx].append(idx)

    weights = [0.0] * len(projected_points)
    counts: list[int] = []
    for members in buckets:
        if not members:
            continue
        counts.append(len(members))
        member_weight = 1.0 / float(max(len(members), MIN_EFFECTIVE_SECTOR_POPULATION))
        for idx in members:
            residual = residuals[idx]
            robust_weight = 1.0 / (1.0 + (residual / (ROBUST_PHASE_RESIDUAL_SIGMA * robust_scale)) ** 2)
            weights[idx] = member_weight * robust_weight
    return weights, counts


def _fit_balanced_ellipse_parameters(
    projected_points: list[tuple[float, float]],
    *,
    sector_count: int = ANGULAR_SECTOR_COUNT,
) -> dict[str, float]:
    rough = _fit_ellipse_parameters(projected_points)
    weights, sector_counts = _compute_sector_weights(
        projected_points,
        rough_ellipse=rough,
        sector_count=sector_count,
    )
    balanced = _fit_ellipse_parameters(projected_points, weights=weights)
    balanced["sector_count"] = float(len(sector_counts))
    balanced["used_sector_count"] = float(len(sector_counts))
    balanced["max_sector_population"] = float(max(sector_counts) if sector_counts else 0)
    balanced["min_sector_population"] = float(min(sector_counts) if sector_counts else 0)
    return balanced


def _fit_circle_center(points_2d: list[tuple[float, float]]) -> tuple[float, float]:
    normal = [[0.0 for _ in range(3)] for _ in range(3)]
    rhs = [0.0 for _ in range(3)]
    for x_val, y_val in points_2d:
        row = [x_val, y_val, 1.0]
        target = -(x_val * x_val + y_val * y_val)
        for row_idx in range(3):
            rhs[row_idx] += row[row_idx] * target
            for col_idx in range(3):
                normal[row_idx][col_idx] += row[row_idx] * row[col_idx]
    coeff_d, coeff_e, _coeff_f = _solve_linear_system(normal, rhs)
    return (-0.5 * coeff_d, -0.5 * coeff_e)


def _radius_debug_metrics(
    points_2d: list[tuple[float, float]],
) -> dict[str, float]:
    if not points_2d:
        raise ValueError("radius metrics require non-empty points")

    center_x, center_y = _fit_circle_center(points_2d)
    radii = [math.hypot(x_val - center_x, y_val - center_y) for x_val, y_val in points_2d]
    min_radius = min(radii)
    max_radius = max(radii)
    mean_radius = sum(radii) / float(len(radii))
    rms_radius_error = math.sqrt(sum((radius - mean_radius) ** 2 for radius in radii) / float(len(radii)))
    median_radius = _median(radii)
    radius_mad = _median([abs(radius - median_radius) for radius in radii])
    low_radius = _quantile(radii, ROBUST_RADIUS_PERCENTILE_LOW)
    high_radius = _quantile(radii, ROBUST_RADIUS_PERCENTILE_HIGH)

    return {
        "fit_center_x": center_x,
        "fit_center_y": center_y,
        "min_radius": min_radius,
        "max_radius": max_radius,
        "radius_span": max_radius - min_radius,
        "mean_radius": mean_radius,
        "rms_radius_error": rms_radius_error,
        "robust_radius_min": low_radius,
        "robust_radius_max": high_radius,
        "robust_radius_span": high_radius - low_radius,
        "robust_radius_median": median_radius,
        "robust_radius_mad": radius_mad,
    }


def _format_radius_debug_block(stage_name: str, metrics: dict[str, float]) -> str:
    return (
        f"{stage_name}:\n"
        f"  fit_center_x={metrics['fit_center_x']:.6f}\n"
        f"  fit_center_y={metrics['fit_center_y']:.6f}\n"
        f"  radius_min={metrics['min_radius']:.6f}\n"
        f"  radius_max={metrics['max_radius']:.6f}\n"
        f"  radius_span={metrics['radius_span']:.6f}\n"
        f"  radius_mean={metrics['mean_radius']:.6f}\n"
        f"  radius_rms_error={metrics['rms_radius_error']:.6f}\n"
        f"  robust_radius_min={metrics['robust_radius_min']:.6f}\n"
        f"  robust_radius_max={metrics['robust_radius_max']:.6f}\n"
        f"  robust_radius_span={metrics['robust_radius_span']:.6f}\n"
        f"  robust_radius_median={metrics['robust_radius_median']:.6f}\n"
        f"  robust_radius_mad={metrics['robust_radius_mad']:.6f}"
    )


def _extract_raw_points(dataset) -> list[tuple[float, float, float]]:
    points: list[tuple[float, float, float]] = []
    for record in dataset.records:
        if getattr(record, "stream_id", "") != "raw_magnetometer":
            continue
        points.append((float(record.mag_x), float(record.mag_y), float(record.mag_z)))
    return points


def _rotate_to_ellipse_axes(u_val: float, v_val: float, cos_theta: float, sin_theta: float) -> tuple[float, float]:
    return (
        cos_theta * u_val + sin_theta * v_val,
        -sin_theta * u_val + cos_theta * v_val,
    )


def _rotate_from_ellipse_axes(x_val: float, y_val: float, cos_theta: float, sin_theta: float) -> tuple[float, float]:
    return (
        cos_theta * x_val - sin_theta * y_val,
        sin_theta * x_val + cos_theta * y_val,
    )


def _build_runtime_params(
    center_3d: tuple[float, float, float],
    axis_u: tuple[float, float, float],
    axis_v: tuple[float, float, float],
    axis_n: tuple[float, float, float],
    ellipse: dict[str, float],
) -> dict[str, float]:
    return {
        "offset_x": center_3d[0],
        "offset_y": center_3d[1],
        "offset_z": center_3d[2],
        "axis_u_x": axis_u[0],
        "axis_u_y": axis_u[1],
        "axis_u_z": axis_u[2],
        "axis_v_x": axis_v[0],
        "axis_v_y": axis_v[1],
        "axis_v_z": axis_v[2],
        "axis_n_x": axis_n[0],
        "axis_n_y": axis_n[1],
        "axis_n_z": axis_n[2],
        "cos_theta": ellipse["cos_theta"],
        "sin_theta": ellipse["sin_theta"],
        "major_radius": ellipse["major_radius"],
        "minor_radius": ellipse["minor_radius"],
        "major_scale": ellipse["major_scale"],
        "minor_scale": ellipse["minor_scale"],
        "axis_ratio": ellipse["axis_ratio"],
    }


def _apply_affine_to_point(point: tuple[float, float, float], inner: dict[str, float]) -> tuple[float, float, float]:
    offset = (
        float(inner.get("offset_x", 0.0)),
        float(inner.get("offset_y", 0.0)),
        float(inner.get("offset_z", 0.0)),
    )
    axis_u = _normalize(
        (
            float(inner.get("axis_u_x", 1.0)),
            float(inner.get("axis_u_y", 0.0)),
            float(inner.get("axis_u_z", 0.0)),
        )
    )
    axis_v = _normalize(
        (
            float(inner.get("axis_v_x", 0.0)),
            float(inner.get("axis_v_y", 1.0)),
            float(inner.get("axis_v_z", 0.0)),
        )
    )
    axis_n = _normalize(
        (
            float(inner.get("axis_n_x", 0.0)),
            float(inner.get("axis_n_y", 0.0)),
            float(inner.get("axis_n_z", 1.0)),
        )
    )
    cos_theta = float(inner.get("cos_theta", 1.0))
    sin_theta = float(inner.get("sin_theta", 0.0))
    major_scale = float(inner.get("major_scale", 1.0))
    minor_scale = float(inner.get("minor_scale", 1.0))

    delta = _vector_sub(point, offset)
    plane_u = _dot(delta, axis_u)
    plane_v = _dot(delta, axis_v)
    plane_n = _dot(delta, axis_n)

    ellipse_x, ellipse_y = _rotate_to_ellipse_axes(plane_u, plane_v, cos_theta, sin_theta)
    circle_x = major_scale * ellipse_x
    circle_y = minor_scale * ellipse_y
    corrected_u, corrected_v = _rotate_from_ellipse_axes(circle_x, circle_y, cos_theta, sin_theta)
    # Map the corrected local plane basis onto the global horizontal frame.
    # This is a rigid plane-to-plane rotation, not a projection back onto XY.
    return (corrected_u, corrected_v, plane_n)


def _summarize_ellipse_cloud(points: list[tuple[float, float, float]]) -> dict[str, float | tuple[float, float, float]]:
    plane_center, axis_u, axis_v, axis_n = _plane_basis(points)
    projected_points = _project_points_to_plane(points, plane_center, axis_u, axis_v)
    ellipse = _fit_balanced_ellipse_parameters(projected_points)
    return {
        "plane_center": plane_center,
        "plane_normal": axis_n,
        "major_radius": ellipse["major_radius"],
        "minor_radius": ellipse["minor_radius"],
        "major_axis": 2.0 * ellipse["major_radius"],
        "minor_axis": 2.0 * ellipse["minor_radius"],
        "axis_delta": 2.0 * (ellipse["major_radius"] - ellipse["minor_radius"]),
    }


def calibrate(dataset, config=None):
    global _last_report
    _ = config

    raw_points = _extract_raw_points(dataset)
    if len(raw_points) < MIN_RAW_SAMPLES:
        raise ValueError(f"need at least {MIN_RAW_SAMPLES} raw_magnetometer samples for calibration")

    plane_center, axis_u, axis_v, axis_n = _plane_basis(raw_points)
    projected_points = _project_points_to_plane(raw_points, plane_center, axis_u, axis_v)
    ellipse = _fit_balanced_ellipse_parameters(projected_points)
    center_3d = _vector_add(
        plane_center,
        _vector_add(
            _vector_scale(axis_u, ellipse["center_x"]),
            _vector_scale(axis_v, ellipse["center_y"]),
        ),
    )

    runtime_params = _build_runtime_params(center_3d, axis_u, axis_v, axis_n, ellipse)
    corrected_points = [_apply_affine_to_point(point, runtime_params) for point in raw_points]
    raw_summary = _summarize_ellipse_cloud(raw_points)
    corrected_summary = _summarize_ellipse_cloud(corrected_points)

    centered_points = [
        (
            projected_point[0] - ellipse["center_x"],
            projected_point[1] - ellipse["center_y"],
        )
        for projected_point in projected_points
    ]
    rotated_points = [
        _rotate_to_ellipse_axes(point[0], point[1], ellipse["cos_theta"], ellipse["sin_theta"])
        for point in centered_points
    ]
    scaled_points = [
        (
            ellipse["major_scale"] * point[0],
            ellipse["minor_scale"] * point[1],
        )
        for point in rotated_points
    ]
    horizontal_xy_points = [(point[0], point[1]) for point in corrected_points]

    debug_blocks = [
        _format_radius_debug_block("Stage projected_centered", _radius_debug_metrics(centered_points)),
        _format_radius_debug_block("Stage rotated_axes", _radius_debug_metrics(rotated_points)),
        _format_radius_debug_block("Stage scaled_circle", _radius_debug_metrics(scaled_points)),
        _format_radius_debug_block("Stage horizontal_xy", _radius_debug_metrics(horizontal_xy_points)),
    ]

    _last_report = (
        f"Raw samples: {len(raw_points)}\n"
        f"Plane center: ({plane_center[0]:.6f}, {plane_center[1]:.6f}, {plane_center[2]:.6f})\n"
        f"Plane normal: ({axis_n[0]:.6f}, {axis_n[1]:.6f}, {axis_n[2]:.6f})\n"
        f"Ellipse center 2D: ({ellipse['center_x']:.6f}, {ellipse['center_y']:.6f})\n"
        f"Affine center 3D: ({center_3d[0]:.6f}, {center_3d[1]:.6f}, {center_3d[2]:.6f})\n"
        f"theta_deg: {math.degrees(math.atan2(ellipse['sin_theta'], ellipse['cos_theta'])):.6f}\n"
        f"Phase weighting: elliptic phase atan2(y'/b, x'/a)\n"
        f"Output mapping: local plane -> horizontal XY, normal -> Z\n"
        f"Used angular sectors: {int(ellipse['used_sector_count'])}\n"
        f"Raw major axis: {raw_summary['major_axis']:.6f}\n"
        f"Raw minor axis: {raw_summary['minor_axis']:.6f}\n"
        f"Raw axis delta: {raw_summary['axis_delta']:.6f}\n"
        f"Corrected major axis: {corrected_summary['major_axis']:.6f}\n"
        f"Corrected minor axis: {corrected_summary['minor_axis']:.6f}\n"
        f"Corrected axis delta: {corrected_summary['axis_delta']:.6f}\n\n"
        f"{chr(10).join(debug_blocks)}"
    )
    if PRINT_DEBUG_REPORT:
        print(_last_report)

    return {
        "algorithm_name": METHOD_NAME,
        "algorithm_version": METHOD_VERSION,
        "schema_version": SCHEMA_VERSION,
        "created_at": _utc_now(),
        "params": runtime_params,
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
            "axis_u_x": 1.0,
            "axis_u_y": 0.0,
            "axis_u_z": 0.0,
            "axis_v_x": 0.0,
            "axis_v_y": 1.0,
            "axis_v_z": 0.0,
            "axis_n_x": 0.0,
            "axis_n_y": 0.0,
            "axis_n_z": 1.0,
            "cos_theta": 1.0,
            "sin_theta": 0.0,
            "major_radius": 1.0,
            "minor_radius": 1.0,
            "major_scale": 1.0,
            "minor_scale": 1.0,
            "axis_ratio": 1.0,
        },
    )
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)


def process(sample, params):
    inner = params.get("params", {}) if isinstance(params, dict) else {}
    raw_vector = (
        float(sample["mag_x"]),
        float(sample["mag_y"]),
        float(sample["mag_z"]),
    )
    corrected = _apply_affine_to_point(raw_vector, inner)

    heading = None
    if not (math.isclose(corrected[0], 0.0, abs_tol=1e-9) and math.isclose(corrected[1], 0.0, abs_tol=1e-9)):
        heading = (math.degrees(math.atan2(corrected[0], corrected[1])) + 360.0) % 360.0

    return {
        "timestamp_mcu": sample["timestamp_mcu"],
        "timestamp_pc_rx": sample["timestamp_pc_rx"],
        "timestamp_pc_est": sample.get("timestamp_pc_est"),
        "mag_x": corrected[0],
        "mag_y": corrected[1],
        "mag_z": corrected[2],
        "heading": heading,
        "flags": "improved_affine_rt",
    }
