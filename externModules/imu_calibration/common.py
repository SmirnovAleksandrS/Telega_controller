from __future__ import annotations

import math
import statistics
from datetime import datetime, timezone
from typing import Any, Callable, Sequence


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")


def vector_add(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def vector_sub(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vector_scale(a: tuple[float, float, float], scalar: float) -> tuple[float, float, float]:
    return (a[0] * scalar, a[1] * scalar, a[2] * scalar)


def dot(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def norm(a: tuple[float, float, float]) -> float:
    return math.sqrt(dot(a, a))


def normalize(a: tuple[float, float, float]) -> tuple[float, float, float]:
    length = norm(a)
    if length <= 1e-12:
        return (0.0, 0.0, 0.0)
    return (a[0] / length, a[1] / length, a[2] / length)


def matrix_vector_mul(matrix: Sequence[Sequence[float]], vector: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
    )


def determinant3(matrix: Sequence[Sequence[float]]) -> float:
    return (
        matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
    )


def matrix_from_flat(values: Sequence[float]) -> list[list[float]]:
    if len(values) != 9:
        raise ValueError("matrix flattening expects 9 values")
    return [
        [float(values[0]), float(values[1]), float(values[2])],
        [float(values[3]), float(values[4]), float(values[5])],
        [float(values[6]), float(values[7]), float(values[8])],
    ]


def flatten_matrix(matrix: Sequence[Sequence[float]]) -> list[float]:
    return [
        float(matrix[0][0]),
        float(matrix[0][1]),
        float(matrix[0][2]),
        float(matrix[1][0]),
        float(matrix[1][1]),
        float(matrix[1][2]),
        float(matrix[2][0]),
        float(matrix[2][1]),
        float(matrix[2][2]),
    ]


def identity3() -> list[list[float]]:
    return [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ]


def median_sample_period_s(samples: Sequence[dict[str, Any]]) -> float:
    deltas: list[float] = []
    last_timestamp = None
    for sample in samples:
        timestamp = int(sample.get("timestamp_mcu", 0))
        if last_timestamp is not None and timestamp > last_timestamp:
            deltas.append((timestamp - last_timestamp) / 1000.0)
        last_timestamp = timestamp
    if not deltas:
        return 0.02
    return max(1e-3, float(statistics.median(deltas)))


def _prefix_sum_vectors(vectors: Sequence[tuple[float, float, float]]) -> list[tuple[float, float, float]]:
    prefix = [(0.0, 0.0, 0.0)]
    running = (0.0, 0.0, 0.0)
    for vector in vectors:
        running = vector_add(running, vector)
        prefix.append(running)
    return prefix


def _prefix_sum_scalars(values: Sequence[float]) -> list[float]:
    prefix = [0.0]
    running = 0.0
    for value in values:
        running += float(value)
        prefix.append(running)
    return prefix


def _window_bounds(index: int, count: int, window_size: int) -> tuple[int, int]:
    left = window_size // 2
    right = window_size - left
    start = max(0, index - left)
    end = min(count, index + right)
    return (start, end)


def moving_average_vectors(vectors: Sequence[tuple[float, float, float]], window_size: int) -> list[tuple[float, float, float]]:
    if not vectors:
        return []
    size = max(1, int(window_size))
    prefix = _prefix_sum_vectors(vectors)
    averaged: list[tuple[float, float, float]] = []
    count = len(vectors)
    for index in range(count):
        start, end = _window_bounds(index, count, size)
        width = float(max(1, end - start))
        total = vector_sub(prefix[end], prefix[start])
        averaged.append(vector_scale(total, 1.0 / width))
    return averaged


def moving_average_scalars(values: Sequence[float], window_size: int) -> list[float]:
    if not values:
        return []
    size = max(1, int(window_size))
    prefix = _prefix_sum_scalars(values)
    averaged: list[float] = []
    count = len(values)
    for index in range(count):
        start, end = _window_bounds(index, count, size)
        width = float(max(1, end - start))
        averaged.append((prefix[end] - prefix[start]) / width)
    return averaged


def mean_vector(vectors: Sequence[tuple[float, float, float]]) -> tuple[float, float, float]:
    if not vectors:
        return (0.0, 0.0, 0.0)
    inv = 1.0 / float(len(vectors))
    sx = sy = sz = 0.0
    for vector in vectors:
        sx += vector[0]
        sy += vector[1]
        sz += vector[2]
    return (sx * inv, sy * inv, sz * inv)


def axis_variances(vectors: Sequence[tuple[float, float, float]]) -> tuple[float, float, float]:
    if len(vectors) < 2:
        return (0.0, 0.0, 0.0)
    xs = [vector[0] for vector in vectors]
    ys = [vector[1] for vector in vectors]
    zs = [vector[2] for vector in vectors]
    return (
        statistics.pvariance(xs),
        statistics.pvariance(ys),
        statistics.pvariance(zs),
    )


def detect_quasi_static_segments(
    samples: Sequence[dict[str, Any]],
    *,
    threshold_g2: float,
    static_time_s: float,
    vector_getter: Callable[[dict[str, Any]], tuple[float, float, float]],
) -> dict[str, Any]:
    if not samples:
        return {
            "sample_period_s": 0.02,
            "window_size": 1,
            "min_samples": 1,
            "segments": [],
            "scores": [],
        }
    vectors = [vector_getter(sample) for sample in samples]
    sample_period_s = median_sample_period_s(samples)
    window_size = max(1, int(round(static_time_s / sample_period_s)))
    min_samples = max(1, int(math.ceil(static_time_s / sample_period_s)))
    baseline = moving_average_vectors(vectors, window_size)
    hpf = [vector_sub(vector, trend) for vector, trend in zip(vectors, baseline)]
    rectified = [(abs(vector[0]), abs(vector[1]), abs(vector[2])) for vector in hpf]
    lpf = moving_average_vectors(rectified, window_size)
    scores = [dot(vector, vector) for vector in lpf]
    mask = [score < threshold_g2 for score in scores]

    segments: list[dict[str, Any]] = []
    start_index: int | None = None
    for index, is_static in enumerate(mask + [False]):
        if is_static and start_index is None:
            start_index = index
            continue
        if is_static or start_index is None:
            continue
        end_index = index - 1
        sample_count = end_index - start_index + 1
        if sample_count >= min_samples:
            segment_vectors = vectors[start_index:end_index + 1]
            start_time = int(samples[start_index].get("timestamp_mcu", 0))
            end_time = int(samples[end_index].get("timestamp_mcu", start_time))
            duration_s = max(sample_period_s, (end_time - start_time) / 1000.0 if end_time >= start_time else sample_count * sample_period_s)
            mid_index = start_index + (sample_count // 2)
            segments.append(
                {
                    "start_index": start_index,
                    "end_index": end_index,
                    "mid_index": mid_index,
                    "sample_count": sample_count,
                    "duration_s": duration_s,
                    "start_time_mcu": start_time,
                    "end_time_mcu": end_time,
                    "mean_time_mcu": int(samples[mid_index].get("timestamp_mcu", start_time)),
                    "mean_vector": mean_vector(segment_vectors),
                }
            )
        start_index = None

    return {
        "sample_period_s": sample_period_s,
        "window_size": window_size,
        "min_samples": min_samples,
        "segments": segments,
        "scores": scores,
        "mask": mask,
    }


def choose_best_quasi_static_config(
    samples: Sequence[dict[str, Any]],
    *,
    thresholds_g2: Sequence[float],
    static_times_s: Sequence[float],
    vector_getter: Callable[[dict[str, Any]], tuple[float, float, float]],
) -> dict[str, Any]:
    best: dict[str, Any] | None = None
    for threshold_g2 in thresholds_g2:
        for static_time_s in static_times_s:
            detected = detect_quasi_static_segments(
                samples,
                threshold_g2=threshold_g2,
                static_time_s=static_time_s,
                vector_getter=vector_getter,
            )
            segments = detected["segments"]
            quality = (
                len(segments),
                sum(segment["sample_count"] for segment in segments),
                -float(threshold_g2),
                -float(static_time_s),
            )
            candidate = dict(detected)
            candidate["threshold_g2"] = float(threshold_g2)
            candidate["static_time_s"] = float(static_time_s)
            candidate["quality"] = quality
            if best is None or quality > best["quality"]:
                best = candidate
    if best is None:
        return {
            "threshold_g2": 0.0,
            "static_time_s": 0.0,
            "sample_period_s": 0.02,
            "window_size": 1,
            "min_samples": 1,
            "segments": [],
            "scores": [],
            "mask": [],
            "quality": (0, 0, 0.0, 0.0),
        }
    return best


def pair_series_by_timestamp(
    first: Sequence[dict[str, Any]],
    second: Sequence[dict[str, Any]],
) -> dict[str, Any]:
    second_map: dict[int, list[dict[str, Any]]] = {}
    for sample in second:
        timestamp = int(sample.get("timestamp_mcu", 0))
        second_map.setdefault(timestamp, []).append(sample)
    matched_pairs: list[tuple[dict[str, Any], dict[str, Any]]] = []
    for sample in first:
        timestamp = int(sample.get("timestamp_mcu", 0))
        bucket = second_map.get(timestamp)
        if not bucket:
            continue
        matched_pairs.append((sample, bucket.pop(0)))
    matched_count = len(matched_pairs)
    dropped = len(first) + len(second) - (2 * matched_count)
    overlap_ratio = 0.0
    if max(len(first), len(second)) > 0:
        overlap_ratio = matched_count / float(max(len(first), len(second)))
    return {
        "pairs": matched_pairs,
        "matched_count": matched_count,
        "dropped_count": dropped,
        "overlap_ratio": overlap_ratio,
    }


def quat_mul(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    return (
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    )


def quat_conjugate(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    return (q[0], -q[1], -q[2], -q[3])


def quat_normalize(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    length = math.sqrt(sum(component * component for component in q))
    if length <= 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return tuple(component / length for component in q)


def quat_from_omega_dt(omega: tuple[float, float, float], dt_s: float) -> tuple[float, float, float, float]:
    angle = norm(omega) * max(0.0, float(dt_s))
    if angle <= 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    axis = normalize(omega)
    half_angle = angle * 0.5
    sin_half = math.sin(half_angle)
    return (
        math.cos(half_angle),
        axis[0] * sin_half,
        axis[1] * sin_half,
        axis[2] * sin_half,
    )


def quat_rotate_vector(q: tuple[float, float, float, float], vector: tuple[float, float, float]) -> tuple[float, float, float]:
    pure = (0.0, vector[0], vector[1], vector[2])
    rotated = quat_mul(quat_mul(q, pure), quat_conjugate(q))
    return (rotated[1], rotated[2], rotated[3])


def integrate_gyro_samples(
    samples: Sequence[dict[str, Any]],
    *,
    matrix: Sequence[Sequence[float]],
    bias: tuple[float, float, float],
) -> tuple[float, float, float, float]:
    if len(samples) < 2:
        return (1.0, 0.0, 0.0, 0.0)
    q = (1.0, 0.0, 0.0, 0.0)
    for current, nxt in zip(samples[:-1], samples[1:]):
        dt_s = max(0.0, (int(nxt.get("timestamp_mcu", 0)) - int(current.get("timestamp_mcu", 0))) / 1000.0)
        if dt_s <= 0.0:
            continue
        raw = (
            float(current.get("gyro_x", 0.0)),
            float(current.get("gyro_y", 0.0)),
            float(current.get("gyro_z", 0.0)),
        )
        corrected = matrix_vector_mul(matrix, vector_sub(raw, bias))
        dq = quat_from_omega_dt(corrected, dt_s)
        q = quat_normalize(quat_mul(q, dq))
    return q


def nelder_mead(
    objective: Callable[[list[float]], float],
    x0: Sequence[float],
    *,
    step: Sequence[float] | float = 0.1,
    max_iter: int = 600,
    tolerance: float = 1e-8,
) -> tuple[list[float], float]:
    start = [float(value) for value in x0]
    if isinstance(step, (int, float)):
        steps = [float(step)] * len(start)
    else:
        steps = [float(value) for value in step]
        if len(steps) != len(start):
            raise ValueError("nelder_mead step vector size mismatch")
    simplex: list[list[float]] = [list(start)]
    for index, delta in enumerate(steps):
        vertex = list(start)
        vertex[index] = vertex[index] + (delta if abs(delta) > 1e-9 else 0.05)
        simplex.append(vertex)

    values = [objective(vertex) for vertex in simplex]
    alpha = 1.0
    gamma = 2.0
    rho = 0.5
    sigma = 0.5

    for _ in range(max_iter):
        order = sorted(range(len(simplex)), key=lambda idx: values[idx])
        simplex = [simplex[idx] for idx in order]
        values = [values[idx] for idx in order]
        best = simplex[0]
        worst = simplex[-1]
        spread = max(abs(value - values[0]) for value in values)
        if spread <= tolerance:
            break
        centroid = [0.0] * len(start)
        for vertex in simplex[:-1]:
            for index, value in enumerate(vertex):
                centroid[index] += value
        inv = 1.0 / float(len(start))
        centroid = [value * inv for value in centroid]

        reflected = [centroid[index] + alpha * (centroid[index] - worst[index]) for index in range(len(start))]
        reflected_value = objective(reflected)
        if values[0] <= reflected_value < values[-2]:
            simplex[-1] = reflected
            values[-1] = reflected_value
            continue
        if reflected_value < values[0]:
            expanded = [centroid[index] + gamma * (reflected[index] - centroid[index]) for index in range(len(start))]
            expanded_value = objective(expanded)
            if expanded_value < reflected_value:
                simplex[-1] = expanded
                values[-1] = expanded_value
            else:
                simplex[-1] = reflected
                values[-1] = reflected_value
            continue

        contracted = [centroid[index] + rho * (worst[index] - centroid[index]) for index in range(len(start))]
        contracted_value = objective(contracted)
        if contracted_value < values[-1]:
            simplex[-1] = contracted
            values[-1] = contracted_value
            continue

        best = simplex[0]
        new_simplex = [best]
        for vertex in simplex[1:]:
            shrunk = [best[index] + sigma * (vertex[index] - best[index]) for index in range(len(start))]
            new_simplex.append(shrunk)
        simplex = new_simplex
        values = [objective(vertex) for vertex in simplex]

    order = sorted(range(len(simplex)), key=lambda idx: values[idx])
    best_index = order[0]
    return (list(simplex[best_index]), float(values[best_index]))


def fit_accelerometer_affine(mean_vectors: Sequence[tuple[float, float, float]]) -> dict[str, Any]:
    if len(mean_vectors) < 4:
        raise ValueError("need at least 4 static mean vectors")
    mins = [min(vector[index] for vector in mean_vectors) for index in range(3)]
    maxs = [max(vector[index] for vector in mean_vectors) for index in range(3)]
    bias0 = [(mins[index] + maxs[index]) * 0.5 for index in range(3)]
    centered = [vector_sub(vector, (bias0[0], bias0[1], bias0[2])) for vector in mean_vectors]
    avg_radius = sum(norm(vector) for vector in centered) / float(len(centered))
    initial_scale = 1.0 / avg_radius if avg_radius > 1e-9 else 1.0
    x0 = [bias0[0], bias0[1], bias0[2], initial_scale, 0.0, 0.0, 0.0, initial_scale, 0.0, 0.0, 0.0, initial_scale]

    def objective(values: list[float]) -> float:
        bias = (values[0], values[1], values[2])
        matrix = matrix_from_flat(values[3:])
        det = determinant3(matrix)
        cost = 0.0
        for raw_vector in mean_vectors:
            corrected = matrix_vector_mul(matrix, vector_sub(raw_vector, bias))
            radius_error = norm(corrected) - 1.0
            cost += radius_error * radius_error
        ideal = identity3()
        regularization = 0.0
        for row in range(3):
            for col in range(3):
                delta = matrix[row][col] - ideal[row][col]
                regularization += delta * delta
        cost += 1e-4 * regularization
        if det <= 0.0:
            cost += 100.0 + (abs(det) * 10.0)
        elif det < 1e-3:
            cost += 10.0 * (1e-3 - det) * (1e-3 - det)
        return cost

    best, cost = nelder_mead(
        objective,
        x0,
        step=[0.02, 0.02, 0.02, 0.05, 0.01, 0.01, 0.01, 0.05, 0.01, 0.01, 0.01, 0.05],
        max_iter=900,
        tolerance=1e-10,
    )
    bias = (best[0], best[1], best[2])
    matrix = matrix_from_flat(best[3:])
    corrected_means = [matrix_vector_mul(matrix, vector_sub(vector, bias)) for vector in mean_vectors]
    return {
        "bias": bias,
        "matrix": matrix,
        "cost": float(cost),
        "corrected_means": corrected_means,
    }


def fit_gyroscope_matrix(
    intervals: Sequence[dict[str, Any]],
    *,
    bias: tuple[float, float, float],
) -> dict[str, Any]:
    if not intervals:
        raise ValueError("need at least one motion interval")
    x0 = flatten_matrix(identity3())

    def objective(values: list[float]) -> float:
        matrix = matrix_from_flat(values)
        det = determinant3(matrix)
        cost = 0.0
        for interval in intervals:
            q = integrate_gyro_samples(interval["gyro_samples"], matrix=matrix, bias=bias)
            predicted = normalize(quat_rotate_vector(q, interval["gravity_start"]))
            target = normalize(interval["gravity_end"])
            diff = vector_sub(predicted, target)
            cost += dot(diff, diff)
        ideal = identity3()
        regularization = 0.0
        for row in range(3):
            for col in range(3):
                delta = matrix[row][col] - ideal[row][col]
                regularization += delta * delta
        cost += 5e-4 * regularization
        if det <= 0.0:
            cost += 100.0 + (abs(det) * 10.0)
        elif det < 0.1:
            cost += 10.0 * (0.1 - det) * (0.1 - det)
        return cost

    best, cost = nelder_mead(
        objective,
        x0,
        step=[0.03, 0.01, 0.01, 0.01, 0.03, 0.01, 0.01, 0.01, 0.03],
        max_iter=700,
        tolerance=1e-10,
    )
    return {
        "matrix": matrix_from_flat(best),
        "cost": float(cost),
    }

