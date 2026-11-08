"""Pure geometry for route-association visualization."""

import math


def polyline_length_xy(points):
    if len(points) < 2:
        return 0.0

    return sum(
        math.hypot(b[0] - a[0], b[1] - a[1])
        for a, b in zip(points, points[1:])
    )


def slice_polyline_xy(points, start_s, end_s):
    """Extract an interpolated route segment in global arc length."""

    if len(points) < 2:
        return []

    if not all(
        math.isfinite(float(v))
        for v in (start_s, end_s)
    ) or end_s <= start_s:
        return []

    if any(
        len(p) != 3 or not all(math.isfinite(float(v)) for v in p)
        for p in points
    ):
        return []

    start = max(0.0, float(start_s))
    end = min(polyline_length_xy(points), float(end_s))

    if end <= start:
        return []

    result = []
    cumulative = 0.0

    for a, b in zip(points, points[1:]):
        span = math.hypot(b[0] - a[0], b[1] - a[1])

        if span <= 1e-9:
            continue

        left = max(start, cumulative)
        right = min(end, cumulative + span)

        if right > left:
            for s in (left, right):
                t = (s - cumulative) / span

                point = tuple(
                    float(u) + (float(v) - float(u)) * t
                    for u, v in zip(a, b)
                )

                if (
                    not result
                    or math.dist(result[-1], point) > 1e-7
                ):
                    result.append(point)

        cumulative += span

    return result if len(result) >= 2 else []


def clip_intervals(starts, ends, window_start, window_end):
    """Clip supported intervals without bridging gaps."""

    if len(starts) != len(ends):
        return []

    if not all(
        math.isfinite(float(v))
        for v in (window_start, window_end)
    ):
        return []

    if window_end <= window_start:
        return []

    intervals = []
    previous_end = float("-inf")

    for start, end in zip(starts, ends):
        start = float(start)
        end = float(end)

        if not all(math.isfinite(v) for v in (start, end)):
            return []

        if start >= end or start < previous_end - 1e-6:
            return []

        previous_end = end

        left = max(start, window_start)
        right = min(end, window_end)

        if right > left:
            intervals.append((left, right))

    return intervals
