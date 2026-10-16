"""ROS-independent geometry utilities for route-relative coordinates."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class RouteProjection:
    s_m: float
    d_m: float
    projection_x: float
    projection_y: float
    heading_rad: float
    distance_m: float
    segment_index: int


def project_point_to_polyline(
    points: list[tuple[float, float]],
    x: float,
    y: float,
) -> RouteProjection:
    """Project an XY point onto an ordered route polyline.

    Positive d_m denotes the left side of the directed
    polyline in the mathematical XY plane.

    The longitudinal coordinate s_m is measured from
    the beginning of the supplied polyline.
    """
    if len(points) < 2:
        raise ValueError("A polyline requires at least two points")

    best = None
    cumulative_s = 0.0

    for i in range(len(points) - 1):
        ax, ay = points[i]
        bx, by = points[i + 1]

        dx = bx - ax
        dy = by - ay

        length_sq = dx * dx + dy * dy

        if length_sq == 0.0:
            continue

        length = math.sqrt(length_sq)

        t = (
            (x - ax) * dx + (y - ay) * dy
        ) / length_sq

        t = max(0.0, min(1.0, t))

        px = ax + t * dx
        py = ay + t * dy

        error_x = x - px
        error_y = y - py

        distance = math.hypot(error_x, error_y)

        signed_distance = (dx * error_y - dy * error_x) / length

        if distance == 0.0:
            signed_distance = 0.0

        candidate = RouteProjection(
            s_m=cumulative_s + t * length,
            d_m=signed_distance,
            projection_x=px,
            projection_y=py,
            heading_rad=math.atan2(dy, dx),
            distance_m=distance,
            segment_index=i,
        )

        # On equal-distance matches, retain the earliest
        # route segment. This is deterministic, but does
        # not resolve ambiguous intersection branches.
        if best is None or distance < best.distance_m:
            best = candidate

        cumulative_s += length

    if best is None:
        raise ValueError("Polyline contains no nonzero segments")

    return best

@dataclass(frozen=True)
class RouteSample:
    s_m: float
    x: float
    y: float
    heading_rad: float
    segment_index: int
def interpolate_polyline_at_s(
    points: list[tuple[float, float]],
    s_m: float,
) -> RouteSample:
    """Interpolate a directed polyline at a given arc length."""

    if len(points) < 2:
        raise ValueError("A polyline requires at least two points")

    segments = []
    cumulative_s = 0.0

    for i in range(len(points) - 1):
        ax, ay = points[i]
        bx, by = points[i + 1]

        dx = bx - ax
        dy = by - ay

        length = math.hypot(dx, dy)

        if length == 0.0:
            continue

        segments.append(
            (i, ax, ay, dx, dy, length, cumulative_s)
        )

        cumulative_s += length

    if not segments:
        raise ValueError("Polyline contains no nonzero segments")

    if not math.isfinite(s_m):
        raise ValueError("Arc length must be finite")

    if s_m < 0.0 or s_m > cumulative_s:
        raise ValueError("Arc length is outside the route")

    for index, segment in enumerate(segments):
        i, ax, ay, dx, dy, length, start_s = segment

        end_s = start_s + length
        is_last = index == len(segments) - 1

        # At interior vertices, prefer the outgoing segment.
        if s_m < end_s or is_last:
            t = (s_m - start_s) / length

            return RouteSample(
                s_m=s_m,
                x=ax + t * dx,
                y=ay + t * dy,
                heading_rad=math.atan2(dy, dx),
                segment_index=i,
            )

    raise RuntimeError("Failed to interpolate route")

@dataclass(frozen=True)
class RouteWindow:
    start_s_m: float
    end_s_m: float
    samples: tuple[RouteSample, ...]
def extract_route_window(
    points: list[tuple[float, float]],
    center_s_m: float,
    backward_m: float,
    forward_m: float,
) -> RouteWindow:
    """Extract a route interval while preserving route-relative s."""

    if len(points) < 2:
        raise ValueError("A polyline requires at least two points")

    for name, value in (
        ("center_s_m", center_s_m),
        ("backward_m", backward_m),
        ("forward_m", forward_m),
    ):
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")

    if backward_m < 0.0 or forward_m < 0.0:
        raise ValueError("Window distances cannot be negative")

    # Cumulative arc length at every original route vertex.
    vertex_s = [0.0]

    for i in range(len(points) - 1):
        ax, ay = points[i]
        bx, by = points[i + 1]

        length = math.hypot(bx - ax, by - ay)

        vertex_s.append(vertex_s[-1] + length)

    total_length = vertex_s[-1]

    if not math.isfinite(total_length):
        raise ValueError("Route length must be finite")

    if total_length == 0.0:
        raise ValueError("Polyline contains no nonzero segments")

    if center_s_m < 0.0 or center_s_m > total_length:
        raise ValueError("Window center is outside the route")

    # Clip the requested interval to the actual route limits.
    start_s = max(0.0, center_s_m - backward_m)
    end_s = min(total_length, center_s_m + forward_m)

    samples = [
        interpolate_polyline_at_s(points, start_s)
    ]

    previous_s = start_s

    # Preserve original route vertices inside the window.
    # Skip repeated arc-length values from duplicate points.
    for s in vertex_s[1:-1]:
        if start_s < s < end_s and s > previous_s:
            samples.append(
                interpolate_polyline_at_s(points, s)
            )
            previous_s = s

    # A zero-length window contains only its center sample.
    if end_s > start_s:
        samples.append(
            interpolate_polyline_at_s(points, end_s)
        )

    return RouteWindow(
        start_s_m=start_s,
        end_s_m=end_s,
        samples=tuple(samples),
    )
