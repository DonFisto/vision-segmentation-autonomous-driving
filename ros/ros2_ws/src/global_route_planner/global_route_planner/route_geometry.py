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
