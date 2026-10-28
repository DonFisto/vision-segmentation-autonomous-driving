"""ROS-independent comparison of perceived lanes and route windows."""

from dataclasses import dataclass
import math
from statistics import median

from global_route_planner.route_geometry import (
    RouteWindow,
    project_point_to_polyline,
)


@dataclass(frozen=True)
class LanePoint:
    """One perception-derived lane centerline sample."""

    x_m: float
    y_m: float
    heading_rad: float
    confidence: float


@dataclass(frozen=True)
class LaneRouteComparison:
    """Descriptive geometric metrics, not an accepted association."""

    sample_count: int
    median_distance_m: float
    median_abs_lateral_m: float
    median_abs_heading_error_deg: float
    median_confidence: float
    min_route_s_m: float
    max_route_s_m: float
    projected_s_span_m: float


def _absolute_heading_error_deg(first_rad, second_rad):
    difference = first_rad - second_rad

    wrapped = math.atan2(
        math.sin(difference),
        math.cos(difference),
    )

    return abs(math.degrees(wrapped))


def compare_lane_to_route(
    route_window: RouteWindow,
    lane_points: list[LanePoint],
) -> LaneRouteComparison | None:
    """Compare lane samples against a directed route window.

    The route window retains global RoutePlan arc-length values.

    Returns None for an empty lane observation.

    Results describe geometric agreement only. They do not
    establish association validity or perception reliability.
    """

    if not lane_points:
        return None

    if (
        len(route_window.samples) < 2
        or route_window.end_s_m <= route_window.start_s_m
    ):
        raise ValueError("Route window has insufficient geometry")

    route_points = [
        (sample.x, sample.y)
        for sample in route_window.samples
    ]

    if not all(
        math.isfinite(x) and math.isfinite(y)
        for x, y in route_points
    ):
        raise ValueError("Route contains non-finite coordinates")

    distances = []
    lateral_errors = []
    heading_errors = []
    confidences = []
    route_positions = []

    for point in lane_points:
        values = (
            point.x_m,
            point.y_m,
            point.heading_rad,
            point.confidence,
        )

        if not all(math.isfinite(value) for value in values):
            raise ValueError("Lane sample contains non-finite values")

        if not 0.0 <= point.confidence <= 1.0:
            raise ValueError("Lane confidence outside [0, 1]")

        projection = project_point_to_polyline(
            route_points,
            point.x_m,
            point.y_m,
        )

        distances.append(projection.distance_m)
        lateral_errors.append(abs(projection.d_m))

        heading_errors.append(
            _absolute_heading_error_deg(
                point.heading_rad,
                projection.heading_rad,
            )
        )

        confidences.append(point.confidence)

        # Projection returns s relative to the supplied polyline.
        # Restore the arc-length origin of the global RoutePlan.
        route_positions.append(
            route_window.start_s_m + projection.s_m
        )

    minimum_s = min(route_positions)
    maximum_s = max(route_positions)

    return LaneRouteComparison(
        sample_count=len(lane_points),
        median_distance_m=median(distances),
        median_abs_lateral_m=median(lateral_errors),
        median_abs_heading_error_deg=median(heading_errors),
        median_confidence=median(confidences),
        min_route_s_m=minimum_s,
        max_route_s_m=maximum_s,
        projected_s_span_m=maximum_s - minimum_s,
    )
