"""Estimate spatial support of a perceived lane along a route window.

This is current-frame spatial evidence, not a temporal filter or a
planning-validity decision. All interval coordinates use global route s.
"""

from dataclasses import dataclass
import math

from global_route_planner.lane_route_geometry import LanePoint
from global_route_planner.route_geometry import (
    RouteWindow,
    project_point_to_polyline,
)


@dataclass(frozen=True)
class CoverageParameters:
    min_sample_confidence: float = 0.50
    max_route_distance_m: float = 1.0
    max_heading_error_deg: float = 15.0
    max_sample_gap_m: float = 1.5

    def __post_init__(self):
        values = (
            self.min_sample_confidence,
            self.max_route_distance_m,
            self.max_heading_error_deg,
            self.max_sample_gap_m,
        )

        if not all(math.isfinite(v) for v in values):
            raise ValueError("Coverage parameters must be finite")

        if not 0.0 <= self.min_sample_confidence <= 1.0:
            raise ValueError("Invalid confidence threshold")

        if self.max_route_distance_m <= 0.0:
            raise ValueError("Route distance threshold must be positive")

        if not 0.0 < self.max_heading_error_deg <= 180.0:
            raise ValueError("Invalid heading threshold")

        if self.max_sample_gap_m <= 0.0:
            raise ValueError("Maximum sample gap must be positive")


@dataclass(frozen=True)
class LaneRouteCoverage:
    window_length_m: float
    supported_length_m: float
    coverage_ratio: float
    longest_contiguous_m: float
    supported_intervals: tuple[tuple[float, float], ...]
    valid_sample_count: int
    connected_pair_count: int


def estimate_lane_route_coverage(
    route_window: RouteWindow,
    lane_points: list[LanePoint],
    parameters: CoverageParameters | None = None,
) -> LaneRouteCoverage:
    """Measure support only between consecutive, compatible lane samples.

    Points must be ordered along the lane's driving direction.

    Each pair requires two quality-gated endpoints, increasing route s,
    and short gaps in both route s and world XY.

    Gaps are not bridged or filled.
    """

    params = parameters or CoverageParameters()

    start = route_window.start_s_m
    end = route_window.end_s_m

    if not (
        math.isfinite(start)
        and math.isfinite(end)
        and end > start
    ):
        raise ValueError("Invalid route window interval")

    samples = route_window.samples

    if len(samples) < 2:
        raise ValueError("Route window requires at least two samples")

    if not all(
        math.isfinite(sample.x) and math.isfinite(sample.y)
        for sample in samples
    ):
        raise ValueError("Non-finite route window geometry")

    polyline = [
        (sample.x, sample.y)
        for sample in samples
    ]

    valid = []

    for point in lane_points:
        values = (
            point.x_m,
            point.y_m,
            point.heading_rad,
            point.confidence,
        )

        if not all(math.isfinite(v) for v in values):
            raise ValueError("Non-finite lane sample")

        if not 0.0 <= point.confidence <= 1.0:
            raise ValueError("Invalid lane confidence")

        projection = project_point_to_polyline(
            polyline,
            point.x_m,
            point.y_m,
        )

        heading_delta = math.atan2(
            math.sin(point.heading_rad - projection.heading_rad),
            math.cos(point.heading_rad - projection.heading_rad),
        )

        accepted = (
            point.confidence >= params.min_sample_confidence
            and projection.distance_m <= params.max_route_distance_m
            and abs(math.degrees(heading_delta))
            <= params.max_heading_error_deg
        )

        if accepted:
            valid.append((
                start + projection.s_m,
                point.x_m,
                point.y_m,
            ))
        else:
            valid.append(None)

    intervals = []
    connected_pairs = 0
    eps = 1e-6

    for left, right in zip(valid, valid[1:]):
        if left is None or right is None:
            continue

        s0, x0, y0 = left
        s1, x1, y1 = right

        delta_s = s1 - s0
        world_gap = math.hypot(x1 - x0, y1 - y0)

        if not (
            eps < delta_s <= params.max_sample_gap_m + eps
        ):
            continue

        if world_gap > params.max_sample_gap_m + eps:
            continue

        a = max(start, s0)
        b = min(end, s1)

        if b > a + eps:
            intervals.append((a, b))
            connected_pairs += 1

    # Merge overlapping or immediately adjacent supported intervals.
    # This avoids counting the same route distance multiple times.

    merged = []

    for a, b in sorted(intervals):
        if merged and a <= merged[-1][1] + eps:
            merged[-1] = (
                merged[-1][0],
                max(merged[-1][1], b),
            )
        else:
            merged.append((a, b))

    supported = sum(
        b - a
        for a, b in merged
    )

    length = end - start

    return LaneRouteCoverage(
        window_length_m=length,
        supported_length_m=supported,
        coverage_ratio=min(1.0, supported / length),
        longest_contiguous_m=max(
            (b - a for a, b in merged),
            default=0.0,
        ),
        supported_intervals=tuple(merged),
        valid_sample_count=sum(
            point is not None
            for point in valid
        ),
        connected_pair_count=connected_pairs,
    )
