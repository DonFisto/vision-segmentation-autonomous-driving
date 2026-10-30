"""Ego-relative summary of lane support intervals along a RoutePlan.

Pure geometry. Uses global RoutePlan arc-length s, not ego-frame distance.
Does not infer support across gaps or make a planning-validity decision.
"""

from dataclasses import dataclass
import math

from global_route_planner.lane_route_coverage import LaneRouteCoverage
from global_route_planner.route_geometry import RouteWindow


@dataclass(frozen=True)
class ForwardLaneRouteCoverage:
    forward_window_length_m: float
    supported_ahead_m: float
    coverage_ratio_ahead: float
    longest_contiguous_ahead_m: float
    continuous_from_ego_m: float
    first_support_gap_m: float | None
    supported_intervals_ahead: tuple[tuple[float, float], ...]


def summarize_forward_coverage(
    coverage: LaneRouteCoverage,
    route_window: RouteWindow,
    ego_s_m: float,
) -> ForwardLaneRouteCoverage:
    """Summarize supported geometry from ego_s_m to the window end.

    Continuous support from ego is zero unless a supported interval
    contains the ego's route position.

    Unsupported gaps are never bridged.
    """

    start = route_window.start_s_m
    end = route_window.end_s_m
    ego = float(ego_s_m)
    eps = 1e-6

    if not all(math.isfinite(v) for v in (start, end, ego)):
        raise ValueError("Route bounds and ego s must be finite")

    if end <= start:
        raise ValueError("Route window must have positive length")

    if not start <= ego <= end:
        raise ValueError("Ego s must be inside the route window")

    if (
        not math.isfinite(coverage.window_length_m)
        or abs(coverage.window_length_m - (end - start)) > eps
    ):
        raise ValueError("Coverage length does not match route window")

    # Validate intervals in global route coordinates.
    previous_end = start

    for a, b in coverage.supported_intervals:
        if not all(math.isfinite(v) for v in (a, b)):
            raise ValueError("Non-finite supported interval")

        if not (start <= a < b <= end):
            raise ValueError("Supported interval outside route window")

        if a < previous_end - eps:
            raise ValueError("Intervals overlap or are unordered")

        previous_end = b

    # Clip support intervals to the forward route window.
    ahead = []

    for a, b in coverage.supported_intervals:
        left = max(a, ego)

        if b <= left:
            continue

        # Merge adjacent intervals without bridging real gaps.
        if ahead and left <= ahead[-1][1] + eps:
            ahead[-1] = (
                ahead[-1][0],
                max(ahead[-1][1], b),
            )
        else:
            ahead.append((left, b))

    forward_length = end - ego

    supported = sum(
        b - a
        for a, b in ahead
    )

    longest = max(
        (b - a for a, b in ahead),
        default=0.0,
    )

    first_gap = None
    continuous = 0.0

    if ahead:
        first_start, first_end = ahead[0]

        first_gap = max(0.0, first_start - ego)

        if first_gap <= eps:
            continuous = first_end - ego

    return ForwardLaneRouteCoverage(
        forward_window_length_m=forward_length,
        supported_ahead_m=supported,
        coverage_ratio_ahead=(
            min(1.0, supported / forward_length)
            if forward_length > 0.0
            else 0.0
        ),
        longest_contiguous_ahead_m=longest,
        continuous_from_ego_m=continuous,
        first_support_gap_m=first_gap,
        supported_intervals_ahead=tuple(ahead),
    )
