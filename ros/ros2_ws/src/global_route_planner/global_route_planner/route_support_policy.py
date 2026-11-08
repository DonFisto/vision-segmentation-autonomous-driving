"""ROS-independent diagnostic policy for perceived route support.

Consumes previously calculated association and forward-coverage results.

Does not alter geometric evidence, authorize motion, or determine
whether the vehicle can safely execute a trajectory.
"""

from dataclasses import dataclass
import math

from global_route_planner.lane_route_decision import AssociationDecision
from global_route_planner.lane_route_forward_coverage import (
    ForwardLaneRouteCoverage,
)


@dataclass(frozen=True)
class RouteSupportThresholds:
    """Experimental thresholds; not calibrated safety limits."""

    min_supported_ahead_m: float = 12.0
    min_near_ego_interval_m: float = 8.0
    max_initial_gap_m: float = 0.5
    min_coverage_ratio_ahead: float = 0.25

    def __post_init__(self):
        values = (
            self.min_supported_ahead_m,
            self.min_near_ego_interval_m,
            self.max_initial_gap_m,
            self.min_coverage_ratio_ahead,
        )

        if not all(math.isfinite(v) for v in values):
            raise ValueError("Support thresholds must be finite")

        if self.min_supported_ahead_m <= 0.0:
            raise ValueError("Minimum total support must be positive")

        if self.min_near_ego_interval_m <= 0.0:
            raise ValueError("Minimum near-ego support must be positive")

        if self.max_initial_gap_m < 0.0:
            raise ValueError("Initial gap tolerance cannot be negative")

        if not 0.0 <= self.min_coverage_ratio_ahead <= 1.0:
            raise ValueError("Coverage ratio threshold outside [0, 1]")


@dataclass(frozen=True)
class RouteSupportDecision:
    """Diagnostic interpretation of perceived route support."""

    status: str
    reason: str
    near_ego_interval_m: float
    initial_gap_tolerated: bool


def evaluate_route_support(
    association: AssociationDecision,
    coverage: ForwardLaneRouteCoverage | None,
    thresholds: RouteSupportThresholds | None = None,
) -> RouteSupportDecision:
    """Evaluate local route support without modifying measurements.

    Decision precedence:
      1. Preserve rejected or unavailable association.
      2. Require a valid forward-coverage measurement.
      3. Require support near the ego.
      4. Require sufficient extent and forward coverage.

    A tolerated initial gap does not become geometric support.
    The near-ego interval length excludes the initial gap.
    """

    params = thresholds or RouteSupportThresholds()

    if association.status not in (
        "ASSOCIATED",
        "UNAVAILABLE",
        "REJECTED",
    ):
        raise ValueError("Unknown association status")

    if association.status == "REJECTED":
        return RouteSupportDecision(
            status="REJECTED",
            reason=association.reason,
            near_ego_interval_m=0.0,
            initial_gap_tolerated=False,
        )

    if association.status == "UNAVAILABLE":
        return RouteSupportDecision(
            status="UNAVAILABLE",
            reason=association.reason,
            near_ego_interval_m=0.0,
            initial_gap_tolerated=False,
        )

    if coverage is None:
        return RouteSupportDecision(
            status="UNAVAILABLE",
            reason="NO_COVERAGE_MEASUREMENT",
            near_ego_interval_m=0.0,
            initial_gap_tolerated=False,
        )

    values = (
        coverage.forward_window_length_m,
        coverage.supported_ahead_m,
        coverage.coverage_ratio_ahead,
        coverage.longest_contiguous_ahead_m,
        coverage.continuous_from_ego_m,
    )

    if not all(math.isfinite(v) for v in values):
        raise ValueError("Non-finite forward-coverage metric")

    forward_length = coverage.forward_window_length_m
    supported = coverage.supported_ahead_m
    ratio = coverage.coverage_ratio_ahead
    continuous = coverage.continuous_from_ego_m
    longest = coverage.longest_contiguous_ahead_m

    eps = 1e-6

    if (
        forward_length < 0.0
        or supported < 0.0
        or longest < 0.0
        or continuous < 0.0
        or supported > forward_length + eps
        or longest > supported + eps
        or continuous > supported + eps
        or not 0.0 <= ratio <= 1.0
    ):
        raise ValueError("Invalid forward-coverage metrics")

    intervals = coverage.supported_intervals_ahead
    gap = coverage.first_support_gap_m

    if intervals:
        if gap is None or not math.isfinite(gap):
            raise ValueError("Missing or non-finite first support gap")

        if gap < 0.0 or gap > forward_length + eps:
            raise ValueError("Invalid first support gap")

        previous_end = None
        interval_total = 0.0

        for start, end in intervals:
            if not (math.isfinite(start) and math.isfinite(end)):
                raise ValueError("Non-finite support interval")

            if end <= start:
                raise ValueError("Invalid support interval")

            if previous_end is not None and start < previous_end - eps:
                raise ValueError("Overlapping or unordered intervals")

            interval_total += end - start
            previous_end = end

        if abs(interval_total - supported) > eps:
            raise ValueError("Support length disagrees with intervals")

    elif gap is not None or supported > eps:
        raise ValueError("Missing intervals or inconsistent gap")

    if forward_length <= 0.0:
        return RouteSupportDecision(
            status="UNAVAILABLE",
            reason="ZERO_FORWARD_WINDOW",
            near_ego_interval_m=0.0,
            initial_gap_tolerated=False,
        )

    if not intervals or supported <= 0.0:
        return RouteSupportDecision(
            status="UNAVAILABLE",
            reason="NO_FORWARD_SUPPORT",
            near_ego_interval_m=0.0,
            initial_gap_tolerated=False,
        )

    if gap > params.max_initial_gap_m:
        return RouteSupportDecision(
            status="PARTIAL",
            reason="INITIAL_GAP_TOO_LARGE",
            near_ego_interval_m=0.0,
            initial_gap_tolerated=False,
        )

    # The first interval is eligible only because its initial gap
    # is within the configured tolerance. We do not bridge that gap.
    first_start, first_end = intervals[0]
    near_ego_interval = first_end - first_start
    gap_tolerated = gap > 0.0

    if near_ego_interval < params.min_near_ego_interval_m:
        status = "PARTIAL"
        reason = "INSUFFICIENT_NEAR_EGO_SUPPORT"

    elif supported < params.min_supported_ahead_m:
        status = "PARTIAL"
        reason = "INSUFFICIENT_TOTAL_SUPPORT"

    elif ratio < params.min_coverage_ratio_ahead:
        status = "PARTIAL"
        reason = "LOW_FORWARD_COVERAGE"

    else:
        status = "SUPPORTED"
        reason = "FORWARD_SUPPORT_THRESHOLDS_MET"

    return RouteSupportDecision(
        status=status,
        reason=reason,
        near_ego_interval_m=near_ego_interval,
        initial_gap_tolerated=gap_tolerated,
    )
