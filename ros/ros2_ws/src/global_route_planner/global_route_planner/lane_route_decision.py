"""ROS-independent lane-to-route association validity evaluation.

This module evaluates previously measured geometric agreement.
It does not perform geometric projection or access CARLA ground truth.
"""

from dataclasses import dataclass
import math

from global_route_planner.lane_route_geometry import (
    LaneRouteComparison,
)


@dataclass(frozen=True)
class AssociationThresholds:
    """Experimental thresholds for diagnostic association decisions."""

    min_samples: int = 2
    min_median_confidence: float = 0.50
    min_projected_s_span_m: float = 6.0
    max_median_distance_m: float = 1.0
    max_median_heading_error_deg: float = 15.0

    def __post_init__(self):
        if self.min_samples < 2:
            raise ValueError("min_samples must be at least 2")

        numeric_values = (
            self.min_median_confidence,
            self.min_projected_s_span_m,
            self.max_median_distance_m,
            self.max_median_heading_error_deg,
        )

        if not all(math.isfinite(v) for v in numeric_values):
            raise ValueError("Thresholds must be finite")

        if not 0.0 <= self.min_median_confidence <= 1.0:
            raise ValueError("Confidence threshold outside [0, 1]")

        if self.min_projected_s_span_m <= 0.0:
            raise ValueError("Minimum span must be positive")

        if self.max_median_distance_m <= 0.0:
            raise ValueError("Maximum distance must be positive")

        if not 0.0 < self.max_median_heading_error_deg <= 180.0:
            raise ValueError("Heading threshold outside (0, 180]")


@dataclass(frozen=True)
class AssociationDecision:
    """Association state and its explicit diagnostic reason."""

    status: str
    reason: str


def evaluate_lane_route_association(
    comparison: LaneRouteComparison | None,
    thresholds: AssociationThresholds | None = None,
) -> AssociationDecision:
    """Evaluate whether perceived geometry supports a route association.

    ASSOCIATED:
        Sufficient perception evidence agrees with route geometry.

    UNAVAILABLE:
        Perception evidence is missing or insufficient.

    REJECTED:
        Sufficient evidence exists but geometry disagrees.

    Projected longitudinal span is only a preliminary support metric.
    It does not establish continuous route coverage.
    """

    if thresholds is None:
        thresholds = AssociationThresholds()

    if comparison is None:
        return AssociationDecision(
            status="UNAVAILABLE",
            reason="NO_PERCEIVED_LANE",
        )

    numeric_values = (
        comparison.median_distance_m,
        comparison.median_abs_lateral_m,
        comparison.median_abs_heading_error_deg,
        comparison.median_confidence,
        comparison.min_route_s_m,
        comparison.max_route_s_m,
        comparison.projected_s_span_m,
    )

    if not all(math.isfinite(v) for v in numeric_values):
        raise ValueError("Comparison contains non-finite metrics")

    if comparison.sample_count < 0:
        raise ValueError("Negative sample count")

    if (
        comparison.median_distance_m < 0.0
        or comparison.median_abs_lateral_m < 0.0
        or comparison.projected_s_span_m < 0.0
    ):
        raise ValueError("Comparison contains negative distances")

    if not 0.0 <= comparison.median_confidence <= 1.0:
        raise ValueError("Comparison confidence outside [0, 1]")

    if not 0.0 <= comparison.median_abs_heading_error_deg <= 180.0:
        raise ValueError("Comparison heading error outside [0, 180]")

    if comparison.sample_count < thresholds.min_samples:
        return AssociationDecision(
            status="UNAVAILABLE",
            reason="INSUFFICIENT_SAMPLES",
        )

    if comparison.median_confidence < thresholds.min_median_confidence:
        return AssociationDecision(
            status="UNAVAILABLE",
            reason="LOW_PERCEPTION_CONFIDENCE",
        )

    if comparison.projected_s_span_m < thresholds.min_projected_s_span_m:
        return AssociationDecision(
            status="UNAVAILABLE",
            reason="INSUFFICIENT_PROJECTED_SPAN",
        )

    if comparison.median_distance_m > thresholds.max_median_distance_m:
        return AssociationDecision(
            status="REJECTED",
            reason="EXCESSIVE_GEOMETRIC_DISTANCE",
        )

    if (
        comparison.median_abs_heading_error_deg
        > thresholds.max_median_heading_error_deg
    ):
        return AssociationDecision(
            status="REJECTED",
            reason="EXCESSIVE_HEADING_ERROR",
        )

    return AssociationDecision(
        status="ASSOCIATED",
        reason="GEOMETRY_AND_CONFIDENCE_VALID",
    )
