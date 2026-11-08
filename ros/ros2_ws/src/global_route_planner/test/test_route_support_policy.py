"""Tests for the ROS-independent route-support policy."""

import unittest

from global_route_planner.lane_route_decision import AssociationDecision
from global_route_planner.lane_route_forward_coverage import (
    ForwardLaneRouteCoverage,
)
from global_route_planner.route_support_policy import (
    RouteSupportThresholds,
    evaluate_route_support,
)


class TestRouteSupportPolicy(unittest.TestCase):

    def setUp(self):
        self.associated = AssociationDecision(
            status="ASSOCIATED",
            reason="GEOMETRY_AND_CONFIDENCE_VALID",
        )

    def coverage(self, intervals=((0.0, 25.0),), horizon=50.0):
        intervals = tuple(intervals)

        supported = sum(
            end - start
            for start, end in intervals
        )

        first_gap = intervals[0][0] if intervals else None

        continuous = (
            intervals[0][1] - intervals[0][0]
            if intervals and first_gap == 0.0
            else 0.0
        )

        return ForwardLaneRouteCoverage(
            forward_window_length_m=horizon,
            supported_ahead_m=supported,
            coverage_ratio_ahead=(
                supported / horizon if horizon > 0.0 else 0.0
            ),
            longest_contiguous_ahead_m=max(
                (end - start for start, end in intervals),
                default=0.0,
            ),
            continuous_from_ego_m=continuous,
            first_support_gap_m=first_gap,
            supported_intervals_ahead=intervals,
        )

    def evaluate(self, coverage, association=None, thresholds=None):
        return evaluate_route_support(
            association or self.associated,
            coverage,
            thresholds,
        )

    def test_continuous_support(self):
        result = self.evaluate(self.coverage())

        self.assertEqual(result.status, "SUPPORTED")
        self.assertFalse(result.initial_gap_tolerated)
        self.assertEqual(result.near_ego_interval_m, 25.0)

    def test_small_initial_gap_is_tolerated(self):
        coverage = self.coverage(((0.082, 25.582),))

        result = self.evaluate(coverage)

        self.assertEqual(result.status, "SUPPORTED")
        self.assertTrue(result.initial_gap_tolerated)
        self.assertAlmostEqual(result.near_ego_interval_m, 25.5)

        # Original strict geometric measurement remains unchanged.
        self.assertEqual(coverage.continuous_from_ego_m, 0.0)
        self.assertAlmostEqual(coverage.first_support_gap_m, 0.082)

    def test_gap_at_threshold(self):
        result = self.evaluate(
            self.coverage(((0.5, 25.5),))
        )

        self.assertEqual(result.status, "SUPPORTED")

    def test_large_initial_gap(self):
        result = self.evaluate(
            self.coverage(((1.621, 26.622),))
        )

        self.assertEqual(result.status, "PARTIAL")
        self.assertEqual(result.reason, "INITIAL_GAP_TOO_LARGE")

    def test_insufficient_near_ego_support(self):
        result = self.evaluate(
            self.coverage(((0.0, 1.996), (5.0, 27.211)))
        )

        self.assertEqual(result.status, "PARTIAL")
        self.assertEqual(
            result.reason,
            "INSUFFICIENT_NEAR_EGO_SUPPORT",
        )

    def test_insufficient_total_support(self):
        result = self.evaluate(
            self.coverage(((0.0, 10.0),))
        )

        self.assertEqual(result.status, "PARTIAL")
        self.assertEqual(
            result.reason,
            "INSUFFICIENT_TOTAL_SUPPORT",
        )

    def test_low_forward_coverage_ratio(self):
        result = self.evaluate(
            self.coverage(((0.0, 20.0),), horizon=100.0)
        )

        self.assertEqual(result.status, "PARTIAL")
        self.assertEqual(result.reason, "LOW_FORWARD_COVERAGE")

    def test_disconnected_but_near_ego_support_is_sufficient(self):
        result = self.evaluate(
            self.coverage(((0.0, 16.721), (25.0, 30.498)))
        )

        self.assertEqual(result.status, "SUPPORTED")
        self.assertEqual(result.near_ego_interval_m, 16.721)

    def test_disconnected_support_with_short_first_interval(self):
        result = self.evaluate(
            self.coverage(((0.0, 2.0), (10.0, 35.0)))
        )

        self.assertEqual(result.status, "PARTIAL")
        self.assertEqual(
            result.reason,
            "INSUFFICIENT_NEAR_EGO_SUPPORT",
        )

    def test_no_forward_support(self):
        result = self.evaluate(self.coverage(()))

        self.assertEqual(result.status, "UNAVAILABLE")
        self.assertEqual(result.reason, "NO_FORWARD_SUPPORT")

    def test_missing_coverage_measurement(self):
        result = self.evaluate(None)

        self.assertEqual(result.status, "UNAVAILABLE")
        self.assertEqual(result.reason, "NO_COVERAGE_MEASUREMENT")

    def test_zero_forward_window(self):
        result = self.evaluate(
            self.coverage((), horizon=0.0)
        )

        self.assertEqual(result.status, "UNAVAILABLE")
        self.assertEqual(result.reason, "ZERO_FORWARD_WINDOW")

    def test_unavailable_association_takes_precedence(self):
        association = AssociationDecision(
            status="UNAVAILABLE",
            reason="LOW_PERCEPTION_CONFIDENCE",
        )

        result = self.evaluate(
            self.coverage(),
            association,
        )

        self.assertEqual(result.status, "UNAVAILABLE")
        self.assertEqual(
            result.reason,
            "LOW_PERCEPTION_CONFIDENCE",
        )

    def test_rejected_association_takes_precedence(self):
        association = AssociationDecision(
            status="REJECTED",
            reason="EXCESSIVE_GEOMETRIC_DISTANCE",
        )

        result = self.evaluate(
            self.coverage(),
            association,
        )

        self.assertEqual(result.status, "REJECTED")

    def test_invalid_thresholds(self):
        with self.assertRaises(ValueError):
            RouteSupportThresholds(
                max_initial_gap_m=-1.0
            )

        with self.assertRaises(ValueError):
            RouteSupportThresholds(
                min_coverage_ratio_ahead=1.5
            )

    def test_nonfinite_coverage_is_rejected(self):
        coverage = self.coverage()

        invalid = ForwardLaneRouteCoverage(
            forward_window_length_m=50.0,
            supported_ahead_m=float("nan"),
            coverage_ratio_ahead=coverage.coverage_ratio_ahead,
            longest_contiguous_ahead_m=25.0,
            continuous_from_ego_m=25.0,
            first_support_gap_m=0.0,
            supported_intervals_ahead=((0.0, 25.0),),
        )

        with self.assertRaises(ValueError):
            self.evaluate(invalid)

    def test_inconsistent_support_length_is_rejected(self):
        invalid = ForwardLaneRouteCoverage(
            forward_window_length_m=50.0,
            supported_ahead_m=30.0,
            coverage_ratio_ahead=0.6,
            longest_contiguous_ahead_m=25.0,
            continuous_from_ego_m=25.0,
            first_support_gap_m=0.0,
            supported_intervals_ahead=((0.0, 25.0),),
        )

        with self.assertRaises(ValueError):
            self.evaluate(invalid)

    def test_unknown_association_status_is_rejected(self):
        association = AssociationDecision(
            status="UNKNOWN",
            reason="INVALID",
        )

        with self.assertRaises(ValueError):
            self.evaluate(
                self.coverage(),
                association,
            )


if __name__ == "__main__":
    unittest.main()
