"""Tests for lane-to-route association validity evaluation."""

import unittest
from dataclasses import replace

from global_route_planner.lane_route_geometry import (
    LaneRouteComparison,
)
from global_route_planner.lane_route_decision import (
    AssociationThresholds,
    evaluate_lane_route_association,
)


class TestLaneRouteDecision(unittest.TestCase):

    def setUp(self):
        self.good_comparison = LaneRouteComparison(
            sample_count=51,
            median_distance_m=0.021,
            median_abs_lateral_m=0.021,
            median_abs_heading_error_deg=0.15,
            median_confidence=0.783,
            min_route_s_m=5.0,
            max_route_s_m=30.0,
            projected_s_span_m=25.0,
        )

    def evaluate(self, **changes):
        comparison = replace(
            self.good_comparison,
            **changes,
        )

        return evaluate_lane_route_association(comparison)

    def test_valid_association(self):
        result = self.evaluate()

        self.assertEqual(result.status, "ASSOCIATED")
        self.assertEqual(
            result.reason,
            "GEOMETRY_AND_CONFIDENCE_VALID",
        )

    def test_missing_perception(self):
        result = evaluate_lane_route_association(None)

        self.assertEqual(result.status, "UNAVAILABLE")
        self.assertEqual(result.reason, "NO_PERCEIVED_LANE")

    def test_low_confidence(self):
        result = self.evaluate(
            median_confidence=0.234,
        )

        self.assertEqual(result.status, "UNAVAILABLE")
        self.assertEqual(
            result.reason,
            "LOW_PERCEPTION_CONFIDENCE",
        )

    def test_low_confidence_takes_priority_over_geometry(self):
        result = self.evaluate(
            median_confidence=0.2,
            median_distance_m=4.0,
        )

        self.assertEqual(result.status, "UNAVAILABLE")
        self.assertEqual(
            result.reason,
            "LOW_PERCEPTION_CONFIDENCE",
        )

    def test_insufficient_samples(self):
        result = self.evaluate(sample_count=1)

        self.assertEqual(result.status, "UNAVAILABLE")
        self.assertEqual(
            result.reason,
            "INSUFFICIENT_SAMPLES",
        )

    def test_insufficient_projected_span(self):
        result = self.evaluate(
            projected_s_span_m=3.0,
        )

        self.assertEqual(result.status, "UNAVAILABLE")
        self.assertEqual(
            result.reason,
            "INSUFFICIENT_PROJECTED_SPAN",
        )

    def test_excessive_geometric_distance(self):
        result = self.evaluate(
            median_distance_m=3.0,
            median_abs_lateral_m=3.0,
        )

        self.assertEqual(result.status, "REJECTED")
        self.assertEqual(
            result.reason,
            "EXCESSIVE_GEOMETRIC_DISTANCE",
        )

    def test_opposite_direction(self):
        result = self.evaluate(
            median_abs_heading_error_deg=180.0,
        )

        self.assertEqual(result.status, "REJECTED")
        self.assertEqual(
            result.reason,
            "EXCESSIVE_HEADING_ERROR",
        )

    def test_distance_rejection_precedes_heading_rejection(self):
        result = self.evaluate(
            median_distance_m=3.0,
            median_abs_heading_error_deg=180.0,
        )

        self.assertEqual(
            result.reason,
            "EXCESSIVE_GEOMETRIC_DISTANCE",
        )

    def test_threshold_boundaries_are_inclusive(self):
        result = self.evaluate(
            sample_count=2,
            median_confidence=0.5,
            projected_s_span_m=6.0,
            median_distance_m=1.0,
            median_abs_heading_error_deg=15.0,
        )

        self.assertEqual(result.status, "ASSOCIATED")

    def test_rejects_invalid_comparison(self):
        with self.assertRaises(ValueError):
            self.evaluate(
                median_distance_m=float("nan"),
            )

    def test_rejects_invalid_thresholds(self):
        with self.assertRaises(ValueError):
            AssociationThresholds(
                min_median_confidence=1.5,
            )


if __name__ == "__main__":
    unittest.main()
