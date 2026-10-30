"""Unit tests for ego-relative lane-route spatial coverage."""

import unittest

from global_route_planner.lane_route_coverage import LaneRouteCoverage
from global_route_planner.lane_route_forward_coverage import (
    summarize_forward_coverage,
)
from global_route_planner.route_geometry import RouteSample, RouteWindow


class TestForwardLaneRouteCoverage(unittest.TestCase):

    def setUp(self):
        self.window = RouteWindow(
            start_s_m=10.0,
            end_s_m=60.0,
            samples=(
                RouteSample(10.0, 10.0, 0.0, 0.0, 0),
                RouteSample(60.0, 60.0, 0.0, 0.0, 0),
            ),
        )

    def summarize(self, intervals, ego=20.0):
        intervals = tuple(intervals)

        coverage = LaneRouteCoverage(
            window_length_m=50.0,
            supported_length_m=sum(b - a for a, b in intervals),
            coverage_ratio=0.0,
            longest_contiguous_m=0.0,
            supported_intervals=intervals,
            valid_sample_count=0,
            connected_pair_count=0,
        )

        return summarize_forward_coverage(
            coverage,
            self.window,
            ego,
        )

    def test_full_forward_support(self):
        result = self.summarize([(10, 60)])

        self.assertEqual(result.supported_ahead_m, 40)
        self.assertEqual(result.continuous_from_ego_m, 40)
        self.assertEqual(result.coverage_ratio_ahead, 1.0)
        self.assertEqual(result.first_support_gap_m, 0)

    def test_support_starts_ahead(self):
        result = self.summarize([(25, 45)])

        self.assertEqual(result.supported_ahead_m, 20)
        self.assertEqual(result.first_support_gap_m, 5)
        self.assertEqual(result.continuous_from_ego_m, 0)
        self.assertEqual(result.coverage_ratio_ahead, 0.5)

    def test_ego_inside_interval(self):
        result = self.summarize([(15, 35)])

        self.assertEqual(
            result.supported_intervals_ahead,
            ((20, 35),),
        )
        self.assertEqual(result.continuous_from_ego_m, 15)

    def test_disconnected_intervals(self):
        result = self.summarize([
            (15, 25),
            (35, 55),
        ])

        self.assertEqual(result.supported_ahead_m, 25)
        self.assertEqual(result.longest_contiguous_ahead_m, 20)
        self.assertEqual(result.continuous_from_ego_m, 5)

    def test_first_support_gap_when_disconnected(self):
        result = self.summarize([
            (25, 30),
            (40, 50),
        ])

        self.assertEqual(result.first_support_gap_m, 5)
        self.assertEqual(result.longest_contiguous_ahead_m, 10)
        self.assertEqual(result.continuous_from_ego_m, 0)

    def test_empty_coverage(self):
        result = self.summarize([])

        self.assertEqual(result.supported_ahead_m, 0)
        self.assertEqual(result.continuous_from_ego_m, 0)
        self.assertIsNone(result.first_support_gap_m)

    def test_all_support_behind(self):
        result = self.summarize([(10, 15)])

        self.assertEqual(result.supported_ahead_m, 0)
        self.assertEqual(result.supported_intervals_ahead, ())
        self.assertIsNone(result.first_support_gap_m)

    def test_ego_at_interval_end(self):
        result = self.summarize([
            (15, 20),
            (30, 40),
        ])

        self.assertEqual(result.first_support_gap_m, 10)
        self.assertEqual(result.continuous_from_ego_m, 0)

    def test_ego_at_window_end(self):
        result = self.summarize([(10, 60)], ego=60)

        self.assertEqual(result.forward_window_length_m, 0)
        self.assertEqual(result.coverage_ratio_ahead, 0)
        self.assertEqual(result.supported_ahead_m, 0)

    def test_ego_at_window_start(self):
        result = self.summarize([(10, 30)], ego=10)

        self.assertEqual(result.forward_window_length_m, 50)
        self.assertEqual(result.continuous_from_ego_m, 20)

    def test_adjacent_intervals(self):
        result = self.summarize([
            (10, 30),
            (30, 45),
        ])

        self.assertEqual(
            result.supported_intervals_ahead,
            ((20, 45),),
        )
        self.assertEqual(result.continuous_from_ego_m, 25)

    def test_preserves_global_route_coordinates(self):
        result = self.summarize([
            (15, 30),
            (40, 55),
        ], ego=22)

        self.assertEqual(
            result.supported_intervals_ahead,
            ((22, 30), (40, 55)),
        )

    def test_rejects_nonfinite_ego(self):
        with self.assertRaises(ValueError):
            self.summarize([], ego=float("nan"))

    def test_rejects_ego_outside_window(self):
        for ego in (9, 61):
            with self.subTest(ego=ego):
                with self.assertRaises(ValueError):
                    self.summarize([], ego=ego)

    def test_rejects_overlapping_intervals(self):
        with self.assertRaises(ValueError):
            self.summarize([
                (15, 30),
                (25, 40),
            ])

    def test_rejects_unordered_intervals(self):
        with self.assertRaises(ValueError):
            self.summarize([
                (40, 45),
                (15, 25),
            ])

    def test_rejects_outside_window(self):
        with self.assertRaises(ValueError):
            self.summarize([(5, 20)])

    def test_rejects_nonfinite_interval(self):
        with self.assertRaises(ValueError):
            self.summarize([
                (15, float("inf")),
            ])

    def test_rejects_mismatched_window_length(self):
        coverage = LaneRouteCoverage(
            window_length_m=40.0,
            supported_length_m=0.0,
            coverage_ratio=0.0,
            longest_contiguous_m=0.0,
            supported_intervals=(),
            valid_sample_count=0,
            connected_pair_count=0,
        )

        with self.assertRaises(ValueError):
            summarize_forward_coverage(
                coverage,
                self.window,
                20.0,
            )


if __name__ == "__main__":
    unittest.main()
