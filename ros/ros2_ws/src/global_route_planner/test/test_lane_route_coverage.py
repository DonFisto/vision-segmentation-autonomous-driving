"""Unit tests for lane-route spatial coverage."""

import math
import unittest

from global_route_planner.route_geometry import (
    RouteWindow,
    RouteSample,
)

from global_route_planner.lane_route_geometry import LanePoint

from global_route_planner.lane_route_coverage import (
    CoverageParameters,
    estimate_lane_route_coverage,
)


class TestLaneRouteCoverage(unittest.TestCase):

    def setUp(self):
        self.window = RouteWindow(
            start_s_m=10.0,
            end_s_m=60.0,
            samples=(
                RouteSample(10.0, 10.0, 0.0, 0.0, 0),
                RouteSample(60.0, 60.0, 0.0, 0.0, 0),
            ),
        )

    def lane(
        self,
        xs,
        y=0.0,
        heading=0.0,
        confidence=0.8,
    ):
        return [
            LanePoint(
                x_m=float(x),
                y_m=float(y),
                heading_rad=heading,
                confidence=confidence,
            )
            for x in xs
        ]

    def estimate(self, points):
        return estimate_lane_route_coverage(
            self.window,
            points,
        )

    def test_contiguous_coverage(self):
        result = self.estimate(
            self.lane(range(15, 26))
        )

        self.assertEqual(
            result.supported_intervals,
            ((15.0, 25.0),),
        )

        self.assertAlmostEqual(
            result.supported_length_m,
            10.0,
        )

        self.assertAlmostEqual(
            result.coverage_ratio,
            0.2,
        )

        self.assertAlmostEqual(
            result.longest_contiguous_m,
            10.0,
        )

    def test_disconnected_coverage(self):
        points = self.lane(
            list(range(15, 21))
            + list(range(40, 46))
        )

        result = self.estimate(points)

        self.assertEqual(
            result.supported_intervals,
            ((15.0, 20.0), (40.0, 45.0)),
        )

        self.assertAlmostEqual(
            result.supported_length_m,
            10.0,
        )

        self.assertAlmostEqual(
            result.longest_contiguous_m,
            5.0,
        )

    def test_empty_lane(self):
        result = self.estimate([])

        self.assertEqual(result.coverage_ratio, 0.0)
        self.assertEqual(result.supported_intervals, ())

    def test_sparse_samples_do_not_bridge_gaps(self):
        result = self.estimate(
            self.lane([15, 25, 35])
        )

        self.assertEqual(
            result.supported_length_m,
            0.0,
        )

    def test_opposite_direction(self):
        result = self.estimate(
            self.lane(
                range(15, 26),
                heading=math.pi,
            )
        )

        self.assertEqual(
            result.supported_length_m,
            0.0,
        )

    def test_reverse_sample_order(self):
        result = self.estimate(
            self.lane(range(25, 14, -1))
        )

        self.assertEqual(
            result.supported_length_m,
            0.0,
        )

    def test_lateral_offset(self):
        result = self.estimate(
            self.lane(
                range(15, 26),
                y=3.0,
            )
        )

        self.assertEqual(
            result.supported_length_m,
            0.0,
        )

    def test_low_confidence(self):
        result = self.estimate(
            self.lane(
                range(15, 26),
                confidence=0.3,
            )
        )

        self.assertEqual(
            result.supported_length_m,
            0.0,
        )

    def test_low_confidence_breaks_coverage(self):
        points = self.lane(range(15, 26))

        points[5] = LanePoint(
            x_m=20.0,
            y_m=0.0,
            heading_rad=0.0,
            confidence=0.1,
        )

        result = self.estimate(points)

        self.assertEqual(
            result.supported_intervals,
            ((15.0, 19.0), (21.0, 25.0)),
        )

    def test_samples_outside_window(self):
        result = self.estimate(
            self.lane(range(0, 71))
        )

        self.assertEqual(
            result.supported_intervals,
            ((10.0, 60.0),),
        )

        self.assertAlmostEqual(
            result.supported_length_m,
            50.0,
        )

    def test_all_samples_outside_window(self):
        result = self.estimate(
            self.lane(range(65, 76))
        )

        self.assertEqual(
            result.supported_length_m,
            0.0,
        )

    def test_preserves_global_route_s(self):
        result = self.estimate(
            self.lane(range(15, 26))
        )

        self.assertEqual(
            result.supported_intervals,
            ((15.0, 25.0),),
        )

    def test_invalid_parameters(self):
        with self.assertRaises(ValueError):
            CoverageParameters(
                max_sample_gap_m=0.0
            )

    def test_nonfinite_lane_sample(self):
        with self.assertRaises(ValueError):
            self.estimate(
                self.lane([15, float("nan")])
            )

    def test_nonfinite_window(self):
        invalid_window = RouteWindow(
            start_s_m=10.0,
            end_s_m=float("inf"),
            samples=self.window.samples,
        )

        with self.assertRaises(ValueError):
            estimate_lane_route_coverage(
                invalid_window,
                [],
            )

    def test_repeated_samples(self):
        result = self.estimate(
            self.lane([15, 16, 16, 17])
        )

        self.assertEqual(
            result.supported_intervals,
            ((15.0, 17.0),),
        )

    def test_disconnected_pairs(self):
        result = self.estimate(
            self.lane([15, 16, 17, 25, 26, 27])
        )

        self.assertEqual(
            result.supported_intervals,
            ((15.0, 17.0), (25.0, 27.0)),
        )


if __name__ == "__main__":
    unittest.main()
