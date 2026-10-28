"""Unit tests for perceived lane-to-route geometry comparison."""

import math
import unittest

from global_route_planner.route_geometry import extract_route_window
from global_route_planner.lane_route_geometry import (
    LanePoint,
    compare_lane_to_route,
)


class TestLaneRouteGeometry(unittest.TestCase):

    def setUp(self):
        self.route = [
            (0.0, 0.0),
            (100.0, 0.0),
        ]

        self.window = extract_route_window(
            self.route,
            center_s_m=40.0,
            backward_m=10.0,
            forward_m=30.0,
        )

    def make_lane(self, y=0.0, heading=0.0):
        return [
            LanePoint(
                x_m=x,
                y_m=y,
                heading_rad=heading,
                confidence=0.8,
            )
            for x in (35.0, 40.0, 45.0, 50.0)
        ]

    def test_matching_lane(self):
        result = compare_lane_to_route(
            self.window,
            self.make_lane(),
        )

        self.assertEqual(result.sample_count, 4)
        self.assertAlmostEqual(result.median_distance_m, 0.0)
        self.assertAlmostEqual(result.median_abs_lateral_m, 0.0)
        self.assertAlmostEqual(
            result.median_abs_heading_error_deg,
            0.0,
        )

    def test_parallel_offset_lane(self):
        result = compare_lane_to_route(
            self.window,
            self.make_lane(y=3.0),
        )

        self.assertAlmostEqual(result.median_distance_m, 3.0)
        self.assertAlmostEqual(result.median_abs_lateral_m, 3.0)

    def test_opposite_direction(self):
        result = compare_lane_to_route(
            self.window,
            self.make_lane(heading=math.pi),
        )

        self.assertAlmostEqual(
            result.median_abs_heading_error_deg,
            180.0,
        )

    def test_empty_lane(self):
        result = compare_lane_to_route(
            self.window,
            [],
        )

        self.assertIsNone(result)

    def test_preserves_global_route_s(self):
        result = compare_lane_to_route(
            self.window,
            self.make_lane(),
        )

        self.assertAlmostEqual(result.min_route_s_m, 35.0)
        self.assertAlmostEqual(result.max_route_s_m, 50.0)
        self.assertAlmostEqual(result.projected_s_span_m, 15.0)

    def test_heading_wraparound(self):
        lane = self.make_lane(
            heading=math.radians(359.0),
        )

        result = compare_lane_to_route(self.window, lane)

        self.assertAlmostEqual(
            result.median_abs_heading_error_deg,
            1.0,
        )

    def test_endpoint_distance_not_equal_lateral_error(self):
        lane = [
            LanePoint(
                x_m=75.0,
                y_m=3.0,
                heading_rad=0.0,
                confidence=0.8,
            )
        ]

        result = compare_lane_to_route(self.window, lane)

        # Window ends at x=70.
        self.assertAlmostEqual(
            result.median_distance_m,
            math.sqrt(34.0),
        )

        self.assertAlmostEqual(
            result.median_abs_lateral_m,
            3.0,
        )

    def test_rejects_nonfinite_lane_geometry(self):
        lane = [
            LanePoint(
                x_m=float("nan"),
                y_m=0.0,
                heading_rad=0.0,
                confidence=0.8,
            )
        ]

        with self.assertRaises(ValueError):
            compare_lane_to_route(self.window, lane)

    def test_rejects_invalid_confidence(self):
        lane = [
            LanePoint(
                x_m=40.0,
                y_m=0.0,
                heading_rad=0.0,
                confidence=1.5,
            )
        ]

        with self.assertRaises(ValueError):
            compare_lane_to_route(self.window, lane)


if __name__ == "__main__":
    unittest.main()
