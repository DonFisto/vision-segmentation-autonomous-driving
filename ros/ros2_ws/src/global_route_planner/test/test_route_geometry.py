import math
import unittest

from global_route_planner.route_geometry import (
    interpolate_polyline_at_s,
    project_point_to_polyline,
)

class TestRouteGeometry(unittest.TestCase):
    def test_interpolation_on_straight_route(self):
        route = [(0.0, 0.0), (10.0, 0.0)]

        result = interpolate_polyline_at_s(route, 4.0)

        self.assertAlmostEqual(result.x, 4.0)
        self.assertAlmostEqual(result.y, 0.0)
        self.assertAlmostEqual(result.heading_rad, 0.0)

    def test_interpolation_across_segments(self):
        route = [
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
        ]

        result = interpolate_polyline_at_s(route, 14.0)

        self.assertAlmostEqual(result.x, 10.0)
        self.assertAlmostEqual(result.y, 4.0)
        self.assertAlmostEqual(
            result.heading_rad,
            math.pi / 2,
        )

    def test_interpolation_at_vertex(self):
        route = [
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
        ]

        result = interpolate_polyline_at_s(route, 10.0)

        self.assertAlmostEqual(result.x, 10.0)
        self.assertAlmostEqual(result.y, 0.0)
        self.assertAlmostEqual(
            result.heading_rad,
            math.pi / 2,
        )
        self.assertEqual(result.segment_index, 1)

    def test_interpolation_outside_route(self):
        route = [(0.0, 0.0), (10.0, 0.0)]

        with self.assertRaises(ValueError):
            interpolate_polyline_at_s(route, 12.0)

        with self.assertRaises(ValueError):
            interpolate_polyline_at_s(route, -1.0)

    def test_projection_beyond_route_endpoint(self):
        route = [(0.0, 0.0), (10.0, 0.0)]

        result = project_point_to_polyline(
            route, 12.0, 0.0
        )

        self.assertAlmostEqual(result.s_m, 10.0)
        self.assertAlmostEqual(result.d_m, 0.0)
        self.assertAlmostEqual(result.distance_m, 2.0)
        self.assertAlmostEqual(result.projection_x, 10.0)

    def test_projection_beyond_endpoint_with_lateral_error(self):
        route = [(0.0, 0.0), (10.0, 0.0)]

        result = project_point_to_polyline(
            route, 12.0, 3.0
        )

        self.assertAlmostEqual(result.s_m, 10.0)
        self.assertAlmostEqual(result.d_m, 3.0)
        self.assertAlmostEqual(
            result.distance_m,
            math.sqrt(13.0),
        )
    def test_projection_on_straight_route(self):
        route = [(0.0, 0.0), (10.0, 0.0)]

        result = project_point_to_polyline(
            route, 4.0, 2.0
        )

        self.assertAlmostEqual(result.s_m, 4.0)
        self.assertAlmostEqual(result.d_m, 2.0)
        self.assertAlmostEqual(result.distance_m, 2.0)
        self.assertAlmostEqual(result.projection_x, 4.0)
        self.assertAlmostEqual(result.projection_y, 0.0)
        self.assertAlmostEqual(result.heading_rad, 0.0)

    def test_negative_lateral_displacement(self):
        route = [(0.0, 0.0), (10.0, 0.0)]

        result = project_point_to_polyline(
            route, 4.0, -2.0
        )

        self.assertAlmostEqual(result.d_m, -2.0)

    def test_cumulative_arc_length(self):
        route = [
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
        ]

        result = project_point_to_polyline(
            route, 12.0, 4.0
        )

        self.assertAlmostEqual(result.s_m, 14.0)
        self.assertAlmostEqual(result.d_m, -2.0)
        self.assertAlmostEqual(
            result.heading_rad,
            math.pi / 2,
        )
        self.assertEqual(result.segment_index, 1)

    def test_degenerate_polyline(self):
        route = [(1.0, 1.0), (1.0, 1.0)]

        with self.assertRaises(ValueError):
            project_point_to_polyline(
                route, 2.0, 2.0
            )


if __name__ == "__main__":
    unittest.main()
