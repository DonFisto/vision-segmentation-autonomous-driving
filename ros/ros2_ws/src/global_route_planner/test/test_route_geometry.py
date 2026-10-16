import math
import unittest

from global_route_planner.route_geometry import (
    extract_route_window,
    interpolate_polyline_at_s,
    project_point_to_polyline,
)

class TestRouteGeometry(unittest.TestCase):
    def test_route_window_zero_length(self):
        route = [(0.0, 0.0), (10.0, 0.0)]

        window = extract_route_window(
            route,
            center_s_m=4.0,
            backward_m=0.0,
            forward_m=0.0,
        )

        self.assertAlmostEqual(window.start_s_m, 4.0)
        self.assertAlmostEqual(window.end_s_m, 4.0)

        self.assertEqual(len(window.samples), 1)
        self.assertAlmostEqual(window.samples[0].s_m, 4.0)
        self.assertAlmostEqual(window.samples[0].x, 4.0)

    def test_route_window_preserves_global_s(self):
        route = [
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
        ]

        window = extract_route_window(
            route,
            center_s_m=10.0,
            backward_m=4.0,
            forward_m=5.0,
        )

        self.assertAlmostEqual(window.start_s_m, 6.0)
        self.assertAlmostEqual(window.end_s_m, 15.0)

        self.assertEqual(len(window.samples), 3)

        self.assertEqual(
            [sample.s_m for sample in window.samples],
            [6.0, 10.0, 15.0],
        )

        self.assertAlmostEqual(window.samples[0].x, 6.0)
        self.assertAlmostEqual(window.samples[1].x, 10.0)
        self.assertAlmostEqual(window.samples[2].y, 5.0)

    def test_route_window_clips_start(self):
        route = [(0.0, 0.0), (10.0, 0.0)]

        window = extract_route_window(
            route,
            center_s_m=2.0,
            backward_m=10.0,
            forward_m=3.0,
        )

        self.assertAlmostEqual(window.start_s_m, 0.0)
        self.assertAlmostEqual(window.end_s_m, 5.0)

    def test_route_window_clips_end(self):
        route = [(0.0, 0.0), (10.0, 0.0)]

        window = extract_route_window(
            route,
            center_s_m=9.0,
            backward_m=2.0,
            forward_m=10.0,
        )

        self.assertAlmostEqual(window.start_s_m, 7.0)
        self.assertAlmostEqual(window.end_s_m, 10.0)

    def test_route_window_skips_duplicate_vertices(self):
        route = [
            (0.0, 0.0),
            (5.0, 0.0),
            (5.0, 0.0),
            (10.0, 0.0),
        ]

        window = extract_route_window(
            route,
            center_s_m=5.0,
            backward_m=5.0,
            forward_m=5.0,
        )

        self.assertEqual(
            [sample.s_m for sample in window.samples],
            [0.0, 5.0, 10.0],
        )

    def test_route_window_rejects_invalid_distances(self):
        route = [(0.0, 0.0), (10.0, 0.0)]

        with self.assertRaises(ValueError):
            extract_route_window(route, 5.0, -1.0, 5.0)

        with self.assertRaises(ValueError):
            extract_route_window(route, 5.0, 5.0, -1.0)

        with self.assertRaises(ValueError):
            extract_route_window(route, 15.0, 5.0, 5.0)

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
