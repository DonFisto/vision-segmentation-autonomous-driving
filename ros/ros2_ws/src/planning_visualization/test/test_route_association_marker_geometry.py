"""Tests for route-interval visualization geometry."""

import unittest

from planning_visualization.route_association_marker_geometry import (
    clip_intervals,
    slice_polyline_xy,
    polyline_length_xy,
)


class TestArcSlicing(unittest.TestCase):

    def test_length(self):
        self.assertEqual(
            polyline_length_xy([(0, 0, 0), (3, 4, 5)]),
            5,
        )

    def test_straight_interval(self):
        points = [(0, 0, 0), (10, 0, 0), (20, 0, 0)]

        self.assertEqual(
            slice_polyline_xy(points, 5, 15),
            [(5., 0., 0.), (10., 0., 0.), (15., 0., 0.)],
        )

    def test_clipping(self):
        self.assertEqual(
            slice_polyline_xy(
                [(0, 0, 0), (10, 0, 0)],
                -5,
                2,
            ),
            [(0., 0., 0.), (2., 0., 0.)],
        )

    def test_no_support_across_gap(self):
        intervals = clip_intervals(
            [0, 12],
            [5, 18],
            0,
            20,
        )

        self.assertEqual(
            intervals,
            [(0., 5.), (12., 18.)],
        )

    def test_reject_overlapping_intervals(self):
        self.assertEqual(
            clip_intervals([0, 3], [5, 8], 0, 10),
            [],
        )

    def test_reject_mismatched_intervals(self):
        self.assertEqual(
            clip_intervals([0, 5], [4], 0, 10),
            [],
        )


if __name__ == "__main__":
    unittest.main()
