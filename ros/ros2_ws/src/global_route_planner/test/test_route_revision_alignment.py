"""Tests for conservative world-coordinate route alignment."""

import math
import unittest

from global_route_planner.route_revision_alignment import (
    RouteAlignmentParameters,
    compare_route_revisions,
)


class TestRouteRevisionAlignment(unittest.TestCase):

    def compare(self, old, new, params=None):
        return compare_route_revisions(old, new, params)

    def test_same_route(self):
        result = self.compare(
            [(0, 0), (50, 0)],
            [(0, 0), (50, 0)],
        )

        self.assertEqual(result.status, "ALIGNED")
        self.assertAlmostEqual(result.matched_prefix_m, 30)

    def test_advanced_start(self):
        result = self.compare(
            [(0, 0), (50, 0)],
            [(12, 0), (50, 0)],
        )

        self.assertEqual(result.status, "ALIGNED")
        self.assertAlmostEqual(result.previous_start_s_m, 12)
        self.assertAlmostEqual(result.previous_end_s_m, 42)

    def test_turn_at_new_start(self):
        result = self.compare(
            [(0, 0), (10, 0), (10, 30)],
            [(10, 0), (10, 30)],
        )

        self.assertEqual(result.status, "ALIGNED")
        self.assertAlmostEqual(result.previous_start_s_m, 10)

    def test_mismatched_heading(self):
        result = self.compare(
            [(0, 0), (50, 0)],
            [(40, 0), (0, 0)],
        )

        self.assertEqual(result.status, "DIVERGED")

    def test_parallel_offset(self):
        result = self.compare(
            [(0, 0), (50, 0)],
            [(0, 3), (50, 3)],
        )

        self.assertEqual(result.status, "DIVERGED")

    def test_small_position_noise(self):
        result = self.compare(
            [(0, 0), (50, 0)],
            [(0, 0.15), (50, 0.15)],
        )

        self.assertEqual(result.status, "ALIGNED")
        self.assertAlmostEqual(
            result.max_observed_position_error_m,
            0.15,
        )

    def test_divergent_turn(self):
        result = self.compare(
            [(0, 0), (50, 0)],
            [(10, 0), (20, 0), (20, 30)],
        )

        self.assertEqual(result.status, "DIVERGED")
        self.assertGreaterEqual(result.matched_prefix_m, 9.0)

    def test_short_new_route(self):
        result = self.compare(
            [(0, 0), (50, 0)],
            [(43, 0), (50, 0)],
        )

        self.assertEqual(result.status, "INSUFFICIENT_OVERLAP")

    def test_crossing_only(self):
        result = self.compare(
            [(-20, 0), (20, 0)],
            [(0, -20), (0, 20)],
        )

        self.assertEqual(result.status, "DIVERGED")

    def test_change_after_checked_horizon(self):
        result = self.compare(
            [(0, 0), (80, 0)],
            [(10, 0), (50, 0), (50, 10)],
        )

        self.assertEqual(result.status, "ALIGNED")
        self.assertAlmostEqual(result.checked_length_m, 30)

    def test_duplicate_vertices(self):
        result = self.compare(
            [(0, 0), (0, 0), (50, 0)],
            [(10, 0), (10, 0), (40, 0)],
        )

        self.assertEqual(result.status, "ALIGNED")

    def test_invalid_route(self):
        cases = [
            ([(0, 0)], [(0, 0), (20, 0)]),
            ([(0, 0), (20, 0)], [(0, 0), (0, 0)]),
            ([(0, 0), (math.inf, 0)], [(0, 0), (20, 0)]),
        ]

        for old, new in cases:
            with self.subTest(old=old, new=new):
                with self.assertRaises(ValueError):
                    self.compare(old, new)

    def test_invalid_parameters(self):
        cases = [
            {"sample_spacing_m": 0.0},
            {"max_heading_error_deg": 200.0},
            {"min_overlap_m": 31.0},
            {"max_position_error_m": 0.0},
            {"max_check_length_m": float("nan")},
            {"min_progress_fraction": 0.0},
        ]

        for kwargs in cases:
            with self.subTest(kwargs=kwargs):
                with self.assertRaises(ValueError):
                    RouteAlignmentParameters(**kwargs)

    def test_previous_route_ends_too_soon(self):
        result = self.compare(
            [(0, 0), (12, 0)],
            [(10, 0), (30, 0)],
        )

        self.assertEqual(result.status, "INSUFFICIENT_OVERLAP")

    def test_backtracking(self):
        result = self.compare(
            [(0, 0), (50, 0)],
            [(0, 0), (8, 0), (7, 0), (30, 0)],
        )

        self.assertEqual(result.status, "DIVERGED")


if __name__ == "__main__":
    unittest.main()
