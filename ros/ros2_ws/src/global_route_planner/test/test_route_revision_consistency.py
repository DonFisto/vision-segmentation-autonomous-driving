"""Tests for combined route identity and geometric consistency."""

import math
import unittest

from global_route_planner.route_identity import RouteIdentity
from global_route_planner.route_revision_consistency import (
    assess_route_revision_consistency,
)


class TestRouteRevisionConsistency(unittest.TestCase):

    def setUp(self):
        self.old = [(0.0, 0.0), (60.0, 0.0)]
        self.new = [(12.0, 0.0), (60.0, 0.0)]

    def identity(self, mission=10, route=1, map_revision=7):
        return RouteIdentity(
            mission_id=mission,
            route_id=route,
            map_revision=map_revision,
        )

    def assess(self, old_id, new_id, old=None, new=None):
        return assess_route_revision_consistency(
            previous_identity=old_id,
            previous_points=(
                self.old if old is None and old_id is not None
                else old
            ),
            current_identity=new_id,
            current_points=self.new if new is None else new,
        )

    def test_initial_route(self):
        result = self.assess(None, self.identity())

        self.assertEqual(result.status, "INITIAL_ROUTE")
        self.assertTrue(result.accept_current_route)
        self.assertEqual(result.history_action, "RESET")

    def test_same_route(self):
        identity = self.identity()
        result = self.assess(
            identity, identity,
            old=self.old, new=list(self.old),
        )

        self.assertEqual(result.status, "SAME_ROUTE")
        self.assertTrue(result.accept_current_route)
        self.assertEqual(result.history_action, "KEEP")
        self.assertTrue(result.reuse_route_coordinates)

    def test_same_id_changed_geometry(self):
        identity = self.identity()
        result = self.assess(identity, identity)

        self.assertEqual(
            result.status,
            "ROUTE_ID_GEOMETRY_CONFLICT",
        )
        self.assertFalse(result.accept_current_route)
        self.assertEqual(result.history_action, "KEEP")

    def test_aligned_revision(self):
        result = self.assess(
            self.identity(route=1),
            self.identity(route=2),
        )

        self.assertEqual(result.status, "ROUTE_REVISED_ALIGNED")
        self.assertTrue(result.accept_current_route)
        self.assertEqual(result.history_action, "RESET")
        self.assertFalse(result.reuse_route_coordinates)
        self.assertTrue(
            result.geometry_correspondence_candidate
        )
        self.assertAlmostEqual(
            result.alignment.previous_start_s_m,
            12.0,
        )

    def test_identical_geometry_new_revision_still_resets(self):
        result = self.assess(
            self.identity(route=1),
            self.identity(route=2),
            old=self.old,
            new=list(self.old),
        )

        self.assertEqual(result.status, "ROUTE_REVISED_ALIGNED")
        self.assertEqual(result.history_action, "RESET")
        self.assertFalse(result.reuse_route_coordinates)

    def test_diverged_revision(self):
        result = self.assess(
            self.identity(route=1),
            self.identity(route=2),
            new=[(12.0, 0.0), (12.0, 40.0)],
        )

        self.assertEqual(result.status, "ROUTE_REVISED_DIVERGED")
        self.assertFalse(
            result.geometry_correspondence_candidate
        )

    def test_insufficient_overlap(self):
        result = self.assess(
            self.identity(route=1),
            self.identity(route=2),
            old=[(0.0, 0.0), (15.0, 0.0)],
            new=[(12.0, 0.0), (40.0, 0.0)],
        )

        self.assertEqual(result.status, "ROUTE_REVISED_UNCERTAIN")
        self.assertEqual(
            result.alignment.status,
            "INSUFFICIENT_OVERLAP",
        )

    def test_mission_changed(self):
        result = self.assess(
            self.identity(mission=10),
            self.identity(mission=11, route=2),
        )

        self.assertEqual(result.status, "MISSION_CHANGED")
        self.assertEqual(result.history_action, "RESET")
        self.assertIsNone(result.alignment)

    def test_map_changed(self):
        result = self.assess(
            self.identity(map_revision=7),
            self.identity(map_revision=8, route=2),
        )

        self.assertEqual(result.status, "MAP_CHANGED")
        self.assertEqual(result.history_action, "RESET")
        self.assertIsNone(result.alignment)

    def test_out_of_order_revision(self):
        result = self.assess(
            self.identity(route=20),
            self.identity(route=19),
        )

        self.assertEqual(result.status, "OUT_OF_ORDER_REVISION")
        self.assertFalse(result.accept_current_route)
        self.assertEqual(result.history_action, "KEEP")

    def test_previous_geometry_without_identity(self):
        with self.assertRaises(ValueError):
            assess_route_revision_consistency(
                None, self.old,
                self.identity(), self.new,
            )

    def test_missing_previous_geometry(self):
        with self.assertRaises(ValueError):
            assess_route_revision_consistency(
                self.identity(), None,
                self.identity(route=2), self.new,
            )

    def test_nonfinite_current_geometry(self):
        with self.assertRaises(ValueError):
            self.assess(
                None,
                self.identity(),
                new=[(0.0, 0.0), (math.nan, 1.0)],
            )

    def test_zero_length_current_geometry(self):
        with self.assertRaises(ValueError):
            self.assess(
                None,
                self.identity(),
                new=[(0.0, 0.0), (0.0, 0.0)],
            )

    def test_invalid_previous_geometry(self):
        with self.assertRaises(ValueError):
            self.assess(
                self.identity(route=1),
                self.identity(route=2),
                old=[(0.0, 0.0)],
            )

    def test_invalid_identity(self):
        with self.assertRaises(TypeError):
            assess_route_revision_consistency(
                None, None, None, self.new,
            )

    def test_invalid_alignment_parameters(self):
        with self.assertRaises(TypeError):
            assess_route_revision_consistency(
                self.identity(route=1),
                self.old,
                self.identity(route=2),
                self.new,
                alignment_parameters="invalid",
            )

    def test_alignment_only_covers_checked_prefix(self):
        result = self.assess(
            self.identity(route=1),
            self.identity(route=2),
            old=[(0.0, 0.0), (100.0, 0.0)],
            new=[
                (10.0, 0.0),
                (50.0, 0.0),
                (50.0, 20.0),
            ],
        )

        self.assertEqual(result.status, "ROUTE_REVISED_ALIGNED")
        self.assertAlmostEqual(
            result.alignment.checked_length_m,
            30.0,
        )
        self.assertTrue(
            result.geometry_correspondence_candidate
        )


if __name__ == "__main__":
    unittest.main()
