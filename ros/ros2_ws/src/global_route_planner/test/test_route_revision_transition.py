"""Tests for route revision identity transitions."""

import unittest

from global_route_planner.route_identity import (
    RouteIdentity,
    RouteIdentitySequence,
)
from global_route_planner.route_revision_transition import (
    classify_route_transition,
)


class TestRouteRevisionTransition(unittest.TestCase):

    def identity(self, mission=10, route=1, map_revision=7):
        return RouteIdentity(
            mission_id=mission,
            route_id=route,
            map_revision=map_revision,
        )

    def test_initial_route(self):
        result = classify_route_transition(
            None, self.identity()
        )

        self.assertEqual(result.status, "INITIAL_ROUTE")
        self.assertFalse(result.preserve_observations)

    def test_same_route(self):
        route = self.identity()
        result = classify_route_transition(route, route)

        self.assertEqual(result.status, "SAME_ROUTE")
        self.assertTrue(result.preserve_observations)
        self.assertTrue(result.reuse_route_coordinates)

    def test_new_revision(self):
        result = classify_route_transition(
            self.identity(route=1),
            self.identity(route=2),
        )

        self.assertEqual(result.status, "ROUTE_REVISED")
        self.assertFalse(result.preserve_observations)
        self.assertFalse(result.reuse_route_coordinates)
        self.assertTrue(result.requires_geometry_alignment)

    def test_nonconsecutive_revision(self):
        result = classify_route_transition(
            self.identity(route=1),
            self.identity(route=20),
        )

        self.assertEqual(result.status, "ROUTE_REVISED")

    def test_out_of_order_revision(self):
        result = classify_route_transition(
            self.identity(route=20),
            self.identity(route=19),
        )

        self.assertEqual(result.status, "OUT_OF_ORDER_REVISION")
        self.assertFalse(result.preserve_observations)

    def test_new_mission(self):
        result = classify_route_transition(
            self.identity(mission=10),
            self.identity(mission=11, route=2),
        )

        self.assertEqual(result.status, "MISSION_CHANGED")
        self.assertFalse(result.preserve_observations)

    def test_same_route_id_new_mission(self):
        result = classify_route_transition(
            self.identity(mission=10),
            self.identity(mission=11),
        )

        self.assertEqual(result.status, "MISSION_CHANGED")

    def test_map_changed(self):
        result = classify_route_transition(
            self.identity(map_revision=7),
            self.identity(map_revision=8),
        )

        self.assertEqual(result.status, "MAP_CHANGED")
        self.assertFalse(result.reuse_route_coordinates)

    def test_map_change_takes_precedence(self):
        result = classify_route_transition(
            self.identity(mission=10, map_revision=7),
            self.identity(mission=11, map_revision=8),
        )

        self.assertEqual(result.status, "MAP_CHANGED")

    def test_invalid_current_identity(self):
        with self.assertRaises(TypeError):
            classify_route_transition(self.identity(), None)

    def test_invalid_previous_identity(self):
        with self.assertRaises(TypeError):
            classify_route_transition("invalid", self.identity())

    def test_real_sequence_replanning(self):
        sequence = RouteIdentitySequence(
            map_revision=7,
            session_nonce=42,
        )

        sequence.accept_goal()
        first = sequence.issue_route()
        second = sequence.issue_route()

        result = classify_route_transition(first, second)

        self.assertEqual(result.status, "ROUTE_REVISED")
        self.assertTrue(result.requires_geometry_alignment)

    def test_real_sequence_new_goal(self):
        sequence = RouteIdentitySequence(
            map_revision=7,
            session_nonce=42,
        )

        sequence.accept_goal()
        first = sequence.issue_route()

        sequence.accept_goal()
        second = sequence.issue_route()

        result = classify_route_transition(first, second)

        self.assertEqual(result.status, "MISSION_CHANGED")


if __name__ == "__main__":
    unittest.main()
