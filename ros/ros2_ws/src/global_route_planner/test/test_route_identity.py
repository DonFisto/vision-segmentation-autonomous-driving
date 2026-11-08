"""Tests for mission and route revision identity."""

import unittest
from dataclasses import FrozenInstanceError

from global_route_planner.route_identity import (
    RouteIdentity,
    RouteIdentitySequence,
    UINT32_MAX,
    UINT64_MAX,
)


class TestRouteIdentity(unittest.TestCase):

    def setUp(self):
        self.ids = RouteIdentitySequence(
            map_revision=7,
            session_nonce=17,
        )

    def test_no_route_before_goal(self):
        with self.assertRaises(RuntimeError):
            self.ids.issue_route()

    def test_first_route(self):
        mission = self.ids.accept_goal()
        route = self.ids.issue_route()

        self.assertEqual(route.mission_id, mission)
        self.assertEqual(route.route_id, 1)

    def test_replans_keep_mission(self):
        self.ids.accept_goal()

        a = self.ids.issue_route()
        b = self.ids.issue_route()

        self.assertEqual(a.mission_id, b.mission_id)
        self.assertEqual(b.route_id, a.route_id + 1)

    def test_new_goal_changes_mission(self):
        self.ids.accept_goal()
        a = self.ids.issue_route()

        self.ids.accept_goal()
        b = self.ids.issue_route()

        self.assertNotEqual(a.mission_id, b.mission_id)

    def test_route_counter_continues_across_goals(self):
        self.ids.accept_goal()
        self.ids.issue_route()
        self.ids.accept_goal()

        self.assertEqual(self.ids.issue_route().route_id, 2)

    def test_session_nonce_disambiguates(self):
        a = RouteIdentitySequence(7, session_nonce=17)
        b = RouteIdentitySequence(7, session_nonce=18)

        self.assertNotEqual(a.accept_goal(), b.accept_goal())

    def test_map_revision_preserved(self):
        self.ids.accept_goal()
        self.assertEqual(self.ids.issue_route().map_revision, 7)

    def test_uint64_value_ranges(self):
        self.ids.accept_goal()
        result = self.ids.issue_route()

        self.assertLessEqual(result.mission_id, UINT64_MAX)
        self.assertLessEqual(result.route_id, UINT64_MAX)

    def test_frozen_result(self):
        self.ids.accept_goal()
        result = self.ids.issue_route()

        with self.assertRaises(FrozenInstanceError):
            result.route_id = 99

    def test_invalid_map_revision(self):
        for value in (-1, True, 1.5, UINT64_MAX + 1):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    RouteIdentitySequence(value, session_nonce=17)

    def test_invalid_nonce(self):
        for value in (0, -1, True, UINT32_MAX + 1):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    RouteIdentitySequence(7, session_nonce=value)

    def test_mission_counter_overflow(self):
        self.ids._mission_counter = UINT32_MAX

        with self.assertRaises(OverflowError):
            self.ids.accept_goal()

    def test_route_counter_overflow(self):
        self.ids.accept_goal()
        self.ids._route_counter = UINT64_MAX

        with self.assertRaises(OverflowError):
            self.ids.issue_route()


if __name__ == "__main__":
    unittest.main()
