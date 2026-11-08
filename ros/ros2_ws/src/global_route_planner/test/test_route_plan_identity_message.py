"""Verify mission identity propagation through RoutePlan messages."""

import unittest
from types import SimpleNamespace
from unittest.mock import patch

from autonomy_interfaces.msg import RoutePlan
from builtin_interfaces.msg import Time
from rclpy.serialization import serialize_message, deserialize_message

from global_route_planner.route_identity import RouteIdentitySequence
from global_route_planner.route_plan_builder import (
    build_route_plan_message,
    build_invalid_route_plan_message,
)


class TestRoutePlanIdentityMessage(unittest.TestCase):

    def setUp(self):
        self.ids = RouteIdentitySequence(
            map_revision=17,
            session_nonce=42,
        )
        self.ids.accept_goal()

        self.start = SimpleNamespace(
            projected_x=0.0,
            projected_y=0.0,
            projected_z=0.0,
            projected_yaw_deg=0.0,
            projection_distance_m=0.1,
            sample_s_error_m=0.1,
        )

        self.goal = SimpleNamespace(
            projected_x=20.0,
            projected_y=0.0,
            projected_z=0.0,
            projected_yaw_deg=0.0,
            projection_distance_m=0.1,
            sample_s_error_m=0.1,
        )

    def build_valid(self, identity):
        graph = SimpleNamespace(
            nodes={
                1: SimpleNamespace(
                    x=0.0, y=0.0, z=0.0, yaw_deg=0.0
                ),
                2: SimpleNamespace(
                    x=20.0, y=0.0, z=0.0, yaw_deg=0.0
                ),
            }
        )

        route = SimpleNamespace(
            node_path=(1, 2),
            edge_path=(),
        )

        with patch(
            "global_route_planner.route_plan_builder.route_lane_segment_ids",
            return_value=[],
        ):
            return build_route_plan_message(
                graph=graph,
                route=route,
                start_association=self.start,
                goal_association=self.goal,
                mission_id=identity.mission_id,
                route_id=identity.route_id,
                map_revision=identity.map_revision,
                stamp=Time(sec=123),
                frame_id="carla_world",
                max_projection_distance_m=5.0,
                max_sample_s_error_m=2.5,
            )

    def test_valid_route_identity(self):
        identity = self.ids.issue_route()
        message = self.build_valid(identity)

        self.assertEqual(message.mission_id, identity.mission_id)
        self.assertEqual(message.route_id, identity.route_id)
        self.assertEqual(message.map_revision, identity.map_revision)
        self.assertEqual(message.status, RoutePlan.STATUS_VALID)

    def test_invalid_route_preserves_mission(self):
        identity = self.ids.issue_route()

        message = build_invalid_route_plan_message(
            start_association=self.start,
            goal_association=self.goal,
            mission_id=identity.mission_id,
            route_id=identity.route_id,
            map_revision=identity.map_revision,
            stamp=Time(sec=123),
            frame_id="carla_world",
        )

        self.assertEqual(message.mission_id, identity.mission_id)
        self.assertEqual(message.route_id, identity.route_id)
        self.assertEqual(message.status, RoutePlan.STATUS_INVALID)

    def test_serialization_round_trip(self):
        identity = self.ids.issue_route()
        original = self.build_valid(identity)

        encoded = serialize_message(original)
        restored = deserialize_message(encoded, RoutePlan)

        self.assertEqual(restored.mission_id, identity.mission_id)
        self.assertEqual(restored.route_id, identity.route_id)
        self.assertEqual(restored.map_revision, identity.map_revision)


if __name__ == "__main__":
    unittest.main()
