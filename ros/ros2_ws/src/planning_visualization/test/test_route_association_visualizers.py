"""Synthetic ROS2 marker-generation tests."""

import unittest

from autonomy_interfaces.msg import (
    LaneMap,
    LaneSegment,
    LaneSample,
    RoutePlan,
    RouteAssociationStatus,
)

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

from planning_visualization.lane_map_visualizer_node import (
    build_lane_map_markers,
)

from planning_visualization.route_association_visualizer_node import (
    build_association_markers,
)


def make_lane_map():
    message = LaneMap()
    message.header.frame_id = "carla_world"
    message.schema_version = 3
    message.map_session_id = 42
    message.map_revision = 5

    segment = LaneSegment()
    segment.id = 7
    segment.drivable = True

    for x, y in ((0., 3.), (10., 3.), (20., 3.)):
        sample = LaneSample()
        sample.pose.position.x = x
        sample.pose.position.y = y
        sample.pose.orientation.w = 1.0
        sample.confidence = 0.95
        segment.centerline.append(sample)

    message.segments.append(segment)
    return message


def make_route():
    message = RoutePlan()
    message.header.frame_id = "carla_world"
    message.mission_id = 123
    message.route_id = 6
    message.map_revision = 10
    message.status = RoutePlan.STATUS_VALID

    message.coarse_reference_path.header.frame_id = "carla_world"

    for x in (0., 20.):
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = 3.
        pose.pose.orientation.w = 1.
        message.coarse_reference_path.poses.append(pose)

    return message


def make_status():
    message = RouteAssociationStatus()
    message.header.frame_id = "carla_world"

    message.mission_id = 123
    message.route_id = 6
    message.route_map_revision = 10

    message.map_session_id = 42
    message.lane_map_revision = 5

    message.selected_segment_valid = True
    message.selected_segment_id = 7
    message.metrics_valid = True

    message.temporal_status = "SUPPORTED"
    message.decision_status = "SUPPORTED"
    message.evidence_time_status = "FRESH"

    message.window_start_s_m = 0.
    message.window_end_s_m = 20.
    message.supported_interval_start_s_m = [0.]
    message.supported_interval_end_s_m = [10.]

    return message


class TestVisualizationMessages(unittest.TestCase):

    def test_lane_map_reflects_world_y(self):
        markers = build_lane_map_markers(
            make_lane_map()
        ).markers

        center = next(
            m for m in markers
            if m.ns == "lane_map_center"
        )

        self.assertEqual(
            center.header.frame_id,
            "carla_world_viz",
        )
        self.assertAlmostEqual(center.points[0].y, -3.)

    def test_empty_map_clears(self):
        message = make_lane_map()
        message.segments.clear()

        result = build_lane_map_markers(message)

        self.assertEqual(len(result.markers), 1)
        self.assertEqual(
            result.markers[0].action,
            Marker.DELETEALL,
        )

    def test_confirmed_interval_is_green(self):
        result = build_association_markers(
            make_route(),
            make_lane_map(),
            make_status(),
        )

        support = [
            marker for marker in result.markers
            if marker.ns == "observed_support"
        ]

        self.assertEqual(len(support), 1)
        self.assertGreater(
            support[0].color.g,
            support[0].color.r,
        )
        self.assertAlmostEqual(
            support[0].points[-1].x,
            10.,
        )

    def test_candidate_is_not_green(self):
        status = make_status()
        status.temporal_status = "CANDIDATE"

        result = build_association_markers(
            make_route(),
            make_lane_map(),
            status,
        )

        support = [
            marker for marker in result.markers
            if marker.ns == "observed_support"
        ]

        self.assertEqual(len(support), 1)
        self.assertGreater(
            support[0].color.r,
            support[0].color.g,
        )

    def test_wrong_map_revision_clears(self):
        status = make_status()
        status.lane_map_revision = 4

        result = build_association_markers(
            make_route(),
            make_lane_map(),
            status,
        )

        self.assertEqual(len(result.markers), 1)

    def test_stale_evidence_has_no_positive_support(self):
        status = make_status()
        status.evidence_time_status = "STALE_EVIDENCE"

        result = build_association_markers(
            make_route(),
            make_lane_map(),
            status,
        )

        self.assertFalse(
            any(
                marker.ns == "observed_support"
                for marker in result.markers
            )
        )

    def test_invalid_route_clears(self):
        route = make_route()
        route.status = RoutePlan.STATUS_INVALID

        result = build_association_markers(
            route,
            make_lane_map(),
            make_status(),
        )

        self.assertEqual(len(result.markers), 1)


    def test_lane_map_builder_preserves_source_header(self):
        message = make_lane_map()

        result = build_lane_map_markers(message)

        self.assertEqual(
            message.header.frame_id,
            "carla_world",
        )

        self.assertTrue(
            any(
                marker.ns == "lane_map_center"
                for marker in result.markers
            )
        )

    def test_association_builder_preserves_source_headers(self):
        route = make_route()
        lane_map = make_lane_map()
        status = make_status()

        build_association_markers(
            route,
            lane_map,
            status,
        )

        self.assertEqual(
            route.header.frame_id,
            "carla_world",
        )
        self.assertEqual(
            lane_map.header.frame_id,
            "carla_world",
        )
        self.assertEqual(
            status.header.frame_id,
            "carla_world",
        )


if __name__ == "__main__":
    unittest.main()
