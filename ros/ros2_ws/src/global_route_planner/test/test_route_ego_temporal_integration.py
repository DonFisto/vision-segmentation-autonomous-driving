"""Synthetic ROS2-message tests for route-support diagnostic callbacks."""

import time
import unittest

import rclpy
from autonomy_interfaces.msg import (
    LaneMap,
    LaneSample,
    LaneSegment,
    RoutePlan,
)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from global_route_planner.route_ego_diagnostic_node import (
    RouteEgoDiagnosticNode,
)


class TestRouteEgoTemporalIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._owns_context = not rclpy.ok()
        if cls._owns_context:
            rclpy.init(args=[])

    @classmethod
    def tearDownClass(cls):
        if cls._owns_context and rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.node = RouteEgoDiagnosticNode()
        self.node._route_callback(self.route())
        self.node._odom_callback(self.odometry())

    def tearDown(self):
        self.node.destroy_node()

    def route(self, revision=1, start=0.0, valid=True):
        msg = RoutePlan()
        msg.header.frame_id = "carla_world"
        msg.mission_id = 123
        msg.route_id = revision
        msg.map_revision = 7
        msg.status = (
            RoutePlan.STATUS_VALID
            if valid else RoutePlan.STATUS_INVALID
        )

        msg.coarse_reference_path.header.frame_id = "carla_world"

        if valid:
            for x in (start, 60.0):
                pose = PoseStamped()
                pose.header.frame_id = "carla_world"
                pose.pose.position.x = float(x)
                pose.pose.orientation.w = 1.0
                msg.coarse_reference_path.poses.append(pose)

        return msg

    def odometry(self):
        msg = Odometry()
        msg.header.frame_id = "carla_world"
        msg.header.stamp.sec = 1000
        msg.pose.pose.orientation.w = 1.0
        return msg

    def lane_map(
        self, revision, segment_ids=(7,),
        session=42, evidence=None,
    ):
        if evidence is None:
            evidence = revision

        msg = LaneMap()
        msg.header.frame_id = "carla_world"
        msg.header.stamp.sec = 1000
        msg.schema_version = 3
        msg.map_revision = revision
        msg.map_session_id = session
        msg.evidence_revision = evidence

        if evidence > 0:
            msg.latest_evidence_stamp.sec = 999
            msg.latest_evidence_stamp.nanosec = (
                evidence * 1_000_000
            )

        for segment_id in segment_ids:
            segment = LaneSegment()
            segment.id = segment_id
            segment.drivable = True
            segment.confidence = 1.0

            for x in range(36):
                sample = LaneSample()
                sample.s = float(x)
                sample.pose.position.x = float(x)
                sample.pose.orientation.w = 1.0
                sample.confidence = 0.95
                segment.centerline.append(sample)

            msg.segments.append(segment)

        return msg

    def process(self, revision, segment_ids=(7,), **kwargs):
        self.node._lane_map_callback(
            self.lane_map(revision, segment_ids, **kwargs)
        )
        self.node._report_projection()
        return self.node._temporal_support.snapshot(
            time.monotonic()
        )

    def test_no_lane_map_remains_waiting(self):
        self.node._report_projection()
        self.assertEqual(
            self.node._temporal_support.snapshot(
                time.monotonic()
            ).status,
            "WAITING",
        )

    def test_three_revisions_confirm_support(self):
        self.assertEqual(self.process(1).status, "CANDIDATE")
        self.assertEqual(self.process(2).status, "CANDIDATE")
        result = self.process(3)
        self.assertEqual(result.status, "SUPPORTED")
        self.assertEqual(result.observations_in_revision, 3)

    def test_duplicate_revision_does_not_count(self):
        self.process(1)
        result = self.process(1)
        self.assertEqual(result.status, "CANDIDATE")
        self.assertEqual(result.observations_in_revision, 1)

    def test_ambiguous_segments_are_unavailable(self):
        result = self.process(1, (7, 8))
        self.assertEqual(result.status, "UNAVAILABLE")
        self.assertEqual(result.consecutive_supported, 0)

    def test_empty_lane_map_cancels_support(self):
        for revision in (1, 2, 3):
            self.process(revision)

        result = self.process(4, ())
        self.assertEqual(result.status, "UNAVAILABLE")
        self.assertEqual(result.consecutive_supported, 0)

    def test_stale_lane_map_not_reprocessed(self):
        self.process(1)
        self.node._lane_map_callback(self.lane_map(2))
        self.node._latest_lane_map_time -= (
            self.node._max_lane_map_age_s + 1.0
        )
        self.node._report_projection()

        tracker = self.node._temporal_support
        self.assertEqual(
            tracker.snapshot(time.monotonic()).observations_in_revision,
            1,
        )
        self.assertEqual(
            tracker.snapshot(time.monotonic() + 3.0).status,
            "STALE",
        )

    def test_new_route_revision_resets_history(self):
        self.process(1)
        self.process(2)
        self.node._route_callback(self.route(2, start=10.0))

        result = self.node._temporal_support.snapshot(
            time.monotonic()
        )
        self.assertEqual(result.status, "WAITING")
        self.assertEqual(result.route_identity.route_id, 2)

    def test_older_valid_route_is_ignored(self):
        self.node._route_callback(self.route(2, start=10.0))
        self.node._route_callback(self.route(1))

        self.assertEqual(self.node._active_route.route_id, 2)
        self.assertEqual(
            self.node._temporal_support.route_identity.route_id,
            2,
        )

    def test_invalid_route_resets_active_geometry(self):
        self.node._route_callback(self.route(2, valid=False))

        self.assertIsNone(self.node._active_route)
        self.assertIsNone(self.node._route_points)
        self.assertIsNone(
            self.node._temporal_support.route_identity
        )

    def test_old_valid_route_not_replayed_after_invalid(self):
        self.node._route_callback(self.route(2, valid=False))
        self.node._route_callback(self.route(1))

        self.assertIsNone(self.node._active_route)
        self.assertEqual(
            self.node._last_route_identity.route_id,
            2,
        )


    def test_map_only_positive_does_not_increment(self):
        self.process(1)
        snap = self.process(2, evidence=1)

        self.assertEqual(snap.status, "CANDIDATE")
        self.assertEqual(snap.consecutive_supported, 1)
        self.assertEqual(snap.observations_in_revision, 1)

        snap = self.process(3, evidence=2)
        self.assertEqual(snap.consecutive_supported, 2)

    def test_pruning_only_negative_cancels_support(self):
        for revision in (1, 2, 3):
            self.process(revision)

        snap = self.process(4, (), evidence=3)

        self.assertEqual(snap.status, "UNAVAILABLE")
        self.assertEqual(snap.consecutive_supported, 0)

    def test_mapper_session_restart_resets(self):
        for revision in (10, 11, 12):
            self.process(revision)

        self.assertEqual(
            self.node._temporal_support.snapshot(
                time.monotonic()
            ).status,
            "SUPPORTED",
        )

        snap = self.process(1, session=43)

        self.assertEqual(snap.status, "CANDIDATE")
        self.assertEqual(snap.consecutive_supported, 1)
        self.assertEqual(snap.observations_in_revision, 1)

    def test_stale_evidence_is_not_positive(self):
        msg = self.lane_map(1)
        msg.latest_evidence_stamp.sec = 995

        self.node._lane_map_callback(msg)
        self.node._report_projection()

        snap = self.node._temporal_support.snapshot(
            time.monotonic()
        )
        self.assertEqual(snap.status, "UNAVAILABLE")
        self.assertEqual(snap.consecutive_supported, 0)

    def test_stale_new_evidence_cancels_existing_support(self):
        for revision in (1, 2, 3):
            self.process(revision)

        odom = self.odometry()
        odom.header.stamp.sec = 1005
        self.node._odom_callback(odom)

        msg = self.lane_map(4)
        msg.header.stamp.sec = 1005

        self.node._lane_map_callback(msg)
        self.node._report_projection()

        snap = self.node._temporal_support.snapshot(
            time.monotonic()
        )
        self.assertEqual(snap.status, "UNAVAILABLE")

    def test_legacy_schema_cannot_confirm(self):
        msg = self.lane_map(1)
        msg.schema_version = 2

        self.node._lane_map_callback(msg)
        self.node._report_projection()

        snap = self.node._temporal_support.snapshot(
            time.monotonic()
        )
        self.assertEqual(snap.status, "WAITING")

    def test_clock_regression_resets_evidence(self):
        self.process(1)

        odom = self.odometry()
        odom.header.stamp.sec = 900
        self.node._odom_callback(odom)

        msg = self.lane_map(2)
        msg.header.stamp.sec = 900

        self.node._lane_map_callback(msg)
        self.node._report_projection()

        snap = self.node._temporal_support.snapshot(
            time.monotonic()
        )
        self.assertEqual(snap.status, "WAITING")
        self.assertEqual(snap.observations_in_revision, 0)

    def test_old_evidence_revision_does_not_count(self):
        self.process(1)
        snap = self.process(2, evidence=0)

        self.assertEqual(snap.status, "CANDIDATE")
        self.assertEqual(snap.consecutive_supported, 1)
        self.assertEqual(snap.observations_in_revision, 1)


    def test_previous_mapper_session_cannot_replay(self):
        self.process(1, session=42)
        self.process(1, session=43)

        result = self.process(2, session=42)

        self.assertEqual(self.node._active_map_session_id, 43)
        self.assertEqual(result.status, "CANDIDATE")
        self.assertEqual(result.observations_in_revision, 1)

    def test_clock_regression_recovers_with_new_evidence(self):
        self.process(1)

        odom = self.odometry()
        odom.header.stamp.sec = 900
        self.node._odom_callback(odom)

        # Trigger the clock-regression reset.
        msg = self.lane_map(2)
        msg.header.stamp.sec = 900
        msg.latest_evidence_stamp.sec = 899

        self.node._lane_map_callback(msg)
        self.node._report_projection()

        snapshot = self.node._temporal_support.snapshot(
            time.monotonic()
        )
        self.assertEqual(snapshot.status, "WAITING")

        # The same previously integrated evidence must not revive
        # positive support after the clock reset.
        msg = self.lane_map(2, evidence=1)
        msg.header.stamp.sec = 900
        msg.latest_evidence_stamp.sec = 899

        self.node._lane_map_callback(msg)
        self.node._report_projection()

        snapshot = self.node._temporal_support.snapshot(
            time.monotonic()
        )
        self.assertEqual(snapshot.status, "WAITING")

        # A new integration, with a timestamp valid in the reset
        # clock domain, can begin a fresh confirmation streak.
        msg = self.lane_map(3, evidence=2)
        msg.header.stamp.sec = 900
        msg.latest_evidence_stamp.sec = 899

        self.node._lane_map_callback(msg)
        self.node._report_projection()

        snapshot = self.node._temporal_support.snapshot(
            time.monotonic()
        )

        self.assertEqual(snapshot.status, "CANDIDATE")
        self.assertEqual(snapshot.consecutive_supported, 1)
        self.assertEqual(snapshot.observations_in_revision, 1)


    def test_published_association_status_matches_decision(self):
        self.process(1)
        status = self.node._last_association_status

        self.assertIsNotNone(status)
        self.assertEqual(status.route_id, 1)
        self.assertEqual(status.map_session_id, 42)
        self.assertEqual(status.evidence_revision, 1)
        self.assertEqual(status.selected_segment_id, 7)
        self.assertTrue(status.selected_segment_valid)
        self.assertEqual(status.temporal_status, "CANDIDATE")
        self.assertTrue(status.metrics_valid)
        self.assertGreater(status.coverage_ratio_ahead, 0.0)
        self.assertEqual(
            len(status.supported_interval_start_s_m),
            len(status.supported_interval_end_s_m),
        )

    def test_published_status_tracks_temporal_confirmation(self):
        for revision in (1, 2, 3):
            self.process(revision)

        status = self.node._last_association_status

        self.assertEqual(status.temporal_status, "SUPPORTED")
        self.assertEqual(status.consecutive_supported, 3)

    def test_pruning_only_status_cannot_advance_confirmation(self):
        self.process(1)
        self.process(2, evidence=1)

        status = self.node._last_association_status

        self.assertEqual(
            status.provenance_status,
            "MAP_ONLY_CHANGE",
        )
        self.assertEqual(status.evidence_revision, 1)
        self.assertEqual(status.consecutive_supported, 1)

    def test_empty_map_status_clears_selected_segment(self):
        self.process(1)
        self.process(2, (), evidence=1)

        status = self.node._last_association_status

        self.assertEqual(status.decision_status, "UNAVAILABLE")
        self.assertFalse(status.selected_segment_valid)
        self.assertFalse(status.metrics_valid)


if __name__ == "__main__":
    unittest.main()
