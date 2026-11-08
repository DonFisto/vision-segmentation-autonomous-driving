"""Tests for rolling LaneMap evidence provenance."""

import unittest

from autonomy_interfaces.msg import LaneMap
from rclpy.serialization import (
    serialize_message,
    deserialize_message,
)

from lane_reasoning_nodes.tracked_lane_mapping_node import (
    populate_lane_map_provenance,
)


def make_map(session, map_rev, evidence_rev, stamp_ns):
    message = LaneMap()

    populate_lane_map_provenance(
        message,
        session_id=session,
        map_revision=map_rev,
        evidence_revision=evidence_rev,
        active_evidence_stamp_ns=stamp_ns,
    )

    return message


class TestLaneMapProvenance(unittest.TestCase):

    def test_integrated_evidence(self):
        msg = make_map(42, 10, 7, 12_345_678_901)

        self.assertEqual(msg.schema_version, 3)
        self.assertEqual(msg.map_session_id, 42)
        self.assertEqual(msg.map_revision, 10)
        self.assertEqual(msg.evidence_revision, 7)
        self.assertEqual(msg.latest_evidence_stamp.sec, 12)
        self.assertEqual(
            msg.latest_evidence_stamp.nanosec, 345_678_901
        )

    def test_pruning_does_not_imply_new_evidence(self):
        before = make_map(42, 10, 7, 12_345_678_901)
        after = make_map(42, 11, 7, 12_345_678_901)

        self.assertGreater(
            after.map_revision, before.map_revision
        )
        self.assertEqual(
            after.evidence_revision, before.evidence_revision
        )

    def test_mapper_restart_changes_session(self):
        old = make_map(42, 100, 90, 12_345_678_901)
        restarted = make_map(43, 1, 1, 13_000_000_000)

        self.assertNotEqual(
            old.map_session_id, restarted.map_session_id
        )

    def test_no_active_evidence(self):
        msg = make_map(42, 12, 7, None)

        self.assertEqual(msg.evidence_revision, 7)
        self.assertEqual(msg.latest_evidence_stamp.sec, 0)
        self.assertEqual(msg.latest_evidence_stamp.nanosec, 0)

    def test_serialization(self):
        original = make_map(42, 10, 7, 12_345_678_901)

        restored = deserialize_message(
            serialize_message(original),
            LaneMap,
        )

        self.assertEqual(restored.map_session_id, 42)
        self.assertEqual(restored.evidence_revision, 7)
        self.assertEqual(restored.latest_evidence_stamp.sec, 12)


if __name__ == "__main__":
    unittest.main()
