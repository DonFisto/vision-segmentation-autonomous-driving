"""Tests for LaneMap evidence freshness classification."""

import unittest

from global_route_planner.lane_map_provenance_gate import (
    LaneMapProvenance,
    classify_lane_map_provenance,
)


def provenance(session=42, map_rev=1, evidence_rev=1, stamp=100):
    return LaneMapProvenance(
        session_id=session,
        map_revision=map_rev,
        evidence_revision=evidence_rev,
        latest_evidence_stamp_ns=stamp,
    )


class TestLaneMapProvenanceGate(unittest.TestCase):

    def classify(self, previous, current):
        return classify_lane_map_provenance(previous, current)

    def test_initial_map_with_evidence(self):
        result = self.classify(None, provenance())
        self.assertEqual(result.status, "INITIAL_MAP")
        self.assertTrue(result.reset_history)
        self.assertTrue(result.may_confirm_positive)

    def test_initial_map_without_active_evidence(self):
        result = self.classify(
            None, provenance(evidence_rev=0, stamp=0)
        )
        self.assertFalse(result.may_confirm_positive)
        self.assertTrue(result.may_apply_negative)

    def test_new_evidence(self):
        result = self.classify(
            provenance(),
            provenance(map_rev=2, evidence_rev=2, stamp=200),
        )
        self.assertEqual(result.status, "NEW_EVIDENCE")
        self.assertTrue(result.may_confirm_positive)
        self.assertFalse(result.reset_history)

    def test_pruning_only(self):
        result = self.classify(
            provenance(),
            provenance(map_rev=2),
        )
        self.assertEqual(result.status, "MAP_ONLY_CHANGE")
        self.assertFalse(result.may_confirm_positive)
        self.assertTrue(result.may_apply_negative)

    def test_map_only_change_with_same_evidence(self):
        result = self.classify(
            provenance(map_rev=3, evidence_rev=2, stamp=200),
            provenance(map_rev=4, evidence_rev=2, stamp=200),
        )
        self.assertEqual(result.status, "MAP_ONLY_CHANGE")

    def test_pruning_removes_all_active_evidence(self):
        result = self.classify(
            provenance(),
            provenance(map_rev=2, evidence_rev=1, stamp=0),
        )
        self.assertTrue(result.accept_map)
        self.assertFalse(result.may_confirm_positive)
        self.assertTrue(result.may_apply_negative)

    def test_duplicate_is_not_recounted(self):
        current = provenance()
        result = self.classify(current, current)
        self.assertEqual(result.status, "DUPLICATE")
        self.assertFalse(result.accept_map)

    def test_old_map_is_rejected(self):
        result = self.classify(
            provenance(map_rev=5, evidence_rev=5, stamp=500),
            provenance(map_rev=4, evidence_rev=4, stamp=400),
        )
        self.assertEqual(result.status, "OUT_OF_ORDER")

    def test_evidence_revision_regression(self):
        result = self.classify(
            provenance(map_rev=5, evidence_rev=5, stamp=500),
            provenance(map_rev=6, evidence_rev=4, stamp=400),
        )
        self.assertEqual(result.status, "OUT_OF_ORDER")

    def test_same_map_revision_conflict(self):
        result = self.classify(
            provenance(),
            provenance(map_rev=1, evidence_rev=2, stamp=200),
        )
        self.assertEqual(result.status, "REVISION_CONFLICT")

    def test_session_change_requires_reset(self):
        result = self.classify(
            provenance(session=42, map_rev=100),
            provenance(session=43, map_rev=1),
        )
        self.assertEqual(result.status, "SESSION_CHANGED")
        self.assertTrue(result.reset_history)
        self.assertTrue(result.accept_map)

    def test_new_evidence_timestamp_must_advance(self):
        result = self.classify(
            provenance(),
            provenance(map_rev=2, evidence_rev=2, stamp=100),
        )
        self.assertEqual(
            result.status,
            "NON_MONOTONIC_EVIDENCE_TIMESTAMP",
        )
        self.assertFalse(result.accept_map)

    def test_no_evidence_cannot_have_timestamp(self):
        with self.assertRaises(ValueError):
            provenance(evidence_rev=0, stamp=100)

    def test_invalid_provenance_values(self):
        for kwargs in (
            {"session": 0},
            {"map_rev": -1},
            {"evidence_rev": True},
            {"stamp": -1},
        ):
            with self.subTest(kwargs=kwargs):
                with self.assertRaises(ValueError):
                    provenance(**kwargs)


if __name__ == "__main__":
    unittest.main()
