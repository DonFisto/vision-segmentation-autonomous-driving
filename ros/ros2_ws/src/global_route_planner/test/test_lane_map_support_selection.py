"""Tests for conservative LaneMap support arbitration."""

import math
import unittest

from global_route_planner.route_support_policy import (
    RouteSupportDecision,
)
from global_route_planner.lane_map_support_selection import (
    SegmentSupportCandidate,
    select_lane_map_support,
)


def candidate(segment_id, status, length=0.0):
    return SegmentSupportCandidate(
        segment_id=segment_id,
        decision=RouteSupportDecision(
            status=status,
            reason="TEST",
            near_ego_interval_m=length,
            initial_gap_tolerated=False,
        ),
    )


class TestLaneMapSupportSelection(unittest.TestCase):

    def select(self, *items):
        return select_lane_map_support(items)

    def test_empty_lane_map(self):
        result = self.select()
        self.assertEqual(result.decision.status, "UNAVAILABLE")
        self.assertEqual(result.decision.reason, "NO_USABLE_SEGMENTS")
        self.assertIsNone(result.selected_segment_id)

    def test_single_supported(self):
        result = self.select(candidate(7, "SUPPORTED", 20))
        self.assertEqual(result.decision.status, "SUPPORTED")
        self.assertEqual(result.selected_segment_id, 7)

    def test_single_partial(self):
        result = self.select(candidate(7, "PARTIAL", 6))
        self.assertEqual(result.decision.status, "PARTIAL")

    def test_supported_over_partial(self):
        result = self.select(
            candidate(1, "PARTIAL", 25),
            candidate(2, "SUPPORTED", 15),
        )
        self.assertEqual(result.selected_segment_id, 2)
        self.assertEqual(result.decision.status, "SUPPORTED")

    def test_ambiguous_supported(self):
        result = self.select(
            candidate(1, "SUPPORTED", 20),
            candidate(2, "SUPPORTED", 20),
        )
        self.assertEqual(result.decision.status, "UNAVAILABLE")
        self.assertEqual(
            result.decision.reason,
            "AMBIGUOUS_SUPPORTED_SEGMENTS",
        )
        self.assertEqual(result.supported_candidate_count, 2)
        self.assertIsNone(result.selected_segment_id)

    def test_best_partial_interval(self):
        result = self.select(
            candidate(1, "PARTIAL", 3),
            candidate(2, "PARTIAL", 10),
        )
        self.assertEqual(result.selected_segment_id, 2)

    def test_partial_tie_uses_segment_id(self):
        result = self.select(
            candidate(9, "PARTIAL", 10),
            candidate(3, "PARTIAL", 10),
        )
        self.assertEqual(result.selected_segment_id, 3)

    def test_partial_over_rejected(self):
        result = self.select(
            candidate(1, "REJECTED"),
            candidate(2, "PARTIAL", 3),
        )
        self.assertEqual(result.decision.status, "PARTIAL")

    def test_unavailable_over_rejected(self):
        result = self.select(
            candidate(1, "REJECTED"),
            candidate(2, "UNAVAILABLE"),
        )
        self.assertEqual(result.decision.status, "UNAVAILABLE")

    def test_all_rejected(self):
        result = self.select(
            candidate(8, "REJECTED"),
            candidate(4, "REJECTED"),
        )
        self.assertEqual(result.decision.status, "REJECTED")
        self.assertEqual(result.selected_segment_id, 4)

    def test_all_unavailable(self):
        result = self.select(
            candidate(8, "UNAVAILABLE"),
            candidate(4, "UNAVAILABLE"),
        )
        self.assertEqual(result.decision.status, "UNAVAILABLE")
        self.assertEqual(result.selected_segment_id, 4)

    def test_duplicate_ids(self):
        with self.assertRaises(ValueError):
            self.select(
                candidate(1, "PARTIAL"),
                candidate(1, "SUPPORTED", 20),
            )

    def test_negative_segment_id(self):
        with self.assertRaises(ValueError):
            candidate(-1, "SUPPORTED", 20)

    def test_boolean_segment_id(self):
        with self.assertRaises(ValueError):
            candidate(True, "SUPPORTED", 20)

    def test_invalid_candidate_type(self):
        with self.assertRaises(TypeError):
            select_lane_map_support(["invalid"])

    def test_unknown_status(self):
        with self.assertRaises(ValueError):
            candidate(1, "UNKNOWN")

    def test_nonfinite_support_length(self):
        with self.assertRaises(ValueError):
            candidate(1, "SUPPORTED", math.nan)


if __name__ == "__main__":
    unittest.main()
