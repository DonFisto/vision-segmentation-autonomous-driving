"""Tests for simulation-clock evidence freshness."""

import math
import unittest

from global_route_planner.lane_map_evidence_time import (
    EvidenceTimeParameters,
    classify_evidence_time,
)

S = 1_000_000_000


class TestLaneMapEvidenceTime(unittest.TestCase):

    def test_fresh(self):
        result = classify_evidence_time(99 * S, 100 * S)
        self.assertEqual(result.status, "FRESH")
        self.assertTrue(result.may_confirm_positive)
        self.assertAlmostEqual(result.evidence_age_s, 1.0)

    def test_exact_age_boundary(self):
        result = classify_evidence_time(98 * S, 100 * S)
        self.assertEqual(result.status, "FRESH")

    def test_stale_evidence(self):
        result = classify_evidence_time(97 * S, 100 * S)
        self.assertEqual(result.status, "STALE_EVIDENCE")
        self.assertFalse(result.may_confirm_positive)

    def test_future_evidence(self):
        result = classify_evidence_time(101 * S, 100 * S)
        self.assertEqual(result.status, "FUTURE_EVIDENCE")
        self.assertFalse(result.may_confirm_positive)

    def test_no_active_evidence(self):
        result = classify_evidence_time(0, 100 * S)
        self.assertEqual(result.status, "NO_ACTIVE_EVIDENCE")

    def test_reference_time_unavailable(self):
        result = classify_evidence_time(99 * S, 0)
        self.assertEqual(result.status, "REFERENCE_TIME_UNAVAILABLE")

    def test_clock_regression(self):
        result = classify_evidence_time(
            90 * S, 91 * S,
            previous_reference_stamp_ns=100 * S,
        )
        self.assertEqual(result.status, "CLOCK_REGRESSION")
        self.assertTrue(result.reset_temporal_history)

    def test_equal_reference_time(self):
        result = classify_evidence_time(
            99 * S, 100 * S,
            previous_reference_stamp_ns=100 * S,
        )
        self.assertEqual(result.status, "FRESH")

    def test_custom_age_limit(self):
        params = EvidenceTimeParameters(max_evidence_age_s=0.5)
        result = classify_evidence_time(
            99 * S, 100 * S,
            parameters=params,
        )
        self.assertEqual(result.status, "STALE_EVIDENCE")

    def test_invalid_timestamps(self):
        for value in (-1, True, 1.5, math.nan):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    classify_evidence_time(value, 100 * S)

    def test_invalid_previous_timestamp(self):
        with self.assertRaises(ValueError):
            classify_evidence_time(
                99 * S, 100 * S,
                previous_reference_stamp_ns=-1,
            )

    def test_invalid_parameters(self):
        for value in (0, -1, math.nan, math.inf):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    EvidenceTimeParameters(max_evidence_age_s=value)


if __name__ == "__main__":
    unittest.main()
