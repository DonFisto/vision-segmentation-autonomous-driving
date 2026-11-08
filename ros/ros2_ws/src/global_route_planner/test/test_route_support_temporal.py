"""ROS-independent temporal route-support tracking tests."""

import math
import unittest

from global_route_planner.route_identity import RouteIdentity
from global_route_planner.route_support_policy import RouteSupportDecision
from global_route_planner.route_support_temporal import (
    TemporalRouteSupportTracker,
    TemporalSupportParameters,
)


class TestTemporalRouteSupportTracker(unittest.TestCase):
    def setUp(self):
        self.tracker = TemporalRouteSupportTracker()
        self.old = [(0.0, 0.0), (60.0, 0.0)]
        self.new = [(10.0, 0.0), (60.0, 0.0)]
        self.first = RouteIdentity(11, 1, 7)
        self.second = RouteIdentity(11, 2, 7)
        self.tracker.register_route(self.first, self.old)

    def decision(self, status):
        return RouteSupportDecision(status, "TEST", 12.0, False)

    def report(self, status, at, identity=None):
        return self.tracker.observe(
            identity or self.tracker.route_identity,
            self.decision(status),
            at,
            lane_map_revision=(
                max(0, int(at * 10)) if math.isfinite(at) else 10
            ),
        )

    def test_waiting_after_initial_route(self):
        snapshot = self.tracker.snapshot(0.0)
        self.assertEqual(snapshot.status, "WAITING")
        self.assertEqual(snapshot.observations_in_revision, 0)

    def test_no_route(self):
        tracker = TemporalRouteSupportTracker()
        self.assertEqual(tracker.snapshot(0).status, "NO_ROUTE")

    def test_three_fresh_supports_confirm(self):
        self.assertEqual(self.report("SUPPORTED", 1).status, "CANDIDATE")
        self.assertEqual(self.report("SUPPORTED", 2).status, "CANDIDATE")
        snap = self.report("SUPPORTED", 3)
        self.assertEqual(snap.status, "SUPPORTED")
        self.assertEqual(snap.consecutive_supported, 3)
        self.assertEqual(snap.observations_in_revision, 3)

    def test_gap_breaks_support_streak(self):
        self.report("SUPPORTED", 1)
        self.report("SUPPORTED", 2)
        snap = self.report("SUPPORTED", 4)
        self.assertEqual(snap.status, "CANDIDATE")
        self.assertEqual(snap.consecutive_supported, 1)

    def test_partial_immediately_cancels_support(self):
        for t in (1, 2, 3):
            self.report("SUPPORTED", t)
        snap = self.report("PARTIAL", 4)
        self.assertEqual(snap.status, "PARTIAL")
        self.assertEqual(snap.consecutive_supported, 0)
        self.assertEqual(self.report("SUPPORTED", 5).status, "CANDIDATE")

    def test_unavailable_immediately_cancels_support(self):
        for t in (1, 2, 3):
            self.report("SUPPORTED", t)
        self.assertEqual(
            self.report("UNAVAILABLE", 4).status,
            "UNAVAILABLE",
        )

    def test_rejected_immediately_cancels_support(self):
        for t in (1, 2, 3):
            self.report("SUPPORTED", t)
        self.assertEqual(
            self.report("REJECTED", 4).status,
            "REJECTED",
        )

    def test_stale_supported_does_not_remain_supported(self):
        for t in (1, 2, 3):
            self.report("SUPPORTED", t)
        snap = self.tracker.snapshot(5.01)
        self.assertEqual(snap.status, "STALE")
        self.assertGreater(snap.latest_observation_age_s, 2)

    def test_freshness_weight_is_not_confidence_probability(self):
        self.report("SUPPORTED", 1)
        self.assertAlmostEqual(
            self.tracker.snapshot(1).freshness_weight, 1.0
        )
        self.assertAlmostEqual(
            self.tracker.snapshot(2).freshness_weight,
            math.exp(-1),
        )

    def test_same_route_preserves_history(self):
        self.report("SUPPORTED", 1)
        self.report("SUPPORTED", 2)
        consistency = self.tracker.register_route(
            self.first, list(self.old)
        )
        self.assertEqual(consistency.history_action, "KEEP")
        self.assertEqual(self.report("SUPPORTED", 3).status, "SUPPORTED")

    def test_aligned_revision_resets_history(self):
        for t in (1, 2, 3):
            self.report("SUPPORTED", t)
        consistency = self.tracker.register_route(self.second, self.new)
        self.assertEqual(consistency.status, "ROUTE_REVISED_ALIGNED")
        self.assertEqual(self.tracker.snapshot(4).status, "WAITING")
        self.assertEqual(self.report("SUPPORTED", 5).status, "CANDIDATE")

    def test_diverged_revision_resets_history(self):
        self.report("SUPPORTED", 1)
        consistency = self.tracker.register_route(
            self.second, [(10, 0), (10, 40)]
        )
        self.assertEqual(consistency.status, "ROUTE_REVISED_DIVERGED")
        self.assertEqual(self.tracker.snapshot(2).status, "WAITING")

    def test_new_mission_resets(self):
        self.report("SUPPORTED", 1)
        other = RouteIdentity(12, 3, 7)
        result = self.tracker.register_route(other, self.new)
        self.assertEqual(result.status, "MISSION_CHANGED")
        self.assertEqual(self.tracker.snapshot(2).status, "WAITING")

    def test_map_changed_resets(self):
        self.report("SUPPORTED", 1)
        other = RouteIdentity(11, 3, 8)
        result = self.tracker.register_route(other, self.new)
        self.assertEqual(result.status, "MAP_CHANGED")
        self.assertEqual(self.tracker.snapshot(2).status, "WAITING")

    def test_out_of_order_is_ignored(self):
        self.tracker.register_route(self.second, self.new)
        self.report("SUPPORTED", 1)
        result = self.tracker.register_route(self.first, self.old)
        self.assertEqual(result.status, "OUT_OF_ORDER_REVISION")
        self.assertEqual(self.tracker.route_identity, self.second)
        self.assertEqual(
            self.tracker.snapshot(1).consecutive_supported, 1
        )

    def test_same_identity_geometry_conflict_is_ignored(self):
        self.report("SUPPORTED", 1)
        result = self.tracker.register_route(self.first, self.new)
        self.assertEqual(result.status, "ROUTE_ID_GEOMETRY_CONFLICT")
        self.assertEqual(
            self.tracker.snapshot(1).consecutive_supported, 1
        )

    def test_wrong_revision_observation_rejected(self):
        with self.assertRaises(ValueError):
            self.report("SUPPORTED", 1, self.second)
        self.assertEqual(self.tracker.snapshot(2).status, "WAITING")

    def test_duplicate_and_reversed_times_rejected(self):
        self.report("SUPPORTED", 2)
        for t, revision in ((2, 21), (1, 22)):
            with self.subTest(t=t):
                with self.assertRaises(ValueError):
                    self.tracker.observe(
                        self.first, self.decision("SUPPORTED"), t,
                        lane_map_revision=revision,
                    )
        self.assertEqual(
            self.tracker.snapshot(2).observations_in_revision, 1
        )

    def test_invalid_decision_rejected_without_state_change(self):
        with self.assertRaises(TypeError):
            self.tracker.observe(self.first, "SUPPORTED", 1, 10)
        with self.assertRaises(ValueError):
            self.report("UNKNOWN", 1)
        self.assertEqual(self.tracker.snapshot(2).status, "WAITING")

    def test_invalid_time_rejected(self):
        for timestamp in (-1, math.nan, math.inf):
            with self.subTest(timestamp=timestamp):
                with self.assertRaises(ValueError):
                    self.report("SUPPORTED", timestamp)
        with self.assertRaises(ValueError):
            self.tracker.snapshot(math.inf)

    def test_snapshot_before_last_observation_rejected(self):
        self.report("PARTIAL", 4)
        with self.assertRaises(ValueError):
            self.tracker.snapshot(3)

    def test_repeated_lane_map_does_not_count_twice(self):
        self.report("SUPPORTED", 1)
        with self.assertRaises(ValueError):
            self.tracker.observe(
                self.first, self.decision("SUPPORTED"), 1.1,
                lane_map_revision=10,
            )
        self.assertEqual(
            self.tracker.snapshot(1.1).consecutive_supported, 1
        )

    def test_out_of_order_lane_map_is_rejected(self):
        self.report("SUPPORTED", 2)
        with self.assertRaises(ValueError):
            self.tracker.observe(
                self.first, self.decision("SUPPORTED"), 3,
                lane_map_revision=19,
            )
        self.assertEqual(
            self.tracker.snapshot(3).observations_in_revision, 1
        )

    def test_expired_support_streak_does_not_revive(self):
        tracker = TemporalRouteSupportTracker(
            TemporalSupportParameters(max_observation_age_s=0.5)
        )
        tracker.register_route(self.first, self.old)
        tracker.observe(
            self.first, self.decision("SUPPORTED"), 1.0, 10
        )
        snap = tracker.observe(
            self.first, self.decision("SUPPORTED"), 2.0, 11
        )
        self.assertEqual(snap.consecutive_supported, 1)

    def test_invalid_lane_map_revision(self):
        for revision in (-1, True, 1.5):
            with self.subTest(revision=revision):
                with self.assertRaises(ValueError):
                    self.tracker.observe(
                        self.first, self.decision("SUPPORTED"), 1,
                        lane_map_revision=revision,
                    )

    def test_invalid_parameters(self):
        for kwargs in (
            {"min_consecutive_supported": 0},
            {"min_consecutive_supported": True},
            {"max_inter_observation_gap_s": 0},
            {"max_observation_age_s": math.nan},
            {"freshness_decay_tau_s": -1},
        ):
            with self.subTest(kwargs=kwargs):
                with self.assertRaises(ValueError):
                    TemporalSupportParameters(**kwargs)
        with self.assertRaises(TypeError):
            TemporalRouteSupportTracker(parameters="invalid")

    def test_custom_confirmation_threshold(self):
        tracker = TemporalRouteSupportTracker(
            TemporalSupportParameters(min_consecutive_supported=1)
        )
        tracker.register_route(self.first, self.old)
        snap = tracker.observe(
            self.first, self.decision("SUPPORTED"), 1, 10
        )
        self.assertEqual(snap.status, "SUPPORTED")


if __name__ == "__main__":
    unittest.main()
