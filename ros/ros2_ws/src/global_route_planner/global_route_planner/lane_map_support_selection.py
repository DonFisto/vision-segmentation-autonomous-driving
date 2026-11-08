"""Select one diagnostic route-support decision per LaneMap.

This module does not assign the ego to a lane or authorize motion.
Multiple supported segments remain ambiguous until a separate
lane-selection or tracking layer resolves them.
"""

from dataclasses import dataclass
import math

from global_route_planner.route_support_policy import (
    RouteSupportDecision,
)

VALID_STATUSES = {
    "SUPPORTED",
    "PARTIAL",
    "UNAVAILABLE",
    "REJECTED",
}


@dataclass(frozen=True)
class SegmentSupportCandidate:
    segment_id: int
    decision: RouteSupportDecision

    def __post_init__(self):
        if (
            type(self.segment_id) is not int
            or not 0 <= self.segment_id < 2**64
        ):
            raise ValueError("segment_id must be uint64")

        if not isinstance(self.decision, RouteSupportDecision):
            raise TypeError("Expected RouteSupportDecision")

        if self.decision.status not in VALID_STATUSES:
            raise ValueError("Unknown support status")

        distance = self.decision.near_ego_interval_m

        if not math.isfinite(distance) or distance < 0.0:
            raise ValueError("Invalid near-ego support length")


@dataclass(frozen=True)
class LaneMapSupportSelection:
    decision: RouteSupportDecision
    selected_segment_id: int | None
    candidate_count: int
    supported_candidate_count: int
    selection_reason: str


def select_lane_map_support(
    candidates,
) -> LaneMapSupportSelection:
    """Consolidate candidate decisions deterministically.

    Priority:
      - Exactly one SUPPORTED candidate: select it.
      - Multiple SUPPORTED candidates: unavailable (ambiguous).
      - Otherwise select the longest PARTIAL near-ego interval.
      - Otherwise prefer UNAVAILABLE over REJECTED.
      - Report REJECTED only when all candidates are rejected.

    This returns one decision, but does not deduplicate LaneMap
    revisions. The temporal tracker enforces revision freshness.
    """

    items = tuple(candidates)

    if not all(
        isinstance(item, SegmentSupportCandidate)
        for item in items
    ):
        raise TypeError("All candidates must be SegmentSupportCandidate")

    ids = [item.segment_id for item in items]

    if len(ids) != len(set(ids)):
        raise ValueError("Duplicate LaneMap segment IDs")

    def make(
        decision,
        segment_id,
        reason,
        supported_count,
    ):
        return LaneMapSupportSelection(
            decision=decision,
            selected_segment_id=segment_id,
            candidate_count=len(items),
            supported_candidate_count=supported_count,
            selection_reason=reason,
        )

    if not items:
        return make(
            RouteSupportDecision(
                "UNAVAILABLE",
                "NO_USABLE_SEGMENTS",
                0.0,
                False,
            ),
            None,
            "NO_CANDIDATES",
            0,
        )

    supported = [
        item for item in items
        if item.decision.status == "SUPPORTED"
    ]

    if len(supported) > 1:
        return make(
            RouteSupportDecision(
                "UNAVAILABLE",
                "AMBIGUOUS_SUPPORTED_SEGMENTS",
                0.0,
                False,
            ),
            None,
            "MULTIPLE_SUPPORTED_CANDIDATES",
            len(supported),
        )

    if len(supported) == 1:
        selected = supported[0]

        return make(
            selected.decision,
            selected.segment_id,
            "UNIQUE_SUPPORTED_CANDIDATE",
            1,
        )

    partial = [
        item for item in items
        if item.decision.status == "PARTIAL"
    ]

    if partial:
        selected = min(
            partial,
            key=lambda item: (
                -item.decision.near_ego_interval_m,
                item.segment_id,
            ),
        )

        return make(
            selected.decision,
            selected.segment_id,
            "BEST_PARTIAL_CANDIDATE",
            0,
        )

    unavailable = [
        item for item in items
        if item.decision.status == "UNAVAILABLE"
    ]

    if unavailable:
        selected = min(
            unavailable,
            key=lambda item: item.segment_id,
        )

        return make(
            selected.decision,
            selected.segment_id,
            "UNAVAILABLE_EVIDENCE",
            0,
        )

    # Every remaining candidate is REJECTED.
    selected = min(
        items,
        key=lambda item: item.segment_id,
    )

    return make(
        selected.decision,
        selected.segment_id,
        "ALL_CANDIDATES_REJECTED",
        0,
    )
