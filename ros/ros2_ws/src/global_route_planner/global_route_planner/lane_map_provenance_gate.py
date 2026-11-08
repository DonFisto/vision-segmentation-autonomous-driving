"""Classify LaneMap evidence provenance before temporal confirmation.

Pure Python. This module does not process ROS messages, maintain a
temporal filter, or decide whether a vehicle can move.

A new map revision is not necessarily a new perception observation.
"""

from dataclasses import dataclass

UINT64_MAX = (1 << 64) - 1


@dataclass(frozen=True)
class LaneMapProvenance:
    session_id: int
    map_revision: int
    evidence_revision: int
    latest_evidence_stamp_ns: int

    def __post_init__(self):
        values = (
            ("session_id", self.session_id, 1),
            ("map_revision", self.map_revision, 0),
            ("evidence_revision", self.evidence_revision, 0),
            (
                "latest_evidence_stamp_ns",
                self.latest_evidence_stamp_ns,
                0,
            ),
        )

        for name, value, minimum in values:
            if (
                type(value) is not int
                or not minimum <= value <= UINT64_MAX
            ):
                raise ValueError(f"Invalid {name}")

        if (
            self.evidence_revision == 0
            and self.latest_evidence_stamp_ns != 0
        ):
            raise ValueError(
                "Evidence timestamp without integrated evidence"
            )


@dataclass(frozen=True)
class LaneMapProvenanceDecision:
    status: str
    accept_map: bool
    reset_history: bool
    may_confirm_positive: bool
    may_apply_negative: bool


def classify_lane_map_provenance(
    previous: LaneMapProvenance | None,
    current: LaneMapProvenance,
) -> LaneMapProvenanceDecision:
    """Classify a received LaneMap relative to the last accepted map.

    may_confirm_positive is an eligibility flag, not a freshness
    guarantee. The caller must separately validate the latest
    evidence timestamp against the applicable sensor clock.
    """

    if not isinstance(current, LaneMapProvenance):
        raise TypeError("Expected LaneMapProvenance")

    if previous is not None and not isinstance(
        previous, LaneMapProvenance
    ):
        raise TypeError("Invalid previous provenance")

    def decision(
        status,
        accept=False,
        reset=False,
        positive=False,
        negative=False,
    ):
        return LaneMapProvenanceDecision(
            status=status,
            accept_map=accept,
            reset_history=reset,
            may_confirm_positive=positive,
            may_apply_negative=negative,
        )

    has_active_evidence = (
        current.evidence_revision > 0
        and current.latest_evidence_stamp_ns > 0
    )

    if previous is None:
        return decision(
            "INITIAL_MAP",
            accept=True,
            reset=True,
            positive=has_active_evidence,
            negative=True,
        )

    if current.session_id != previous.session_id:
        return decision(
            "SESSION_CHANGED",
            accept=True,
            reset=True,
            positive=has_active_evidence,
            negative=True,
        )

    if (
        current.map_revision < previous.map_revision
        or current.evidence_revision < previous.evidence_revision
    ):
        return decision("OUT_OF_ORDER")

    if current.map_revision == previous.map_revision:
        if current == previous:
            return decision("DUPLICATE")
        return decision("REVISION_CONFLICT")

    # The map revision increased.
    if current.evidence_revision == previous.evidence_revision:
        # Pruning may remove all active observations. Without
        # new integration, it cannot create a newer evidence stamp.
        if current.latest_evidence_stamp_ns not in (
            previous.latest_evidence_stamp_ns,
            0,
        ):
            return decision("REVISION_CONFLICT")

        return decision(
            "MAP_ONLY_CHANGE",
            accept=True,
            positive=False,
            negative=True,
        )

    # The evidence revision increased: successful integrations
    # occurred. The retained evidence timestamp must advance,
    # unless all currently active evidence has been removed.
    if (
        current.latest_evidence_stamp_ns > 0
        and previous.latest_evidence_stamp_ns > 0
        and current.latest_evidence_stamp_ns
        <= previous.latest_evidence_stamp_ns
    ):
        return decision("NON_MONOTONIC_EVIDENCE_TIMESTAMP")

    return decision(
        "NEW_EVIDENCE",
        accept=True,
        positive=has_active_evidence,
        negative=True,
    )
