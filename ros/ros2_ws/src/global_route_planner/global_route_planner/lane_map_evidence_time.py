"""Validate LaneMap evidence age in a shared sensor-clock domain.

Both timestamps must use the same clock (normally CARLA/ROS time).
Never compare ROS simulation timestamps with time.monotonic().
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class EvidenceTimeParameters:
    max_evidence_age_s: float = 2.0

    def __post_init__(self):
        if (
            not math.isfinite(self.max_evidence_age_s)
            or self.max_evidence_age_s <= 0.0
        ):
            raise ValueError("max_evidence_age_s must be positive and finite")


@dataclass(frozen=True)
class EvidenceTimeDecision:
    status: str
    may_confirm_positive: bool
    reset_temporal_history: bool
    evidence_age_s: float | None


def classify_evidence_time(
    evidence_stamp_ns: int,
    reference_stamp_ns: int,
    previous_reference_stamp_ns: int | None = None,
    parameters: EvidenceTimeParameters | None = None,
) -> EvidenceTimeDecision:
    """Check evidence freshness and detect backward clock jumps."""

    params = parameters or EvidenceTimeParameters()

    if not isinstance(params, EvidenceTimeParameters):
        raise TypeError("Invalid evidence-time parameters")

    for name, value in (
        ("evidence_stamp_ns", evidence_stamp_ns),
        ("reference_stamp_ns", reference_stamp_ns),
        ("previous_reference_stamp_ns", previous_reference_stamp_ns),
    ):
        if value is None and name == "previous_reference_stamp_ns":
            continue
        if type(value) is not int or value < 0:
            raise ValueError(f"Invalid {name}")

    def result(status, positive=False, reset=False, age=None):
        return EvidenceTimeDecision(
            status=status,
            may_confirm_positive=positive,
            reset_temporal_history=reset,
            evidence_age_s=age,
        )

    if (
        previous_reference_stamp_ns is not None
        and reference_stamp_ns < previous_reference_stamp_ns
    ):
        return result("CLOCK_REGRESSION", reset=True)

    if reference_stamp_ns == 0:
        return result("REFERENCE_TIME_UNAVAILABLE")

    if evidence_stamp_ns == 0:
        return result("NO_ACTIVE_EVIDENCE")

    age_ns = reference_stamp_ns - evidence_stamp_ns

    if age_ns < 0:
        return result("FUTURE_EVIDENCE")

    age_s = age_ns / 1_000_000_000.0

    if age_s > params.max_evidence_age_s:
        return result("STALE_EVIDENCE", age=age_s)

    return result("FRESH", positive=True, age=age_s)
