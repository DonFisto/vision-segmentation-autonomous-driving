"""ROS-independent temporal confirmation of route-support diagnostics.

Observation times are caller-provided monotonic seconds from one clock.
This module does not validate ROS sensor timestamps, infer missing evidence,
or turn a support decision into permission for vehicle movement.

All geometry-dependent history resets on every accepted route revision,
even when the revisions are aligned in world coordinates.
"""

from dataclasses import dataclass
import math

from global_route_planner.route_identity import RouteIdentity
from global_route_planner.route_revision_alignment import RouteAlignmentParameters
from global_route_planner.route_revision_consistency import (
    RouteRevisionConsistency,
    assess_route_revision_consistency,
)
from global_route_planner.route_support_policy import RouteSupportDecision


@dataclass(frozen=True)
class TemporalSupportParameters:
    min_consecutive_supported: int = 3
    max_inter_observation_gap_s: float = 1.5
    max_observation_age_s: float = 2.0
    freshness_decay_tau_s: float = 1.0

    def __post_init__(self):
        if (
            type(self.min_consecutive_supported) is not int
            or self.min_consecutive_supported < 1
        ):
            raise ValueError("min_consecutive_supported must be a positive int")

        for name in (
            "max_inter_observation_gap_s",
            "max_observation_age_s",
            "freshness_decay_tau_s",
        ):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")


@dataclass(frozen=True)
class TemporalSupportSnapshot:
    status: str
    reason: str
    route_identity: RouteIdentity | None
    latest_decision_status: str | None
    consecutive_supported: int
    observations_in_revision: int
    last_lane_map_revision: int | None
    latest_observation_age_s: float | None
    freshness_weight: float


class TemporalRouteSupportTracker:
    """Maintain evidence only within one unchanged route revision.

    SUPPORTED confirms repeated diagnostic decisions, not a probability
    or an autonomous-driving safety authorization.
    PARTIAL, UNAVAILABLE, and REJECTED take effect immediately.
    """

    def __init__(self, parameters: TemporalSupportParameters | None = None):
        if parameters is None:
            parameters = TemporalSupportParameters()
        if not isinstance(parameters, TemporalSupportParameters):
            raise TypeError("parameters must be TemporalSupportParameters")

        self._params = parameters
        self._identity = None
        self._points = None
        self._clear_history()

    def _clear_history(self):
        self._latest_status = None
        self._last_observation_time_s = None
        self._consecutive_supported = 0
        self._observations_in_revision = 0
        self._last_lane_map_revision = None

    def reset_evidence_history(self) -> None:
        """Clear temporal evidence without replacing the active route."""
        self._clear_history()

    @property
    def route_identity(self) -> RouteIdentity | None:
        return self._identity

    def register_route(
        self,
        identity: RouteIdentity,
        points: list[tuple[float, float]],
        alignment_parameters: RouteAlignmentParameters | None = None,
    ) -> RouteRevisionConsistency:
        """Apply route consistency; never replace an ignored route."""

        consistency = assess_route_revision_consistency(
            previous_identity=self._identity,
            previous_points=self._points,
            current_identity=identity,
            current_points=points,
            alignment_parameters=alignment_parameters,
        )

        if consistency.accept_current_route:
            if consistency.history_action == "RESET":
                self._clear_history()
            elif consistency.history_action != "KEEP":
                raise RuntimeError("Unknown route history action")

            self._identity = identity
            self._points = tuple((float(x), float(y)) for x, y in points)

        return consistency

    def observe(
        self,
        identity: RouteIdentity,
        decision: RouteSupportDecision,
        observed_at_s: float,
        lane_map_revision: int,
    ) -> TemporalSupportSnapshot:
        """Consume one fresh observation from the active route revision.

        The caller must verify upstream sensor freshness and ensure
        this decision belongs to the supplied RouteIdentity.
        LaneMap revisions must advance for independent observations.
        """

        if self._identity is None or identity != self._identity:
            raise ValueError("Observation does not match the active route")
        if not isinstance(decision, RouteSupportDecision):
            raise TypeError("decision must be RouteSupportDecision")
        if decision.status not in (
            "SUPPORTED", "PARTIAL", "UNAVAILABLE", "REJECTED"
        ):
            raise ValueError("Unknown route support status")
        if type(lane_map_revision) is not int or lane_map_revision < 0:
            raise ValueError("lane_map_revision must be a nonnegative int")
        if (
            self._last_lane_map_revision is not None
            and lane_map_revision <= self._last_lane_map_revision
        ):
            raise ValueError("LaneMap revision must increase strictly")
        if not math.isfinite(observed_at_s) or observed_at_s < 0.0:
            raise ValueError(
                "Observation timestamp must be finite and nonnegative"
            )
        if (
            self._last_observation_time_s is not None
            and observed_at_s <= self._last_observation_time_s
        ):
            raise ValueError("Observation timestamps must increase strictly")

        if decision.status == "SUPPORTED":
            if (
                self._latest_status == "SUPPORTED"
                and self._last_observation_time_s is not None
                and observed_at_s - self._last_observation_time_s
                <= min(
                    self._params.max_inter_observation_gap_s,
                    self._params.max_observation_age_s,
                )
            ):
                self._consecutive_supported += 1
            else:
                self._consecutive_supported = 1
        else:
            self._consecutive_supported = 0

        self._latest_status = decision.status
        self._last_observation_time_s = observed_at_s
        self._last_lane_map_revision = lane_map_revision
        self._observations_in_revision += 1

        return self.snapshot(observed_at_s)

    def snapshot(self, now_s: float) -> TemporalSupportSnapshot:
        """Evaluate whether the last observation remains fresh."""

        if not math.isfinite(now_s) or now_s < 0.0:
            raise ValueError("Snapshot time must be finite and nonnegative")
        if (
            self._last_observation_time_s is not None
            and now_s < self._last_observation_time_s
        ):
            raise ValueError("Snapshot time precedes last observation")

        age = (
            None if self._last_observation_time_s is None
            else now_s - self._last_observation_time_s
        )
        freshness = (
            0.0 if age is None
            else math.exp(-age / self._params.freshness_decay_tau_s)
        )

        if self._identity is None:
            status, reason = "NO_ROUTE", "NO_ACTIVE_ROUTE"
        elif age is None:
            status, reason = "WAITING", "NO_SUPPORT_OBSERVATION"
        elif age > self._params.max_observation_age_s:
            status, reason = "STALE", "SUPPORT_OBSERVATION_EXPIRED"
        elif self._latest_status == "SUPPORTED":
            if (
                self._consecutive_supported
                >= self._params.min_consecutive_supported
            ):
                status, reason = (
                    "SUPPORTED",
                    "CONSECUTIVE_SUPPORT_CONFIRMED",
                )
            else:
                status, reason = (
                    "CANDIDATE",
                    "INSUFFICIENT_CONSECUTIVE_SUPPORT",
                )
        else:
            status = self._latest_status
            reason = "LATEST_SUPPORT_" + self._latest_status

        return TemporalSupportSnapshot(
            status=status,
            reason=reason,
            route_identity=self._identity,
            latest_decision_status=self._latest_status,
            consecutive_supported=self._consecutive_supported,
            observations_in_revision=self._observations_in_revision,
            last_lane_map_revision=self._last_lane_map_revision,
            latest_observation_age_s=age,
            freshness_weight=freshness,
        )
