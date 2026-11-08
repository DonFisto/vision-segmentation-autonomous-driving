"""Combine route identity transitions with conservative geometry alignment.

Pure decision layer: no ROS, no perception, no temporal filter.

An aligned route revision does not share its predecessor's route-relative
s coordinates, and alignment does not make old perception evidence current.
"""

from dataclasses import dataclass
import math

from global_route_planner.route_identity import RouteIdentity
from global_route_planner.route_revision_transition import (
    classify_route_transition,
)
from global_route_planner.route_revision_alignment import (
    RouteAlignmentParameters,
    RouteRevisionAlignment,
    compare_route_revisions,
)


@dataclass(frozen=True)
class RouteRevisionConsistency:
    status: str
    reason: str
    transition_status: str
    accept_current_route: bool
    history_action: str
    reuse_route_coordinates: bool
    geometry_correspondence_candidate: bool
    alignment: RouteRevisionAlignment | None


def _validated_polyline(points, name):
    if points is None:
        raise ValueError(f"{name} geometry is missing")

    try:
        normalized = tuple(
            (float(x), float(y))
            for x, y in points
        )
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} must contain XY pairs") from exc

    if len(normalized) < 2:
        raise ValueError(f"{name} requires at least two points")

    if not all(
        math.isfinite(x) and math.isfinite(y)
        for x, y in normalized
    ):
        raise ValueError(f"{name} contains non-finite points")

    length = 0.0

    for (x0, y0), (x1, y1) in zip(
        normalized, normalized[1:]
    ):
        segment_length = math.hypot(x1 - x0, y1 - y0)

        if not math.isfinite(segment_length):
            raise ValueError(
                f"{name} has non-finite segment length"
            )

        length += segment_length

    if not math.isfinite(length) or length <= 0.0:
        raise ValueError(f"{name} has invalid or zero length")

    return normalized


def assess_route_revision_consistency(
    previous_identity: RouteIdentity | None,
    previous_points: list[tuple[float, float]] | None,
    current_identity: RouteIdentity,
    current_points: list[tuple[float, float]],
    alignment_parameters: RouteAlignmentParameters | None = None,
) -> RouteRevisionConsistency:
    """Classify handling of an incoming geometrically valid route.

    KEEP retains the currently accepted route's history.
    RESET discards geometry-dependent history.
    An ignored route must not replace the accepted route.

    ALIGNED revisions may support future world-coordinate
    reprojection, but existing route-relative observations are
    never transferred by this function.
    """

    transition = classify_route_transition(
        previous_identity,
        current_identity,
    )

    if alignment_parameters is not None and not isinstance(
        alignment_parameters,
        RouteAlignmentParameters,
    ):
        raise TypeError(
            "alignment_parameters must be RouteAlignmentParameters"
        )

    current_xy = _validated_polyline(
        current_points,
        "Current route",
    )

    if previous_identity is None:
        if previous_points is not None:
            raise ValueError(
                "Previous geometry given without previous identity"
            )
        previous_xy = None
    else:
        previous_xy = _validated_polyline(
            previous_points,
            "Previous route",
        )

    def make(
        status,
        reason,
        accept,
        history,
        reuse=False,
        candidate=False,
        alignment=None,
    ):
        return RouteRevisionConsistency(
            status=status,
            reason=reason,
            transition_status=transition.status,
            accept_current_route=accept,
            history_action=history,
            reuse_route_coordinates=reuse,
            geometry_correspondence_candidate=candidate,
            alignment=alignment,
        )

    if transition.status == "INITIAL_ROUTE":
        return make(
            "INITIAL_ROUTE",
            "FIRST_ACCEPTED_ROUTE",
            True,
            "RESET",
        )

    if transition.status == "MAP_CHANGED":
        return make(
            "MAP_CHANGED",
            "GLOBAL_MAP_REVISION_CHANGED",
            True,
            "RESET",
        )

    if transition.status == "MISSION_CHANGED":
        return make(
            "MISSION_CHANGED",
            "ACCEPTED_GOAL_CHANGED",
            True,
            "RESET",
        )

    if transition.status == "OUT_OF_ORDER_REVISION":
        return make(
            "OUT_OF_ORDER_REVISION",
            "OLDER_ROUTE_REVISION",
            False,
            "KEEP",
        )

    if transition.status == "SAME_ROUTE":
        # A route revision must be immutable. The same identity
        # with different geometry is an inconsistent publication.
        if previous_xy != current_xy:
            return make(
                "ROUTE_ID_GEOMETRY_CONFLICT",
                "SAME_ROUTE_ID_DIFFERENT_GEOMETRY",
                False,
                "KEEP",
            )

        return make(
            "SAME_ROUTE",
            "UNCHANGED_ROUTE_GEOMETRY",
            True,
            "KEEP",
            reuse=True,
        )

    if transition.status != "ROUTE_REVISED":
        raise RuntimeError(
            f"Unexpected transition: {transition.status}"
        )

    alignment = compare_route_revisions(
        list(previous_xy),
        list(current_xy),
        alignment_parameters,
    )

    if alignment.status == "ALIGNED":
        return make(
            "ROUTE_REVISED_ALIGNED",
            alignment.reason,
            True,
            "RESET",
            candidate=True,
            alignment=alignment,
        )

    if alignment.status == "DIVERGED":
        return make(
            "ROUTE_REVISED_DIVERGED",
            alignment.reason,
            True,
            "RESET",
            alignment=alignment,
        )

    if alignment.status == "INSUFFICIENT_OVERLAP":
        return make(
            "ROUTE_REVISED_UNCERTAIN",
            alignment.reason,
            True,
            "RESET",
            alignment=alignment,
        )

    raise RuntimeError(
        f"Unexpected alignment: {alignment.status}"
    )
