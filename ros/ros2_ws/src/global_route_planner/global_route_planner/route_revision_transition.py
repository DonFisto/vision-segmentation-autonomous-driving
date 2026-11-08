"""Classify transitions between global route revisions.

Pure identity logic. Never assumes two route-relative coordinates
refer to the same world location across different route revisions.
"""

from dataclasses import dataclass

from global_route_planner.route_identity import RouteIdentity


@dataclass(frozen=True)
class RouteRevisionTransition:
    status: str
    preserve_observations: bool
    reuse_route_coordinates: bool
    requires_geometry_alignment: bool


def classify_route_transition(
    previous: RouteIdentity | None,
    current: RouteIdentity,
) -> RouteRevisionTransition:
    """Determine how consumers should handle a route identity change."""

    if not isinstance(current, RouteIdentity):
        raise TypeError("Current route must be a RouteIdentity")

    if previous is not None and not isinstance(
        previous, RouteIdentity
    ):
        raise TypeError("Previous route must be a RouteIdentity or None")

    if previous is None:
        return RouteRevisionTransition(
            status="INITIAL_ROUTE",
            preserve_observations=False,
            reuse_route_coordinates=False,
            requires_geometry_alignment=False,
        )

    if current.map_revision != previous.map_revision:
        return RouteRevisionTransition(
            status="MAP_CHANGED",
            preserve_observations=False,
            reuse_route_coordinates=False,
            requires_geometry_alignment=False,
        )

    if current.mission_id != previous.mission_id:
        return RouteRevisionTransition(
            status="MISSION_CHANGED",
            preserve_observations=False,
            reuse_route_coordinates=False,
            requires_geometry_alignment=False,
        )

    if current.route_id < previous.route_id:
        return RouteRevisionTransition(
            status="OUT_OF_ORDER_REVISION",
            preserve_observations=False,
            reuse_route_coordinates=False,
            requires_geometry_alignment=False,
        )

    if current.route_id > previous.route_id:
        return RouteRevisionTransition(
            status="ROUTE_REVISED",
            preserve_observations=False,
            reuse_route_coordinates=False,
            requires_geometry_alignment=True,
        )

    return RouteRevisionTransition(
        status="SAME_ROUTE",
        preserve_observations=True,
        reuse_route_coordinates=True,
        requires_geometry_alignment=False,
    )
