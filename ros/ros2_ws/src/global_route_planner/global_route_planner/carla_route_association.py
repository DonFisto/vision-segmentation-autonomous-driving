"""Associate CARLA-world positions with sampled routing states."""

from dataclasses import dataclass
import math

import carla

from global_route_planner.routing_graph import (
    DirectedRoutingGraph,
    RoutingNodeKey,
)
from global_route_planner.routing_graph_index import (
    RoutingGraphIndex,
)


DEFAULT_MAX_PROJECTION_DISTANCE_M = 5.0
DEFAULT_MAX_SAMPLE_S_ERROR_M = 2.5


class RouteAssociationError(RuntimeError):
    """Raised when a world position cannot be safely map-associated."""


@dataclass(frozen=True)
class CarlaRouteAssociation:
    """Result of associating one CARLA-world position."""

    node_key: RoutingNodeKey

    road_id: int
    section_id: int
    lane_id: int

    waypoint_s_m: float

    projected_x: float
    projected_y: float
    projected_z: float
    projected_yaw_deg: float

    projection_distance_m: float
    sample_s_error_m: float
    sample_distance_m: float


def distance_xyz_m(
    ax,
    ay,
    az,
    bx,
    by,
    bz,
) -> float:
    dx = float(bx) - float(ax)
    dy = float(by) - float(ay)
    dz = float(bz) - float(az)

    return math.sqrt(
        dx * dx
        + dy * dy
        + dz * dz
    )


def associate_carla_world_position(
    carla_map,
    routing_graph: DirectedRoutingGraph,
    routing_index: RoutingGraphIndex,
    x: float,
    y: float,
    z: float,
    max_projection_distance_m: float = (
        DEFAULT_MAX_PROJECTION_DISTANCE_M
    ),
    max_sample_s_error_m: float = (
        DEFAULT_MAX_SAMPLE_S_ERROR_M
    ),
) -> CarlaRouteAssociation:
    """Project a CARLA-world position to a routing state.

    Association is rejected when CARLA must project too far to
    reach a driving lane, or when the corresponding sampled
    routing state is too far away in OpenDRIVE s.
    """

    if max_projection_distance_m < 0.0:
        raise ValueError(
            "max_projection_distance_m cannot be negative"
        )

    if max_sample_s_error_m < 0.0:
        raise ValueError(
            "max_sample_s_error_m cannot be negative"
        )

    query = carla.Location(
        x=float(x),
        y=float(y),
        z=float(z),
    )

    waypoint = carla_map.get_waypoint(
        query,
        project_to_road=True,
        lane_type=carla.LaneType.Driving,
    )

    if waypoint is None:
        raise RouteAssociationError(
            "CARLA could not project the position "
            "onto a driving lane."
        )

    projected = (
        waypoint.transform.location
    )

    projection_distance_m = (
        distance_xyz_m(
            query.x,
            query.y,
            query.z,
            projected.x,
            projected.y,
            projected.z,
        )
    )

    if (
        projection_distance_m
        > max_projection_distance_m
    ):
        raise RouteAssociationError(
            "Driving-lane projection exceeds threshold: "
            f"{projection_distance_m:.3f} m > "
            f"{max_projection_distance_m:.3f} m"
        )

    match = routing_index.nearest_on_lane(
        road_id=waypoint.road_id,
        section_id=waypoint.section_id,
        lane_id=waypoint.lane_id,
        target_s_m=float(
            waypoint.s
        ),
        max_s_error_m=(
            max_sample_s_error_m
        ),
    )

    if match is None:
        raise RouteAssociationError(
            "No sampled routing node matches "
            "projected OpenDRIVE lane state: "
            f"road={waypoint.road_id} "
            f"section={waypoint.section_id} "
            f"lane={waypoint.lane_id} "
            f"s={waypoint.s:.3f}"
        )

    node = routing_graph.nodes[
        match.node_key
    ]

    sample_distance_m = (
        distance_xyz_m(
            projected.x,
            projected.y,
            projected.z,
            node.x,
            node.y,
            node.z,
        )
    )

    return CarlaRouteAssociation(
        node_key=match.node_key,
        road_id=int(
            waypoint.road_id
        ),
        section_id=int(
            waypoint.section_id
        ),
        lane_id=int(
            waypoint.lane_id
        ),
        waypoint_s_m=float(
            waypoint.s
        ),
        projected_x=float(
            projected.x
        ),
        projected_y=float(
            projected.y
        ),
        projected_z=float(
            projected.z
        ),
        projected_yaw_deg=float(
            waypoint.transform.rotation.yaw
        ),
        projection_distance_m=(
            projection_distance_m
        ),
        sample_s_error_m=(
            match.s_error_m
        ),
        sample_distance_m=(
            sample_distance_m
        ),
    )
