"""Convert CARLA map topology into project graph structures."""

import math
from typing import Tuple

from global_route_planner.topology import (
    DirectedTopologyGraph,
    TopologyEdge,
    TopologyEdgeKey,
    TopologyNode,
    TopologyNodeKey,
    TopologyPoint,
)


DEFAULT_SAMPLING_RESOLUTION_M = 2.0
DEFAULT_MAX_SAMPLING_STEPS = 1000


def waypoint_key(waypoint) -> TopologyNodeKey:
    """Create semantic graph identity from a CARLA waypoint."""

    return TopologyNodeKey(
        road_id=int(waypoint.road_id),
        section_id=int(waypoint.section_id),
        lane_id=int(waypoint.lane_id),
        s_cm=int(round(float(waypoint.s) * 100.0)),
    )


def waypoint_to_node(waypoint) -> TopologyNode:
    """Convert a CARLA topology endpoint to a graph node."""

    transform = waypoint.transform
    location = transform.location

    return TopologyNode(
        key=waypoint_key(waypoint),
        s_m=float(waypoint.s),
        x=float(location.x),
        y=float(location.y),
        z=float(location.z),
        yaw_deg=float(transform.rotation.yaw),
        is_junction=bool(waypoint.is_junction),
    )


def waypoint_to_point(waypoint) -> TopologyPoint:
    """Convert a CARLA waypoint to pure sampled geometry."""

    transform = waypoint.transform
    location = transform.location

    return TopologyPoint(
        x=float(location.x),
        y=float(location.y),
        z=float(location.z),
        yaw_deg=float(transform.rotation.yaw),
        road_id=int(waypoint.road_id),
        section_id=int(waypoint.section_id),
        lane_id=int(waypoint.lane_id),
        s_m=float(waypoint.s),
        is_junction=bool(waypoint.is_junction),
    )


def point_distance_m(a: TopologyPoint, b: TopologyPoint) -> float:
    dx = b.x - a.x
    dy = b.y - a.y
    dz = b.z - a.z

    return math.sqrt(
        dx * dx
        + dy * dy
        + dz * dz
    )


def endpoint_distance_m(
    source: TopologyNode,
    target: TopologyNode,
) -> float:
    """Straight-line chord distance for diagnostics."""

    dx = target.x - source.x
    dy = target.y - source.y
    dz = target.z - source.z

    return math.sqrt(
        dx * dx
        + dy * dy
        + dz * dz
    )


def sample_topology_edge_geometry(
    entry_waypoint,
    exit_waypoint,
    sampling_resolution_m: float,
    max_steps: int,
) -> Tuple[Tuple[TopologyPoint, ...], float]:
    """Sample legal nominal geometry from entry to exit.

    The current Town10HD runtime experiment found zero
    internal branch events for all 200 topology segments.

    Nevertheless, this function fails explicitly if CARLA
    returns multiple successors inside a topology segment.
    We do not silently choose an arbitrary branch.
    """

    if sampling_resolution_m <= 0.0:
        raise ValueError(
            "sampling_resolution_m must be positive"
        )

    if max_steps <= 0:
        raise ValueError(
            "max_steps must be positive"
        )

    exit_location = exit_waypoint.transform.location

    sampled_waypoints = [entry_waypoint]
    current = entry_waypoint

    for _ in range(max_steps):
        gap_m = current.transform.location.distance(
            exit_location
        )

        if gap_m <= sampling_resolution_m:
            break

        successors = current.next(
            sampling_resolution_m
        )

        if not successors:
            raise RuntimeError(
                "Topology geometry sampling reached a "
                "waypoint with no successor before the "
                "expected exit endpoint."
            )

        if len(successors) != 1:
            raise RuntimeError(
                "Ambiguous internal topology sampling: "
                f"expected one successor but received "
                f"{len(successors)}."
            )

        current = successors[0]
        sampled_waypoints.append(current)

    else:
        raise RuntimeError(
            "Topology geometry sampling exceeded "
            f"{max_steps} steps."
        )

    # Always terminate at the exact topology endpoint.
    sampled_waypoints.append(exit_waypoint)

    geometry = tuple(
        waypoint_to_point(waypoint)
        for waypoint in sampled_waypoints
    )

    length_m = sum(
        point_distance_m(a, b)
        for a, b in zip(
            geometry[:-1],
            geometry[1:],
        )
    )

    return geometry, length_m


def build_topology_graph(
    carla_map,
    sampling_resolution_m: float = (
        DEFAULT_SAMPLING_RESOLUTION_M
    ),
    max_sampling_steps: int = (
        DEFAULT_MAX_SAMPLING_STEPS
    ),
) -> DirectedTopologyGraph:
    """Build a directed global-routing graph."""

    graph = DirectedTopologyGraph()

    for entry_waypoint, exit_waypoint in carla_map.get_topology():
        source = waypoint_to_node(entry_waypoint)
        target = waypoint_to_node(exit_waypoint)

        geometry, length_m = (
            sample_topology_edge_geometry(
                entry_waypoint=entry_waypoint,
                exit_waypoint=exit_waypoint,
                sampling_resolution_m=(
                    sampling_resolution_m
                ),
                max_steps=max_sampling_steps,
            )
        )

        edge_key = TopologyEdgeKey(
            source=source.key,
            target=target.key,
        )

        edge = TopologyEdge(
            key=edge_key,
            source=source.key,
            target=target.key,
            geometry=geometry,
            length_m=length_m,
            endpoint_distance_m=endpoint_distance_m(
                source,
                target,
            ),
            entry_is_junction=source.is_junction,
            exit_is_junction=target.is_junction,
        )

        graph.add_edge(
            source_node=source,
            target_node=target,
            edge=edge,
        )

    return graph
