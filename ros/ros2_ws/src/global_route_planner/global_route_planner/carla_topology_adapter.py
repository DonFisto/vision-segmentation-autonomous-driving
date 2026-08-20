"""Convert CARLA map topology into project graph structures."""

import math

from global_route_planner.topology import (
    DirectedTopologyGraph,
    TopologyEdge,
    TopologyEdgeKey,
    TopologyNode,
    TopologyNodeKey,
)


def waypoint_key(waypoint) -> TopologyNodeKey:
    """Create semantic graph identity from a CARLA waypoint."""

    return TopologyNodeKey(
        road_id=int(waypoint.road_id),
        section_id=int(waypoint.section_id),
        lane_id=int(waypoint.lane_id),
        s_cm=int(round(float(waypoint.s) * 100.0)),
    )


def waypoint_to_node(waypoint) -> TopologyNode:
    """Convert a CARLA waypoint endpoint to a graph node."""

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


def endpoint_distance_m(
    source: TopologyNode,
    target: TopologyNode,
) -> float:
    """Straight-line endpoint distance.

    This is diagnostic geometry only and must not yet be
    interpreted as the final routing edge cost.
    """

    dx = target.x - source.x
    dy = target.y - source.y
    dz = target.z - source.z

    return math.sqrt(
        dx * dx
        + dy * dy
        + dz * dz
    )


def build_topology_graph(carla_map) -> DirectedTopologyGraph:
    """Build directed graph from carla.Map.get_topology()."""

    graph = DirectedTopologyGraph()

    for entry_waypoint, exit_waypoint in carla_map.get_topology():
        source = waypoint_to_node(entry_waypoint)
        target = waypoint_to_node(exit_waypoint)

        edge_key = TopologyEdgeKey(
            source=source.key,
            target=target.key,
        )

        edge = TopologyEdge(
            key=edge_key,
            source=source.key,
            target=target.key,
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
