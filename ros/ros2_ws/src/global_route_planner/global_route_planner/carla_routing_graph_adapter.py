"""Construct sampled route-search connectivity from CARLA topology."""

from dataclasses import dataclass
import math

import carla

from global_route_planner.routing_graph import (
    DirectedRoutingGraph,
    RoutingEdge,
    RoutingEdgeKey,
    RoutingEdgeType,
    RoutingNode,
    RoutingNodeKey,
)

from global_route_planner.routing_graph_index import (
    RoutingGraphIndex,
)


DEFAULT_LANE_CHANGE_PENALTY_M = 10.0
DEFAULT_MAX_LANE_CHANGE_S_ERROR_M = 2.5


@dataclass(frozen=True)
class RoutingGraphBuildResult:
    """Routing graph plus construction diagnostics."""

    graph: DirectedRoutingGraph

    lane_follow_edges_added: int
    lane_follow_duplicates_skipped: int

    lane_change_permissions_seen: int
    lane_change_edges_added: int
    lane_change_duplicates_skipped: int
    lane_change_target_misses: int


def point_key(
    point,
) -> RoutingNodeKey:
    """Semantic identity of one sampled topology point."""

    return RoutingNodeKey(
        road_id=int(point.road_id),
        section_id=int(point.section_id),
        lane_id=int(point.lane_id),
        s_cm=int(
            round(
                float(point.s_m)
                * 100.0
            )
        ),
    )


def point_to_node(
    point,
) -> RoutingNode:
    """Convert sampled topology geometry to routing state."""

    return RoutingNode(
        key=point_key(point),
        s_m=float(point.s_m),
        x=float(point.x),
        y=float(point.y),
        z=float(point.z),
        yaw_deg=float(point.yaw_deg),
        is_junction=bool(
            point.is_junction
        ),
    )


def node_distance_m(
    a: RoutingNode,
    b: RoutingNode,
) -> float:
    """Euclidean displacement between routing samples."""

    dx = b.x - a.x
    dy = b.y - a.y
    dz = b.z - a.z

    return math.sqrt(
        dx * dx
        + dy * dy
        + dz * dz
    )



def build_routing_graph(
    carla_map,
    topology_graph,
    lane_change_penalty_m: float = (
        DEFAULT_LANE_CHANGE_PENALTY_M
    ),
    max_lane_change_s_error_m: float = (
        DEFAULT_MAX_LANE_CHANGE_S_ERROR_M
    ),
) -> RoutingGraphBuildResult:
    """Expand coarse topology into lane-level routing connectivity."""

    if lane_change_penalty_m < 0.0:
        raise ValueError(
            "lane_change_penalty_m cannot be negative"
        )

    if max_lane_change_s_error_m < 0.0:
        raise ValueError(
            "max_lane_change_s_error_m "
            "cannot be negative"
        )

    graph = DirectedRoutingGraph()

    # ---------------------------------------------------------
    # 1. Add every sampled topology point as a routing node.
    # ---------------------------------------------------------

    for topology_edge in topology_graph.edges.values():
        for point in topology_edge.geometry:
            graph.add_node(
                point_to_node(
                    point
                )
            )

    # ---------------------------------------------------------
    # 2. Add longitudinal sample-to-sample transitions.
    # ---------------------------------------------------------

    lane_follow_edges_added = 0
    lane_follow_duplicates_skipped = 0

    for topology_edge in topology_graph.edges.values():
        geometry = (
            topology_edge.geometry
        )

        for first, second in zip(
            geometry[:-1],
            geometry[1:],
        ):
            source_key = point_key(
                first
            )

            target_key = point_key(
                second
            )

            # Quantization can theoretically collapse two
            # extremely close consecutive points.
            if source_key == target_key:
                continue

            edge_key = RoutingEdgeKey(
                source=source_key,
                target=target_key,
                transition_type=(
                    RoutingEdgeType.LANE_FOLLOW
                ),
            )

            if edge_key in graph.edges:
                lane_follow_duplicates_skipped += 1
                continue

            source_node = graph.nodes[
                source_key
            ]

            target_node = graph.nodes[
                target_key
            ]

            length_m = node_distance_m(
                source_node,
                target_node,
            )

            graph.add_edge(
                RoutingEdge(
                    key=edge_key,
                    source=source_key,
                    target=target_key,
                    transition_type=(
                        RoutingEdgeType.LANE_FOLLOW
                    ),
                    length_m=length_m,
                    cost_m=length_m,
                    source_topology_edge_key=(
                        topology_edge.key
                    ),
                )
            )

            lane_follow_edges_added += 1

    # ---------------------------------------------------------
    # 3. Build reusable semantic lane/s index.
    # ---------------------------------------------------------

    routing_index = RoutingGraphIndex(
        graph
    )

    # ---------------------------------------------------------
    # 4. Add legal lateral transitions.
    #
    # Only interior geometry samples are considered, matching
    # the corrected runtime probe. Topology endpoints are not
    # treated as lane-change locations.
    # ---------------------------------------------------------

    lane_change_permissions_seen = 0
    lane_change_edges_added = 0
    lane_change_duplicates_skipped = 0
    lane_change_target_misses = 0

    for topology_edge in topology_graph.edges.values():
        for point in topology_edge.geometry[1:-1]:
            if point.is_junction:
                continue

            source_key = point_key(
                point
            )

            waypoint = (
                carla_map.get_waypoint_xodr(
                    int(point.road_id),
                    int(point.lane_id),
                    float(point.s_m),
                )
            )

            if waypoint is None:
                continue

            if (
                int(waypoint.section_id)
                != int(point.section_id)
            ):
                continue

            candidates = (
                (
                    RoutingEdgeType.LANE_CHANGE_LEFT,
                    waypoint.left_lane_marking,
                    carla.LaneChange.Left,
                    waypoint.get_left_lane,
                ),
                (
                    RoutingEdgeType.LANE_CHANGE_RIGHT,
                    waypoint.right_lane_marking,
                    carla.LaneChange.Right,
                    waypoint.get_right_lane,
                ),
            )

            for (
                transition_type,
                marking,
                permission,
                neighbor_getter,
            ) in candidates:

                if marking is None:
                    continue

                if not (
                    marking.lane_change
                    & permission
                ):
                    continue

                lane_change_permissions_seen += 1

                neighbor = (
                    neighbor_getter()
                )

                if neighbor is None:
                    lane_change_target_misses += 1
                    continue

                if (
                    neighbor.lane_type
                    != carla.LaneType.Driving
                ):
                    lane_change_target_misses += 1
                    continue

                if (
                    int(neighbor.road_id)
                    != int(waypoint.road_id)
                ):
                    lane_change_target_misses += 1
                    continue

                if (
                    int(neighbor.section_id)
                    != int(waypoint.section_id)
                ):
                    lane_change_target_misses += 1
                    continue

                target_match = (
                    routing_index.nearest_on_lane(
                        road_id=neighbor.road_id,
                        section_id=neighbor.section_id,
                        lane_id=neighbor.lane_id,
                        target_s_m=float(
                            neighbor.s
                        ),
                        max_s_error_m=(
                            max_lane_change_s_error_m
                        ),
                    )
                )

                if target_match is None:
                    lane_change_target_misses += 1
                    continue

                target_key = (
                    target_match.node_key
                )

                if target_key == source_key:
                    continue

                edge_key = RoutingEdgeKey(
                    source=source_key,
                    target=target_key,
                    transition_type=(
                        transition_type
                    ),
                )

                if edge_key in graph.edges:
                    lane_change_duplicates_skipped += 1
                    continue

                source_node = graph.nodes[
                    source_key
                ]

                target_node = graph.nodes[
                    target_key
                ]

                length_m = node_distance_m(
                    source_node,
                    target_node,
                )

                cost_m = (
                    length_m
                    + lane_change_penalty_m
                )

                graph.add_edge(
                    RoutingEdge(
                        key=edge_key,
                        source=source_key,
                        target=target_key,
                        transition_type=(
                            transition_type
                        ),
                        length_m=length_m,
                        cost_m=cost_m,
                        source_topology_edge_key=(
                            topology_edge.key
                        ),
                    )
                )

                lane_change_edges_added += 1

    return RoutingGraphBuildResult(
        graph=graph,
        lane_follow_edges_added=(
            lane_follow_edges_added
        ),
        lane_follow_duplicates_skipped=(
            lane_follow_duplicates_skipped
        ),
        lane_change_permissions_seen=(
            lane_change_permissions_seen
        ),
        lane_change_edges_added=(
            lane_change_edges_added
        ),
        lane_change_duplicates_skipped=(
            lane_change_duplicates_skipped
        ),
        lane_change_target_misses=(
            lane_change_target_misses
        ),
    )
