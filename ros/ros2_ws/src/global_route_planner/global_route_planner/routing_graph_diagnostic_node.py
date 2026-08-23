#!/usr/bin/env python3

"""Runtime validation of the sampled lane-level routing graph."""

from collections import (
    Counter,
    deque,
)
import math

import carla
import rclpy
from rclpy.node import Node

from global_route_planner.carla_routing_graph_adapter import (
    build_routing_graph,
)
from global_route_planner.carla_topology_adapter import (
    build_topology_graph,
)
from global_route_planner.connectivity import (
    strongly_connected_components,
    weakly_connected_components,
)
from global_route_planner.routing import (
    a_star_shortest_path,
    dijkstra_shortest_path,
)
from global_route_planner.routing_graph import (
    RoutingEdgeType,
)


def reachable_nodes(
    graph,
    start,
):
    visited = {
        start
    }

    queue = deque(
        [start]
    )

    while queue:
        current = queue.popleft()

        for edge_key in graph.outgoing.get(
            current,
            (),
        ):
            target = graph.edges[
                edge_key
            ].target

            if target in visited:
                continue

            visited.add(
                target
            )

            queue.append(
                target
            )

    return visited


def node_distance_m(
    graph,
    first,
    second,
):
    a = graph.nodes[
        first
    ]

    b = graph.nodes[
        second
    ]

    dx = b.x - a.x
    dy = b.y - a.y
    dz = b.z - a.z

    return math.sqrt(
        dx * dx
        + dy * dy
        + dz * dz
    )


class RoutingGraphDiagnosticNode(Node):
    def __init__(self) -> None:
        super().__init__(
            "global_route_sampled_graph_diagnostic"
        )

        self.declare_parameter(
            "host",
            "localhost",
        )

        self.declare_parameter(
            "port",
            2000,
        )

        self.declare_parameter(
            "timeout_s",
            5.0,
        )

        self.declare_parameter(
            "sampling_resolution_m",
            2.0,
        )

        self.declare_parameter(
            "lane_change_penalty_m",
            10.0,
        )

        self.declare_parameter(
            "max_lane_change_s_error_m",
            2.5,
        )

        host = str(
            self.get_parameter(
                "host"
            ).value
        )

        port = int(
            self.get_parameter(
                "port"
            ).value
        )

        timeout_s = float(
            self.get_parameter(
                "timeout_s"
            ).value
        )

        sampling_resolution_m = float(
            self.get_parameter(
                "sampling_resolution_m"
            ).value
        )

        lane_change_penalty_m = float(
            self.get_parameter(
                "lane_change_penalty_m"
            ).value
        )

        max_lane_change_s_error_m = float(
            self.get_parameter(
                "max_lane_change_s_error_m"
            ).value
        )

        self.get_logger().info(
            f"Connecting to CARLA at "
            f"{host}:{port}"
        )

        client = carla.Client(
            host,
            port,
        )

        client.set_timeout(
            timeout_s
        )

        world = client.get_world()
        carla_map = world.get_map()

        topology_graph = (
            build_topology_graph(
                carla_map,
                sampling_resolution_m=(
                    sampling_resolution_m
                ),
            )
        )

        build_result = (
            build_routing_graph(
                carla_map=carla_map,
                topology_graph=topology_graph,
                lane_change_penalty_m=(
                    lane_change_penalty_m
                ),
                max_lane_change_s_error_m=(
                    max_lane_change_s_error_m
                ),
            )
        )

        self._report(
            map_name=carla_map.name,
            topology_graph=topology_graph,
            build_result=build_result,
            lane_change_penalty_m=(
                lane_change_penalty_m
            ),
        )

    def _report(
        self,
        map_name,
        topology_graph,
        build_result,
        lane_change_penalty_m,
    ) -> None:
        logger = self.get_logger()

        graph = build_result.graph

        weak_components = (
            weakly_connected_components(
                graph
            )
        )

        strong_components = (
            strongly_connected_components(
                graph
            )
        )

        weak_sizes = sorted(
            (
                len(component)
                for component
                in weak_components
            ),
            reverse=True,
        )

        strong_sizes = sorted(
            (
                len(component)
                for component
                in strong_components
            ),
            reverse=True,
        )

        transition_counts = Counter(
            edge.transition_type.name
            for edge in graph.edges.values()
        )

        start = min(
            graph.nodes
        )

        reachable = reachable_nodes(
            graph,
            start,
        )

        if len(reachable) < 2:
            raise RuntimeError(
                "Routing graph start has no "
                "nontrivial reachable destination."
            )

        goal = max(
            (
                node
                for node in reachable
                if node != start
            ),
            key=lambda node: (
                node_distance_m(
                    graph,
                    start,
                    node,
                ),
                node,
            ),
        )

        route = dijkstra_shortest_path(
            graph=graph,
            start=start,
            goal=goal,
        )

        # -----------------------------------------------------
        # Validate the geometric lower-bound assumption used
        # by the Euclidean A* heuristic.
        #
        # For every routing edge:
        #
        #   cost(u,v) >= euclidean_distance(u,v)
        #
        # Together with the triangle inequality this makes
        # Euclidean distance to the goal a consistent
        # heuristic.
        # -----------------------------------------------------

        min_cost_minus_displacement_m = math.inf

        for edge in graph.edges.values():
            displacement_m = (
                node_distance_m(
                    graph,
                    edge.source,
                    edge.target,
                )
            )

            slack_m = (
                float(edge.cost_m)
                - displacement_m
            )

            min_cost_minus_displacement_m = min(
                min_cost_minus_displacement_m,
                slack_m,
            )

            if slack_m < -1e-6:
                raise RuntimeError(
                    "Euclidean A* heuristic consistency "
                    "assumption failed: "
                    f"edge={edge.key} "
                    f"cost_m={edge.cost_m:.6f} "
                    f"displacement_m={displacement_m:.6f} "
                    f"slack_m={slack_m:.6f}"
                )

        def goal_heuristic(
            node,
        ):
            return node_distance_m(
                graph,
                node,
                goal,
            )

        astar_route = (
            a_star_shortest_path(
                graph=graph,
                start=start,
                goal=goal,
                heuristic=goal_heuristic,
            )
        )

        if not math.isclose(
            astar_route.total_cost_m,
            route.total_cost_m,
            rel_tol=1e-9,
            abs_tol=1e-6,
        ):
            raise RuntimeError(
                "A* and Dijkstra disagree on "
                "minimum route cost: "
                f"dijkstra={route.total_cost_m:.6f} "
                f"astar={astar_route.total_cost_m:.6f}"
            )

        if (
            astar_route.settled_nodes
            > route.settled_nodes
        ):
            raise RuntimeError(
                "A* settled more nodes than Dijkstra "
                "for the diagnostic route: "
                f"dijkstra={route.settled_nodes} "
                f"astar={astar_route.settled_nodes}"
            )

        astar_summed_cost_m = sum(
            graph.edges[
                edge_key
            ].cost_m
            for edge_key
            in astar_route.edge_path
        )

        if not math.isclose(
            astar_route.total_cost_m,
            astar_summed_cost_m,
            rel_tol=1e-9,
            abs_tol=1e-6,
        ):
            raise RuntimeError(
                "A* cost does not equal "
                "its edge-cost sum."
            )

        for index, edge_key in enumerate(
            astar_route.edge_path
        ):
            edge = graph.edges[
                edge_key
            ]

            if (
                edge.source
                != astar_route.node_path[index]
                or edge.target
                != astar_route.node_path[index + 1]
            ):
                raise RuntimeError(
                    "A* route continuity failed."
                )

        astar_lane_changes = sum(
            1
            for edge_key
            in astar_route.edge_path
            if graph.edges[
                edge_key
            ].transition_type
            != RoutingEdgeType.LANE_FOLLOW
        )

        summed_cost_m = sum(
            graph.edges[
                edge_key
            ].cost_m
            for edge_key in route.edge_path
        )

        if not math.isclose(
            route.total_cost_m,
            summed_cost_m,
            rel_tol=1e-9,
            abs_tol=1e-6,
        ):
            raise RuntimeError(
                "Routing-graph Dijkstra cost "
                "does not equal edge-cost sum."
            )

        for index, edge_key in enumerate(
            route.edge_path
        ):
            edge = graph.edges[
                edge_key
            ]

            if (
                edge.source
                != route.node_path[index]
                or edge.target
                != route.node_path[index + 1]
            ):
                raise RuntimeError(
                    "Routing path continuity failed."
                )

        lane_changes = sum(
            1
            for edge_key in route.edge_path
            if graph.edges[
                edge_key
            ].transition_type
            != RoutingEdgeType.LANE_FOLLOW
        )

        # -----------------------------------------------------
        # Explicitly validate that Dijkstra can use a legal
        # lateral transition. The general route above may not
        # require a lane change even though the graph contains
        # them.
        # -----------------------------------------------------

        lateral_edges = [
            edge
            for edge in graph.edges.values()
            if edge.transition_type
            != RoutingEdgeType.LANE_FOLLOW
        ]

        if not lateral_edges:
            raise RuntimeError(
                "Routing graph contains no lane-change edges."
            )

        lane_change_probe_edge = min(
            lateral_edges,
            key=lambda edge: edge.key,
        )

        lane_change_probe = (
            dijkstra_shortest_path(
                graph=graph,
                start=lane_change_probe_edge.source,
                goal=lane_change_probe_edge.target,
            )
        )

        def lane_change_probe_heuristic(
            node,
        ):
            return node_distance_m(
                graph,
                node,
                lane_change_probe_edge.target,
            )

        astar_lane_change_probe = (
            a_star_shortest_path(
                graph=graph,
                start=lane_change_probe_edge.source,
                goal=lane_change_probe_edge.target,
                heuristic=(
                    lane_change_probe_heuristic
                ),
            )
        )

        if not math.isclose(
            astar_lane_change_probe.total_cost_m,
            lane_change_probe.total_cost_m,
            rel_tol=1e-9,
            abs_tol=1e-6,
        ):
            raise RuntimeError(
                "A* and Dijkstra disagree on "
                "lane-change probe cost."
            )

        astar_probe_lane_changes = sum(
            1
            for edge_key
            in astar_lane_change_probe.edge_path
            if graph.edges[
                edge_key
            ].transition_type
            != RoutingEdgeType.LANE_FOLLOW
        )

        if astar_probe_lane_changes < 1:
            raise RuntimeError(
                "A* lane-change probe did not use "
                "a lateral routing transition."
            )

        probe_lane_changes = sum(
            1
            for edge_key
            in lane_change_probe.edge_path
            if graph.edges[
                edge_key
            ].transition_type
            != RoutingEdgeType.LANE_FOLLOW
        )

        probe_summed_cost_m = sum(
            graph.edges[
                edge_key
            ].cost_m
            for edge_key
            in lane_change_probe.edge_path
        )

        if probe_lane_changes < 1:
            raise RuntimeError(
                "Lane-change routing probe reached the "
                "adjacent lane without using any lateral "
                "routing transition."
            )

        if not math.isclose(
            lane_change_probe.total_cost_m,
            probe_summed_cost_m,
            rel_tol=1e-9,
            abs_tol=1e-6,
        ):
            raise RuntimeError(
                "Lane-change routing probe cost does not "
                "equal its edge-cost sum."
            )

        logger.info(
            f"map={map_name}"
        )

        logger.info(
            "topology_graph "
            f"nodes={len(topology_graph.nodes)} "
            f"edges={len(topology_graph.edges)}"
        )

        logger.info(
            "routing_graph "
            f"nodes={len(graph.nodes)} "
            f"edges={len(graph.edges)}"
        )

        logger.info(
            "routing_edges "
            f"{dict(transition_counts)}"
        )

        logger.info(
            "build "
            f"lane_follow_added="
            f"{build_result.lane_follow_edges_added} "
            f"lane_follow_duplicates="
            f"{build_result.lane_follow_duplicates_skipped} "
            f"lane_change_permissions="
            f"{build_result.lane_change_permissions_seen} "
            f"lane_change_added="
            f"{build_result.lane_change_edges_added} "
            f"lane_change_duplicates="
            f"{build_result.lane_change_duplicates_skipped} "
            f"lane_change_target_misses="
            f"{build_result.lane_change_target_misses}"
        )

        logger.info(
            "weak_connectivity "
            f"components={len(weak_components)} "
            f"largest_sizes={weak_sizes[:10]}"
        )

        logger.info(
            "strong_connectivity "
            f"components={len(strong_components)} "
            f"largest_sizes={strong_sizes[:10]}"
        )

        logger.info(
            "reachability "
            f"start_reaches="
            f"{len(reachable)}/"
            f"{len(graph.nodes)}"
        )

        logger.info(
            f"lane_change_penalty_m="
            f"{lane_change_penalty_m:.3f}"
        )

        logger.info(
            f"start={route.start}"
        )

        logger.info(
            f"goal={route.goal}"
        )

        logger.info(
            "dijkstra "
            f"cost_m={route.total_cost_m:.3f} "
            f"nodes={len(route.node_path)} "
            f"edges={len(route.edge_path)} "
            f"lane_changes={lane_changes} "
            f"settled={route.settled_nodes}"
        )

        logger.info(
            "route_validation "
            f"summed_cost_m={summed_cost_m:.3f} "
            "continuous=true"
        )

        logger.info(
            "heuristic_consistency "
            f"edges={len(graph.edges)} "
            "violations=0 "
            f"min_cost_minus_displacement_m="
            f"{min_cost_minus_displacement_m:.6f}"
        )

        logger.info(
            "a_star "
            f"cost_m={astar_route.total_cost_m:.3f} "
            f"nodes={len(astar_route.node_path)} "
            f"edges={len(astar_route.edge_path)} "
            f"lane_changes={astar_lane_changes} "
            f"settled={astar_route.settled_nodes}"
        )

        settled_reduction = (
            route.settled_nodes
            - astar_route.settled_nodes
        )

        settled_reduction_percent = (
            100.0
            * settled_reduction
            / route.settled_nodes
        )

        logger.info(
            "search_comparison "
            "cost_equal=true "
            f"same_edge_path="
            f"{astar_route.edge_path == route.edge_path} "
            f"dijkstra_settled={route.settled_nodes} "
            f"astar_settled={astar_route.settled_nodes} "
            f"settled_reduction={settled_reduction} "
            f"settled_reduction_percent="
            f"{settled_reduction_percent:.2f}"
        )

        logger.info(
            "a_star_lane_change_probe "
            f"cost_m="
            f"{astar_lane_change_probe.total_cost_m:.3f} "
            f"edges="
            f"{len(astar_lane_change_probe.edge_path)} "
            f"lane_changes={astar_probe_lane_changes} "
            f"settled="
            f"{astar_lane_change_probe.settled_nodes}"
        )

        logger.info(
            "lane_change_probe "
            f"source={lane_change_probe.start} "
            f"goal={lane_change_probe.goal} "
            f"direct_type="
            f"{lane_change_probe_edge.transition_type.name} "
            f"direct_length_m="
            f"{lane_change_probe_edge.length_m:.3f} "
            f"direct_cost_m="
            f"{lane_change_probe_edge.cost_m:.3f} "
            f"route_cost_m="
            f"{lane_change_probe.total_cost_m:.3f} "
            f"route_edges="
            f"{len(lane_change_probe.edge_path)} "
            f"lane_changes={probe_lane_changes}"
        )

        for index, edge_key in enumerate(
            route.edge_path[:20]
        ):
            edge = graph.edges[
                edge_key
            ]

            logger.info(
                f"route_edge[{index}] "
                f"type={edge.transition_type.name} "
                f"{edge.source} -> "
                f"{edge.target} "
                f"length_m={edge.length_m:.3f} "
                f"cost_m={edge.cost_m:.3f}"
            )

        if len(route.edge_path) > 20:
            logger.info(
                "route_edge[...] "
                f"{len(route.edge_path) - 20} "
                "additional edges omitted"
            )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = None

    try:
        node = (
            RoutingGraphDiagnosticNode()
        )
    finally:
        if node is not None:
            node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()
