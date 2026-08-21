#!/usr/bin/env python3

"""One-shot runtime diagnostic for Dijkstra routing."""

from collections import deque
import math

import carla
import rclpy
from rclpy.node import Node

from global_route_planner.carla_topology_adapter import (
    build_topology_graph,
)
from global_route_planner.connectivity import (
    strongly_connected_components,
    weakly_connected_components,
)
from global_route_planner.routing import (
    dijkstra_shortest_path,
)


def reachable_nodes(graph, start):
    """Unweighted directed reachability from start."""

    visited = {start}
    queue = deque([start])

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

            visited.add(target)
            queue.append(target)

    return visited


def node_distance_m(graph, a, b):
    """Euclidean distance between graph node positions."""

    na = graph.nodes[a]
    nb = graph.nodes[b]

    dx = nb.x - na.x
    dy = nb.y - na.y
    dz = nb.z - na.z

    return math.sqrt(
        dx * dx
        + dy * dy
        + dz * dz
    )


class RoutingDiagnosticNode(Node):
    def __init__(self) -> None:
        super().__init__(
            "global_route_dijkstra_diagnostic"
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

        graph = build_topology_graph(
            carla_map,
            sampling_resolution_m=(
                sampling_resolution_m
            ),
        )

        self._run_diagnostic(
            carla_map.name,
            graph,
        )

    def _run_diagnostic(
        self,
        map_name,
        graph,
    ) -> None:
        logger = self.get_logger()

        if not graph.nodes:
            raise RuntimeError(
                "Topology graph is empty."
            )

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
                for component in weak_components
            ),
            reverse=True,
        )

        strong_sizes = sorted(
            (
                len(component)
                for component in strong_components
            ),
            reverse=True,
        )

        # Deterministic semantic start node.
        start = min(
            graph.nodes.keys()
        )

        reachable = reachable_nodes(
            graph,
            start,
        )

        if len(reachable) < 2:
            raise RuntimeError(
                "Selected start has no "
                "nontrivial reachable goal."
            )

        candidates = (
            reachable - {start}
        )

        # Pick a distant reachable goal without using
        # routing cost to choose it.
        goal = max(
            candidates,
            key=lambda node: (
                node_distance_m(
                    graph,
                    start,
                    node,
                ),
                node,
            ),
        )

        result = dijkstra_shortest_path(
            graph=graph,
            start=start,
            goal=goal,
        )

        summed_cost_m = sum(
            graph.edges[
                edge_key
            ].length_m
            for edge_key in result.edge_path
        )

        if len(result.node_path) != (
            len(result.edge_path) + 1
        ):
            raise RuntimeError(
                "Node/edge route length "
                "relationship is invalid."
            )

        for index, edge_key in enumerate(
            result.edge_path
        ):
            edge = graph.edges[
                edge_key
            ]

            expected_source = (
                result.node_path[index]
            )

            expected_target = (
                result.node_path[index + 1]
            )

            if (
                edge.source
                != expected_source
                or edge.target
                != expected_target
            ):
                raise RuntimeError(
                    "Route edge continuity "
                    "validation failed."
                )

        if not math.isclose(
            result.total_cost_m,
            summed_cost_m,
            rel_tol=1e-9,
            abs_tol=1e-6,
        ):
            raise RuntimeError(
                "Dijkstra result cost does not "
                "equal the sum of route edges."
            )

        junction_edges = sum(
            1
            for edge_key in result.edge_path
            if graph.edges[
                edge_key
            ].junction_involved
        )

        logger.info(
            f"map={map_name}"
        )

        logger.info(
            "graph "
            f"nodes={len(graph.nodes)} "
            f"edges={len(graph.edges)}"
        )

        logger.info(
            "weak_connectivity "
            f"components={len(weak_components)} "
            f"sizes={weak_sizes}"
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
            f"start={result.start}"
        )

        logger.info(
            f"goal={result.goal}"
        )

        logger.info(
            "dijkstra "
            f"cost_m={result.total_cost_m:.3f} "
            f"nodes={len(result.node_path)} "
            f"edges={len(result.edge_path)} "
            f"settled={result.settled_nodes}"
        )

        logger.info(
            "route_validation "
            f"summed_cost_m={summed_cost_m:.3f} "
            f"junction_edges={junction_edges} "
            "continuous=true"
        )

        for index, edge_key in enumerate(
            result.edge_path[:12]
        ):
            edge = graph.edges[
                edge_key
            ]

            logger.info(
                f"route_edge[{index}] "
                f"{edge.source} -> "
                f"{edge.target} "
                f"length_m={edge.length_m:.3f}"
            )

        if len(result.edge_path) > 12:
            logger.info(
                "route_edge[...] "
                f"{len(result.edge_path) - 12} "
                "additional edges omitted"
            )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = None

    try:
        node = RoutingDiagnosticNode()
    finally:
        if node is not None:
            node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()
