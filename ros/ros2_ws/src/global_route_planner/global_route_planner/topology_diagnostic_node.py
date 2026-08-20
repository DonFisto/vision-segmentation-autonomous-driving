#!/usr/bin/env python3

"""One-shot ROS2 diagnostic for CARLA global topology."""

import statistics

import carla
import rclpy
from rclpy.node import Node

from global_route_planner.carla_topology_adapter import (
    build_topology_graph,
)


class TopologyDiagnosticNode(Node):
    def __init__(self) -> None:
        super().__init__(
            "global_route_topology_diagnostic"
        )

        self.declare_parameter("host", "localhost")
        self.declare_parameter("port", 2000)
        self.declare_parameter("timeout_s", 5.0)

        host = str(
            self.get_parameter("host").value
        )

        port = int(
            self.get_parameter("port").value
        )

        timeout_s = float(
            self.get_parameter("timeout_s").value
        )

        self.get_logger().info(
            f"Connecting to CARLA at {host}:{port}"
        )

        client = carla.Client(host, port)
        client.set_timeout(timeout_s)

        world = client.get_world()
        carla_map = world.get_map()

        graph = build_topology_graph(carla_map)

        self._report(
            map_name=carla_map.name,
            graph=graph,
        )

    def _report(
        self,
        map_name,
        graph,
    ) -> None:
        logger = self.get_logger()

        logger.info(
            f"map={map_name}"
        )

        logger.info(
            "topology "
            f"nodes={len(graph.nodes)} "
            f"edges={len(graph.edges)}"
        )

        out_distribution = (
            graph.out_degree_distribution()
        )

        in_distribution = (
            graph.in_degree_distribution()
        )

        logger.info(
            "out_degree_distribution="
            + self._format_distribution(
                out_distribution
            )
        )

        logger.info(
            "in_degree_distribution="
            + self._format_distribution(
                in_distribution
            )
        )

        branching = graph.branching_nodes()
        merging = graph.merging_nodes()
        sources = graph.source_nodes()
        sinks = graph.sink_nodes()

        logger.info(
            "roles "
            f"branching={len(branching)} "
            f"merging={len(merging)} "
            f"sources={len(sources)} "
            f"sinks={len(sinks)}"
        )

        endpoint_distances = [
            edge.endpoint_distance_m
            for edge in graph.edges.values()
        ]

        if endpoint_distances:
            logger.info(
                "endpoint_distance_m "
                f"min={min(endpoint_distances):.3f} "
                f"mean="
                f"{statistics.mean(endpoint_distances):.3f} "
                f"max={max(endpoint_distances):.3f} "
                "(diagnostic only; not route cost)"
            )

        junction_edges = sum(
            1
            for edge in graph.edges.values()
            if edge.junction_involved
        )

        logger.info(
            f"junction_involved_edges={junction_edges}"
        )

        for node_key in branching[:5]:
            targets = []

            for edge_key in graph.outgoing[node_key]:
                edge = graph.edges[edge_key]
                targets.append(str(edge.target))

            logger.info(
                "branch "
                f"{node_key} "
                "-> "
                + ", ".join(targets)
            )

    @staticmethod
    def _format_distribution(distribution) -> str:
        parts = []

        for degree in sorted(distribution):
            parts.append(
                f"{degree}:{distribution[degree]}"
            )

        return "{" + ", ".join(parts) + "}"


def main(args=None) -> None:
    rclpy.init(args=args)

    node = None

    try:
        node = TopologyDiagnosticNode()
    finally:
        if node is not None:
            node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()
