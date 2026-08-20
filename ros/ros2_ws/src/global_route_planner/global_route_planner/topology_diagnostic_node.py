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

        self.declare_parameter(
            "sampling_resolution_m",
            2.0,
        )

        host = str(
            self.get_parameter("host").value
        )

        port = int(
            self.get_parameter("port").value
        )

        timeout_s = float(
            self.get_parameter("timeout_s").value
        )

        sampling_resolution_m = float(
            self.get_parameter(
                "sampling_resolution_m"
            ).value
        )

        self.get_logger().info(
            f"Connecting to CARLA at {host}:{port}"
        )

        client = carla.Client(host, port)
        client.set_timeout(timeout_s)

        world = client.get_world()
        carla_map = world.get_map()

        graph = build_topology_graph(
            carla_map,
            sampling_resolution_m=(
                sampling_resolution_m
            ),
        )

        self._report(
            map_name=carla_map.name,
            sampling_resolution_m=(
                sampling_resolution_m
            ),
            graph=graph,
        )

    def _report(
        self,
        map_name,
        sampling_resolution_m,
        graph,
    ) -> None:
        logger = self.get_logger()

        logger.info(
            f"map={map_name}"
        )

        logger.info(
            f"sampling_resolution_m="
            f"{sampling_resolution_m:.3f}"
        )

        logger.info(
            "topology "
            f"nodes={len(graph.nodes)} "
            f"edges={len(graph.edges)}"
        )

        logger.info(
            "out_degree_distribution="
            + self._format_distribution(
                graph.out_degree_distribution()
            )
        )

        logger.info(
            "in_degree_distribution="
            + self._format_distribution(
                graph.in_degree_distribution()
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

        edges = list(graph.edges.values())

        lengths = [
            edge.length_m
            for edge in edges
        ]

        chords = [
            edge.endpoint_distance_m
            for edge in edges
        ]

        sample_counts = [
            len(edge.geometry)
            for edge in edges
        ]

        ratios = [
            (
                edge.length_m
                / edge.endpoint_distance_m
            )
            if edge.endpoint_distance_m > 1e-6
            else 1.0
            for edge in edges
        ]

        logger.info(
            "edge_length_m "
            f"min={min(lengths):.3f} "
            f"mean={statistics.mean(lengths):.3f} "
            f"max={max(lengths):.3f}"
        )

        logger.info(
            "endpoint_chord_m "
            f"min={min(chords):.3f} "
            f"mean={statistics.mean(chords):.3f} "
            f"max={max(chords):.3f}"
        )

        logger.info(
            "geometry_samples "
            f"min={min(sample_counts)} "
            f"mean={statistics.mean(sample_counts):.2f} "
            f"max={max(sample_counts)}"
        )

        logger.info(
            "arc_chord_ratio "
            f"min={min(ratios):.4f} "
            f"mean={statistics.mean(ratios):.4f} "
            f"max={max(ratios):.4f}"
        )

        junction_edges = sum(
            1
            for edge in edges
            if edge.junction_involved
        )

        logger.info(
            f"junction_involved_edges="
            f"{junction_edges}"
        )

        for node_key in branching[:5]:
            targets = []

            for edge_key in graph.outgoing[node_key]:
                edge = graph.edges[edge_key]

                targets.append(
                    f"{edge.target}"
                    f"(L={edge.length_m:.1f}m)"
                )

            logger.info(
                f"branch {node_key} -> "
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
