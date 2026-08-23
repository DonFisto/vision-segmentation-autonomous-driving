#!/usr/bin/env python3

"""Validate CARLA-world pose association with the routing graph."""

import math
from statistics import mean

import carla
import rclpy
from rclpy.node import Node

from global_route_planner.carla_route_association import (
    RouteAssociationError,
    associate_carla_world_position,
)
from global_route_planner.carla_routing_graph_adapter import (
    build_routing_graph,
)
from global_route_planner.carla_topology_adapter import (
    build_topology_graph,
)
from global_route_planner.routing import (
    a_star_shortest_path,
    dijkstra_shortest_path,
)
from global_route_planner.routing_graph import (
    RoutingEdgeType,
)
from global_route_planner.routing_graph_index import (
    RoutingGraphIndex,
)


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


class RouteAssociationDiagnosticNode(Node):
    def __init__(self) -> None:
        super().__init__(
            "global_route_association_diagnostic"
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

        self.declare_parameter(
            "max_projection_distance_m",
            5.0,
        )

        self.declare_parameter(
            "max_association_s_error_m",
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

        max_projection_distance_m = float(
            self.get_parameter(
                "max_projection_distance_m"
            ).value
        )

        max_association_s_error_m = float(
            self.get_parameter(
                "max_association_s_error_m"
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

        routing_result = (
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

        graph = routing_result.graph

        routing_index = (
            RoutingGraphIndex(
                graph
            )
        )

        spawn_points = (
            carla_map.get_spawn_points()
        )

        if len(spawn_points) < 2:
            raise RuntimeError(
                "CARLA map has fewer than two spawn points."
            )

        associations = []
        failures = []

        for index, transform in enumerate(
            spawn_points
        ):
            location = (
                transform.location
            )

            try:
                association = (
                    associate_carla_world_position(
                        carla_map=carla_map,
                        routing_graph=graph,
                        routing_index=routing_index,
                        x=location.x,
                        y=location.y,
                        z=location.z,
                        max_projection_distance_m=(
                            max_projection_distance_m
                        ),
                        max_sample_s_error_m=(
                            max_association_s_error_m
                        ),
                    )
                )
            except RouteAssociationError as exc:
                failures.append(
                    (
                        index,
                        str(exc),
                    )
                )
                continue

            associations.append(
                (
                    index,
                    association,
                )
            )

        if len(associations) < 2:
            raise RuntimeError(
                "Fewer than two spawn points "
                "could be associated."
            )

        projection_errors = [
            association.projection_distance_m
            for _, association
            in associations
        ]

        s_errors = [
            association.sample_s_error_m
            for _, association
            in associations
        ]

        spatial_sample_errors = [
            association.sample_distance_m
            for _, association
            in associations
        ]

        start_spawn_index, start = (
            associations[0]
        )

        candidates = [
            (
                spawn_index,
                association,
            )
            for (
                spawn_index,
                association,
            )
            in associations
            if association.node_key
            != start.node_key
        ]

        if not candidates:
            raise RuntimeError(
                "All associated spawn points "
                "collapsed to one routing node."
            )

        (
            goal_spawn_index,
            goal,
        ) = max(
            candidates,
            key=lambda candidate: (
                node_distance_m(
                    graph,
                    start.node_key,
                    candidate[1].node_key,
                ),
                candidate[1].node_key,
            ),
        )

        route = (
            dijkstra_shortest_path(
                graph=graph,
                start=start.node_key,
                goal=goal.node_key,
            )
        )

        def goal_heuristic(
            node,
        ):
            return node_distance_m(
                graph,
                node,
                goal.node_key,
            )

        astar_route = (
            a_star_shortest_path(
                graph=graph,
                start=start.node_key,
                goal=goal.node_key,
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
                "Associated-route A* and Dijkstra "
                "costs disagree: "
                f"dijkstra={route.total_cost_m:.6f} "
                f"astar={astar_route.total_cost_m:.6f}"
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
                "Associated A* route cost does not "
                "equal its edge-cost sum."
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
                    "Associated A* route "
                    "continuity failed."
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
            for edge_key
            in route.edge_path
        )

        if not math.isclose(
            route.total_cost_m,
            summed_cost_m,
            rel_tol=1e-9,
            abs_tol=1e-6,
        ):
            raise RuntimeError(
                "Association route cost validation failed."
            )

        lane_changes = sum(
            1
            for edge_key
            in route.edge_path
            if graph.edges[
                edge_key
            ].transition_type
            != RoutingEdgeType.LANE_FOLLOW
        )

        # Deliberately invalid query: same x/y as a valid
        # spawn location but 100 m above the road. CARLA can
        # project it, but our residual threshold must reject it.
        first_location = (
            spawn_points[
                start_spawn_index
            ].location
        )

        offroad_rejected = False

        try:
            associate_carla_world_position(
                carla_map=carla_map,
                routing_graph=graph,
                routing_index=routing_index,
                x=first_location.x,
                y=first_location.y,
                z=first_location.z + 100.0,
                max_projection_distance_m=(
                    max_projection_distance_m
                ),
                max_sample_s_error_m=(
                    max_association_s_error_m
                ),
            )
        except RouteAssociationError:
            offroad_rejected = True

        if not offroad_rejected:
            raise RuntimeError(
                "Far-away position was incorrectly accepted."
            )

        logger = self.get_logger()

        logger.info(
            f"map={carla_map.name}"
        )

        logger.info(
            "routing_graph "
            f"nodes={len(graph.nodes)} "
            f"edges={len(graph.edges)}"
        )

        logger.info(
            "spawn_association "
            f"total={len(spawn_points)} "
            f"success={len(associations)} "
            f"failures={len(failures)}"
        )

        logger.info(
            "projection_distance_m "
            f"min={min(projection_errors):.3f} "
            f"mean={mean(projection_errors):.3f} "
            f"max={max(projection_errors):.3f}"
        )

        logger.info(
            "sample_s_error_m "
            f"min={min(s_errors):.3f} "
            f"mean={mean(s_errors):.3f} "
            f"max={max(s_errors):.3f}"
        )

        logger.info(
            "sample_spatial_error_m "
            f"min={min(spatial_sample_errors):.3f} "
            f"mean={mean(spatial_sample_errors):.3f} "
            f"max={max(spatial_sample_errors):.3f}"
        )

        for (
            spawn_index,
            message,
        ) in failures[:10]:
            logger.warning(
                f"association_failure "
                f"spawn_index={spawn_index} "
                f"reason={message}"
            )

        logger.info(
            "start_association "
            f"spawn_index={start_spawn_index} "
            f"node={start.node_key} "
            f"projection_m="
            f"{start.projection_distance_m:.3f} "
            f"s_error_m="
            f"{start.sample_s_error_m:.3f}"
        )

        logger.info(
            "goal_association "
            f"spawn_index={goal_spawn_index} "
            f"node={goal.node_key} "
            f"projection_m="
            f"{goal.projection_distance_m:.3f} "
            f"s_error_m="
            f"{goal.sample_s_error_m:.3f}"
        )

        logger.info(
            "associated_route "
            f"cost_m={route.total_cost_m:.3f} "
            f"nodes={len(route.node_path)} "
            f"edges={len(route.edge_path)} "
            f"lane_changes={lane_changes} "
            f"settled={route.settled_nodes} "
            "continuous_cost=true"
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
            "associated_route_a_star "
            f"cost_m={astar_route.total_cost_m:.3f} "
            f"nodes={len(astar_route.node_path)} "
            f"edges={len(astar_route.edge_path)} "
            f"lane_changes={astar_lane_changes} "
            f"settled={astar_route.settled_nodes} "
            "continuous_cost=true"
        )

        logger.info(
            "associated_search_comparison "
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
            "offroad_rejection=true"
        )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = None

    try:
        node = (
            RouteAssociationDiagnosticNode()
        )
    finally:
        if node is not None:
            node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()
