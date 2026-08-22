#!/usr/bin/env python3

"""Plan once from live CARLA odometry to a configured map goal."""

import math
import time

import carla
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

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
    dijkstra_shortest_path,
)
from global_route_planner.routing_graph import (
    RoutingEdgeType,
)
from global_route_planner.routing_graph_index import (
    RoutingGraphIndex,
)


class LiveRouteDiagnosticNode(Node):
    """One-shot route computation from live hero odometry."""

    def __init__(self) -> None:
        super().__init__(
            "global_route_live_diagnostic"
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

        self.declare_parameter(
            "odom_topic",
            "/carla/hero_odom",
        )

        self.declare_parameter(
            "expected_frame_id",
            "carla_world",
        )

        # Temporary diagnostic goal source.
        self.declare_parameter(
            "goal_spawn_index",
            44,
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

        self._max_projection_distance_m = float(
            self.get_parameter(
                "max_projection_distance_m"
            ).value
        )

        self._max_association_s_error_m = float(
            self.get_parameter(
                "max_association_s_error_m"
            ).value
        )

        odom_topic = str(
            self.get_parameter(
                "odom_topic"
            ).value
        )

        self._expected_frame_id = str(
            self.get_parameter(
                "expected_frame_id"
            ).value
        )

        goal_spawn_index = int(
            self.get_parameter(
                "goal_spawn_index"
            ).value
        )

        self.done = False
        self.failure_reason = None

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

        self._carla_map = (
            world.get_map()
        )

        topology_graph = (
            build_topology_graph(
                self._carla_map,
                sampling_resolution_m=(
                    sampling_resolution_m
                ),
            )
        )

        routing_result = (
            build_routing_graph(
                carla_map=self._carla_map,
                topology_graph=topology_graph,
                lane_change_penalty_m=(
                    lane_change_penalty_m
                ),
                max_lane_change_s_error_m=(
                    max_lane_change_s_error_m
                ),
            )
        )

        self._graph = (
            routing_result.graph
        )

        self._routing_index = (
            RoutingGraphIndex(
                self._graph
            )
        )

        spawn_points = (
            self._carla_map.get_spawn_points()
        )

        if (
            goal_spawn_index < 0
            or goal_spawn_index
            >= len(spawn_points)
        ):
            raise ValueError(
                "goal_spawn_index is outside "
                f"[0, {len(spawn_points) - 1}]: "
                f"{goal_spawn_index}"
            )

        self._goal_spawn_index = (
            goal_spawn_index
        )

        goal_location = (
            spawn_points[
                goal_spawn_index
            ].location
        )

        self._goal_association = (
            associate_carla_world_position(
                carla_map=self._carla_map,
                routing_graph=self._graph,
                routing_index=self._routing_index,
                x=goal_location.x,
                y=goal_location.y,
                z=goal_location.z,
                max_projection_distance_m=(
                    self._max_projection_distance_m
                ),
                max_sample_s_error_m=(
                    self._max_association_s_error_m
                ),
            )
        )

        self.get_logger().info(
            f"map={self._carla_map.name}"
        )

        self.get_logger().info(
            "routing_graph "
            f"nodes={len(self._graph.nodes)} "
            f"edges={len(self._graph.edges)}"
        )

        self.get_logger().info(
            "goal_association "
            f"spawn_index={goal_spawn_index} "
            f"node={self._goal_association.node_key} "
            f"projection_m="
            f"{self._goal_association.projection_distance_m:.3f} "
            f"s_error_m="
            f"{self._goal_association.sample_s_error_m:.3f}"
        )

        self.get_logger().info(
            f"waiting_for_odometry "
            f"topic={odom_topic} "
            f"expected_frame={self._expected_frame_id}"
        )

        self._subscription = (
            self.create_subscription(
                Odometry,
                odom_topic,
                self._odom_callback,
                qos_profile_sensor_data,
            )
        )

    def _fail(
        self,
        reason: str,
    ) -> None:
        self.failure_reason = (
            reason
        )

        self.done = True

        self.get_logger().error(
            reason
        )

    def _odom_callback(
        self,
        message: Odometry,
    ) -> None:
        if self.done:
            return

        frame_id = (
            message.header.frame_id
        )

        if (
            self._expected_frame_id
            and frame_id
            != self._expected_frame_id
        ):
            self._fail(
                "Odometry frame mismatch: "
                f"received '{frame_id}', "
                f"expected "
                f"'{self._expected_frame_id}'"
            )
            return

        position = (
            message.pose.pose.position
        )

        try:
            start_association = (
                associate_carla_world_position(
                    carla_map=self._carla_map,
                    routing_graph=self._graph,
                    routing_index=self._routing_index,
                    x=position.x,
                    y=position.y,
                    z=position.z,
                    max_projection_distance_m=(
                        self._max_projection_distance_m
                    ),
                    max_sample_s_error_m=(
                        self._max_association_s_error_m
                    ),
                )
            )
        except RouteAssociationError as exc:
            self._fail(
                "Live start association failed: "
                f"{exc}"
            )
            return

        goal = (
            self._goal_association
        )

        route = (
            dijkstra_shortest_path(
                graph=self._graph,
                start=start_association.node_key,
                goal=goal.node_key,
            )
        )

        summed_cost_m = sum(
            self._graph.edges[
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
            self._fail(
                "Live route cost validation failed."
            )
            return

        for index, edge_key in enumerate(
            route.edge_path
        ):
            edge = self._graph.edges[
                edge_key
            ]

            if (
                edge.source
                != route.node_path[index]
                or edge.target
                != route.node_path[
                    index + 1
                ]
            ):
                self._fail(
                    "Live route continuity validation failed."
                )
                return

        lane_changes = sum(
            1
            for edge_key
            in route.edge_path
            if self._graph.edges[
                edge_key
            ].transition_type
            != RoutingEdgeType.LANE_FOLLOW
        )

        self.get_logger().info(
            "live_odometry "
            f"frame={frame_id} "
            f"x={position.x:.3f} "
            f"y={position.y:.3f} "
            f"z={position.z:.3f}"
        )

        self.get_logger().info(
            "start_association "
            f"node={start_association.node_key} "
            f"road={start_association.road_id} "
            f"section={start_association.section_id} "
            f"lane={start_association.lane_id} "
            f"s={start_association.waypoint_s_m:.3f} "
            f"projection_m="
            f"{start_association.projection_distance_m:.3f} "
            f"sample_s_error_m="
            f"{start_association.sample_s_error_m:.3f} "
            f"sample_spatial_error_m="
            f"{start_association.sample_distance_m:.3f}"
        )

        self.get_logger().info(
            "goal_association "
            f"spawn_index={self._goal_spawn_index} "
            f"node={goal.node_key} "
            f"road={goal.road_id} "
            f"section={goal.section_id} "
            f"lane={goal.lane_id} "
            f"s={goal.waypoint_s_m:.3f}"
        )

        self.get_logger().info(
            "live_route "
            f"cost_m={route.total_cost_m:.3f} "
            f"nodes={len(route.node_path)} "
            f"edges={len(route.edge_path)} "
            f"lane_changes={lane_changes} "
            f"settled={route.settled_nodes} "
            "continuous=true"
        )

        for index, edge_key in enumerate(
            route.edge_path[:10]
        ):
            edge = self._graph.edges[
                edge_key
            ]

            self.get_logger().info(
                f"route_edge[{index}] "
                f"type={edge.transition_type.name} "
                f"{edge.source} -> "
                f"{edge.target} "
                f"length_m={edge.length_m:.3f} "
                f"cost_m={edge.cost_m:.3f}"
            )

        if len(route.edge_path) > 10:
            self.get_logger().info(
                "route_edge[...] "
                f"{len(route.edge_path) - 10} "
                "additional edges omitted"
            )

        self.done = True


def main(args=None) -> None:
    rclpy.init(args=args)

    node = (
        LiveRouteDiagnosticNode()
    )

    wait_timeout_s = 15.0
    start_time = time.monotonic()

    try:
        while (
            rclpy.ok()
            and not node.done
        ):
            rclpy.spin_once(
                node,
                timeout_sec=0.1,
            )

            if (
                time.monotonic()
                - start_time
                > wait_timeout_s
            ):
                raise RuntimeError(
                    "Timed out waiting for "
                    "live hero odometry."
                )

        if node.failure_reason:
            raise RuntimeError(
                node.failure_reason
            )

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
