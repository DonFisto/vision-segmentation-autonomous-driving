#!/usr/bin/env python3

"""Publish global RoutePlan messages from live ego odometry."""

import carla

from autonomy_interfaces.msg import RoutePlan
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)

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
from global_route_planner.global_segment_ids import (
    topology_segment_id_map,
)
from global_route_planner.route_plan_builder import (
    build_route_plan_message,
    stable_map_revision,
)
from global_route_planner.routing import (
    NoPathError,
    dijkstra_shortest_path,
)
from global_route_planner.routing_graph import (
    RoutingEdgeType,
)
from global_route_planner.routing_graph_index import (
    RoutingGraphIndex,
)


class GlobalRoutePlannerNode(Node):
    """Associate live ego pose, route, and publish RoutePlan."""

    def __init__(self) -> None:
        super().__init__(
            "global_route_planner"
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
            "route_plan_topic",
            "/planning/route_plan",
        )

        self.declare_parameter(
            "frame_id",
            "carla_world",
        )

        # Temporary goal source for this milestone.
        # A dedicated goal interface comes later.
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

        route_plan_topic = str(
            self.get_parameter(
                "route_plan_topic"
            ).value
        )

        self._frame_id = str(
            self.get_parameter(
                "frame_id"
            ).value
        )

        goal_spawn_index = int(
            self.get_parameter(
                "goal_spawn_index"
            ).value
        )

        self._route_id = 0
        self._last_start_node = None

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

        self._map_revision = (
            stable_map_revision(
                map_name=(
                    self._carla_map.name
                ),
                opendrive_text=(
                    self._carla_map.to_opendrive()
                ),
            )
        )

        topology_segment_ids = (
            topology_segment_id_map(
                topology_graph=topology_graph,
                map_revision=self._map_revision,
            )
        )

        self.get_logger().info(
            "global_segment_ids "
            f"unique={len(set(topology_segment_ids.values()))} "
            f"topology_edges={len(topology_graph.edges)}"
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
                "goal_spawn_index outside "
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

        route_qos = QoSProfile(
            history=(
                QoSHistoryPolicy.KEEP_LAST
            ),
            depth=1,
            reliability=(
                QoSReliabilityPolicy.RELIABLE
            ),
            durability=(
                QoSDurabilityPolicy.TRANSIENT_LOCAL
            ),
        )

        self._route_publisher = (
            self.create_publisher(
                RoutePlan,
                route_plan_topic,
                route_qos,
            )
        )

        self._odom_subscription = (
            self.create_subscription(
                Odometry,
                odom_topic,
                self._odom_callback,
                qos_profile_sensor_data,
            )
        )

        self.get_logger().info(
            f"map={self._carla_map.name} "
            f"map_revision={self._map_revision}"
        )

        self.get_logger().info(
            "routing_graph "
            f"nodes={len(self._graph.nodes)} "
            f"edges={len(self._graph.edges)}"
        )

        self.get_logger().info(
            "goal "
            f"spawn_index={goal_spawn_index} "
            f"node={self._goal_association.node_key}"
        )

        self.get_logger().info(
            f"subscribing odom={odom_topic} "
            f"publishing route_plan="
            f"{route_plan_topic}"
        )

    def _next_route_id(
        self,
    ) -> int:
        self._route_id = (
            self._route_id + 1
        ) & ((1 << 64) - 1)

        if self._route_id == 0:
            self._route_id = 1

        return self._route_id

    def _odom_callback(
        self,
        message: Odometry,
    ) -> None:
        if (
            message.header.frame_id
            != self._frame_id
        ):
            self.get_logger().error(
                "Ignoring odometry with frame "
                f"'{message.header.frame_id}'; "
                f"expected '{self._frame_id}'"
            )
            return

        position = (
            message.pose.pose.position
        )

        try:
            start = (
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
            self.get_logger().error(
                "Start association failed: "
                f"{exc}"
            )
            return

        # Avoid running Dijkstra at the odometry frequency.
        # Replan when association moves to a new sampled state.
        if (
            start.node_key
            == self._last_start_node
        ):
            return

        try:
            route = (
                dijkstra_shortest_path(
                    graph=self._graph,
                    start=start.node_key,
                    goal=(
                        self._goal_association.node_key
                    ),
                )
            )
        except NoPathError as exc:
            self.get_logger().error(
                f"Route search failed: {exc}"
            )
            return

        route_id = (
            self._next_route_id()
        )

        stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        message_out = (
            build_route_plan_message(
                graph=self._graph,
                route=route,
                start_association=start,
                goal_association=(
                    self._goal_association
                ),
                route_id=route_id,
                map_revision=(
                    self._map_revision
                ),
                stamp=stamp,
                frame_id=self._frame_id,
                max_projection_distance_m=(
                    self._max_projection_distance_m
                ),
                max_sample_s_error_m=(
                    self._max_association_s_error_m
                ),
            )
        )

        self._route_publisher.publish(
            message_out
        )

        self._last_start_node = (
            start.node_key
        )

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
            "published_route_plan "
            f"route_id={route_id} "
            f"start={start.node_key} "
            f"goal="
            f"{self._goal_association.node_key} "
            f"cost_m={route.total_cost_m:.3f} "
            f"path_poses="
            f"{len(route.node_path)} "
            f"lane_changes={lane_changes} "
            f"lane_segment_ids="
            f"{len(message_out.lane_segment_ids)} "
            f"status="
            f"{message_out.status} "
            f"confidence="
            f"{message_out.confidence:.3f}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = (
        GlobalRoutePlannerNode()
    )

    try:
        rclpy.spin(
            node
        )
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
