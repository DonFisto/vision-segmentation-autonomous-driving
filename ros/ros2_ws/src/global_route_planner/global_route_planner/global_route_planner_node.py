#!/usr/bin/env python3

"""Publish global RoutePlan messages from live ego odometry."""

import carla

from autonomy_interfaces.msg import RoutePlan
from geometry_msgs.msg import PoseStamped
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

        self.declare_parameter(
            "goal_topic",
            "/planning/goal",
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

        goal_topic = str(
            self.get_parameter(
                "goal_topic"
            ).value
        )

        self._route_id = 0

        self._latest_start_association = None
        self._last_planned_start_node = None
        self._goal_association = None

        self._waiting_for_goal_logged = False

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

        self._goal_subscription = (
            self.create_subscription(
                PoseStamped,
                goal_topic,
                self._goal_callback,
                10,
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
            "goal_input "
            f"topic={goal_topic} "
            "type=geometry_msgs/PoseStamped "
            "orientation_constraint=false"
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

    def _associate_position(
        self,
        position,
    ):
        """Associate one carla_world position with routing state."""

        return associate_carla_world_position(
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

    def _goal_callback(
        self,
        message: PoseStamped,
    ) -> None:
        """Accept a new global mission goal."""

        if (
            message.header.frame_id
            != self._frame_id
        ):
            self.get_logger().error(
                "Rejecting goal with frame "
                f"'{message.header.frame_id}'; "
                f"expected '{self._frame_id}'. "
                "Previous valid goal remains active."
            )
            return

        try:
            goal = self._associate_position(
                message.pose.position
            )
        except RouteAssociationError as exc:
            self.get_logger().error(
                "Rejecting goal: "
                f"{exc}. "
                "Previous valid goal remains active."
            )
            return

        self._goal_association = goal

        self._waiting_for_goal_logged = False

        self.get_logger().info(
            "accepted_goal "
            f"requested_x="
            f"{message.pose.position.x:.3f} "
            f"requested_y="
            f"{message.pose.position.y:.3f} "
            f"requested_z="
            f"{message.pose.position.z:.3f} "
            f"node={goal.node_key} "
            f"projection_m="
            f"{goal.projection_distance_m:.3f} "
            f"sample_s_error_m="
            f"{goal.sample_s_error_m:.3f} "
            "orientation_constraint=false"
        )

        if (
            self._latest_start_association
            is None
        ):
            self.get_logger().info(
                "Goal accepted; waiting for "
                "ego odometry before routing."
            )
            return

        # A newly accepted goal always triggers a route,
        # even if the ego remains on the same sampled node.
        self._publish_route(
            start=(
                self._latest_start_association
            ),
            reason="new_goal",
        )

    def _publish_route(
        self,
        start,
        reason: str,
    ) -> None:
        """Search and publish one route from start to current goal."""

        goal = self._goal_association

        if goal is None:
            return

        try:
            route = (
                dijkstra_shortest_path(
                    graph=self._graph,
                    start=start.node_key,
                    goal=goal.node_key,
                )
            )
        except NoPathError as exc:
            self.get_logger().error(
                "Route search failed "
                f"reason={reason}: {exc}"
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
                goal_association=goal,
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

        self._last_planned_start_node = (
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
            f"reason={reason} "
            f"route_id={route_id} "
            f"start={start.node_key} "
            f"goal={goal.node_key} "
            f"cost_m={route.total_cost_m:.3f} "
            f"path_poses="
            f"{len(route.node_path)} "
            f"lane_changes={lane_changes} "
            f"lane_segment_ids="
            f"{len(message_out.lane_segment_ids)} "
            f"status={message_out.status} "
            f"confidence="
            f"{message_out.confidence:.3f}"
        )

    def _odom_callback(
        self,
        message: Odometry,
    ) -> None:
        """Update live start state and replan when it advances."""

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

        try:
            start = self._associate_position(
                message.pose.pose.position
            )
        except RouteAssociationError as exc:
            self.get_logger().error(
                "Start association failed: "
                f"{exc}"
            )
            return

        self._latest_start_association = (
            start
        )

        if self._goal_association is None:
            if not self._waiting_for_goal_logged:
                self.get_logger().info(
                    "start_ready "
                    f"node={start.node_key} "
                    "waiting_for_goal=true"
                )

                self._waiting_for_goal_logged = True

            return

        # Do not run Dijkstra at odometry frequency.
        if (
            start.node_key
            == self._last_planned_start_node
        ):
            return

        self._publish_route(
            start=start,
            reason="start_node_changed",
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
