"""Diagnose RoutePlan and ego odometry integration."""

import rclpy

from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)

from autonomy_interfaces.msg import RoutePlan
from nav_msgs.msg import Odometry


class RouteEgoDiagnosticNode(Node):

    def __init__(self):
        super().__init__("route_ego_diagnostic")

        self.declare_parameter(
            "frame_id",
            "carla_world",
        )

        self._frame_id = str(
            self.get_parameter("frame_id").value
        )

        self._active_route = None
        self._odom_received = False

        route_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._route_subscription = self.create_subscription(
            RoutePlan,
            "/planning/route_plan",
            self._route_callback,
            route_qos,
        )

        self._odom_subscription = self.create_subscription(
            Odometry,
            "/carla/hero_odom",
            self._odom_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            "Route ego diagnostic initialized. "
            "Waiting for RoutePlan and odometry."
        )

    def _route_callback(self, message: RoutePlan):

        if message.header.frame_id != self._frame_id:
            self.get_logger().error(
                "RoutePlan frame mismatch: "
                f"received={message.header.frame_id}, "
                f"expected={self._frame_id}"
            )
            self._active_route = None
            return

        if (
            message.coarse_reference_path.header.frame_id
            != self._frame_id
        ):
            self.get_logger().error(
                "RoutePlan path frame mismatch."
            )
            self._active_route = None
            return

        if message.status != RoutePlan.STATUS_VALID:
            self.get_logger().warn(
                "RoutePlan not valid: "
                f"route_id={message.route_id}, "
                f"status={message.status}"
            )
            self._active_route = None
            return

        pose_count = len(
            message.coarse_reference_path.poses
        )

        if pose_count < 2:
            self.get_logger().warn(
                "RoutePlan has insufficient geometry: "
                f"poses={pose_count}"
            )
            self._active_route = None
            return

        self._active_route = message

        self.get_logger().info(
            "Accepted RoutePlan: "
            f"route_id={message.route_id}, "
            f"map_revision={message.map_revision}, "
            f"poses={pose_count}"
        )

    def _odom_callback(self, message: Odometry):

        if message.header.frame_id != self._frame_id:
            self.get_logger().error(
                "Odometry frame mismatch: "
                f"received={message.header.frame_id}, "
                f"expected={self._frame_id}"
            )
            return

        if not self._odom_received:
            self.get_logger().info(
                "First valid odometry received: "
                f"x={message.pose.pose.position.x:.3f}, "
                f"y={message.pose.pose.position.y:.3f}"
            )

            self._odom_received = True


def main(args=None):

    rclpy.init(args=args)

    node = RouteEgoDiagnosticNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
