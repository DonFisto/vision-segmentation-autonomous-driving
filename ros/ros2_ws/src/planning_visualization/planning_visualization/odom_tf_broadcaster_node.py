#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class OdomTfBroadcasterNode(Node):
    """Broadcast the CARLA odometry parent->child transform on /tf."""

    def __init__(self) -> None:
        super().__init__("odom_tf_broadcaster")

        self._broadcaster = TransformBroadcaster(self)

        self._subscription = self.create_subscription(
            Odometry,
            "/carla/hero_odom",
            self._odom_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            "Broadcasting TF from /carla/hero_odom"
        )

    def _odom_callback(self, msg: Odometry) -> None:
        if not msg.header.frame_id:
            self.get_logger().warn(
                "Ignoring odometry with empty parent frame."
            )
            return

        if not msg.child_frame_id:
            self.get_logger().warn(
                "Ignoring odometry with empty child frame."
            )
            return

        transform = TransformStamped()

        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = msg.header.frame_id
        transform.child_frame_id = msg.child_frame_id

        transform.transform.translation.x = (
            msg.pose.pose.position.x
        )
        transform.transform.translation.y = (
            msg.pose.pose.position.y
        )
        transform.transform.translation.z = (
            msg.pose.pose.position.z
        )

        transform.transform.rotation = (
            msg.pose.pose.orientation
        )

        self._broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)

    node = OdomTfBroadcasterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
