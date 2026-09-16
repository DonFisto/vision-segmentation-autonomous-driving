#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from nav_msgs.msg import Path
from autonomy_interfaces.msg import RoutePlan


class RoutePlanVisualizerNode(Node):
    """Expose RoutePlan coarse geometry as a directly visualizable Path."""

    def __init__(self) -> None:
        super().__init__("route_plan_visualizer")

        route_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._path_publisher = self.create_publisher(
            Path,
            "/planning/route_path",
            route_qos,
        )

        self._route_subscription = self.create_subscription(
            RoutePlan,
            "/planning/route_plan",
            self._route_callback,
            route_qos,
        )

        self.get_logger().info(
            "RoutePlan visualizer ready: "
            "/planning/route_plan -> /planning/route_path"
        )

    def _route_callback(self, msg: RoutePlan) -> None:
        # Publish an empty path for invalid routes so visualization clients
        # clear any previously displayed valid route.
        if msg.status != RoutePlan.STATUS_VALID:
            path = Path()
            path.header = msg.header

            self._path_publisher.publish(path)

            self.get_logger().warn(
                f"RoutePlan {msg.route_id} is not valid "
                f"(status={msg.status}); published empty route path."
            )
            return

        path = msg.coarse_reference_path

        # RoutePlan should already carry carla_world here. Preserve the
        # planner's frame rather than introducing a new visualization frame.
        if not path.header.frame_id:
            path.header = msg.header

        self._path_publisher.publish(path)

        self.get_logger().info(
            f"Published route {msg.route_id}: "
            f"{len(path.poses)} poses, "
            f"frame='{path.header.frame_id}'"
        )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = RoutePlanVisualizerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
