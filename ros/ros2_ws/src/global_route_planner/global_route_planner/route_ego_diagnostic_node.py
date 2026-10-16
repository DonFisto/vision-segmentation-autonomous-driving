"""Diagnose live RoutePlan and ego odometry integration."""

import math
import time

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

from global_route_planner.route_geometry import (
    project_point_to_polyline,
)


class RouteEgoDiagnosticNode(Node):
    """Project the live ego position onto the active global route."""

    def __init__(self):
        super().__init__("route_ego_diagnostic")

        self.declare_parameter("frame_id", "carla_world")
        self.declare_parameter("max_projection_distance_m", 5.0)
        self.declare_parameter("max_odom_age_s", 2.0)
        self.declare_parameter("report_period_s", 1.0)

        self._frame_id = str(
            self.get_parameter("frame_id").value
        )

        self._max_projection_distance_m = float(
            self.get_parameter("max_projection_distance_m").value
        )

        self._max_odom_age_s = float(
            self.get_parameter("max_odom_age_s").value
        )

        self._report_period_s = float(
            self.get_parameter("report_period_s").value
        )

        for name, value in (
            ("max_projection_distance_m", self._max_projection_distance_m),
            ("max_odom_age_s", self._max_odom_age_s),
            ("report_period_s", self._report_period_s),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")

        self._active_route = None
        self._route_points = None

        self._latest_odom = None
        self._latest_odom_time = None

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

        self._projection_timer = self.create_timer(
            self._report_period_s,
            self._report_projection,
        )

        self.get_logger().info(
            "Route ego diagnostic initialized. "
            "Waiting for RoutePlan and odometry."
        )

    def _route_callback(self, message: RoutePlan):
        """Validate and cache the active route geometry."""

        # Never retain an older route after a new route message.
        self._active_route = None
        self._route_points = None

        if message.header.frame_id != self._frame_id:
            self.get_logger().error(
                "RoutePlan frame mismatch: "
                f"received={message.header.frame_id}, "
                f"expected={self._frame_id}"
            )
            return

        path = message.coarse_reference_path

        if path.header.frame_id != self._frame_id:
            self.get_logger().error(
                "RoutePlan path frame mismatch: "
                f"received={path.header.frame_id}, "
                f"expected={self._frame_id}"
            )
            return

        if message.status != RoutePlan.STATUS_VALID:
            self.get_logger().warn(
                "RoutePlan not valid: "
                f"route_id={message.route_id}, "
                f"status={message.status}"
            )
            return

        pose_count = len(path.poses)

        if pose_count < 2:
            self.get_logger().warn(
                "RoutePlan has insufficient geometry: "
                f"poses={pose_count}"
            )
            return

        # Validate the individual pose frames if populated.
        for pose in path.poses:
            pose_frame = pose.header.frame_id

            if pose_frame and pose_frame != self._frame_id:
                self.get_logger().error(
                    "RoutePlan pose frame mismatch: "
                    f"received={pose_frame}, "
                    f"expected={self._frame_id}"
                )
                return

        points = [
            (
                float(pose.pose.position.x),
                float(pose.pose.position.y),
            )
            for pose in path.poses
        ]

        if not all(
            math.isfinite(x) and math.isfinite(y)
            for x, y in points
        ):
            self.get_logger().error(
                "Rejecting RoutePlan: non-finite coordinates"
            )
            return

        has_valid_segment = any(
            math.hypot(
                points[i + 1][0] - points[i][0],
                points[i + 1][1] - points[i][1],
            ) > 0.0
            for i in range(len(points) - 1)
        )

        if not has_valid_segment:
            self.get_logger().error(
                "Rejecting RoutePlan: degenerate geometry"
            )
            return

        self._route_points = points
        self._active_route = message

        self.get_logger().info(
            "Accepted RoutePlan: "
            f"route_id={message.route_id}, "
            f"map_revision={message.map_revision}, "
            f"poses={pose_count}"
        )

    def _odom_callback(self, message: Odometry):
        """Cache valid ego odometry and its arrival time."""

        if message.header.frame_id != self._frame_id:
            self._latest_odom = None
            self._latest_odom_time = None

            self.get_logger().error(
                "Odometry frame mismatch: "
                f"received={message.header.frame_id}, "
                f"expected={self._frame_id}"
            )
            return

        position = message.pose.pose.position

        if not (
            math.isfinite(position.x)
            and math.isfinite(position.y)
        ):
            self._latest_odom = None
            self._latest_odom_time = None

            self.get_logger().error(
                "Rejecting odometry: non-finite ego coordinates"
            )
            return

        self._latest_odom = message
        self._latest_odom_time = time.monotonic()

        if not self._odom_received:
            self.get_logger().info(
                "First valid odometry received: "
                f"x={position.x:.3f}, "
                f"y={position.y:.3f}"
            )

            self._odom_received = True

    def _report_projection(self):
        """Report ego projection onto the active route at 1 Hz."""

        if (
            self._active_route is None
            or self._route_points is None
            or self._latest_odom is None
        ):
            return

        if (
            self._latest_odom_time is None
            or time.monotonic() - self._latest_odom_time
            > self._max_odom_age_s
        ):
            self.get_logger().warn(
                "Ego projection unavailable: stale odometry"
            )
            return

        position = self._latest_odom.pose.pose.position

        try:
            projection = project_point_to_polyline(
                self._route_points,
                float(position.x),
                float(position.y),
            )
        except (ValueError, OverflowError) as exc:
            self.get_logger().error(
                f"Route projection failed: {exc}"
            )
            return

        values = (
            projection.s_m,
            projection.d_m,
            projection.distance_m,
            projection.heading_rad,
        )

        if not all(math.isfinite(v) for v in values):
            self.get_logger().error(
                "Route projection failed: non-finite result"
            )
            return

        within_distance_gate = (
            projection.distance_m
            <= self._max_projection_distance_m
        )

        self.get_logger().info(
            "ego_route_projection "
            f"route_id={self._active_route.route_id} "
            f"s_m={projection.s_m:.3f} "
            f"d_m={projection.d_m:.3f} "
            f"distance_m={projection.distance_m:.3f} "
            f"segment={projection.segment_index} "
            f"distance_gate="
            f"{'PASS' if within_distance_gate else 'FAIL'}"
        )


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
