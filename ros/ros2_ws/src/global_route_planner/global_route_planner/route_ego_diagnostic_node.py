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
    extract_route_window,
    project_point_to_polyline,
)


def quaternion_to_yaw(quaternion):
    """Extract planar yaw in radians from a normalized ROS quaternion."""

    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)

    components = (x, y, z, w)

    if not all(math.isfinite(value) for value in components):
        raise ValueError("Quaternion contains non-finite values")

    norm = math.sqrt(x*x + y*y + z*z + w*w)

    if not math.isfinite(norm) or norm < 1e-6:
        raise ValueError("Quaternion has invalid norm")

    x /= norm
    y /= norm
    z /= norm
    w /= norm

    sin_yaw = 2.0 * (w*z + x*y)
    cos_yaw = 1.0 - 2.0 * (y*y + z*z)

    return math.atan2(sin_yaw, cos_yaw)


def normalize_angle(angle_rad):
    """Normalize an angle to [-pi, pi)."""

    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


class RouteEgoDiagnosticNode(Node):
    """Project the live ego pose onto the active global route."""

    def __init__(self):
        super().__init__("route_ego_diagnostic")

        self.declare_parameter("frame_id", "carla_world")
        self.declare_parameter("max_projection_distance_m", 5.0)
        self.declare_parameter("max_heading_error_deg", 45.0)
        self.declare_parameter("max_odom_age_s", 2.0)
        self.declare_parameter("report_period_s", 1.0)
        self.declare_parameter("window_backward_m", 10.0)
        self.declare_parameter("window_forward_m", 50.0)

        self._frame_id = str(
            self.get_parameter("frame_id").value
        )

        self._max_projection_distance_m = float(
            self.get_parameter("max_projection_distance_m").value
        )

        self._max_heading_error_deg = float(
            self.get_parameter("max_heading_error_deg").value
        )

        self._max_odom_age_s = float(
            self.get_parameter("max_odom_age_s").value
        )

        self._report_period_s = float(
            self.get_parameter("report_period_s").value
        )

        self._window_backward_m = float(
            self.get_parameter("window_backward_m").value
        )

        self._window_forward_m = float(
            self.get_parameter("window_forward_m").value
        )

        for name, value in (
            ("max_projection_distance_m", self._max_projection_distance_m),
            ("max_heading_error_deg", self._max_heading_error_deg),
            ("max_odom_age_s", self._max_odom_age_s),
            ("report_period_s", self._report_period_s),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")

        for name, value in (
            ("window_backward_m", self._window_backward_m),
            ("window_forward_m", self._window_forward_m),
        ):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(
                    f"{name} must be finite and nonnegative"
                )

        if self._max_heading_error_deg > 180.0:
            raise ValueError(
                "max_heading_error_deg cannot exceed 180"
            )

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

        try:
            quaternion_to_yaw(message.pose.pose.orientation)
        except ValueError as exc:
            self._latest_odom = None
            self._latest_odom_time = None

            self.get_logger().error(
                f"Rejecting odometry: {exc}"
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
        """Report ego route coordinates and heading consistency."""

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
        orientation = self._latest_odom.pose.pose.orientation

        try:
            projection = project_point_to_polyline(
                self._route_points,
                float(position.x),
                float(position.y),
            )

            ego_yaw = quaternion_to_yaw(orientation)

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
            ego_yaw,
        )

        if not all(math.isfinite(v) for v in values):
            self.get_logger().error(
                "Route projection failed: non-finite result"
            )
            return

        heading_error_rad = normalize_angle(
            ego_yaw - projection.heading_rad
        )

        heading_error_deg = math.degrees(
            heading_error_rad
        )

        ego_yaw_deg = math.degrees(ego_yaw)
        route_heading_deg = math.degrees(
            projection.heading_rad
        )

        within_distance_gate = (
            projection.distance_m
            <= self._max_projection_distance_m
        )

        within_heading_gate = (
            abs(heading_error_deg)
            <= self._max_heading_error_deg
        )

        try:
            window = extract_route_window(
                self._route_points,
                center_s_m=projection.s_m,
                backward_m=self._window_backward_m,
                forward_m=self._window_forward_m,
            )
        except (ValueError, OverflowError) as exc:
            self.get_logger().error(
                f"Local route window failed: {exc}"
            )
            return

        window_usable = (
            window.end_s_m > window.start_s_m
            and len(window.samples) >= 2
        )

        self.get_logger().info(
            "ego_route_projection "
            f"route_id={self._active_route.route_id} "
            f"s_m={projection.s_m:.3f} "
            f"d_m={projection.d_m:.3f} "
            f"distance_m={projection.distance_m:.3f} "
            f"segment={projection.segment_index} "
            f"ego_yaw_deg={ego_yaw_deg:.2f} "
            f"route_heading_deg={route_heading_deg:.2f} "
            f"heading_error_deg={heading_error_deg:.2f} "
            f"distance_gate="
            f"{'PASS' if within_distance_gate else 'FAIL'} "
            f"heading_gate="
            f"{'PASS' if within_heading_gate else 'FAIL'} "
            f"window_start_s={window.start_s_m:.3f} "
            f"window_end_s={window.end_s_m:.3f} "
            f"window_samples={len(window.samples)} "
            f"window_geometry="
            f"{'USABLE' if window_usable else 'UNUSABLE'}"
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
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
