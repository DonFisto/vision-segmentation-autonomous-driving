"""Visualize diagnostic-owned LaneMap–RoutePlan association results."""

import copy
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)

from autonomy_interfaces.msg import (
    LaneMap,
    RoutePlan,
    RouteAssociationStatus,
)
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from planning_visualization.route_association_marker_geometry import (
    clip_intervals,
    slice_polyline_xy,
)


FRAME = "carla_world_viz"


def make_line(header, marker_id, namespace, width, color, points):
    marker = Marker()
    marker.header = copy.deepcopy(header)
    marker.header.frame_id = FRAME

    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0

    marker.scale.x = width

    (
        marker.color.r,
        marker.color.g,
        marker.color.b,
        marker.color.a,
    ) = color

    for x, y, z in points:
        if not all(
            math.isfinite(float(v))
            for v in (x, y, z)
        ):
            return None

        point = Point()
        point.x = float(x)
        point.y = -float(y)
        point.z = float(z) + 0.17

        marker.points.append(point)

    return marker if len(marker.points) >= 2 else None


def build_association_markers(route, lane_map, status):
    result = MarkerArray()

    clear = Marker()
    clear.header = copy.deepcopy(status.header)
    clear.header.frame_id = FRAME
    clear.action = Marker.DELETEALL
    result.markers.append(clear)

    if route is None or route.status != RoutePlan.STATUS_VALID:
        return result

    if (
        int(route.mission_id) != int(status.mission_id)
        or int(route.route_id) != int(status.route_id)
        or int(route.map_revision) != int(status.route_map_revision)
    ):
        return result

    if route.header.frame_id != "carla_world":
        return result

    if (
        lane_map is None
        or lane_map.header.frame_id != "carla_world"
        or int(lane_map.map_session_id) != int(status.map_session_id)
        or int(lane_map.map_revision) != int(status.lane_map_revision)
    ):
        return result

    if (
        not status.metrics_valid
        or status.temporal_status in (
            "STALE",
            "WAITING",
            "UNAVAILABLE",
        )
    ):
        return result

    points = [
        (
            float(p.pose.position.x),
            float(p.pose.position.y),
            float(p.pose.position.z),
        )
        for p in route.coarse_reference_path.poses
    ]

    if len(points) < 2:
        return result

    start = float(status.window_start_s_m)
    end = float(status.window_end_s_m)

    window = slice_polyline_xy(points, start, end)

    base = make_line(
        status.header,
        0,
        "association_window",
        0.17,
        (0.57, 0.60, 0.66, 0.75),
        window,
    )

    if base is not None:
        result.markers.append(base)

    # Green only for temporally confirmed support.
    # Candidates and partial observations remain amber.
    if (
        status.decision_status in ("SUPPORTED", "PARTIAL")
        and status.temporal_status in (
            "SUPPORTED",
            "CANDIDATE",
            "PARTIAL",
        )
        and status.evidence_time_status == "FRESH"
    ):
        color = (
            (0.12, 0.90, 0.32, 1.0)
            if status.temporal_status == "SUPPORTED"
            else (1.0, 0.71, 0.16, 1.0)
        )

        intervals = clip_intervals(
            status.supported_interval_start_s_m,
            status.supported_interval_end_s_m,
            start,
            end,
        )

        for index, (a, b) in enumerate(intervals):
            marker = make_line(
                status.header,
                10 + index,
                "observed_support",
                0.30,
                color,
                slice_polyline_xy(points, a, b),
            )

            if marker is not None:
                result.markers.append(marker)

    if status.selected_segment_valid:
        for segment in lane_map.segments:
            if int(segment.id) != int(status.selected_segment_id):
                continue

            segment_points = [
                (
                    float(p.pose.position.x),
                    float(p.pose.position.y),
                    float(p.pose.position.z),
                )
                for p in segment.centerline
            ]

            if status.temporal_status in (
                "SUPPORTED",
                "CANDIDATE",
            ):
                color = (0.0, 0.87, 0.88, 0.98)
            elif status.temporal_status == "PARTIAL":
                color = (1.0, 0.71, 0.16, 0.98)
            else:
                color = (1.0, 0.38, 0.28, 0.95)

            marker = make_line(
                status.header,
                100,
                "selected_lane",
                0.22,
                color,
                segment_points,
            )

            if marker is not None:
                result.markers.append(marker)

            break

    return result


class RouteAssociationVisualizerNode(Node):
    def __init__(self):
        super().__init__("route_association_visualizer")

        reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._route = None
        self._maps = {}
        self._status = None

        self._pub = self.create_publisher(
            MarkerArray,
            "/planning/route_association/markers_viz",
            reliable,
        )

        self.create_subscription(
            RoutePlan,
            "/planning/route_plan",
            self._on_route,
            latched,
        )

        self.create_subscription(
            LaneMap,
            "/perception/lane/local_map/vector",
            self._on_map,
            reliable,
        )

        self.create_subscription(
            RouteAssociationStatus,
            "/planning/route_association/status",
            self._on_status,
            reliable,
        )

        self.get_logger().info(
            "Association markers ready in carla_world_viz"
        )

    def _publish(self):
        if self._status is None:
            return

        key = (
            int(self._status.map_session_id),
            int(self._status.lane_map_revision),
        )

        matching_map = self._maps.get(key)

        self._pub.publish(
            build_association_markers(
                self._route,
                matching_map,
                self._status,
            )
        )

    def _on_route(self, message):
        self._route = message
        self._publish()

    def _on_map(self, message):
        key = (
            int(message.map_session_id),
            int(message.map_revision),
        )

        self._maps[key] = message

        while len(self._maps) > 32:
            self._maps.pop(next(iter(self._maps)))

        self._publish()

    def _on_status(self, message):
        self._status = message
        self._publish()


def main(args=None):
    rclpy.init(args=args)
    node = RouteAssociationVisualizerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
