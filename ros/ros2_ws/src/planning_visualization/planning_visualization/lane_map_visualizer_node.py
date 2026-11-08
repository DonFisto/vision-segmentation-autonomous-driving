"""Render rolling LaneMap geometry for Foxglove."""

import copy
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from autonomy_interfaces.msg import LaneMap
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


VIS_FRAME = "carla_world_viz"


def point_viz(point, z_offset=0.10):
    output = Point()
    output.x = float(point.x)
    output.y = -float(point.y)
    output.z = float(point.z) + z_offset
    return output


def line_marker(header, marker_id, namespace, width, color):
    marker = Marker()
    marker.header = copy.deepcopy(header)
    marker.header.frame_id = VIS_FRAME

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

    return marker


def build_lane_map_markers(message):
    """Convert a schema-v3 world-frame LaneMap into Foxglove markers."""

    result = MarkerArray()

    clear = Marker()
    clear.header = copy.deepcopy(message.header)
    clear.header.frame_id = VIS_FRAME
    clear.action = Marker.DELETEALL
    result.markers.append(clear)

    if (
        int(message.schema_version) != 3
        or str(message.header.frame_id) != "carla_world"
    ):
        return result

    def valid(point):
        return all(
            math.isfinite(float(value))
            for value in (point.x, point.y, point.z)
        )

    for index, segment in enumerate(message.segments):
        if not bool(segment.drivable):
            continue

        # The centerline is the actual rolling LaneMap geometry.
        samples = [
            sample
            for sample in segment.centerline
            if valid(sample.pose.position)
        ]

        if len(samples) >= 2:
            center = line_marker(
                message.header,
                index,
                "lane_map_center",
                0.14,
                (0.10, 0.82, 0.76, 1.0),
            )

            for sample in samples:
                center.points.append(
                    point_viz(sample.pose.position)
                )

                confidence = float(sample.confidence)

                if not math.isfinite(confidence):
                    confidence = 0.0

                confidence = max(
                    0.0, min(1.0, confidence)
                )

                color = ColorRGBA()
                color.r = 1.0 - confidence
                color.g = 0.25 + 0.70 * confidence
                color.b = 0.20
                color.a = 1.0

                center.colors.append(color)

            result.markers.append(center)

        # Left and right boundaries.
        for side, boundary in (
            ("left", segment.left_boundary),
            ("right", segment.right_boundary),
        ):
            points = [
                point
                for point in boundary.points
                if valid(point)
            ]

            if len(points) < 2:
                continue

            marker = line_marker(
                message.header,
                index,
                "lane_map_" + side,
                0.07,
                (0.90, 0.75, 0.18, 0.85),
            )

            for point in points:
                marker.points.append(
                    point_viz(point, 0.06)
                )

            result.markers.append(marker)

    return result


class LaneMapVisualizerNode(Node):
    def __init__(self):
        super().__init__("lane_map_visualizer")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.publisher = self.create_publisher(
            MarkerArray,
            "/perception/viz/lane/map_markers",
            qos,
        )

        self.subscriber = self.create_subscription(
            LaneMap,
            "/perception/lane/local_map/vector",
            self.on_map,
            qos,
        )

        self._last_arrival = None
        self._expired = False

        self.create_timer(0.5, self._clear_expired)

        self.get_logger().info(
            "LaneMap visualization ready in carla_world_viz"
        )

    def on_map(self, message):
        self._last_arrival = time.monotonic()
        self._expired = False

        self.publisher.publish(
            build_lane_map_markers(message)
        )

    def _clear_expired(self):
        if (
            self._last_arrival is None
            or self._expired
            or time.monotonic() - self._last_arrival <= 2.0
        ):
            return

        self._expired = True

        message = LaneMap()
        message.header.frame_id = "carla_world"

        self.publisher.publish(
            build_lane_map_markers(message)
        )


def main(args=None):
    rclpy.init(args=args)
    node = LaneMapVisualizerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
