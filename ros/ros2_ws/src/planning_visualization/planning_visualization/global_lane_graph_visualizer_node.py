#!/usr/bin/env python3

import carla

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from global_route_planner.carla_topology_adapter import (
    build_topology_graph,
)
from global_route_planner.carla_routing_graph_adapter import (
    build_routing_graph,
)
from global_route_planner.routing_graph import (
    RoutingEdgeType,
)


class GlobalLaneGraphVisualizerNode(Node):
    """Publish the global OpenDRIVE routing graph as Foxglove markers."""

    def __init__(self) -> None:
        super().__init__("global_lane_graph_visualizer")

        self.declare_parameter("host", "localhost")
        self.declare_parameter("port", 2000)
        self.declare_parameter("timeout_s", 5.0)

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
            "frame_id",
            "carla_world",
        )

        self.declare_parameter(
            "marker_topic",
            "/planning/global_lane_graph/markers",
        )

        host = str(
            self.get_parameter("host").value
        )

        port = int(
            self.get_parameter("port").value
        )

        timeout_s = float(
            self.get_parameter("timeout_s").value
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

        self._frame_id = str(
            self.get_parameter("frame_id").value
        )

        marker_topic = str(
            self.get_parameter("marker_topic").value
        )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._publisher = self.create_publisher(
            MarkerArray,
            marker_topic,
            qos,
        )

        self.get_logger().info(
            f"Connecting to CARLA at {host}:{port}"
        )

        client = carla.Client(host, port)
        client.set_timeout(timeout_s)

        world = client.get_world()
        carla_map = world.get_map()

        topology_graph = build_topology_graph(
            carla_map,
            sampling_resolution_m=(
                sampling_resolution_m
            ),
        )

        routing_result = build_routing_graph(
            carla_map=carla_map,
            topology_graph=topology_graph,
            lane_change_penalty_m=(
                lane_change_penalty_m
            ),
            max_lane_change_s_error_m=(
                max_lane_change_s_error_m
            ),
        )

        self._graph = routing_result.graph

        marker_array = self._build_markers()

        self._publisher.publish(marker_array)

        self.get_logger().info(
            f"Published global lane graph: "
            f"nodes={len(self._graph.nodes)} "
            f"edges={len(self._graph.edges)} "
            f"topic={marker_topic}"
        )

    def _base_marker(
        self,
        marker_id: int,
        namespace: str,
        width_m: float,
        r: float,
        g: float,
        b: float,
        a: float,
    ) -> Marker:
        marker = Marker()

        marker.header.stamp = (
            self.get_clock().now().to_msg()
        )
        marker.header.frame_id = self._frame_id

        marker.ns = namespace
        marker.id = marker_id

        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0

        marker.scale.x = width_m

        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = a

        return marker

    @staticmethod
    def _point(node) -> Point:
        point = Point()

        point.x = float(node.x)
        point.y = float(node.y)

        # Slight elevation helps prevent z-fighting with the
        # Foxglove ground/grid plane.
        point.z = float(node.z) + 0.05

        return point

    def _build_markers(self) -> MarkerArray:
        lane_follow = self._base_marker(
            marker_id=0,
            namespace="lane_follow",
            width_m=0.12,
            r=0.55,
            g=0.65,
            b=0.75,
            a=0.80,
        )

        lane_change_left = self._base_marker(
            marker_id=1,
            namespace="lane_change_left",
            width_m=0.20,
            r=0.25,
            g=0.85,
            b=0.35,
            a=0.90,
        )

        lane_change_right = self._base_marker(
            marker_id=2,
            namespace="lane_change_right",
            width_m=0.20,
            r=0.95,
            g=0.55,
            b=0.20,
            a=0.90,
        )

        markers_by_type = {
            RoutingEdgeType.LANE_FOLLOW: lane_follow,
            RoutingEdgeType.LANE_CHANGE_LEFT: (
                lane_change_left
            ),
            RoutingEdgeType.LANE_CHANGE_RIGHT: (
                lane_change_right
            ),
        }

        for edge in self._graph.edges.values():
            marker = markers_by_type.get(
                edge.transition_type
            )

            if marker is None:
                continue

            source = self._graph.nodes[
                edge.source
            ]

            target = self._graph.nodes[
                edge.target
            ]

            marker.points.append(
                self._point(source)
            )

            marker.points.append(
                self._point(target)
            )

        result = MarkerArray()

        result.markers = [
            lane_follow,
            lane_change_left,
            lane_change_right,
        ]

        return result


def main(args=None) -> None:
    rclpy.init(args=args)

    node = GlobalLaneGraphVisualizerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
