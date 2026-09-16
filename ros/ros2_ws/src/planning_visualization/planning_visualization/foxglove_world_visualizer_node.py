#!/usr/bin/env python3

import copy

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry, Path
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import MarkerArray


class FoxgloveWorldVisualizerNode(Node):
    """Adapt CARLA/planning/lane geometry for Foxglove visualization only."""

    def __init__(self) -> None:
        super().__init__("foxglove_world_visualizer")

        self.declare_parameter(
            "source_world_frame",
            "carla_world",
        )
        self.declare_parameter(
            "visualization_world_frame",
            "carla_world_viz",
        )
        self.declare_parameter(
            "visualization_ego_frame",
            "hero_viz",
        )

        self._source_world_frame = str(
            self.get_parameter(
                "source_world_frame"
            ).value
        )
        self._viz_world_frame = str(
            self.get_parameter(
                "visualization_world_frame"
            ).value
        )
        self._viz_ego_frame = str(
            self.get_parameter(
                "visualization_ego_frame"
            ).value
        )

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ---------------------------------------------------------
        # Global planning visualization.
        # ---------------------------------------------------------

        self._route_pub = self.create_publisher(
            Path,
            "/planning/route_path_viz",
            latched_qos,
        )

        self._graph_pub = self.create_publisher(
            MarkerArray,
            "/planning/global_lane_graph/markers_viz",
            latched_qos,
        )

        self.create_subscription(
            Path,
            "/planning/route_path",
            self._route_callback,
            latched_qos,
        )

        self.create_subscription(
            MarkerArray,
            "/planning/global_lane_graph/markers",
            self._graph_callback,
            latched_qos,
        )

        # ---------------------------------------------------------
        # Local lane visualization.
        #
        # These paths are already expressed in the tracker's
        # forward-left ego convention. Therefore their coordinates
        # must NOT be reflected. We only attach them to hero_viz.
        # ---------------------------------------------------------

        self._lane_path_topics = {
            "/perception/lane/left_boundary":
                "/perception/viz/lane/left_boundary",

            "/perception/lane/right_boundary":
                "/perception/viz/lane/right_boundary",

            "/perception/lane/centerline":
                "/perception/viz/lane/centerline",

            "/perception/lane/tracked_left_boundary":
                "/perception/viz/lane/tracked_left_boundary",

            "/perception/lane/tracked_right_boundary":
                "/perception/viz/lane/tracked_right_boundary",

            "/perception/lane/tracked_centerline":
                "/perception/viz/lane/tracked_centerline",
        }

        self._lane_publishers = {}
        self._lane_subscriptions = []

        for source_topic, target_topic in (
            self._lane_path_topics.items()
        ):
            publisher = self.create_publisher(
                Path,
                target_topic,
                10,
            )

            self._lane_publishers[
                source_topic
            ] = publisher

            subscription = self.create_subscription(
                Path,
                source_topic,
                lambda msg, source=source_topic:
                    self._lane_path_callback(
                        source,
                        msg,
                    ),
                10,
            )

            self._lane_subscriptions.append(
                subscription
            )

        # ---------------------------------------------------------
        # Visualization TF.
        # ---------------------------------------------------------

        self.create_subscription(
            Odometry,
            "/carla/hero_odom",
            self._odom_callback,
            qos_profile_sensor_data,
        )

        self._tf_broadcaster = (
            TransformBroadcaster(self)
        )

        self.get_logger().info(
            "Foxglove visualization adapter ready: "
            f"{self._source_world_frame} -> "
            f"{self._viz_world_frame}, "
            f"ego={self._viz_ego_frame}"
        )

        for source, target in (
            self._lane_path_topics.items()
        ):
            self.get_logger().info(
                f"Lane visualization: "
                f"{source} -> {target}"
            )

    # -------------------------------------------------------------
    # CARLA world -> ROS-style visualization world reflection.
    # -------------------------------------------------------------

    @staticmethod
    def _reflect_position(position) -> None:
        position.y = -float(position.y)

    @staticmethod
    def _reflect_quaternion(
        orientation,
    ) -> None:
        # Basis reflection S = diag(1, -1, 1):
        # R_viz = S R_carla S
        orientation.x = -float(
            orientation.x
        )
        orientation.y = float(
            orientation.y
        )
        orientation.z = -float(
            orientation.z
        )
        orientation.w = float(
            orientation.w
        )

    # -------------------------------------------------------------
    # Global RoutePlan path.
    # -------------------------------------------------------------

    def _route_callback(
        self,
        msg: Path,
    ) -> None:
        path = copy.deepcopy(msg)

        path.header.frame_id = (
            self._viz_world_frame
        )

        for pose_stamped in path.poses:
            pose_stamped.header.frame_id = (
                self._viz_world_frame
            )

            self._reflect_position(
                pose_stamped.pose.position
            )

            self._reflect_quaternion(
                pose_stamped.pose.orientation
            )

        self._route_pub.publish(path)

    # -------------------------------------------------------------
    # Global routing graph.
    # -------------------------------------------------------------

    def _graph_callback(
        self,
        msg: MarkerArray,
    ) -> None:
        markers = copy.deepcopy(msg)

        for marker in markers.markers:
            marker.header.frame_id = (
                self._viz_world_frame
            )

            self._reflect_position(
                marker.pose.position
            )

            self._reflect_quaternion(
                marker.pose.orientation
            )

            for point in marker.points:
                point.y = -float(point.y)

        self._graph_pub.publish(markers)

    # -------------------------------------------------------------
    # Ego-relative lane geometry.
    # -------------------------------------------------------------

    def _lane_path_callback(
        self,
        source_topic: str,
        msg: Path,
    ) -> None:
        path = copy.deepcopy(msg)

        # Geometry is already forward-left and must not be mirrored.
        path.header.frame_id = (
            self._viz_ego_frame
        )

        for pose_stamped in path.poses:
            pose_stamped.header.frame_id = (
                self._viz_ego_frame
            )

        publisher = (
            self._lane_publishers[
                source_topic
            ]
        )

        publisher.publish(path)

    # -------------------------------------------------------------
    # carla_world_viz -> hero_viz
    # -------------------------------------------------------------

    def _odom_callback(
        self,
        msg: Odometry,
    ) -> None:
        transform = TransformStamped()

        transform.header.stamp = (
            msg.header.stamp
        )
        transform.header.frame_id = (
            self._viz_world_frame
        )
        transform.child_frame_id = (
            self._viz_ego_frame
        )

        transform.transform.translation.x = float(
            msg.pose.pose.position.x
        )

        transform.transform.translation.y = -float(
            msg.pose.pose.position.y
        )

        transform.transform.translation.z = float(
            msg.pose.pose.position.z
        )

        transform.transform.rotation.x = -float(
            msg.pose.pose.orientation.x
        )

        transform.transform.rotation.y = float(
            msg.pose.pose.orientation.y
        )

        transform.transform.rotation.z = -float(
            msg.pose.pose.orientation.z
        )

        transform.transform.rotation.w = float(
            msg.pose.pose.orientation.w
        )

        self._tf_broadcaster.sendTransform(
            transform
        )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = FoxgloveWorldVisualizerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
