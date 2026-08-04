#!/usr/bin/env python3

from __future__ import annotations

import json
import math

import cv2
import numpy as np
import rclpy

from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String


class LaneGeometryNode(Node):
    """
    First lane-geometry milestone:

    - consume binary road-marking mask;
    - consume camera calibration;
    - project visible ground markings into a metric BEV;
    - publish a raw BEV mask and a Foxglove debug image.

    Assumptions:
    - locally planar road;
    - camera has zero pitch, roll and yaw relative to the vehicle;
    - camera mount matches the CARLA bridge:
        x = 1.5 m forward
        z = 2.4 m above ground
    """

    def __init__(self) -> None:
        super().__init__("lane_geometry_node")

        self.bridge = CvBridge()

        self.declare_parameter(
            "mask_topic",
            "/perception/road_marking/mask",
        )
        self.declare_parameter(
            "camera_info_topic",
            "/carla/rgb/camera_info",
        )

        self.declare_parameter(
            "bev_mask_topic",
            "/perception/lane/bev_mask",
        )
        self.declare_parameter(
            "bev_debug_topic",
            "/perception/lane/bev_debug/compressed",
        )
        self.declare_parameter(
            "status_topic",
            "/perception/lane/bev_status",
        )

        self.declare_parameter("frame_id", "hero")

        self.declare_parameter("camera_forward_m", 1.5)
        self.declare_parameter("camera_height_m", 2.4)

        self.declare_parameter("forward_min_m", 0.0)
        self.declare_parameter("forward_max_m", 40.0)
        self.declare_parameter("left_extent_m", 12.0)
        self.declare_parameter("right_extent_m", -12.0)
        self.declare_parameter("resolution_m", 0.10)

        self.declare_parameter("grid_spacing_m", 5.0)
        self.declare_parameter("jpeg_quality", 80)

        self.mask_topic = str(
            self.get_parameter("mask_topic").value
        )
        self.camera_info_topic = str(
            self.get_parameter("camera_info_topic").value
        )

        self.bev_mask_topic = str(
            self.get_parameter("bev_mask_topic").value
        )
        self.bev_debug_topic = str(
            self.get_parameter("bev_debug_topic").value
        )
        self.status_topic = str(
            self.get_parameter("status_topic").value
        )

        self.frame_id = str(
            self.get_parameter("frame_id").value
        )

        self.camera_forward_m = float(
            self.get_parameter("camera_forward_m").value
        )
        self.camera_height_m = float(
            self.get_parameter("camera_height_m").value
        )

        self.forward_min_m = float(
            self.get_parameter("forward_min_m").value
        )
        self.forward_max_m = float(
            self.get_parameter("forward_max_m").value
        )
        self.left_extent_m = float(
            self.get_parameter("left_extent_m").value
        )
        self.right_extent_m = float(
            self.get_parameter("right_extent_m").value
        )
        self.resolution_m = float(
            self.get_parameter("resolution_m").value
        )

        self.grid_spacing_m = float(
            self.get_parameter("grid_spacing_m").value
        )
        self.jpeg_quality = int(
            self.get_parameter("jpeg_quality").value
        )

        self._validate_parameters()

        self.bev_height = int(
            math.ceil(
                (
                    self.forward_max_m
                    - self.forward_min_m
                )
                / self.resolution_m
            )
        )

        self.bev_width = int(
            math.ceil(
                (
                    self.left_extent_m
                    - self.right_extent_m
                )
                / self.resolution_m
            )
        )

        self.camera_info: CameraInfo | None = None
        self.map_x: np.ndarray | None = None
        self.map_y: np.ndarray | None = None
        self.waiting_warning_emitted = False
        self.frame_count = 0

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10,
        )

        self.mask_sub = self.create_subscription(
            Image,
            self.mask_topic,
            self.mask_callback,
            qos_profile_sensor_data,
        )

        self.bev_mask_pub = self.create_publisher(
            Image,
            self.bev_mask_topic,
            qos_profile_sensor_data,
        )

        self.bev_debug_pub = self.create_publisher(
            CompressedImage,
            self.bev_debug_topic,
            qos_profile_sensor_data,
        )

        self.status_pub = self.create_publisher(
            String,
            self.status_topic,
            10,
        )

        self.get_logger().info(
            f"Road-marking input: {self.mask_topic}"
        )
        self.get_logger().info(
            f"CameraInfo input: {self.camera_info_topic}"
        )
        self.get_logger().info(
            f"BEV output: {self.bev_mask_topic}"
        )
        self.get_logger().info(
            "BEV dimensions: "
            f"{self.bev_width}x{self.bev_height}, "
            f"{self.resolution_m:.2f} m/pixel"
        )

    def _validate_parameters(self) -> None:
        if self.resolution_m <= 0.0:
            raise ValueError("resolution_m must be positive")

        if self.forward_max_m <= self.forward_min_m:
            raise ValueError(
                "forward_max_m must exceed forward_min_m"
            )

        if self.left_extent_m <= self.right_extent_m:
            raise ValueError(
                "left_extent_m must exceed right_extent_m"
            )

        if self.camera_height_m <= 0.0:
            raise ValueError(
                "camera_height_m must be positive"
            )

    def camera_info_callback(
        self,
        msg: CameraInfo,
    ) -> None:
        if msg.width <= 0 or msg.height <= 0:
            self.get_logger().error(
                "CameraInfo has invalid dimensions"
            )
            return

        if len(msg.k) != 9:
            self.get_logger().error(
                "CameraInfo K matrix has invalid length"
            )
            return

        signature = (
            int(msg.width),
            int(msg.height),
            tuple(float(value) for value in msg.k),
        )

        previous_signature = None

        if self.camera_info is not None:
            previous_signature = (
                int(self.camera_info.width),
                int(self.camera_info.height),
                tuple(
                    float(value)
                    for value in self.camera_info.k
                ),
            )

        self.camera_info = msg

        if signature != previous_signature:
            self._build_remap(msg)
            self.get_logger().info(
                "Built ground-plane remap from CameraInfo: "
                f"{msg.width}x{msg.height}, "
                f"fx={msg.k[0]:.2f}, fy={msg.k[4]:.2f}, "
                f"cx={msg.k[2]:.2f}, cy={msg.k[5]:.2f}"
            )

    def _build_remap(
        self,
        camera_info: CameraInfo,
    ) -> None:
        fx = float(camera_info.k[0])
        fy = float(camera_info.k[4])
        cx = float(camera_info.k[2])
        cy = float(camera_info.k[5])

        if fx <= 0.0 or fy <= 0.0:
            raise ValueError(
                "CameraInfo focal lengths must be positive"
            )

        rows = np.arange(
            self.bev_height,
            dtype=np.float32,
        )[:, None]

        columns = np.arange(
            self.bev_width,
            dtype=np.float32,
        )[None, :]

        # Top row is far field; bottom row is near field.
        forward = (
            self.forward_max_m
            - (rows + 0.5) * self.resolution_m
        )

        # Left side of the BEV is positive vehicle-left.
        lateral_left = (
            self.left_extent_m
            - (columns + 0.5) * self.resolution_m
        )

        forward = np.broadcast_to(
            forward,
            (self.bev_height, self.bev_width),
        )

        lateral_left = np.broadcast_to(
            lateral_left,
            (self.bev_height, self.bev_width),
        )

        # Zero-pitch pinhole projection.
        #
        # Vehicle frame:
        #   x = forward
        #   y = left
        #   z = up
        #
        # Optical frame:
        #   X = right
        #   Y = down
        #   Z = forward
        optical_depth = (
            forward - self.camera_forward_m
        )

        valid = optical_depth > 0.10

        safe_depth = np.where(
            valid,
            optical_depth,
            1.0,
        )

        source_u = (
            cx
            - fx * lateral_left / safe_depth
        )

        source_v = (
            cy
            + fy * self.camera_height_m / safe_depth
        )

        valid &= source_u >= 0.0
        valid &= source_u < float(camera_info.width)
        valid &= source_v >= 0.0
        valid &= source_v < float(camera_info.height)

        self.map_x = np.where(
            valid,
            source_u,
            -1.0,
        ).astype(np.float32)

        self.map_y = np.where(
            valid,
            source_v,
            -1.0,
        ).astype(np.float32)

    def mask_callback(
        self,
        msg: Image,
    ) -> None:
        if (
            self.camera_info is None
            or self.map_x is None
            or self.map_y is None
        ):
            if not self.waiting_warning_emitted:
                self.get_logger().warning(
                    "Waiting for CameraInfo before generating BEV"
                )
                self.waiting_warning_emitted = True
            return

        try:
            mask = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="mono8",
            )
        except Exception as exc:
            self.get_logger().error(
                f"Could not decode road-marking mask: {exc}"
            )
            return

        expected_shape = (
            int(self.camera_info.height),
            int(self.camera_info.width),
        )

        if mask.shape[:2] != expected_shape:
            self.get_logger().error(
                "Mask/CameraInfo size mismatch: "
                f"mask={mask.shape[:2]}, "
                f"camera={expected_shape}"
            )
            return

        bev = cv2.remap(
            mask,
            self.map_x,
            self.map_y,
            interpolation=cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0,
        )

        bev = np.where(
            bev > 0,
            255,
            0,
        ).astype(np.uint8)

        bev_msg = self.bridge.cv2_to_imgmsg(
            bev,
            encoding="mono8",
        )
        bev_msg.header = msg.header
        bev_msg.header.frame_id = self.frame_id
        self.bev_mask_pub.publish(bev_msg)

        debug = self._build_debug_image(bev)

        ok, encoded = cv2.imencode(
            ".jpg",
            debug,
            [
                int(cv2.IMWRITE_JPEG_QUALITY),
                self.jpeg_quality,
            ],
        )

        if ok:
            debug_msg = CompressedImage()
            debug_msg.header = bev_msg.header
            debug_msg.format = "jpeg"
            debug_msg.data = encoded.tobytes()
            self.bev_debug_pub.publish(debug_msg)

        self.frame_count += 1

        status = String()
        status.data = json.dumps(
            {
                "frame": self.frame_count,
                "width": self.bev_width,
                "height": self.bev_height,
                "resolution_m": self.resolution_m,
                "forward_min_m": self.forward_min_m,
                "forward_max_m": self.forward_max_m,
                "left_extent_m": self.left_extent_m,
                "right_extent_m": self.right_extent_m,
                "positive_fraction": float(
                    np.count_nonzero(bev)
                    / bev.size
                ),
            },
            separators=(",", ":"),
        )
        self.status_pub.publish(status)

    def _build_debug_image(
        self,
        bev: np.ndarray,
    ) -> np.ndarray:
        debug = cv2.cvtColor(
            bev,
            cv2.COLOR_GRAY2BGR,
        )

        # Dim the raw white mask slightly so grid lines remain visible.
        debug[bev > 0] = (220, 220, 220)

        spacing = max(
            self.grid_spacing_m,
            self.resolution_m,
        )

        forward_value = (
            math.ceil(
                self.forward_min_m / spacing
            )
            * spacing
        )

        while forward_value <= self.forward_max_m:
            row, _ = self.metric_to_pixel(
                forward_value,
                0.0,
            )

            if 0 <= row < self.bev_height:
                cv2.line(
                    debug,
                    (0, row),
                    (self.bev_width - 1, row),
                    (80, 80, 80),
                    1,
                )

                cv2.putText(
                    debug,
                    f"{forward_value:.0f}m",
                    (4, max(12, row - 3)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.35,
                    (160, 160, 160),
                    1,
                    cv2.LINE_AA,
                )

            forward_value += spacing

        lateral_value = (
            math.ceil(
                self.right_extent_m / spacing
            )
            * spacing
        )

        while lateral_value <= self.left_extent_m:
            _, column = self.metric_to_pixel(
                self.forward_min_m,
                lateral_value,
            )

            if 0 <= column < self.bev_width:
                color = (
                    (0, 180, 255)
                    if abs(lateral_value) < 1e-6
                    else (80, 80, 80)
                )

                cv2.line(
                    debug,
                    (column, 0),
                    (column, self.bev_height - 1),
                    color,
                    1,
                )

            lateral_value += spacing

        ego_row, ego_column = self.metric_to_pixel(
            0.0,
            0.0,
        )

        if (
            0 <= ego_row < self.bev_height
            and 0 <= ego_column < self.bev_width
        ):
            triangle = np.array(
                [
                    [ego_column, ego_row - 8],
                    [ego_column - 5, ego_row + 2],
                    [ego_column + 5, ego_row + 2],
                ],
                dtype=np.int32,
            )

            cv2.fillConvexPoly(
                debug,
                triangle,
                (0, 0, 255),
            )

        return debug

    def metric_to_pixel(
        self,
        forward_m: float,
        lateral_left_m: float,
    ) -> tuple[int, int]:
        row = int(
            round(
                (
                    self.forward_max_m
                    - forward_m
                )
                / self.resolution_m
                - 0.5
            )
        )

        column = int(
            round(
                (
                    self.left_extent_m
                    - lateral_left_m
                )
                / self.resolution_m
                - 0.5
            )
        )

        return row, column


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LaneGeometryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
