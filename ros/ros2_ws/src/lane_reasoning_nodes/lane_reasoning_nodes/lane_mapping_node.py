#!/usr/bin/env python3

import json
import math
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class LaneMappingNode(Node):
    """
    Skeleton accumulated lane mapping node.

    It consumes vehicle-frame lane geometry and hero odometry, then accumulates
    centerline evidence into a separate occupancy-grid lane layer.

    Map values:
      - -1 unknown
      - 0 free/empty
      - 100 lane evidence
    """

    def __init__(self):
        super().__init__("lane_mapping_node")

        self.declare_parameter("lane_geometry_topic", "/perception/lane_geometry")
        self.declare_parameter("hero_odom_topic", "/carla/hero_odom")
        self.declare_parameter("lane_map_topic", "/perception/accumulated_lane_map")
        self.declare_parameter("lane_map_debug_topic", "/perception/accumulated_lane_map_debug/compressed")
        self.declare_parameter("lane_mapping_status_topic", "/perception/lane_mapping_status")

        self.declare_parameter("map_size_m", 300.0)
        self.declare_parameter("resolution", 0.25)
        self.declare_parameter("lane_hit_inc", 3.0)
        self.declare_parameter("lane_decay", 0.999)
        self.declare_parameter("occupied_thresh", 2.0)
        self.declare_parameter("line_thickness_cells", 2)
        self.declare_parameter("publish_period_sec", 0.5)
        self.declare_parameter("pixel_scale", 2)

        self.map_size_m = float(self.get_parameter("map_size_m").value)
        self.resolution = float(self.get_parameter("resolution").value)
        self.size_cells = int(round(self.map_size_m / self.resolution))

        self.scores = np.zeros((self.size_cells, self.size_cells), dtype=np.float32)

        self.origin_x: Optional[float] = None
        self.origin_y: Optional[float] = None
        self.latest_pose: Optional[Tuple[float, float, float]] = None

        self.received_geometry_count = 0
        self.integrated_geometry_count = 0
        self.last_error = None

        lane_geometry_topic = self.get_parameter("lane_geometry_topic").value
        hero_odom_topic = self.get_parameter("hero_odom_topic").value
        lane_map_topic = self.get_parameter("lane_map_topic").value
        lane_map_debug_topic = self.get_parameter("lane_map_debug_topic").value
        lane_mapping_status_topic = self.get_parameter("lane_mapping_status_topic").value

        self.create_subscription(String, lane_geometry_topic, self.geometry_callback, 10)
        self.create_subscription(Odometry, hero_odom_topic, self.odom_callback, 10)

        self.map_pub = self.create_publisher(OccupancyGrid, lane_map_topic, 10)
        self.debug_pub = self.create_publisher(CompressedImage, lane_map_debug_topic, 10)
        self.status_pub = self.create_publisher(String, lane_mapping_status_topic, 10)

        period = float(self.get_parameter("publish_period_sec").value)
        self.create_timer(period, self.publish_outputs)

        self.get_logger().info("Lane mapping node started")
        self.get_logger().info(f"Map size: {self.map_size_m} m, resolution: {self.resolution} m/cell")
        self.get_logger().info(f"Input geometry: {lane_geometry_topic}")
        self.get_logger().info(f"Output map: {lane_map_topic}")

    def odom_callback(self, msg: Odometry) -> None:
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        self.latest_pose = (x, y, yaw)

        if self.origin_x is None or self.origin_y is None:
            self.origin_x = x - 0.5 * self.map_size_m
            self.origin_y = y - 0.5 * self.map_size_m
            self.get_logger().info(
                f"Initialized lane map origin at ({self.origin_x:.2f}, {self.origin_y:.2f})"
            )

    def geometry_callback(self, msg: String) -> None:
        self.received_geometry_count += 1

        if self.latest_pose is None:
            self.last_error = "missing hero odometry"
            return

        try:
            geometry = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.last_error = f"invalid lane_geometry JSON: {exc}"
            return

        if not geometry.get("lane_detected", False):
            self.last_error = "geometry reports no lane"
            return

        points = geometry.get("centerline_points_m", [])
        if not points:
            self.last_error = "empty centerline"
            return

        map_points = []
        for point in points:
            if not isinstance(point, list) or len(point) < 2:
                continue
            local_x = float(point[0])
            local_y = float(point[1])
            map_point = self.local_to_map(local_x, local_y)
            if map_point is not None:
                map_points.append(map_point)

        if len(map_points) < 2:
            self.last_error = "not enough valid map points"
            return

        self.integrate_polyline(map_points)
        self.integrated_geometry_count += 1
        self.last_error = None

    def local_to_map(self, local_x: float, local_y: float):
        if self.latest_pose is None or self.origin_x is None or self.origin_y is None:
            return None

        hero_x, hero_y, yaw = self.latest_pose

        world_x = hero_x + local_x * math.cos(yaw) - local_y * math.sin(yaw)
        world_y = hero_y + local_x * math.sin(yaw) + local_y * math.cos(yaw)

        mx = int(round((world_x - self.origin_x) / self.resolution))
        my = int(round((world_y - self.origin_y) / self.resolution))

        if mx < 0 or mx >= self.size_cells or my < 0 or my >= self.size_cells:
            return None

        return (mx, my)

    def integrate_polyline(self, map_points):
        decay = float(self.get_parameter("lane_decay").value)
        hit_inc = float(self.get_parameter("lane_hit_inc").value)
        thickness = int(self.get_parameter("line_thickness_cells").value)

        if decay < 1.0:
            self.scores *= decay

        temp = np.zeros_like(self.scores, dtype=np.uint8)

        for p0, p1 in zip(map_points[:-1], map_points[1:]):
            cv2.line(temp, p0, p1, 255, thickness=max(1, thickness))

        self.scores[temp > 0] += hit_inc
        np.clip(self.scores, 0.0, 100.0, out=self.scores)

    def publish_outputs(self):
        self.publish_map()
        self.publish_debug()
        self.publish_status()

    def publish_map(self):
        occupied_thresh = float(self.get_parameter("occupied_thresh").value)

        data = np.zeros_like(self.scores, dtype=np.int8)
        data[self.scores <= 0.0] = 0
        data[self.scores >= occupied_thresh] = 100

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "carla_world"

        msg.info.resolution = self.resolution
        msg.info.width = self.size_cells
        msg.info.height = self.size_cells

        if self.origin_x is not None and self.origin_y is not None:
            msg.info.origin.position.x = float(self.origin_x)
            msg.info.origin.position.y = float(self.origin_y)

        msg.info.origin.orientation.w = 1.0
        msg.data = data.flatten().tolist()

        self.map_pub.publish(msg)

    def publish_debug(self):
        occupied_thresh = float(self.get_parameter("occupied_thresh").value)
        pixel_scale = int(self.get_parameter("pixel_scale").value)
        pixel_scale = max(1, pixel_scale)

        img = np.zeros((self.size_cells, self.size_cells, 3), dtype=np.uint8)

        # Dark gray background.
        img[:, :] = (25, 25, 25)

        # Green lane evidence.
        img[self.scores >= occupied_thresh] = (0, 255, 0)

        # Hero position, if available.
        if self.latest_pose is not None and self.origin_x is not None and self.origin_y is not None:
            hero_x, hero_y, yaw = self.latest_pose
            mx = int(round((hero_x - self.origin_x) / self.resolution))
            my = int(round((hero_y - self.origin_y) / self.resolution))

            if 0 <= mx < self.size_cells and 0 <= my < self.size_cells:
                cv2.circle(img, (mx, my), 5, (255, 0, 0), -1)

                arrow_len = 20
                ex = int(round(mx + arrow_len * math.cos(yaw)))
                ey = int(round(my + arrow_len * math.sin(yaw)))
                cv2.arrowedLine(img, (mx, my), (ex, ey), (255, 0, 0), 2)

        # Flip vertically for easier image display.
        img = cv2.flip(img, 0)

        if pixel_scale != 1:
            img = cv2.resize(
                img,
                (img.shape[1] // pixel_scale, img.shape[0] // pixel_scale),
                interpolation=cv2.INTER_AREA,
            )

        ok, encoded = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        if not ok:
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "carla_world"
        msg.format = "jpeg"
        msg.data = encoded.tobytes()

        self.debug_pub.publish(msg)

    def publish_status(self):
        occupied_thresh = float(self.get_parameter("occupied_thresh").value)
        lane_cells = int(np.count_nonzero(self.scores >= occupied_thresh))

        status = {
            "received_geometry_count": self.received_geometry_count,
            "integrated_geometry_count": self.integrated_geometry_count,
            "odom_available": self.latest_pose is not None,
            "map_initialized": self.origin_x is not None and self.origin_y is not None,
            "lane_cells": lane_cells,
            "last_error": self.last_error,
        }

        self.status_pub.publish(String(data=json.dumps(status)))


def main(args=None):
    rclpy.init(args=args)
    node = LaneMappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
