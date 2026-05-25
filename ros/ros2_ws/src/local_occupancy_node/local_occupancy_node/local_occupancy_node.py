#!/usr/bin/env python3
import json
import math
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from cv_bridge import CvBridge


class LocalOccupancyNode(Node):
    """
    Local occupancy-grid prototype with separated static and dynamic obstacle layers.

    Inputs:
      /perception/semantic_mask      mono8 Cityscapes19 class IDs
      /perception/depth/image        32FC1 relative depth

    Outputs:
      /perception/local_occupancy_grid
      /perception/local_static_obstacle_grid
      /perception/local_dynamic_obstacle_grid
      /perception/local_occupancy_debug/compressed
      /perception/local_occupancy_status

    Combined grid convention:
      -1 unknown
       0 free
     100 occupied

    Debug image convention:
      gray   = unknown
      green  = free road
      red    = static non-drivable
      orange = dynamic obstacle
    """

    def __init__(self):
        super().__init__("local_occupancy_node")

        self.bridge = CvBridge()

        self.declare_parameter("semantic_topic", "/perception/semantic_mask")
        self.declare_parameter("depth_topic", "/perception/depth/image")

        self.declare_parameter("grid_topic", "/perception/local_occupancy_grid")
        self.declare_parameter("static_grid_topic", "/perception/local_static_obstacle_grid")
        self.declare_parameter("dynamic_grid_topic", "/perception/local_dynamic_obstacle_grid")
        self.declare_parameter("debug_topic", "/perception/local_occupancy_debug/compressed")
        self.declare_parameter("status_topic", "/perception/local_occupancy_status")

        self.declare_parameter("resolution", 0.25)
        self.declare_parameter("forward_m", 18.0)
        self.declare_parameter("width_m", 10.0)
        self.declare_parameter("frame_id", "carla_camera")

        self.declare_parameter("roi_x_min_ratio", 0.10)
        self.declare_parameter("roi_x_max_ratio", 0.90)
        self.declare_parameter("roi_y_min_ratio", 0.20)
        self.declare_parameter("roi_y_max_ratio", 0.95)

        # Static classes such as buildings, walls and vegetation are vertical.
        # Only their lower image region should be projected into the ground map.
        self.declare_parameter("static_y_min_ratio", 0.45)

        self.declare_parameter("hfov_deg", 90.0)
        self.declare_parameter("min_forward_m", 1.0)
        self.declare_parameter("far_power", 1.3)

        self.declare_parameter("pixel_stride", 3)

        self.declare_parameter("use_depth_obstacles", True)
        self.declare_parameter("depth_closer_is_larger", True)
        self.declare_parameter("close_depth_thresh", 40.0)

        self.declare_parameter("static_dilate_cells", 1)
        self.declare_parameter("dynamic_dilate_cells", 1)
        self.declare_parameter("free_dilate_cells", 1)

        self.declare_parameter("jpeg_quality", 70)

        self.semantic_topic = self.get_parameter("semantic_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value

        self.grid_topic = self.get_parameter("grid_topic").value
        self.static_grid_topic = self.get_parameter("static_grid_topic").value
        self.dynamic_grid_topic = self.get_parameter("dynamic_grid_topic").value
        self.debug_topic = self.get_parameter("debug_topic").value
        self.status_topic = self.get_parameter("status_topic").value

        self.resolution = float(self.get_parameter("resolution").value)
        self.forward_m = float(self.get_parameter("forward_m").value)
        self.width_m = float(self.get_parameter("width_m").value)
        self.frame_id = self.get_parameter("frame_id").value

        self.roi_x_min_ratio = float(self.get_parameter("roi_x_min_ratio").value)
        self.roi_x_max_ratio = float(self.get_parameter("roi_x_max_ratio").value)
        self.roi_y_min_ratio = float(self.get_parameter("roi_y_min_ratio").value)
        self.roi_y_max_ratio = float(self.get_parameter("roi_y_max_ratio").value)
        self.static_y_min_ratio = float(self.get_parameter("static_y_min_ratio").value)

        self.hfov_deg = float(self.get_parameter("hfov_deg").value)
        self.min_forward_m = float(self.get_parameter("min_forward_m").value)
        self.far_power = float(self.get_parameter("far_power").value)

        self.pixel_stride = int(self.get_parameter("pixel_stride").value)

        self.use_depth_obstacles = bool(self.get_parameter("use_depth_obstacles").value)
        self.depth_closer_is_larger = bool(self.get_parameter("depth_closer_is_larger").value)
        self.close_depth_thresh = float(self.get_parameter("close_depth_thresh").value)

        self.static_dilate_cells = int(self.get_parameter("static_dilate_cells").value)
        self.dynamic_dilate_cells = int(self.get_parameter("dynamic_dilate_cells").value)
        self.free_dilate_cells = int(self.get_parameter("free_dilate_cells").value)

        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)

        self.grid_width_cells = int(round(self.width_m / self.resolution))
        self.grid_height_cells = int(round(self.forward_m / self.resolution))

        self.latest_depth = None
        self.latest_depth_time = 0.0

        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_cb,
            10,
        )

        self.semantic_sub = self.create_subscription(
            Image,
            self.semantic_topic,
            self.semantic_cb,
            10,
        )

        self.grid_pub = self.create_publisher(
            OccupancyGrid,
            self.grid_topic,
            10,
        )

        self.static_grid_pub = self.create_publisher(
            OccupancyGrid,
            self.static_grid_topic,
            10,
        )

        self.dynamic_grid_pub = self.create_publisher(
            OccupancyGrid,
            self.dynamic_grid_topic,
            10,
        )

        self.debug_pub = self.create_publisher(
            CompressedImage,
            self.debug_topic,
            10,
        )

        self.status_pub = self.create_publisher(
            String,
            self.status_topic,
            10,
        )

        self.get_logger().info(f"Semantic input        : {self.semantic_topic}")
        self.get_logger().info(f"Depth input           : {self.depth_topic}")
        self.get_logger().info(f"Combined grid output  : {self.grid_topic}")
        self.get_logger().info(f"Static grid output    : {self.static_grid_topic}")
        self.get_logger().info(f"Dynamic grid output   : {self.dynamic_grid_topic}")
        self.get_logger().info(f"Debug output          : {self.debug_topic}")
        self.get_logger().info(f"Status output         : {self.status_topic}")
        self.get_logger().info(f"Grid size             : {self.grid_width_cells} x {self.grid_height_cells} cells")

    def depth_cb(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            self.latest_depth = depth.astype(np.float32)
            self.latest_depth_time = time.time()
        except Exception as e:
            self.get_logger().warn(f"Could not read depth image: {e}")

    def semantic_cb(self, msg: Image):
        try:
            sem = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
            sem = sem.astype(np.uint8)
        except Exception as e:
            self.get_logger().warn(f"Could not read semantic mask: {e}")
            return

        h, w = sem.shape[:2]

        depth = None
        if self.latest_depth is not None:
            depth = self.latest_depth
            if depth.shape[:2] != sem.shape[:2]:
                depth = cv2.resize(depth, (w, h), interpolation=cv2.INTER_LINEAR)

        combined, free_layer, static_layer, dynamic_layer = self.build_layers(sem, depth)

        self.publish_grid(combined, msg.header, self.grid_pub)
        self.publish_grid(static_layer, msg.header, self.static_grid_pub)
        self.publish_grid(dynamic_layer, msg.header, self.dynamic_grid_pub)
        self.publish_debug(combined, free_layer, static_layer, dynamic_layer, msg.header)
        self.publish_status(combined, free_layer, static_layer, dynamic_layer, msg.header)

    def build_layers(self, sem, depth):
        h, w = sem.shape[:2]

        free = np.zeros((self.grid_height_cells, self.grid_width_cells), dtype=np.uint8)
        static = np.zeros((self.grid_height_cells, self.grid_width_cells), dtype=np.uint8)
        dynamic = np.zeros((self.grid_height_cells, self.grid_width_cells), dtype=np.uint8)

        # Cityscapes19 split.
        free_ids = {
            0,   # road
        }

        static_ids = {
            1,   # sidewalk
            2,   # building
            3,   # wall
            4,   # fence
            5,   # pole
            8,   # vegetation
            9,   # terrain
        }

        dynamic_ids = {
            11,  # person
            12,  # rider
            13,  # car
            14,  # truck
            15,  # bus
            16,  # train
            17,  # motorcycle
            18,  # bicycle
        }

        x1 = int(w * self.roi_x_min_ratio)
        x2 = int(w * self.roi_x_max_ratio)
        y1 = int(h * self.roi_y_min_ratio)
        y2 = int(h * self.roi_y_max_ratio)

        x1 = max(0, min(w - 1, x1))
        x2 = max(x1 + 1, min(w, x2))
        y1 = max(0, min(h - 1, y1))
        y2 = max(y1 + 1, min(h, y2))

        stride = max(1, self.pixel_stride)

        for v in range(y1, y2, stride):
            for u in range(x1, x2, stride):
                cls = int(sem[v, u])

                is_free = cls in free_ids
                is_static = cls in static_ids
                is_dynamic = cls in dynamic_ids

                # Static semantic classes are often vertical surfaces.
                # Avoid projecting upper-image building/tree/wall pixels as if
                # they were ground-plane obstacles in front of the hero vehicle.
                if is_static and v < int(h * self.static_y_min_ratio):
                    is_static = False

                # Depth is currently used only as auxiliary information.
                # Important: do not promote unknown/noisy non-road pixels to static obstacles
                # using depth alone, because this creates false red regions in the far front
                # of the local occupancy map.
                #
                # For now, semantic class decides free/static/dynamic.

                if not is_free and not is_static and not is_dynamic:
                    continue

                gx, gy = self.project_pixel_to_grid(u, v, w, h, y1, y2)

                if gx is None or gy is None:
                    continue

                if is_dynamic:
                    dynamic[gy, gx] = 1
                elif is_static:
                    static[gy, gx] = 1
                elif is_free:
                    free[gy, gx] = 1

        free, static, dynamic = self.postprocess_layers(free, static, dynamic)

        combined = np.full((self.grid_height_cells, self.grid_width_cells), -1, dtype=np.int16)

        # Priority:
        # dynamic obstacle > static obstacle > free > unknown
        combined[free > 0] = 0
        combined[static > 0] = 100
        combined[dynamic > 0] = 100

        static_grid = np.full_like(combined, -1)
        static_grid[free > 0] = 0
        static_grid[static > 0] = 100

        dynamic_grid = np.full_like(combined, -1)
        dynamic_grid[free > 0] = 0
        dynamic_grid[dynamic > 0] = 100

        return combined, free, static_grid, dynamic_grid

    def project_pixel_to_grid(self, u, v, image_w, image_h, roi_y1, roi_y2):
        denom = max(1.0, float(roi_y2 - roi_y1))
        y_norm = (float(v) - float(roi_y1)) / denom
        y_norm = max(0.0, min(1.0, y_norm))

        # Bottom of ROI is near, top of ROI is far.
        far_factor = (1.0 - y_norm) ** self.far_power
        forward = self.min_forward_m + far_factor * (self.forward_m - self.min_forward_m)

        if forward < 0.0 or forward > self.forward_m:
            return None, None

        x_norm = (float(u) - (float(image_w) / 2.0)) / (float(image_w) / 2.0)
        half_fov_rad = math.radians(self.hfov_deg / 2.0)
        lateral = x_norm * forward * math.tan(half_fov_rad)

        if lateral < -self.width_m / 2.0 or lateral > self.width_m / 2.0:
            return None, None

        gx = int((lateral + self.width_m / 2.0) / self.resolution)
        gy = int(forward / self.resolution)

        if gx < 0 or gx >= self.grid_width_cells:
            return None, None

        if gy < 0 or gy >= self.grid_height_cells:
            return None, None

        return gx, gy

    def postprocess_layers(self, free, static, dynamic):
        if self.free_dilate_cells > 0:
            k = 2 * self.free_dilate_cells + 1
            kernel = np.ones((k, k), np.uint8)
            free = cv2.dilate(free, kernel, iterations=1)

        if self.static_dilate_cells > 0:
            k = 2 * self.static_dilate_cells + 1
            kernel = np.ones((k, k), np.uint8)
            static = cv2.dilate(static, kernel, iterations=1)

        if self.dynamic_dilate_cells > 0:
            k = 2 * self.dynamic_dilate_cells + 1
            kernel = np.ones((k, k), np.uint8)
            dynamic = cv2.dilate(dynamic, kernel, iterations=1)

        # Obstacle layers override free.
        free[(static > 0) | (dynamic > 0)] = 0

        return free, static, dynamic

    def publish_grid(self, grid, header, pub):
        msg = OccupancyGrid()
        msg.header.stamp = header.stamp
        msg.header.frame_id = self.frame_id

        msg.info.resolution = float(self.resolution)
        msg.info.width = int(self.grid_width_cells)
        msg.info.height = int(self.grid_height_cells)

        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = -self.width_m / 2.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = grid.astype(np.int8).reshape(-1).tolist()
        pub.publish(msg)

    def publish_debug(self, combined, free, static_grid, dynamic_grid, header):
        h, w = combined.shape[:2]

        img = np.zeros((h, w, 3), dtype=np.uint8)

        static = static_grid == 100
        dynamic = dynamic_grid == 100

        img[combined == -1] = (80, 80, 80)      # unknown: gray
        img[free > 0] = (0, 180, 0)             # free: green
        img[static] = (0, 0, 220)               # static: red
        img[dynamic] = (0, 165, 255)            # dynamic: orange

        # Near field at bottom for intuitive display.
        img = cv2.flip(img, 0)
        img = cv2.resize(img, (w * 4, h * 4), interpolation=cv2.INTER_NEAREST)

        ok, jpg = cv2.imencode(
            ".jpg",
            img,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )

        if not ok:
            return

        msg = CompressedImage()
        msg.header = header
        msg.format = "jpeg"
        msg.data = jpg.tobytes()

        self.debug_pub.publish(msg)

    def publish_status(self, combined, free_layer, static_grid, dynamic_grid, header):
        total = combined.size

        free_count = int(np.count_nonzero(combined == 0))
        occupied_count = int(np.count_nonzero(combined == 100))
        unknown_count = int(np.count_nonzero(combined == -1))

        static_count = int(np.count_nonzero(static_grid == 100))
        dynamic_count = int(np.count_nonzero(dynamic_grid == 100))

        h, w = combined.shape[:2]

        def ratio(region, value):
            if region.size == 0:
                return 0.0
            return float(np.count_nonzero(region == value) / region.size)

        def summarize_region(c_region, s_region, d_region):
            return {
                "free": ratio(c_region, 0),
                "occupied": ratio(c_region, 100),
                "static": ratio(s_region, 100),
                "dynamic": ratio(d_region, 100),
                "unknown": ratio(c_region, -1),
            }

        near_y1 = 0
        near_y2 = max(1, int(h * 0.40))

        mid_y1 = near_y2
        mid_y2 = max(mid_y1 + 1, int(h * 0.75))

        left_x1 = 0
        left_x2 = int(w * 0.33)

        center_x1 = int(w * 0.33)
        center_x2 = int(w * 0.67)

        right_x1 = int(w * 0.67)
        right_x2 = w

        def regions(y1, y2):
            return {
                "left": (
                    combined[y1:y2, left_x1:left_x2],
                    static_grid[y1:y2, left_x1:left_x2],
                    dynamic_grid[y1:y2, left_x1:left_x2],
                ),
                "center": (
                    combined[y1:y2, center_x1:center_x2],
                    static_grid[y1:y2, center_x1:center_x2],
                    dynamic_grid[y1:y2, center_x1:center_x2],
                ),
                "right": (
                    combined[y1:y2, right_x1:right_x2],
                    static_grid[y1:y2, right_x1:right_x2],
                    dynamic_grid[y1:y2, right_x1:right_x2],
                ),
            }

        near_regions = regions(near_y1, near_y2)
        mid_regions = regions(mid_y1, mid_y2)

        near = {
            k: summarize_region(*v)
            for k, v in near_regions.items()
        }

        mid = {
            k: summarize_region(*v)
            for k, v in mid_regions.items()
        }

        left_score = (
            0.65 * near["left"]["free"]
            + 0.35 * mid["left"]["free"]
            - 0.55 * near["left"]["static"]
            - 0.85 * near["left"]["dynamic"]
        )

        center_score = (
            0.65 * near["center"]["free"]
            + 0.35 * mid["center"]["free"]
            - 0.70 * near["center"]["static"]
            - 1.00 * near["center"]["dynamic"]
        )

        right_score = (
            0.65 * near["right"]["free"]
            + 0.35 * mid["right"]["free"]
            - 0.55 * near["right"]["static"]
            - 0.85 * near["right"]["dynamic"]
        )

        scores = {
            "left": float(left_score),
            "center": float(center_score),
            "right": float(right_score),
        }

        near_center_free = near["center"]["free"]
        near_center_static = near["center"]["static"]
        near_center_dynamic = near["center"]["dynamic"]
        near_center_occupied = near["center"]["occupied"]

        # Conservative recommendation:
        # dynamic obstacles in the near center are more urgent than static boundaries.
        if near_center_dynamic >= 0.18:
            if left_score > right_score and left_score > 0.02:
                recommended = "left"
            elif right_score > left_score and right_score > 0.02:
                recommended = "right"
            else:
                recommended = "stop"
        elif near_center_occupied >= 0.35:
            if left_score > right_score and left_score > 0.02:
                recommended = "left"
            elif right_score > left_score and right_score > 0.02:
                recommended = "right"
            else:
                recommended = "stop"
        elif near_center_free >= 0.35 and center_score >= max(left_score, right_score) - 0.10:
            recommended = "forward"
        else:
            if left_score > right_score + 0.08:
                recommended = "left"
            elif right_score > left_score + 0.08:
                recommended = "right"
            elif center_score > 0.03:
                recommended = "slow"
            else:
                recommended = "stop"

        status = {
            "stamp": {
                "sec": int(header.stamp.sec),
                "nanosec": int(header.stamp.nanosec),
            },
            "frame_id": self.frame_id,
            "grid": {
                "resolution": self.resolution,
                "width_cells": self.grid_width_cells,
                "height_cells": self.grid_height_cells,
                "width_m": self.width_m,
                "forward_m": self.forward_m,
            },
            "global_ratios": {
                "free": float(free_count / total),
                "occupied": float(occupied_count / total),
                "static": float(static_count / total),
                "dynamic": float(dynamic_count / total),
                "unknown": float(unknown_count / total),
            },
            "near_field": near,
            "mid_field": mid,
            "scores": scores,
            "recommended_direction": recommended,
            "legacy_ratios": {
                "near_center_free": near_center_free,
                "near_center_occupied": near_center_occupied,
                "near_center_static": near_center_static,
                "near_center_dynamic": near_center_dynamic,
            },
        }

        msg = String()
        msg.data = json.dumps(status, indent=2)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = LocalOccupancyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
