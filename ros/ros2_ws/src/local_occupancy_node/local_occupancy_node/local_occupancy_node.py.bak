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
    First local occupancy-grid prototype.

    Inputs:
      /perception/semantic_mask
      /perception/depth/image

    Outputs:
      /perception/local_occupancy_grid
      /perception/local_occupancy_debug/compressed
      /perception/local_occupancy_status
    """

    def __init__(self):
        super().__init__("local_occupancy_node")

        self.bridge = CvBridge()

        self.declare_parameter("semantic_topic", "/perception/semantic_mask")
        self.declare_parameter("depth_topic", "/perception/depth/image")
        self.declare_parameter("grid_topic", "/perception/local_occupancy_grid")
        self.declare_parameter("debug_topic", "/perception/local_occupancy_debug/compressed")
        self.declare_parameter("status_topic", "/perception/local_occupancy_status")

        self.declare_parameter("resolution", 0.25)
        self.declare_parameter("forward_m", 20.0)
        self.declare_parameter("width_m", 12.0)
        self.declare_parameter("frame_id", "carla_camera")

        self.declare_parameter("roi_x_min_ratio", 0.05)
        self.declare_parameter("roi_x_max_ratio", 0.95)
        self.declare_parameter("roi_y_min_ratio", 0.25)
        self.declare_parameter("roi_y_max_ratio", 0.95)

        self.declare_parameter("hfov_deg", 90.0)
        self.declare_parameter("min_forward_m", 1.0)
        self.declare_parameter("far_power", 1.8)

        self.declare_parameter("pixel_stride", 4)

        self.declare_parameter("use_depth_obstacles", True)
        self.declare_parameter("depth_closer_is_larger", True)
        self.declare_parameter("close_depth_thresh", 40.0)

        self.declare_parameter("obstacle_dilate_cells", 1)
        self.declare_parameter("free_dilate_cells", 0)

        self.declare_parameter("jpeg_quality", 70)

        self.semantic_topic = self.get_parameter("semantic_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.grid_topic = self.get_parameter("grid_topic").value
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

        self.hfov_deg = float(self.get_parameter("hfov_deg").value)
        self.min_forward_m = float(self.get_parameter("min_forward_m").value)
        self.far_power = float(self.get_parameter("far_power").value)

        self.pixel_stride = int(self.get_parameter("pixel_stride").value)

        self.use_depth_obstacles = bool(self.get_parameter("use_depth_obstacles").value)
        self.depth_closer_is_larger = bool(self.get_parameter("depth_closer_is_larger").value)
        self.close_depth_thresh = float(self.get_parameter("close_depth_thresh").value)

        self.obstacle_dilate_cells = int(self.get_parameter("obstacle_dilate_cells").value)
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

        self.get_logger().info(f"Semantic input : {self.semantic_topic}")
        self.get_logger().info(f"Depth input    : {self.depth_topic}")
        self.get_logger().info(f"Grid output    : {self.grid_topic}")
        self.get_logger().info(f"Debug output   : {self.debug_topic}")
        self.get_logger().info(f"Grid size      : {self.grid_width_cells} x {self.grid_height_cells} cells")
        self.get_logger().info("This is an approximate local occupancy prototype, not metric SLAM yet.")

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

        grid = self.build_grid(sem, depth)

        self.publish_grid(grid, msg.header)
        self.publish_debug(grid, msg.header)
        self.publish_status(grid, msg.header)

    def build_grid(self, sem, depth):
        h, w = sem.shape[:2]

        grid = np.full(
            (self.grid_height_cells, self.grid_width_cells),
            -1,
            dtype=np.int16,
        )

        # Cityscapes19 IDs.
        road_ids = {0}

        obstacle_ids = {
            1,   # sidewalk / non-drivable walkpath
            2,   # building / structure
            3,   # wall
            4,   # fence
            5,   # pole
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

                is_free = cls in road_ids
                is_obstacle = cls in obstacle_ids

                if self.use_depth_obstacles and depth is not None:
                    d = float(depth[v, u])
                    if math.isfinite(d):
                        if self.depth_closer_is_larger:
                            close = d >= self.close_depth_thresh
                        else:
                            close = d <= self.close_depth_thresh

                        if close and not is_free:
                            is_obstacle = True

                if not is_free and not is_obstacle:
                    continue

                gx, gy = self.project_pixel_to_grid(u, v, w, h, y1, y2)

                if gx is None or gy is None:
                    continue

                if is_obstacle:
                    grid[gy, gx] = 100
                elif is_free and grid[gy, gx] != 100:
                    grid[gy, gx] = 0

        return self.postprocess_grid(grid)

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

    def postprocess_grid(self, grid):
        occupied = (grid == 100).astype(np.uint8)
        free = (grid == 0).astype(np.uint8)

        if self.obstacle_dilate_cells > 0:
            k = 2 * self.obstacle_dilate_cells + 1
            kernel = np.ones((k, k), np.uint8)
            occupied = cv2.dilate(occupied, kernel, iterations=1)

        if self.free_dilate_cells > 0:
            k = 2 * self.free_dilate_cells + 1
            kernel = np.ones((k, k), np.uint8)
            free = cv2.dilate(free, kernel, iterations=1)

        out = np.full_like(grid, -1)
        out[free > 0] = 0
        out[occupied > 0] = 100

        return out

    def publish_grid(self, grid, header):
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

        self.grid_pub.publish(msg)

    def publish_debug(self, grid, header):
        h, w = grid.shape[:2]

        img = np.zeros((h, w, 3), dtype=np.uint8)

        # BGR colors.
        img[grid == -1] = (80, 80, 80)    # unknown: gray
        img[grid == 0] = (0, 180, 0)      # free: green
        img[grid == 100] = (0, 0, 220)    # occupied: red

        # Make near field appear at the bottom of the image.
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

    def publish_status(self, grid, header):
        total = grid.size

        free = int(np.count_nonzero(grid == 0))
        occupied = int(np.count_nonzero(grid == 100))
        unknown = int(np.count_nonzero(grid == -1))

        h, w = grid.shape[:2]

        def cell_ratio(region, value):
            if region.size == 0:
                return 0.0
            return float(np.count_nonzero(region == value) / region.size)

        def summarize_region(region):
            return {
                "free": cell_ratio(region, 0),
                "occupied": cell_ratio(region, 100),
                "unknown": cell_ratio(region, -1),
            }

        # Near field: closest 40 percent of the grid.
        near_y1 = 0
        near_y2 = max(1, int(h * 0.40))

        # Mid field: 40 to 75 percent of the grid.
        mid_y1 = near_y2
        mid_y2 = max(mid_y1 + 1, int(h * 0.75))

        # Split laterally into left / center / right.
        left_x1 = 0
        left_x2 = int(w * 0.33)

        center_x1 = int(w * 0.33)
        center_x2 = int(w * 0.67)

        right_x1 = int(w * 0.67)
        right_x2 = w

        near_left = grid[near_y1:near_y2, left_x1:left_x2]
        near_center = grid[near_y1:near_y2, center_x1:center_x2]
        near_right = grid[near_y1:near_y2, right_x1:right_x2]

        mid_left = grid[mid_y1:mid_y2, left_x1:left_x2]
        mid_center = grid[mid_y1:mid_y2, center_x1:center_x2]
        mid_right = grid[mid_y1:mid_y2, right_x1:right_x2]

        near = {
            "left": summarize_region(near_left),
            "center": summarize_region(near_center),
            "right": summarize_region(near_right),
        }

        mid = {
            "left": summarize_region(mid_left),
            "center": summarize_region(mid_center),
            "right": summarize_region(mid_right),
        }

        near_center_occupied = near["center"]["occupied"]
        near_center_free = near["center"]["free"]

        # Simple local occupancy recommendation.
        #
        # Logic:
        # - if near center is occupied, pick the freer side or stop;
        # - if near center is free, go forward;
        # - otherwise use mid-field evidence to bias left/right/slow.
        left_score = (
            0.65 * near["left"]["free"]
            + 0.35 * mid["left"]["free"]
            - 0.70 * near["left"]["occupied"]
        )
        center_score = (
            0.65 * near["center"]["free"]
            + 0.35 * mid["center"]["free"]
            - 0.90 * near["center"]["occupied"]
        )
        right_score = (
            0.65 * near["right"]["free"]
            + 0.35 * mid["right"]["free"]
            - 0.70 * near["right"]["occupied"]
        )

        scores = {
            "left": float(left_score),
            "center": float(center_score),
            "right": float(right_score),
        }

        if near_center_occupied >= 0.30:
            if left_score > right_score and left_score > 0.05:
                recommended = "left"
            elif right_score > left_score and right_score > 0.05:
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
            elif center_score > 0.05:
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
                "free": float(free / total),
                "occupied": float(occupied / total),
                "unknown": float(unknown / total),
            },
            "near_field": near,
            "mid_field": mid,
            "scores": scores,
            "recommended_direction": recommended,
            "legacy_ratios": {
                "near_center_free": near_center_free,
                "near_center_occupied": near_center_occupied,
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
