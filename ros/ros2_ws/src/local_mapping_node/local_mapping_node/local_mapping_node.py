#!/usr/bin/env python3
import json
import math
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class LocalMappingNode(Node):
    """
    First accumulated local mapping prototype.

    Inputs:
      /carla/hero_odom
      /perception/local_occupancy_grid
      /perception/local_static_obstacle_grid
      /perception/local_dynamic_obstacle_grid

    Outputs:
      /perception/accumulated_local_map
      /perception/accumulated_static_map
      /perception/accumulated_dynamic_map
      /perception/accumulated_local_map_debug/compressed
      /perception/local_mapping_status

    Map convention:
      -1 unknown
       0 free
     100 occupied

    Debug convention:
      gray   unknown
      green  accumulated free
      red    accumulated static obstacle
      orange current / recent dynamic obstacle
      blue   hero position
    """

    def __init__(self):
        super().__init__("local_mapping_node")

        self.declare_parameter("odom_topic", "/carla/hero_odom")
        self.declare_parameter("local_grid_topic", "/perception/local_occupancy_grid")
        self.declare_parameter("static_grid_topic", "/perception/local_static_obstacle_grid")
        self.declare_parameter("dynamic_grid_topic", "/perception/local_dynamic_obstacle_grid")

        self.declare_parameter("accumulated_map_topic", "/perception/accumulated_local_map")
        self.declare_parameter("accumulated_static_topic", "/perception/accumulated_static_map")
        self.declare_parameter("accumulated_dynamic_topic", "/perception/accumulated_dynamic_map")
        self.declare_parameter("debug_topic", "/perception/accumulated_local_map_debug/compressed")
        self.declare_parameter("status_topic", "/perception/local_mapping_status")

        # Accumulated map parameters.
        self.declare_parameter("map_size_m", 60.0)
        self.declare_parameter("resolution", 0.25)
        self.declare_parameter("frame_id", "carla_world")

        # Local grid interpretation.
        # Must match local_occupancy_node known-good settings.
        self.declare_parameter("local_forward_m", 18.0)
        self.declare_parameter("local_width_m", 10.0)

        # Coordinate convention tuning.
        # These are useful because CARLA/Unreal, ROS, image coordinates and
        # debug-image coordinates do not all use the same handedness.
        self.declare_parameter("yaw_sign", 1.0)
        self.declare_parameter("lateral_sign", 1.0)

        # Update probabilities / log-odds-like increments.
        self.declare_parameter("static_hit_inc", 4.0)
        self.declare_parameter("free_dec", 1.0)
        self.declare_parameter("dynamic_hit_inc", 5.0)
        self.declare_parameter("dynamic_decay", 0.90)

        self.declare_parameter("static_occupied_thresh", 5.0)
        self.declare_parameter("dynamic_occupied_thresh", 3.0)
        self.declare_parameter("free_thresh", -3.0)

        self.declare_parameter("max_abs_score", 20.0)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("pixel_scale", 3)
        self.declare_parameter("jpeg_quality", 75)

        self.odom_topic = self.get_parameter("odom_topic").value
        self.local_grid_topic = self.get_parameter("local_grid_topic").value
        self.static_grid_topic = self.get_parameter("static_grid_topic").value
        self.dynamic_grid_topic = self.get_parameter("dynamic_grid_topic").value

        self.accumulated_map_topic = self.get_parameter("accumulated_map_topic").value
        self.accumulated_static_topic = self.get_parameter("accumulated_static_topic").value
        self.accumulated_dynamic_topic = self.get_parameter("accumulated_dynamic_topic").value
        self.debug_topic = self.get_parameter("debug_topic").value
        self.status_topic = self.get_parameter("status_topic").value

        self.map_size_m = float(self.get_parameter("map_size_m").value)
        self.resolution = float(self.get_parameter("resolution").value)
        self.frame_id = self.get_parameter("frame_id").value

        self.local_forward_m = float(self.get_parameter("local_forward_m").value)
        self.local_width_m = float(self.get_parameter("local_width_m").value)

        self.yaw_sign = float(self.get_parameter("yaw_sign").value)
        self.lateral_sign = float(self.get_parameter("lateral_sign").value)

        self.static_hit_inc = float(self.get_parameter("static_hit_inc").value)
        self.free_dec = float(self.get_parameter("free_dec").value)
        self.dynamic_hit_inc = float(self.get_parameter("dynamic_hit_inc").value)
        self.dynamic_decay = float(self.get_parameter("dynamic_decay").value)

        self.static_occupied_thresh = float(self.get_parameter("static_occupied_thresh").value)
        self.dynamic_occupied_thresh = float(self.get_parameter("dynamic_occupied_thresh").value)
        self.free_thresh = float(self.get_parameter("free_thresh").value)

        self.max_abs_score = float(self.get_parameter("max_abs_score").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.pixel_scale = int(self.get_parameter("pixel_scale").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)

        self.map_cells = int(round(self.map_size_m / self.resolution))

        # Internal score maps.
        self.static_score = np.zeros((self.map_cells, self.map_cells), dtype=np.float32)
        self.free_score = np.zeros((self.map_cells, self.map_cells), dtype=np.float32)
        self.dynamic_score = np.zeros((self.map_cells, self.map_cells), dtype=np.float32)

        self.map_origin_x = None
        self.map_origin_y = None

        self.latest_odom = None
        self.latest_local_grid = None
        self.latest_static_grid = None
        self.latest_dynamic_grid = None

        self.last_update_time = 0.0

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_cb,
            20,
        )

        self.local_grid_sub = self.create_subscription(
            OccupancyGrid,
            self.local_grid_topic,
            self.local_grid_cb,
            10,
        )

        self.static_grid_sub = self.create_subscription(
            OccupancyGrid,
            self.static_grid_topic,
            self.static_grid_cb,
            10,
        )

        self.dynamic_grid_sub = self.create_subscription(
            OccupancyGrid,
            self.dynamic_grid_topic,
            self.dynamic_grid_cb,
            10,
        )

        self.map_pub = self.create_publisher(
            OccupancyGrid,
            self.accumulated_map_topic,
            10,
        )

        self.static_pub = self.create_publisher(
            OccupancyGrid,
            self.accumulated_static_topic,
            10,
        )

        self.dynamic_pub = self.create_publisher(
            OccupancyGrid,
            self.accumulated_dynamic_topic,
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

        timer_period = 1.0 / max(0.1, self.publish_rate_hz)
        self.timer = self.create_timer(timer_period, self.publish_cb)

        self.get_logger().info(f"Odometry input          : {self.odom_topic}")
        self.get_logger().info(f"Local grid input        : {self.local_grid_topic}")
        self.get_logger().info(f"Static grid input       : {self.static_grid_topic}")
        self.get_logger().info(f"Dynamic grid input      : {self.dynamic_grid_topic}")
        self.get_logger().info(f"Accumulated map output  : {self.accumulated_map_topic}")
        self.get_logger().info(f"Debug output            : {self.debug_topic}")
        self.get_logger().info(f"Map size                : {self.map_size_m} m x {self.map_size_m} m")
        self.get_logger().info(f"Resolution              : {self.resolution} m/cell")
        self.get_logger().info("Using CARLA hero odometry as ground-truth motion.")

    def odom_cb(self, msg: Odometry):
        self.latest_odom = msg

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.map_origin_x is None:
            self.map_origin_x = float(x) - self.map_size_m / 2.0
            self.map_origin_y = float(y) - self.map_size_m / 2.0
            self.get_logger().info(
                f"Initialized accumulated map origin at "
                f"({self.map_origin_x:.2f}, {self.map_origin_y:.2f})"
            )

    def local_grid_cb(self, msg: OccupancyGrid):
        self.latest_local_grid = msg
        self.try_integrate()

    def static_grid_cb(self, msg: OccupancyGrid):
        self.latest_static_grid = msg
        self.try_integrate()

    def dynamic_grid_cb(self, msg: OccupancyGrid):
        self.latest_dynamic_grid = msg
        self.try_integrate()

    def quaternion_to_yaw(self, q):
        # ROS quaternion to yaw.
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def grid_to_array(self, grid_msg: OccupancyGrid):
        data = np.array(grid_msg.data, dtype=np.int16)
        return data.reshape((grid_msg.info.height, grid_msg.info.width))

    def try_integrate(self):
        if self.latest_odom is None:
            return

        if self.latest_local_grid is None:
            return

        if self.latest_static_grid is None:
            return

        if self.latest_dynamic_grid is None:
            return

        if self.map_origin_x is None or self.map_origin_y is None:
            return

        local = self.grid_to_array(self.latest_local_grid)
        static = self.grid_to_array(self.latest_static_grid)
        dynamic = self.grid_to_array(self.latest_dynamic_grid)

        self.integrate_local_maps(local, static, dynamic, self.latest_odom)
        self.last_update_time = time.time()

    def integrate_local_maps(self, local, static, dynamic, odom):
        hero_x = float(odom.pose.pose.position.x)
        hero_y = float(odom.pose.pose.position.y)
        yaw = self.yaw_sign * self.quaternion_to_yaw(odom.pose.pose.orientation)

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        h, w = local.shape[:2]

        # These should match local_occupancy_node dimensions, but infer from grid size.
        local_res = float(self.latest_local_grid.info.resolution)
        local_width_m = float(w) * local_res

        # Decay dynamic layer every integration.
        self.dynamic_score *= self.dynamic_decay

        for gy in range(h):
            # Local occupancy grid row 0 corresponds to near field.
            forward = (float(gy) + 0.5) * local_res

            if forward < 0.0 or forward > self.local_forward_m:
                continue

            for gx in range(w):
                value = int(local[gy, gx])

                if value == -1:
                    continue

                # Local occupancy grid lateral coordinate comes from image columns.
                # lateral_sign allows quick correction of left/right convention.
                lateral = self.lateral_sign * ((float(gx) + 0.5) * local_res - local_width_m / 2.0)

                world_x = hero_x + forward * cos_yaw - lateral * sin_yaw
                world_y = hero_y + forward * sin_yaw + lateral * cos_yaw

                mx, my = self.world_to_map(world_x, world_y)
                if mx is None:
                    continue

                is_static = int(static[gy, gx]) == 100
                is_dynamic = int(dynamic[gy, gx]) == 100
                is_free = value == 0

                if is_static:
                    self.static_score[my, mx] += self.static_hit_inc

                if is_dynamic:
                    self.dynamic_score[my, mx] += self.dynamic_hit_inc

                if is_free:
                    # Free observations reduce static confidence cautiously.
                    self.free_score[my, mx] -= self.free_dec
                    self.static_score[my, mx] -= 0.25 * self.free_dec

        self.static_score = np.clip(self.static_score, -self.max_abs_score, self.max_abs_score)
        self.free_score = np.clip(self.free_score, -self.max_abs_score, self.max_abs_score)
        self.dynamic_score = np.clip(self.dynamic_score, 0.0, self.max_abs_score)

    def world_to_map(self, x, y):
        mx = int((x - self.map_origin_x) / self.resolution)
        my = int((y - self.map_origin_y) / self.resolution)

        if mx < 0 or mx >= self.map_cells:
            return None, None

        if my < 0 or my >= self.map_cells:
            return None, None

        return mx, my

    def map_to_occupancy(self):
        combined = np.full((self.map_cells, self.map_cells), -1, dtype=np.int16)
        static_grid = np.full_like(combined, -1)
        dynamic_grid = np.full_like(combined, -1)

        free_mask = self.free_score <= self.free_thresh
        static_mask = self.static_score >= self.static_occupied_thresh
        dynamic_mask = self.dynamic_score >= self.dynamic_occupied_thresh

        combined[free_mask] = 0
        combined[static_mask] = 100
        combined[dynamic_mask] = 100

        static_grid[free_mask] = 0
        static_grid[static_mask] = 100

        dynamic_grid[free_mask] = 0
        dynamic_grid[dynamic_mask] = 100

        return combined, static_grid, dynamic_grid

    def publish_cb(self):
        if self.map_origin_x is None or self.latest_odom is None:
            return

        combined, static_grid, dynamic_grid = self.map_to_occupancy()

        stamp = self.get_clock().now().to_msg()

        self.publish_grid(combined, self.map_pub, stamp)
        self.publish_grid(static_grid, self.static_pub, stamp)
        self.publish_grid(dynamic_grid, self.dynamic_pub, stamp)
        self.publish_debug(combined, static_grid, dynamic_grid, stamp)
        self.publish_status(combined, static_grid, dynamic_grid, stamp)

    def publish_grid(self, arr, pub, stamp):
        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id

        msg.info.resolution = float(self.resolution)
        msg.info.width = int(self.map_cells)
        msg.info.height = int(self.map_cells)

        msg.info.origin.position.x = float(self.map_origin_x)
        msg.info.origin.position.y = float(self.map_origin_y)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = arr.astype(np.int8).reshape(-1).tolist()
        pub.publish(msg)

    def publish_debug(self, combined, static_grid, dynamic_grid, stamp):
        img = np.zeros((self.map_cells, self.map_cells, 3), dtype=np.uint8)

        img[combined == -1] = (80, 80, 80)      # unknown gray
        img[combined == 0] = (0, 180, 0)        # free green
        img[static_grid == 100] = (0, 0, 220)   # static red
        img[dynamic_grid == 100] = (0, 165, 255)  # dynamic orange

        # Draw hero position.
        if self.latest_odom is not None:
            hx = float(self.latest_odom.pose.pose.position.x)
            hy = float(self.latest_odom.pose.pose.position.y)
            mx, my = self.world_to_map(hx, hy)
            if mx is not None:
                cv2.circle(img, (mx, my), 3, (255, 0, 0), -1)

                yaw = self.quaternion_to_yaw(self.latest_odom.pose.pose.orientation)
                end_x = int(mx + 12 * math.cos(yaw))
                end_y = int(my + 12 * math.sin(yaw))
                cv2.arrowedLine(img, (mx, my), (end_x, end_y), (255, 0, 0), 2, tipLength=0.3)

        # Display convention:
        # CARLA world Y already has the desired visual orientation for this debug view.
        # Do not flip vertically here, otherwise left/right turns appear inverted.
        # img = cv2.flip(img, 0)

        scale = max(1, self.pixel_scale)
        img = cv2.resize(
            img,
            (self.map_cells * scale, self.map_cells * scale),
            interpolation=cv2.INTER_NEAREST,
        )

        ok, jpg = cv2.imencode(
            ".jpg",
            img,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )

        if not ok:
            return

        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.format = "jpeg"
        msg.data = jpg.tobytes()

        self.debug_pub.publish(msg)

    def publish_status(self, combined, static_grid, dynamic_grid, stamp):
        total = combined.size

        status = {
            "stamp": {
                "sec": int(stamp.sec),
                "nanosec": int(stamp.nanosec),
            },
            "frame_id": self.frame_id,
            "map": {
                "size_m": self.map_size_m,
                "resolution": self.resolution,
                "cells": self.map_cells,
                "origin_x": self.map_origin_x,
                "origin_y": self.map_origin_y,
                "yaw_sign": self.yaw_sign,
                "lateral_sign": self.lateral_sign,
            },
            "ratios": {
                "free": float(np.count_nonzero(combined == 0) / total),
                "occupied": float(np.count_nonzero(combined == 100) / total),
                "static": float(np.count_nonzero(static_grid == 100) / total),
                "dynamic": float(np.count_nonzero(dynamic_grid == 100) / total),
                "unknown": float(np.count_nonzero(combined == -1) / total),
            },
            "last_update_age_sec": float(time.time() - self.last_update_time) if self.last_update_time > 0 else None,
        }

        msg = String()
        msg.data = json.dumps(status, indent=2)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalMappingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
