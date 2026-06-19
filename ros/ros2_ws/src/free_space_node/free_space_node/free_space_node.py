#!/usr/bin/env python3
import json
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge


class FreeSpaceNode(Node):
    """
    Semantic-depth free-space estimator.

    Inputs:
      /perception/semantic_mask     mono8 Cityscapes19 class IDs
      /perception/depth/image       32FC1 relative depth from Depth Anything

    Outputs:
      /perception/free_space_mask/compressed
      /perception/obstacle_mask/compressed
      /perception/free_space_status
    """

    def __init__(self):
        super().__init__("free_space_node")

        self.bridge = CvBridge()

        self.declare_parameter("semantic_topic", "/perception/semantic_mask")
        self.declare_parameter("depth_topic", "/perception/depth/image")
        self.declare_parameter("free_mask_topic", "/perception/free_space_mask/compressed")
        self.declare_parameter("obstacle_mask_topic", "/perception/obstacle_mask/compressed")
        self.declare_parameter("status_topic", "/perception/free_space_status")

        # Depth Anything convention in your current system:
        # larger value seems closer.
        self.declare_parameter("depth_closer_is_larger", True)

        # Depth threshold used to mark close unknown structures as obstacles.
        self.declare_parameter("close_depth_thresh", 40.0)

        # Region of interest: avoid using bottom-most road pixels too strongly.
        self.declare_parameter("roi_x_min_ratio", 0.20)
        self.declare_parameter("roi_x_max_ratio", 0.80)
        self.declare_parameter("roi_y_min_ratio", 0.35)
        self.declare_parameter("roi_y_max_ratio", 0.85)

        # Decision thresholds.
        self.declare_parameter("center_obstacle_ratio_stop", 0.25)
        self.declare_parameter("center_free_ratio_clear", 0.55)

        self.declare_parameter("jpeg_quality", 65)

        self.semantic_topic = self.get_parameter("semantic_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.free_mask_topic = self.get_parameter("free_mask_topic").value
        self.obstacle_mask_topic = self.get_parameter("obstacle_mask_topic").value
        self.status_topic = self.get_parameter("status_topic").value

        self.depth_closer_is_larger = bool(self.get_parameter("depth_closer_is_larger").value)
        self.close_depth_thresh = float(self.get_parameter("close_depth_thresh").value)

        self.roi_x_min_ratio = float(self.get_parameter("roi_x_min_ratio").value)
        self.roi_x_max_ratio = float(self.get_parameter("roi_x_max_ratio").value)
        self.roi_y_min_ratio = float(self.get_parameter("roi_y_min_ratio").value)
        self.roi_y_max_ratio = float(self.get_parameter("roi_y_max_ratio").value)

        self.center_obstacle_ratio_stop = float(self.get_parameter("center_obstacle_ratio_stop").value)
        self.center_free_ratio_clear = float(self.get_parameter("center_free_ratio_clear").value)

        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)

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

        self.free_pub = self.create_publisher(
            CompressedImage,
            self.free_mask_topic,
            10,
        )

        self.obstacle_pub = self.create_publisher(
            CompressedImage,
            self.obstacle_mask_topic,
            10,
        )

        self.status_pub = self.create_publisher(
            String,
            self.status_topic,
            10,
        )

        self.get_logger().info(f"Semantic input : {self.semantic_topic}")
        self.get_logger().info(f"Depth input    : {self.depth_topic}")
        self.get_logger().info(f"Free mask out  : {self.free_mask_topic}")
        self.get_logger().info(f"Obstacle out   : {self.obstacle_mask_topic}")
        self.get_logger().info(f"Status out     : {self.status_topic}")

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

        H, W = sem.shape[:2]

        # Cityscapes19 IDs:
        # 0 road, 1 sidewalk, 2 building, 3 wall, 4 fence, 5 pole,
        # 6 traffic light, 7 traffic sign, 8 vegetation, 9 terrain,
        # 10 sky, 11 person, 12 rider, 13 car, 14 truck, 15 bus,
        # 16 train, 17 motorcycle, 18 bicycle

        road_ids = {0}
        optional_free_ids = set()  # keep sidewalk out for now

        obstacle_ids = {
            1,   # sidewalk
            #2,   # building
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

        free_mask = np.isin(sem, list(road_ids | optional_free_ids))
        semantic_obstacle_mask = np.isin(sem, list(obstacle_ids))

        depth_obstacle_mask = np.zeros_like(free_mask, dtype=bool)

        if self.latest_depth is not None:
            depth = self.latest_depth

            if depth.shape[:2] != sem.shape[:2]:
                depth = cv2.resize(depth, (W, H), interpolation=cv2.INTER_LINEAR)

            valid = np.isfinite(depth)

            if self.depth_closer_is_larger:
                close_depth = valid & (depth >= self.close_depth_thresh)
            else:
                close_depth = valid & (depth <= self.close_depth_thresh)

            # Do not let the immediate road dominate. Only use depth obstacle
            # inside a forward-looking ROI.
            x1 = int(W * self.roi_x_min_ratio)
            x2 = int(W * self.roi_x_max_ratio)
            y1 = int(H * self.roi_y_min_ratio)
            y2 = int(H * self.roi_y_max_ratio)

            roi_gate = np.zeros_like(free_mask, dtype=bool)
            roi_gate[y1:y2, x1:x2] = True

            # Depth says close AND semantic is not road.
            # This avoids interpreting the road itself as an obstacle.
            depth_obstacle_mask = close_depth & roi_gate & ~free_mask

        obstacle_mask = semantic_obstacle_mask | depth_obstacle_mask

        # Free space is road-like semantic class, excluding obstacle pixels.
        final_free_mask = free_mask & ~obstacle_mask

        status = self.compute_status(final_free_mask, obstacle_mask, msg.header)
        self.publish_masks(final_free_mask, obstacle_mask, msg.header)
        self.publish_status(status)

    def compute_status(self, free_mask, obstacle_mask, header):
        H, W = free_mask.shape[:2]

        x1 = int(W * self.roi_x_min_ratio)
        x2 = int(W * self.roi_x_max_ratio)
        y1 = int(H * self.roi_y_min_ratio)
        y2 = int(H * self.roi_y_max_ratio)

        roi_free = free_mask[y1:y2, x1:x2]
        roi_obstacle = obstacle_mask[y1:y2, x1:x2]

        width = roi_free.shape[1]
        third = max(width // 3, 1)

        left_free = roi_free[:, :third]
        center_free = roi_free[:, third:2 * third]
        right_free = roi_free[:, 2 * third:]

        left_obst = roi_obstacle[:, :third]
        center_obst = roi_obstacle[:, third:2 * third]
        right_obst = roi_obstacle[:, 2 * third:]

        def ratio(mask):
            if mask.size == 0:
                return 0.0
            return float(np.count_nonzero(mask) / mask.size)

        left_free_ratio = ratio(left_free)
        center_free_ratio = ratio(center_free)
        right_free_ratio = ratio(right_free)

        left_obstacle_ratio = ratio(left_obst)
        center_obstacle_ratio = ratio(center_obst)
        right_obstacle_ratio = ratio(right_obst)

        if center_obstacle_ratio >= self.center_obstacle_ratio_stop:
            if left_free_ratio > right_free_ratio:
                recommended = "left"
            elif right_free_ratio > left_free_ratio:
                recommended = "right"
            else:
                recommended = "stop"
        elif center_free_ratio >= self.center_free_ratio_clear:
            recommended = "forward"
        else:
            if left_free_ratio > right_free_ratio:
                recommended = "left"
            elif right_free_ratio > left_free_ratio:
                recommended = "right"
            else:
                recommended = "slow"

        return {
            "stamp": {
                "sec": int(header.stamp.sec),
                "nanosec": int(header.stamp.nanosec),
            },
            "frame_id": header.frame_id,
            "roi": {
                "x_min_ratio": self.roi_x_min_ratio,
                "x_max_ratio": self.roi_x_max_ratio,
                "y_min_ratio": self.roi_y_min_ratio,
                "y_max_ratio": self.roi_y_max_ratio,
            },
            "free_ratio": {
                "left": left_free_ratio,
                "center": center_free_ratio,
                "right": right_free_ratio,
            },
            "obstacle_ratio": {
                "left": left_obstacle_ratio,
                "center": center_obstacle_ratio,
                "right": right_obstacle_ratio,
            },
            "recommended_direction": recommended,
        }

    def publish_status(self, status):
        msg = String()
        msg.data = json.dumps(status, indent=2)
        self.status_pub.publish(msg)

    def publish_masks(self, free_mask, obstacle_mask, header):
        free_vis = np.zeros((*free_mask.shape, 3), dtype=np.uint8)
        obst_vis = np.zeros((*obstacle_mask.shape, 3), dtype=np.uint8)

        # green free space
        free_vis[free_mask] = (0, 255, 0)

        # red obstacles
        obst_vis[obstacle_mask] = (0, 0, 255)

        self.publish_compressed(free_vis, header, self.free_pub)
        self.publish_compressed(obst_vis, header, self.obstacle_pub)

    def publish_compressed(self, bgr, header, pub):
        ok, jpg = cv2.imencode(
            ".jpg",
            bgr,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )

        if not ok:
            return

        msg = CompressedImage()
        msg.header = header
        msg.format = "jpeg"
        msg.data = jpg.tobytes()
        pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FreeSpaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
