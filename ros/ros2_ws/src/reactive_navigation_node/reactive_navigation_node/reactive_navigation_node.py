#!/usr/bin/env python3
import json
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

import numpy as np


class ReactiveNavigationNode(Node):
    def __init__(self):
        super().__init__("reactive_navigation_node")

        self.bridge = CvBridge()

        self.declare_parameter("fused_topic", "/perception/fused_objects")
        self.declare_parameter("depth_topic", "/perception/depth/image")
        self.declare_parameter("cmd_topic", "/carla/cmd_vel")

        # Normalized command values expected by bridge
        self.declare_parameter("cruise_speed", 0.25)
        self.declare_parameter("slow_speed", 0.12)
        self.declare_parameter("reverse_speed", -0.22)

        self.declare_parameter("avoid_steer", 0.65)
        self.declare_parameter("max_steer", 0.75)

        # Depth Anything convention in your current output:
        # larger value seems closer.
        self.declare_parameter("depth_closer_is_larger", True)

        # Fused-object thresholds
        self.declare_parameter("fused_close_thresh", 150.0)
        self.declare_parameter("fused_danger_thresh", 210.0)

        # Raw depth ROI thresholds
        self.declare_parameter("roi_close_thresh", 150.0)
        self.declare_parameter("roi_danger_thresh", 210.0)

        # Image geometry / ROI
        self.declare_parameter("center_band", 0.35)
        self.declare_parameter("roi_y_min_ratio", 0.45)
        self.declare_parameter("roi_y_max_ratio", 0.95)
        self.declare_parameter("roi_x_min_ratio", 0.25)
        self.declare_parameter("roi_x_max_ratio", 0.75)

        # State-machine durations
        self.declare_parameter("reverse_duration_sec", 1.0)
        self.declare_parameter("recovery_turn_duration_sec", 0.8)

        self.declare_parameter("timeout_sec", 1.0)
        self.declare_parameter("stop_on_timeout", True)

        self.fused_topic = self.get_parameter("fused_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.cmd_topic = self.get_parameter("cmd_topic").value

        self.cruise_speed = float(self.get_parameter("cruise_speed").value)
        self.slow_speed = float(self.get_parameter("slow_speed").value)
        self.reverse_speed = float(self.get_parameter("reverse_speed").value)

        self.avoid_steer = float(self.get_parameter("avoid_steer").value)
        self.max_steer = float(self.get_parameter("max_steer").value)

        self.depth_closer_is_larger = bool(self.get_parameter("depth_closer_is_larger").value)

        self.fused_close_thresh = float(self.get_parameter("fused_close_thresh").value)
        self.fused_danger_thresh = float(self.get_parameter("fused_danger_thresh").value)

        self.roi_close_thresh = float(self.get_parameter("roi_close_thresh").value)
        self.roi_danger_thresh = float(self.get_parameter("roi_danger_thresh").value)

        self.center_band = float(self.get_parameter("center_band").value)
        self.roi_y_min_ratio = float(self.get_parameter("roi_y_min_ratio").value)
        self.roi_y_max_ratio = float(self.get_parameter("roi_y_max_ratio").value)
        self.roi_x_min_ratio = float(self.get_parameter("roi_x_min_ratio").value)
        self.roi_x_max_ratio = float(self.get_parameter("roi_x_max_ratio").value)

        self.reverse_duration_sec = float(self.get_parameter("reverse_duration_sec").value)
        self.recovery_turn_duration_sec = float(self.get_parameter("recovery_turn_duration_sec").value)

        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.stop_on_timeout = bool(self.get_parameter("stop_on_timeout").value)

        self.latest_depth = None
        self.latest_depth_time = 0.0
        self.last_fused_time = 0.0

        self.mode = "CRUISE"
        self.mode_until = 0.0
        self.recovery_until = 0.0
        self.avoid_direction = 1.0

        self.fused_sub = self.create_subscription(
            String,
            self.fused_topic,
            self.fused_cb,
            10,
        )

        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_cb,
            10,
        )

        self.pub = self.create_publisher(
            Twist,
            self.cmd_topic,
            10,
        )

        self.timer = self.create_timer(0.1, self.timer_cb)

        self.get_logger().info(f"Listening fused objects: {self.fused_topic}")
        self.get_logger().info(f"Listening depth image  : {self.depth_topic}")
        self.get_logger().info(f"Publishing commands   : {self.cmd_topic}")
        self.get_logger().info("Behavior: fused-object avoidance + raw depth ROI emergency fallback.")

    def clamp(self, value, lo, hi):
        return max(lo, min(hi, value))

    def publish_cmd(self, speed, steer, reason=""):
        cmd = Twist()
        cmd.linear.x = float(self.clamp(speed, -1.0, 1.0))
        cmd.angular.z = float(self.clamp(steer, -self.max_steer, self.max_steer))
        self.pub.publish(cmd)

        if reason:
            self.get_logger().info(
                f"{reason} | speed={cmd.linear.x:+.2f}, steer={cmd.angular.z:+.2f}",
                throttle_duration_sec=0.5,
            )

    def is_close(self, depth_value, threshold):
        if self.depth_closer_is_larger:
            return depth_value >= threshold
        return depth_value <= threshold

    def closer_key(self, item):
        # item = (obj, depth, x_offset)
        depth = item[1]
        return -depth if self.depth_closer_is_larger else depth

    def depth_cb(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            self.latest_depth = depth.astype(np.float32)
            self.latest_depth_time = time.time()
        except Exception as e:
            self.get_logger().warn(f"Could not read depth image: {e}")

    def parse_objects(self, msg):
        try:
            data = json.loads(msg.data)
            return data.get("objects", [])
        except Exception as e:
            self.get_logger().warn(f"Could not parse fused JSON: {e}")
            return []

    def get_object_depth(self, obj):
        if "depth_p10" in obj:
            return float(obj["depth_p10"])
        if "depth_min" in obj:
            return float(obj["depth_min"])
        if "depth_median" in obj:
            return float(obj["depth_median"])
        return None

    def get_object_center_offset(self, obj):
        bbox = obj.get("bbox", {})
        cx = float(bbox.get("cx", 400.0))

        # Prefer image width from latest depth
        if self.latest_depth is not None:
            image_width = float(self.latest_depth.shape[1])
        else:
            image_width = 800.0

        x_norm = (cx - image_width / 2.0) / (image_width / 2.0)
        return self.clamp(x_norm, -1.0, 1.0)

    def is_obstacle_class(self, obj):
        label = obj.get("class", "")

        obstacle_classes = {
            "wall",
            "fence",
            "pole",
            "building",   # useful for close structures/overhangs if detected
            "person",
            "rider",
            "car",
            "truck",
            "bus",
            "train",
            "motorcycle",
            "bicycle",
        }

        return label in obstacle_classes

    def choose_avoid_direction_from_x(self, x_offset):
        # Obstacle left -> turn right. Obstacle right -> turn left.
        # If turn direction is wrong in CARLA, multiply output by -1.
        if abs(x_offset) < 0.08:
            return self.avoid_direction
        return 1.0 if x_offset < 0.0 else -1.0

    def enter_reverse_mode(self, reason, x_offset=0.0):
        now = time.time()
        self.avoid_direction = self.choose_avoid_direction_from_x(x_offset)
        self.mode = "REVERSING"
        self.mode_until = now + self.reverse_duration_sec
        self.recovery_until = self.mode_until + self.recovery_turn_duration_sec

        self.get_logger().warn(
            f"ENTER REVERSING: {reason}, x={x_offset:+.2f}, dir={self.avoid_direction:+.1f}"
        )

    def get_depth_roi_signal(self):
        """
        Emergency fallback based directly on the depth image.

        Looks at lower-center image region. This catches walls/structures even if
        object detection does not publish them as fused objects.
        """
        if self.latest_depth is None:
            return None

        depth = self.latest_depth
        H, W = depth.shape[:2]

        x1 = int(W * self.roi_x_min_ratio)
        x2 = int(W * self.roi_x_max_ratio)
        y1 = int(H * self.roi_y_min_ratio)
        y2 = int(H * self.roi_y_max_ratio)

        roi = depth[y1:y2, x1:x2]
        valid = roi[np.isfinite(roi)]

        if valid.size < 50:
            return None

        # Conservative proximity signal.
        # If larger=closer, use 90th percentile.
        # If smaller=closer, use 10th percentile.
        if self.depth_closer_is_larger:
            proximity = float(np.percentile(valid, 90))
        else:
            proximity = float(np.percentile(valid, 10))

        # Direction: compare left and right halves of the ROI.
        mid = roi.shape[1] // 2
        left = roi[:, :mid]
        right = roi[:, mid:]

        left_valid = left[np.isfinite(left)]
        right_valid = right[np.isfinite(right)]

        if left_valid.size < 20 or right_valid.size < 20:
            x_offset = 0.0
        else:
            if self.depth_closer_is_larger:
                left_signal = float(np.percentile(left_valid, 90))
                right_signal = float(np.percentile(right_valid, 90))
            else:
                left_signal = float(np.percentile(left_valid, 10))
                right_signal = float(np.percentile(right_valid, 10))

            # If left is closer, obstacle is left -> x_offset negative.
            if self.depth_closer_is_larger:
                diff = right_signal - left_signal
            else:
                diff = left_signal - right_signal

            x_offset = self.clamp(diff / 80.0, -1.0, 1.0)

        return proximity, x_offset

    def timer_cb(self):
        now = time.time()

        if self.mode == "REVERSING":
            if now < self.mode_until:
                self.publish_cmd(
                    self.reverse_speed,
                    -self.avoid_direction * self.avoid_steer,
                    reason="REVERSING_AND_TURNING",
                )
                return
            self.mode = "RECOVERY_TURN"

        if self.mode == "RECOVERY_TURN":
            if now < self.recovery_until:
                self.publish_cmd(
                    self.slow_speed,
                    self.avoid_direction * self.avoid_steer,
                    reason="RECOVERY_TURN",
                )
                return
            self.mode = "CRUISE"

        # Depth ROI emergency fallback
        roi_signal = self.get_depth_roi_signal()
        if roi_signal is not None:
            proximity, x_offset = roi_signal

            if self.is_close(proximity, self.roi_danger_thresh):
                self.enter_reverse_mode(
                    f"DEPTH_ROI danger proximity={proximity:.1f}",
                    x_offset=x_offset,
                )
                return

            if self.is_close(proximity, self.roi_close_thresh):
                steer = self.choose_avoid_direction_from_x(x_offset) * self.avoid_steer
                self.publish_cmd(
                    self.slow_speed,
                    steer,
                    reason=f"DEPTH_ROI avoid proximity={proximity:.1f}",
                )
                return

        # Watchdog
        if self.last_fused_time > 0.0:
            elapsed = now - self.last_fused_time
            if elapsed > self.timeout_sec and self.stop_on_timeout:
                self.publish_cmd(0.0, 0.0, reason=f"TIMEOUT no fused objects for {elapsed:.1f}s")
                return

    def fused_cb(self, msg):
        self.last_fused_time = time.time()

        if self.mode in ("REVERSING", "RECOVERY_TURN"):
            return

        objects = self.parse_objects(msg)

        relevant = []
        for obj in objects:
            if not self.is_obstacle_class(obj):
                continue

            depth = self.get_object_depth(obj)
            if depth is None:
                continue

            x_offset = self.get_object_center_offset(obj)
            relevant.append((obj, depth, x_offset))

        if not relevant:
            # Timer depth ROI may override this if needed.
            self.publish_cmd(self.cruise_speed, 0.0, reason="CLEAR_FUSED")
            return

        closest_obj, closest_depth, closest_x = sorted(relevant, key=self.closer_key)[0]

        label = closest_obj.get("class", "unknown")
        track_id = closest_obj.get("track_id", -1)
        centered = abs(closest_x) <= self.center_band

        if centered and self.is_close(closest_depth, self.fused_danger_thresh):
            self.enter_reverse_mode(
                f"FUSED danger {label}#{track_id} depth={closest_depth:.1f}",
                x_offset=closest_x,
            )
            return

        if self.is_close(closest_depth, self.fused_close_thresh):
            steer = self.choose_avoid_direction_from_x(closest_x) * self.avoid_steer
            speed = self.slow_speed if centered else self.cruise_speed

            self.publish_cmd(
                speed,
                steer,
                reason=f"FUSED avoid {label}#{track_id} depth={closest_depth:.1f}, x={closest_x:+.2f}",
            )
            return

        self.publish_cmd(
            self.cruise_speed,
            0.0,
            reason=f"CRUISE closest {label}#{track_id} depth={closest_depth:.1f}",
        )


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
