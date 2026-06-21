#!/usr/bin/env python3

import json
import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class LaneGuidanceNode(Node):
    """
    Skeleton lane guidance node.

    Produces a recommendation from lane detection + free-space/local occupancy
    status, but does not control the vehicle.

    Output:
      /navigation/lane_guidance_status
    """

    def __init__(self):
        super().__init__("lane_guidance_node")

        self.declare_parameter("lane_status_topic", "/perception/lane_status")
        self.declare_parameter("free_space_status_topic", "/perception/free_space_status")
        self.declare_parameter("local_occupancy_status_topic", "/perception/local_occupancy_status")
        self.declare_parameter("lane_guidance_status_topic", "/navigation/lane_guidance_status")

        self.declare_parameter("min_lane_confidence", 0.20)
        self.declare_parameter("kp_offset", 0.003)
        self.declare_parameter("kp_heading", 0.015)
        self.declare_parameter("steering_sign", 1.0)
        self.declare_parameter("max_abs_steer", 0.50)
        self.declare_parameter("lane_follow_speed", 0.30)
        self.declare_parameter("fallback_speed", 0.18)
        self.declare_parameter("publish_rate_hz", 5.0)

        self.latest_lane_status: Optional[dict] = None
        self.latest_free_space_status: Optional[dict] = None
        self.latest_local_occupancy_status: Optional[dict] = None

        self.create_subscription(
            String,
            self.get_parameter("lane_status_topic").value,
            self.lane_status_callback,
            10,
        )

        self.create_subscription(
            String,
            self.get_parameter("free_space_status_topic").value,
            self.free_space_status_callback,
            10,
        )

        self.create_subscription(
            String,
            self.get_parameter("local_occupancy_status_topic").value,
            self.local_occupancy_status_callback,
            10,
        )

        self.guidance_pub = self.create_publisher(
            String,
            self.get_parameter("lane_guidance_status_topic").value,
            10,
        )

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / max(rate, 0.1), self.publish_guidance)

        self.get_logger().info("Lane guidance node started")
        self.get_logger().info("This node publishes recommendations only; it does not publish /carla/cmd_vel")

    def parse_json_msg(self, msg: String, source: str):
        try:
            return json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f"Invalid JSON from {source}: {exc}")
            return None

    def lane_status_callback(self, msg: String) -> None:
        self.latest_lane_status = self.parse_json_msg(msg, "lane_status")

    def free_space_status_callback(self, msg: String) -> None:
        self.latest_free_space_status = self.parse_json_msg(msg, "free_space_status")

    def local_occupancy_status_callback(self, msg: String) -> None:
        self.latest_local_occupancy_status = self.parse_json_msg(msg, "local_occupancy_status")

    def publish_guidance(self) -> None:
        min_conf = float(self.get_parameter("min_lane_confidence").value)

        if self.latest_lane_status is None:
            guidance = {
                "stamp": time.time(),
                "mode": "waiting_for_lane_status",
                "recommended_steer": 0.0,
                "recommended_speed": 0.0,
                "confidence": 0.0,
                "safe_to_use_for_control": False,
                "reason": "no lane_status received yet",
            }
            self.guidance_pub.publish(String(data=json.dumps(guidance)))
            return

        lane_detected = bool(self.latest_lane_status.get("lane_detected", False))
        confidence = float(self.latest_lane_status.get("confidence") or 0.0)

        if not lane_detected or confidence < min_conf:
            guidance = {
                "stamp": time.time(),
                "mode": "fallback_no_lane",
                "recommended_steer": 0.0,
                "recommended_speed": float(self.get_parameter("fallback_speed").value),
                "confidence": confidence,
                "safe_to_use_for_control": False,
                "reason": "lane not detected or confidence too low",
                "lane_status": self.latest_lane_status,
            }
            self.guidance_pub.publish(String(data=json.dumps(guidance)))
            return

        center_offset_px = float(self.latest_lane_status.get("center_offset_px") or 0.0)
        heading_error_deg = float(self.latest_lane_status.get("heading_error_deg") or 0.0)

        kp_offset = float(self.get_parameter("kp_offset").value)
        kp_heading = float(self.get_parameter("kp_heading").value)
        steering_sign = float(self.get_parameter("steering_sign").value)
        max_abs_steer = float(self.get_parameter("max_abs_steer").value)

        raw_steer = steering_sign * (kp_offset * center_offset_px + kp_heading * heading_error_deg)
        recommended_steer = float(np.clip(-raw_steer, -max_abs_steer, max_abs_steer))

        guidance = {
            "stamp": time.time(),
            "mode": "lane_following_recommendation",
            "recommended_steer": round(recommended_steer, 4),
            "recommended_speed": float(self.get_parameter("lane_follow_speed").value),
            "confidence": confidence,
            "safe_to_use_for_control": False,
            "reason": "recommendation only; not connected to cmd_vel",
            "center_offset_px": center_offset_px,
            "heading_error_deg": heading_error_deg,
            "has_free_space_status": self.latest_free_space_status is not None,
            "has_local_occupancy_status": self.latest_local_occupancy_status is not None,
        }

        self.guidance_pub.publish(String(data=json.dumps(guidance)))


def main(args=None):
    rclpy.init(args=args)
    node = LaneGuidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
