#!/usr/bin/env python3

import json
import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry


class LaneProjectionNode(Node):
    """
    Skeleton lane projection node.

    Converts image-space lane status into an approximate vehicle-frame lane geometry.

    This is not a calibrated camera projection yet. It creates a simple geometric
    estimate from center offset and heading error so that downstream mapping and
    guidance interfaces can be developed.
    """

    def __init__(self):
        super().__init__("lane_projection_node")

        self.declare_parameter("lane_status_topic", "/perception/lane_status")
        self.declare_parameter("hero_odom_topic", "/carla/hero_odom")
        self.declare_parameter("lane_geometry_topic", "/perception/lane_geometry")
        self.declare_parameter("lane_projection_status_topic", "/perception/lane_projection_status")

        self.declare_parameter("frame_id", "hero")
        self.declare_parameter("px_to_lateral_m", 0.01)
        self.declare_parameter("forward_samples_m", "2.0,5.0,10.0,15.0")
        self.declare_parameter("min_confidence", 0.05)
        self.declare_parameter("lateral_sign", 1.0)

        self.latest_odom: Optional[Odometry] = None
        self.received_status_count = 0
        self.published_geometry_count = 0

        lane_status_topic = self.get_parameter("lane_status_topic").value
        hero_odom_topic = self.get_parameter("hero_odom_topic").value
        lane_geometry_topic = self.get_parameter("lane_geometry_topic").value
        lane_projection_status_topic = self.get_parameter("lane_projection_status_topic").value

        self.create_subscription(String, lane_status_topic, self.lane_status_callback, 10)
        self.create_subscription(Odometry, hero_odom_topic, self.odom_callback, 10)

        self.geometry_pub = self.create_publisher(String, lane_geometry_topic, 10)
        self.status_pub = self.create_publisher(String, lane_projection_status_topic, 10)

        self.get_logger().info("Lane projection node started")
        self.get_logger().info(f"Input lane status: {lane_status_topic}")
        self.get_logger().info(f"Output lane geometry: {lane_geometry_topic}")

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def parse_forward_samples(self):
        raw = str(self.get_parameter("forward_samples_m").value)
        samples = []
        for item in raw.split(","):
            item = item.strip()
            if not item:
                continue
            try:
                samples.append(float(item))
            except ValueError:
                pass
        return samples if samples else [2.0, 5.0, 10.0, 15.0]

    def lane_status_callback(self, msg: String) -> None:
        self.received_status_count += 1

        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.publish_status(False, f"invalid lane_status JSON: {exc}")
            return

        confidence = float(status.get("confidence") or 0.0)
        lane_detected = bool(status.get("lane_detected", False))
        min_confidence = float(self.get_parameter("min_confidence").value)

        if not lane_detected or confidence < min_confidence:
            geometry = {
                "stamp": time.time(),
                "frame": self.get_parameter("frame_id").value,
                "lane_detected": False,
                "centerline_points_m": [],
                "left_boundary_points_m": [],
                "right_boundary_points_m": [],
                "confidence": confidence,
                "source": "lane_projection_node",
                "note": "lane not detected or confidence below threshold",
            }
            self.geometry_pub.publish(String(data=json.dumps(geometry)))
            self.publish_status(True, "published empty geometry")
            return

        center_offset_px = float(status.get("center_offset_px") or 0.0)
        heading_error_deg = float(status.get("heading_error_deg") or 0.0)

        px_to_lateral_m = float(self.get_parameter("px_to_lateral_m").value)
        lateral_sign = float(self.get_parameter("lateral_sign").value)

        lateral_offset_m = lateral_sign * center_offset_px * px_to_lateral_m
        heading_rad = math.radians(heading_error_deg)

        points = []
        for x_m in self.parse_forward_samples():
            y_m = lateral_offset_m + math.tan(heading_rad) * x_m
            points.append([round(x_m, 3), round(y_m, 3)])

        geometry = {
            "stamp": time.time(),
            "frame": self.get_parameter("frame_id").value,
            "lane_detected": True,
            "centerline_points_m": points,
            "left_boundary_points_m": [],
            "right_boundary_points_m": [],
            "center_offset_px": center_offset_px,
            "heading_error_deg": heading_error_deg,
            "approx_lateral_offset_m": round(lateral_offset_m, 3),
            "confidence": confidence,
            "source": "lane_projection_node",
            "note": "approximate image-space to vehicle-frame projection; not calibrated yet",
        }

        self.geometry_pub.publish(String(data=json.dumps(geometry)))
        self.published_geometry_count += 1
        self.publish_status(True, "published approximate lane geometry")

    def publish_status(self, ok: bool, message: str) -> None:
        odom_available = self.latest_odom is not None
        status = {
            "ok": ok,
            "message": message,
            "received_status_count": self.received_status_count,
            "published_geometry_count": self.published_geometry_count,
            "odom_available": odom_available,
        }
        self.status_pub.publish(String(data=json.dumps(status)))


def main(args=None):
    rclpy.init(args=args)
    node = LaneProjectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
