#!/usr/bin/env python3
import json
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String
from cv_bridge import CvBridge


class FusionNode(Node):
    def __init__(self):
        super().__init__("fusion_node")

        self.bridge = CvBridge()

        self.declare_parameter("tracks_topic", "/perception/tracks")
        self.declare_parameter("depth_topic", "/perception/depth/image")
        self.declare_parameter("output_topic", "/perception/fused_objects")
        self.declare_parameter("min_valid_depth", 0.0)

        self.tracks_topic = self.get_parameter("tracks_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.min_valid_depth = float(self.get_parameter("min_valid_depth").value)

        self.latest_depth = None
        self.latest_depth_header = None

        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_cb,
            10,
        )

        self.tracks_sub = self.create_subscription(
            Detection2DArray,
            self.tracks_topic,
            self.tracks_cb,
            10,
        )

        self.pub = self.create_publisher(String, self.output_topic, 10)

        self.get_logger().info(f"Subscribing tracks: {self.tracks_topic}")
        self.get_logger().info(f"Subscribing depth : {self.depth_topic}")
        self.get_logger().info(f"Publishing fused objects: {self.output_topic}")

    def depth_cb(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            self.latest_depth = depth.astype(np.float32)
            self.latest_depth_header = msg.header
        except Exception as e:
            self.get_logger().warn(f"Failed to read depth image: {e}")

    def parse_class_and_id(self, class_id: str):
        """
        Tracker currently encodes labels as e.g. 'car#7'.
        """
        if "#" in class_id:
            label, tid = class_id.rsplit("#", 1)
            try:
                return label, int(tid)
            except ValueError:
                return label, -1
        return class_id, -1

    def bbox_depth_stats(self, depth, cx, cy, w, h):
        H, W = depth.shape[:2]

        x1 = int(max(0, round(cx - w / 2)))
        y1 = int(max(0, round(cy - h / 2)))
        x2 = int(min(W, round(cx + w / 2)))
        y2 = int(min(H, round(cy + h / 2)))

        if x2 <= x1 or y2 <= y1:
            return None

        crop = depth[y1:y2, x1:x2]

        valid = crop[np.isfinite(crop)]
        valid = valid[valid > self.min_valid_depth]

        if valid.size == 0:
            return None

        return {
            "depth_median": float(np.median(valid)),
            "depth_mean": float(np.mean(valid)),
            "depth_min": float(np.min(valid)),
            "depth_p10": float(np.percentile(valid, 10)),
            "num_pixels": int(valid.size),
        }

    def tracks_cb(self, msg: Detection2DArray):
        if self.latest_depth is None:
            return

        fused = []

        for det in msg.detections:
            bbox = det.bbox
            cx = float(bbox.center.position.x)
            cy = float(bbox.center.position.y)
            w = float(bbox.size_x)
            h = float(bbox.size_y)

            label = "unknown"
            track_id = -1
            score = 0.0

            if det.results:
                best = max(det.results, key=lambda r: r.hypothesis.score)
                label, track_id = self.parse_class_and_id(best.hypothesis.class_id)
                score = float(best.hypothesis.score)

            stats = self.bbox_depth_stats(self.latest_depth, cx, cy, w, h)
            if stats is None:
                continue

            fused.append({
                "track_id": track_id,
                "class": label,
                "score": score,
                "bbox": {
                    "cx": cx,
                    "cy": cy,
                    "w": w,
                    "h": h,
                },
                **stats,
            })

        out = String()
        out.data = json.dumps({
            "stamp": {
                "sec": msg.header.stamp.sec,
                "nanosec": msg.header.stamp.nanosec,
            },
            "frame_id": msg.header.frame_id,
            "objects": fused,
        })

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
