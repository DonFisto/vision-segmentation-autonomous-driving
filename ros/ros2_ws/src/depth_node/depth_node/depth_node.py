#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

import cv2
import numpy as np
import torch
from PIL import Image as PILImage
from transformers import pipeline


class DepthNode(Node):
    def __init__(self):
        super().__init__("depth_node")

        self.bridge = CvBridge()

        self.declare_parameter("input_topic", "/carla/rgb/image_raw")
        self.declare_parameter("model_name", "depth-anything/Depth-Anything-V2-Small-hf")
        self.declare_parameter("jpeg_quality", 60)

        self.input_topic = self.get_parameter("input_topic").value
        self.model_name = self.get_parameter("model_name").value
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)

        device = 0 if torch.cuda.is_available() else -1

        self.get_logger().info(f"Loading Depth Anything: {self.model_name}")
        self.pipe = pipeline(
            task="depth-estimation",
            model=self.model_name,
            device=device,
        )
        self.get_logger().info("Depth model loaded.")

        self.sub = self.create_subscription(Image, self.input_topic, self.image_cb, 10)

        self.depth_pub = self.create_publisher(Image, "/perception/depth/image", 10)
        self.color_pub = self.create_publisher(
            CompressedImage,
            "/perception/depth/colormap/compressed",
            10,
        )

        self.get_logger().info(f"Subscribed to {self.input_topic}")
        self.get_logger().info("Publishing:")
        self.get_logger().info("  /perception/depth/image")
        self.get_logger().info("  /perception/depth/colormap/compressed")

    def image_cb(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return

        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        pil = PILImage.fromarray(rgb)

        result = self.pipe(pil)
        depth = np.array(result["depth"]).astype(np.float32)

        d_min = float(depth.min())
        d_max = float(depth.max())

        if d_max > d_min:
            depth_norm = (depth - d_min) / (d_max - d_min)
        else:
            depth_norm = np.zeros_like(depth, dtype=np.float32)

        depth_u8 = (depth_norm * 255.0).astype(np.uint8)
        color = cv2.applyColorMap(depth_u8, cv2.COLORMAP_INFERNO)

        # Raw float32 depth for fusion
        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding="32FC1")
        depth_msg.header = msg.header
        self.depth_pub.publish(depth_msg)

        # Compressed visualization for Foxglove
        ok, jpg = cv2.imencode(
            ".jpg",
            color,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )

        if ok:
            comp = CompressedImage()
            comp.header = msg.header
            comp.format = "jpeg"
            comp.data = jpg.tobytes()
            self.color_pub.publish(comp)


def main(args=None):
    rclpy.init(args=args)
    node = DepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
