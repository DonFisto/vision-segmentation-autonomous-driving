#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

import cv2
import numpy as np

from mmseg.apis import init_model, inference_model


CONFIG_PATH = "/home/danielmartinez/vision-segmentation-autonomous-driving/configs/cityscapes/segformer_b0_cityscapes.py"
CHECKPOINT_PATH = "/home/danielmartinez/vision-segmentation-autonomous-driving/work_dirs/segformer_b0_cityscapes/iter_80000.pth"


class SemanticSegNode(Node):
    def __init__(self):
        super().__init__('semantic_seg_node')

        self.bridge = CvBridge()

        # Parameters
        # Subscribe to raw image locally (best performance; no network penalty if not forwarded)
        self.declare_parameter('input_topic', '/carla/rgb/image_raw')

        # Publish options
        self.declare_parameter('publish_mask', True)
        self.declare_parameter('mask_topic', '/perception/semantic_mask')

        # Overlay publishing as JPEG (small bandwidth for Foxglove)
        self.declare_parameter('publish_overlay_compressed', True)
        self.declare_parameter('overlay_topic', '/perception/semantic_overlay/compressed')
        self.declare_parameter('jpeg_quality', 60)  # 40–80 typical

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.publish_mask = bool(self.get_parameter('publish_mask').value)
        self.mask_topic = str(self.get_parameter('mask_topic').value)
        self.publish_overlay_compressed = bool(self.get_parameter('publish_overlay_compressed').value)
        self.overlay_topic = str(self.get_parameter('overlay_topic').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)

        # Load model once at startup
        self.get_logger().info("Loading segmentation model...")
        self.model = init_model(CONFIG_PATH, CHECKPOINT_PATH, device='cuda:0')
        self.get_logger().info("Model loaded.")

        # Cityscapes color palette (19 classes)
        self.color_map = np.array([
            [128, 64,128],
            [244, 35,232],
            [ 70, 70, 70],
            [102,102,156],
            [190,153,153],
            [153,153,153],
            [250,170, 30],
            [220,220,  0],
            [107,142, 35],
            [152,251,152],
            [ 70,130,180],
            [220, 20, 60],
            [255,  0,  0],
            [  0,  0,142],
            [  0,  0, 70],
            [  0, 60,100],
            [  0, 80,100],
            [  0,  0,230],
            [119, 11, 32]
        ], dtype=np.uint8)

        # Subscriber (RAW Image)
        self.subscription = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            10
        )

        # Publishers
        self.mask_pub = None
        if self.publish_mask:
            self.mask_pub = self.create_publisher(Image, self.mask_topic, 10)

        self.overlay_comp_pub = None
        if self.publish_overlay_compressed:
            self.overlay_comp_pub = self.create_publisher(CompressedImage, self.overlay_topic, 10)

        self.get_logger().info(f"Subscribing: {self.input_topic}")
        if self.mask_pub is not None:
            self.get_logger().info(f"Publishing mask: {self.mask_topic} (mono8)")
        if self.overlay_comp_pub is not None:
            self.get_logger().info(f"Publishing overlay: {self.overlay_topic} (CompressedImage jpeg q={self.jpeg_quality})")

    def image_callback(self, msg: Image):
        # Convert ROS Image -> OpenCV BGR
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run inference
        result = inference_model(self.model, frame)

        # Extract predicted segmentation mask
        seg = result.pred_sem_seg.data[0].cpu().numpy().astype(np.uint8)

        # Publish raw mask (mono8)
        if self.mask_pub is not None:
            mask_msg = self.bridge.cv2_to_imgmsg(seg, encoding='mono8')
            mask_msg.header = msg.header
            self.mask_pub.publish(mask_msg)

        # Build colored segmentation
        color_mask = self.color_map[seg]

        # Create overlay
        overlay = cv2.addWeighted(frame, 0.6, color_mask, 0.4, 0)

        # Publish overlay as JPEG CompressedImage
        if self.overlay_comp_pub is not None:
            ok, jpg = cv2.imencode('.jpg', overlay, [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_quality)])
            if ok:
                comp = CompressedImage()
                comp.header = msg.header
                comp.format = "jpeg"
                comp.data = jpg.tobytes()
                self.overlay_comp_pub.publish(comp)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticSegNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
