#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
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

        # Subscriber
        self.subscription = self.create_subscription(
            Image,
            '/carla/rgb/image_raw',
            self.image_callback,
            10
        )

        # Publishers
        self.mask_pub = self.create_publisher(Image, '/perception/semantic_mask', 10)
        self.overlay_pub = self.create_publisher(Image, '/perception/semantic_overlay', 10)

    def image_callback(self, msg):

        # Convert ROS Image → OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run inference
        result = inference_model(self.model, frame)

        # Extract predicted segmentation mask
        seg = result.pred_sem_seg.data[0].cpu().numpy()

        # Publish raw mask (mono8)
        mask_msg = self.bridge.cv2_to_imgmsg(seg.astype(np.uint8), encoding='mono8')
        mask_msg.header = msg.header
        self.mask_pub.publish(mask_msg)

        # Convert mask to colored segmentation
        color_mask = self.color_map[seg]

        # Create overlay
        overlay = cv2.addWeighted(frame, 0.6, color_mask, 0.4, 0)

        overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        overlay_msg.header = msg.header
        self.overlay_pub.publish(overlay_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticSegNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
