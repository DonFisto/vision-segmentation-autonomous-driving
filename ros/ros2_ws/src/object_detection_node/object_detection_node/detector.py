#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from vision_msgs.msg import ObjectHypothesisWithPose
from vision_msgs.msg import ObjectHypothesis
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D
from cv_bridge import CvBridge

import numpy as np
import cv2


class ObjectDetectionNode(Node):

    def __init__(self):
        super().__init__('object_detection_node')

        self.bridge = CvBridge()

        # Cityscapes IDs you want to detect (default: vehicles)
        # 13 car, 14 truck, 15 bus, 16 train, 17 motorcycle, 18 bicycle
        self.target_ids = [6, 7, 11, 12, 13, 14, 15, 16, 17, 18]
        self.id2name = {
            0:"road", 1:"sidewalk", 2:"building", 3:"wall", 4:"fence", 5:"pole",
            6:"traffic light", 7:"traffic sign", 8:"vegetation", 9:"terrain", 10:"sky",
            11:"person", 12:"rider", 13:"car", 14:"truck", 15:"bus", 16:"train",
            17:"motorcycle", 18:"bicycle"
        }
        # Filtering
        self.min_area = 30  # pixels
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

        self.sub = self.create_subscription(
            Image,
            '/perception/semantic_mask',
            self.mask_callback,
            10
        )

        self.pub = self.create_publisher(
            Detection2DArray,
            '/perception/detections',
            10
        )

        self.get_logger().info(
            f"Object Detection Node started. target_ids={self.target_ids}"
        )

    def mask_callback(self, msg: Image):
        # This is a class-id image (0..18), not a binary mask
        seg = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

        # 1) Convert class-id map -> binary mask for the target classes
        bin_mask = np.isin(seg, self.target_ids).astype(np.uint8) * 255

        # 2) Clean up noise (optional but recommended)
        # Opening removes tiny dots; closing can fill small holes
        bin_mask = cv2.morphologyEx(bin_mask, cv2.MORPH_OPEN, self.kernel, iterations=1)
        bin_mask = cv2.morphologyEx(bin_mask, cv2.MORPH_CLOSE, self.kernel, iterations=1)

        # 3) Connected components on the binary mask
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            bin_mask, connectivity=8
        )

        detections_msg = Detection2DArray()
        detections_msg.header = msg.header

        # stats: [label, x, y, w, h, area] for each component (label 0 = background)
        for label in range(1, num_labels):
            x, y, w, h, area = stats[label]

            if area < self.min_area:
                continue

            # BoundingBox2D uses center + size
            bbox = BoundingBox2D()
            bbox.center.position.x = float(x + w / 2.0)
            bbox.center.position.y = float(y + h / 2.0)
            bbox.size_x = float(w)
            bbox.size_y = float(h)

            det = Detection2D()
            det.header = msg.header
            det.bbox = bbox

            # Component region mask (where this connected component is)
            component = (labels == label)

            # Majority vote of class IDs inside this component (from the original seg map)
            ids = seg[component]
            # guard: empty shouldn't happen, but safe
            if ids.size == 0:
                continue

            cls_id = int(np.bincount(ids.ravel(), minlength=19).argmax())
            cls_name = self.id2name.get(cls_id, str(cls_id))

            # Attach classification result so the overlay node can show text
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = cls_name
            hyp.hypothesis.score = float(area) / float(seg.size)  # simple proxy score
            det.results.append(hyp)

            detections_msg.detections.append(det)

        self.pub.publish(detections_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
