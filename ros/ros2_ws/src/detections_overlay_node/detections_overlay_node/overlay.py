#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import Detection2DArray

from cv_bridge import CvBridge
import cv2
import numpy as np

from message_filters import Subscriber, ApproximateTimeSynchronizer


class DetectionsOverlayNode(Node):
    def __init__(self):
        super().__init__("detections_overlay_node")

        self.image_topic = "/carla/rgb/image_raw/compressed"
        self.det_topic = "/perception/detections"

        self.out_raw_topic = "/perception/detections_overlay"
        self.out_comp_topic = "/perception/detections_overlay/compressed"

        self.jpeg_quality = 60

        self.bridge = CvBridge()

        self.pub_raw = self.create_publisher(Image, self.out_raw_topic, 10)
        self.pub_comp = self.create_publisher(CompressedImage, self.out_comp_topic, 10)

        # Subscribe to compressed image
        self.sub_img = Subscriber(self, CompressedImage, self.image_topic)
        self.sub_det = Subscriber(self, Detection2DArray, self.det_topic)

        self.sync = ApproximateTimeSynchronizer(
            [self.sub_img, self.sub_det],
            queue_size=20,
            slop=0.15,
            allow_headerless=False
        )
        self.sync.registerCallback(self.cb)

        self.get_logger().info(f"Image input      : {self.image_topic}")
        self.get_logger().info(f"Detections input : {self.det_topic}")
        self.get_logger().info(f"Overlay raw      : {self.out_raw_topic}")
        self.get_logger().info(f"Overlay compressed: {self.out_comp_topic}")

    def cb(self, img_msg: CompressedImage, det_msg: Detection2DArray):
        # Decode compressed JPEG
        np_arr = np.frombuffer(img_msg.data, dtype=np.uint8)
        cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if cv_img is None:
            self.get_logger().warn("Failed to decode compressed image")
            return

        # Draw detections
        for det in det_msg.detections:
            bbox = det.bbox

            cx = float(bbox.center.position.x)
            cy = float(bbox.center.position.y)
            w = float(bbox.size_x)
            h = float(bbox.size_y)

            x1 = int(round(cx - w / 2.0))
            y1 = int(round(cy - h / 2.0))
            x2 = int(round(cx + w / 2.0))
            y2 = int(round(cy + h / 2.0))

            x1 = max(0, min(x1, cv_img.shape[1] - 1))
            x2 = max(0, min(x2, cv_img.shape[1] - 1))
            y1 = max(0, min(y1, cv_img.shape[0] - 1))
            y2 = max(0, min(y2, cv_img.shape[0] - 1))

            cv2.rectangle(cv_img, (x1, y1), (x2, y2), (0, 255, 0), 2)

            label = "obj"
            score = None
            if det.results:
                best = max(det.results, key=lambda r: r.hypothesis.score)
                score = float(best.hypothesis.score)
                label = str(best.hypothesis.class_id) if best.hypothesis.class_id else "obj"

            text = f"{label} {score:.2f}" if score is not None else label
            ty = max(0, y1 - 5)
            cv2.putText(cv_img, text, (x1, ty),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 0), 2)

        # Publish RAW overlay
        raw_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
        raw_msg.header = img_msg.header
        self.pub_raw.publish(raw_msg)

        # Publish COMPRESSED overlay
        ok, jpg = cv2.imencode(
            '.jpg',
            cv_img,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        )

        if ok:
            comp_msg = CompressedImage()
            comp_msg.header = img_msg.header
            comp_msg.format = "jpeg"
            comp_msg.data = jpg.tobytes()
            self.pub_comp.publish(comp_msg)


def main():
    rclpy.init()
    node = DetectionsOverlayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
