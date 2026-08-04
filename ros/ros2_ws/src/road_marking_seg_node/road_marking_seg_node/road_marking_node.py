#!/usr/bin/env python3

from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
import torch

from cv_bridge import CvBridge
from mmseg.apis import inference_model, init_model
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


class RoadMarkingSegNode(Node):
    def __init__(self) -> None:
        super().__init__("road_marking_seg_node")

        self.bridge = CvBridge()

        self.declare_parameter("config_path", "")
        self.declare_parameter("checkpoint_path", "")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("threshold", 0.50)

        self.declare_parameter(
            "input_topic",
            "/carla/rgb/image_raw",
        )
        self.declare_parameter(
            "mask_topic",
            "/perception/road_marking/mask",
        )
        self.declare_parameter(
            "probability_topic",
            "/perception/road_marking/probability",
        )
        self.declare_parameter(
            "overlay_topic",
            "/perception/road_marking/overlay/compressed",
        )

        self.declare_parameter("publish_probability", False)
        self.declare_parameter("publish_overlay", True)
        self.declare_parameter("jpeg_quality", 70)
        self.declare_parameter("log_every", 30)

        config_path = Path(
            str(self.get_parameter("config_path").value)
        ).expanduser()

        checkpoint_path = Path(
            str(self.get_parameter("checkpoint_path").value)
        ).expanduser()

        device = str(self.get_parameter("device").value)

        self.threshold = float(
            self.get_parameter("threshold").value
        )
        self.publish_probability = bool(
            self.get_parameter("publish_probability").value
        )
        self.publish_overlay = bool(
            self.get_parameter("publish_overlay").value
        )
        self.jpeg_quality = int(
            self.get_parameter("jpeg_quality").value
        )
        self.log_every = max(
            1,
            int(self.get_parameter("log_every").value),
        )

        input_topic = str(
            self.get_parameter("input_topic").value
        )
        mask_topic = str(
            self.get_parameter("mask_topic").value
        )
        probability_topic = str(
            self.get_parameter("probability_topic").value
        )
        overlay_topic = str(
            self.get_parameter("overlay_topic").value
        )

        if not config_path.is_file():
            raise FileNotFoundError(
                f"Config not found: {config_path}"
            )

        if not checkpoint_path.is_file():
            raise FileNotFoundError(
                f"Checkpoint not found: {checkpoint_path}"
            )

        if not 0.0 <= self.threshold <= 1.0:
            raise ValueError(
                "threshold must be between 0 and 1"
            )

        self.get_logger().info(
            f"Loading config: {config_path}"
        )
        self.get_logger().info(
            f"Loading checkpoint: {checkpoint_path}"
        )
        self.get_logger().info(
            f"Using device: {device}"
        )

        self.model = init_model(
            str(config_path),
            str(checkpoint_path),
            device=device,
        )
        self.model.eval()

        self.subscription = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.mask_pub = self.create_publisher(
            Image,
            mask_topic,
            qos_profile_sensor_data,
        )

        self.probability_pub = None
        if self.publish_probability:
            self.probability_pub = self.create_publisher(
                Image,
                probability_topic,
                qos_profile_sensor_data,
            )

        self.overlay_pub = None
        if self.publish_overlay:
            self.overlay_pub = self.create_publisher(
                CompressedImage,
                overlay_topic,
                qos_profile_sensor_data,
            )

        self.frame_count = 0
        self.total_inference_ms = 0.0
        self.warned_missing_logits = False

        self.get_logger().info(
            f"Subscribed to: {input_topic}"
        )
        self.get_logger().info(
            f"Publishing mask: {mask_topic}"
        )
        self.get_logger().info(
            f"Probability threshold: {self.threshold:.2f}"
        )

    def extract_probability(self, result) -> np.ndarray:
        seg_logits = getattr(result, "seg_logits", None)

        if seg_logits is not None:
            logits = seg_logits.data

            if logits.ndim == 4:
                logits = logits[0]

            if logits.ndim != 3 or logits.shape[0] != 2:
                raise RuntimeError(
                    "Expected logits with shape [2, H, W], "
                    f"received {tuple(logits.shape)}"
                )

            probabilities = torch.softmax(
                logits.float(),
                dim=0,
            )

            return (
                probabilities[1]
                .detach()
                .cpu()
                .numpy()
                .astype(np.float32)
            )

        if not self.warned_missing_logits:
            self.get_logger().warning(
                "Inference output has no seg_logits. "
                "Using argmax output; threshold will have "
                "no effect."
            )
            self.warned_missing_logits = True

        prediction = (
            result.pred_sem_seg.data
            .squeeze()
            .detach()
            .cpu()
            .numpy()
        )

        return (prediction == 1).astype(np.float32)

    def image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )

            start = time.perf_counter()

            with torch.inference_mode():
                result = inference_model(
                    self.model,
                    frame,
                )

            probability = self.extract_probability(result)

            if probability.shape != frame.shape[:2]:
                probability = cv2.resize(
                    probability,
                    (frame.shape[1], frame.shape[0]),
                    interpolation=cv2.INTER_LINEAR,
                )

            binary = (
                probability >= self.threshold
            ).astype(np.uint8)

            inference_ms = (
                time.perf_counter() - start
            ) * 1000.0

            self.frame_count += 1
            self.total_inference_ms += inference_ms

            # Published mask convention:
            #   0   = background
            #   255 = road marking
            mask = binary * 255

            mask_msg = self.bridge.cv2_to_imgmsg(
                mask,
                encoding="mono8",
            )
            mask_msg.header = msg.header
            self.mask_pub.publish(mask_msg)

            if self.probability_pub is not None:
                probability_msg = self.bridge.cv2_to_imgmsg(
                    probability,
                    encoding="32FC1",
                )
                probability_msg.header = msg.header
                self.probability_pub.publish(
                    probability_msg
                )

            if self.overlay_pub is not None:
                colored = frame.copy()
                colored[binary == 1] = (0, 255, 255)

                overlay = cv2.addWeighted(
                    frame,
                    0.65,
                    colored,
                    0.35,
                    0.0,
                )

                ok, encoded = cv2.imencode(
                    ".jpg",
                    overlay,
                    [
                        int(cv2.IMWRITE_JPEG_QUALITY),
                        self.jpeg_quality,
                    ],
                )

                if ok:
                    overlay_msg = CompressedImage()
                    overlay_msg.header = msg.header
                    overlay_msg.format = "jpeg"
                    overlay_msg.data = encoded.tobytes()
                    self.overlay_pub.publish(overlay_msg)

            if self.frame_count % self.log_every == 0:
                average_ms = (
                    self.total_inference_ms
                    / self.frame_count
                )
                processing_fps = (
                    1000.0 / average_ms
                    if average_ms > 0.0
                    else 0.0
                )

                self.get_logger().info(
                    f"frames={self.frame_count} "
                    f"inference={inference_ms:.1f} ms "
                    f"average={average_ms:.1f} ms "
                    f"fps={processing_fps:.1f} "
                    f"positive={100.0 * binary.mean():.2f}%"
                )

        except Exception as exc:
            self.get_logger().error(
                f"Inference failed: {exc}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RoadMarkingSegNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
