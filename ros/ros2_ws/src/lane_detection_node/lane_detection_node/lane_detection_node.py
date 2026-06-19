#!/usr/bin/env python3

import json
import math
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String


class LaneDetectionNode(Node):
    """
    Classical lane detector.

    Inputs:
      - RGB compressed image
      - semantic mask, used only to restrict the search to road pixels

    Outputs:
      - lane candidate mask
      - lane overlay
      - lane status as JSON string

    This is intentionally a simple baseline, not a trained lane detection model.
    """

    def __init__(self):
        super().__init__("lane_detection_node")

        # Topics
        self.declare_parameter("rgb_topic", "/carla/rgb/image_raw/compressed")
        self.declare_parameter("semantic_topic", "/perception/semantic_mask")
        self.declare_parameter("lane_mask_topic", "/perception/lane_mask/compressed")
        self.declare_parameter("lane_overlay_topic", "/perception/lane_overlay/compressed")
        self.declare_parameter("lane_status_topic", "/perception/lane_status")

        # Semantic ROI
        self.declare_parameter("use_semantic_roi", True)
        self.declare_parameter("road_class_id", 0)
        self.declare_parameter("min_road_area_ratio", 0.02)
        self.declare_parameter("road_dilate_iterations", 2)

        # Image ROI
        self.declare_parameter("roi_x_min_ratio", 0.05)
        self.declare_parameter("roi_x_max_ratio", 0.95)
        self.declare_parameter("roi_y_min_ratio", 0.45)
        self.declare_parameter("roi_y_max_ratio", 0.98)

        # Lane color thresholds in HLS
        self.declare_parameter("white_l_min", 185)
        self.declare_parameter("white_s_max", 90)

        self.declare_parameter("yellow_h_min", 12)
        self.declare_parameter("yellow_h_max", 40)
        self.declare_parameter("yellow_l_min", 80)
        self.declare_parameter("yellow_l_max", 255)
        self.declare_parameter("yellow_s_min", 70)
        self.declare_parameter("yellow_s_max", 255)

        # Optional edge filtering
        self.declare_parameter("use_canny_edges", False)
        self.declare_parameter("canny_low", 60)
        self.declare_parameter("canny_high", 160)

        # Morphology
        self.declare_parameter("morph_kernel_size", 5)
        self.declare_parameter("dilate_iterations", 1)

        # Fitting
        self.declare_parameter("min_lane_pixels", 80)
        self.declare_parameter("min_y_span_px", 60)
        self.declare_parameter("split_margin_px", 20)
        self.declare_parameter("max_abs_dxdy", 1.5)
        self.declare_parameter("lane_width_px", 360.0)
        self.declare_parameter("fit_top_y_ratio", 0.62)
        self.declare_parameter("fit_bottom_y_ratio", 0.95)

        # Runtime
        self.declare_parameter("process_every_n", 1)
        self.declare_parameter("publish_debug_overlay", True)

        self.rgb_topic = self.get_parameter("rgb_topic").value
        self.semantic_topic = self.get_parameter("semantic_topic").value
        self.lane_mask_topic = self.get_parameter("lane_mask_topic").value
        self.lane_overlay_topic = self.get_parameter("lane_overlay_topic").value
        self.lane_status_topic = self.get_parameter("lane_status_topic").value

        self.latest_semantic_msg: Optional[Image] = None
        self.frame_count = 0

        self.rgb_sub = self.create_subscription(
            CompressedImage,
            self.rgb_topic,
            self.rgb_callback,
            10,
        )

        self.semantic_sub = self.create_subscription(
            Image,
            self.semantic_topic,
            self.semantic_callback,
            10,
        )

        self.lane_mask_pub = self.create_publisher(
            CompressedImage,
            self.lane_mask_topic,
            10,
        )

        self.lane_overlay_pub = self.create_publisher(
            CompressedImage,
            self.lane_overlay_topic,
            10,
        )

        self.status_pub = self.create_publisher(
            String,
            self.lane_status_topic,
            10,
        )

        self.get_logger().info("Lane detection node started")
        self.get_logger().info(f"RGB topic: {self.rgb_topic}")
        self.get_logger().info(f"Semantic topic: {self.semantic_topic}")

    def semantic_callback(self, msg: Image) -> None:
        self.latest_semantic_msg = msg

    def rgb_callback(self, msg: CompressedImage) -> None:
        self.frame_count += 1
        process_every_n = int(self.get_parameter("process_every_n").value)
        if process_every_n > 1 and self.frame_count % process_every_n != 0:
            return

        bgr = self.decode_compressed_bgr(msg)
        if bgr is None:
            return

        h, w = bgr.shape[:2]

        roi_mask = self.build_roi_mask(h, w)
        road_mask = self.build_road_mask(h, w)

        lane_mask = self.detect_lane_candidates(bgr, roi_mask, road_mask)
        left_fit, right_fit = self.fit_lane_sides(lane_mask)

        status = self.compute_status(
            image_width=w,
            image_height=h,
            left_fit=left_fit,
            right_fit=right_fit,
            lane_mask=lane_mask,
        )

        self.publish_mask(msg, lane_mask)
        self.publish_overlay(msg, bgr, lane_mask, left_fit, right_fit, status)
        self.status_pub.publish(String(data=json.dumps(status)))

    def decode_compressed_bgr(self, msg: CompressedImage) -> Optional[np.ndarray]:
        data = np.frombuffer(msg.data, dtype=np.uint8)
        image = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if image is None:
            self.get_logger().warning("Failed to decode RGB compressed image")
            return None
        return image

    def image_msg_to_numpy(self, msg: Image) -> Optional[np.ndarray]:
        encoding = msg.encoding.lower()

        try:
            if encoding in ("mono8", "8uc1"):
                arr = np.frombuffer(msg.data, dtype=np.uint8)
                arr = arr.reshape((msg.height, msg.step))
                return arr[:, :msg.width]

            if encoding in ("16uc1", "mono16"):
                arr = np.frombuffer(msg.data, dtype=np.uint16)
                arr = arr.reshape((msg.height, msg.step // 2))
                return arr[:, :msg.width]

            if encoding == "32sc1":
                arr = np.frombuffer(msg.data, dtype=np.int32)
                arr = arr.reshape((msg.height, msg.step // 4))
                return arr[:, :msg.width]

            if encoding in ("rgb8", "bgr8"):
                arr = np.frombuffer(msg.data, dtype=np.uint8)
                arr = arr.reshape((msg.height, msg.step // 3, 3))
                return arr[:, :msg.width, :]

            self.get_logger().warning(f"Unsupported semantic mask encoding: {msg.encoding}")
            return None

        except ValueError as exc:
            self.get_logger().warning(f"Failed to reshape semantic mask: {exc}")
            return None

    def build_roi_mask(self, h: int, w: int) -> np.ndarray:
        x_min = int(w * float(self.get_parameter("roi_x_min_ratio").value))
        x_max = int(w * float(self.get_parameter("roi_x_max_ratio").value))
        y_min = int(h * float(self.get_parameter("roi_y_min_ratio").value))
        y_max = int(h * float(self.get_parameter("roi_y_max_ratio").value))

        mask = np.zeros((h, w), dtype=np.uint8)
        mask[y_min:y_max, x_min:x_max] = 255
        return mask

    def build_road_mask(self, h: int, w: int) -> np.ndarray:
        use_semantic_roi = bool(self.get_parameter("use_semantic_roi").value)
        if not use_semantic_roi or self.latest_semantic_msg is None:
            return np.full((h, w), 255, dtype=np.uint8)

        semantic = self.image_msg_to_numpy(self.latest_semantic_msg)
        if semantic is None:
            return np.full((h, w), 255, dtype=np.uint8)

        if semantic.ndim == 3:
            # Fallback for color-coded masks. The expected format is class-id mono8,
            # but this keeps the node from crashing if the topic changes.
            semantic = semantic[:, :, 0]

        semantic = cv2.resize(
            semantic,
            (w, h),
            interpolation=cv2.INTER_NEAREST,
        )

        road_class_id = int(self.get_parameter("road_class_id").value)
        road_mask = np.zeros((h, w), dtype=np.uint8)
        road_mask[semantic == road_class_id] = 255

        min_road_area_ratio = float(self.get_parameter("min_road_area_ratio").value)
        road_area_ratio = float(np.mean(road_mask > 0))

        if road_area_ratio < min_road_area_ratio:
            self.get_logger().warning(
                f"Road semantic ROI too small ({road_area_ratio:.3f}); "
                "falling back to image ROI only"
            )
            return np.full((h, w), 255, dtype=np.uint8)

        iterations = int(self.get_parameter("road_dilate_iterations").value)
        if iterations > 0:
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
            road_mask = cv2.dilate(road_mask, kernel, iterations=iterations)

        return road_mask

    def detect_lane_candidates(
        self,
        bgr: np.ndarray,
        roi_mask: np.ndarray,
        road_mask: np.ndarray,
    ) -> np.ndarray:
        hls = cv2.cvtColor(bgr, cv2.COLOR_BGR2HLS)

        white_l_min = int(self.get_parameter("white_l_min").value)
        white_s_max = int(self.get_parameter("white_s_max").value)

        white_mask = cv2.inRange(
            hls,
            np.array([0, white_l_min, 0], dtype=np.uint8),
            np.array([179, 255, white_s_max], dtype=np.uint8),
        )

        yellow_h_min = int(self.get_parameter("yellow_h_min").value)
        yellow_h_max = int(self.get_parameter("yellow_h_max").value)
        yellow_l_min = int(self.get_parameter("yellow_l_min").value)
        yellow_l_max = int(self.get_parameter("yellow_l_max").value)
        yellow_s_min = int(self.get_parameter("yellow_s_min").value)
        yellow_s_max = int(self.get_parameter("yellow_s_max").value)

        yellow_mask = cv2.inRange(
            hls,
            np.array([yellow_h_min, yellow_l_min, yellow_s_min], dtype=np.uint8),
            np.array([yellow_h_max, yellow_l_max, yellow_s_max], dtype=np.uint8),
        )

        lane_mask = cv2.bitwise_or(white_mask, yellow_mask)
        lane_mask = cv2.bitwise_and(lane_mask, roi_mask)
        lane_mask = cv2.bitwise_and(lane_mask, road_mask)

        use_canny_edges = bool(self.get_parameter("use_canny_edges").value)
        if use_canny_edges:
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (5, 5), 0)
            canny_low = int(self.get_parameter("canny_low").value)
            canny_high = int(self.get_parameter("canny_high").value)
            edges = cv2.Canny(gray, canny_low, canny_high)
            edge_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            edges = cv2.dilate(edges, edge_kernel, iterations=1)
            lane_mask = cv2.bitwise_and(lane_mask, edges)

        kernel_size = int(self.get_parameter("morph_kernel_size").value)
        kernel_size = max(1, kernel_size)
        if kernel_size % 2 == 0:
            kernel_size += 1

        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (kernel_size, kernel_size),
        )

        lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_OPEN, kernel)
        lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_CLOSE, kernel)

        dilate_iterations = int(self.get_parameter("dilate_iterations").value)
        if dilate_iterations > 0:
            lane_mask = cv2.dilate(lane_mask, kernel, iterations=dilate_iterations)

        return lane_mask

    def fit_lane_sides(self, lane_mask: np.ndarray):
        h, w = lane_mask.shape[:2]
        ys, xs = np.nonzero(lane_mask > 0)

        if len(xs) == 0:
            return None, None

        mid_x = w // 2
        split_margin = int(self.get_parameter("split_margin_px").value)

        left_selection = xs < (mid_x - split_margin)
        right_selection = xs > (mid_x + split_margin)

        left_fit = self.fit_side(xs[left_selection], ys[left_selection], "left")
        right_fit = self.fit_side(xs[right_selection], ys[right_selection], "right")

        return left_fit, right_fit

    def fit_side(self, xs: np.ndarray, ys: np.ndarray, side: str):
        min_lane_pixels = int(self.get_parameter("min_lane_pixels").value)
        min_y_span_px = int(self.get_parameter("min_y_span_px").value)
        max_abs_dxdy = float(self.get_parameter("max_abs_dxdy").value)

        if len(xs) < min_lane_pixels:
            return None

        if int(np.max(ys) - np.min(ys)) < min_y_span_px:
            return None

        try:
            # Fit x = a*y + b.
            a, b = np.polyfit(
                ys.astype(np.float32),
                xs.astype(np.float32),
                deg=1,
            )
        except np.linalg.LinAlgError:
            return None

        if abs(float(a)) > max_abs_dxdy:
            return None

        return {
            "side": side,
            "a": float(a),
            "b": float(b),
            "pixels": int(len(xs)),
            "y_min": int(np.min(ys)),
            "y_max": int(np.max(ys)),
        }

    def x_at_y(self, fit, y: float, image_width: int) -> float:
        x = fit["a"] * y + fit["b"]
        return float(np.clip(x, 0, image_width - 1))

    def compute_status(
        self,
        image_width: int,
        image_height: int,
        left_fit,
        right_fit,
        lane_mask: np.ndarray,
    ) -> dict:
        y_top = int(image_height * float(self.get_parameter("fit_top_y_ratio").value))
        y_bottom = int(image_height * float(self.get_parameter("fit_bottom_y_ratio").value))
        lane_width_px = float(self.get_parameter("lane_width_px").value)

        left_visible = left_fit is not None
        right_visible = right_fit is not None

        center_bottom = None
        center_top = None
        inferred_side = None

        if left_visible and right_visible:
            left_bottom = self.x_at_y(left_fit, y_bottom, image_width)
            right_bottom = self.x_at_y(right_fit, y_bottom, image_width)
            left_top = self.x_at_y(left_fit, y_top, image_width)
            right_top = self.x_at_y(right_fit, y_top, image_width)

            center_bottom = 0.5 * (left_bottom + right_bottom)
            center_top = 0.5 * (left_top + right_top)

        elif left_visible:
            left_bottom = self.x_at_y(left_fit, y_bottom, image_width)
            left_top = self.x_at_y(left_fit, y_top, image_width)

            center_bottom = left_bottom + 0.5 * lane_width_px
            center_top = left_top + 0.5 * lane_width_px
            inferred_side = "right"

        elif right_visible:
            right_bottom = self.x_at_y(right_fit, y_bottom, image_width)
            right_top = self.x_at_y(right_fit, y_top, image_width)

            center_bottom = right_bottom - 0.5 * lane_width_px
            center_top = right_top - 0.5 * lane_width_px
            inferred_side = "left"

        total_lane_pixels = int(np.count_nonzero(lane_mask))

        if center_bottom is None or center_top is None:
            return {
                "lane_detected": False,
                "left_lane_visible": False,
                "right_lane_visible": False,
                "inferred_side": None,
                "center_offset_px": None,
                "heading_error_deg": None,
                "confidence": 0.0,
                "lane_pixels": total_lane_pixels,
            }

        image_center = image_width * 0.5
        center_offset_px = float(center_bottom - image_center)

        dx = float(center_top - center_bottom)
        dy = float(y_bottom - y_top)
        heading_error_deg = float(math.degrees(math.atan2(dx, dy)))

        min_lane_pixels = int(self.get_parameter("min_lane_pixels").value)
        pixel_score = min(1.0, total_lane_pixels / max(1.0, 4.0 * min_lane_pixels))

        visibility_score = 1.0 if (left_visible and right_visible) else 0.55
        confidence = float(np.clip(visibility_score * pixel_score, 0.0, 1.0))

        return {
            "lane_detected": confidence > 0.05,
            "left_lane_visible": left_visible,
            "right_lane_visible": right_visible,
            "inferred_side": inferred_side,
            "center_offset_px": round(center_offset_px, 2),
            "heading_error_deg": round(heading_error_deg, 2),
            "confidence": round(confidence, 3),
            "lane_pixels": total_lane_pixels,
        }

    def publish_mask(self, input_msg: CompressedImage, lane_mask: np.ndarray) -> None:
        ok, encoded = cv2.imencode(".png", lane_mask)
        if not ok:
            self.get_logger().warning("Failed to encode lane mask")
            return

        out = CompressedImage()
        out.header = input_msg.header
        out.format = "png"
        out.data = encoded.tobytes()
        self.lane_mask_pub.publish(out)

    def publish_overlay(
        self,
        input_msg: CompressedImage,
        bgr: np.ndarray,
        lane_mask: np.ndarray,
        left_fit,
        right_fit,
        status: dict,
    ) -> None:
        publish_debug_overlay = bool(self.get_parameter("publish_debug_overlay").value)
        if not publish_debug_overlay:
            return

        overlay = bgr.copy()
        h, w = overlay.shape[:2]

        # Paint candidate lane pixels.
        color_layer = np.zeros_like(overlay)
        color_layer[lane_mask > 0] = (0, 255, 255)
        overlay = cv2.addWeighted(overlay, 1.0, color_layer, 0.45, 0.0)

        y_top = int(h * float(self.get_parameter("fit_top_y_ratio").value))
        y_bottom = int(h * float(self.get_parameter("fit_bottom_y_ratio").value))

        if left_fit is not None:
            self.draw_fit(overlay, left_fit, y_top, y_bottom, w, (255, 0, 0))

        if right_fit is not None:
            self.draw_fit(overlay, right_fit, y_top, y_bottom, w, (0, 0, 255))

        if status.get("lane_detected"):
            image_center = w * 0.5
            center_bottom = image_center + float(status["center_offset_px"])

            heading = math.radians(float(status["heading_error_deg"]))
            dy = float(y_bottom - y_top)
            center_top = center_bottom + math.tan(heading) * dy

            cv2.line(
                overlay,
                (int(center_bottom), y_bottom),
                (int(center_top), y_top),
                (0, 255, 0),
                3,
            )

            cv2.circle(overlay, (int(image_center), y_bottom), 6, (255, 255, 255), -1)
            cv2.circle(overlay, (int(center_bottom), y_bottom), 6, (0, 255, 0), -1)

        text = (
            f"offset={status.get('center_offset_px')} px | "
            f"heading={status.get('heading_error_deg')} deg | "
            f"conf={status.get('confidence')}"
        )
        cv2.putText(
            overlay,
            text,
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        ok, encoded = cv2.imencode(".jpg", overlay, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        if not ok:
            self.get_logger().warning("Failed to encode lane overlay")
            return

        out = CompressedImage()
        out.header = input_msg.header
        out.format = "jpeg"
        out.data = encoded.tobytes()
        self.lane_overlay_pub.publish(out)

    def draw_fit(
        self,
        image: np.ndarray,
        fit,
        y_top: int,
        y_bottom: int,
        image_width: int,
        color: Tuple[int, int, int],
    ) -> None:
        x_top = self.x_at_y(fit, y_top, image_width)
        x_bottom = self.x_at_y(fit, y_bottom, image_width)

        cv2.line(
            image,
            (int(x_bottom), int(y_bottom)),
            (int(x_top), int(y_top)),
            color,
            3,
        )


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
