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

    v2:
      - RGB/HLS thresholding for white/yellow markings
      - optional semantic road ROI
      - optional connected-component cleanup
      - Hough segment extraction
      - geometric filtering to reject crosswalks/horizontal markings
      - conservative confidence scoring
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
        self.declare_parameter("min_road_area_ratio", 0.01)
        self.declare_parameter("road_dilate_iterations", 2)

        # Image ROI
        self.declare_parameter("roi_x_min_ratio", 0.12)
        self.declare_parameter("roi_x_max_ratio", 0.88)
        self.declare_parameter("roi_y_min_ratio", 0.52)
        self.declare_parameter("roi_y_max_ratio", 0.96)

        # Lane color thresholds in HLS
        self.declare_parameter("white_l_min", 170)
        self.declare_parameter("white_s_max", 95)

        self.declare_parameter("yellow_h_min", 12)
        self.declare_parameter("yellow_h_max", 40)
        self.declare_parameter("yellow_l_min", 80)
        self.declare_parameter("yellow_l_max", 255)
        self.declare_parameter("yellow_s_min", 80)
        self.declare_parameter("yellow_s_max", 255)

        # LDv3.2 shadow-aware local contrast white detection.
        # This helps detect white lane markings in shadows, where absolute
        # lightness thresholds fail.
        self.declare_parameter("use_adaptive_white", True)
        self.declare_parameter("use_clahe", True)
        self.declare_parameter("clahe_clip_limit", 2.0)
        self.declare_parameter("clahe_tile_grid_size", 8)
        self.declare_parameter("adaptive_white_block_size", 41)
        self.declare_parameter("adaptive_white_c", -6)
        self.declare_parameter("adaptive_white_min_l", 60)
        self.declare_parameter("adaptive_white_s_max", 145)

        # Gate adaptive white so it only helps in shadows/dark regions.
        # In bright regions, adaptive thresholding tends to over-detect
        # asphalt texture, cracks, and sunlit speckles.
        self.declare_parameter("adaptive_white_shadow_l_max", 155)
        self.declare_parameter("adaptive_white_use_shadow_gate", True)

        # Optional edge filtering
        self.declare_parameter("use_canny_edges", False)
        self.declare_parameter("canny_low", 60)
        self.declare_parameter("canny_high", 160)

        # Morphology
        self.declare_parameter("morph_kernel_size", 3)
        self.declare_parameter("dilate_iterations", 0)

        # Connected component filtering
        self.declare_parameter("use_component_filter", True)
        self.declare_parameter("min_component_area", 20)
        self.declare_parameter("max_component_area", 2200)
        self.declare_parameter("max_component_width", 120)
        self.declare_parameter("min_component_height", 8)
        self.declare_parameter("min_component_aspect", 0.35)
        self.declare_parameter("max_component_fill_ratio", 0.75)
        self.declare_parameter("min_component_y_ratio", 0.48)

        # Hough candidate filtering
        self.declare_parameter("use_hough_fit", True)
        self.declare_parameter("hough_threshold", 18)
        self.declare_parameter("hough_min_line_length", 25)
        self.declare_parameter("hough_max_line_gap", 20)
        self.declare_parameter("min_segment_angle_deg", 18.0)
        self.declare_parameter("max_segment_angle_deg", 88.0)
        self.declare_parameter("min_segment_length_px", 22.0)
        self.declare_parameter("min_segment_y_ratio", 0.48)
        self.declare_parameter("max_segments_per_side", 10)

        # Fitting
        self.declare_parameter("min_lane_pixels", 80)
        self.declare_parameter("min_y_span_px", 55)
        self.declare_parameter("split_margin_px", 10)
        self.declare_parameter("max_abs_dxdy", 1.8)
        self.declare_parameter("lane_width_px", 360.0)
        self.declare_parameter("fit_top_y_ratio", 0.62)
        self.declare_parameter("fit_bottom_y_ratio", 0.95)

        # Confidence and temporal consistency
        self.declare_parameter("max_confident_heading_deg", 25.0)
        self.declare_parameter("max_output_heading_deg", 38.0)
        self.declare_parameter("max_confident_offset_ratio", 0.33)
        self.declare_parameter("smoothing_alpha", 0.35)
        self.declare_parameter("max_offset_jump_px", 120.0)
        self.declare_parameter("max_heading_jump_deg", 25.0)

        # LDv3 consistency and confidence gates
        self.declare_parameter("enforce_perspective_consistency", True)
        self.declare_parameter("perspective_slope_tolerance", 0.18)
        self.declare_parameter("min_lane_width_px", 180.0)
        self.declare_parameter("max_lane_width_px", 650.0)
        self.declare_parameter("single_side_confidence_cap", 0.58)
        self.declare_parameter("invalid_width_confidence_cap", 0.35)
        self.declare_parameter("min_stable_frames", 3)
        self.declare_parameter("temporal_confidence_cap", 0.65)

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

        self.prev_center_offset_px: Optional[float] = None
        self.prev_heading_error_deg: Optional[float] = None
        self.stable_lane_frames = 0

        self.create_subscription(CompressedImage, self.rgb_topic, self.rgb_callback, 10)
        self.create_subscription(Image, self.semantic_topic, self.semantic_callback, 10)

        self.lane_mask_pub = self.create_publisher(CompressedImage, self.lane_mask_topic, 10)
        self.lane_overlay_pub = self.create_publisher(CompressedImage, self.lane_overlay_topic, 10)
        self.status_pub = self.create_publisher(String, self.lane_status_topic, 10)

        self.get_logger().info("Lane detection node v2 started")
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
        left_fit, right_fit, fit_debug = self.fit_lane_sides(lane_mask)

        status = self.compute_status(
            image_width=w,
            image_height=h,
            left_fit=left_fit,
            right_fit=right_fit,
            lane_mask=lane_mask,
            fit_debug=fit_debug,
        )

        self.publish_mask(msg, lane_mask)
        self.publish_overlay(msg, bgr, lane_mask, left_fit, right_fit, status, fit_debug)
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
            semantic = semantic[:, :, 0]

        semantic = cv2.resize(semantic, (w, h), interpolation=cv2.INTER_NEAREST)

        road_class_id = int(self.get_parameter("road_class_id").value)
        road_mask = np.zeros((h, w), dtype=np.uint8)
        road_mask[semantic == road_class_id] = 255

        road_area_ratio = float(np.mean(road_mask > 0))
        min_road_area_ratio = float(self.get_parameter("min_road_area_ratio").value)

        if road_area_ratio < min_road_area_ratio:
            self.get_logger().warning(
                f"Road semantic ROI too small ({road_area_ratio:.3f}); falling back to image ROI only"
            )
            return np.full((h, w), 255, dtype=np.uint8)

        iterations = int(self.get_parameter("road_dilate_iterations").value)
        if iterations > 0:
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
            road_mask = cv2.dilate(road_mask, kernel, iterations=iterations)

        return road_mask

    def detect_adaptive_white(self, bgr: np.ndarray, hls: np.ndarray) -> np.ndarray:
        if not bool(self.get_parameter("use_adaptive_white").value):
            h, w = bgr.shape[:2]
            return np.zeros((h, w), dtype=np.uint8)

        l_channel = hls[:, :, 1]
        s_channel = hls[:, :, 2]

        work = l_channel.copy()

        if bool(self.get_parameter("use_clahe").value):
            clip_limit = float(self.get_parameter("clahe_clip_limit").value)
            tile_size = int(self.get_parameter("clahe_tile_grid_size").value)
            tile_size = max(2, tile_size)

            clahe = cv2.createCLAHE(
                clipLimit=clip_limit,
                tileGridSize=(tile_size, tile_size),
            )
            work = clahe.apply(work)

        block_size = int(self.get_parameter("adaptive_white_block_size").value)
        block_size = max(3, block_size)
        if block_size % 2 == 0:
            block_size += 1

        adaptive_c = int(self.get_parameter("adaptive_white_c").value)

        adaptive = cv2.adaptiveThreshold(
            work,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            block_size,
            adaptive_c,
        )

        min_l = int(self.get_parameter("adaptive_white_min_l").value)
        s_max = int(self.get_parameter("adaptive_white_s_max").value)

        min_l_mask = cv2.inRange(l_channel, min_l, 255)
        low_saturation_mask = cv2.inRange(s_channel, 0, s_max)

        adaptive_white = cv2.bitwise_and(adaptive, min_l_mask)
        adaptive_white = cv2.bitwise_and(adaptive_white, low_saturation_mask)

        if bool(self.get_parameter("adaptive_white_use_shadow_gate").value):
            shadow_l_max = int(self.get_parameter("adaptive_white_shadow_l_max").value)
            shadow_mask = cv2.inRange(l_channel, 0, shadow_l_max)
            adaptive_white = cv2.bitwise_and(adaptive_white, shadow_mask)

        return adaptive_white

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

        yellow_mask = cv2.inRange(
            hls,
            np.array([
                int(self.get_parameter("yellow_h_min").value),
                int(self.get_parameter("yellow_l_min").value),
                int(self.get_parameter("yellow_s_min").value),
            ], dtype=np.uint8),
            np.array([
                int(self.get_parameter("yellow_h_max").value),
                int(self.get_parameter("yellow_l_max").value),
                int(self.get_parameter("yellow_s_max").value),
            ], dtype=np.uint8),
        )

        adaptive_white_mask = self.detect_adaptive_white(bgr, hls)

        lane_mask = cv2.bitwise_or(white_mask, yellow_mask)
        lane_mask = cv2.bitwise_or(lane_mask, adaptive_white_mask)

        lane_mask = cv2.bitwise_and(lane_mask, roi_mask)
        lane_mask = cv2.bitwise_and(lane_mask, road_mask)

        if bool(self.get_parameter("use_canny_edges").value):
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(
                gray,
                int(self.get_parameter("canny_low").value),
                int(self.get_parameter("canny_high").value),
            )
            edge_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            edges = cv2.dilate(edges, edge_kernel, iterations=1)
            lane_mask = cv2.bitwise_and(lane_mask, edges)

        kernel_size = int(self.get_parameter("morph_kernel_size").value)
        kernel_size = max(1, kernel_size)
        if kernel_size % 2 == 0:
            kernel_size += 1

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_size, kernel_size))

        lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_OPEN, kernel)
        lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_CLOSE, kernel)

        dilate_iterations = int(self.get_parameter("dilate_iterations").value)
        if dilate_iterations > 0:
            lane_mask = cv2.dilate(lane_mask, kernel, iterations=dilate_iterations)

        lane_mask = self.filter_lane_components(lane_mask)

        return lane_mask

    def filter_lane_components(self, lane_mask: np.ndarray) -> np.ndarray:
        if not bool(self.get_parameter("use_component_filter").value):
            return lane_mask

        h, _ = lane_mask.shape[:2]

        min_area = int(self.get_parameter("min_component_area").value)
        max_area = int(self.get_parameter("max_component_area").value)
        max_width = int(self.get_parameter("max_component_width").value)
        min_height = int(self.get_parameter("min_component_height").value)
        min_aspect = float(self.get_parameter("min_component_aspect").value)
        max_fill = float(self.get_parameter("max_component_fill_ratio").value)
        min_y = int(h * float(self.get_parameter("min_component_y_ratio").value))

        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(lane_mask, connectivity=8)
        filtered = np.zeros_like(lane_mask)

        for label in range(1, num_labels):
            x = int(stats[label, cv2.CC_STAT_LEFT])
            y = int(stats[label, cv2.CC_STAT_TOP])
            bw = int(stats[label, cv2.CC_STAT_WIDTH])
            bh = int(stats[label, cv2.CC_STAT_HEIGHT])
            area = int(stats[label, cv2.CC_STAT_AREA])

            if area < min_area or area > max_area:
                continue
            if y + bh < min_y:
                continue
            if bw > max_width:
                continue
            if bh < min_height:
                continue

            aspect = bh / max(1.0, float(bw))
            if aspect < min_aspect:
                continue

            fill_ratio = area / max(1.0, float(bw * bh))
            if fill_ratio > max_fill:
                continue

            filtered[labels == label] = 255

        return filtered

    def fit_lane_sides(self, lane_mask: np.ndarray):
        if bool(self.get_parameter("use_hough_fit").value):
            return self.fit_lane_sides_hough(lane_mask)

        left_fit, right_fit = self.fit_lane_sides_pixels(lane_mask)
        return left_fit, right_fit, {
            "fit_method": "pixels",
            "accepted_segments": 0,
            "left_segments": 0,
            "right_segments": 0,
            "total_segment_length": 0.0,
        }

    def fit_lane_sides_pixels(self, lane_mask: np.ndarray):
        h, w = lane_mask.shape[:2]
        ys, xs = np.nonzero(lane_mask > 0)

        if len(xs) == 0:
            return None, None

        mid_x = w // 2
        split_margin = int(self.get_parameter("split_margin_px").value)

        left_selection = xs < (mid_x - split_margin)
        right_selection = xs > (mid_x + split_margin)

        left_fit = self.fit_side_from_points(xs[left_selection], ys[left_selection], "left")
        right_fit = self.fit_side_from_points(xs[right_selection], ys[right_selection], "right")

        return left_fit, right_fit

    def fit_lane_sides_hough(self, lane_mask: np.ndarray):
        h, w = lane_mask.shape[:2]
        y_top = int(h * float(self.get_parameter("fit_top_y_ratio").value))
        y_bottom = int(h * float(self.get_parameter("fit_bottom_y_ratio").value))

        lines = cv2.HoughLinesP(
            lane_mask,
            rho=1,
            theta=np.pi / 180.0,
            threshold=int(self.get_parameter("hough_threshold").value),
            minLineLength=int(self.get_parameter("hough_min_line_length").value),
            maxLineGap=int(self.get_parameter("hough_max_line_gap").value),
        )

        debug = {
            "fit_method": "hough",
            "raw_segments": 0,
            "accepted_segments": 0,
            "left_segments": 0,
            "right_segments": 0,
            "total_segment_length": 0.0,
            "segments": [],
        }

        if lines is None:
            return None, None, debug

        debug["raw_segments"] = int(len(lines))

        min_angle = float(self.get_parameter("min_segment_angle_deg").value)
        max_angle = float(self.get_parameter("max_segment_angle_deg").value)
        min_length = float(self.get_parameter("min_segment_length_px").value)
        min_y = int(h * float(self.get_parameter("min_segment_y_ratio").value))
        max_abs_dxdy = float(self.get_parameter("max_abs_dxdy").value)
        split_margin = int(self.get_parameter("split_margin_px").value)
        max_segments = int(self.get_parameter("max_segments_per_side").value)

        mid_x = 0.5 * w
        left_candidates = []
        right_candidates = []

        for raw in lines:
            x1, y1, x2, y2 = [float(v) for v in raw[0]]

            dx = x2 - x1
            dy = y2 - y1
            length = math.hypot(dx, dy)

            if length < min_length:
                continue

            if max(y1, y2) < min_y:
                continue

            angle = abs(math.degrees(math.atan2(dy, dx)))
            if angle < min_angle or angle > max_angle:
                continue

            if abs(dy) < 1.0:
                continue

            # Fit x = a*y + b for the segment.
            a = dx / dy
            b = x1 - a * y1

            if abs(a) > max_abs_dxdy:
                continue

            x_bottom = a * y_bottom + b
            x_top = a * y_top + b

            if x_bottom < 0 or x_bottom >= w:
                continue

            score = length * (0.5 + 0.5 * max(y1, y2) / max(1.0, h))

            candidate = {
                "x1": x1,
                "y1": y1,
                "x2": x2,
                "y2": y2,
                "a": a,
                "b": b,
                "x_bottom": x_bottom,
                "x_top": x_top,
                "length": length,
                "angle": angle,
                "score": score,
            }

            side = None
            if x_bottom < mid_x - split_margin:
                side = "left"
            elif x_bottom > mid_x + split_margin:
                side = "right"
            else:
                continue

            if bool(self.get_parameter("enforce_perspective_consistency").value):
                slope_tolerance = float(self.get_parameter("perspective_slope_tolerance").value)

                # Image coordinates: y grows downward.
                # A left lane boundary normally has a <= 0 because it converges
                # toward the image center when moving upward.
                # A right lane boundary normally has a >= 0.
                #
                # The tolerance keeps near-vertical straight lanes valid.
                if side == "left" and a > slope_tolerance:
                    continue
                if side == "right" and a < -slope_tolerance:
                    continue

            candidate["side"] = side

            if side == "left":
                left_candidates.append(candidate)
            else:
                right_candidates.append(candidate)

        left_candidates = sorted(left_candidates, key=lambda c: c["score"], reverse=True)[:max_segments]
        right_candidates = sorted(right_candidates, key=lambda c: c["score"], reverse=True)[:max_segments]

        debug["accepted_segments"] = len(left_candidates) + len(right_candidates)
        debug["left_segments"] = len(left_candidates)
        debug["right_segments"] = len(right_candidates)
        debug["total_segment_length"] = round(
            sum(c["length"] for c in left_candidates + right_candidates),
            2,
        )
        debug["segments"] = left_candidates + right_candidates

        left_fit = self.fit_side_from_segments(left_candidates, "left")
        right_fit = self.fit_side_from_segments(right_candidates, "right")

        return left_fit, right_fit, debug

    def fit_side_from_segments(self, candidates, side: str):
        if not candidates:
            return None

        xs = []
        ys = []

        for c in candidates:
            xs.extend([c["x1"], c["x2"]])
            ys.extend([c["y1"], c["y2"]])

        return self.fit_side_from_points(np.array(xs), np.array(ys), side, len(candidates))

    def fit_side_from_points(self, xs: np.ndarray, ys: np.ndarray, side: str, segment_count: int = 0):
        min_lane_pixels = int(self.get_parameter("min_lane_pixels").value)
        min_y_span_px = int(self.get_parameter("min_y_span_px").value)
        max_abs_dxdy = float(self.get_parameter("max_abs_dxdy").value)

        if segment_count == 0 and len(xs) < min_lane_pixels:
            return None

        if len(xs) < 2:
            return None

        if int(np.max(ys) - np.min(ys)) < min_y_span_px:
            return None

        try:
            # Fit x = a*y + b.
            a, b = np.polyfit(ys.astype(np.float32), xs.astype(np.float32), deg=1)
        except np.linalg.LinAlgError:
            return None

        if abs(float(a)) > max_abs_dxdy:
            return None

        return {
            "side": side,
            "a": float(a),
            "b": float(b),
            "pixels": int(len(xs)),
            "segments": int(segment_count),
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
        fit_debug: dict,
    ) -> dict:
        y_top = int(image_height * float(self.get_parameter("fit_top_y_ratio").value))
        y_bottom = int(image_height * float(self.get_parameter("fit_bottom_y_ratio").value))
        lane_width_px = float(self.get_parameter("lane_width_px").value)

        left_visible = left_fit is not None
        right_visible = right_fit is not None

        center_bottom = None
        center_top = None
        inferred_side = None

        lane_width_bottom_px = None
        lane_width_top_px = None
        lane_width_valid = None

        if left_visible and right_visible:
            left_bottom = self.x_at_y(left_fit, y_bottom, image_width)
            right_bottom = self.x_at_y(right_fit, y_bottom, image_width)
            left_top = self.x_at_y(left_fit, y_top, image_width)
            right_top = self.x_at_y(right_fit, y_top, image_width)

            lane_width_bottom_px = float(right_bottom - left_bottom)
            lane_width_top_px = float(right_top - left_top)

            min_lane_width = float(self.get_parameter("min_lane_width_px").value)
            max_lane_width = float(self.get_parameter("max_lane_width_px").value)

            lane_width_valid = (
                min_lane_width <= lane_width_bottom_px <= max_lane_width
                and lane_width_top_px > 0.0
                and lane_width_top_px <= lane_width_bottom_px * 1.25
            )

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
            self.stable_lane_frames = 0
            return {
                "lane_detected": False,
                "left_lane_visible": False,
                "right_lane_visible": False,
                "inferred_side": None,
                "center_offset_px": None,
                "heading_error_deg": None,
                "confidence": 0.0,
                "lane_pixels": total_lane_pixels,
                "stable_lane_frames": self.stable_lane_frames,
                "lane_width_bottom_px": lane_width_bottom_px,
                "lane_width_top_px": lane_width_top_px,
                "lane_width_valid": lane_width_valid,
                "fit_debug": fit_debug,
            }

        image_center = image_width * 0.5
        center_offset_px = float(center_bottom - image_center)

        dx = float(center_top - center_bottom)
        dy = float(y_bottom - y_top)
        heading_error_deg = float(math.degrees(math.atan2(dx, dy)))

        max_output_heading = float(self.get_parameter("max_output_heading_deg").value)
        if abs(heading_error_deg) > max_output_heading:
            confidence = 0.0
            lane_detected = False
            fit_debug["confidence_caps"] = ["max_output_heading_exceeded"]
        else:
            confidence = self.compute_confidence(
                image_width=image_width,
                left_visible=left_visible,
                right_visible=right_visible,
                center_offset_px=center_offset_px,
                heading_error_deg=heading_error_deg,
                fit_debug=fit_debug,
                total_lane_pixels=total_lane_pixels,
                lane_width_valid=lane_width_valid,
            )
            lane_detected = confidence > 0.10

        center_offset_px, heading_error_deg, confidence = self.apply_temporal_smoothing(
            center_offset_px,
            heading_error_deg,
            confidence,
        )

        if confidence > 0.10:
            self.prev_center_offset_px = center_offset_px
            self.prev_heading_error_deg = heading_error_deg
        else:
            lane_detected = False

        return {
            "lane_detected": lane_detected,
            "left_lane_visible": left_visible,
            "right_lane_visible": right_visible,
            "inferred_side": inferred_side,
            "center_offset_px": round(center_offset_px, 2),
            "heading_error_deg": round(heading_error_deg, 2),
            "confidence": round(float(confidence), 3),
            "lane_pixels": total_lane_pixels,
            "stable_lane_frames": self.stable_lane_frames,
            "lane_width_bottom_px": None if lane_width_bottom_px is None else round(lane_width_bottom_px, 2),
            "lane_width_top_px": None if lane_width_top_px is None else round(lane_width_top_px, 2),
            "lane_width_valid": lane_width_valid,
            "fit_debug": fit_debug,
        }

    def compute_confidence(
        self,
        image_width: int,
        left_visible: bool,
        right_visible: bool,
        center_offset_px: float,
        heading_error_deg: float,
        fit_debug: dict,
        total_lane_pixels: int,
        lane_width_valid,
    ) -> float:
        confidence_caps = []

        if fit_debug.get("fit_method") == "hough":
            total_length = float(fit_debug.get("total_segment_length", 0.0))
            accepted_segments = int(fit_debug.get("accepted_segments", 0))

            length_score = min(1.0, total_length / 220.0)
            segment_score = min(1.0, accepted_segments / 4.0)
            evidence_score = 0.65 * length_score + 0.35 * segment_score
        else:
            evidence_score = min(1.0, total_lane_pixels / 400.0)

        if left_visible and right_visible:
            visibility_score = 1.0
        else:
            visibility_score = 0.55
            confidence_caps.append("single_side_detection")

        max_conf_heading = float(self.get_parameter("max_confident_heading_deg").value)
        max_output_heading = float(self.get_parameter("max_output_heading_deg").value)
        abs_heading = abs(heading_error_deg)

        if abs_heading <= max_conf_heading:
            heading_score = 1.0
        else:
            heading_score = max(
                0.0,
                1.0 - (abs_heading - max_conf_heading) / max(1.0, max_output_heading - max_conf_heading),
            )

        max_conf_offset = float(self.get_parameter("max_confident_offset_ratio").value) * image_width
        abs_offset = abs(center_offset_px)

        if abs_offset <= max_conf_offset:
            offset_score = 1.0
        else:
            offset_score = max(0.25, 1.0 - (abs_offset - max_conf_offset) / max(1.0, image_width * 0.25))

        confidence = visibility_score * evidence_score * heading_score * offset_score

        if not (left_visible and right_visible):
            cap = float(self.get_parameter("single_side_confidence_cap").value)
            confidence = min(confidence, cap)

        if lane_width_valid is False:
            cap = float(self.get_parameter("invalid_width_confidence_cap").value)
            confidence = min(confidence, cap)
            confidence_caps.append("invalid_lane_width")

        fit_debug["confidence_components"] = {
            "evidence_score": round(float(evidence_score), 3),
            "visibility_score": round(float(visibility_score), 3),
            "heading_score": round(float(heading_score), 3),
            "offset_score": round(float(offset_score), 3),
        }
        fit_debug["confidence_caps"] = confidence_caps

        return float(np.clip(confidence, 0.0, 1.0))

    def apply_temporal_smoothing(
        self,
        center_offset_px: float,
        heading_error_deg: float,
        confidence: float,
    ):
        if self.prev_center_offset_px is None or self.prev_heading_error_deg is None:
            self.stable_lane_frames = 1 if confidence > 0.10 else 0

            min_stable_frames = int(self.get_parameter("min_stable_frames").value)
            if self.stable_lane_frames < min_stable_frames:
                confidence = min(
                    confidence,
                    float(self.get_parameter("temporal_confidence_cap").value),
                )

            return center_offset_px, heading_error_deg, confidence

        offset_jump = abs(center_offset_px - self.prev_center_offset_px)
        heading_jump = abs(heading_error_deg - self.prev_heading_error_deg)

        max_offset_jump = float(self.get_parameter("max_offset_jump_px").value)
        max_heading_jump = float(self.get_parameter("max_heading_jump_deg").value)

        if offset_jump > max_offset_jump or heading_jump > max_heading_jump:
            self.stable_lane_frames = 1
            confidence *= 0.45
        else:
            self.stable_lane_frames += 1

        min_stable_frames = int(self.get_parameter("min_stable_frames").value)
        if self.stable_lane_frames < min_stable_frames:
            confidence = min(
                confidence,
                float(self.get_parameter("temporal_confidence_cap").value),
            )

        alpha = float(self.get_parameter("smoothing_alpha").value)
        alpha = float(np.clip(alpha, 0.0, 1.0))

        smoothed_offset = alpha * center_offset_px + (1.0 - alpha) * self.prev_center_offset_px
        smoothed_heading = alpha * heading_error_deg + (1.0 - alpha) * self.prev_heading_error_deg

        return smoothed_offset, smoothed_heading, confidence

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
        fit_debug: dict,
    ) -> None:
        if not bool(self.get_parameter("publish_debug_overlay").value):
            return

        overlay = bgr.copy()
        h, w = overlay.shape[:2]

        color_layer = np.zeros_like(overlay)
        color_layer[lane_mask > 0] = (0, 255, 255)
        overlay = cv2.addWeighted(overlay, 1.0, color_layer, 0.45, 0.0)

        # Draw accepted Hough segments in cyan.
        for segment in fit_debug.get("segments", []):
            cv2.line(
                overlay,
                (int(segment["x1"]), int(segment["y1"])),
                (int(segment["x2"]), int(segment["y2"])),
                (255, 255, 0),
                2,
            )

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
            f"conf={status.get('confidence')} | "
            f"seg={fit_debug.get('accepted_segments', 0)}"
        )

        cv2.putText(
            overlay,
            text,
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
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
