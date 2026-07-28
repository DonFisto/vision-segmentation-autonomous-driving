#!/usr/bin/env python3

import json
import math
from typing import Any, Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from perception_geometry import BEVSpec, CameraIPM, IPMConfig


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def score_from_error(error: float, full_error: float, floor: float = 0.0) -> float:
    if full_error <= 1e-6:
        return 1.0
    return clamp(1.0 - abs(error) / full_error, floor, 1.0)


class LaneDetectionNode(Node):
    """
    LDv4: BEV-first classical lane detector.

    Key idea:
      - keep LDv3.3 shadow-gated color/adaptive mask
      - warp mask to BEV
      - reject horizontal/intersection-like structures in BEV
      - select ego-lane left/right candidates in BEV
      - publish image-space and BEV-space selected geometry
    """

    def __init__(self) -> None:
        super().__init__("lane_detection_node")

        # Topics
        self.declare_parameter("image_topic", "/carla/rgb/image_raw/compressed")
        self.declare_parameter("semantic_mask_topic", "/perception/semantic_mask")
        self.declare_parameter("lane_mask_topic", "/perception/lane_mask/compressed")
        self.declare_parameter("lane_overlay_topic", "/perception/lane_overlay/compressed")
        self.declare_parameter("lane_status_topic", "/perception/lane_status")
        self.declare_parameter("lane_bev_debug_topic", "/perception/lane_bev_debug/compressed")

        # Semantic ROI
        self.declare_parameter("use_semantic_roi", True)
        self.declare_parameter("require_semantic_roi", False)
        self.declare_parameter("road_class_id", 0)
        self.declare_parameter("min_road_area_ratio", 0.01)
        self.declare_parameter("road_dilate_iterations", 2)

        # Image ROI
        self.declare_parameter("roi_x_min_ratio", 0.10)
        self.declare_parameter("roi_x_max_ratio", 0.90)
        self.declare_parameter("roi_y_min_ratio", 0.48)
        self.declare_parameter("roi_y_max_ratio", 0.97)

        # LDv3.3 strong white threshold
        self.declare_parameter("white_l_min", 190)
        self.declare_parameter("white_l_max", 255)
        self.declare_parameter("white_s_min", 0)
        self.declare_parameter("white_s_max", 70)

        # LDv3.3 yellow threshold
        self.declare_parameter("yellow_h_min", 12)
        self.declare_parameter("yellow_h_max", 40)
        self.declare_parameter("yellow_l_min", 80)
        self.declare_parameter("yellow_l_max", 255)
        self.declare_parameter("yellow_s_min", 95)
        self.declare_parameter("yellow_s_max", 255)

        # LDv3.3 shadow-gated adaptive white
        self.declare_parameter("use_adaptive_white", True)
        self.declare_parameter("use_clahe", True)
        self.declare_parameter("clahe_clip_limit", 1.5)
        self.declare_parameter("clahe_tile_grid_size", 8)
        self.declare_parameter("adaptive_white_block_size", 61)
        self.declare_parameter("adaptive_white_c", -10)
        self.declare_parameter("adaptive_white_min_l", 50)
        self.declare_parameter("adaptive_white_s_max", 120)
        self.declare_parameter("adaptive_white_use_shadow_gate", True)
        self.declare_parameter("adaptive_white_shadow_l_max", 125)

        # Optional edge gate
        self.declare_parameter("use_canny_edges", False)
        self.declare_parameter("canny_low", 40)
        self.declare_parameter("canny_high", 130)

        # Morphology
        self.declare_parameter("morph_kernel_size", 3)
        self.declare_parameter("dilate_iterations", 0)

        # Component filtering
        self.declare_parameter("use_component_filter", True)
        self.declare_parameter("min_component_area", 10)
        self.declare_parameter("max_component_area", 2600)
        self.declare_parameter("max_component_width", 150)
        self.declare_parameter("min_component_height", 5)
        self.declare_parameter("min_component_aspect", 0.16)
        self.declare_parameter("max_component_fill_ratio", 0.70)

        # Hough parameters. Kept with old names so the startup script remains compatible.
        self.declare_parameter("use_hough_fit", True)
        self.declare_parameter("hough_threshold", 16)
        self.declare_parameter("hough_min_line_length", 18)
        self.declare_parameter("hough_max_line_gap", 24)
        self.declare_parameter("min_segment_angle_deg", 22.0)
        self.declare_parameter("max_segment_angle_deg", 88.0)
        self.declare_parameter("min_segment_length_px", 16.0)
        self.declare_parameter("min_y_span_px", 32)
        self.declare_parameter("max_abs_dxdy", 2.0)
        self.declare_parameter("max_segments_per_side", 4)

        # Old compatibility parameters. Some are not directly used by LDv4,
        # but they are declared so existing launch/startup commands do not fail.
        self.declare_parameter("max_confident_heading_deg", 20.0)
        self.declare_parameter("max_output_heading_deg", 38.0)
        self.declare_parameter("enforce_perspective_consistency", True)
        self.declare_parameter("perspective_slope_tolerance", 0.18)
        self.declare_parameter("min_lane_width_px", 200.0)
        self.declare_parameter("max_lane_width_px", 600.0)
        self.declare_parameter("single_side_confidence_cap", 0.50)
        self.declare_parameter("invalid_width_confidence_cap", 0.35)
        self.declare_parameter("min_stable_frames", 3)
        self.declare_parameter("temporal_confidence_cap", 0.60)

        # BEV projection
        self.declare_parameter("bev_forward_m", 45.0)
        self.declare_parameter("bev_width_m", 16.0)
        self.declare_parameter("bev_resolution", 0.10)

        self.declare_parameter("src_bottom_y_ratio", 0.97)
        self.declare_parameter("src_top_y_ratio", 0.48)
        self.declare_parameter("src_bottom_left_x_ratio", 0.10)
        self.declare_parameter("src_bottom_right_x_ratio", 0.90)
        self.declare_parameter("src_top_left_x_ratio", 0.42)
        self.declare_parameter("src_top_right_x_ratio", 0.58)

        # BEV candidate selection
        self.declare_parameter("bev_fit_top_y_ratio", 0.08)
        self.declare_parameter("bev_max_angle_from_vertical_deg", 28.0)
        self.declare_parameter("bev_min_y_span_px", 30.0)
        self.declare_parameter("bev_max_abs_dxdy", 0.75)
        self.declare_parameter("bev_min_side_distance_m", 0.35)

        self.declare_parameter("expected_lane_width_m", 3.6)
        self.declare_parameter("min_lane_width_m", 2.3)
        self.declare_parameter("max_lane_width_m", 5.4)
        self.declare_parameter("pair_center_full_error_m", 3.0)
        self.declare_parameter("pair_width_full_error_m", 1.8)
        self.declare_parameter("pair_parallel_full_error", 0.45)
        self.declare_parameter("pair_heading_full_error_deg", 24.0)
        self.declare_parameter("min_pair_score", 0.18)

        # Temporal/confidence
        self.declare_parameter("smoothing_alpha", 0.35)
        self.declare_parameter("max_offset_jump_px", 150.0)
        self.declare_parameter("max_heading_jump_deg", 30.0)

        # Debug/output
        self.declare_parameter("overlay_jpeg_quality", 85)
        self.declare_parameter("bev_debug_jpeg_quality", 90)
        self.declare_parameter("draw_all_bev_candidates", True)

        # Shared camera/IPM geometry utility.
        # This centralizes the image <-> BEV <-> metric conversions that used
        # to be duplicated inside this node.
        self.ipm = self.make_ipm()

        self.bridge = CvBridge()
        self.latest_semantic_mask: Optional[np.ndarray] = None

        self.prev_offset_px: Optional[float] = None
        self.prev_heading_deg: Optional[float] = None
        self.stable_frames = 0

        image_topic = str(self.p("image_topic"))
        semantic_topic = str(self.p("semantic_mask_topic"))

        self.image_sub = self.create_subscription(
            CompressedImage,
            image_topic,
            self.image_cb,
            qos_profile_sensor_data,
        )
        self.semantic_sub = self.create_subscription(
            Image,
            semantic_topic,
            self.semantic_cb,
            qos_profile_sensor_data,
        )

        self.mask_pub = self.create_publisher(
            CompressedImage,
            str(self.p("lane_mask_topic")),
            10,
        )
        self.overlay_pub = self.create_publisher(
            CompressedImage,
            str(self.p("lane_overlay_topic")),
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            str(self.p("lane_status_topic")),
            10,
        )
        self.bev_debug_pub = self.create_publisher(
            CompressedImage,
            str(self.p("lane_bev_debug_topic")),
            10,
        )

        self.get_logger().info("LDv4 BEV-first lane detector started")
        self.get_logger().info(f"Subscribing image: {image_topic}")
        self.get_logger().info(f"Subscribing semantic mask: {semantic_topic}")
        self.get_logger().info(f"Publishing overlay: {self.p('lane_overlay_topic')}")
        self.get_logger().info(f"Publishing BEV debug: {self.p('lane_bev_debug_topic')}")

    def p(self, name: str) -> Any:
        return self.get_parameter(name).value

    def make_ipm(self) -> CameraIPM:
        return CameraIPM(
            IPMConfig(
                src_bottom_y_ratio=float(self.p("src_bottom_y_ratio")),
                src_top_y_ratio=float(self.p("src_top_y_ratio")),
                src_bottom_left_x_ratio=float(self.p("src_bottom_left_x_ratio")),
                src_bottom_right_x_ratio=float(self.p("src_bottom_right_x_ratio")),
                src_top_left_x_ratio=float(self.p("src_top_left_x_ratio")),
                src_top_right_x_ratio=float(self.p("src_top_right_x_ratio")),
            ),
            BEVSpec(
                forward_m=float(self.p("bev_forward_m")),
                backward_m=0.0,
                width_m=float(self.p("bev_width_m")),
                resolution_m=float(self.p("bev_resolution")),
            ),
        )

    def semantic_cb(self, msg: Image) -> None:
        try:
            mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if mask.ndim == 3:
                mask = mask[:, :, 0]
            self.latest_semantic_mask = mask.astype(np.uint8)
        except Exception as exc:
            self.get_logger().warn(f"Could not decode semantic mask: {exc}")

    def image_cb(self, msg: CompressedImage) -> None:
        frame = self.decode_compressed(msg)
        if frame is None:
            return

        lane_mask, overlay, bev_debug, status = self.process_frame(frame)

        self.publish_compressed(
            self.mask_pub,
            lane_mask,
            msg.header,
            ext=".png",
            fmt="png",
        )
        self.publish_compressed(
            self.overlay_pub,
            overlay,
            msg.header,
            ext=".jpg",
            fmt="jpeg",
            jpeg_quality=int(self.p("overlay_jpeg_quality")),
        )
        self.publish_compressed(
            self.bev_debug_pub,
            bev_debug,
            msg.header,
            ext=".jpg",
            fmt="jpeg",
            jpeg_quality=int(self.p("bev_debug_jpeg_quality")),
        )

        status_msg = String()
        status_msg.data = json.dumps(status, separators=(",", ":"))
        self.status_pub.publish(status_msg)

    def decode_compressed(self, msg: CompressedImage) -> Optional[np.ndarray]:
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8)
            return cv2.imdecode(arr, cv2.IMREAD_COLOR)
        except Exception as exc:
            self.get_logger().warn(f"Could not decode compressed image: {exc}")
            return None

    def publish_compressed(
        self,
        pub,
        image: np.ndarray,
        header,
        ext: str,
        fmt: str,
        jpeg_quality: int = 85,
    ) -> None:
        params = []
        if ext.lower() in [".jpg", ".jpeg"]:
            params = [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)]

        ok, encoded = cv2.imencode(ext, image, params)
        if not ok:
            self.get_logger().warn(f"Could not encode {fmt} image")
            return

        out = CompressedImage()
        out.header = header
        out.format = fmt
        out.data = encoded.tobytes()
        pub.publish(out)

    def build_roi_mask(self, h: int, w: int) -> np.ndarray:
        x0 = int(w * float(self.p("roi_x_min_ratio")))
        x1 = int(w * float(self.p("roi_x_max_ratio")))
        y0 = int(h * float(self.p("roi_y_min_ratio")))
        y1 = int(h * float(self.p("roi_y_max_ratio")))

        roi = np.zeros((h, w), dtype=np.uint8)
        roi[y0:y1, x0:x1] = 255
        return roi

    def build_semantic_road_mask(self, h: int, w: int) -> Optional[np.ndarray]:
        if not bool(self.p("use_semantic_roi")):
            return None

        if self.latest_semantic_mask is None:
            return None

        sem = self.latest_semantic_mask
        if sem.shape[:2] != (h, w):
            sem = cv2.resize(sem, (w, h), interpolation=cv2.INTER_NEAREST)

        road_class_id = int(self.p("road_class_id"))
        road = (sem == road_class_id).astype(np.uint8) * 255

        if np.count_nonzero(road) < float(self.p("min_road_area_ratio")) * h * w:
            return None

        dilate_iter = int(self.p("road_dilate_iterations"))
        if dilate_iter > 0:
            road = cv2.dilate(
                road,
                np.ones((5, 5), dtype=np.uint8),
                iterations=dilate_iter,
            )

        return road

    def detect_adaptive_white(self, bgr: np.ndarray, hls: np.ndarray) -> np.ndarray:
        h, w = bgr.shape[:2]

        if not bool(self.p("use_adaptive_white")):
            return np.zeros((h, w), dtype=np.uint8)

        l_channel = hls[:, :, 1]
        s_channel = hls[:, :, 2]
        work = l_channel.copy()

        if bool(self.p("use_clahe")):
            clip = float(self.p("clahe_clip_limit"))
            tile = max(2, int(self.p("clahe_tile_grid_size")))
            clahe = cv2.createCLAHE(clipLimit=clip, tileGridSize=(tile, tile))
            work = clahe.apply(work)

        block = max(3, int(self.p("adaptive_white_block_size")))
        if block % 2 == 0:
            block += 1

        adaptive = cv2.adaptiveThreshold(
            work,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            block,
            int(self.p("adaptive_white_c")),
        )

        min_l_mask = cv2.inRange(l_channel, int(self.p("adaptive_white_min_l")), 255)
        low_s_mask = cv2.inRange(s_channel, 0, int(self.p("adaptive_white_s_max")))

        out = cv2.bitwise_and(adaptive, min_l_mask)
        out = cv2.bitwise_and(out, low_s_mask)

        if bool(self.p("adaptive_white_use_shadow_gate")):
            shadow_mask = cv2.inRange(l_channel, 0, int(self.p("adaptive_white_shadow_l_max")))
            out = cv2.bitwise_and(out, shadow_mask)

        return out

    def detect_lane_mask(self, bgr: np.ndarray) -> tuple[np.ndarray, dict[str, int]]:
        h, w = bgr.shape[:2]
        hls = cv2.cvtColor(bgr, cv2.COLOR_BGR2HLS)

        white_mask = cv2.inRange(
            hls,
            np.array(
                [
                    0,
                    int(self.p("white_l_min")),
                    int(self.p("white_s_min")),
                ],
                dtype=np.uint8,
            ),
            np.array(
                [
                    179,
                    int(self.p("white_l_max")),
                    int(self.p("white_s_max")),
                ],
                dtype=np.uint8,
            ),
        )

        yellow_mask = cv2.inRange(
            hls,
            np.array(
                [
                    int(self.p("yellow_h_min")),
                    int(self.p("yellow_l_min")),
                    int(self.p("yellow_s_min")),
                ],
                dtype=np.uint8,
            ),
            np.array(
                [
                    int(self.p("yellow_h_max")),
                    int(self.p("yellow_l_max")),
                    int(self.p("yellow_s_max")),
                ],
                dtype=np.uint8,
            ),
        )

        adaptive_white = self.detect_adaptive_white(bgr, hls)

        lane_mask = cv2.bitwise_or(white_mask, yellow_mask)
        lane_mask = cv2.bitwise_or(lane_mask, adaptive_white)

        roi = self.build_roi_mask(h, w)
        lane_mask = cv2.bitwise_and(lane_mask, roi)

        road = self.build_semantic_road_mask(h, w)
        if road is not None:
            lane_mask = cv2.bitwise_and(lane_mask, road)
        elif bool(self.p("require_semantic_roi")):
            lane_mask[:] = 0

        if bool(self.p("use_canny_edges")):
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, int(self.p("canny_low")), int(self.p("canny_high")))
            edges = cv2.dilate(edges, np.ones((3, 3), dtype=np.uint8), iterations=1)
            lane_mask = cv2.bitwise_and(lane_mask, edges)

        k = max(1, int(self.p("morph_kernel_size")))
        if k > 1:
            kernel = np.ones((k, k), dtype=np.uint8)
            lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_OPEN, kernel, iterations=1)
            lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        dilate_iter = int(self.p("dilate_iterations"))
        if dilate_iter > 0:
            lane_mask = cv2.dilate(
                lane_mask,
                np.ones((k, k), dtype=np.uint8),
                iterations=dilate_iter,
            )

        debug = {
            "strong_white_pixels": int(np.count_nonzero(white_mask)),
            "yellow_pixels": int(np.count_nonzero(yellow_mask)),
            "adaptive_white_pixels": int(np.count_nonzero(adaptive_white)),
            "merged_pixels_before_components": int(np.count_nonzero(lane_mask)),
        }

        if bool(self.p("use_component_filter")):
            lane_mask = self.filter_components(lane_mask)

        debug["final_lane_pixels"] = int(np.count_nonzero(lane_mask))
        return lane_mask, debug

    def filter_components(self, mask: np.ndarray) -> np.ndarray:
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        out = np.zeros_like(mask)

        min_area = int(self.p("min_component_area"))
        max_area = int(self.p("max_component_area"))
        max_width = int(self.p("max_component_width"))
        min_height = int(self.p("min_component_height"))
        min_aspect = float(self.p("min_component_aspect"))
        max_fill = float(self.p("max_component_fill_ratio"))

        for label in range(1, num_labels):
            x, y, w, h, area = stats[label]

            if area < min_area or area > max_area:
                continue
            if w > max_width:
                continue
            if h < min_height:
                continue

            aspect = max(w, h) / max(1.0, float(min(w, h)))
            if aspect < min_aspect:
                continue

            fill = area / max(1.0, float(w * h))
            if fill > max_fill:
                continue

            out[labels == label] = 255

        return out

    def bev_shape(self) -> tuple[int, int]:
        self.ipm = self.make_ipm()
        return self.ipm.bev_spec.width_px, self.ipm.bev_spec.height_px

    def build_bev_homography(self, image_w: int, image_h: int) -> tuple[np.ndarray, np.ndarray, int, int]:
        self.ipm = self.make_ipm()
        image_to_bev, bev_to_image = self.ipm.homographies(image_w, image_h)
        return image_to_bev, bev_to_image, self.ipm.bev_spec.width_px, self.ipm.bev_spec.height_px

    def warp_to_bev(self, lane_mask: np.ndarray, h_mat: np.ndarray, bev_w: int, bev_h: int) -> np.ndarray:
        bev = self.ipm.warp_image_to_bev(
            lane_mask,
            h_mat,
            interpolation=cv2.INTER_NEAREST,
            border_value=0,
        )

        # Light BEV cleanup. This mainly removes isolated road-texture speckles.
        kernel = np.ones((3, 3), dtype=np.uint8)
        bev = cv2.morphologyEx(bev, cv2.MORPH_OPEN, kernel, iterations=1)
        return bev

    def bev_x_to_lateral(self, x: float, bev_w: int) -> float:
        _, lateral_m = self.ipm.bev_pixel_to_metric(float(x), 0.0)
        return float(lateral_m)

    def bev_y_to_forward(self, y: float, bev_h: int) -> float:
        forward_m, _ = self.ipm.bev_pixel_to_metric(0.0, float(y))
        return float(forward_m)

    def lateral_to_bev_x(self, lateral_m: float, bev_w: int) -> float:
        x_px, _ = self.ipm.metric_to_bev_pixel(0.0, float(lateral_m))
        return float(x_px)

    def extract_bev_candidates(self, bev_mask: np.ndarray) -> tuple[list[dict[str, Any]], list[dict[str, Any]], dict[str, Any]]:
        bev_h, bev_w = bev_mask.shape[:2]
        center_x = bev_w * 0.5

        lines = cv2.HoughLinesP(
            bev_mask,
            rho=1,
            theta=np.pi / 180.0,
            threshold=int(self.p("hough_threshold")),
            minLineLength=int(self.p("hough_min_line_length")),
            maxLineGap=int(self.p("hough_max_line_gap")),
        )

        raw_count = 0 if lines is None else len(lines)
        left: list[dict[str, Any]] = []
        right: list[dict[str, Any]] = []

        if lines is None:
            return left, right, {
                "raw_bev_hough_segments": 0,
                "accepted_bev_segments": 0,
                "candidate_count_left": 0,
                "candidate_count_right": 0,
            }

        max_angle_vertical = float(self.p("bev_max_angle_from_vertical_deg"))
        min_y_span = float(self.p("bev_min_y_span_px"))
        min_length = float(self.p("min_segment_length_px"))
        max_abs_dxdy = float(self.p("bev_max_abs_dxdy"))
        min_side_distance_px = float(self.p("bev_min_side_distance_m")) / float(self.p("bev_resolution"))
        max_per_side = int(self.p("max_segments_per_side"))

        fit_y_bottom = float(bev_h - 1)
        fit_y_top = float(bev_h) * float(self.p("bev_fit_top_y_ratio"))

        accepted = 0

        for raw in lines[:, 0, :]:
            x1, y1, x2, y2 = [float(v) for v in raw]

            dx = x2 - x1
            dy = y2 - y1
            length = math.hypot(dx, dy)
            y_span = abs(dy)

            if length < min_length:
                continue
            if y_span < min_y_span:
                continue

            # In BEV, lane boundaries should be close to vertical.
            angle_from_vertical = abs(math.degrees(math.atan2(dx, dy)))
            if angle_from_vertical > 90.0:
                angle_from_vertical = 180.0 - angle_from_vertical

            if abs(angle_from_vertical) > max_angle_vertical:
                continue

            abs_dxdy = abs(dx) / max(1.0, abs(dy))
            if abs_dxdy > max_abs_dxdy:
                continue

            if abs(dy) < 1e-6:
                continue

            # Fit x = a*y + b.
            a = dx / dy
            b = x1 - a * y1

            x_bottom = a * fit_y_bottom + b
            x_top = a * fit_y_top + b

            if not (-0.25 * bev_w <= x_bottom <= 1.25 * bev_w):
                continue
            if not (-0.25 * bev_w <= x_top <= 1.25 * bev_w):
                continue

            if x_bottom < center_x - min_side_distance_px:
                side = "left"
            elif x_bottom > center_x + min_side_distance_px:
                side = "right"
            else:
                continue

            candidate = {
                "side": side,
                "a": float(a),
                "b": float(b),
                "x_bottom": float(x_bottom),
                "y_bottom": float(fit_y_bottom),
                "x_top": float(x_top),
                "y_top": float(fit_y_top),
                "length": float(length),
                "angle_from_vertical_deg": float(angle_from_vertical),
                "raw": [int(x1), int(y1), int(x2), int(y2)],
            }

            accepted += 1
            if side == "left":
                left.append(candidate)
            else:
                right.append(candidate)

        def sort_key(c: dict[str, Any]) -> tuple[float, float]:
            lateral_abs = abs(self.bev_x_to_lateral(float(c["x_bottom"]), bev_w))
            return (-float(c["length"]), lateral_abs)

        left = sorted(left, key=sort_key)[:max_per_side]
        right = sorted(right, key=sort_key)[:max_per_side]

        debug = {
            "raw_bev_hough_segments": int(raw_count),
            "accepted_bev_segments": int(accepted),
            "candidate_count_left": int(len(left)),
            "candidate_count_right": int(len(right)),
        }

        return left, right, debug

    def score_pair(self, left: dict[str, Any], right: dict[str, Any], bev_w: int, bev_h: int) -> dict[str, Any]:
        lb = float(left["x_bottom"])
        rb = float(right["x_bottom"])
        lt = float(left["x_top"])
        rt = float(right["x_top"])

        left_lat_b = self.bev_x_to_lateral(lb, bev_w)
        right_lat_b = self.bev_x_to_lateral(rb, bev_w)
        left_lat_t = self.bev_x_to_lateral(lt, bev_w)
        right_lat_t = self.bev_x_to_lateral(rt, bev_w)

        width_bottom = right_lat_b - left_lat_b
        width_top = right_lat_t - left_lat_t

        center_bottom = 0.5 * (left_lat_b + right_lat_b)
        center_top = 0.5 * (left_lat_t + right_lat_t)

        expected_width = float(self.p("expected_lane_width_m"))
        min_width = float(self.p("min_lane_width_m"))
        max_width = float(self.p("max_lane_width_m"))

        width_valid = (
            min_width <= width_bottom <= max_width
            and min_width * 0.6 <= width_top <= max_width * 1.25
        )

        center_score = score_from_error(
            center_bottom,
            float(self.p("pair_center_full_error_m")),
            floor=0.0,
        )

        width_score = score_from_error(
            width_bottom - expected_width,
            float(self.p("pair_width_full_error_m")),
            floor=0.0,
        )
        if not width_valid:
            width_score *= 0.25

        width_consistency = score_from_error(
            width_top - width_bottom,
            float(self.p("pair_width_full_error_m")),
            floor=0.0,
        )

        parallel_score = score_from_error(
            float(left["a"]) - float(right["a"]),
            float(self.p("pair_parallel_full_error")),
            floor=0.0,
        )

        forward_bottom = self.bev_y_to_forward(float(left["y_bottom"]), bev_h)
        forward_top = self.bev_y_to_forward(float(left["y_top"]), bev_h)
        df = max(1e-3, forward_top - forward_bottom)
        heading_deg = math.degrees(math.atan2(center_top - center_bottom, df))
        heading_score = score_from_error(
            heading_deg,
            float(self.p("pair_heading_full_error_deg")),
            floor=0.0,
        )

        side_score = 1.0 if left_lat_b < 0.0 < right_lat_b else 0.25

        evidence_score = clamp(
            (float(left["length"]) + float(right["length"])) / max(1.0, 0.8 * bev_h),
            0.0,
            1.0,
        )

        temporal_score = 1.0
        if self.prev_offset_px is not None and self.prev_heading_deg is not None:
            # Convert center lateral error to approximate image-px using old broad image scale.
            approximate_offset_px = center_bottom / max(1e-3, float(self.p("bev_width_m"))) * 800.0
            temporal_offset_score = score_from_error(
                approximate_offset_px - self.prev_offset_px,
                float(self.p("max_offset_jump_px")),
                floor=0.0,
            )
            temporal_heading_score = score_from_error(
                heading_deg - self.prev_heading_deg,
                float(self.p("max_heading_jump_deg")),
                floor=0.0,
            )
            temporal_score = 0.55 * temporal_offset_score + 0.45 * temporal_heading_score

        pair_score = (
            0.24 * center_score
            + 0.22 * width_score
            + 0.14 * width_consistency
            + 0.14 * parallel_score
            + 0.10 * heading_score
            + 0.08 * side_score
            + 0.06 * evidence_score
            + 0.02 * temporal_score
        )

        return {
            "left": left,
            "right": right,
            "score": float(clamp(pair_score, 0.0, 1.0)),
            "center_bottom_m": float(center_bottom),
            "center_top_m": float(center_top),
            "heading_error_deg": float(heading_deg),
            "width_bottom_m": float(width_bottom),
            "width_top_m": float(width_top),
            "width_valid": bool(width_valid),
            "components": {
                "center_score": round(float(center_score), 3),
                "width_score": round(float(width_score), 3),
                "width_consistency": round(float(width_consistency), 3),
                "parallel_score": round(float(parallel_score), 3),
                "heading_score": round(float(heading_score), 3),
                "side_score": round(float(side_score), 3),
                "evidence_score": round(float(evidence_score), 3),
                "temporal_score": round(float(temporal_score), 3),
            },
        }

    def select_pair(
        self,
        left_candidates: list[dict[str, Any]],
        right_candidates: list[dict[str, Any]],
        bev_w: int,
        bev_h: int,
    ) -> tuple[Optional[dict[str, Any]], list[dict[str, Any]]]:
        pairs = []
        for left in left_candidates:
            for right in right_candidates:
                pairs.append(self.score_pair(left, right, bev_w, bev_h))

        pairs = sorted(pairs, key=lambda p: float(p["score"]), reverse=True)

        if not pairs:
            return None, pairs

        best = pairs[0]
        if float(best["score"]) < float(self.p("min_pair_score")):
            return None, pairs

        return best, pairs

    def select_single_side(
        self,
        left_candidates: list[dict[str, Any]],
        right_candidates: list[dict[str, Any]],
        bev_w: int,
    ) -> tuple[Optional[dict[str, Any]], Optional[dict[str, Any]], Optional[str], float]:
        expected_width_px = float(self.p("expected_lane_width_m")) / float(self.p("bev_resolution"))
        center_x = bev_w * 0.5

        best_left = None
        best_right = None
        best_score = 0.0
        source = None

        for cand in left_candidates:
            inferred_right = dict(cand)
            inferred_right["side"] = "right"
            inferred_right["x_bottom"] = float(cand["x_bottom"]) + expected_width_px
            inferred_right["x_top"] = float(cand["x_top"]) + expected_width_px

            center_bottom = 0.5 * (float(cand["x_bottom"]) + float(inferred_right["x_bottom"]))
            score = score_from_error(center_bottom - center_x, 0.35 * bev_w, floor=0.0)
            score = 0.65 * score + 0.35 * clamp(float(cand["length"]) / 180.0, 0.0, 1.0)

            if score > best_score:
                best_score = score
                best_left = cand
                best_right = inferred_right
                source = "left_only_inferred_right"

        for cand in right_candidates:
            inferred_left = dict(cand)
            inferred_left["side"] = "left"
            inferred_left["x_bottom"] = float(cand["x_bottom"]) - expected_width_px
            inferred_left["x_top"] = float(cand["x_top"]) - expected_width_px

            center_bottom = 0.5 * (float(cand["x_bottom"]) + float(inferred_left["x_bottom"]))
            score = score_from_error(center_bottom - center_x, 0.35 * bev_w, floor=0.0)
            score = 0.65 * score + 0.35 * clamp(float(cand["length"]) / 180.0, 0.0, 1.0)

            if score > best_score:
                best_score = score
                best_left = inferred_left
                best_right = cand
                source = "right_only_inferred_left"

        return best_left, best_right, source, float(best_score)

    def bev_line_to_image_endpoint(
        self,
        line: dict[str, Any],
        h_inv: np.ndarray,
    ) -> dict[str, float]:
        image_line = self.ipm.bev_line_to_image_line(
            {
                "x_bottom": float(line["x_bottom"]),
                "y_bottom": float(line["y_bottom"]),
                "x_top": float(line["x_top"]),
                "y_top": float(line["y_top"]),
            },
            h_inv,
        )

        return {
            "x_bottom": round(float(image_line["x_bottom"]), 2),
            "y_bottom": round(float(image_line["y_bottom"]), 2),
            "x_top": round(float(image_line["x_top"]), 2),
            "y_top": round(float(image_line["y_top"]), 2),
        }

    def bev_line_to_metric_endpoint(
        self,
        line: dict[str, Any],
        bev_w: int,
        bev_h: int,
    ) -> dict[str, float]:
        metric_line = self.ipm.bev_line_to_metric_line(
            {
                "x_bottom": float(line["x_bottom"]),
                "y_bottom": float(line["y_bottom"]),
                "x_top": float(line["x_top"]),
                "y_top": float(line["y_top"]),
            }
        )

        return {
            "forward_bottom_m": round(float(metric_line["forward_bottom_m"]), 3),
            "lateral_bottom_m": round(float(metric_line["lateral_bottom_m"]), 3),
            "forward_top_m": round(float(metric_line["forward_top_m"]), 3),
            "lateral_top_m": round(float(metric_line["lateral_top_m"]), 3),
        }

    def make_center_line(self, left: dict[str, Any], right: dict[str, Any]) -> dict[str, Any]:
        return {
            "x_bottom": 0.5 * (float(left["x_bottom"]) + float(right["x_bottom"])),
            "y_bottom": 0.5 * (float(left["y_bottom"]) + float(right["y_bottom"])),
            "x_top": 0.5 * (float(left["x_top"]) + float(right["x_top"])),
            "y_top": 0.5 * (float(left["y_top"]) + float(right["y_top"])),
        }

    def compute_image_offset_px(
        self,
        center_line: dict[str, Any],
        h_inv: np.ndarray,
        image_w: int,
    ) -> float:
        center_img = self.bev_line_to_image_endpoint(center_line, h_inv)
        return float(center_img["x_bottom"]) - image_w * 0.5

    def apply_temporal_confidence(
        self,
        offset_px: float,
        heading_deg: float,
        confidence: float,
    ) -> float:
        if self.prev_offset_px is None or self.prev_heading_deg is None:
            self.stable_frames = 1
        else:
            offset_jump = abs(offset_px - self.prev_offset_px)
            heading_jump = abs(heading_deg - self.prev_heading_deg)

            if (
                offset_jump > float(self.p("max_offset_jump_px"))
                or heading_jump > float(self.p("max_heading_jump_deg"))
            ):
                self.stable_frames = 1
                confidence *= 0.55
            else:
                self.stable_frames += 1

        if self.stable_frames < int(self.p("min_stable_frames")):
            confidence = min(confidence, float(self.p("temporal_confidence_cap")))

        return float(clamp(confidence, 0.0, 1.0))

    def process_frame(self, bgr: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict[str, Any]]:
        image_h, image_w = bgr.shape[:2]

        lane_mask, mask_debug = self.detect_lane_mask(bgr)

        h_mat, h_inv, bev_w, bev_h = self.build_bev_homography(image_w, image_h)
        bev_mask = self.warp_to_bev(lane_mask, h_mat, bev_w, bev_h)

        left_candidates, right_candidates, candidate_debug = self.extract_bev_candidates(bev_mask)
        best_pair, all_pairs = self.select_pair(left_candidates, right_candidates, bev_w, bev_h)

        selected_left = None
        selected_right = None
        selection_source = None
        pair_score = 0.0
        pair_components: dict[str, float] = {}
        width_bottom_m = None
        width_top_m = None
        width_valid = None
        single_side_score = 0.0

        if best_pair is not None:
            selected_left = best_pair["left"]
            selected_right = best_pair["right"]
            selection_source = "bev_pair"
            pair_score = float(best_pair["score"])
            pair_components = best_pair["components"]
            width_bottom_m = float(best_pair["width_bottom_m"])
            width_top_m = float(best_pair["width_top_m"])
            width_valid = bool(best_pair["width_valid"])
        else:
            selected_left, selected_right, selection_source, single_side_score = self.select_single_side(
                left_candidates,
                right_candidates,
                bev_w,
            )

        lane_detected = selected_left is not None and selected_right is not None

        if not lane_detected:
            self.stable_frames = 0
            status = {
                "version": "LDv4_bev",
                "lane_detected": False,
                "image_width": int(image_w),
                "image_height": int(image_h),
                "confidence": 0.0,
                "center_offset_px": None,
                "heading_error_deg": None,
                "left_lane_visible": False,
                "right_lane_visible": False,
                "selected_left": None,
                "selected_right": None,
                "bev_left": None,
                "bev_right": None,
                "bev_center": None,
                "selection_source": None,
                "selected_pair_score": 0.0,
                "single_side_score": 0.0,
                "lane_width_bottom_m": None,
                "lane_width_top_m": None,
                "lane_width_valid": None,
                "stable_lane_frames": 0,
                "candidate_pair_count": int(len(all_pairs)),
                "candidate_count_left": int(len(left_candidates)),
                "candidate_count_right": int(len(right_candidates)),
                "mask_debug": mask_debug,
                "fit_debug": candidate_debug,
                "confidence_components": {},
                "confidence_caps": ["no_valid_bev_lane_geometry"],
            }

            overlay = self.draw_overlay(
                bgr,
                lane_mask,
                h_inv,
                left_candidates,
                right_candidates,
                None,
                None,
                status,
            )
            bev_debug = self.render_bev_debug(
                bev_mask,
                left_candidates,
                right_candidates,
                None,
                None,
                status,
            )
            return lane_mask, overlay, bev_debug, status

        center_line = self.make_center_line(selected_left, selected_right)

        center_offset_px = self.compute_image_offset_px(center_line, h_inv, image_w)

        center_bottom_lat = self.bev_x_to_lateral(float(center_line["x_bottom"]), bev_w)
        center_top_lat = self.bev_x_to_lateral(float(center_line["x_top"]), bev_w)
        center_bottom_fwd = self.bev_y_to_forward(float(center_line["y_bottom"]), bev_h)
        center_top_fwd = self.bev_y_to_forward(float(center_line["y_top"]), bev_h)

        heading_error_deg = math.degrees(
            math.atan2(
                center_top_lat - center_bottom_lat,
                max(1e-3, center_top_fwd - center_bottom_fwd),
            )
        )

        if best_pair is not None:
            confidence = 0.82 * pair_score + 0.18 * clamp(np.count_nonzero(bev_mask) / 900.0, 0.0, 1.0)
            if width_valid is False:
                confidence = min(confidence, float(self.p("invalid_width_confidence_cap")))
        else:
            confidence = min(float(self.p("single_side_confidence_cap")), single_side_score)

        max_heading = float(self.p("max_output_heading_deg"))
        if abs(heading_error_deg) > max_heading:
            confidence = 0.0
            caps = ["max_output_heading_exceeded"]
        else:
            caps = []

        confidence = self.apply_temporal_confidence(center_offset_px, heading_error_deg, confidence)

        alpha = clamp(float(self.p("smoothing_alpha")), 0.0, 1.0)
        if self.prev_offset_px is not None and self.prev_heading_deg is not None and confidence > 0.0:
            smoothed_offset = alpha * center_offset_px + (1.0 - alpha) * self.prev_offset_px
            smoothed_heading = alpha * heading_error_deg + (1.0 - alpha) * self.prev_heading_deg
        else:
            smoothed_offset = center_offset_px
            smoothed_heading = heading_error_deg

        if confidence > 0.0:
            self.prev_offset_px = float(smoothed_offset)
            self.prev_heading_deg = float(smoothed_heading)

        status = {
            "version": "LDv4_bev",
            "lane_detected": bool(confidence > 0.10),
            "image_width": int(image_w),
            "image_height": int(image_h),
            "confidence": round(float(confidence), 3),
            "center_offset_px": round(float(smoothed_offset), 2),
            "heading_error_deg": round(float(smoothed_heading), 2),
            "left_lane_visible": selection_source != "right_only_inferred_left",
            "right_lane_visible": selection_source != "left_only_inferred_right",
            "selected_left": self.bev_line_to_image_endpoint(selected_left, h_inv),
            "selected_right": self.bev_line_to_image_endpoint(selected_right, h_inv),
            "bev_left": self.bev_line_to_metric_endpoint(selected_left, bev_w, bev_h),
            "bev_right": self.bev_line_to_metric_endpoint(selected_right, bev_w, bev_h),
            "bev_center": self.bev_line_to_metric_endpoint(center_line, bev_w, bev_h),
            "selection_source": selection_source,
            "selected_pair_score": round(float(pair_score), 3),
            "single_side_score": round(float(single_side_score), 3),
            "lane_width_bottom_m": None if width_bottom_m is None else round(float(width_bottom_m), 3),
            "lane_width_top_m": None if width_top_m is None else round(float(width_top_m), 3),
            "lane_width_valid": width_valid,
            "stable_lane_frames": int(self.stable_frames),
            "candidate_pair_count": int(len(all_pairs)),
            "candidate_count_left": int(len(left_candidates)),
            "candidate_count_right": int(len(right_candidates)),
            "mask_debug": mask_debug,
            "fit_debug": candidate_debug,
            "confidence_components": pair_components,
            "confidence_caps": caps,
        }

        overlay = self.draw_overlay(
            bgr,
            lane_mask,
            h_inv,
            left_candidates,
            right_candidates,
            selected_left,
            selected_right,
            status,
        )
        bev_debug = self.render_bev_debug(
            bev_mask,
            left_candidates,
            right_candidates,
            selected_left,
            selected_right,
            status,
        )

        return lane_mask, overlay, bev_debug, status

    def draw_image_line_from_bev(
        self,
        image: np.ndarray,
        line: dict[str, Any],
        h_inv: np.ndarray,
        color: tuple[int, int, int],
        thickness: int,
    ) -> None:
        ep = self.bev_line_to_image_endpoint(line, h_inv)
        p0 = (int(round(ep["x_bottom"])), int(round(ep["y_bottom"])))
        p1 = (int(round(ep["x_top"])), int(round(ep["y_top"])))
        cv2.line(image, p0, p1, color, thickness, cv2.LINE_AA)

    def draw_overlay(
        self,
        bgr: np.ndarray,
        lane_mask: np.ndarray,
        h_inv: np.ndarray,
        left_candidates: list[dict[str, Any]],
        right_candidates: list[dict[str, Any]],
        selected_left: Optional[dict[str, Any]],
        selected_right: Optional[dict[str, Any]],
        status: dict[str, Any],
    ) -> np.ndarray:
        overlay = bgr.copy()

        mask_pixels = lane_mask > 0
        if np.any(mask_pixels):
            yellow = overlay.copy()
            yellow[mask_pixels] = (0, 255, 255)
            overlay = cv2.addWeighted(overlay, 0.75, yellow, 0.25, 0.0)

        # Draw candidates lightly.
        if bool(self.p("draw_all_bev_candidates")):
            for cand in left_candidates:
                self.draw_image_line_from_bev(overlay, cand, h_inv, (255, 255, 0), 1)
            for cand in right_candidates:
                self.draw_image_line_from_bev(overlay, cand, h_inv, (255, 255, 0), 1)

        # Selected lanes.
        if selected_left is not None:
            self.draw_image_line_from_bev(overlay, selected_left, h_inv, (255, 0, 0), 4)
        if selected_right is not None:
            self.draw_image_line_from_bev(overlay, selected_right, h_inv, (0, 0, 255), 4)

        if selected_left is not None and selected_right is not None:
            center = self.make_center_line(selected_left, selected_right)
            self.draw_image_line_from_bev(overlay, center, h_inv, (0, 255, 0), 4)

        text = (
            f"offset={status.get('center_offset_px')} px | "
            f"heading={status.get('heading_error_deg')} deg | "
            f"conf={status.get('confidence')} | "
            f"L={status.get('candidate_count_left', 0)} "
            f"R={status.get('candidate_count_right', 0)}"
        )
        cv2.putText(
            overlay,
            text,
            (12, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        return overlay

    def render_bev_debug(
        self,
        bev_mask: np.ndarray,
        left_candidates: list[dict[str, Any]],
        right_candidates: list[dict[str, Any]],
        selected_left: Optional[dict[str, Any]],
        selected_right: Optional[dict[str, Any]],
        status: dict[str, Any],
    ) -> np.ndarray:
        bev_h, bev_w = bev_mask.shape[:2]

        debug = np.zeros((bev_h, bev_w, 3), dtype=np.uint8)
        debug[:, :] = (20, 20, 20)
        debug[bev_mask > 0] = (90, 90, 90)

        def draw_bev_line(line: dict[str, Any], color: tuple[int, int, int], thickness: int) -> None:
            p0 = (int(round(line["x_bottom"])), int(round(line["y_bottom"])))
            p1 = (int(round(line["x_top"])), int(round(line["y_top"])))
            cv2.line(debug, p0, p1, color, thickness, cv2.LINE_AA)

        if bool(self.p("draw_all_bev_candidates")):
            for cand in left_candidates:
                draw_bev_line(cand, (255, 255, 0), 1)
            for cand in right_candidates:
                draw_bev_line(cand, (255, 255, 0), 1)

        if selected_left is not None:
            draw_bev_line(selected_left, (255, 0, 0), 3)
        if selected_right is not None:
            draw_bev_line(selected_right, (0, 0, 255), 3)

        if selected_left is not None and selected_right is not None:
            center = self.make_center_line(selected_left, selected_right)
            draw_bev_line(center, (0, 255, 0), 3)

        # Ego marker at bottom center.
        ego = (int(bev_w * 0.5), bev_h - 1)
        cv2.circle(debug, ego, 4, (0, 255, 255), -1)
        cv2.arrowedLine(
            debug,
            (ego[0], ego[1]),
            (ego[0], max(0, ego[1] - 35)),
            (0, 255, 255),
            2,
            cv2.LINE_AA,
            tipLength=0.3,
        )

        text = (
            f"LDv4 BEV | conf={status.get('confidence')} | "
            f"src={status.get('selection_source')} | "
            f"L={len(left_candidates)} R={len(right_candidates)}"
        )
        cv2.putText(
            debug,
            text,
            (8, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

        # Make it readable in Foxglove.
        scale = 3
        debug = cv2.resize(
            debug,
            (bev_w * scale, bev_h * scale),
            interpolation=cv2.INTER_NEAREST,
        )

        return debug


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LaneDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
