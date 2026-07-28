#!/usr/bin/env python3

import json
import math
from typing import Any, Optional

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from perception_geometry import BEVSpec, CameraIPM, IPMConfig


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class LaneMappingNode(Node):
    """
    Lane Mapping v2.1

    Rolling ego-centric lane memory.

    Compatible with:
    - old lane_status: center_offset_px + heading_error_deg + confidence
    - new lane_status: selected_left / selected_right endpoints

    Main useful debug topic:
      /perception/rolling_lane_map_debug/compressed
    """

    def __init__(self) -> None:
        super().__init__("lane_mapping_node")

        self.declare_parameter("lane_status_topic", "/perception/lane_status")

        self.declare_parameter("rolling_lane_map_topic", "/perception/rolling_lane_map")
        self.declare_parameter("rolling_lane_map_debug_topic", "/perception/rolling_lane_map_debug/compressed")

        # Kept for compatibility with the current Foxglove layout.
        self.declare_parameter("accumulated_lane_map_topic", "/perception/accumulated_lane_map")
        self.declare_parameter("accumulated_lane_map_debug_topic", "/perception/accumulated_lane_map_debug/compressed")

        self.declare_parameter("lane_mapping_status_topic", "/perception/lane_mapping_status")

        # Existing good mapping parameters.
        self.declare_parameter("map_size_m", 300.0)
        self.declare_parameter("resolution", 0.25)
        self.declare_parameter("lane_hit_inc", 3.0)
        self.declare_parameter("lane_decay", 0.999)
        self.declare_parameter("occupied_thresh", 2.0)
        self.declare_parameter("line_thickness_cells", 2)
        self.declare_parameter("pixel_scale", 2)

        # Rolling debug/map parameters.
        self.declare_parameter("rolling_forward_m", 45.0)
        self.declare_parameter("rolling_backward_m", 8.0)
        self.declare_parameter("rolling_width_m", 16.0)
        self.declare_parameter("rolling_resolution", 0.20)
        self.declare_parameter("rolling_pixel_scale", 5)
        self.declare_parameter("rolling_lane_decay", 0.97)

        # Integration threshold.
        # Use 0.65 for debugging. Raise to 0.85/0.90 later.
        self.declare_parameter("min_confidence", 0.65)

        # Debug threshold: draw current-frame geometry even if not integrated.
        self.declare_parameter("debug_min_confidence", 0.10)

        # Fallback geometry if lane_status has no selected_left/right.
        self.declare_parameter("default_image_width", 800.0)
        self.declare_parameter("default_image_height", 600.0)
        self.declare_parameter("fit_top_y_ratio", 0.48)
        self.declare_parameter("fit_bottom_y_ratio", 0.94)
        self.declare_parameter("fallback_lane_width_px", 420.0)
        self.declare_parameter("fallback_lane_width_top_ratio", 0.55)

        # Approximate image-to-BEV trapezoid.
        self.declare_parameter("src_bottom_y_ratio", 0.94)
        self.declare_parameter("src_top_y_ratio", 0.48)
        self.declare_parameter("src_bottom_left_x_ratio", 0.12)
        self.declare_parameter("src_bottom_right_x_ratio", 0.88)
        self.declare_parameter("src_top_left_x_ratio", 0.42)
        self.declare_parameter("src_top_right_x_ratio", 0.58)

        self.declare_parameter("frame_id", "hero")
        self.declare_parameter("debug_jpeg_quality", 90)

        self.forward_m = float(self.p("rolling_forward_m"))
        self.backward_m = float(self.p("rolling_backward_m"))
        self.width_m = float(self.p("rolling_width_m"))
        self.resolution_m = float(self.p("rolling_resolution"))

        # Shared image <-> BEV <-> metric geometry utility.
        # For image projection we use backward_m=0.0 because camera IPM maps
        # the visible road in front of the vehicle. The rolling map can still
        # include backward_m for visualization/storage.
        self.ipm = self.make_ipm()

        self.rows = int(round((self.forward_m + self.backward_m) / self.resolution_m))
        self.cols = int(round(self.width_m / self.resolution_m))

        self.left_scores = np.zeros((self.rows, self.cols), dtype=np.float32)
        self.right_scores = np.zeros((self.rows, self.cols), dtype=np.float32)
        self.center_scores = np.zeros((self.rows, self.cols), dtype=np.float32)

        self.current_left = np.zeros((self.rows, self.cols), dtype=np.uint8)
        self.current_right = np.zeros((self.rows, self.cols), dtype=np.uint8)
        self.current_center = np.zeros((self.rows, self.cols), dtype=np.uint8)

        self.received_frames = 0
        self.integrated_frames = 0
        self.debug_drawn_frames = 0

        self.sub = self.create_subscription(
            String,
            str(self.p("lane_status_topic")),
            self.lane_status_cb,
            10,
        )

        self.rolling_map_pub = self.create_publisher(
            OccupancyGrid,
            str(self.p("rolling_lane_map_topic")),
            10,
        )
        self.rolling_debug_pub = self.create_publisher(
            CompressedImage,
            str(self.p("rolling_lane_map_debug_topic")),
            10,
        )

        self.accumulated_map_pub = self.create_publisher(
            OccupancyGrid,
            str(self.p("accumulated_lane_map_topic")),
            10,
        )
        self.accumulated_debug_pub = self.create_publisher(
            CompressedImage,
            str(self.p("accumulated_lane_map_debug_topic")),
            10,
        )

        self.status_pub = self.create_publisher(
            String,
            str(self.p("lane_mapping_status_topic")),
            10,
        )

        self.get_logger().info("Lane Mapping v2.1 started")
        self.get_logger().info(f"Subscribing: {self.p('lane_status_topic')}")
        self.get_logger().info(f"Publishing rolling debug: {self.p('rolling_lane_map_debug_topic')}")

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
                forward_m=float(self.p("rolling_forward_m")),
                backward_m=0.0,
                width_m=float(self.p("rolling_width_m")),
                resolution_m=float(self.p("rolling_resolution")),
            ),
        )

    def lane_status_cb(self, msg: String) -> None:
        self.received_frames += 1

        try:
            status = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warn(f"Could not parse lane_status JSON: {exc}")
            return

        self.decay_memory()
        self.clear_current_layers()

        confidence = float(status.get("confidence") or 0.0)
        lane_detected = bool(status.get("lane_detected", confidence > 0.0))

        geometry = self.extract_or_build_geometry(status)

        debug_drawn = False
        integrated = False

        if geometry is not None and lane_detected and confidence >= float(self.p("debug_min_confidence")):
            debug_drawn = self.draw_geometry_to_current_layers(geometry)
            if debug_drawn:
                self.debug_drawn_frames += 1

        if geometry is not None and lane_detected and confidence >= float(self.p("min_confidence")):
            integrated = self.integrate_geometry(geometry)
            if integrated:
                self.integrated_frames += 1

        self.publish_all(status, confidence, geometry, debug_drawn, integrated)

    def decay_memory(self) -> None:
        decay = float(self.p("rolling_lane_decay"))
        self.left_scores *= decay
        self.right_scores *= decay
        self.center_scores *= decay

    def clear_current_layers(self) -> None:
        self.current_left.fill(0)
        self.current_right.fill(0)
        self.current_center.fill(0)

    def extract_or_build_geometry(self, status: dict[str, Any]) -> Optional[dict[str, Any]]:
        image_width = float(status.get("image_width") or self.p("default_image_width"))
        image_height = float(status.get("image_height") or self.p("default_image_height"))

        # Best path: LDv4 publishes metric BEV geometry directly.
        bev_left = status.get("bev_left")
        bev_right = status.get("bev_right")

        if isinstance(bev_left, dict) or isinstance(bev_right, dict):
            return {
                "image_width": image_width,
                "image_height": image_height,
                "left_metric": bev_left if isinstance(bev_left, dict) else None,
                "right_metric": bev_right if isinstance(bev_right, dict) else None,
                "source": "bev_metric_endpoints",
            }

        # Compatibility path: image-space selected lane endpoints.
        selected_left = status.get("selected_left")
        selected_right = status.get("selected_right")

        if isinstance(selected_left, dict) or isinstance(selected_right, dict):
            return {
                "image_width": image_width,
                "image_height": image_height,
                "left": selected_left if isinstance(selected_left, dict) else None,
                "right": selected_right if isinstance(selected_right, dict) else None,
                "source": "selected_image_endpoints",
            }

        # Last fallback: old offset + heading lane status.
        offset = status.get("center_offset_px")
        heading = status.get("heading_error_deg")

        if offset is None or heading is None:
            return None

        try:
            offset_px = float(offset)
            heading_deg = float(heading)
        except Exception:
            return None

        y_bottom = image_height * float(self.p("fit_bottom_y_ratio"))
        y_top = image_height * float(self.p("fit_top_y_ratio"))
        dy = y_bottom - y_top

        center_bottom = image_width * 0.5 + offset_px
        center_top = center_bottom + math.tan(math.radians(heading_deg)) * dy

        lane_width_bottom = float(self.p("fallback_lane_width_px"))
        lane_width_top = lane_width_bottom * float(self.p("fallback_lane_width_top_ratio"))

        left = {
            "x_bottom": center_bottom - 0.5 * lane_width_bottom,
            "y_bottom": y_bottom,
            "x_top": center_top - 0.5 * lane_width_top,
            "y_top": y_top,
        }
        right = {
            "x_bottom": center_bottom + 0.5 * lane_width_bottom,
            "y_bottom": y_bottom,
            "x_top": center_top + 0.5 * lane_width_top,
            "y_top": y_top,
        }

        return {
            "image_width": image_width,
            "image_height": image_height,
            "left": left,
            "right": right,
            "source": "offset_heading_fallback",
        }

    def image_point_to_bev(
        self,
        u: float,
        v: float,
        image_width: float,
        image_height: float,
    ) -> Optional[tuple[float, float]]:
        try:
            self.ipm = self.make_ipm()
            image_to_bev, _ = self.ipm.homographies(int(round(image_width)), int(round(image_height)))
            forward_m, lateral_m = self.ipm.image_point_to_metric(float(u), float(v), image_to_bev)
        except Exception:
            return None

        if not self.ipm.metric_in_bounds(forward_m, lateral_m, margin_m=2.0):
            return None

        return float(forward_m), float(lateral_m)

    def metric_to_grid(self, forward_m: float, lateral_m: float) -> Optional[tuple[int, int]]:
        if forward_m < -self.backward_m or forward_m > self.forward_m:
            return None
        if lateral_m < -0.5 * self.width_m or lateral_m > 0.5 * self.width_m:
            return None

        col = int(round((lateral_m / self.width_m + 0.5) * max(1, self.cols - 1)))
        row = int(round(((self.forward_m - forward_m) / (self.forward_m + self.backward_m)) * max(1, self.rows - 1)))

        if not (0 <= row < self.rows and 0 <= col < self.cols):
            return None

        return row, col

    def endpoint_to_metric(
        self,
        endpoint: dict[str, Any],
        image_width: float,
        image_height: float,
    ) -> Optional[tuple[tuple[float, float], tuple[float, float]]]:
        try:
            p_bottom = self.image_point_to_bev(
                float(endpoint["x_bottom"]),
                float(endpoint["y_bottom"]),
                image_width,
                image_height,
            )
            p_top = self.image_point_to_bev(
                float(endpoint["x_top"]),
                float(endpoint["y_top"]),
                image_width,
                image_height,
            )
        except Exception:
            return None

        if p_bottom is None or p_top is None:
            return None

        return p_bottom, p_top

    def draw_metric_line_to_grids(
        self,
        score_grid: np.ndarray,
        current_grid: np.ndarray,
        p0: tuple[float, float],
        p1: tuple[float, float],
        score_inc: float,
        thickness: int,
        integrate: bool,
    ) -> bool:
        g0 = self.metric_to_grid(p0[0], p0[1])
        g1 = self.metric_to_grid(p1[0], p1[1])

        if g0 is None or g1 is None:
            return False

        r0, c0 = g0
        r1, c1 = g1

        cv2.line(current_grid, (c0, r0), (c1, r1), 255, max(1, thickness), cv2.LINE_AA)

        if integrate:
            cv2.line(score_grid, (c0, r0), (c1, r1), float(score_inc), max(1, thickness), cv2.LINE_AA)

        return True

    def geometry_to_metric_lines(self, geometry: dict[str, Any]) -> dict[str, Any]:
        source = geometry.get("source", "unknown")

        left_line = None
        right_line = None
        center_line = None

        if source == "bev_metric_endpoints":
            left_metric = geometry.get("left_metric")
            right_metric = geometry.get("right_metric")

            if isinstance(left_metric, dict):
                try:
                    left_line = (
                        (
                            float(left_metric["forward_bottom_m"]),
                            float(left_metric["lateral_bottom_m"]),
                        ),
                        (
                            float(left_metric["forward_top_m"]),
                            float(left_metric["lateral_top_m"]),
                        ),
                    )
                except Exception:
                    left_line = None

            if isinstance(right_metric, dict):
                try:
                    right_line = (
                        (
                            float(right_metric["forward_bottom_m"]),
                            float(right_metric["lateral_bottom_m"]),
                        ),
                        (
                            float(right_metric["forward_top_m"]),
                            float(right_metric["lateral_top_m"]),
                        ),
                    )
                except Exception:
                    right_line = None

        else:
            image_width = float(geometry["image_width"])
            image_height = float(geometry["image_height"])

            left_ep = geometry.get("left")
            right_ep = geometry.get("right")

            if isinstance(left_ep, dict):
                left_line = self.endpoint_to_metric(left_ep, image_width, image_height)

            if isinstance(right_ep, dict):
                right_line = self.endpoint_to_metric(right_ep, image_width, image_height)

        if left_line is not None and right_line is not None:
            center_bottom = (
                0.5 * (left_line[0][0] + right_line[0][0]),
                0.5 * (left_line[0][1] + right_line[0][1]),
            )
            center_top = (
                0.5 * (left_line[1][0] + right_line[1][0]),
                0.5 * (left_line[1][1] + right_line[1][1]),
            )
            center_line = (center_bottom, center_top)

        return {
            "left": left_line,
            "right": right_line,
            "center": center_line,
            "source": source,
        }

    def draw_geometry_to_current_layers(self, geometry: dict[str, Any]) -> bool:
        lines = self.geometry_to_metric_lines(geometry)
        thickness = int(self.p("line_thickness_cells"))
        hit = float(self.p("lane_hit_inc"))

        any_drawn = False

        if lines["left"] is not None:
            any_drawn |= self.draw_metric_line_to_grids(
                self.left_scores,
                self.current_left,
                lines["left"][0],
                lines["left"][1],
                hit,
                thickness,
                integrate=False,
            )

        if lines["right"] is not None:
            any_drawn |= self.draw_metric_line_to_grids(
                self.right_scores,
                self.current_right,
                lines["right"][0],
                lines["right"][1],
                hit,
                thickness,
                integrate=False,
            )

        if lines["center"] is not None:
            any_drawn |= self.draw_metric_line_to_grids(
                self.center_scores,
                self.current_center,
                lines["center"][0],
                lines["center"][1],
                hit,
                thickness,
                integrate=False,
            )

        return bool(any_drawn)

    def integrate_geometry(self, geometry: dict[str, Any]) -> bool:
        lines = self.geometry_to_metric_lines(geometry)
        thickness = int(self.p("line_thickness_cells"))
        hit = float(self.p("lane_hit_inc"))

        any_drawn = False

        if lines["left"] is not None:
            any_drawn |= self.draw_metric_line_to_grids(
                self.left_scores,
                self.current_left,
                lines["left"][0],
                lines["left"][1],
                hit,
                thickness,
                integrate=True,
            )

        if lines["right"] is not None:
            any_drawn |= self.draw_metric_line_to_grids(
                self.right_scores,
                self.current_right,
                lines["right"][0],
                lines["right"][1],
                hit,
                thickness,
                integrate=True,
            )

        if lines["center"] is not None:
            any_drawn |= self.draw_metric_line_to_grids(
                self.center_scores,
                self.current_center,
                lines["center"][0],
                lines["center"][1],
                hit,
                thickness,
                integrate=True,
            )

        return bool(any_drawn)

    def make_occupancy_grid(self) -> OccupancyGrid:
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.p("frame_id"))

        msg.info.resolution = float(self.resolution_m)
        msg.info.width = int(self.cols)
        msg.info.height = int(self.rows)
        msg.info.origin.position.x = -float(self.backward_m)
        msg.info.origin.position.y = -0.5 * float(self.width_m)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        thresh = float(self.p("occupied_thresh"))
        combined = np.maximum.reduce([self.left_scores, self.right_scores, self.center_scores])

        occ = np.zeros_like(combined, dtype=np.int8)
        occ[combined >= thresh] = 100

        msg.data = occ.flatten().astype(np.int8).tolist()
        return msg

    def render_debug(self) -> np.ndarray:
        thresh = float(self.p("occupied_thresh"))

        debug = np.zeros((self.rows, self.cols, 3), dtype=np.uint8)
        debug[:, :] = (20, 20, 20)

        left_mask = self.left_scores >= thresh
        right_mask = self.right_scores >= thresh
        center_mask = self.center_scores >= thresh

        debug[left_mask] = (255, 0, 0)
        debug[right_mask] = (0, 0, 255)
        debug[center_mask] = (0, 180, 0)

        current_left = self.current_left > 0
        current_right = self.current_right > 0
        current_center = self.current_center > 0

        debug[current_left] = (255, 180, 0)
        debug[current_right] = (0, 180, 255)
        debug[current_center] = (0, 255, 0)

        hero = self.metric_to_grid(0.0, 0.0)
        if hero is not None:
            hr, hc = hero
            cv2.circle(debug, (hc, hr), 3, (0, 255, 255), -1)
            cv2.arrowedLine(
                debug,
                (hc, min(self.rows - 1, hr + 10)),
                (hc, max(0, hr - 18)),
                (0, 255, 255),
                2,
                cv2.LINE_AA,
                tipLength=0.35,
            )

        scale = max(1, int(self.p("rolling_pixel_scale")))
        if scale != 1:
            debug = cv2.resize(
                debug,
                (self.cols * scale, self.rows * scale),
                interpolation=cv2.INTER_NEAREST,
            )

        return debug

    def publish_debug(self, pub, image: np.ndarray) -> None:
        ok, encoded = cv2.imencode(
            ".jpg",
            image,
            [int(cv2.IMWRITE_JPEG_QUALITY), int(self.p("debug_jpeg_quality"))],
        )
        if not ok:
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.p("frame_id"))
        msg.format = "jpeg"
        msg.data = encoded.tobytes()
        pub.publish(msg)

    def publish_all(
        self,
        lane_status: dict[str, Any],
        confidence: float,
        geometry: Optional[dict[str, Any]],
        debug_drawn: bool,
        integrated: bool,
    ) -> None:
        grid = self.make_occupancy_grid()

        self.rolling_map_pub.publish(grid)
        self.accumulated_map_pub.publish(grid)

        debug = self.render_debug()
        self.publish_debug(self.rolling_debug_pub, debug)
        self.publish_debug(self.accumulated_debug_pub, debug)

        status = {
            "version": "lane_mapping_v2.1",
            "received_frames": int(self.received_frames),
            "debug_drawn_frames": int(self.debug_drawn_frames),
            "integrated_frames": int(self.integrated_frames),
            "last_debug_drawn": bool(debug_drawn),
            "last_integrated": bool(integrated),
            "last_lane_confidence": float(confidence),
            "min_confidence": float(self.p("min_confidence")),
            "debug_min_confidence": float(self.p("debug_min_confidence")),
            "geometry_available": geometry is not None,
            "geometry_source": None if geometry is None else geometry.get("source"),
            "rows": int(self.rows),
            "cols": int(self.cols),
            "left_cells": int(np.count_nonzero(self.left_scores >= float(self.p("occupied_thresh")))),
            "right_cells": int(np.count_nonzero(self.right_scores >= float(self.p("occupied_thresh")))),
            "center_cells": int(np.count_nonzero(self.center_scores >= float(self.p("occupied_thresh")))),
            "input_lane_detected": bool(lane_status.get("lane_detected", False)),
            "input_center_offset_px": lane_status.get("center_offset_px"),
            "input_heading_error_deg": lane_status.get("heading_error_deg"),
        }

        msg = String()
        msg.data = json.dumps(status, separators=(",", ":"))
        self.status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LaneMappingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
