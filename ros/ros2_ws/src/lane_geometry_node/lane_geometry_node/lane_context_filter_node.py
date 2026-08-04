#!/usr/bin/env python3
from __future__ import annotations

import json
import math
from dataclasses import dataclass

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String


@dataclass
class StampedMask:
    stamp_ns: int
    message: Image
    mask: np.ndarray


class LaneContextFilterNode(Node):
    """Suppress crosswalks, stop lines, and dense junction paint in metric BEV."""

    def __init__(self) -> None:
        super().__init__("lane_context_filter_node")
        self.bridge = CvBridge()

        defaults = {
            "longitudinal_topic": "/perception/lane/longitudinal_mask",
            "transverse_topic": "/perception/lane/transverse_mask",
            "exclusion_topic": "/perception/lane/exclusion_mask",
            "candidate_topic": "/perception/lane/candidate_mask",
            "debug_topic": "/perception/lane/context_debug/compressed",
            "status_topic": "/perception/lane/context_status",
            "forward_min_m": 0.0,
            "forward_max_m": 40.0,
            "left_extent_m": 12.0,
            "right_extent_m": -12.0,
            "resolution_m": 0.10,
            "process_forward_min_m": 5.0,
            "process_forward_max_m": 30.0,
            "process_left_extent_m": 10.0,
            "process_right_extent_m": -10.0,
            "row_smoothing_m": 0.50,
            "crosswalk_min_lateral_coverage": 0.18,
            "crosswalk_min_longitudinal_runs": 6.0,
            "transverse_min_lateral_coverage": 0.18,
            "minimum_exclusion_band_m": 0.30,
            "dense_window_forward_m": 3.0,
            "dense_window_lateral_m": 8.0,
            "dense_union_fraction": 0.10,
            "dense_longitudinal_fraction": 0.035,
            "dense_transverse_fraction": 0.015,
            "exclusion_padding_forward_m": 0.80,
            "exclusion_padding_lateral_m": 0.30,
            "min_candidate_component_area_px": 6,
            "min_candidate_forward_span_m": 0.60,
            "grid_spacing_m": 5.0,
            "jpeg_quality": 80,
            "max_sync_difference_ms": 50.0,
            "log_every": 30,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        p = lambda name: self.get_parameter(name).value
        self.longitudinal_topic = str(p("longitudinal_topic"))
        self.transverse_topic = str(p("transverse_topic"))
        self.exclusion_topic = str(p("exclusion_topic"))
        self.candidate_topic = str(p("candidate_topic"))
        self.debug_topic = str(p("debug_topic"))
        self.status_topic = str(p("status_topic"))

        self.fmin = float(p("forward_min_m"))
        self.fmax = float(p("forward_max_m"))
        self.left = float(p("left_extent_m"))
        self.right = float(p("right_extent_m"))
        self.res = float(p("resolution_m"))

        self.pfmin = float(p("process_forward_min_m"))
        self.pfmax = float(p("process_forward_max_m"))
        self.pleft = float(p("process_left_extent_m"))
        self.pright = float(p("process_right_extent_m"))

        self.row_smoothing_m = float(p("row_smoothing_m"))
        self.crosswalk_min_coverage = float(p("crosswalk_min_lateral_coverage"))
        self.crosswalk_min_runs = float(p("crosswalk_min_longitudinal_runs"))
        self.transverse_min_coverage = float(p("transverse_min_lateral_coverage"))
        self.minimum_exclusion_band_m = float(p("minimum_exclusion_band_m"))

        self.dense_window_forward_m = float(p("dense_window_forward_m"))
        self.dense_window_lateral_m = float(p("dense_window_lateral_m"))
        self.dense_union_fraction = float(p("dense_union_fraction"))
        self.dense_long_fraction = float(p("dense_longitudinal_fraction"))
        self.dense_trans_fraction = float(p("dense_transverse_fraction"))

        self.exclusion_padding_forward_m = float(p("exclusion_padding_forward_m"))
        self.exclusion_padding_lateral_m = float(p("exclusion_padding_lateral_m"))
        self.min_candidate_area = int(p("min_candidate_component_area_px"))
        self.min_candidate_forward_span_m = float(p("min_candidate_forward_span_m"))
        self.grid_spacing_m = float(p("grid_spacing_m"))
        self.jpeg_quality = int(p("jpeg_quality"))
        self.max_sync_difference_ns = int(float(p("max_sync_difference_ms")) * 1_000_000.0)
        self.log_every = max(1, int(p("log_every")))

        self._validate()
        self.height = int(math.ceil((self.fmax - self.fmin) / self.res))
        self.width = int(math.ceil((self.left - self.right) / self.res))
        self.roi_mask = self._build_roi_mask()

        self.latest_longitudinal: StampedMask | None = None
        self.latest_transverse: StampedMask | None = None
        self.last_processed_pair: tuple[int, int] | None = None
        self.frames = 0

        self.long_sub = self.create_subscription(
            Image, self.longitudinal_topic, self._longitudinal_callback, qos_profile_sensor_data
        )
        self.trans_sub = self.create_subscription(
            Image, self.transverse_topic, self._transverse_callback, qos_profile_sensor_data
        )
        self.exclusion_pub = self.create_publisher(
            Image, self.exclusion_topic, qos_profile_sensor_data
        )
        self.candidate_pub = self.create_publisher(
            Image, self.candidate_topic, qos_profile_sensor_data
        )
        self.debug_pub = self.create_publisher(
            CompressedImage, self.debug_topic, qos_profile_sensor_data
        )
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.get_logger().info(f"Longitudinal input: {self.longitudinal_topic}")
        self.get_logger().info(f"Transverse input: {self.transverse_topic}")
        self.get_logger().info(f"Candidate output: {self.candidate_topic}")
        self.get_logger().info(
            f"ROI: {self.pfmin:.1f}-{self.pfmax:.1f} m forward, "
            f"{self.pright:.1f}..{self.pleft:.1f} m lateral"
        )

    def _validate(self) -> None:
        if self.res <= 0.0 or self.fmax <= self.fmin or self.left <= self.right:
            raise ValueError("Invalid BEV geometry")
        if not self.fmin <= self.pfmin < self.pfmax <= self.fmax:
            raise ValueError("Processing forward range is outside the BEV")
        if not self.right <= self.pright < self.pleft <= self.left:
            raise ValueError("Processing lateral range is outside the BEV")

    @staticmethod
    def _stamp_ns(message: Image) -> int:
        stamp = message.header.stamp
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _decode(self, message: Image) -> np.ndarray | None:
        try:
            mask = self.bridge.imgmsg_to_cv2(message, desired_encoding="mono8")
        except Exception as exc:
            self.get_logger().error(f"Could not decode input mask: {exc}")
            return None
        if mask.shape[:2] != (self.height, self.width):
            self.get_logger().error(
                f"Input shape {mask.shape[:2]} != expected {(self.height, self.width)}"
            )
            return None
        return np.where(mask > 0, 255, 0).astype(np.uint8)

    def _longitudinal_callback(self, message: Image) -> None:
        mask = self._decode(message)
        if mask is not None:
            self.latest_longitudinal = StampedMask(self._stamp_ns(message), message, mask)
            self._try_process()

    def _transverse_callback(self, message: Image) -> None:
        mask = self._decode(message)
        if mask is not None:
            self.latest_transverse = StampedMask(self._stamp_ns(message), message, mask)
            self._try_process()

    def _try_process(self) -> None:
        if self.latest_longitudinal is None or self.latest_transverse is None:
            return
        long_sample = self.latest_longitudinal
        trans_sample = self.latest_transverse
        if abs(long_sample.stamp_ns - trans_sample.stamp_ns) > self.max_sync_difference_ns:
            return
        pair = (long_sample.stamp_ns, trans_sample.stamp_ns)
        if pair == self.last_processed_pair:
            return
        self.last_processed_pair = pair
        self._process_pair(long_sample, trans_sample)

    def _process_pair(self, long_sample: StampedMask, trans_sample: StampedMask) -> None:
        longitudinal = cv2.bitwise_and(long_sample.mask, self.roi_mask)
        transverse = cv2.bitwise_and(trans_sample.mask, self.roi_mask)
        union = cv2.bitwise_or(longitudinal, transverse)

        row_mask, band_count = self._row_exclusion(longitudinal, transverse, union)
        dense_mask = self._dense_junction_exclusion(longitudinal, transverse, union)
        exclusion = self._pad_exclusion(cv2.bitwise_or(row_mask, dense_mask))
        exclusion = cv2.bitwise_and(exclusion, self.roi_mask)

        candidate = cv2.bitwise_and(longitudinal, cv2.bitwise_not(exclusion))
        candidate = self._filter_candidate_components(candidate)

        source = long_sample.message
        self._publish_mask(exclusion, self.exclusion_pub, source)
        self._publish_mask(candidate, self.candidate_pub, source)
        self._publish_debug(
            self._debug(longitudinal, transverse, exclusion, candidate), source
        )

        self.frames += 1
        self._publish_status(
            longitudinal, transverse, exclusion, candidate, band_count, dense_mask
        )

        if self.frames % self.log_every == 0:
            original = max(1, int(np.count_nonzero(longitudinal)))
            kept = int(np.count_nonzero(candidate))
            removed = int(np.count_nonzero(cv2.bitwise_and(longitudinal, exclusion)))
            self.get_logger().info(
                f"frames={self.frames} bands={band_count} "
                f"kept={100.0 * kept / original:.1f}% "
                f"excluded={100.0 * removed / original:.1f}%"
            )

    def _row_exclusion(
        self, longitudinal: np.ndarray, transverse: np.ndarray, union: np.ndarray
    ) -> tuple[np.ndarray, int]:
        roi_columns = np.any(self.roi_mask > 0, axis=0)
        column_count = max(1, int(np.count_nonzero(roi_columns)))

        union_rows = union[:, roi_columns] > 0
        long_rows = longitudinal[:, roi_columns] > 0
        trans_rows = transverse[:, roi_columns] > 0

        union_coverage = np.count_nonzero(union_rows, axis=1) / column_count
        trans_coverage = np.count_nonzero(trans_rows, axis=1) / column_count

        padded = np.pad(long_rows.astype(np.uint8), ((0, 0), (1, 0)), mode="constant")
        run_starts = (padded[:, 1:] > 0) & (padded[:, :-1] == 0)
        long_runs = np.count_nonzero(run_starts, axis=1).astype(np.float32)

        kernel = self._odd_pixels(self.row_smoothing_m)
        union_coverage = self._smooth_1d(union_coverage, kernel)
        trans_coverage = self._smooth_1d(trans_coverage, kernel)
        long_runs = self._smooth_1d(long_runs, kernel)

        crosswalk_rows = (
            (union_coverage >= self.crosswalk_min_coverage)
            & (long_runs >= self.crosswalk_min_runs)
        )
        transverse_rows = trans_coverage >= self.transverse_min_coverage
        active_rows = (crosswalk_rows | transverse_rows) & np.any(self.roi_mask > 0, axis=1)

        minimum_rows = max(1, int(round(self.minimum_exclusion_band_m / self.res)))
        kept_rows, band_count = self._keep_row_bands(active_rows, minimum_rows)

        mask = np.zeros_like(union)
        mask[kept_rows, :] = self.roi_mask[kept_rows, :]
        return mask, band_count

    def _dense_junction_exclusion(
        self, longitudinal: np.ndarray, transverse: np.ndarray, union: np.ndarray
    ) -> np.ndarray:
        ksize = (
            self._odd_pixels(self.dense_window_lateral_m),
            self._odd_pixels(self.dense_window_forward_m),
        )
        density = lambda mask: cv2.boxFilter(
            (mask > 0).astype(np.float32),
            ddepth=-1,
            ksize=ksize,
            normalize=True,
            borderType=cv2.BORDER_CONSTANT,
        )
        dense = (
            (density(union) >= self.dense_union_fraction)
            & (density(longitudinal) >= self.dense_long_fraction)
            & (density(transverse) >= self.dense_trans_fraction)
            & (self.roi_mask > 0)
        )
        return dense.astype(np.uint8) * 255

    def _pad_exclusion(self, exclusion: np.ndarray) -> np.ndarray:
        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (
                self._odd_pixels(2.0 * self.exclusion_padding_lateral_m),
                self._odd_pixels(2.0 * self.exclusion_padding_forward_m),
            ),
        )
        return cv2.dilate(exclusion, kernel, iterations=1)

    def _filter_candidate_components(self, candidate: np.ndarray) -> np.ndarray:
        min_height = max(1, int(round(self.min_candidate_forward_span_m / self.res)))
        count, labels, stats, _ = cv2.connectedComponentsWithStats(candidate, connectivity=8)
        output = np.zeros_like(candidate)
        for label in range(1, count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            height = int(stats[label, cv2.CC_STAT_HEIGHT])
            if area >= self.min_candidate_area and height >= min_height:
                output[labels == label] = 255
        return output

    def _build_roi_mask(self) -> np.ndarray:
        roi = np.zeros((self.height, self.width), dtype=np.uint8)
        far_row, _ = self.metric_to_pixel(self.pfmax, 0.0)
        near_row, _ = self.metric_to_pixel(self.pfmin, 0.0)
        _, left_col = self.metric_to_pixel(self.pfmin, self.pleft)
        _, right_col = self.metric_to_pixel(self.pfmin, self.pright)
        r0, r1 = sorted((max(0, far_row), min(self.height - 1, near_row)))
        c0, c1 = sorted((max(0, left_col), min(self.width - 1, right_col)))
        roi[r0 : r1 + 1, c0 : c1 + 1] = 255
        return roi

    def _publish_mask(self, mask: np.ndarray, publisher, source: Image) -> None:
        message = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
        message.header = source.header
        publisher.publish(message)

    def _publish_debug(self, image: np.ndarray, source: Image) -> None:
        ok, encoded = cv2.imencode(
            ".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        )
        if ok:
            message = CompressedImage()
            message.header = source.header
            message.format = "jpeg"
            message.data = encoded.tobytes()
            self.debug_pub.publish(message)

    def _publish_status(
        self,
        longitudinal: np.ndarray,
        transverse: np.ndarray,
        exclusion: np.ndarray,
        candidate: np.ndarray,
        band_count: int,
        dense_mask: np.ndarray,
    ) -> None:
        longitudinal_pixels = int(np.count_nonzero(longitudinal))
        denominator = max(1, longitudinal_pixels)
        excluded = int(np.count_nonzero(cv2.bitwise_and(longitudinal, exclusion)))
        candidate_pixels = int(np.count_nonzero(candidate))
        message = String()
        message.data = json.dumps(
            {
                "frame": self.frames,
                "row_exclusion_bands": band_count,
                "longitudinal_pixels": longitudinal_pixels,
                "transverse_pixels": int(np.count_nonzero(transverse)),
                "dense_region_pixels": int(np.count_nonzero(dense_mask)),
                "excluded_longitudinal_pixels": excluded,
                "candidate_pixels": candidate_pixels,
                "candidate_fraction": candidate_pixels / denominator,
                "excluded_fraction": excluded / denominator,
            },
            separators=(",", ":"),
        )
        self.status_pub.publish(message)

    def _debug(
        self,
        longitudinal: np.ndarray,
        transverse: np.ndarray,
        exclusion: np.ndarray,
        candidate: np.ndarray,
    ) -> np.ndarray:
        debug = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        union = cv2.bitwise_or(longitudinal, transverse)
        debug[union > 0] = (75, 75, 75)
        debug[transverse > 0] = (220, 120, 0)
        debug[exclusion > 0] = (0, 0, 220)
        debug[candidate > 0] = (0, 220, 0)
        self._draw_grid(debug)
        cv2.putText(debug, "green: retained candidate", (6, 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 220, 0), 1, cv2.LINE_AA)
        cv2.putText(debug, "red: excluded context", (6, 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 0, 220), 1, cv2.LINE_AA)
        cv2.putText(debug, "blue: transverse support", (6, 48),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, (220, 120, 0), 1, cv2.LINE_AA)
        return debug

    def _draw_grid(self, image: np.ndarray) -> None:
        spacing = max(self.grid_spacing_m, self.res)
        forward = math.ceil(self.fmin / spacing) * spacing
        while forward <= self.fmax:
            row, _ = self.metric_to_pixel(forward, 0.0)
            if 0 <= row < self.height:
                cv2.line(image, (0, row), (self.width - 1, row), (45, 45, 45), 1)
            forward += spacing
        lateral = math.ceil(self.right / spacing) * spacing
        while lateral <= self.left:
            _, col = self.metric_to_pixel(self.fmin, lateral)
            if 0 <= col < self.width:
                color = (0, 150, 255) if abs(lateral) < 1e-6 else (45, 45, 45)
                cv2.line(image, (col, 0), (col, self.height - 1), color, 1)
            lateral += spacing

    def metric_to_pixel(self, forward_m: float, lateral_left_m: float) -> tuple[int, int]:
        row = int(round((self.fmax - forward_m) / self.res - 0.5))
        col = int(round((self.left - lateral_left_m) / self.res - 0.5))
        return row, col

    def _odd_pixels(self, distance_m: float) -> int:
        pixels = max(1, int(round(distance_m / self.res)))
        return pixels if pixels % 2 == 1 else pixels + 1

    @staticmethod
    def _smooth_1d(values: np.ndarray, kernel_size: int) -> np.ndarray:
        if kernel_size <= 1:
            return values.astype(np.float32)
        vector = values.astype(np.float32).reshape(-1, 1)
        return cv2.blur(
            vector, (1, kernel_size), borderType=cv2.BORDER_REPLICATE
        ).reshape(-1)

    @staticmethod
    def _keep_row_bands(active_rows: np.ndarray, minimum_rows: int) -> tuple[np.ndarray, int]:
        output = np.zeros_like(active_rows, dtype=bool)
        start: int | None = None
        count = 0
        for index, active in enumerate(active_rows):
            if active and start is None:
                start = index
            is_last = index == len(active_rows) - 1
            if start is not None and ((not active) or is_last):
                end = index if active and is_last else index - 1
                if end - start + 1 >= minimum_rows:
                    output[start : end + 1] = True
                    count += 1
                start = None
        return output, count


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LaneContextFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
