#!/usr/bin/env python3
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


@dataclass
class BoundaryMeasurement:
    coefficients: np.ndarray
    forward_min_m: float
    forward_max_m: float
    point_count: int
    rmse_m: float
    confidence: float
    quality_source: str


@dataclass
class BoundaryTrack:
    side: str
    coefficients: np.ndarray | None = None
    confidence: float = 0.0
    confirmed: bool = False
    hits: int = 0
    misses: int = 0
    age: int = 0
    accepted_updates: int = 0
    rejected_updates: int = 0

    pending_coefficients: np.ndarray | None = None
    pending_hits: int = 0

    forward_min_m: float = 0.0
    forward_max_m: float = 0.0
    last_update_kind: str = "none"

    def active(self) -> bool:
        return self.confirmed and self.coefficients is not None


class LaneTrackingNode(Node):
    """
    Temporally associate and smooth frame-level lane-boundary curves.

    Inputs:
      /perception/lane/left_boundary
      /perception/lane/right_boundary

    Outputs:
      /perception/lane/tracked_left_boundary
      /perception/lane/tracked_right_boundary
      /perception/lane/tracked_centerline
      /perception/lane/tracking_debug/compressed
      /perception/lane/tracking_status

    Curve convention:
      lateral_left = a * forward^2 + b * forward + c
    """

    def __init__(self) -> None:
        super().__init__("lane_tracking_node")

        defaults = {
            "left_input_topic": "/perception/lane/left_boundary",
            "right_input_topic": "/perception/lane/right_boundary",
            "curve_status_topic": "/perception/lane/curve_status",
            "left_output_topic": "/perception/lane/tracked_left_boundary",
            "right_output_topic": "/perception/lane/tracked_right_boundary",
            "center_output_topic": "/perception/lane/tracked_centerline",
            "debug_topic": "/perception/lane/tracking_debug/compressed",
            "status_topic": "/perception/lane/tracking_status",

            # BEV geometry. Must match lane_geometry_node.
            "forward_min_m": 0.0,
            "forward_max_m": 40.0,
            "left_extent_m": 12.0,
            "right_extent_m": -12.0,
            "resolution_m": 0.10,

            # Association and confirmation.
            "reference_forward_m": 7.0,
            "association_mid_forward_m": 15.0,
            "association_far_forward_m": 25.0,
            "maximum_lateral_jump_m": 0.75,
            "maximum_heading_jump_rad": 0.18,
            "maximum_curvature_jump_per_m": 0.035,
            "new_track_confirmation_frames": 3,
            "replacement_confirmation_frames": 3,
            "missing_track_hold_frames": 5,

            # Smoothing and confidence.
            "base_smoothing_alpha": 0.35,
            "minimum_measurement_confidence": 0.25,
            "confidence_decay_per_missing_frame": 0.82,
            "minimum_publish_confidence": 0.12,

            # Lane-pair constraints.
            "minimum_lane_width_m": 2.5,
            "maximum_lane_width_m": 4.8,
            "expected_lane_width_m": 3.5,
            "maximum_lane_width_change_m": 0.50,
            "maximum_lane_width_variation_m": 0.85,
            "lane_width_smoothing_alpha": 0.20,
            "minimum_width_confidence_for_inference": 0.35,
            "maximum_inferred_frames": 8,
            "inferred_boundary_confidence_scale": 0.55,

            # Output.
            "path_start_forward_m": 5.0,
            "path_end_forward_m": 30.0,
            "path_step_m": 0.5,
            "grid_spacing_m": 5.0,
            "jpeg_quality": 80,
            "maximum_sync_difference_ms": 50.0,
            "curve_status_match_tolerance_m": 0.20,
            "fallback_confidence_cap": 0.60,
            "log_every": 30,
        }

        for name, value in defaults.items():
            self.declare_parameter(name, value)

        p = lambda name: self.get_parameter(name).value

        self.left_input_topic = str(p("left_input_topic"))
        self.right_input_topic = str(p("right_input_topic"))
        self.curve_status_topic = str(p("curve_status_topic"))
        self.left_output_topic = str(p("left_output_topic"))
        self.right_output_topic = str(p("right_output_topic"))
        self.center_output_topic = str(p("center_output_topic"))
        self.debug_topic = str(p("debug_topic"))
        self.status_topic = str(p("status_topic"))

        self.forward_min_m = float(p("forward_min_m"))
        self.forward_max_m = float(p("forward_max_m"))
        self.left_extent_m = float(p("left_extent_m"))
        self.right_extent_m = float(p("right_extent_m"))
        self.resolution_m = float(p("resolution_m"))

        self.reference_forward_m = float(p("reference_forward_m"))
        self.association_forward_values = np.asarray(
            [
                self.reference_forward_m,
                float(p("association_mid_forward_m")),
                float(p("association_far_forward_m")),
            ],
            dtype=np.float64,
        )
        self.maximum_lateral_jump_m = float(p("maximum_lateral_jump_m"))
        self.maximum_heading_jump_rad = float(
            p("maximum_heading_jump_rad")
        )
        self.maximum_curvature_jump_per_m = float(
            p("maximum_curvature_jump_per_m")
        )
        self.new_track_confirmation_frames = int(
            p("new_track_confirmation_frames")
        )
        self.replacement_confirmation_frames = int(
            p("replacement_confirmation_frames")
        )
        self.missing_track_hold_frames = int(
            p("missing_track_hold_frames")
        )

        self.base_smoothing_alpha = float(p("base_smoothing_alpha"))
        self.minimum_measurement_confidence = float(
            p("minimum_measurement_confidence")
        )
        self.confidence_decay = float(
            p("confidence_decay_per_missing_frame")
        )
        self.minimum_publish_confidence = float(
            p("minimum_publish_confidence")
        )

        self.minimum_lane_width_m = float(p("minimum_lane_width_m"))
        self.maximum_lane_width_m = float(p("maximum_lane_width_m"))
        self.expected_lane_width_m = float(p("expected_lane_width_m"))
        self.maximum_lane_width_change_m = float(
            p("maximum_lane_width_change_m")
        )
        self.maximum_lane_width_variation_m = float(
            p("maximum_lane_width_variation_m")
        )
        self.lane_width_smoothing_alpha = float(
            p("lane_width_smoothing_alpha")
        )
        self.minimum_width_confidence_for_inference = float(
            p("minimum_width_confidence_for_inference")
        )
        self.maximum_inferred_frames = int(p("maximum_inferred_frames"))
        self.inferred_confidence_scale = float(
            p("inferred_boundary_confidence_scale")
        )

        self.path_start_forward_m = float(p("path_start_forward_m"))
        self.path_end_forward_m = float(p("path_end_forward_m"))
        self.path_step_m = float(p("path_step_m"))
        self.grid_spacing_m = float(p("grid_spacing_m"))
        self.jpeg_quality = int(p("jpeg_quality"))
        self.maximum_sync_difference_ns = int(
            float(p("maximum_sync_difference_ms")) * 1_000_000.0
        )
        self.curve_status_match_tolerance_m = float(
            p("curve_status_match_tolerance_m")
        )
        self.fallback_confidence_cap = float(
            p("fallback_confidence_cap")
        )
        self.log_every = max(1, int(p("log_every")))

        self._validate_parameters()

        self.height = int(
            math.ceil(
                (self.forward_max_m - self.forward_min_m)
                / self.resolution_m
            )
        )
        self.width = int(
            math.ceil(
                (self.left_extent_m - self.right_extent_m)
                / self.resolution_m
            )
        )

        self.left_track = BoundaryTrack(side="left")
        self.right_track = BoundaryTrack(side="right")

        self.lane_width_m = self.expected_lane_width_m
        self.lane_width_confidence = 0.0
        self.left_inferred_frames = 0
        self.right_inferred_frames = 0

        self.latest_left: Path | None = None
        self.latest_right: Path | None = None
        self.latest_curve_status: dict | None = None
        self.current_raw_pair_confidence: float | None = None
        self.current_curve_status_frame: int | None = None
        self.last_processed_pair: tuple[int, int] | None = None
        self.frame_count = 0

        self.left_sub = self.create_subscription(
            Path,
            self.left_input_topic,
            self._left_callback,
            qos_profile_sensor_data,
        )
        self.right_sub = self.create_subscription(
            Path,
            self.right_input_topic,
            self._right_callback,
            qos_profile_sensor_data,
        )
        self.curve_status_sub = self.create_subscription(
            String,
            self.curve_status_topic,
            self._curve_status_callback,
            10,
        )

        self.left_pub = self.create_publisher(
            Path,
            self.left_output_topic,
            10,
        )
        self.right_pub = self.create_publisher(
            Path,
            self.right_output_topic,
            10,
        )
        self.center_pub = self.create_publisher(
            Path,
            self.center_output_topic,
            10,
        )
        self.debug_pub = self.create_publisher(
            CompressedImage,
            self.debug_topic,
            qos_profile_sensor_data,
        )
        self.status_pub = self.create_publisher(
            String,
            self.status_topic,
            10,
        )

        self.get_logger().info(
            f"Left input: {self.left_input_topic}"
        )
        self.get_logger().info(
            f"Right input: {self.right_input_topic}"
        )
        self.get_logger().info(
            f"Curve quality input: {self.curve_status_topic}"
        )
        self.get_logger().info(
            "Temporal lane tracking enabled: confirmation, gated "
            "association, confidence-weighted smoothing, short holds, "
            "and single-side inference."
        )

    def _validate_parameters(self) -> None:
        if self.resolution_m <= 0.0:
            raise ValueError("resolution_m must be positive")
        if self.forward_max_m <= self.forward_min_m:
            raise ValueError("invalid BEV forward range")
        if self.left_extent_m <= self.right_extent_m:
            raise ValueError("invalid BEV lateral range")
        if not 0.0 < self.base_smoothing_alpha <= 1.0:
            raise ValueError("base_smoothing_alpha must be in (0, 1]")
        if not 0.0 < self.confidence_decay <= 1.0:
            raise ValueError(
                "confidence_decay_per_missing_frame must be in (0, 1]"
            )
        if self.path_step_m <= 0.0:
            raise ValueError("path_step_m must be positive")
        if self.minimum_lane_width_m >= self.maximum_lane_width_m:
            raise ValueError("invalid lane-width range")

    @staticmethod
    def _stamp_ns(message: Path) -> int:
        stamp = message.header.stamp
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _left_callback(self, message: Path) -> None:
        self.latest_left = message
        self._try_process()

    def _right_callback(self, message: Path) -> None:
        self.latest_right = message
        self._try_process()

    def _curve_status_callback(self, message: String) -> None:
        try:
            parsed = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warning(
                "Ignoring malformed /perception/lane/curve_status JSON."
            )
            return

        if isinstance(parsed, dict):
            self.latest_curve_status = parsed

    def _try_process(self) -> None:
        if self.latest_left is None or self.latest_right is None:
            return

        left_stamp = self._stamp_ns(self.latest_left)
        right_stamp = self._stamp_ns(self.latest_right)

        if abs(left_stamp - right_stamp) > self.maximum_sync_difference_ns:
            return

        pair = (left_stamp, right_stamp)
        if pair == self.last_processed_pair:
            return

        self.last_processed_pair = pair
        self._process_pair(self.latest_left, self.latest_right)

    def _process_pair(self, left_message: Path, right_message: Path) -> None:
        left_measurement = self._measurement_from_path(
            left_message,
            "left",
        )
        right_measurement = self._measurement_from_path(
            right_message,
            "right",
        )

        self.current_raw_pair_confidence = None
        self.current_curve_status_frame = None
        if (
            left_measurement is not None
            and right_measurement is not None
            and left_measurement.quality_source == "curve_status"
            and right_measurement.quality_source == "curve_status"
            and self.latest_curve_status is not None
        ):
            pair_confidence = self.latest_curve_status.get(
                "pair_confidence"
            )
            if pair_confidence is not None:
                self.current_raw_pair_confidence = float(
                    np.clip(float(pair_confidence), 0.0, 1.0)
                )

            frame = self.latest_curve_status.get("frame")
            if frame is not None:
                self.current_curve_status_frame = int(frame)

        self._update_track(self.left_track, left_measurement)
        self._update_track(self.right_track, right_measurement)

        direct_left = self._track_output(self.left_track)
        direct_right = self._track_output(self.right_track)

        direct_left, direct_right = self._validate_and_update_lane_pair(
            direct_left,
            direct_right,
        )

        output_left = direct_left
        output_right = direct_right
        left_inferred = False
        right_inferred = False

        if output_left is not None:
            self.left_inferred_frames = 0
        if output_right is not None:
            self.right_inferred_frames = 0

        if (
            output_left is None
            and output_right is not None
            and self._can_infer_width()
            and self.left_inferred_frames < self.maximum_inferred_frames
        ):
            output_left = output_right.copy()
            output_left[2] += self.lane_width_m
            left_inferred = True
            self.left_inferred_frames += 1

        if (
            output_right is None
            and output_left is not None
            and self._can_infer_width()
            and self.right_inferred_frames < self.maximum_inferred_frames
        ):
            output_right = output_left.copy()
            output_right[2] -= self.lane_width_m
            right_inferred = True
            self.right_inferred_frames += 1

        output_left_confidence = self._output_confidence(
            self.left_track,
            left_inferred,
            output_right if left_inferred else None,
        )
        output_right_confidence = self._output_confidence(
            self.right_track,
            right_inferred,
            output_left if right_inferred else None,
        )

        if output_left_confidence < self.minimum_publish_confidence:
            output_left = None
            left_inferred = False

        if output_right_confidence < self.minimum_publish_confidence:
            output_right = None
            right_inferred = False

        header = left_message.header
        if not header.frame_id:
            header = right_message.header

        left_path = self._build_path(output_left, header)
        right_path = self._build_path(output_right, header)
        center_path = self._build_center_path(
            output_left,
            output_right,
            header,
        )

        self.left_pub.publish(left_path)
        self.right_pub.publish(right_path)
        self.center_pub.publish(center_path)

        debug = self._build_debug(
            raw_left=(
                left_measurement.coefficients
                if left_measurement is not None
                else None
            ),
            raw_right=(
                right_measurement.coefficients
                if right_measurement is not None
                else None
            ),
            tracked_left=output_left,
            tracked_right=output_right,
            left_inferred=left_inferred,
            right_inferred=right_inferred,
        )
        self._publish_debug(debug, header)

        self.frame_count += 1
        self._publish_status(
            left_measurement=left_measurement,
            right_measurement=right_measurement,
            output_left=output_left,
            output_right=output_right,
            left_inferred=left_inferred,
            right_inferred=right_inferred,
            left_confidence=output_left_confidence,
            right_confidence=output_right_confidence,
        )

        if self.frame_count % self.log_every == 0:
            self.get_logger().info(
                f"frames={self.frame_count} "
                f"left={self.left_track.last_update_kind} "
                f"right={self.right_track.last_update_kind} "
                f"width={self.lane_width_m:.2f}m "
                f"width_conf={self.lane_width_confidence:.2f}"
            )

    def _measurement_from_path(
        self,
        message: Path,
        side: str,
    ) -> BoundaryMeasurement | None:
        if len(message.poses) < 3:
            return None

        points = np.asarray(
            [
                (
                    float(pose.pose.position.x),
                    float(pose.pose.position.y),
                )
                for pose in message.poses
            ],
            dtype=np.float64,
        )

        finite = np.isfinite(points).all(axis=1)
        points = points[finite]
        if len(points) < 3:
            return None

        forward = points[:, 0]
        lateral = points[:, 1]
        forward_span = float(np.ptp(forward))

        if forward_span < 2.0:
            return None

        try:
            coefficients = np.polyfit(forward, lateral, 2)
        except (np.linalg.LinAlgError, ValueError, TypeError):
            return None

        diagnostics = self._matching_curve_diagnostics(
            side,
            coefficients,
        )

        if diagnostics is not None:
            confidence = float(
                np.clip(
                    float(diagnostics.get("confidence", 0.0)),
                    0.0,
                    1.0,
                )
            )
            rmse_m = float(diagnostics.get("rmse_m", 0.0))
            point_count = int(
                diagnostics.get("inlier_count", len(points))
            )
            forward_min_m = float(
                diagnostics.get("forward_min_m", np.min(forward))
            )
            forward_max_m = float(
                diagnostics.get("forward_max_m", np.max(forward))
            )
            quality_source = "curve_status"
        else:
            # The path is sampled from an already-fitted polynomial, so
            # refitting it gives an artificially tiny residual. This
            # fallback is deliberately capped and should only be used when
            # the status message is unavailable or does not match.
            residuals = lateral - np.polyval(coefficients, forward)
            rmse_m = float(
                np.sqrt(np.mean(np.square(residuals)))
            )
            span_factor = min(1.0, forward_span / 15.0)
            point_factor = min(1.0, len(points) / 30.0)
            confidence = min(
                self.fallback_confidence_cap,
                span_factor * point_factor,
            )
            point_count = len(points)
            forward_min_m = float(np.min(forward))
            forward_max_m = float(np.max(forward))
            quality_source = "path_fallback"

        if confidence < self.minimum_measurement_confidence:
            return None

        return BoundaryMeasurement(
            coefficients=coefficients,
            forward_min_m=forward_min_m,
            forward_max_m=forward_max_m,
            point_count=point_count,
            rmse_m=rmse_m,
            confidence=confidence,
            quality_source=quality_source,
        )

    def _matching_curve_diagnostics(
        self,
        side: str,
        path_coefficients: np.ndarray,
    ) -> dict | None:
        status = self.latest_curve_status
        if not isinstance(status, dict):
            return None

        diagnostics = status.get(side)
        if not isinstance(diagnostics, dict):
            return None

        status_coefficients = diagnostics.get("coefficients")
        if (
            not isinstance(status_coefficients, list)
            or len(status_coefficients) != 3
        ):
            return None

        try:
            status_coefficients_array = np.asarray(
                status_coefficients,
                dtype=np.float64,
            )
        except (TypeError, ValueError):
            return None

        if not np.isfinite(status_coefficients_array).all():
            return None

        forward = self.association_forward_values
        difference = np.abs(
            np.polyval(status_coefficients_array, forward)
            - np.polyval(path_coefficients, forward)
        )

        if (
            float(np.max(difference))
            > self.curve_status_match_tolerance_m
        ):
            return None

        return diagnostics

    def _update_track(
        self,
        track: BoundaryTrack,
        measurement: BoundaryMeasurement | None,
    ) -> None:
        track.age += 1

        if measurement is None:
            self._handle_missing_measurement(track)
            return

        if not track.active():
            self._update_unconfirmed_track(track, measurement)
            return

        if self._measurement_matches_track(track, measurement):
            alpha = float(
                np.clip(
                    self.base_smoothing_alpha * measurement.confidence,
                    0.05,
                    self.base_smoothing_alpha,
                )
            )
            track.coefficients = (
                (1.0 - alpha) * track.coefficients
                + alpha * measurement.coefficients
            )
            track.confidence = float(
                np.clip(
                    0.75 * track.confidence
                    + 0.25 * measurement.confidence,
                    0.0,
                    1.0,
                )
            )
            track.forward_min_m = (
                (1.0 - alpha) * track.forward_min_m
                + alpha * measurement.forward_min_m
            )
            track.forward_max_m = (
                (1.0 - alpha) * track.forward_max_m
                + alpha * measurement.forward_max_m
            )
            track.hits += 1
            track.misses = 0
            track.accepted_updates += 1
            track.pending_coefficients = None
            track.pending_hits = 0
            track.last_update_kind = "accepted"
            return

        track.rejected_updates += 1
        track.last_update_kind = "rejected"
        self._update_replacement_candidate(track, measurement)

        track.misses += 1
        track.confidence *= self.confidence_decay

        if track.misses > self.missing_track_hold_frames:
            if track.pending_hits >= self.replacement_confirmation_frames:
                self._replace_track_with_pending(track, measurement)
            else:
                self._deactivate_track(track)

    def _update_unconfirmed_track(
        self,
        track: BoundaryTrack,
        measurement: BoundaryMeasurement,
    ) -> None:
        if (
            track.pending_coefficients is None
            or not self._coefficients_match(
                track.pending_coefficients,
                measurement.coefficients,
            )
        ):
            track.pending_coefficients = measurement.coefficients.copy()
            track.pending_hits = 1
        else:
            alpha = float(
                np.clip(
                    self.base_smoothing_alpha * measurement.confidence,
                    0.05,
                    self.base_smoothing_alpha,
                )
            )
            track.pending_coefficients = (
                (1.0 - alpha) * track.pending_coefficients
                + alpha * measurement.coefficients
            )
            track.pending_hits += 1

        track.hits = track.pending_hits
        track.misses = 0
        track.last_update_kind = "confirming"

        if track.pending_hits >= self.new_track_confirmation_frames:
            track.coefficients = track.pending_coefficients.copy()
            track.confidence = measurement.confidence
            track.confirmed = True
            track.forward_min_m = measurement.forward_min_m
            track.forward_max_m = measurement.forward_max_m
            track.accepted_updates += 1
            track.pending_coefficients = None
            track.pending_hits = 0
            track.last_update_kind = "confirmed"

    def _handle_missing_measurement(self, track: BoundaryTrack) -> None:
        if not track.active():
            track.pending_coefficients = None
            track.pending_hits = 0
            track.hits = 0
            track.last_update_kind = "missing"
            return

        track.misses += 1
        track.confidence *= self.confidence_decay
        track.last_update_kind = "held"

        if track.misses > self.missing_track_hold_frames:
            self._deactivate_track(track)

    def _update_replacement_candidate(
        self,
        track: BoundaryTrack,
        measurement: BoundaryMeasurement,
    ) -> None:
        if (
            track.pending_coefficients is None
            or not self._coefficients_match(
                track.pending_coefficients,
                measurement.coefficients,
            )
        ):
            track.pending_coefficients = measurement.coefficients.copy()
            track.pending_hits = 1
        else:
            alpha = float(
                np.clip(
                    self.base_smoothing_alpha * measurement.confidence,
                    0.05,
                    self.base_smoothing_alpha,
                )
            )
            track.pending_coefficients = (
                (1.0 - alpha) * track.pending_coefficients
                + alpha * measurement.coefficients
            )
            track.pending_hits += 1

    def _replace_track_with_pending(
        self,
        track: BoundaryTrack,
        measurement: BoundaryMeasurement,
    ) -> None:
        track.coefficients = track.pending_coefficients.copy()
        track.confidence = measurement.confidence
        track.confirmed = True
        track.hits = track.pending_hits
        track.misses = 0
        track.forward_min_m = measurement.forward_min_m
        track.forward_max_m = measurement.forward_max_m
        track.pending_coefficients = None
        track.pending_hits = 0
        track.accepted_updates += 1
        track.last_update_kind = "replaced"

    @staticmethod
    def _deactivate_track(track: BoundaryTrack) -> None:
        track.coefficients = None
        track.confidence = 0.0
        track.confirmed = False
        track.hits = 0
        track.misses = 0
        track.pending_coefficients = None
        track.pending_hits = 0
        track.last_update_kind = "lost"

    def _measurement_matches_track(
        self,
        track: BoundaryTrack,
        measurement: BoundaryMeasurement,
    ) -> bool:
        assert track.coefficients is not None
        return self._coefficients_match(
            track.coefficients,
            measurement.coefficients,
        )

    def _coefficients_match(
        self,
        first: np.ndarray,
        second: np.ndarray,
    ) -> bool:
        forward = self.association_forward_values
        lateral_difference = np.abs(
            np.polyval(first, forward) - np.polyval(second, forward)
        )

        reference = self.reference_forward_m
        first_heading = math.atan(
            2.0 * float(first[0]) * reference + float(first[1])
        )
        second_heading = math.atan(
            2.0 * float(second[0]) * reference + float(second[1])
        )
        heading_difference = abs(first_heading - second_heading)

        curvature_difference = abs(
            2.0 * float(first[0]) - 2.0 * float(second[0])
        )

        return bool(
            float(np.max(lateral_difference))
            <= self.maximum_lateral_jump_m
            and heading_difference <= self.maximum_heading_jump_rad
            and curvature_difference
            <= self.maximum_curvature_jump_per_m
        )

    def _track_output(
        self,
        track: BoundaryTrack,
    ) -> np.ndarray | None:
        if not track.active():
            return None
        if track.confidence < self.minimum_publish_confidence:
            return None
        return track.coefficients.copy()

    def _validate_and_update_lane_pair(
        self,
        left: np.ndarray | None,
        right: np.ndarray | None,
    ) -> tuple[np.ndarray | None, np.ndarray | None]:
        if left is None or right is None:
            return left, right

        forward = self.association_forward_values
        widths = np.polyval(left, forward) - np.polyval(right, forward)
        mean_width = float(np.mean(widths))
        width_variation = float(np.ptp(widths))

        plausible = bool(
            np.all(widths >= self.minimum_lane_width_m)
            and np.all(widths <= self.maximum_lane_width_m)
            and width_variation <= self.maximum_lane_width_variation_m
        )

        if self.lane_width_confidence > 0.0:
            plausible = plausible and (
                abs(mean_width - self.lane_width_m)
                <= self.maximum_lane_width_change_m
            )

        if plausible:
            if self.lane_width_confidence <= 0.0:
                self.lane_width_m = mean_width
            else:
                alpha = self.lane_width_smoothing_alpha
                self.lane_width_m = (
                    (1.0 - alpha) * self.lane_width_m
                    + alpha * mean_width
                )

            pair_confidence = math.sqrt(
                max(self.left_track.confidence, 0.0)
                * max(self.right_track.confidence, 0.0)
            )
            if self.current_raw_pair_confidence is not None:
                pair_confidence = min(
                    pair_confidence,
                    self.current_raw_pair_confidence,
                )
            self.lane_width_confidence = float(
                np.clip(
                    0.85 * self.lane_width_confidence
                    + 0.15 * pair_confidence,
                    0.0,
                    1.0,
                )
            )
            return left, right

        self.lane_width_confidence *= 0.92

        if self.left_track.confidence >= self.right_track.confidence:
            return left, None
        return None, right

    def _can_infer_width(self) -> bool:
        return (
            self.lane_width_confidence
            >= self.minimum_width_confidence_for_inference
            and self.minimum_lane_width_m
            <= self.lane_width_m
            <= self.maximum_lane_width_m
        )

    def _output_confidence(
        self,
        track: BoundaryTrack,
        inferred: bool,
        source_coefficients: np.ndarray | None,
    ) -> float:
        if not inferred:
            return track.confidence if track.active() else 0.0

        source_confidence = max(
            self.left_track.confidence,
            self.right_track.confidence,
        )
        return float(
            source_confidence
            * self.lane_width_confidence
            * self.inferred_confidence_scale
        )

    def _build_path(
        self,
        coefficients: np.ndarray | None,
        header,
    ) -> Path:
        path = Path()
        path.header = header

        if coefficients is None:
            return path

        for forward_m in np.arange(
            self.path_start_forward_m,
            self.path_end_forward_m + 0.5 * self.path_step_m,
            self.path_step_m,
            dtype=np.float64,
        ):
            pose = PoseStamped()
            pose.header = header
            pose.pose.position.x = float(forward_m)
            pose.pose.position.y = float(
                np.polyval(coefficients, forward_m)
            )
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        return path

    def _build_center_path(
        self,
        left: np.ndarray | None,
        right: np.ndarray | None,
        header,
    ) -> Path:
        path = Path()
        path.header = header

        if left is None or right is None:
            return path

        center = 0.5 * (left + right)
        return self._build_path(center, header)

    def _build_debug(
        self,
        raw_left: np.ndarray | None,
        raw_right: np.ndarray | None,
        tracked_left: np.ndarray | None,
        tracked_right: np.ndarray | None,
        left_inferred: bool,
        right_inferred: bool,
    ) -> np.ndarray:
        image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        self._draw_grid(image)

        if raw_left is not None:
            self._draw_curve(image, raw_left, (170, 120, 0), 1)
        if raw_right is not None:
            self._draw_curve(image, raw_right, (170, 120, 0), 1)

        if tracked_left is not None:
            color = (0, 180, 255) if left_inferred else (0, 220, 0)
            self._draw_curve(image, tracked_left, color, 3)

        if tracked_right is not None:
            color = (0, 180, 255) if right_inferred else (0, 0, 230)
            self._draw_curve(image, tracked_right, color, 3)

        if tracked_left is not None and tracked_right is not None:
            center = 0.5 * (tracked_left + tracked_right)
            self._draw_curve(image, center, (0, 220, 220), 2)

        cv2.putText(
            image,
            "green: tracked left",
            (6, 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.40,
            (0, 220, 0),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            "red: tracked right",
            (6, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.40,
            (0, 0, 230),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            "yellow: tracked centerline",
            (6, 48),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.40,
            (0, 220, 220),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            "orange: inferred boundary",
            (6, 64),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.40,
            (0, 180, 255),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            "cyan: raw frame measurement",
            (6, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.40,
            (170, 120, 0),
            1,
            cv2.LINE_AA,
        )

        return image

    def _draw_curve(
        self,
        image: np.ndarray,
        coefficients: np.ndarray,
        color: tuple[int, int, int],
        thickness: int,
    ) -> None:
        pixels: list[tuple[int, int]] = []

        for forward_m in np.arange(
            self.path_start_forward_m,
            self.path_end_forward_m + 0.1,
            0.25,
            dtype=np.float64,
        ):
            lateral_m = float(np.polyval(coefficients, forward_m))
            row, column = self.metric_to_pixel(forward_m, lateral_m)

            if 0 <= row < self.height and 0 <= column < self.width:
                pixels.append((column, row))

        if len(pixels) >= 2:
            cv2.polylines(
                image,
                [np.asarray(pixels, dtype=np.int32)],
                isClosed=False,
                color=color,
                thickness=thickness,
                lineType=cv2.LINE_AA,
            )

    def _draw_grid(self, image: np.ndarray) -> None:
        spacing = max(self.grid_spacing_m, self.resolution_m)

        forward = math.ceil(self.forward_min_m / spacing) * spacing
        while forward <= self.forward_max_m:
            row, _ = self.metric_to_pixel(forward, 0.0)
            if 0 <= row < self.height:
                cv2.line(
                    image,
                    (0, row),
                    (self.width - 1, row),
                    (45, 45, 45),
                    1,
                )
            forward += spacing

        lateral = math.ceil(self.right_extent_m / spacing) * spacing
        while lateral <= self.left_extent_m:
            _, column = self.metric_to_pixel(self.forward_min_m, lateral)
            if 0 <= column < self.width:
                color = (
                    (0, 150, 255)
                    if abs(lateral) < 1e-6
                    else (45, 45, 45)
                )
                cv2.line(
                    image,
                    (column, 0),
                    (column, self.height - 1),
                    color,
                    1,
                )
            lateral += spacing

    def metric_to_pixel(
        self,
        forward_m: float,
        lateral_left_m: float,
    ) -> tuple[int, int]:
        row = int(
            round(
                (self.forward_max_m - forward_m)
                / self.resolution_m
                - 0.5
            )
        )
        column = int(
            round(
                (self.left_extent_m - lateral_left_m)
                / self.resolution_m
                - 0.5
            )
        )
        return row, column

    def _publish_debug(self, image: np.ndarray, header) -> None:
        ok, encoded = cv2.imencode(
            ".jpg",
            image,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok:
            return

        message = CompressedImage()
        message.header = header
        message.format = "jpeg"
        message.data = encoded.tobytes()
        self.debug_pub.publish(message)

    def _publish_status(
        self,
        left_measurement: BoundaryMeasurement | None,
        right_measurement: BoundaryMeasurement | None,
        output_left: np.ndarray | None,
        output_right: np.ndarray | None,
        left_inferred: bool,
        right_inferred: bool,
        left_confidence: float,
        right_confidence: float,
    ) -> None:
        lane_width_at_reference = None
        center_offset_at_reference = None

        if output_left is not None and output_right is not None:
            left_reference = float(
                np.polyval(output_left, self.reference_forward_m)
            )
            right_reference = float(
                np.polyval(output_right, self.reference_forward_m)
            )
            lane_width_at_reference = left_reference - right_reference
            center_offset_at_reference = 0.5 * (
                left_reference + right_reference
            )

        message = String()
        message.data = json.dumps(
            {
                "frame": self.frame_count,
                "curve_status_frame": self.current_curve_status_frame,
                "raw_pair_confidence": self.current_raw_pair_confidence,
                "lane_width_estimate_m": self.lane_width_m,
                "lane_width_confidence": self.lane_width_confidence,
                "lane_width_at_reference_m": lane_width_at_reference,
                "center_offset_at_reference_m": center_offset_at_reference,
                "left": self._track_status(
                    self.left_track,
                    left_measurement,
                    output_left,
                    left_inferred,
                    left_confidence,
                ),
                "right": self._track_status(
                    self.right_track,
                    right_measurement,
                    output_right,
                    right_inferred,
                    right_confidence,
                ),
                "centerline_available": (
                    output_left is not None and output_right is not None
                ),
            },
            separators=(",", ":"),
        )
        self.status_pub.publish(message)

    def _track_status(
        self,
        track: BoundaryTrack,
        measurement: BoundaryMeasurement | None,
        output: np.ndarray | None,
        inferred: bool,
        output_confidence: float,
    ) -> dict:
        return {
            "state": track.last_update_kind,
            "confirmed": track.confirmed,
            "hits": track.hits,
            "misses": track.misses,
            "confidence": track.confidence,
            "output_confidence": output_confidence,
            "inferred": inferred,
            "accepted_updates": track.accepted_updates,
            "rejected_updates": track.rejected_updates,
            "measurement": (
                {
                    "coefficients": [
                        float(value)
                        for value in measurement.coefficients
                    ],
                    "quality_source": measurement.quality_source,
                    "confidence": measurement.confidence,
                    "rmse_m": measurement.rmse_m,
                    "point_count": measurement.point_count,
                    "forward_min_m": measurement.forward_min_m,
                    "forward_max_m": measurement.forward_max_m,
                }
                if measurement is not None
                else None
            ),
            "output_coefficients": (
                [float(value) for value in output]
                if output is not None
                else None
            ),
            "lateral_at_reference_m": (
                float(np.polyval(output, self.reference_forward_m))
                if output is not None
                else None
            ),
        }


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LaneTrackingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
