#!/usr/bin/env python3
from __future__ import annotations

import json
import math
from collections import deque
from dataclasses import dataclass, field

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


@dataclass(frozen=True)
class OdomPose:
    stamp_ns: int
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass
class BoundaryMeasurement:
    coefficients: np.ndarray
    forward_min_m: float
    forward_max_m: float
    point_count: int
    rmse_m: float
    confidence: float
    quality_source: str
    status_frame: int | None = None
    pair_confidence: float | None = None


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
      /perception/lane/curve_status
      /carla/hero_odom

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
            "odom_topic": "/carla/hero_odom",
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

            # Odometry-aware propagation. The current CARLA bridge publishes
            # CARLA/Unreal y and yaw signs, while lane curves use ROS-style
            # forward-left coordinates. These defaults convert odometry into
            # the same convention as the lane tracker without changing the
            # bridge topics used by other nodes.
            "odom_buffer_seconds": 3.0,
            "maximum_odom_age_ms": 150.0,
            "odom_y_sign": -1.0,
            "odom_yaw_sign": -1.0,
            "maximum_propagation_translation_m": 3.0,
            "maximum_propagation_yaw_rad": 0.45,
            "propagation_sample_step_m": 0.25,
            "minimum_propagated_points": 12,

            # Output.
            "path_start_forward_m": 5.0,
            "path_end_forward_m": 30.0,
            "path_step_m": 0.5,
            "grid_spacing_m": 5.0,
            "jpeg_quality": 80,
            "maximum_sync_difference_ms": 50.0,
            "curve_status_match_tolerance_m": 0.20,
            "curve_status_history_size": 12,
            "fallback_confidence_cap": 0.60,
            "minimum_pair_confidence_for_width_update": 0.30,

            # A frame-level fitter may return a valid curve belonging to a
            # distant adjacent lane when only one ego-lane side is visible.
            # Reject such measurements before they can create or replace a
            # temporal ego-lane track.
            "single_boundary_side_tolerance_m": 0.75,
            "maximum_single_boundary_abs_lateral_m": 5.50,

            "log_every": 30,
        }

        for name, value in defaults.items():
            self.declare_parameter(name, value)

        p = lambda name: self.get_parameter(name).value

        self.left_input_topic = str(p("left_input_topic"))
        self.right_input_topic = str(p("right_input_topic"))
        self.curve_status_topic = str(p("curve_status_topic"))
        self.odom_topic = str(p("odom_topic"))
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

        self.odom_buffer_seconds = float(p("odom_buffer_seconds"))
        self.maximum_odom_age_ns = int(
            float(p("maximum_odom_age_ms")) * 1_000_000.0
        )
        self.odom_y_sign = float(p("odom_y_sign"))
        self.odom_yaw_sign = float(p("odom_yaw_sign"))
        self.maximum_propagation_translation_m = float(
            p("maximum_propagation_translation_m")
        )
        self.maximum_propagation_yaw_rad = float(
            p("maximum_propagation_yaw_rad")
        )
        self.propagation_sample_step_m = float(
            p("propagation_sample_step_m")
        )
        self.minimum_propagated_points = int(
            p("minimum_propagated_points")
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
        self.curve_status_history_size = max(
            2,
            int(p("curve_status_history_size")),
        )
        self.fallback_confidence_cap = float(
            p("fallback_confidence_cap")
        )
        self.minimum_pair_confidence_for_width_update = float(
            p("minimum_pair_confidence_for_width_update")
        )
        self.single_boundary_side_tolerance_m = float(
            p("single_boundary_side_tolerance_m")
        )
        self.maximum_single_boundary_abs_lateral_m = float(
            p("maximum_single_boundary_abs_lateral_m")
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
        self.curve_status_history: deque[dict] = deque(
            maxlen=self.curve_status_history_size
        )
        self.selected_curve_status: dict | None = None
        self.current_raw_pair_confidence: float | None = None
        self.current_curve_status_frame: int | None = None
        self.current_lane_width_update_applied = False
        self.current_lane_width_update_reason = "not_evaluated"

        self.odom_buffer: deque[OdomPose] = deque()
        self.previous_lane_pose: OdomPose | None = None
        self.motion_status = self._empty_motion_status("not_initialized")

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
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self._odom_callback,
            50,
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
            f"Odometry input: {self.odom_topic}"
        )
        self.get_logger().info(
            "Temporal lane tracking enabled: odometry propagation, "
            "confirmation, gated association, confidence-weighted "
            "smoothing, short holds, and single-side inference."
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
        if self.odom_buffer_seconds <= 0.0:
            raise ValueError("odom_buffer_seconds must be positive")
        if self.maximum_odom_age_ns <= 0:
            raise ValueError("maximum_odom_age_ms must be positive")
        if self.maximum_propagation_translation_m <= 0.0:
            raise ValueError(
                "maximum_propagation_translation_m must be positive"
            )
        if self.maximum_propagation_yaw_rad <= 0.0:
            raise ValueError(
                "maximum_propagation_yaw_rad must be positive"
            )
        if self.propagation_sample_step_m <= 0.0:
            raise ValueError(
                "propagation_sample_step_m must be positive"
            )
        if self.minimum_propagated_points < 3:
            raise ValueError(
                "minimum_propagated_points must be at least 3"
            )
        if self.single_boundary_side_tolerance_m < 0.0:
            raise ValueError(
                "single_boundary_side_tolerance_m must be non-negative"
            )
        if self.maximum_single_boundary_abs_lateral_m <= 0.0:
            raise ValueError(
                "maximum_single_boundary_abs_lateral_m must be positive"
            )

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
            self.curve_status_history.append(parsed)
            self._try_process()

    def _odom_callback(self, message: Odometry) -> None:
        stamp = message.header.stamp
        stamp_ns = (
            int(stamp.sec) * 1_000_000_000
            + int(stamp.nanosec)
        )
        if stamp_ns <= 0:
            return

        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = self._quaternion_to_yaw(orientation)

        pose = OdomPose(
            stamp_ns=stamp_ns,
            x_m=float(position.x),
            y_m=self.odom_y_sign * float(position.y),
            yaw_rad=self._normalize_angle(
                self.odom_yaw_sign * yaw
            ),
        )
        self.odom_buffer.append(pose)

        newest_stamp = self.odom_buffer[-1].stamp_ns
        oldest_allowed = newest_stamp - int(
            self.odom_buffer_seconds * 1_000_000_000.0
        )
        while (
            self.odom_buffer
            and self.odom_buffer[0].stamp_ns < oldest_allowed
        ):
            self.odom_buffer.popleft()

    @staticmethod
    def _quaternion_to_yaw(orientation) -> float:
        siny_cosp = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )
        cosy_cosp = 1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        )
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _empty_motion_status(self, reason: str) -> dict:
        return {
            "odom_available": False,
            "odom_age_ms": None,
            "odom_buffer_size": len(
                getattr(self, "odom_buffer", ())
            ),
            "delta_forward_m": 0.0,
            "delta_lateral_m": 0.0,
            "delta_yaw_rad": 0.0,
            "propagation_applied": False,
            "propagated_tracks": 0,
            "reason": reason,
        }

    def _nearest_odom_pose(
        self,
        stamp_ns: int,
    ) -> tuple[OdomPose | None, float | None]:
        if not self.odom_buffer:
            return None, None

        pose = min(
            self.odom_buffer,
            key=lambda item: abs(item.stamp_ns - stamp_ns),
        )
        age_ns = abs(pose.stamp_ns - stamp_ns)
        age_ms = age_ns / 1_000_000.0

        if age_ns > self.maximum_odom_age_ns:
            return None, age_ms
        return pose, age_ms

    def _prepare_motion_for_frame(self, stamp_ns: int) -> None:
        current_pose, age_ms = self._nearest_odom_pose(stamp_ns)
        self.motion_status = self._empty_motion_status(
            "odom_unavailable"
        )
        self.motion_status["odom_age_ms"] = age_ms
        self.motion_status["odom_buffer_size"] = len(self.odom_buffer)

        if current_pose is None:
            # Measurements processed without a matching odometry sample are
            # already expressed in the current camera frame. Reinitialize the
            # pose anchor on the next valid sample rather than applying one
            # accumulated transform to tracks that may have been updated in
            # the meantime.
            self.previous_lane_pose = None
            return

        self.motion_status["odom_available"] = True

        if self.previous_lane_pose is None:
            self.previous_lane_pose = current_pose
            self.motion_status["reason"] = "initialized"
            return

        previous_pose = self.previous_lane_pose
        world_dx = current_pose.x_m - previous_pose.x_m
        world_dy = current_pose.y_m - previous_pose.y_m

        cos_previous = math.cos(previous_pose.yaw_rad)
        sin_previous = math.sin(previous_pose.yaw_rad)
        delta_forward = (
            cos_previous * world_dx + sin_previous * world_dy
        )
        delta_lateral = (
            -sin_previous * world_dx + cos_previous * world_dy
        )
        delta_yaw = self._normalize_angle(
            current_pose.yaw_rad - previous_pose.yaw_rad
        )

        self.motion_status["delta_forward_m"] = delta_forward
        self.motion_status["delta_lateral_m"] = delta_lateral
        self.motion_status["delta_yaw_rad"] = delta_yaw

        translation = math.hypot(delta_forward, delta_lateral)
        if translation < 1e-4 and abs(delta_yaw) < 1e-5:
            self.previous_lane_pose = current_pose
            self.motion_status["reason"] = "stationary"
            return

        if (
            translation > self.maximum_propagation_translation_m
            or abs(delta_yaw) > self.maximum_propagation_yaw_rad
        ):
            self._reset_tracks_after_motion_jump()
            self.previous_lane_pose = current_pose
            self.motion_status["reason"] = "motion_jump_reset"
            return

        propagated = self._propagate_all_tracks(
            previous_pose,
            current_pose,
        )
        self.previous_lane_pose = current_pose
        self.motion_status["propagated_tracks"] = propagated
        self.motion_status["propagation_applied"] = propagated > 0
        self.motion_status["reason"] = (
            "propagated" if propagated > 0 else "no_active_tracks"
        )

    def _reset_tracks_after_motion_jump(self) -> None:
        self._deactivate_track(self.left_track)
        self._deactivate_track(self.right_track)
        self.left_inferred_frames = 0
        self.right_inferred_frames = 0
        self.lane_width_confidence *= 0.5

    def _propagate_all_tracks(
        self,
        previous_pose: OdomPose,
        current_pose: OdomPose,
    ) -> int:
        propagated = 0
        for track in (self.left_track, self.right_track):
            if self._propagate_track(
                track,
                previous_pose,
                current_pose,
            ):
                propagated += 1
        return propagated

    def _propagate_track(
        self,
        track: BoundaryTrack,
        previous_pose: OdomPose,
        current_pose: OdomPose,
    ) -> bool:
        changed = False

        if track.coefficients is not None:
            result = self._propagate_polynomial(
                track.coefficients,
                track.forward_min_m,
                track.forward_max_m,
                previous_pose,
                current_pose,
            )
            if result is None:
                self._deactivate_track(track)
            else:
                (
                    track.coefficients,
                    track.forward_min_m,
                    track.forward_max_m,
                ) = result
                changed = True

        if track.pending_coefficients is not None:
            pending_result = self._propagate_polynomial(
                track.pending_coefficients,
                self.path_start_forward_m,
                self.path_end_forward_m,
                previous_pose,
                current_pose,
            )
            if pending_result is None:
                track.pending_coefficients = None
                track.pending_hits = 0
            else:
                track.pending_coefficients = pending_result[0]
                changed = True

        return changed

    def _propagate_polynomial(
        self,
        coefficients: np.ndarray,
        forward_min_m: float,
        forward_max_m: float,
        previous_pose: OdomPose,
        current_pose: OdomPose,
    ) -> tuple[np.ndarray, float, float] | None:
        sample_min = max(
            self.forward_min_m,
            min(forward_min_m, forward_max_m),
        )
        sample_max = min(
            self.forward_max_m,
            max(forward_min_m, forward_max_m),
        )

        if sample_max - sample_min < 2.0:
            sample_min = self.path_start_forward_m
            sample_max = self.path_end_forward_m

        forward_old = np.arange(
            sample_min,
            sample_max + 0.5 * self.propagation_sample_step_m,
            self.propagation_sample_step_m,
            dtype=np.float64,
        )
        lateral_old = np.polyval(coefficients, forward_old)

        delta_yaw = self._normalize_angle(
            previous_pose.yaw_rad - current_pose.yaw_rad
        )
        cos_delta = math.cos(delta_yaw)
        sin_delta = math.sin(delta_yaw)

        world_origin_dx = previous_pose.x_m - current_pose.x_m
        world_origin_dy = previous_pose.y_m - current_pose.y_m
        cos_current = math.cos(current_pose.yaw_rad)
        sin_current = math.sin(current_pose.yaw_rad)

        origin_forward_current = (
            cos_current * world_origin_dx
            + sin_current * world_origin_dy
        )
        origin_lateral_current = (
            -sin_current * world_origin_dx
            + cos_current * world_origin_dy
        )

        forward_new = (
            cos_delta * forward_old
            - sin_delta * lateral_old
            + origin_forward_current
        )
        lateral_new = (
            sin_delta * forward_old
            + cos_delta * lateral_old
            + origin_lateral_current
        )

        valid = (
            np.isfinite(forward_new)
            & np.isfinite(lateral_new)
            & (forward_new >= self.forward_min_m)
            & (forward_new <= self.forward_max_m)
            & (lateral_new >= self.right_extent_m - 2.0)
            & (lateral_new <= self.left_extent_m + 2.0)
        )
        forward_new = forward_new[valid]
        lateral_new = lateral_new[valid]

        if len(forward_new) < self.minimum_propagated_points:
            return None

        try:
            propagated_coefficients = np.polyfit(
                forward_new,
                lateral_new,
                2,
            )
        except (np.linalg.LinAlgError, ValueError, TypeError):
            return None

        if not np.isfinite(propagated_coefficients).all():
            return None

        return (
            propagated_coefficients,
            float(np.min(forward_new)),
            float(np.max(forward_new)),
        )

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

        matching_status = self._find_matching_status_for_pair(
            self.latest_left,
            self.latest_right,
        )
        if matching_status is None:
            return

        self.selected_curve_status = matching_status
        self.last_processed_pair = pair
        self._process_pair(self.latest_left, self.latest_right)

    def _find_matching_status_for_pair(
        self,
        left_message: Path,
        right_message: Path,
    ) -> dict | None:
        left_coefficients = self._coefficients_from_path(left_message)
        right_coefficients = self._coefficients_from_path(right_message)

        for status in reversed(self.curve_status_history):
            if self._status_matches_path_coefficients(
                status,
                "left",
                left_coefficients,
            ) and self._status_matches_path_coefficients(
                status,
                "right",
                right_coefficients,
            ):
                return status

        return None

    def _status_matches_path_coefficients(
        self,
        status: dict,
        side: str,
        path_coefficients: np.ndarray | None,
    ) -> bool:
        diagnostics = status.get(side)

        if path_coefficients is None:
            return diagnostics is None

        if not isinstance(diagnostics, dict):
            return False

        values = diagnostics.get("coefficients")
        if not isinstance(values, list) or len(values) != 3:
            return False

        try:
            status_coefficients = np.asarray(values, dtype=np.float64)
        except (TypeError, ValueError):
            return False

        if not np.isfinite(status_coefficients).all():
            return False

        forward = self.association_forward_values
        difference = np.abs(
            np.polyval(status_coefficients, forward)
            - np.polyval(path_coefficients, forward)
        )
        return bool(
            float(np.max(difference))
            <= self.curve_status_match_tolerance_m
        )

    @staticmethod
    def _coefficients_from_path(message: Path) -> np.ndarray | None:
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
        points = points[np.isfinite(points).all(axis=1)]

        if len(points) < 3 or float(np.ptp(points[:, 0])) < 2.0:
            return None

        try:
            return np.polyfit(points[:, 0], points[:, 1], 2)
        except (np.linalg.LinAlgError, ValueError, TypeError):
            return None

    def _process_pair(self, left_message: Path, right_message: Path) -> None:
        lane_stamp_ns = self._stamp_ns(left_message)
        self._prepare_motion_for_frame(lane_stamp_ns)

        self.current_raw_pair_confidence = None
        self.current_curve_status_frame = None
        self.current_lane_width_update_applied = False
        self.current_lane_width_update_reason = "not_evaluated"
        self.current_measurement_gate = {
            "left": "not_evaluated",
            "right": "not_evaluated",
        }

        if self.selected_curve_status is not None:
            status_frame = self.selected_curve_status.get("frame")
            if status_frame is not None:
                self.current_curve_status_frame = int(status_frame)

            pair_confidence = self.selected_curve_status.get(
                "pair_confidence"
            )
            if pair_confidence is not None:
                self.current_raw_pair_confidence = float(
                    np.clip(float(pair_confidence), 0.0, 1.0)
                )

        left_measurement = self._measurement_from_path(
            left_message,
            "left",
        )
        right_measurement = self._measurement_from_path(
            right_message,
            "right",
        )

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
            self.current_measurement_gate[side] = "missing"
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
            self.current_measurement_gate[side] = (
                "rejected_nonfinite_points"
            )
            return None

        forward = points[:, 0]
        lateral = points[:, 1]
        forward_span = float(np.ptp(forward))

        if forward_span < 2.0:
            self.current_measurement_gate[side] = (
                "rejected_short_forward_span"
            )
            return None

        try:
            coefficients = np.polyfit(forward, lateral, 2)
        except (np.linalg.LinAlgError, ValueError, TypeError):
            self.current_measurement_gate[side] = "rejected_polyfit"
            return None

        lateral_at_reference = float(
            np.polyval(coefficients, self.reference_forward_m)
        )

        if (
            abs(lateral_at_reference)
            > self.maximum_single_boundary_abs_lateral_m
        ):
            self.current_measurement_gate[side] = (
                "rejected_far_from_ego_lane"
            )
            return None

        if (
            side == "left"
            and lateral_at_reference
            < -self.single_boundary_side_tolerance_m
        ):
            self.current_measurement_gate[side] = (
                "rejected_wrong_side"
            )
            return None

        if (
            side == "right"
            and lateral_at_reference
            > self.single_boundary_side_tolerance_m
        ):
            self.current_measurement_gate[side] = (
                "rejected_wrong_side"
            )
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
            self.current_measurement_gate[side] = (
                "rejected_low_confidence"
            )
            return None

        self.current_measurement_gate[side] = (
            "accepted_curve_status"
            if quality_source == "curve_status"
            else "accepted_path_fallback"
        )

        status_frame = None
        pair_confidence = None
        if (
            quality_source == "curve_status"
            and self.selected_curve_status is not None
        ):
            frame = self.selected_curve_status.get("frame")
            if frame is not None:
                status_frame = int(frame)

            raw_pair_confidence = self.selected_curve_status.get(
                "pair_confidence"
            )
            if raw_pair_confidence is not None:
                pair_confidence = float(
                    np.clip(float(raw_pair_confidence), 0.0, 1.0)
                )

        return BoundaryMeasurement(
            coefficients=coefficients,
            forward_min_m=forward_min_m,
            forward_max_m=forward_max_m,
            point_count=point_count,
            rmse_m=rmse_m,
            confidence=confidence,
            quality_source=quality_source,
            status_frame=status_frame,
            pair_confidence=pair_confidence,
        )

    def _matching_curve_diagnostics(
        self,
        side: str,
        path_coefficients: np.ndarray,
    ) -> dict | None:
        status = self.selected_curve_status
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
            self.current_lane_width_update_reason = "incomplete_pair"
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
            trusted_pair = (
                self.current_raw_pair_confidence is not None
                and self.current_raw_pair_confidence
                >= self.minimum_pair_confidence_for_width_update
            )

            if trusted_pair:
                if self.lane_width_confidence <= 0.0:
                    self.lane_width_m = mean_width
                else:
                    alpha = self.lane_width_smoothing_alpha
                    self.lane_width_m = (
                        (1.0 - alpha) * self.lane_width_m
                        + alpha * mean_width
                    )

                track_pair_confidence = math.sqrt(
                    max(self.left_track.confidence, 0.0)
                    * max(self.right_track.confidence, 0.0)
                )
                pair_confidence = min(
                    track_pair_confidence,
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
                self.current_lane_width_update_applied = True
                self.current_lane_width_update_reason = "trusted_pair"
            else:
                self.lane_width_confidence *= 0.995
                self.current_lane_width_update_reason = (
                    "pair_confidence_unavailable"
                    if self.current_raw_pair_confidence is None
                    else "pair_confidence_below_threshold"
                )

            return left, right

        self.lane_width_confidence *= 0.92
        self.current_lane_width_update_reason = "geometry_rejected"

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

        motion = self.motion_status
        age_ms = motion.get("odom_age_ms")
        age_text = (
            f"{float(age_ms):.0f}ms"
            if age_ms is not None
            else "n/a"
        )
        cv2.putText(
            image,
            (
                f"odom: {motion.get('reason', 'unknown')} "
                f"age={age_text}"
            ),
            (6, self.height - 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.35,
            (220, 220, 220),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            (
                f"dF={motion.get('delta_forward_m', 0.0):+.2f}m "
                f"dL={motion.get('delta_lateral_m', 0.0):+.2f}m "
                f"dYaw={motion.get('delta_yaw_rad', 0.0):+.3f}"
            ),
            (6, self.height - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.35,
            (220, 220, 220),
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
                "measurement_gate": self.current_measurement_gate,
                "motion": self.motion_status,
                "lane_width_estimate_m": self.lane_width_m,
                "lane_width_confidence": self.lane_width_confidence,
                "lane_width_update_applied": (
                    self.current_lane_width_update_applied
                ),
                "lane_width_update_reason": (
                    self.current_lane_width_update_reason
                ),
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
                    "status_frame": measurement.status_frame,
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
