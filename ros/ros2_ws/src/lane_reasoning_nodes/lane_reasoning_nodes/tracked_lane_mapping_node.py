#!/usr/bin/env python3
from __future__ import annotations

import json
import math
from collections import Counter, deque
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np
import rclpy
from autonomy_interfaces.msg import (
    LaneBoundary,
    LaneMap,
    LaneSample,
    LaneSegment,
)
from geometry_msgs.msg import Point32
from nav_msgs.msg import OccupancyGrid, Odometry, Path
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
class LaneObservation:
    stamp_ns: int
    left_world: np.ndarray | None
    right_world: np.ndarray | None
    center_world: np.ndarray | None
    left_confidence: float
    right_confidence: float
    center_confidence: float


@dataclass(frozen=True)
class SideBinEvidence:
    forwards: np.ndarray
    lateral: np.ndarray
    geometry_weight: np.ndarray
    confidence: np.ndarray
    support_count: np.ndarray
    freshness: np.ndarray
    consistency: np.ndarray


class TrackedLaneMappingNode(Node):
    """Build a short-term, route-ready lane map from tracked lane paths.

    The mapper stores direct lane observations as world-coordinate vectors.
    On publication it reprojects those vectors into the current ego frame to
    produce rolling raster layers and a vector ``LaneMap`` suitable for later
    route and local-planning integration.

    Held and inferred tracker outputs are intentionally not reinserted as new
    evidence. This prevents temporal memory from amplifying itself.
    """

    def __init__(self) -> None:
        super().__init__("tracked_lane_mapping_node")

        defaults = {
            "left_topic": "/perception/lane/tracked_left_boundary",
            "right_topic": "/perception/lane/tracked_right_boundary",
            "center_topic": "/perception/lane/tracked_centerline",
            "tracking_status_topic": "/perception/lane/tracking_status",
            "odom_topic": "/carla/hero_odom",
            "left_grid_topic": "/perception/lane/local_map/left_grid",
            "right_grid_topic": "/perception/lane/local_map/right_grid",
            "center_grid_topic": (
                "/perception/lane/local_map/centerline_grid"
            ),
            "combined_grid_topic": (
                "/perception/lane/local_map/combined_grid"
            ),
            "vector_map_topic": "/perception/lane/local_map/vector",
            "debug_topic": "/perception/lane/local_map/debug/compressed",
            "status_topic": "/perception/lane/local_map/status",
            "world_frame_id": "carla_world",
            "ego_frame_id": "hero",
            "forward_horizon_m": 50.0,
            "backward_horizon_m": 10.0,
            "lateral_horizon_m": 12.0,
            "grid_resolution_m": 0.20,
            "vector_sample_spacing_m": 0.50,
            "history_seconds": 8.0,
            "maximum_observations": 160,
            "evidence_decay_tau_seconds": 5.0,
            "evidence_saturation": 2.0,
            "minimum_side_confidence": 0.45,
            "minimum_center_confidence": 0.40,
            "minimum_pair_confidence": 0.30,
            "minimum_bin_weight": 0.30,
            "minimum_segment_length_m": 6.0,
            "minimum_lane_width_m": 2.5,
            "maximum_lane_width_m": 4.8,

            # Confidence remains bounded by the contributing tracker
            # confidence. Repeated support strengthens geometry but does not
            # make perception certainty converge artificially to one.
            "confidence_consistency_sigma_m": 0.30,
            "segment_confidence_percentile": 20.0,

            # Smooth mapped boundaries before arc-length differentiation.
            # Endpoint curvature is copied from the nearest reliable interior
            # sample so one-sided numerical derivatives do not create spikes.
            "centerline_smoothing_passes": 2,
            "curvature_endpoint_guard_samples": 2,

            "line_thickness_cells": 2,
            "publish_rate_hz": 10.0,
            "debug_jpeg_quality": 85,
            "odom_buffer_seconds": 4.0,
            "maximum_odom_age_ms": 150.0,
            "maximum_current_odom_age_ms": 300.0,
            "odom_y_sign": -1.0,
            "odom_yaw_sign": -1.0,
            "maximum_pending_frames": 64,
        }

        for name, value in defaults.items():
            self.declare_parameter(name, value)

        p = lambda name: self.get_parameter(name).value

        self.left_topic = str(p("left_topic"))
        self.right_topic = str(p("right_topic"))
        self.center_topic = str(p("center_topic"))
        self.tracking_status_topic = str(p("tracking_status_topic"))
        self.odom_topic = str(p("odom_topic"))

        self.left_grid_topic = str(p("left_grid_topic"))
        self.right_grid_topic = str(p("right_grid_topic"))
        self.center_grid_topic = str(p("center_grid_topic"))
        self.combined_grid_topic = str(p("combined_grid_topic"))
        self.vector_map_topic = str(p("vector_map_topic"))
        self.debug_topic = str(p("debug_topic"))
        self.status_topic = str(p("status_topic"))

        self.world_frame_id = str(p("world_frame_id"))
        self.ego_frame_id = str(p("ego_frame_id"))

        self.forward_horizon_m = float(p("forward_horizon_m"))
        self.backward_horizon_m = float(p("backward_horizon_m"))
        self.lateral_horizon_m = float(p("lateral_horizon_m"))
        self.grid_resolution_m = float(p("grid_resolution_m"))
        self.vector_sample_spacing_m = float(
            p("vector_sample_spacing_m")
        )

        self.history_seconds = float(p("history_seconds"))
        self.maximum_observations = int(p("maximum_observations"))
        self.evidence_decay_tau_seconds = float(
            p("evidence_decay_tau_seconds")
        )
        self.evidence_saturation = float(p("evidence_saturation"))

        self.minimum_side_confidence = float(
            p("minimum_side_confidence")
        )
        self.minimum_center_confidence = float(
            p("minimum_center_confidence")
        )
        self.minimum_pair_confidence = float(
            p("minimum_pair_confidence")
        )
        self.minimum_bin_weight = float(p("minimum_bin_weight"))
        self.minimum_segment_length_m = float(
            p("minimum_segment_length_m")
        )
        self.minimum_lane_width_m = float(p("minimum_lane_width_m"))
        self.maximum_lane_width_m = float(p("maximum_lane_width_m"))
        self.confidence_consistency_sigma_m = float(
            p("confidence_consistency_sigma_m")
        )
        self.segment_confidence_percentile = float(
            p("segment_confidence_percentile")
        )
        self.centerline_smoothing_passes = max(
            0,
            int(p("centerline_smoothing_passes")),
        )
        self.curvature_endpoint_guard_samples = max(
            0,
            int(p("curvature_endpoint_guard_samples")),
        )

        self.line_thickness_cells = max(
            1,
            int(p("line_thickness_cells")),
        )
        self.publish_rate_hz = float(p("publish_rate_hz"))
        self.debug_jpeg_quality = int(p("debug_jpeg_quality"))

        self.odom_buffer_seconds = float(p("odom_buffer_seconds"))
        self.maximum_odom_age_ns = int(
            float(p("maximum_odom_age_ms")) * 1_000_000.0
        )
        self.maximum_current_odom_age_ns = int(
            float(p("maximum_current_odom_age_ms")) * 1_000_000.0
        )
        self.odom_y_sign = float(p("odom_y_sign"))
        self.odom_yaw_sign = float(p("odom_yaw_sign"))
        self.maximum_pending_frames = max(
            8,
            int(p("maximum_pending_frames")),
        )

        self._validate_parameters()

        self.forward_cells = int(
            math.ceil(
                (
                    self.forward_horizon_m
                    + self.backward_horizon_m
                )
                / self.grid_resolution_m
            )
        )
        self.lateral_cells = int(
            math.ceil(
                (2.0 * self.lateral_horizon_m)
                / self.grid_resolution_m
            )
        )

        self.odom_buffer: deque[OdomPose] = deque()
        self.observations: deque[LaneObservation] = deque(
            maxlen=self.maximum_observations
        )
        self.pending_frames: dict[int, dict[str, Any]] = {}
        self.processed_stamps: deque[int] = deque(maxlen=512)
        self.processed_stamp_set: set[int] = set()

        self.map_revision = 0
        self.received_path_messages = 0
        self.received_status_messages = 0
        self.integrated_frames = 0
        self.last_integrated_stamp_ns: int | None = None
        self.last_frame_odom_age_ms: float | None = None
        self.drop_reasons: Counter[str] = Counter()

        self.last_left_evidence = np.zeros(
            (self.lateral_cells, self.forward_cells),
            dtype=np.float32,
        )
        self.last_right_evidence = np.zeros_like(self.last_left_evidence)
        self.last_center_evidence = np.zeros_like(self.last_left_evidence)
        self.last_segment_length_m = 0.0
        self.last_segment_available = False
        self.last_segment_confidence = 0.0
        self.last_max_abs_curvature_per_m = 0.0
        self.last_left_min_support = 0
        self.last_right_min_support = 0

        self.left_sub = self.create_subscription(
            Path,
            self.left_topic,
            lambda message: self._path_callback("left", message),
            qos_profile_sensor_data,
        )
        self.right_sub = self.create_subscription(
            Path,
            self.right_topic,
            lambda message: self._path_callback("right", message),
            qos_profile_sensor_data,
        )
        self.center_sub = self.create_subscription(
            Path,
            self.center_topic,
            lambda message: self._path_callback("center", message),
            qos_profile_sensor_data,
        )
        self.status_sub = self.create_subscription(
            String,
            self.tracking_status_topic,
            self._status_callback,
            20,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self._odom_callback,
            100,
        )

        self.left_grid_pub = self.create_publisher(
            OccupancyGrid,
            self.left_grid_topic,
            10,
        )
        self.right_grid_pub = self.create_publisher(
            OccupancyGrid,
            self.right_grid_topic,
            10,
        )
        self.center_grid_pub = self.create_publisher(
            OccupancyGrid,
            self.center_grid_topic,
            10,
        )
        self.combined_grid_pub = self.create_publisher(
            OccupancyGrid,
            self.combined_grid_topic,
            10,
        )
        self.vector_map_pub = self.create_publisher(
            LaneMap,
            self.vector_map_topic,
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

        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self._publish_map,
        )

        self.get_logger().info(
            "Route-ready tracked lane mapper started."
        )
        self.get_logger().info(
            f"Vector map: {self.vector_map_topic} [{self.world_frame_id}]"
        )
        self.get_logger().info(
            "Only direct accepted tracker measurements are integrated."
        )

    def _validate_parameters(self) -> None:
        if self.forward_horizon_m <= 0.0:
            raise ValueError("forward_horizon_m must be positive")
        if self.backward_horizon_m < 0.0:
            raise ValueError("backward_horizon_m must be non-negative")
        if self.lateral_horizon_m <= 0.0:
            raise ValueError("lateral_horizon_m must be positive")
        if self.grid_resolution_m <= 0.0:
            raise ValueError("grid_resolution_m must be positive")
        if self.vector_sample_spacing_m <= 0.0:
            raise ValueError("vector_sample_spacing_m must be positive")
        if self.history_seconds <= 0.0:
            raise ValueError("history_seconds must be positive")
        if self.maximum_observations <= 0:
            raise ValueError("maximum_observations must be positive")
        if self.evidence_decay_tau_seconds <= 0.0:
            raise ValueError(
                "evidence_decay_tau_seconds must be positive"
            )
        if self.evidence_saturation <= 0.0:
            raise ValueError("evidence_saturation must be positive")
        if self.minimum_lane_width_m >= self.maximum_lane_width_m:
            raise ValueError("invalid lane-width range")
        if self.confidence_consistency_sigma_m <= 0.0:
            raise ValueError(
                "confidence_consistency_sigma_m must be positive"
            )
        if not 0.0 <= self.segment_confidence_percentile <= 100.0:
            raise ValueError(
                "segment_confidence_percentile must be in [0, 100]"
            )
        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        if self.odom_buffer_seconds <= 0.0:
            raise ValueError("odom_buffer_seconds must be positive")

    @staticmethod
    def _stamp_ns_from_path(message: Path) -> int:
        stamp = message.header.stamp
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    @staticmethod
    def _yaw_from_quaternion(orientation) -> float:
        siny_cosp = 2.0 * (
            float(orientation.w) * float(orientation.z)
            + float(orientation.x) * float(orientation.y)
        )
        cosy_cosp = 1.0 - 2.0 * (
            float(orientation.y) ** 2
            + float(orientation.z) ** 2
        )
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _odom_callback(self, message: Odometry) -> None:
        stamp = message.header.stamp
        stamp_ns = (
            int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        )
        if stamp_ns <= 0:
            return

        pose = message.pose.pose
        converted = OdomPose(
            stamp_ns=stamp_ns,
            x_m=float(pose.position.x),
            y_m=self.odom_y_sign * float(pose.position.y),
            yaw_rad=self.odom_yaw_sign
            * self._yaw_from_quaternion(pose.orientation),
        )
        self.odom_buffer.append(converted)

        cutoff_ns = stamp_ns - int(
            self.odom_buffer_seconds * 1_000_000_000.0
        )
        while (
            len(self.odom_buffer) > 2
            and self.odom_buffer[1].stamp_ns < cutoff_ns
        ):
            self.odom_buffer.popleft()

    def _path_callback(self, kind: str, message: Path) -> None:
        self.received_path_messages += 1
        stamp_ns = self._stamp_ns_from_path(message)
        if stamp_ns <= 0:
            self.drop_reasons["path_without_stamp"] += 1
            return

        frame = self.pending_frames.setdefault(stamp_ns, {})
        frame[kind] = message
        self._try_process_pending(stamp_ns)
        self._prune_pending_frames()

    def _status_callback(self, message: String) -> None:
        self.received_status_messages += 1
        try:
            status = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            self.drop_reasons["malformed_status"] += 1
            return

        if not isinstance(status, dict):
            self.drop_reasons["non_dict_status"] += 1
            return

        stamp_ns = status.get("stamp_ns")
        if stamp_ns is None:
            self.drop_reasons["status_without_stamp"] += 1
            return

        try:
            stamp_ns = int(stamp_ns)
        except (TypeError, ValueError):
            self.drop_reasons["invalid_status_stamp"] += 1
            return

        frame = self.pending_frames.setdefault(stamp_ns, {})
        frame["status"] = status
        self._try_process_pending(stamp_ns)
        self._prune_pending_frames()

    def _try_process_pending(self, stamp_ns: int) -> None:
        if stamp_ns in self.processed_stamp_set:
            self.pending_frames.pop(stamp_ns, None)
            return

        frame = self.pending_frames.get(stamp_ns)
        if frame is None:
            return

        required = {"left", "right", "center", "status"}
        if not required.issubset(frame):
            return

        self.pending_frames.pop(stamp_ns, None)
        self._remember_processed_stamp(stamp_ns)
        self._integrate_frame(
            stamp_ns=stamp_ns,
            left_path=frame["left"],
            right_path=frame["right"],
            center_path=frame["center"],
            status=frame["status"],
        )

    def _remember_processed_stamp(self, stamp_ns: int) -> None:
        if len(self.processed_stamps) == self.processed_stamps.maxlen:
            oldest = self.processed_stamps.popleft()
            self.processed_stamp_set.discard(oldest)
        self.processed_stamps.append(stamp_ns)
        self.processed_stamp_set.add(stamp_ns)

    def _prune_pending_frames(self) -> None:
        if len(self.pending_frames) <= self.maximum_pending_frames:
            return
        for stamp_ns in sorted(self.pending_frames)[
            : len(self.pending_frames) - self.maximum_pending_frames
        ]:
            self.pending_frames.pop(stamp_ns, None)
            self.drop_reasons["pending_overflow"] += 1

    def _pose_at(self, stamp_ns: int) -> tuple[OdomPose | None, int | None]:
        if not self.odom_buffer:
            return None, None

        poses = list(self.odom_buffer)
        if stamp_ns <= poses[0].stamp_ns:
            age_ns = poses[0].stamp_ns - stamp_ns
            return (
                (poses[0], age_ns)
                if age_ns <= self.maximum_odom_age_ns
                else (None, age_ns)
            )

        if stamp_ns >= poses[-1].stamp_ns:
            age_ns = stamp_ns - poses[-1].stamp_ns
            return (
                (poses[-1], age_ns)
                if age_ns <= self.maximum_odom_age_ns
                else (None, age_ns)
            )

        for first, second in zip(poses[:-1], poses[1:]):
            if first.stamp_ns <= stamp_ns <= second.stamp_ns:
                span_ns = second.stamp_ns - first.stamp_ns
                if span_ns <= 0:
                    return first, abs(stamp_ns - first.stamp_ns)

                alpha = (stamp_ns - first.stamp_ns) / span_ns
                yaw_delta = self._normalize_angle(
                    second.yaw_rad - first.yaw_rad
                )
                interpolated = OdomPose(
                    stamp_ns=stamp_ns,
                    x_m=(1.0 - alpha) * first.x_m + alpha * second.x_m,
                    y_m=(1.0 - alpha) * first.y_m + alpha * second.y_m,
                    yaw_rad=self._normalize_angle(
                        first.yaw_rad + alpha * yaw_delta
                    ),
                )
                age_ns = min(
                    stamp_ns - first.stamp_ns,
                    second.stamp_ns - stamp_ns,
                )
                if age_ns <= self.maximum_odom_age_ns:
                    return interpolated, age_ns
                return None, age_ns

        return None, None

    @staticmethod
    def _path_points(message: Path) -> np.ndarray | None:
        if len(message.poses) < 2:
            return None
        points = np.asarray(
            [
                [
                    float(pose.pose.position.x),
                    float(pose.pose.position.y),
                ]
                for pose in message.poses
            ],
            dtype=np.float64,
        )
        points = points[np.isfinite(points).all(axis=1)]
        if len(points) < 2:
            return None
        return points

    @staticmethod
    def _ego_to_world(points: np.ndarray, pose: OdomPose) -> np.ndarray:
        cosine = math.cos(pose.yaw_rad)
        sine = math.sin(pose.yaw_rad)
        rotation = np.asarray(
            [[cosine, -sine], [sine, cosine]],
            dtype=np.float64,
        )
        return points @ rotation.T + np.asarray(
            [pose.x_m, pose.y_m],
            dtype=np.float64,
        )

    @staticmethod
    def _world_to_ego(points: np.ndarray, pose: OdomPose) -> np.ndarray:
        translated = points - np.asarray(
            [pose.x_m, pose.y_m],
            dtype=np.float64,
        )
        cosine = math.cos(pose.yaw_rad)
        sine = math.sin(pose.yaw_rad)
        inverse_rotation = np.asarray(
            [[cosine, sine], [-sine, cosine]],
            dtype=np.float64,
        )
        return translated @ inverse_rotation.T

    @staticmethod
    def _direct_side(
        side_status: Any,
        path: Path,
        minimum_confidence: float,
    ) -> tuple[np.ndarray | None, float, str]:
        if not isinstance(side_status, dict):
            return None, 0.0, "missing_side_status"

        state = str(side_status.get("state", ""))
        if state not in {"accepted", "confirmed", "replaced"}:
            return None, 0.0, f"state_{state or 'unknown'}"

        if bool(side_status.get("inferred", False)):
            return None, 0.0, "inferred"

        if side_status.get("measurement") is None:
            return None, 0.0, "no_direct_measurement"

        try:
            confidence = float(side_status.get("output_confidence", 0.0))
        except (TypeError, ValueError):
            confidence = 0.0

        if confidence < minimum_confidence:
            return None, confidence, "low_confidence"

        points = TrackedLaneMappingNode._path_points(path)
        if points is None:
            return None, confidence, "empty_path"

        return points, confidence, "accepted"

    def _integrate_frame(
        self,
        stamp_ns: int,
        left_path: Path,
        right_path: Path,
        center_path: Path,
        status: dict[str, Any],
    ) -> None:
        pose, odom_age_ns = self._pose_at(stamp_ns)
        if pose is None:
            self.drop_reasons["odom_unavailable"] += 1
            return

        left_points, left_confidence, left_reason = self._direct_side(
            status.get("left"),
            left_path,
            self.minimum_side_confidence,
        )
        right_points, right_confidence, right_reason = self._direct_side(
            status.get("right"),
            right_path,
            self.minimum_side_confidence,
        )

        if left_points is None:
            self.drop_reasons[f"left_{left_reason}"] += 1
        if right_points is None:
            self.drop_reasons[f"right_{right_reason}"] += 1

        left_world = (
            self._ego_to_world(left_points, pose)
            if left_points is not None
            else None
        )
        right_world = (
            self._ego_to_world(right_points, pose)
            if right_points is not None
            else None
        )

        center_world = None
        center_confidence = 0.0

        try:
            pair_confidence = float(
                status.get("raw_pair_confidence", 0.0) or 0.0
            )
        except (TypeError, ValueError):
            pair_confidence = 0.0

        if (
            left_world is not None
            and right_world is not None
            and bool(status.get("centerline_available", False))
            and pair_confidence >= self.minimum_pair_confidence
        ):
            center_points = self._path_points(center_path)
            center_confidence = min(
                left_confidence,
                right_confidence,
                pair_confidence,
            )
            if (
                center_points is not None
                and center_confidence >= self.minimum_center_confidence
            ):
                center_world = self._ego_to_world(center_points, pose)

        if left_world is None and right_world is None:
            self.drop_reasons["no_direct_sides"] += 1
            return

        observation = LaneObservation(
            stamp_ns=stamp_ns,
            left_world=left_world,
            right_world=right_world,
            center_world=center_world,
            left_confidence=float(left_confidence),
            right_confidence=float(right_confidence),
            center_confidence=float(center_confidence),
        )
        self.observations.append(observation)
        self.integrated_frames += 1
        self.last_integrated_stamp_ns = stamp_ns
        self.map_revision += 1

        self.last_frame_odom_age_ms = (
            None
            if odom_age_ns is None
            else odom_age_ns / 1_000_000.0
        )

    def _prune_observations(self, current_stamp_ns: int) -> None:
        cutoff_ns = current_stamp_ns - int(
            self.history_seconds * 1_000_000_000.0
        )
        removed = 0
        while self.observations and self.observations[0].stamp_ns < cutoff_ns:
            self.observations.popleft()
            removed += 1
        if removed:
            self.map_revision += 1
            self.drop_reasons["expired_observations"] += removed

    def _age_decay(
        self,
        observation_stamp_ns: int,
        current_stamp_ns: int,
    ) -> float:
        age_seconds = max(
            0.0,
            (current_stamp_ns - observation_stamp_ns) / 1_000_000_000.0,
        )
        return float(
            math.exp(-age_seconds / self.evidence_decay_tau_seconds)
        )

    def _observation_weight(
        self,
        confidence: float,
        observation_stamp_ns: int,
        current_stamp_ns: int,
    ) -> float:
        return float(
            max(0.0, confidence)
            * self._age_decay(observation_stamp_ns, current_stamp_ns)
        )

    def _metric_points_to_grid(
        self,
        points_ego: np.ndarray,
    ) -> np.ndarray:
        forward = points_ego[:, 0]
        lateral = points_ego[:, 1]
        columns = np.rint(
            (forward + self.backward_horizon_m)
            / self.grid_resolution_m
        ).astype(np.int32)
        rows = np.rint(
            (lateral + self.lateral_horizon_m)
            / self.grid_resolution_m
        ).astype(np.int32)
        return np.column_stack([columns, rows])

    def _add_path_evidence(
        self,
        evidence: np.ndarray,
        points_world: np.ndarray | None,
        pose: OdomPose,
        weight: float,
    ) -> None:
        if points_world is None or weight <= 0.0:
            return

        points_ego = self._world_to_ego(points_world, pose)
        in_bounds = (
            (points_ego[:, 0] >= -self.backward_horizon_m)
            & (points_ego[:, 0] <= self.forward_horizon_m)
            & (points_ego[:, 1] >= -self.lateral_horizon_m)
            & (points_ego[:, 1] <= self.lateral_horizon_m)
        )
        points_ego = points_ego[in_bounds]
        if len(points_ego) < 2:
            return

        pixels = self._metric_points_to_grid(points_ego)
        mask = np.zeros_like(evidence, dtype=np.uint8)
        cv2.polylines(
            mask,
            [pixels.reshape(-1, 1, 2)],
            isClosed=False,
            color=255,
            thickness=self.line_thickness_cells,
            lineType=cv2.LINE_AA,
        )
        evidence += weight * (mask.astype(np.float32) / 255.0)

    def _rasterize(
        self,
        pose: OdomPose,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        left = np.zeros(
            (self.lateral_cells, self.forward_cells),
            dtype=np.float32,
        )
        right = np.zeros_like(left)
        center = np.zeros_like(left)

        for observation in self.observations:
            self._add_path_evidence(
                left,
                observation.left_world,
                pose,
                self._observation_weight(
                    observation.left_confidence,
                    observation.stamp_ns,
                    pose.stamp_ns,
                ),
            )
            self._add_path_evidence(
                right,
                observation.right_world,
                pose,
                self._observation_weight(
                    observation.right_confidence,
                    observation.stamp_ns,
                    pose.stamp_ns,
                ),
            )
            self._add_path_evidence(
                center,
                observation.center_world,
                pose,
                self._observation_weight(
                    observation.center_confidence,
                    observation.stamp_ns,
                    pose.stamp_ns,
                ),
            )

        return left, right, center

    def _occupancy_values(self, evidence: np.ndarray) -> np.ndarray:
        normalized = np.clip(
            evidence / self.evidence_saturation,
            0.0,
            1.0,
        )
        return np.rint(100.0 * normalized).astype(np.int8)

    def _make_grid(
        self,
        evidence: np.ndarray,
        stamp_ns: int,
    ) -> OccupancyGrid:
        message = OccupancyGrid()
        message.header.stamp.sec = int(stamp_ns // 1_000_000_000)
        message.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
        message.header.frame_id = self.ego_frame_id

        message.info.resolution = float(self.grid_resolution_m)
        message.info.width = int(self.forward_cells)
        message.info.height = int(self.lateral_cells)
        message.info.origin.position.x = -float(self.backward_horizon_m)
        message.info.origin.position.y = -float(self.lateral_horizon_m)
        message.info.origin.position.z = 0.0
        message.info.origin.orientation.w = 1.0
        message.data = self._occupancy_values(evidence).flatten().tolist()
        return message

    def _accumulate_side_bins(
        self,
        side: str,
        pose: OdomPose,
    ) -> SideBinEvidence:
        start = -self.backward_horizon_m
        count = int(
            math.floor(
                (
                    self.forward_horizon_m
                    + self.backward_horizon_m
                )
                / self.vector_sample_spacing_m
            )
        ) + 1
        forwards = start + np.arange(count) * self.vector_sample_spacing_m

        weighted_lateral = np.zeros(count, dtype=np.float64)
        weighted_lateral_squared = np.zeros(count, dtype=np.float64)
        geometry_weight = np.zeros(count, dtype=np.float64)

        confidence_numerator = np.zeros(count, dtype=np.float64)
        confidence_denominator = np.zeros(count, dtype=np.float64)
        freshest_decay = np.zeros(count, dtype=np.float64)
        support_count = np.zeros(count, dtype=np.int32)

        path_attribute = f"{side}_world"
        confidence_attribute = f"{side}_confidence"

        for observation in self.observations:
            points_world = getattr(observation, path_attribute)
            if points_world is None:
                continue

            confidence = float(
                np.clip(
                    float(getattr(observation, confidence_attribute)),
                    0.0,
                    1.0,
                )
            )
            freshness = self._age_decay(
                observation.stamp_ns,
                pose.stamp_ns,
            )
            weight = confidence * freshness
            if weight <= 0.0:
                continue

            points_ego = self._world_to_ego(points_world, pose)
            valid = (
                (points_ego[:, 0] >= start)
                & (points_ego[:, 0] <= self.forward_horizon_m)
                & (np.abs(points_ego[:, 1]) <= self.lateral_horizon_m)
            )
            points_ego = points_ego[valid]
            if len(points_ego) == 0:
                continue

            indices = np.rint(
                (points_ego[:, 0] - start)
                / self.vector_sample_spacing_m
            ).astype(np.int32)
            indices = np.clip(indices, 0, count - 1)

            # Count one independent contribution per observation and bin.
            # Path resampling or transforms can otherwise place two points
            # from the same frame in one bin and inflate support.
            unique_indices, inverse = np.unique(
                indices,
                return_inverse=True,
            )
            lateral_sum = np.zeros(len(unique_indices), dtype=np.float64)
            lateral_count = np.zeros(len(unique_indices), dtype=np.float64)
            np.add.at(lateral_sum, inverse, points_ego[:, 1])
            np.add.at(lateral_count, inverse, 1.0)
            observation_lateral = lateral_sum / np.maximum(
                lateral_count,
                1.0,
            )

            np.add.at(
                weighted_lateral,
                unique_indices,
                weight * observation_lateral,
            )
            np.add.at(
                weighted_lateral_squared,
                unique_indices,
                weight * np.square(observation_lateral),
            )
            np.add.at(geometry_weight, unique_indices, weight)
            np.add.at(
                confidence_numerator,
                unique_indices,
                freshness * confidence,
            )
            np.add.at(
                confidence_denominator,
                unique_indices,
                freshness,
            )
            np.maximum.at(freshest_decay, unique_indices, freshness)
            np.add.at(support_count, unique_indices, 1)

        lateral = np.full(count, np.nan, dtype=np.float64)
        mean_confidence = np.zeros(count, dtype=np.float64)
        consistency = np.zeros(count, dtype=np.float64)

        valid_geometry = geometry_weight > 1e-9
        lateral[valid_geometry] = (
            weighted_lateral[valid_geometry]
            / geometry_weight[valid_geometry]
        )

        valid_confidence = confidence_denominator > 1e-9
        mean_confidence[valid_confidence] = (
            confidence_numerator[valid_confidence]
            / confidence_denominator[valid_confidence]
        )

        variance = np.zeros(count, dtype=np.float64)
        variance[valid_geometry] = np.maximum(
            0.0,
            (
                weighted_lateral_squared[valid_geometry]
                / geometry_weight[valid_geometry]
            )
            - np.square(lateral[valid_geometry]),
        )
        lateral_std = np.sqrt(variance)
        consistency[valid_geometry] = np.exp(
            -0.5
            * np.square(
                lateral_std[valid_geometry]
                / self.confidence_consistency_sigma_m
            )
        )

        confidence = np.clip(
            mean_confidence * freshest_decay * consistency,
            0.0,
            1.0,
        )

        return SideBinEvidence(
            forwards=forwards,
            lateral=lateral,
            geometry_weight=geometry_weight,
            confidence=confidence,
            support_count=support_count,
            freshness=freshest_decay,
            consistency=consistency,
        )

    @staticmethod
    def _longest_contiguous_run(indices: np.ndarray) -> np.ndarray:
        if len(indices) == 0:
            return indices
        split_points = np.where(np.diff(indices) > 1)[0] + 1
        runs = np.split(indices, split_points)
        return max(runs, key=len)

    @staticmethod
    def _smooth(values: np.ndarray) -> np.ndarray:
        if len(values) < 3:
            return values.copy()
        padded = np.pad(values, (1, 1), mode="edge")
        return (
            0.25 * padded[:-2]
            + 0.50 * padded[1:-1]
            + 0.25 * padded[2:]
        )

    def _smooth_passes(self, values: np.ndarray) -> np.ndarray:
        smoothed = values.astype(np.float64, copy=True)
        for _ in range(self.centerline_smoothing_passes):
            smoothed = self._smooth(smoothed)
        return smoothed

    def _signed_curvatures(self, points: np.ndarray) -> np.ndarray:
        count = len(points)
        curvatures = np.zeros(count, dtype=np.float64)
        if count < 5:
            return curvatures

        step_lengths = np.linalg.norm(
            np.diff(points, axis=0),
            axis=1,
        )
        cumulative_s = np.zeros(count, dtype=np.float64)
        cumulative_s[1:] = np.cumsum(step_lengths)

        if float(cumulative_s[-1]) <= 1e-6:
            return curvatures

        for index in range(1, count):
            if cumulative_s[index] <= cumulative_s[index - 1]:
                cumulative_s[index] = (
                    cumulative_s[index - 1] + 1e-6
                )

        # Fit local quadratics x(s) and y(s) over a symmetric window.
        # Derivatives are evaluated at the window centre, so every computed
        # value uses central support rather than one-sided endpoint formulas.
        radius = max(2, self.curvature_endpoint_guard_samples)
        first_reliable = radius
        last_reliable = count - radius - 1

        if first_reliable > last_reliable:
            return curvatures

        for index in range(first_reliable, last_reliable + 1):
            window = slice(index - radius, index + radius + 1)
            local_s = cumulative_s[window] - cumulative_s[index]

            try:
                x_coefficients = np.polyfit(
                    local_s,
                    points[window, 0],
                    2,
                )
                y_coefficients = np.polyfit(
                    local_s,
                    points[window, 1],
                    2,
                )
            except (np.linalg.LinAlgError, ValueError, TypeError):
                continue

            dx_ds = float(x_coefficients[1])
            dy_ds = float(y_coefficients[1])
            d2x_ds2 = 2.0 * float(x_coefficients[0])
            d2y_ds2 = 2.0 * float(y_coefficients[0])

            denominator = (
                dx_ds * dx_ds + dy_ds * dy_ds
            ) ** 1.5
            if denominator <= 1e-9:
                continue

            curvatures[index] = (
                dx_ds * d2y_ds2 - dy_ds * d2x_ds2
            ) / denominator

        # Reject isolated interior differentiation noise while retaining a
        # sustained bend.
        if last_reliable - first_reliable >= 2:
            filtered = curvatures.copy()
            for index in range(
                first_reliable + 1,
                last_reliable,
            ):
                filtered[index] = float(
                    np.median(curvatures[index - 1 : index + 2])
                )
            curvatures = filtered

        # Endpoint samples have no symmetric window. Use the nearest reliable
        # interior estimate instead of publishing numerical artifacts.
        curvatures[:first_reliable] = curvatures[first_reliable]
        curvatures[last_reliable + 1 :] = curvatures[last_reliable]

        return curvatures

    @staticmethod
    def _world_yaw_to_quaternion(yaw_rad: float, pose) -> None:
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = math.sin(0.5 * yaw_rad)
        pose.orientation.w = math.cos(0.5 * yaw_rad)

    @staticmethod
    def _point32(x_m: float, y_m: float) -> Point32:
        point = Point32()
        point.x = float(x_m)
        point.y = float(y_m)
        point.z = 0.0
        return point

    def _build_vector_map(
        self,
        pose: OdomPose,
    ) -> tuple[LaneMap, float]:
        message = LaneMap()
        message.header.stamp.sec = int(pose.stamp_ns // 1_000_000_000)
        message.header.stamp.nanosec = int(
            pose.stamp_ns % 1_000_000_000
        )
        message.header.frame_id = self.world_frame_id
        message.schema_version = 2
        message.map_revision = int(self.map_revision)
        message.globally_consistent = False
        message.forward_horizon_m = float(self.forward_horizon_m)
        message.backward_horizon_m = float(self.backward_horizon_m)
        message.lateral_horizon_m = float(self.lateral_horizon_m)

        self.last_segment_confidence = 0.0
        self.last_max_abs_curvature_per_m = 0.0
        self.last_left_min_support = 0
        self.last_right_min_support = 0

        left_bins = self._accumulate_side_bins("left", pose)
        right_bins = self._accumulate_side_bins("right", pose)

        forwards = left_bins.forwards
        left_lateral = left_bins.lateral
        right_lateral = right_bins.lateral

        widths = left_lateral - right_lateral
        valid = (
            np.isfinite(left_lateral)
            & np.isfinite(right_lateral)
            & (left_bins.geometry_weight >= self.minimum_bin_weight)
            & (right_bins.geometry_weight >= self.minimum_bin_weight)
            & (widths >= self.minimum_lane_width_m)
            & (widths <= self.maximum_lane_width_m)
        )
        run = self._longest_contiguous_run(np.flatnonzero(valid))

        if len(run) < 2:
            return message, 0.0

        segment_length_m = (
            float(forwards[run[-1]] - forwards[run[0]])
        )
        if segment_length_m < self.minimum_segment_length_m:
            return message, segment_length_m

        local_forward = forwards[run]
        local_left = self._smooth_passes(left_lateral[run])
        local_right = self._smooth_passes(right_lateral[run])
        local_center = 0.5 * (local_left + local_right)

        left_local_points = np.column_stack([local_forward, local_left])
        right_local_points = np.column_stack([local_forward, local_right])
        center_local_points = np.column_stack([local_forward, local_center])

        left_world = self._ego_to_world(left_local_points, pose)
        right_world = self._ego_to_world(right_local_points, pose)
        center_world = self._ego_to_world(center_local_points, pose)

        left_confidence = left_bins.confidence[run]
        right_confidence = right_bins.confidence[run]
        sample_confidence = np.sqrt(
            left_confidence * right_confidence
        )
        curvatures = self._signed_curvatures(center_local_points)

        segment_confidence = float(
            np.percentile(
                sample_confidence,
                self.segment_confidence_percentile,
            )
        )
        self.last_segment_confidence = segment_confidence
        self.last_max_abs_curvature_per_m = float(
            np.max(np.abs(curvatures))
        )
        self.last_left_min_support = int(
            np.min(left_bins.support_count[run])
        )
        self.last_right_min_support = int(
            np.min(right_bins.support_count[run])
        )

        segment = LaneSegment()
        segment.id = 1
        segment.predecessor_ids = []
        segment.successor_ids = []
        segment.left_neighbor_id = 0
        segment.right_neighbor_id = 0
        segment.lane_change_left_allowed = False
        segment.lane_change_right_allowed = False
        segment.nominal_width_m = float(np.mean(local_left - local_right))
        segment.speed_limit_known = False
        segment.speed_limit_mps = 0.0
        segment.confidence = segment_confidence
        segment.topology_state = LaneSegment.TOPOLOGY_UNKNOWN
        segment.turn_direction = LaneSegment.TURN_UNKNOWN
        segment.drivable = True
        segment.is_intersection = False

        left_boundary = LaneBoundary()
        left_boundary.id = 2
        left_boundary.side = LaneBoundary.SIDE_LEFT
        left_boundary.source = LaneBoundary.SOURCE_MEMORY
        left_boundary.points = [
            self._point32(point[0], point[1]) for point in left_world
        ]
        left_boundary.point_confidence = [
            float(value) for value in left_confidence
        ]
        left_boundary.mean_confidence = float(np.mean(left_confidence))
        left_boundary.complete = True

        right_boundary = LaneBoundary()
        right_boundary.id = 3
        right_boundary.side = LaneBoundary.SIDE_RIGHT
        right_boundary.source = LaneBoundary.SOURCE_MEMORY
        right_boundary.points = [
            self._point32(point[0], point[1]) for point in right_world
        ]
        right_boundary.point_confidence = [
            float(value) for value in right_confidence
        ]
        right_boundary.mean_confidence = float(np.mean(right_confidence))
        right_boundary.complete = True

        segment.left_boundary = left_boundary
        segment.right_boundary = right_boundary

        cumulative_s = np.zeros(len(center_local_points), dtype=np.float64)
        if len(center_local_points) > 1:
            cumulative_s[1:] = np.cumsum(
                np.linalg.norm(
                    np.diff(center_local_points, axis=0),
                    axis=1,
                )
            )

        for index, point in enumerate(center_world):
            previous_index = max(0, index - 1)
            next_index = min(len(center_local_points) - 1, index + 1)
            tangent = (
                center_local_points[next_index]
                - center_local_points[previous_index]
            )
            local_yaw = math.atan2(float(tangent[1]), float(tangent[0]))
            world_yaw = self._normalize_angle(pose.yaw_rad + local_yaw)

            sample = LaneSample()
            sample.s = float(cumulative_s[index])
            sample.pose.position.x = float(point[0])
            sample.pose.position.y = float(point[1])
            sample.pose.position.z = 0.0
            self._world_yaw_to_quaternion(world_yaw, sample.pose)
            sample.curvature = float(curvatures[index])
            sample.left_width_m = float(local_left[index] - local_center[index])
            sample.right_width_m = float(local_center[index] - local_right[index])
            sample.confidence = float(sample_confidence[index])
            segment.centerline.append(sample)

        message.segments.append(segment)
        return message, float(cumulative_s[-1])

    def _render_debug(
        self,
        left: np.ndarray,
        right: np.ndarray,
        center: np.ndarray,
        segment_available: bool,
        segment_length_m: float,
    ) -> np.ndarray:
        left_value = self._occupancy_values(left).astype(np.uint8)
        right_value = self._occupancy_values(right).astype(np.uint8)
        center_value = self._occupancy_values(center).astype(np.uint8)

        image = np.zeros(
            (self.lateral_cells, self.forward_cells, 3),
            dtype=np.uint8,
        )
        image[:, :, 1] = np.maximum(image[:, :, 1], left_value)
        image[:, :, 2] = np.maximum(image[:, :, 2], right_value)
        image[:, :, 1] = np.maximum(image[:, :, 1], center_value)
        image[:, :, 2] = np.maximum(image[:, :, 2], center_value)

        hero_column = int(
            round(self.backward_horizon_m / self.grid_resolution_m)
        )
        hero_row = int(
            round(self.lateral_horizon_m / self.grid_resolution_m)
        )
        if (
            0 <= hero_row < self.lateral_cells
            and 0 <= hero_column < self.forward_cells
        ):
            cv2.circle(
                image,
                (hero_column, hero_row),
                3,
                (255, 255, 255),
                -1,
            )

        image = np.flipud(image)
        image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        image = cv2.resize(
            image,
            (image.shape[1] * 3, image.shape[0] * 3),
            interpolation=cv2.INTER_NEAREST,
        )

        cv2.putText(
            image,
            "green=left red=right yellow=center",
            (8, 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (230, 230, 230),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            (
                f"obs={len(self.observations)} rev={self.map_revision} "
                f"segment={'yes' if segment_available else 'no'} "
                f"length={segment_length_m:.1f}m"
            ),
            (8, 38),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (230, 230, 230),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            (
                f"conf={self.last_segment_confidence:.2f} "
                f"|k|max={self.last_max_abs_curvature_per_m:.3f}/m"
            ),
            (8, 58),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (230, 230, 230),
            1,
            cv2.LINE_AA,
        )
        return image

    def _publish_debug(
        self,
        image: np.ndarray,
        stamp_ns: int,
    ) -> None:
        ok, encoded = cv2.imencode(
            ".jpg",
            image,
            [
                int(cv2.IMWRITE_JPEG_QUALITY),
                self.debug_jpeg_quality,
            ],
        )
        if not ok:
            return

        message = CompressedImage()
        message.header.stamp.sec = int(stamp_ns // 1_000_000_000)
        message.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
        message.header.frame_id = self.ego_frame_id
        message.format = "jpeg"
        message.data = encoded.tobytes()
        self.debug_pub.publish(message)

    def _publish_status(
        self,
        pose: OdomPose | None,
        current_odom_age_ns: int | None,
    ) -> None:
        status = {
            "schema_version": 2,
            "map_revision": self.map_revision,
            "world_frame_id": self.world_frame_id,
            "ego_frame_id": self.ego_frame_id,
            "odom_available": pose is not None,
            "current_odom_age_ms": (
                None
                if current_odom_age_ns is None
                else current_odom_age_ns / 1_000_000.0
            ),
            "odom_buffer_size": len(self.odom_buffer),
            "observations": len(self.observations),
            "pending_frames": len(self.pending_frames),
            "received_path_messages": self.received_path_messages,
            "received_status_messages": self.received_status_messages,
            "integrated_frames": self.integrated_frames,
            "last_integrated_stamp_ns": self.last_integrated_stamp_ns,
            "last_frame_odom_age_ms": self.last_frame_odom_age_ms,
            "segment_available": self.last_segment_available,
            "segment_length_m": self.last_segment_length_m,
            "segment_confidence": self.last_segment_confidence,
            "max_abs_curvature_per_m": (
                self.last_max_abs_curvature_per_m
            ),
            "left_min_support": self.last_left_min_support,
            "right_min_support": self.last_right_min_support,
            "speed_limit_known": False,
            "left_cells": int(np.count_nonzero(self.last_left_evidence)),
            "right_cells": int(np.count_nonzero(self.last_right_evidence)),
            "center_cells": int(np.count_nonzero(self.last_center_evidence)),
            "drop_reasons": dict(self.drop_reasons),
        }
        message = String()
        message.data = json.dumps(status, separators=(",", ":"))
        self.status_pub.publish(message)

    def _publish_map(self) -> None:
        if not self.odom_buffer:
            self._publish_status(None, None)
            return

        pose = self.odom_buffer[-1]
        now_ns = int(self.get_clock().now().nanoseconds)
        current_odom_age_ns = max(0, now_ns - pose.stamp_ns)
        if current_odom_age_ns > self.maximum_current_odom_age_ns:
            self._publish_status(None, current_odom_age_ns)
            return

        self._prune_observations(pose.stamp_ns)
        left, right, center = self._rasterize(pose)
        combined = np.maximum.reduce([left, right, center])

        self.last_left_evidence = left
        self.last_right_evidence = right
        self.last_center_evidence = center

        self.left_grid_pub.publish(self._make_grid(left, pose.stamp_ns))
        self.right_grid_pub.publish(self._make_grid(right, pose.stamp_ns))
        self.center_grid_pub.publish(self._make_grid(center, pose.stamp_ns))
        self.combined_grid_pub.publish(
            self._make_grid(combined, pose.stamp_ns)
        )

        vector_map, segment_length_m = self._build_vector_map(pose)
        self.last_segment_available = bool(vector_map.segments)
        self.last_segment_length_m = float(segment_length_m)
        self.vector_map_pub.publish(vector_map)

        debug = self._render_debug(
            left,
            right,
            center,
            self.last_segment_available,
            self.last_segment_length_m,
        )
        self._publish_debug(debug, pose.stamp_ns)
        self._publish_status(pose, current_odom_age_ns)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrackedLaneMappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
