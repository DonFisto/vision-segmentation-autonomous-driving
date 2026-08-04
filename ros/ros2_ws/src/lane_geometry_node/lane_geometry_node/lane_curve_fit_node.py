#!/usr/bin/env python3
from __future__ import annotations

import json
import math
from dataclasses import dataclass

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String


@dataclass
class CurveModel:
    coefficients: np.ndarray
    inlier_indices: np.ndarray
    inlier_count: int
    forward_min: float
    forward_max: float
    rmse: float
    score: float
    confidence: float

    def lateral_at(self, forward_m: np.ndarray | float) -> np.ndarray | float:
        return np.polyval(self.coefficients, forward_m)


class LaneCurveFitNode(Node):
    """
    Fit metric lane-boundary curves from the cleaned BEV candidate mask.

    Model convention:
        lateral_left = a * forward^2 + b * forward + c

    Inputs:
        /perception/lane/candidate_mask

    Outputs:
        /perception/lane/left_boundary
        /perception/lane/right_boundary
        /perception/lane/centerline
        /perception/lane/curve_debug/compressed
        /perception/lane/curve_status
    """

    def __init__(self) -> None:
        super().__init__("lane_curve_fit_node")
        self.bridge = CvBridge()

        defaults = {
            "candidate_topic": "/perception/lane/candidate_mask",
            "left_path_topic": "/perception/lane/left_boundary",
            "right_path_topic": "/perception/lane/right_boundary",
            "center_path_topic": "/perception/lane/centerline",
            "debug_topic": "/perception/lane/curve_debug/compressed",
            "status_topic": "/perception/lane/curve_status",
            # Must match the BEV node.
            "forward_min_m": 0.0,
            "forward_max_m": 40.0,
            "left_extent_m": 12.0,
            "right_extent_m": -12.0,
            "resolution_m": 0.10,
            # Reliable fitting region.
            "fit_forward_min_m": 5.0,
            "fit_forward_max_m": 30.0,
            "fit_left_extent_m": 8.0,
            "fit_right_extent_m": -8.0,
            # Point extraction.
            "row_stride_px": 2,
            "maximum_marking_width_m": 0.90,
            # RANSAC.
            "ransac_iterations": 180,
            "ransac_residual_threshold_m": 0.28,
            "ransac_min_points": 18,
            "ransac_min_forward_span_m": 6.0,
            "max_models": 8,
            "max_abs_quadratic_coefficient": 0.025,
            "max_abs_linear_coefficient": 1.20,
            # Ego-lane selection.
            "reference_forward_m": 7.0,
            "expected_lane_width_m": 3.5,
            "minimum_lane_width_m": 2.5,
            "maximum_lane_width_m": 4.8,
            "minimum_boundary_offset_m": 0.25,
            "pair_width_tolerance_m": 1.0,
            # Output.
            "path_start_forward_m": 5.0,
            "path_end_forward_m": 30.0,
            "path_step_m": 0.5,
            "grid_spacing_m": 5.0,
            "jpeg_quality": 80,
            "random_seed": 42,
            "log_every": 30,
        }

        for name, value in defaults.items():
            self.declare_parameter(name, value)

        p = lambda name: self.get_parameter(name).value

        self.candidate_topic = str(p("candidate_topic"))
        self.left_path_topic = str(p("left_path_topic"))
        self.right_path_topic = str(p("right_path_topic"))
        self.center_path_topic = str(p("center_path_topic"))
        self.debug_topic = str(p("debug_topic"))
        self.status_topic = str(p("status_topic"))

        self.forward_min_m = float(p("forward_min_m"))
        self.forward_max_m = float(p("forward_max_m"))
        self.left_extent_m = float(p("left_extent_m"))
        self.right_extent_m = float(p("right_extent_m"))
        self.resolution_m = float(p("resolution_m"))

        self.fit_forward_min_m = float(p("fit_forward_min_m"))
        self.fit_forward_max_m = float(p("fit_forward_max_m"))
        self.fit_left_extent_m = float(p("fit_left_extent_m"))
        self.fit_right_extent_m = float(p("fit_right_extent_m"))

        self.row_stride_px = max(1, int(p("row_stride_px")))
        self.maximum_marking_width_m = float(
            p("maximum_marking_width_m")
        )

        self.ransac_iterations = int(p("ransac_iterations"))
        self.residual_threshold_m = float(
            p("ransac_residual_threshold_m")
        )
        self.ransac_min_points = int(p("ransac_min_points"))
        self.ransac_min_span_m = float(
            p("ransac_min_forward_span_m")
        )
        self.max_models = int(p("max_models"))
        self.max_abs_quadratic = float(
            p("max_abs_quadratic_coefficient")
        )
        self.max_abs_linear = float(
            p("max_abs_linear_coefficient")
        )

        self.reference_forward_m = float(
            p("reference_forward_m")
        )
        self.expected_lane_width_m = float(
            p("expected_lane_width_m")
        )
        self.minimum_lane_width_m = float(
            p("minimum_lane_width_m")
        )
        self.maximum_lane_width_m = float(
            p("maximum_lane_width_m")
        )
        self.minimum_boundary_offset_m = float(
            p("minimum_boundary_offset_m")
        )
        self.pair_width_tolerance_m = float(
            p("pair_width_tolerance_m")
        )

        self.path_start_forward_m = float(
            p("path_start_forward_m")
        )
        self.path_end_forward_m = float(
            p("path_end_forward_m")
        )
        self.path_step_m = float(p("path_step_m"))
        self.grid_spacing_m = float(p("grid_spacing_m"))
        self.jpeg_quality = int(p("jpeg_quality"))
        self.random_seed = int(p("random_seed"))
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

        self.subscription = self.create_subscription(
            Image,
            self.candidate_topic,
            self._callback,
            qos_profile_sensor_data,
        )

        self.left_path_pub = self.create_publisher(
            Path,
            self.left_path_topic,
            10,
        )
        self.right_path_pub = self.create_publisher(
            Path,
            self.right_path_topic,
            10,
        )
        self.center_path_pub = self.create_publisher(
            Path,
            self.center_path_topic,
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

        self.frame_count = 0

        self.get_logger().info(
            f"Candidate input: {self.candidate_topic}"
        )
        self.get_logger().info(
            "Curve model: lateral = a*forward^2 + b*forward + c"
        )
        self.get_logger().info(
            "Fit ROI: "
            f"{self.fit_forward_min_m:.1f}-"
            f"{self.fit_forward_max_m:.1f} m forward, "
            f"{self.fit_right_extent_m:.1f}.."
            f"{self.fit_left_extent_m:.1f} m lateral"
        )

    def _validate_parameters(self) -> None:
        if self.resolution_m <= 0.0:
            raise ValueError("resolution_m must be positive")
        if self.forward_max_m <= self.forward_min_m:
            raise ValueError(
                "forward_max_m must exceed forward_min_m"
            )
        if self.left_extent_m <= self.right_extent_m:
            raise ValueError(
                "left_extent_m must exceed right_extent_m"
            )
        if not (
            self.forward_min_m
            <= self.fit_forward_min_m
            < self.fit_forward_max_m
            <= self.forward_max_m
        ):
            raise ValueError(
                "Fit forward range is outside the BEV"
            )
        if not (
            self.right_extent_m
            <= self.fit_right_extent_m
            < self.fit_left_extent_m
            <= self.left_extent_m
        ):
            raise ValueError(
                "Fit lateral range is outside the BEV"
            )
        if self.path_step_m <= 0.0:
            raise ValueError("path_step_m must be positive")
        if not (
            self.minimum_lane_width_m
            < self.maximum_lane_width_m
        ):
            raise ValueError("Invalid lane-width limits")

    def _callback(self, message: Image) -> None:
        try:
            mask = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding="mono8",
            )
        except Exception as exc:
            self.get_logger().error(
                f"Could not decode candidate mask: {exc}"
            )
            return

        if mask.shape[:2] != (self.height, self.width):
            self.get_logger().error(
                f"Candidate shape {mask.shape[:2]} does not match "
                f"expected {(self.height, self.width)}"
            )
            return

        binary = np.where(mask > 0, 255, 0).astype(
            np.uint8
        )
        points = self._extract_center_points(binary)

        rng = np.random.default_rng(
            self.random_seed + self.frame_count
        )
        models = self._fit_multiple_models(points, rng)
        left_model, right_model, pair_confidence = (
            self._select_ego_lane_pair(models)
        )

        left_path = self._build_path(
            left_model,
            message,
        )
        right_path = self._build_path(
            right_model,
            message,
        )
        center_path = self._build_center_path(
            left_model,
            right_model,
            message,
        )

        self.left_path_pub.publish(left_path)
        self.right_path_pub.publish(right_path)
        self.center_path_pub.publish(center_path)

        debug = self._build_debug(
            binary=binary,
            points=points,
            models=models,
            left_model=left_model,
            right_model=right_model,
        )
        self._publish_debug(debug, message)

        self.frame_count += 1
        self._publish_status(
            points=points,
            models=models,
            left_model=left_model,
            right_model=right_model,
            pair_confidence=pair_confidence,
        )

        if self.frame_count % self.log_every == 0:
            self.get_logger().info(
                f"frames={self.frame_count} "
                f"points={len(points)} models={len(models)} "
                f"left={'yes' if left_model else 'no'} "
                f"right={'yes' if right_model else 'no'} "
                f"pair_conf={pair_confidence:.2f}"
            )

    def _extract_center_points(
        self,
        binary: np.ndarray,
    ) -> np.ndarray:
        far_row, _ = self.metric_to_pixel(
            self.fit_forward_max_m,
            0.0,
        )
        near_row, _ = self.metric_to_pixel(
            self.fit_forward_min_m,
            0.0,
        )
        _, left_column = self.metric_to_pixel(
            self.fit_forward_min_m,
            self.fit_left_extent_m,
        )
        _, right_column = self.metric_to_pixel(
            self.fit_forward_min_m,
            self.fit_right_extent_m,
        )

        row_start, row_end = sorted(
            (
                max(0, far_row),
                min(self.height - 1, near_row),
            )
        )
        column_start, column_end = sorted(
            (
                max(0, left_column),
                min(self.width - 1, right_column),
            )
        )

        maximum_width_pixels = max(
            1,
            int(
                round(
                    self.maximum_marking_width_m
                    / self.resolution_m
                )
            ),
        )

        points: list[tuple[float, float]] = []

        for row in range(
            row_start,
            row_end + 1,
            self.row_stride_px,
        ):
            columns = np.flatnonzero(
                binary[
                    row,
                    column_start : column_end + 1,
                ]
                > 0
            )

            if columns.size == 0:
                continue

            columns = columns + column_start
            split_indices = np.flatnonzero(
                np.diff(columns) > 1
            ) + 1
            runs = np.split(columns, split_indices)

            for run in runs:
                if (
                    run.size == 0
                    or run.size > maximum_width_pixels
                ):
                    continue

                center_column = float(
                    np.mean(run)
                )
                forward_m, lateral_m = (
                    self.pixel_to_metric(
                        float(row),
                        center_column,
                    )
                )

                points.append(
                    (forward_m, lateral_m)
                )

        if not points:
            return np.empty((0, 2), dtype=np.float32)

        return np.asarray(
            points,
            dtype=np.float32,
        )

    def _fit_multiple_models(
        self,
        points: np.ndarray,
        rng: np.random.Generator,
    ) -> list[CurveModel]:
        if len(points) < self.ransac_min_points:
            return []

        remaining_indices = np.arange(
            len(points),
            dtype=np.int32,
        )
        models: list[CurveModel] = []

        for _ in range(self.max_models):
            if (
                len(remaining_indices)
                < self.ransac_min_points
            ):
                break

            local_points = points[remaining_indices]
            fitted = self._fit_one_model(
                local_points,
                rng,
            )

            if fitted is None:
                break

            global_inlier_indices = remaining_indices[
                fitted.inlier_indices
            ]

            model = CurveModel(
                coefficients=fitted.coefficients,
                inlier_indices=global_inlier_indices,
                inlier_count=fitted.inlier_count,
                forward_min=fitted.forward_min,
                forward_max=fitted.forward_max,
                rmse=fitted.rmse,
                score=fitted.score,
                confidence=fitted.confidence,
            )
            models.append(model)

            residuals = np.abs(
                local_points[:, 1]
                - model.lateral_at(
                    local_points[:, 0]
                )
            )
            keep = residuals > (
                self.residual_threshold_m * 1.25
            )
            remaining_indices = remaining_indices[
                keep
            ]

        return models

    def _fit_one_model(
        self,
        points: np.ndarray,
        rng: np.random.Generator,
    ) -> CurveModel | None:
        if len(points) < self.ransac_min_points:
            return None

        best_inliers: np.ndarray | None = None
        best_coefficients: np.ndarray | None = None
        best_score = -math.inf

        for _ in range(self.ransac_iterations):
            sample_indices = rng.choice(
                len(points),
                size=3,
                replace=False,
            )
            sample = points[sample_indices]

            if (
                np.ptp(sample[:, 0])
                < 0.55 * self.ransac_min_span_m
            ):
                continue

            try:
                coefficients = np.polyfit(
                    sample[:, 0],
                    sample[:, 1],
                    2,
                )
            except (
                np.linalg.LinAlgError,
                ValueError,
                TypeError,
            ):
                continue

            if not self._coefficients_plausible(
                coefficients
            ):
                continue

            residuals = np.abs(
                points[:, 1]
                - np.polyval(
                    coefficients,
                    points[:, 0],
                )
            )
            inliers = residuals <= (
                self.residual_threshold_m
            )

            inlier_count = int(
                np.count_nonzero(inliers)
            )
            if inlier_count < self.ransac_min_points:
                continue

            inlier_forward = points[inliers, 0]
            forward_span = float(
                np.ptp(inlier_forward)
            )
            if forward_span < self.ransac_min_span_m:
                continue

            rmse = float(
                np.sqrt(
                    np.mean(
                        np.square(
                            residuals[inliers]
                        )
                    )
                )
            )
            score = (
                float(inlier_count)
                + 5.0 * forward_span
                - 20.0 * rmse
            )

            if score > best_score:
                best_score = score
                best_inliers = inliers
                best_coefficients = coefficients

        if (
            best_inliers is None
            or best_coefficients is None
        ):
            return None

        try:
            refined_coefficients = np.polyfit(
                points[best_inliers, 0],
                points[best_inliers, 1],
                2,
            )
        except (
            np.linalg.LinAlgError,
            ValueError,
            TypeError,
        ):
            return None

        if not self._coefficients_plausible(
            refined_coefficients
        ):
            return None

        refined_residuals = np.abs(
            points[:, 1]
            - np.polyval(
                refined_coefficients,
                points[:, 0],
            )
        )
        refined_inliers = refined_residuals <= (
            self.residual_threshold_m
        )
        refined_indices = np.flatnonzero(
            refined_inliers
        )

        if len(refined_indices) < self.ransac_min_points:
            return None

        inlier_points = points[refined_indices]
        forward_min = float(
            np.min(inlier_points[:, 0])
        )
        forward_max = float(
            np.max(inlier_points[:, 0])
        )
        forward_span = forward_max - forward_min

        if forward_span < self.ransac_min_span_m:
            return None

        rmse = float(
            np.sqrt(
                np.mean(
                    np.square(
                        refined_residuals[
                            refined_inliers
                        ]
                    )
                )
            )
        )

        point_factor = min(
            1.0,
            len(refined_indices)
            / max(
                1.0,
                3.0 * self.ransac_min_points,
            ),
        )
        span_factor = min(
            1.0,
            forward_span
            / max(
                self.ransac_min_span_m,
                18.0,
            ),
        )
        residual_factor = math.exp(
            -rmse
            / max(
                self.residual_threshold_m,
                1e-6,
            )
        )
        confidence = float(
            point_factor
            * span_factor
            * residual_factor
        )

        score = (
            float(len(refined_indices))
            + 5.0 * forward_span
            - 20.0 * rmse
        )

        return CurveModel(
            coefficients=refined_coefficients,
            inlier_indices=refined_indices,
            inlier_count=len(refined_indices),
            forward_min=forward_min,
            forward_max=forward_max,
            rmse=rmse,
            score=score,
            confidence=confidence,
        )

    def _coefficients_plausible(
        self,
        coefficients: np.ndarray,
    ) -> bool:
        a, b, _ = coefficients

        if abs(float(a)) > self.max_abs_quadratic:
            return False
        if abs(float(b)) > self.max_abs_linear:
            return False

        sample_forward = np.array(
            [
                self.fit_forward_min_m,
                self.reference_forward_m,
                self.fit_forward_max_m,
            ],
            dtype=np.float32,
        )
        sample_lateral = np.polyval(
            coefficients,
            sample_forward,
        )

        return bool(
            np.all(
                sample_lateral
                >= self.fit_right_extent_m - 1.0
            )
            and np.all(
                sample_lateral
                <= self.fit_left_extent_m + 1.0
            )
        )

    def _select_ego_lane_pair(
        self,
        models: list[CurveModel],
    ) -> tuple[
        CurveModel | None,
        CurveModel | None,
        float,
    ]:
        left_candidates = [
            model
            for model in models
            if float(
                model.lateral_at(
                    self.reference_forward_m
                )
            )
            >= self.minimum_boundary_offset_m
        ]
        right_candidates = [
            model
            for model in models
            if float(
                model.lateral_at(
                    self.reference_forward_m
                )
            )
            <= -self.minimum_boundary_offset_m
        ]

        best_left: CurveModel | None = None
        best_right: CurveModel | None = None
        best_pair_score = -math.inf
        best_confidence = 0.0

        check_forward = np.array(
            [
                self.reference_forward_m,
                min(
                    15.0,
                    self.fit_forward_max_m,
                ),
                min(
                    25.0,
                    self.fit_forward_max_m,
                ),
            ],
            dtype=np.float32,
        )

        for left_model in left_candidates:
            for right_model in right_candidates:
                widths = (
                    left_model.lateral_at(check_forward)
                    - right_model.lateral_at(check_forward)
                )

                if np.any(
                    widths < self.minimum_lane_width_m
                ) or np.any(
                    widths > self.maximum_lane_width_m
                ):
                    continue

                mean_width = float(np.mean(widths))
                width_variation = float(np.ptp(widths))
                width_error = abs(
                    mean_width
                    - self.expected_lane_width_m
                )

                width_confidence = math.exp(
                    -width_error
                    / max(
                        self.pair_width_tolerance_m,
                        1e-6,
                    )
                )
                parallel_confidence = math.exp(
                    -width_variation / 0.8
                )

                near_left = float(
                    left_model.lateral_at(
                        self.reference_forward_m
                    )
                )
                near_right = float(
                    right_model.lateral_at(
                        self.reference_forward_m
                    )
                )
                center_offset = abs(
                    0.5 * (near_left + near_right)
                )
                center_confidence = math.exp(
                    -center_offset / 1.0
                )

                pair_confidence = float(
                    math.sqrt(
                        max(
                            left_model.confidence,
                            0.0,
                        )
                        * max(
                            right_model.confidence,
                            0.0,
                        )
                    )
                    * width_confidence
                    * parallel_confidence
                    * center_confidence
                )

                pair_score = (
                    pair_confidence
                    + 0.01
                    * (
                        left_model.score
                        + right_model.score
                    )
                )

                if pair_score > best_pair_score:
                    best_pair_score = pair_score
                    best_left = left_model
                    best_right = right_model
                    best_confidence = pair_confidence

        if best_left is not None:
            return (
                best_left,
                best_right,
                best_confidence,
            )

        # Graceful fallback for a single visible side.
        left_fallback = (
            min(
                left_candidates,
                key=lambda model: abs(
                    float(
                        model.lateral_at(
                            self.reference_forward_m
                        )
                    )
                ),
            )
            if left_candidates
            else None
        )
        right_fallback = (
            min(
                right_candidates,
                key=lambda model: abs(
                    float(
                        model.lateral_at(
                            self.reference_forward_m
                        )
                    )
                ),
            )
            if right_candidates
            else None
        )

        # Do not publish an implausible pair as an ego lane.
        if (
            left_fallback is not None
            and right_fallback is not None
        ):
            width = float(
                left_fallback.lateral_at(
                    self.reference_forward_m
                )
                - right_fallback.lateral_at(
                    self.reference_forward_m
                )
            )
            if not (
                self.minimum_lane_width_m
                <= width
                <= self.maximum_lane_width_m
            ):
                if (
                    left_fallback.confidence
                    >= right_fallback.confidence
                ):
                    right_fallback = None
                else:
                    left_fallback = None

        single_confidence = max(
            (
                model.confidence
                for model in (
                    left_fallback,
                    right_fallback,
                )
                if model is not None
            ),
            default=0.0,
        )

        return (
            left_fallback,
            right_fallback,
            0.35 * single_confidence,
        )

    def _build_path(
        self,
        model: CurveModel | None,
        source_message: Image,
    ) -> Path:
        path = Path()
        path.header = source_message.header

        if model is None:
            return path

        start = max(
            self.path_start_forward_m,
            model.forward_min,
        )
        end = min(
            self.path_end_forward_m,
            model.forward_max,
        )

        if end <= start:
            return path

        forward_values = np.arange(
            start,
            end + 0.5 * self.path_step_m,
            self.path_step_m,
            dtype=np.float32,
        )

        for forward_m in forward_values:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(forward_m)
            pose.pose.position.y = float(
                model.lateral_at(
                    float(forward_m)
                )
            )
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        return path

    def _build_center_path(
        self,
        left_model: CurveModel | None,
        right_model: CurveModel | None,
        source_message: Image,
    ) -> Path:
        path = Path()
        path.header = source_message.header

        if (
            left_model is None
            or right_model is None
        ):
            return path

        start = max(
            self.path_start_forward_m,
            left_model.forward_min,
            right_model.forward_min,
        )
        end = min(
            self.path_end_forward_m,
            left_model.forward_max,
            right_model.forward_max,
        )

        if end <= start:
            return path

        forward_values = np.arange(
            start,
            end + 0.5 * self.path_step_m,
            self.path_step_m,
            dtype=np.float32,
        )

        for forward_m in forward_values:
            left_lateral = float(
                left_model.lateral_at(
                    float(forward_m)
                )
            )
            right_lateral = float(
                right_model.lateral_at(
                    float(forward_m)
                )
            )

            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(forward_m)
            pose.pose.position.y = float(
                0.5 * (
                    left_lateral + right_lateral
                )
            )
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        return path

    def _build_debug(
        self,
        binary: np.ndarray,
        points: np.ndarray,
        models: list[CurveModel],
        left_model: CurveModel | None,
        right_model: CurveModel | None,
    ) -> np.ndarray:
        debug = np.zeros(
            (self.height, self.width, 3),
            dtype=np.uint8,
        )
        debug[binary > 0] = (70, 70, 70)

        for forward_m, lateral_m in points:
            row, column = self.metric_to_pixel(
                float(forward_m),
                float(lateral_m),
            )
            if (
                0 <= row < self.height
                and 0 <= column < self.width
            ):
                debug[row, column] = (
                    180,
                    180,
                    180,
                )

        for model in models:
            self._draw_model(
                debug,
                model,
                (220, 180, 0),
                1,
            )

        if left_model is not None:
            self._draw_model(
                debug,
                left_model,
                (0, 220, 0),
                3,
            )

        if right_model is not None:
            self._draw_model(
                debug,
                right_model,
                (0, 0, 230),
                3,
            )

        if (
            left_model is not None
            and right_model is not None
        ):
            self._draw_centerline(
                debug,
                left_model,
                right_model,
            )

        self._draw_grid(debug)

        cv2.putText(
            debug,
            "green: left boundary",
            (6, 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.40,
            (0, 220, 0),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            debug,
            "red: right boundary",
            (6, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.40,
            (0, 0, 230),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            debug,
            "yellow: centerline",
            (6, 48),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.40,
            (0, 220, 220),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            debug,
            "cyan: other fitted candidates",
            (6, 64),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.40,
            (220, 180, 0),
            1,
            cv2.LINE_AA,
        )

        return debug

    def _draw_model(
        self,
        image: np.ndarray,
        model: CurveModel,
        color: tuple[int, int, int],
        thickness: int,
    ) -> None:
        start = max(
            self.fit_forward_min_m,
            model.forward_min,
        )
        end = min(
            self.fit_forward_max_m,
            model.forward_max,
        )

        if end <= start:
            return

        forward_values = np.arange(
            start,
            end + 0.1,
            0.25,
            dtype=np.float32,
        )
        pixels: list[tuple[int, int]] = []

        for forward_m in forward_values:
            lateral_m = float(
                model.lateral_at(
                    float(forward_m)
                )
            )
            row, column = self.metric_to_pixel(
                float(forward_m),
                lateral_m,
            )

            if (
                0 <= row < self.height
                and 0 <= column < self.width
            ):
                pixels.append(
                    (column, row)
                )

        if len(pixels) >= 2:
            cv2.polylines(
                image,
                [
                    np.asarray(
                        pixels,
                        dtype=np.int32,
                    )
                ],
                isClosed=False,
                color=color,
                thickness=thickness,
                lineType=cv2.LINE_AA,
            )

    def _draw_centerline(
        self,
        image: np.ndarray,
        left_model: CurveModel,
        right_model: CurveModel,
    ) -> None:
        start = max(
            self.fit_forward_min_m,
            left_model.forward_min,
            right_model.forward_min,
        )
        end = min(
            self.fit_forward_max_m,
            left_model.forward_max,
            right_model.forward_max,
        )

        if end <= start:
            return

        pixels: list[tuple[int, int]] = []

        for forward_m in np.arange(
            start,
            end + 0.1,
            0.25,
            dtype=np.float32,
        ):
            lateral_m = 0.5 * (
                float(
                    left_model.lateral_at(
                        float(forward_m)
                    )
                )
                + float(
                    right_model.lateral_at(
                        float(forward_m)
                    )
                )
            )
            row, column = self.metric_to_pixel(
                float(forward_m),
                lateral_m,
            )

            if (
                0 <= row < self.height
                and 0 <= column < self.width
            ):
                pixels.append(
                    (column, row)
                )

        if len(pixels) >= 2:
            cv2.polylines(
                image,
                [
                    np.asarray(
                        pixels,
                        dtype=np.int32,
                    )
                ],
                isClosed=False,
                color=(0, 220, 220),
                thickness=2,
                lineType=cv2.LINE_AA,
            )

    def _publish_debug(
        self,
        debug: np.ndarray,
        source_message: Image,
    ) -> None:
        ok, encoded = cv2.imencode(
            ".jpg",
            debug,
            [
                int(cv2.IMWRITE_JPEG_QUALITY),
                self.jpeg_quality,
            ],
        )

        if not ok:
            return

        message = CompressedImage()
        message.header = source_message.header
        message.format = "jpeg"
        message.data = encoded.tobytes()
        self.debug_pub.publish(message)

    def _publish_status(
        self,
        points: np.ndarray,
        models: list[CurveModel],
        left_model: CurveModel | None,
        right_model: CurveModel | None,
        pair_confidence: float,
    ) -> None:
        message = String()
        message.data = json.dumps(
            {
                "frame": self.frame_count,
                "extracted_points": int(len(points)),
                "fitted_models": int(len(models)),
                "pair_confidence": float(
                    pair_confidence
                ),
                "left": self._model_to_dict(
                    left_model
                ),
                "right": self._model_to_dict(
                    right_model
                ),
                "lane_width_at_reference_m": (
                    float(
                        left_model.lateral_at(
                            self.reference_forward_m
                        )
                        - right_model.lateral_at(
                            self.reference_forward_m
                        )
                    )
                    if (
                        left_model is not None
                        and right_model is not None
                    )
                    else None
                ),
                "center_offset_at_reference_m": (
                    float(
                        0.5
                        * (
                            left_model.lateral_at(
                                self.reference_forward_m
                            )
                            + right_model.lateral_at(
                                self.reference_forward_m
                            )
                        )
                    )
                    if (
                        left_model is not None
                        and right_model is not None
                    )
                    else None
                ),
            },
            separators=(",", ":"),
        )
        self.status_pub.publish(message)

    def _model_to_dict(
        self,
        model: CurveModel | None,
    ) -> dict | None:
        if model is None:
            return None

        return {
            "coefficients": [
                float(value)
                for value in model.coefficients
            ],
            "inlier_count": int(
                model.inlier_count
            ),
            "forward_min_m": float(
                model.forward_min
            ),
            "forward_max_m": float(
                model.forward_max
            ),
            "rmse_m": float(model.rmse),
            "confidence": float(
                model.confidence
            ),
            "lateral_at_reference_m": float(
                model.lateral_at(
                    self.reference_forward_m
                )
            ),
        }

    def _draw_grid(
        self,
        image: np.ndarray,
    ) -> None:
        spacing = max(
            self.grid_spacing_m,
            self.resolution_m,
        )

        forward = (
            math.ceil(
                self.forward_min_m / spacing
            )
            * spacing
        )
        while forward <= self.forward_max_m:
            row, _ = self.metric_to_pixel(
                forward,
                0.0,
            )
            if 0 <= row < self.height:
                cv2.line(
                    image,
                    (0, row),
                    (self.width - 1, row),
                    (45, 45, 45),
                    1,
                )
            forward += spacing

        lateral = (
            math.ceil(
                self.right_extent_m / spacing
            )
            * spacing
        )
        while lateral <= self.left_extent_m:
            _, column = self.metric_to_pixel(
                self.forward_min_m,
                lateral,
            )
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

    def pixel_to_metric(
        self,
        row: float,
        column: float,
    ) -> tuple[float, float]:
        forward_m = (
            self.forward_max_m
            - (row + 0.5) * self.resolution_m
        )
        lateral_left_m = (
            self.left_extent_m
            - (column + 0.5) * self.resolution_m
        )
        return forward_m, lateral_left_m

    def metric_to_pixel(
        self,
        forward_m: float,
        lateral_left_m: float,
    ) -> tuple[int, int]:
        row = int(
            round(
                (
                    self.forward_max_m
                    - forward_m
                )
                / self.resolution_m
                - 0.5
            )
        )
        column = int(
            round(
                (
                    self.left_extent_m
                    - lateral_left_m
                )
                / self.resolution_m
                - 0.5
            )
        )
        return row, column


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LaneCurveFitNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
