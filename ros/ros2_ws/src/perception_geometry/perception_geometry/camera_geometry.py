from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Optional

import cv2
import numpy as np


@dataclass(frozen=True)
class IPMConfig:
    """
    Image trapezoid used for inverse perspective mapping.

    Coordinates are normalized image ratios.

    Source point order:
      bottom-left, bottom-right, top-right, top-left
    """

    src_bottom_y_ratio: float = 0.97
    src_top_y_ratio: float = 0.48

    src_bottom_left_x_ratio: float = 0.10
    src_bottom_right_x_ratio: float = 0.90

    src_top_left_x_ratio: float = 0.42
    src_top_right_x_ratio: float = 0.58


@dataclass(frozen=True)
class BEVSpec:
    """
    Bird's-eye-view grid specification.

    Metric convention:
      forward_m: positive in front of the vehicle
      lateral_m: negative left, positive right

    BEV image convention:
      row 0: far forward
      last row: most backward
      center column: ego lateral center
    """

    forward_m: float = 45.0
    backward_m: float = 0.0
    width_m: float = 16.0
    resolution_m: float = 0.10

    @property
    def height_m(self) -> float:
        return self.forward_m + self.backward_m

    @property
    def width_px(self) -> int:
        return max(2, int(round(self.width_m / self.resolution_m)))

    @property
    def height_px(self) -> int:
        return max(2, int(round(self.height_m / self.resolution_m)))


class CameraIPM:
    """
    Shared inverse-perspective-mapping helper.

    This class centralizes the projection logic that was previously duplicated
    across lane detection, lane mapping, occupancy, and free-space modules.
    """

    def __init__(self, ipm_config: IPMConfig, bev_spec: BEVSpec) -> None:
        self.ipm_config = ipm_config
        self.bev_spec = bev_spec

    def as_dict(self) -> dict[str, Any]:
        return {
            "ipm_config": asdict(self.ipm_config),
            "bev_spec": asdict(self.bev_spec),
            "bev_width_px": self.bev_spec.width_px,
            "bev_height_px": self.bev_spec.height_px,
        }

    def image_source_quad(self, image_width: int, image_height: int) -> np.ndarray:
        c = self.ipm_config

        return np.float32(
            [
                [
                    c.src_bottom_left_x_ratio * image_width,
                    c.src_bottom_y_ratio * image_height,
                ],
                [
                    c.src_bottom_right_x_ratio * image_width,
                    c.src_bottom_y_ratio * image_height,
                ],
                [
                    c.src_top_right_x_ratio * image_width,
                    c.src_top_y_ratio * image_height,
                ],
                [
                    c.src_top_left_x_ratio * image_width,
                    c.src_top_y_ratio * image_height,
                ],
            ]
        )

    def bev_destination_quad(self) -> np.ndarray:
        """
        Destination quad corresponding to image_source_quad order:
          bottom-left, bottom-right, top-right, top-left
        """

        bottom_left = self.metric_to_bev_pixel(0.0, -0.5 * self.bev_spec.width_m)
        bottom_right = self.metric_to_bev_pixel(0.0, 0.5 * self.bev_spec.width_m)
        top_right = self.metric_to_bev_pixel(self.bev_spec.forward_m, 0.5 * self.bev_spec.width_m)
        top_left = self.metric_to_bev_pixel(self.bev_spec.forward_m, -0.5 * self.bev_spec.width_m)

        return np.float32(
            [
                [bottom_left[0], bottom_left[1]],
                [bottom_right[0], bottom_right[1]],
                [top_right[0], top_right[1]],
                [top_left[0], top_left[1]],
            ]
        )

    def homographies(self, image_width: int, image_height: int) -> tuple[np.ndarray, np.ndarray]:
        src = self.image_source_quad(image_width, image_height)
        dst = self.bev_destination_quad()

        image_to_bev = cv2.getPerspectiveTransform(src, dst)
        bev_to_image = cv2.getPerspectiveTransform(dst, src)

        return image_to_bev, bev_to_image

    def warp_image_to_bev(
        self,
        image: np.ndarray,
        image_to_bev: np.ndarray,
        interpolation: int = cv2.INTER_NEAREST,
        border_value: int | tuple[int, int, int] = 0,
    ) -> np.ndarray:
        return cv2.warpPerspective(
            image,
            image_to_bev,
            (self.bev_spec.width_px, self.bev_spec.height_px),
            flags=interpolation,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=border_value,
        )

    def metric_to_bev_pixel(self, forward_m: float, lateral_m: float) -> tuple[float, float]:
        """
        Metric local vehicle coordinates to BEV pixel coordinates.

        Returns:
          (u_px, v_px)
        """

        u = (lateral_m + 0.5 * self.bev_spec.width_m) / self.bev_spec.resolution_m
        v = (self.bev_spec.forward_m - forward_m) / self.bev_spec.resolution_m

        return float(u), float(v)

    def bev_pixel_to_metric(self, u_px: float, v_px: float) -> tuple[float, float]:
        """
        BEV pixel coordinates to metric local vehicle coordinates.

        Returns:
          (forward_m, lateral_m)
        """

        lateral_m = u_px * self.bev_spec.resolution_m - 0.5 * self.bev_spec.width_m
        forward_m = self.bev_spec.forward_m - v_px * self.bev_spec.resolution_m

        return float(forward_m), float(lateral_m)

    def image_point_to_bev_pixel(
        self,
        x_px: float,
        y_px: float,
        image_to_bev: np.ndarray,
    ) -> tuple[float, float]:
        point = np.float32([[[x_px, y_px]]])
        out = cv2.perspectiveTransform(point, image_to_bev)[0, 0]
        return float(out[0]), float(out[1])

    def bev_pixel_to_image_point(
        self,
        u_px: float,
        v_px: float,
        bev_to_image: np.ndarray,
    ) -> tuple[float, float]:
        point = np.float32([[[u_px, v_px]]])
        out = cv2.perspectiveTransform(point, bev_to_image)[0, 0]
        return float(out[0]), float(out[1])

    def image_point_to_metric(
        self,
        x_px: float,
        y_px: float,
        image_to_bev: np.ndarray,
    ) -> tuple[float, float]:
        u_px, v_px = self.image_point_to_bev_pixel(x_px, y_px, image_to_bev)
        return self.bev_pixel_to_metric(u_px, v_px)

    def metric_to_image_point(
        self,
        forward_m: float,
        lateral_m: float,
        bev_to_image: np.ndarray,
    ) -> tuple[float, float]:
        u_px, v_px = self.metric_to_bev_pixel(forward_m, lateral_m)
        return self.bev_pixel_to_image_point(u_px, v_px, bev_to_image)

    def point_in_bev_bounds(self, u_px: float, v_px: float, margin_px: float = 0.0) -> bool:
        return (
            -margin_px <= u_px < self.bev_spec.width_px + margin_px
            and -margin_px <= v_px < self.bev_spec.height_px + margin_px
        )

    def metric_in_bounds(
        self,
        forward_m: float,
        lateral_m: float,
        margin_m: float = 0.0,
    ) -> bool:
        return (
            -self.bev_spec.backward_m - margin_m <= forward_m <= self.bev_spec.forward_m + margin_m
            and -0.5 * self.bev_spec.width_m - margin_m <= lateral_m <= 0.5 * self.bev_spec.width_m + margin_m
        )

    def image_line_to_bev_line(
        self,
        line: dict[str, float],
        image_to_bev: np.ndarray,
    ) -> dict[str, float]:
        """
        Converts an image-space endpoint line to a BEV-pixel endpoint line.

        Expected input keys:
          x_bottom, y_bottom, x_top, y_top
        """

        u_bottom, v_bottom = self.image_point_to_bev_pixel(
            float(line["x_bottom"]),
            float(line["y_bottom"]),
            image_to_bev,
        )
        u_top, v_top = self.image_point_to_bev_pixel(
            float(line["x_top"]),
            float(line["y_top"]),
            image_to_bev,
        )

        return {
            "x_bottom": float(u_bottom),
            "y_bottom": float(v_bottom),
            "x_top": float(u_top),
            "y_top": float(v_top),
        }

    def bev_line_to_image_line(
        self,
        line: dict[str, float],
        bev_to_image: np.ndarray,
    ) -> dict[str, float]:
        """
        Converts a BEV-pixel endpoint line to an image-space endpoint line.

        Expected input keys:
          x_bottom, y_bottom, x_top, y_top
        """

        x_bottom, y_bottom = self.bev_pixel_to_image_point(
            float(line["x_bottom"]),
            float(line["y_bottom"]),
            bev_to_image,
        )
        x_top, y_top = self.bev_pixel_to_image_point(
            float(line["x_top"]),
            float(line["y_top"]),
            bev_to_image,
        )

        return {
            "x_bottom": float(x_bottom),
            "y_bottom": float(y_bottom),
            "x_top": float(x_top),
            "y_top": float(y_top),
        }

    def bev_line_to_metric_line(self, line: dict[str, float]) -> dict[str, float]:
        """
        Converts a BEV-pixel endpoint line to a metric endpoint line.

        Expected input keys:
          x_bottom, y_bottom, x_top, y_top
        """

        forward_bottom_m, lateral_bottom_m = self.bev_pixel_to_metric(
            float(line["x_bottom"]),
            float(line["y_bottom"]),
        )
        forward_top_m, lateral_top_m = self.bev_pixel_to_metric(
            float(line["x_top"]),
            float(line["y_top"]),
        )

        return {
            "forward_bottom_m": float(forward_bottom_m),
            "lateral_bottom_m": float(lateral_bottom_m),
            "forward_top_m": float(forward_top_m),
            "lateral_top_m": float(lateral_top_m),
        }

    def metric_line_to_bev_line(self, line: dict[str, float]) -> dict[str, float]:
        """
        Converts a metric endpoint line to a BEV-pixel endpoint line.

        Expected input keys:
          forward_bottom_m, lateral_bottom_m, forward_top_m, lateral_top_m
        """

        x_bottom, y_bottom = self.metric_to_bev_pixel(
            float(line["forward_bottom_m"]),
            float(line["lateral_bottom_m"]),
        )
        x_top, y_top = self.metric_to_bev_pixel(
            float(line["forward_top_m"]),
            float(line["lateral_top_m"]),
        )

        return {
            "x_bottom": float(x_bottom),
            "y_bottom": float(y_bottom),
            "x_top": float(x_top),
            "y_top": float(y_top),
        }

    def image_line_to_metric_line(
        self,
        line: dict[str, float],
        image_to_bev: np.ndarray,
    ) -> dict[str, float]:
        bev_line = self.image_line_to_bev_line(line, image_to_bev)
        return self.bev_line_to_metric_line(bev_line)

    @staticmethod
    def center_line_from_pair(
        left_line: dict[str, float],
        right_line: dict[str, float],
    ) -> dict[str, float]:
        """
        Computes the midpoint line between two endpoint lines.

        Works for both BEV-pixel lines and image-pixel lines if the keys are:
          x_bottom, y_bottom, x_top, y_top
        """

        return {
            "x_bottom": 0.5 * (float(left_line["x_bottom"]) + float(right_line["x_bottom"])),
            "y_bottom": 0.5 * (float(left_line["y_bottom"]) + float(right_line["y_bottom"])),
            "x_top": 0.5 * (float(left_line["x_top"]) + float(right_line["x_top"])),
            "y_top": 0.5 * (float(left_line["y_top"]) + float(right_line["y_top"])),
        }


def declare_ipm_parameters(node: Any, prefix: str = "") -> None:
    """
    Declares a consistent set of IPM/BEV parameters on a ROS node.

    Prefix example:
      prefix="lane_"
      -> lane_src_bottom_y_ratio, lane_bev_forward_m, etc.

    Keep prefix empty for current lane-detection-compatible parameter names.
    """

    def name(base: str) -> str:
        return f"{prefix}{base}"

    node.declare_parameter(name("src_bottom_y_ratio"), IPMConfig.src_bottom_y_ratio)
    node.declare_parameter(name("src_top_y_ratio"), IPMConfig.src_top_y_ratio)
    node.declare_parameter(name("src_bottom_left_x_ratio"), IPMConfig.src_bottom_left_x_ratio)
    node.declare_parameter(name("src_bottom_right_x_ratio"), IPMConfig.src_bottom_right_x_ratio)
    node.declare_parameter(name("src_top_left_x_ratio"), IPMConfig.src_top_left_x_ratio)
    node.declare_parameter(name("src_top_right_x_ratio"), IPMConfig.src_top_right_x_ratio)

    node.declare_parameter(name("bev_forward_m"), BEVSpec.forward_m)
    node.declare_parameter(name("bev_backward_m"), BEVSpec.backward_m)
    node.declare_parameter(name("bev_width_m"), BEVSpec.width_m)
    node.declare_parameter(name("bev_resolution"), BEVSpec.resolution_m)


def ipm_from_ros_parameters(node: Any, prefix: str = "") -> CameraIPM:
    """
    Builds CameraIPM from parameters already declared on a ROS node.
    """

    def value(base: str) -> Any:
        return node.get_parameter(f"{prefix}{base}").value

    config = IPMConfig(
        src_bottom_y_ratio=float(value("src_bottom_y_ratio")),
        src_top_y_ratio=float(value("src_top_y_ratio")),
        src_bottom_left_x_ratio=float(value("src_bottom_left_x_ratio")),
        src_bottom_right_x_ratio=float(value("src_bottom_right_x_ratio")),
        src_top_left_x_ratio=float(value("src_top_left_x_ratio")),
        src_top_right_x_ratio=float(value("src_top_right_x_ratio")),
    )

    bev_spec = BEVSpec(
        forward_m=float(value("bev_forward_m")),
        backward_m=float(value("bev_backward_m")),
        width_m=float(value("bev_width_m")),
        resolution_m=float(value("bev_resolution")),
    )

    return CameraIPM(config, bev_spec)
