import math
import json
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String


class LaneComponentFilterNode(Node):
    """Split a metric BEV road-marking mask by local orientation."""

    def __init__(self) -> None:
        super().__init__("lane_component_filter_node")
        self.bridge = CvBridge()

        defaults = {
            "bev_mask_topic": "/perception/lane/bev_mask",
            "longitudinal_topic": "/perception/lane/longitudinal_mask",
            "transverse_topic": "/perception/lane/transverse_mask",
            "debug_topic": "/perception/lane/components_debug/compressed",
            "status_topic": "/perception/lane/components_status",
            # Must match lane_geometry_node.
            "forward_min_m": 0.0,
            "forward_max_m": 40.0,
            "left_extent_m": 12.0,
            "right_extent_m": -12.0,
            "resolution_m": 0.10,
            # Reliable processing region.
            "process_forward_min_m": 5.0,
            "process_forward_max_m": 30.0,
            "process_left_extent_m": 10.0,
            "process_right_extent_m": -10.0,
            # Oriented morphology.
            "orientation_line_length_m": 1.2,
            "orientation_line_width_px": 1,
            "support_dilate_px": 3,
            "longitudinal_angles_deg": [-30.0, -20.0, -10.0, 0.0, 10.0, 20.0, 30.0],
            "transverse_angles_deg": [60.0, 70.0, 80.0, 90.0, 100.0, 110.0, 120.0],
            "min_input_component_area_px": 8,
            "min_output_component_area_px": 6,
            "min_longitudinal_span_m": 0.8,
            "min_transverse_span_m": 0.8,
            "grid_spacing_m": 5.0,
            "jpeg_quality": 80,
            "log_every": 30,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        p = lambda name: self.get_parameter(name).value
        self.bev_topic = str(p("bev_mask_topic"))
        self.long_topic = str(p("longitudinal_topic"))
        self.trans_topic = str(p("transverse_topic"))
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

        self.min_in_area = int(p("min_input_component_area_px"))
        self.min_out_area = int(p("min_output_component_area_px"))
        self.min_long_span = float(p("min_longitudinal_span_m"))
        self.min_trans_span = float(p("min_transverse_span_m"))
        self.grid_spacing = float(p("grid_spacing_m"))
        self.jpeg_quality = int(p("jpeg_quality"))
        self.log_every = max(1, int(p("log_every")))

        self._validate()
        self.height = int(math.ceil((self.fmax - self.fmin) / self.res))
        self.width = int(math.ceil((self.left - self.right) / self.res))

        length = max(3, int(round(float(p("orientation_line_length_m")) / self.res)))
        length += 1 - length % 2
        line_width = max(1, int(p("orientation_line_width_px")))
        self.long_kernels = [
            self._line_kernel(length, float(a), line_width)
            for a in p("longitudinal_angles_deg")
        ]
        self.trans_kernels = [
            self._line_kernel(length, float(a), line_width)
            for a in p("transverse_angles_deg")
        ]

        support = max(1, int(p("support_dilate_px")))
        support += 1 - support % 2
        self.support_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (support, support)
        )

        self.sub = self.create_subscription(
            Image, self.bev_topic, self._callback, qos_profile_sensor_data
        )
        self.long_pub = self.create_publisher(
            Image, self.long_topic, qos_profile_sensor_data
        )
        self.trans_pub = self.create_publisher(
            Image, self.trans_topic, qos_profile_sensor_data
        )
        self.debug_pub = self.create_publisher(
            CompressedImage, self.debug_topic, qos_profile_sensor_data
        )
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.frames = 0

        self.get_logger().info(f"Input: {self.bev_topic}")
        self.get_logger().info(f"Longitudinal: {self.long_topic}")
        self.get_logger().info(f"Transverse: {self.trans_topic}")
        self.get_logger().info(
            f"ROI: {self.pfmin:.1f}-{self.pfmax:.1f} m forward, "
            f"{self.pright:.1f}..{self.pleft:.1f} m lateral"
        )

    def _validate(self) -> None:
        if self.res <= 0 or self.fmax <= self.fmin or self.left <= self.right:
            raise ValueError("Invalid BEV geometry")
        if not self.fmin <= self.pfmin < self.pfmax <= self.fmax:
            raise ValueError("Processing forward range is outside the BEV")
        if not self.right <= self.pright < self.pleft <= self.left:
            raise ValueError("Processing lateral range is outside the BEV")

    @staticmethod
    def _line_kernel(size: int, angle_from_vertical: float, thickness: int) -> np.ndarray:
        kernel = np.zeros((size, size), np.uint8)
        c = size // 2
        theta = math.radians(angle_from_vertical)
        dx, dy = math.sin(theta) * c, -math.cos(theta) * c
        a = (int(round(c - dx)), int(round(c - dy)))
        b = (int(round(c + dx)), int(round(c + dy)))
        cv2.line(kernel, a, b, 1, thickness, cv2.LINE_8)
        return kernel

    def _callback(self, msg: Image) -> None:
        try:
            bev = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        except Exception as exc:
            self.get_logger().error(f"Cannot decode BEV mask: {exc}")
            return

        if bev.shape[:2] != (self.height, self.width):
            self.get_logger().error(
                f"BEV shape {bev.shape[:2]} != expected {(self.height, self.width)}"
            )
            return

        binary = np.where(bev > 0, 255, 0).astype(np.uint8)
        binary = cv2.bitwise_and(binary, self._roi(binary.shape))
        clean = self._filter_components(binary, self.min_in_area, 1, 1)

        long_score = self._orientation_score(clean, self.long_kernels)
        trans_score = self._orientation_score(clean, self.trans_kernels)
        fg = clean > 0

        long_choice = fg & (long_score > 0) & (
            (trans_score == 0) | (long_score > trans_score)
        )
        trans_choice = fg & (trans_score > 0) & (
            (long_score == 0) | (trans_score > long_score)
        )

        long_mask = long_choice.astype(np.uint8) * 255
        trans_mask = trans_choice.astype(np.uint8) * 255

        long_mask = self._filter_components(
            long_mask,
            self.min_out_area,
            max(1, int(round(self.min_long_span / self.res))),
            1,
        )
        trans_mask = self._filter_components(
            trans_mask,
            self.min_out_area,
            1,
            max(1, int(round(self.min_trans_span / self.res))),
        )
        misc = (
            fg & ~((long_mask > 0) | (trans_mask > 0))
        ).astype(np.uint8) * 255

        self._publish_mask(long_mask, self.long_pub, msg)
        self._publish_mask(trans_mask, self.trans_pub, msg)
        self._publish_debug(self._debug(clean, long_mask, trans_mask, misc), msg)

        self.frames += 1
        self._publish_status(clean, long_mask, trans_mask, misc)
        if self.frames % self.log_every == 0:
            total = max(1, np.count_nonzero(clean))
            self.get_logger().info(
                f"frames={self.frames} foreground={total} "
                f"long={100*np.count_nonzero(long_mask)/total:.1f}% "
                f"trans={100*np.count_nonzero(trans_mask)/total:.1f}%"
            )

    def _roi(self, shape: tuple[int, int]) -> np.ndarray:
        r0, _ = self.metric_to_pixel(self.pfmax, 0.0)
        r1, _ = self.metric_to_pixel(self.pfmin, 0.0)
        _, c0 = self.metric_to_pixel(self.pfmin, self.pleft)
        _, c1 = self.metric_to_pixel(self.pfmin, self.pright)
        rows = sorted((max(0, r0), min(shape[0] - 1, r1)))
        cols = sorted((max(0, c0), min(shape[1] - 1, c1)))
        roi = np.zeros(shape, np.uint8)
        roi[rows[0] : rows[1] + 1, cols[0] : cols[1] + 1] = 255
        return roi

    def _orientation_score(
        self, binary: np.ndarray, kernels: list[np.ndarray]
    ) -> np.ndarray:
        score = np.zeros(binary.shape, np.uint16)
        for kernel in kernels:
            opened = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
            support = cv2.dilate(opened, self.support_kernel)
            support = cv2.bitwise_and(support, binary)
            score += (support > 0).astype(np.uint16)
        return score

    @staticmethod
    def _filter_components(
        binary: np.ndarray, min_area: int, min_height: int, min_width: int
    ) -> np.ndarray:
        count, labels, stats, _ = cv2.connectedComponentsWithStats(
            binary, connectivity=8
        )
        out = np.zeros_like(binary)
        for label in range(1, count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            height = int(stats[label, cv2.CC_STAT_HEIGHT])
            width = int(stats[label, cv2.CC_STAT_WIDTH])
            if area >= min_area and height >= min_height and width >= min_width:
                out[labels == label] = 255
        return out

    def _publish_mask(self, mask: np.ndarray, publisher, source: Image) -> None:
        msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
        msg.header = source.header
        publisher.publish(msg)

    def _publish_debug(self, image: np.ndarray, source: Image) -> None:
        ok, encoded = cv2.imencode(
            ".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        )
        if ok:
            msg = CompressedImage()
            msg.header = source.header
            msg.format = "jpeg"
            msg.data = encoded.tobytes()
            self.debug_pub.publish(msg)

    def _publish_status(
        self, fg: np.ndarray, long_mask: np.ndarray,
        trans_mask: np.ndarray, misc: np.ndarray
    ) -> None:
        total = int(np.count_nonzero(fg))
        denom = max(1, total)
        lp = int(np.count_nonzero(long_mask))
        tp = int(np.count_nonzero(trans_mask))
        mp = int(np.count_nonzero(misc))
        msg = String()
        msg.data = json.dumps(
            {
                "frame": self.frames,
                "foreground_pixels": total,
                "longitudinal_pixels": lp,
                "transverse_pixels": tp,
                "miscellaneous_pixels": mp,
                "longitudinal_fraction": lp / denom,
                "transverse_fraction": tp / denom,
                "miscellaneous_fraction": mp / denom,
            },
            separators=(",", ":"),
        )
        self.status_pub.publish(msg)

    def _debug(
        self, fg: np.ndarray, long_mask: np.ndarray,
        trans_mask: np.ndarray, misc: np.ndarray
    ) -> np.ndarray:
        image = np.zeros((*fg.shape, 3), np.uint8)
        image[fg > 0] = (70, 70, 70)
        image[misc > 0] = (0, 200, 200)       # yellow
        image[long_mask > 0] = (0, 220, 0)    # green
        image[trans_mask > 0] = (0, 0, 230)   # red
        self._draw_grid(image)
        cv2.putText(image, "green: longitudinal", (6, 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 220, 0), 1, cv2.LINE_AA)
        cv2.putText(image, "red: transverse", (6, 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 230), 1, cv2.LINE_AA)
        cv2.putText(image, "yellow: miscellaneous", (6, 48),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 200), 1, cv2.LINE_AA)
        return image

    def _draw_grid(self, image: np.ndarray) -> None:
        spacing = max(self.grid_spacing, self.res)
        forward = math.ceil(self.fmin / spacing) * spacing
        while forward <= self.fmax:
            row, _ = self.metric_to_pixel(forward, 0.0)
            if 0 <= row < image.shape[0]:
                cv2.line(image, (0, row), (image.shape[1] - 1, row), (45, 45, 45), 1)
            forward += spacing

        lateral = math.ceil(self.right / spacing) * spacing
        while lateral <= self.left:
            _, col = self.metric_to_pixel(self.fmin, lateral)
            if 0 <= col < image.shape[1]:
                color = (0, 150, 255) if abs(lateral) < 1e-6 else (45, 45, 45)
                cv2.line(image, (col, 0), (col, image.shape[0] - 1), color, 1)
            lateral += spacing

    def metric_to_pixel(
        self, forward_m: float, lateral_left_m: float
    ) -> tuple[int, int]:
        row = int(round((self.fmax - forward_m) / self.res - 0.5))
        col = int(round((self.left - lateral_left_m) / self.res - 0.5))
        return row, col


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LaneComponentFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
