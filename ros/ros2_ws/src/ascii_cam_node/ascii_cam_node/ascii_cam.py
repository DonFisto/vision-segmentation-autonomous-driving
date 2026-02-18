#!/usr/bin/env python3
"""
ROS2 ASCII camera viewer (terminal "video") for CARLA.

Hardcoded topic: /carla/rgb/image_raw
"""

import argparse
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2

ASCII_RAMP = " .:-=+*#%@"

def frame_to_ascii(gray: np.ndarray, out_w: int, out_h: int) -> str:
    small = cv2.resize(gray, (out_w, out_h), interpolation=cv2.INTER_AREA)
    ramp = ASCII_RAMP
    idx = (small.astype(np.float32) / 255.0) * (len(ramp) - 1)
    idx = idx.astype(np.int32)
    lines = ["".join(ramp[i] for i in row) for row in idx]
    return "\n".join(lines)

class AsciiCam(Node):
    def __init__(self, width: int, fps: float, invert: bool, show_stats: bool):
        super().__init__("ascii_cam")

        self.bridge = CvBridge()

        # Hardcoded topic:
        self.image_topic = "/carla/rgb/image_raw"

        self.out_w = max(20, int(width))
        self.fps = max(1.0, float(fps))
        self.period = 1.0 / self.fps
        self.invert = bool(invert)
        self.show_stats = bool(show_stats)

        # Terminal character cells are taller than wide; compensate height.
        self.aspect_correction = 0.55

        self.last_render_t = 0.0
        self.frames = 0
        self.t0 = time.time()

        self.sub = self.create_subscription(
            Image,
            self.image_topic,
            self.cb,
            10
        )

        # Clear screen once, hide cursor
        sys.stdout.write("\x1b[2J\x1b[?25l")
        sys.stdout.flush()

        self.get_logger().info(
            f"ASCII viewer subscribed to {self.image_topic} | width={self.out_w} | fps={self.fps}"
        )

    def destroy_node(self):
        try:
            sys.stdout.write("\x1b[?25h\n")  # show cursor
            sys.stdout.flush()
        except Exception:
            pass
        super().destroy_node()

    def cb(self, msg: Image):
        now = time.time()
        if now - self.last_render_t < self.period:
            return
        self.last_render_t = now

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        if self.invert:
            gray = 255 - gray

        h, w = gray.shape[:2]
        out_h = int((h / w) * self.out_w * self.aspect_correction)
        out_h = max(10, out_h)

        ascii_img = frame_to_ascii(gray, self.out_w, out_h)

        sys.stdout.write("\x1b[H")  # cursor home
        sys.stdout.write(ascii_img)

        if self.show_stats:
            self.frames += 1
            elapsed = max(1e-6, time.time() - self.t0)
            fps_est = self.frames / elapsed
            sys.stdout.write(
                f"\n\n{self.image_topic} | {w}x{h} -> {self.out_w}x{out_h} | render_fps≈{fps_est:.1f} | invert={self.invert}"
            )
        sys.stdout.flush()

def main():
    parser = argparse.ArgumentParser(description="ROS2 ASCII camera viewer (hardcoded CARLA topic)")
    parser.add_argument("--width", type=int, default=120, help="ASCII output width (chars)")
    parser.add_argument("--fps", type=float, default=15.0, help="Render FPS cap")
    parser.add_argument("--invert", action="store_true", help="Invert grayscale")
    parser.add_argument("--show-stats", action="store_true", help="Show footer stats")
    args = parser.parse_args()

    rclpy.init()
    node = AsciiCam(width=args.width, fps=args.fps, invert=args.invert, show_stats=args.show_stats)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
