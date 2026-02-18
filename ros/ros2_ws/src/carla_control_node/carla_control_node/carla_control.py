#!/usr/bin/env python3
"""
Terminal teleop for CARLA (no GUI required).

This node DOES NOT connect to CARLA and DOES NOT spawn anything.
It only publishes geometry_msgs/Twist to /carla/cmd_vel, which your bridge node
already subscribes to and applies to the spawned vehicle.

Keys:
  W/S : increase/decrease forward speed
  A/D : steer left/right (hold to keep steering)
  X   : brake/stop (sets speed to 0)
  SPACE: quick stop (speed=0, steer=0)
  Q/E : decrease/increase max speed
  Z/C : decrease/increase steer max
  ESC or CTRL+C: quit

Notes:
- Reverse is just negative speed (press S until speed < 0).
- This is "rate-based": you change a target speed and it publishes continuously,
  so it feels responsive compared to one-shot commands.
"""

import sys
import termios
import tty
import select
import time
import argparse

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def get_key_nonblocking(timeout_sec: float = 0.0) -> str:
    """Read one key from stdin if available, else return ''."""
    dr, _, _ = select.select([sys.stdin], [], [], timeout_sec)
    if dr:
        return sys.stdin.read(1)
    return ""


class CarlaTeleop(Node):
    def __init__(self, topic: str, hz: float,
                 speed_step: float, steer_step: float,
                 max_speed: float, max_steer: float,
                 decay: float):
        super().__init__("carla_teleop")

        self.pub = self.create_publisher(Twist, topic, 10)
        self.topic = topic

        self.hz = hz
        self.dt = 1.0 / max(hz, 1e-6)

        self.speed_step = speed_step
        self.steer_step = steer_step

        self.max_speed = max_speed
        self.max_steer = max_steer

        # target commands
        self.speed = 0.0   # maps to Twist.linear.x in [-max_speed, +max_speed]
        self.steer = 0.0   # maps to Twist.angular.z in [-max_steer, +max_steer]

        # optional decay for steering to auto-center (0..1): 0 = no decay, 1 = immediate center
        self.decay = decay

        self.get_logger().info(f"Publishing Twist to {self.topic} at {self.hz} Hz")
        self.get_logger().info("Make sure your bridge node is running (it applies /carla/cmd_vel).")

        self.timer = self.create_timer(self.dt, self._tick)

        # terminal state
        self._old_term = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        self._last_print = time.time()

    def destroy_node(self):
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_term)
        except Exception:
            pass
        super().destroy_node()

    def _clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    def _publish(self):
        msg = Twist()
        # normalize to [-1,1] so bridge can clamp with max_throttle/max_steer safely
        msg.linear.x = float(self._clamp(self.speed / max(self.max_speed, 1e-9), -1.0, 1.0))
        msg.angular.z = float(self._clamp(self.steer / max(self.max_steer, 1e-9), -1.0, 1.0))
        self.pub.publish(msg)

    def _status_line(self):
        return (f"speed={self.speed:+.2f} (max {self.max_speed:.2f}) | "
                f"steer={self.steer:+.2f} (max {self.max_steer:.2f})")

    def _tick(self):
        key = get_key_nonblocking(0.0)

        # ESC
        if key == "\x1b":
            raise KeyboardInterrupt

        # Controls
        if key in ("w", "W"):
            self.speed = self._clamp(self.speed + self.speed_step, -self.max_speed, self.max_speed)
        elif key in ("s", "S"):
            self.speed = self._clamp(self.speed - self.speed_step, -self.max_speed, self.max_speed)
        elif key in ("x", "X"):
            self.speed = 0.0
        elif key == " ":
            self.speed = 0.0
            self.steer = 0.0
        elif key in ("a", "A"):
            self.steer = self._clamp(self.steer + self.steer_step, -self.max_steer, self.max_steer)
        elif key in ("d", "D"):
            self.steer = self._clamp(self.steer - self.steer_step, -self.max_steer, self.max_steer)

        # Adjust limits live
        elif key in ("q", "Q"):
            self.max_speed = max(0.1, self.max_speed - 0.1)
            self.speed = self._clamp(self.speed, -self.max_speed, self.max_speed)
        elif key in ("e", "E"):
            self.max_speed = min(5.0, self.max_speed + 0.1)
        elif key in ("z", "Z"):
            self.max_steer = max(0.1, self.max_steer - 0.05)
            self.steer = self._clamp(self.steer, -self.max_steer, self.max_steer)
        elif key in ("c", "C"):
            self.max_steer = min(2.0, self.max_steer + 0.05)

        # Optional steering decay (auto-center)
        if self.decay > 0.0:
            self.steer *= (1.0 - self._clamp(self.decay, 0.0, 1.0))

        self._publish()

        # print status ~5Hz
        now = time.time()
        if now - self._last_print > 0.2:
            # \r keeps it on one line
            sys.stdout.write("\r" + self._status_line() + " " * 10)
            sys.stdout.flush()
            self._last_print = now


def main():
    parser = argparse.ArgumentParser(description="Terminal teleop publishing Twist to /carla/cmd_vel")
    parser.add_argument("--topic", default="/carla/cmd_vel", help="Twist topic to publish")
    parser.add_argument("--hz", type=float, default=20.0, help="Publish rate (Hz)")
    parser.add_argument("--speed-step", type=float, default=0.1, help="Speed step per keypress")
    parser.add_argument("--steer-step", type=float, default=0.08, help="Steer step per keypress")
    parser.add_argument("--max-speed", type=float, default=1.5, help="Max target speed (m/s-like units)")
    parser.add_argument("--max-steer", type=float, default=1.0, help="Max target steer (unitless)")
    parser.add_argument("--decay", type=float, default=0.12, help="Steer auto-center decay per tick (0..1)")
    args = parser.parse_args()

    rclpy.init()
    node = CarlaTeleop(
        topic=args.topic,
        hz=args.hz,
        speed_step=args.speed_step,
        steer_step=args.steer_step,
        max_speed=args.max_speed,
        max_steer=args.max_steer,
        decay=args.decay,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("\nExited teleop.")


if __name__ == "__main__":
    main()
