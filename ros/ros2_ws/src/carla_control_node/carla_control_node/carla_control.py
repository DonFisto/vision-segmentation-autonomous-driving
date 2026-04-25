#!/usr/bin/env python3
import sys
import termios
import tty
import select
import time
import argparse

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

try:
    import carla
except ImportError:
    carla = None


def get_key_nonblocking(timeout_sec: float = 0.0) -> str:
    dr, _, _ = select.select([sys.stdin], [], [], timeout_sec)
    if dr:
        return sys.stdin.read(1)
    return ""


class CarlaTeleop(Node):
    def __init__(self, topic, hz, speed_step, steer_step, max_speed, max_steer,
                 decay, carla_host, carla_port, tm_port):
        super().__init__("carla_teleop")

        self.pub = self.create_publisher(Twist, topic, 10)
        self.topic = topic

        self.hz = hz
        self.dt = 1.0 / max(hz, 1e-6)

        self.speed_step = speed_step
        self.steer_step = steer_step
        self.max_speed = max_speed
        self.max_steer = max_steer

        self.speed = 0.0
        self.steer = 0.0
        self.decay = decay

        self.carla_host = carla_host
        self.carla_port = carla_port
        self.tm_port = tm_port
        self.autopilot_enabled = False

        self.get_logger().info(f"Publishing Twist to {self.topic} at {self.hz} Hz")
        self.get_logger().info("Keys: W/S speed | A/D steer | SPACE stop | P autopilot toggle | ESC quit")

        self.timer = self.create_timer(self.dt, self._tick)

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

    def _find_hero(self):
        if carla is None:
            self.get_logger().error("CARLA Python API not available in this environment.")
            return None, None

        try:
            client = carla.Client(self.carla_host, self.carla_port)
            client.set_timeout(5.0)
            world = client.get_world()

            for v in world.get_actors().filter("vehicle.*"):
                if v.attributes.get("role_name", "") == "hero":
                    return client, v

            self.get_logger().warn("No hero vehicle found.")
            return client, None

        except Exception as e:
            self.get_logger().error(f"Failed to connect to CARLA: {e}")
            return None, None

    def _toggle_autopilot(self):
        client, hero = self._find_hero()
        if hero is None:
            return

        self.autopilot_enabled = not self.autopilot_enabled

        try:
            tm = client.get_trafficmanager(self.tm_port)
            hero.set_autopilot(self.autopilot_enabled, self.tm_port)

            if self.autopilot_enabled:
                self.speed = 0.0
                self.steer = 0.0

            state = "ON" if self.autopilot_enabled else "OFF"
            self.get_logger().info(f"Hero autopilot {state} on vehicle id={hero.id}")

        except Exception as e:
            self.get_logger().error(f"Failed to toggle autopilot: {e}")

    def _publish(self):
        if self.autopilot_enabled:
            return

        msg = Twist()
        msg.linear.x = float(self._clamp(self.speed / max(self.max_speed, 1e-9), -1.0, 1.0))
        msg.angular.z = float(self._clamp(self.steer / max(self.max_steer, 1e-9), -1.0, 1.0))
        self.pub.publish(msg)

    def _status_line(self):
        mode = "AUTO" if self.autopilot_enabled else "MANUAL"
        return (
            f"mode={mode} | "
            f"speed={self.speed:+.2f} (max {self.max_speed:.2f}) | "
            f"steer={self.steer:+.2f} (max {self.max_steer:.2f})"
        )

    def _tick(self):
        key = get_key_nonblocking(0.0)

        if key == "\x1b":
            raise KeyboardInterrupt

        if key in ("p", "P"):
            self._toggle_autopilot()

        if not self.autopilot_enabled:
            if key in ("w", "W"):
                self.speed = self._clamp(self.speed + self.speed_step, -self.max_speed, self.max_speed)
            elif key in ("s", "S"):
                self.speed = self._clamp(self.speed - self.speed_step, -self.max_speed, self.max_speed)
            elif key in ("x", "X"):
                self.speed = 0.0
            elif key == " ":
                self.speed = 0.0
                self.steer = 0.0

            # Fixed steering:
            # A = left / positive angular.z
            # D = right / negative angular.z
            elif key in ("a", "A"):
                self.steer = self._clamp(self.steer - self.steer_step, -self.max_steer, self.max_steer)
            elif key in ("d", "D"):
                self.steer = self._clamp(self.steer + self.steer_step, -self.max_steer, self.max_steer)

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

            if self.decay > 0.0:
                self.steer *= (1.0 - self._clamp(self.decay, 0.0, 1.0))

        self._publish()

        now = time.time()
        if now - self._last_print > 0.2:
            sys.stdout.write("\r" + self._status_line() + " " * 10)
            sys.stdout.flush()
            self._last_print = now


def main():
    parser = argparse.ArgumentParser(description="Terminal teleop publishing Twist to /carla/cmd_vel")
    parser.add_argument("--topic", default="/carla/cmd_vel")
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--speed-step", type=float, default=0.1)
    parser.add_argument("--steer-step", type=float, default=0.08)
    parser.add_argument("--max-speed", type=float, default=1.5)
    parser.add_argument("--max-steer", type=float, default=1.0)
    parser.add_argument("--decay", type=float, default=0.12)

    parser.add_argument("--carla-host", default="localhost")
    parser.add_argument("--carla-port", type=int, default=2000)
    parser.add_argument("--tm-port", type=int, default=8000)

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
        carla_host=args.carla_host,
        carla_port=args.carla_port,
        tm_port=args.tm_port,
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
