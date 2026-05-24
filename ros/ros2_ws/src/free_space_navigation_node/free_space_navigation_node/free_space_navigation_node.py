#!/usr/bin/env python3
import json
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist


class FreeSpaceNavigationNode(Node):
    """
    Refined navigation node based on /perception/free_space_status.

    This node is intended as a cleaner alternative to the primitive reactive
    navigation node. It does not inspect raw depth directly. Instead, it consumes
    the intermediate free-space representation produced by free_space_node.

    Input:
      /perception/free_space_status

    Output:
      /carla/cmd_vel
    """

    def __init__(self):
        super().__init__("free_space_navigation_node")

        self.declare_parameter("status_topic", "/perception/free_space_status")
        self.declare_parameter("cmd_topic", "/carla/cmd_vel")

        # Normalized speed commands expected by the current CARLA bridge.
        self.declare_parameter("cruise_speed", 0.22)
        self.declare_parameter("slow_speed", 0.10)
        self.declare_parameter("reverse_speed", -0.18)

        # Steering parameters.
        # In the current bridge convention, previous testing suggested:
        #   negative angular.z = left
        #   positive angular.z = right
        self.declare_parameter("steer_value", 0.45)
        self.declare_parameter("left_steer_sign", -1.0)
        self.declare_parameter("right_steer_sign", 1.0)

        # Recovery maneuver.
        self.declare_parameter("reverse_duration_sec", 1.0)
        self.declare_parameter("recovery_turn_duration_sec", 0.8)

        # Decision thresholds.
        self.declare_parameter("center_obstacle_stop", 0.35)
        self.declare_parameter("center_free_cruise", 0.55)
        self.declare_parameter("center_free_recovery", 0.20)
        self.declare_parameter("side_free_margin", 0.08)

        # Safety behavior.
        self.declare_parameter("timeout_sec", 1.0)
        self.declare_parameter("stop_on_timeout", True)

        self.status_topic = self.get_parameter("status_topic").value
        self.cmd_topic = self.get_parameter("cmd_topic").value

        self.cruise_speed = float(self.get_parameter("cruise_speed").value)
        self.slow_speed = float(self.get_parameter("slow_speed").value)
        self.reverse_speed = float(self.get_parameter("reverse_speed").value)

        self.steer_value = float(self.get_parameter("steer_value").value)
        self.left_steer_sign = float(self.get_parameter("left_steer_sign").value)
        self.right_steer_sign = float(self.get_parameter("right_steer_sign").value)

        self.reverse_duration_sec = float(self.get_parameter("reverse_duration_sec").value)
        self.recovery_turn_duration_sec = float(self.get_parameter("recovery_turn_duration_sec").value)

        self.center_obstacle_stop = float(self.get_parameter("center_obstacle_stop").value)
        self.center_free_cruise = float(self.get_parameter("center_free_cruise").value)
        self.center_free_recovery = float(self.get_parameter("center_free_recovery").value)
        self.side_free_margin = float(self.get_parameter("side_free_margin").value)

        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.stop_on_timeout = bool(self.get_parameter("stop_on_timeout").value)

        self.last_status_time = 0.0

        self.mode = "NORMAL"
        self.mode_until = 0.0
        self.recovery_until = 0.0
        self.recovery_direction = "left"

        self.sub = self.create_subscription(
            String,
            self.status_topic,
            self.status_cb,
            10,
        )

        self.pub = self.create_publisher(
            Twist,
            self.cmd_topic,
            10,
        )

        self.timer = self.create_timer(0.1, self.timer_cb)

        self.get_logger().info(f"Refined navigation listening on {self.status_topic}")
        self.get_logger().info(f"Publishing commands to {self.cmd_topic}")
        self.get_logger().info("Do not run primitive reactive_navigation_node at the same time.")

    def clamp(self, value, lo, hi):
        return max(lo, min(hi, value))

    def steer_for_direction(self, direction):
        if direction == "left":
            return self.left_steer_sign * self.steer_value
        if direction == "right":
            return self.right_steer_sign * self.steer_value
        return 0.0

    def publish_cmd(self, speed, steer, reason=""):
        cmd = Twist()
        cmd.linear.x = float(self.clamp(speed, -1.0, 1.0))
        cmd.angular.z = float(self.clamp(steer, -1.0, 1.0))
        self.pub.publish(cmd)

        if reason:
            self.get_logger().info(
                f"{reason} | speed={cmd.linear.x:+.2f}, steer={cmd.angular.z:+.2f}",
                throttle_duration_sec=0.5,
            )

    def parse_status(self, msg):
        try:
            return json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"Could not parse free-space status JSON: {e}")
            return None

    def choose_better_side(self, free_ratio):
        left = float(free_ratio.get("left", 0.0))
        right = float(free_ratio.get("right", 0.0))

        if left > right + self.side_free_margin:
            return "left"
        if right > left + self.side_free_margin:
            return "right"

        # If both are similar, keep previous recovery direction to avoid oscillation.
        return self.recovery_direction

    def enter_recovery(self, direction, reason):
        now = time.time()
        self.mode = "REVERSING"
        self.mode_until = now + self.reverse_duration_sec
        self.recovery_until = self.mode_until + self.recovery_turn_duration_sec
        self.recovery_direction = direction

        self.get_logger().warn(f"ENTER RECOVERY: {reason}, direction={direction}")

    def timer_cb(self):
        now = time.time()

        if self.mode == "REVERSING":
            if now < self.mode_until:
                # When reversing, flip steering direction compared with forward turning.
                steer = -self.steer_for_direction(self.recovery_direction)
                self.publish_cmd(
                    self.reverse_speed,
                    steer,
                    reason=f"REVERSING_{self.recovery_direction.upper()}",
                )
                return
            self.mode = "RECOVERY_TURN"

        if self.mode == "RECOVERY_TURN":
            if now < self.recovery_until:
                steer = self.steer_for_direction(self.recovery_direction)
                self.publish_cmd(
                    self.slow_speed,
                    steer,
                    reason=f"RECOVERY_FORWARD_{self.recovery_direction.upper()}",
                )
                return
            self.mode = "NORMAL"

        if self.last_status_time > 0.0:
            elapsed = now - self.last_status_time
            if elapsed > self.timeout_sec and self.stop_on_timeout:
                self.publish_cmd(0.0, 0.0, reason=f"TIMEOUT no free-space status for {elapsed:.1f}s")

    def status_cb(self, msg):
        self.last_status_time = time.time()

        if self.mode in ("REVERSING", "RECOVERY_TURN"):
            return

        status = self.parse_status(msg)
        if status is None:
            return

        free_ratio = status.get("free_ratio", {})
        obstacle_ratio = status.get("obstacle_ratio", {})
        recommended = status.get("recommended_direction", "slow")

        left_free = float(free_ratio.get("left", 0.0))
        center_free = float(free_ratio.get("center", 0.0))
        right_free = float(free_ratio.get("right", 0.0))

        center_obstacle = float(obstacle_ratio.get("center", 0.0))

        # Safety override 1: if center obstacle ratio is too high, recover.
        if center_obstacle >= self.center_obstacle_stop:
            direction = self.choose_better_side(free_ratio)
            self.enter_recovery(
                direction,
                reason=f"center obstacle ratio={center_obstacle:.2f}",
            )
            return

        # Safety override 2: if center free space is too low, recover.
        # This catches boxed-in situations where the center is mostly unknown
        # or only slightly free, even if explicit obstacle ratio is moderate.
        if center_free <= self.center_free_recovery:
            direction = self.choose_better_side(free_ratio)
            self.enter_recovery(
                direction,
                reason=f"center free too low={center_free:.2f}",
            )
            return

        # Use free_space_node recommendation when available.
        if recommended == "forward":
            if center_free >= self.center_free_cruise:
                self.publish_cmd(
                    self.cruise_speed,
                    0.0,
                    reason=f"FORWARD center_free={center_free:.2f}",
                )
            else:
                self.publish_cmd(
                    self.slow_speed,
                    0.0,
                    reason=f"SLOW_FORWARD center_free={center_free:.2f}",
                )
            return

        if recommended == "left":
            steer = self.steer_for_direction("left")
            self.publish_cmd(
                self.slow_speed,
                steer,
                reason=f"STEER_LEFT free L/C/R={left_free:.2f}/{center_free:.2f}/{right_free:.2f}",
            )
            return

        if recommended == "right":
            steer = self.steer_for_direction("right")
            self.publish_cmd(
                self.slow_speed,
                steer,
                reason=f"STEER_RIGHT free L/C/R={left_free:.2f}/{center_free:.2f}/{right_free:.2f}",
            )
            return

        if recommended == "stop":
            direction = self.choose_better_side(free_ratio)
            self.enter_recovery(
                direction,
                reason="free_space_status recommended stop",
            )
            return

        # Default: cautious slow forward.
        self.publish_cmd(
            self.slow_speed,
            0.0,
            reason=f"DEFAULT_SLOW recommended={recommended}",
        )


def main(args=None):
    rclpy.init(args=args)
    node = FreeSpaceNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
