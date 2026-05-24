#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import numpy as np
import cv2
import carla
import math


class CarlaRgbPublisher(Node):
    def __init__(self):
        super().__init__('carla_rgb_publisher')

        # Parameters
        self.declare_parameter('host', 'localhost')
        self.declare_parameter('port', 2000)
        self.declare_parameter('width', 800)
        self.declare_parameter('height', 600)
        self.declare_parameter('fps', 15)

        # Compressed image params
        self.declare_parameter('publish_raw', True)          # set True if you also want /image_raw
        self.declare_parameter('jpeg_quality', 60)            # 40–80 typical

        # Control params
        self.declare_parameter('max_throttle', 0.6)  # clamp for safety
        self.declare_parameter('max_brake', 1.0)
        self.declare_parameter('max_steer', 0.7)

        host = self.get_parameter('host').value
        port = int(self.get_parameter('port').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        fps = int(self.get_parameter('fps').value)

        self.publish_raw = bool(self.get_parameter('publish_raw').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)

        self.max_throttle = float(self.get_parameter('max_throttle').value)
        self.max_brake = float(self.get_parameter('max_brake').value)
        self.max_steer = float(self.get_parameter('max_steer').value)

        # Publishers
        self.status_pub = self.create_publisher(String, 'carla_status', 10)

        self.hero_odom_pub = self.create_publisher(
            Odometry,
            '/carla/hero_odom',
            10,
        )

        # Compressed camera topic (Foxglove-friendly)
        self.img_comp_pub = self.create_publisher(
            CompressedImage, '/carla/rgb/image_raw/compressed', 10
        )

        # Optional raw topic (big bandwidth)
        self.img_pub = None
        if self.publish_raw:
            self.img_pub = self.create_publisher(Image, '/carla/rgb/image_raw', 10)

        # Subscriber for terminal control (publish Twist)
        self.cmd_sub = self.create_subscription(Twist, '/carla/cmd_vel', self._on_cmd_vel, 10)

        # Connect to CARLA
        self.client = carla.Client(host, port)
        self.client.set_timeout(5.0)
        self.world = self.client.get_world()

        # Spawn vehicle
        bp_lib = self.world.get_blueprint_library()
        vehicle_bps = bp_lib.filter('vehicle.*model3*')
        if not vehicle_bps:
            vehicle_bps = bp_lib.filter('vehicle.*')

        spawn_points = self.world.get_map().get_spawn_points()
        if not spawn_points:
            raise RuntimeError("No spawn points in this CARLA map.")

        vehicle_bps[0].set_attribute("role_name", "hero")

        self.vehicle = self.world.try_spawn_actor(vehicle_bps[0], spawn_points[0])
        if self.vehicle is None:
            raise RuntimeError("Failed to spawn vehicle (spawn point occupied). Try a different spawn point.")

        # Attach RGB camera
        cam_bp = bp_lib.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', str(self.width))
        cam_bp.set_attribute('image_size_y', str(self.height))
        cam_bp.set_attribute('fov', '90')
        cam_bp.set_attribute('sensor_tick', str(1.0 / max(fps, 1)))

        cam_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        self.camera = self.world.spawn_actor(cam_bp, cam_transform, attach_to=self.vehicle)
        self.camera.listen(self._on_image)

        # Status timer
        self.timer = self.create_timer(1.0, self._status_tick)

        self.get_logger().info("Publishing /carla/rgb/image_raw/compressed (sensor_msgs/CompressedImage)")
        if self.publish_raw:
            self.get_logger().info("Also publishing /carla/rgb/image_raw (sensor_msgs/Image) [HIGH BANDWIDTH]")
        self.get_logger().info("Listening for control on /carla/cmd_vel (geometry_msgs/Twist)")
        self.get_logger().info("Controls: linear.x>0 forward, linear.x<0 reverse, angular.z left/right")

    def _carla_rotation_to_quaternion(self, rotation):
        roll = math.radians(rotation.roll)
        pitch = math.radians(rotation.pitch)
        yaw = math.radians(rotation.yaw)

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        return {
            "w": cr * cp * cy + sr * sp * sy,
            "x": sr * cp * cy - cr * sp * sy,
            "y": cr * sp * cy + sr * cp * sy,
            "z": cr * cp * sy - sr * sp * cy,
        }

    def publish_hero_odom(self):
        """Publish CARLA hero pose and velocity as nav_msgs/Odometry."""
        if self.vehicle is None:
            return

        try:
            transform = self.vehicle.get_transform()
            velocity = self.vehicle.get_velocity()
            angular_velocity = self.vehicle.get_angular_velocity()
        except Exception as e:
            self.get_logger().warn(f"Could not read hero odometry from CARLA: {e}")
            return

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "carla_world"
        odom.child_frame_id = "hero"

        odom.pose.pose.position.x = float(transform.location.x)
        odom.pose.pose.position.y = float(transform.location.y)
        odom.pose.pose.position.z = float(transform.location.z)

        q = self._carla_rotation_to_quaternion(transform.rotation)
        odom.pose.pose.orientation.x = float(q["x"])
        odom.pose.pose.orientation.y = float(q["y"])
        odom.pose.pose.orientation.z = float(q["z"])
        odom.pose.pose.orientation.w = float(q["w"])

        odom.twist.twist.linear.x = float(velocity.x)
        odom.twist.twist.linear.y = float(velocity.y)
        odom.twist.twist.linear.z = float(velocity.z)

        odom.twist.twist.angular.x = math.radians(float(angular_velocity.x))
        odom.twist.twist.angular.y = math.radians(float(angular_velocity.y))
        odom.twist.twist.angular.z = math.radians(float(angular_velocity.z))

        self.hero_odom_pub.publish(odom)

    def _status_tick(self):
        self.publish_hero_odom()
        msg = String()
        msg.data = (
            f"Connected: {self.world.get_map().name} | "
            f"vehicle={self.vehicle.id} | camera={self.camera.id}"
        )
        self.status_pub.publish(msg)

    def _on_image(self, image: carla.Image):
        # CARLA provides BGRA uint8 buffer
        arr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))
        bgr = arr[:, :, :3]  # BGR

        stamp = self.get_clock().now().to_msg()

        # Publish compressed JPEG
        ok, jpg = cv2.imencode('.jpg', bgr, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if ok:
            msg = CompressedImage()
            msg.header.stamp = stamp
            msg.header.frame_id = "carla_camera"
            msg.format = "jpeg"
            msg.data = jpg.tobytes()
            self.img_comp_pub.publish(msg)

        # Optional raw publish
        if self.img_pub is not None:
            ros_img = Image()
            ros_img.header.stamp = stamp
            ros_img.header.frame_id = "carla_camera"
            ros_img.height = image.height
            ros_img.width = image.width
            ros_img.encoding = "bgr8"
            ros_img.is_bigendian = False
            ros_img.step = image.width * 3
            ros_img.data = bgr.tobytes()
            self.img_pub.publish(ros_img)

    def _on_cmd_vel(self, msg: Twist):
        x = float(msg.linear.x)     # [-1, 1] expected
        z = float(msg.angular.z)    # [-1, 1] expected

        steer = float(np.clip(z, -1.0, 1.0)) * self.max_steer

        if x > 0.0:
            throttle = float(np.clip(x, 0.0, 1.0)) * self.max_throttle
            brake = 0.0
            reverse = False
        elif x < 0.0:
            throttle = float(np.clip(-x, 0.0, 1.0)) * self.max_throttle
            brake = 0.0
            reverse = True
        else:
            throttle = 0.0
            brake = 1.0
            reverse = False

        control = carla.VehicleControl(
            throttle=throttle,
            steer=steer,
            brake=brake,
            hand_brake=False,
            reverse=reverse
        )
        self.vehicle.apply_control(control)

    def destroy_node(self):
        try:
            if hasattr(self, "camera") and self.camera is not None:
                self.camera.stop()
                self.camera.destroy()
            if hasattr(self, "vehicle") and self.vehicle is not None:
                self.vehicle.destroy()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CarlaRgbPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
