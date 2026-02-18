#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist

import numpy as np
import carla


class CarlaRgbPublisher(Node):
    def __init__(self):
        super().__init__('carla_rgb_publisher')

        # Parameters
        self.declare_parameter('host', 'localhost')
        self.declare_parameter('port', 2000)
        self.declare_parameter('width', 800)
        self.declare_parameter('height', 600)
        self.declare_parameter('fps', 15)

        # Control params
        self.declare_parameter('max_throttle', 0.6)  # clamp for safety
        self.declare_parameter('max_brake', 1.0)
        self.declare_parameter('max_steer', 0.7)

        host = self.get_parameter('host').value
        port = int(self.get_parameter('port').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        fps = int(self.get_parameter('fps').value)

        self.max_throttle = float(self.get_parameter('max_throttle').value)
        self.max_brake = float(self.get_parameter('max_brake').value)
        self.max_steer = float(self.get_parameter('max_steer').value)

        # Publishers
        self.status_pub = self.create_publisher(String, 'carla_status', 10)
        self.img_pub = self.create_publisher(Image, '/carla/rgb/image_raw', 10)

        # Subscriber for terminal control (publish Twist)
        # /carla/cmd_vel: linear.x in [-1,1] (forward/back), angular.z in [-1,1] (steer left/right)
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

        self.get_logger().info("Publishing /carla/rgb/image_raw")
        self.get_logger().info("Listening for control on /carla/cmd_vel (geometry_msgs/Twist)")
        self.get_logger().info(
            "Controls: linear.x>0 forward, linear.x<0 reverse, angular.z left/right"
        )

    def _status_tick(self):
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

        ros_img = Image()
        ros_img.header.stamp = self.get_clock().now().to_msg()
        ros_img.header.frame_id = "carla_camera"
        ros_img.height = image.height
        ros_img.width = image.width
        ros_img.encoding = "bgr8"
        ros_img.is_bigendian = False
        ros_img.step = image.width * 3
        ros_img.data = bgr.tobytes()

        self.img_pub.publish(ros_img)

    def _on_cmd_vel(self, msg: Twist):
        # Map Twist -> CARLA VehicleControl
        x = float(msg.linear.x)     # [-1, 1] expected
        z = float(msg.angular.z)    # [-1, 1] expected

        steer = float(np.clip(z, -1.0, 1.0)) * self.max_steer

        # Forward / reverse support:
        #  x > 0 -> forward throttle
        #  x < 0 -> reverse throttle (reverse gear True)
        #  x == 0 -> full stop brake
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

