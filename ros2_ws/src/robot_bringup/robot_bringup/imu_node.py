#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion


class ImuNode(Node):

    def __init__(self):
        super().__init__('imu_node')

        # ================= Parameters =================
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 50.0)       # Hz
        self.declare_parameter('start_delay', 0.5)         # sec

        # SAFE covariance (เริ่มสูงไว้ก่อน)
        self.declare_parameter('yaw_covariance', 0.2)
        self.declare_parameter('gyro_z_covariance', 0.05)

        self.frame_id = self.get_parameter('frame_id').value
        self.rate = float(self.get_parameter('publish_rate').value)
        self.start_delay = float(self.get_parameter('start_delay').value)

        self.yaw_cov = float(self.get_parameter('yaw_covariance').value)
        self.gyro_z_cov = float(self.get_parameter('gyro_z_covariance').value)

        # ================= Publisher =================
        self.pub_imu = self.create_publisher(Imu, '/imu/data', 10)

        # ================= State =================
        self.yaw = self.prev_yaw    # rad
        self.prev_yaw = 0.0
        self.gyro_z = 0.0         # rad/s

        # ================= Time =================
        self.start_time = self.get_clock().now()

        # ================= Timer =================
        self.timer = self.create_timer(1.0 / self.rate, self.publish_imu)

        self.get_logger().info(
            'IMU node started (sanitize + delay + safe covariance)'
        )

    # ==========================================================
    # Utilities
    # ==========================================================
    @staticmethod
    def yaw_to_quaternion(yaw: float) -> Quaternion:
        half = 0.5 * yaw
        return Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(half),
            w=math.cos(half),
        )

    @staticmethod
    def normalize_angle(a: float) -> float:
        return math.atan2(math.sin(a), math.cos(a))

    def sanitize_yaw(self, yaw: float) -> float:
        """
        กัน NaN / inf / jump
        """
        if not math.isfinite(yaw):
            return self.prev_yaw

        yaw = self.normalize_angle(yaw)

        # กัน jump ใหญ่ผิดปกติ (> 90 deg ต่อ frame)
        if abs(yaw - self.prev_yaw) > math.pi / 2.0:
            yaw = self.prev_yaw

        return yaw

    # ==========================================================
    # Main publish loop
    # ==========================================================
    def publish_imu(self):

        # ---------- Start delay (กัน EKF พังตอน init) ----------
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        if elapsed < self.start_delay:
            return

        # ---------- Sanitize yaw ----------
        self.yaw = self.sanitize_yaw(self.yaw)
        self.prev_yaw = self.yaw

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self.frame_id

        # ---------- Orientation (yaw only) ----------
        imu.orientation = self.yaw_to_quaternion(self.yaw)

        imu.orientation_covariance = [
            99999.0, 0.0,      0.0,
            0.0,     99999.0,  0.0,
            0.0,     0.0,      self.yaw_cov
        ]

        # ---------- Angular velocity ----------
        imu.angular_velocity.x = 0.0
        imu.angular_velocity.y = 0.0
        imu.angular_velocity.z = self.gyro_z

        imu.angular_velocity_covariance = [
            99999.0, 0.0,      0.0,
            0.0,     99999.0,  0.0,
            0.0,     0.0,      self.gyro_z_cov
        ]

        # ---------- Linear acceleration (unused) ----------
        imu.linear_acceleration_covariance = [
            -1.0, 0.0, 0.0,
             0.0,-1.0, 0.0,
             0.0, 0.0,-1.0
        ]

        self.pub_imu.publish(imu)

    def update_from_sensor(self, yaw_rad: float, gyro_z_rad_s: float):
        self.yaw = yaw_rad
        self.gyro_z = gyro_z_rad_s


def main():
    rclpy.init()
    node = ImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
