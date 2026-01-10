#!/usr/bin/env python3
"""
Test publisher: robot เดินเป็นสี่เหลี่ยม วนใน RViz2
- Publish wheel_ticks
- Publish IMU (yaw + gyro_z)
เหมาะสำหรับ EKF + RViz2
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion


class SquareMotionPublisher(Node):

    MOVE_FORWARD = 0
    TURN_LEFT = 1

    def __init__(self):
        super().__init__('square_motion_publisher')

        # ================= Publishers =================
        self.wheel_pub = self.create_publisher(
            Float32MultiArray,
            '/wheel_ticks',
            10
        )

        self.imu_pub = self.create_publisher(
            Imu,
            '/imu/data',
            10
        )

        # ================= Parameters =================
        self.dt = 0.1               # 10 Hz
        self.linear_vel = 10.0      # cm/s
        self.angular_vel = 0.6      # rad/s (หมุนไวหน่อย จะได้ 90° ชัด)

        self.side_length = 50.0     # cm ต่อ 1 ด้าน
        self.turn_angle = math.pi / 2.0  # 90 deg

        # ================= State =================
        self.state = self.MOVE_FORWARD
        self.current_side = 0

        self.distance_acc = 0.0
        self.angle_acc = 0.0

        self.wheel_position = 0.0
        self.yaw = 0.0
        self.gyro_z = 0.0

        # ================= Timer =================
        self.timer = self.create_timer(self.dt, self.update)

        self.get_logger().info('Square motion test started')

    # ==================================================
    def update(self):

        linear = 0.0
        angular = 0.0

        # ================= STATE MACHINE =================
        if self.state == self.MOVE_FORWARD:
            linear = self.linear_vel
            angular = 0.0

            self.distance_acc += linear * self.dt

            if self.distance_acc >= self.side_length:
                self.distance_acc = 0.0
                self.state = self.TURN_LEFT
                self.get_logger().info(
                    f'Side {self.current_side + 1} done → TURN'
                )

        elif self.state == self.TURN_LEFT:
            linear = 0.0
            angular = self.angular_vel

            self.angle_acc += angular * self.dt

            if self.angle_acc >= self.turn_angle:
                self.angle_acc = 0.0
                self.state = self.MOVE_FORWARD
                self.current_side = (self.current_side + 1) % 4

                if self.current_side == 0:
                    self.get_logger().info('Completed one square loop')

        # ================= UPDATE STATE =================
        self.wheel_position += linear * self.dt
        self.yaw += angular * self.dt
        self.gyro_z = angular

        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # ================= WHEEL TICKS =================
        wheel_msg = Float32MultiArray()

        wheel_sep = 0.365  # m (36.5 cm)

        left = (linear - angular * wheel_sep * 50.0) * self.dt
        right = (linear + angular * wheel_sep * 50.0) * self.dt

        left_pos = self.wheel_position - left
        right_pos = self.wheel_position + right

        wheel_msg.data = [
            left_pos,
            right_pos,
            left_pos,
            right_pos
        ]

        self.wheel_pub.publish(wheel_msg)

        # ================= IMU =================
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        half_yaw = 0.5 * self.yaw
        imu_msg.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(half_yaw),
            w=math.cos(half_yaw)
        )

        imu_msg.angular_velocity.z = self.gyro_z

        imu_msg.orientation_covariance = [
            99999.0, 0.0, 0.0,
            0.0, 99999.0, 0.0,
            0.0, 0.0, 0.05
        ]

        imu_msg.angular_velocity_covariance = [
            99999.0, 0.0, 0.0,
            0.0, 99999.0, 0.0,
            0.0, 0.0, 0.02
        ]

        imu_msg.linear_acceleration_covariance = [
            -1.0, 0.0, 0.0,
            0.0, -1.0, 0.0,
            0.0, 0.0, -1.0
        ]

        self.imu_pub.publish(imu_msg)

    # ==================================================


def main():
    rclpy.init()
    node = SquareMotionPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
