#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from scipy.spatial.transform import Rotation as R


class WheelOdometryNode(Node):

    def __init__(self):
        super().__init__('wheel_odometry_node')

        # ================= Parameters =================
        self.declare_parameter('wheel_separation', 0.365)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')

        self.wheel_sep = float(self.get_parameter('wheel_separation').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value

        if self.wheel_sep <= 0.0:
            self.get_logger().fatal('wheel_separation must be > 0')
            raise RuntimeError('Invalid wheel_separation')

        # ================= State =================
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.prev_left = None
        self.prev_right = None

        # ================= ROS I/O =================
        self.sub_ticks = self.create_subscription(
            Float32MultiArray,
            '/wheel_ticks',
            self.on_ticks,
            10
        )

        self.pub_odom = self.create_publisher(
            Odometry,
            '/wheel/odometry',
            10
        )

        self.get_logger().info('wheel_odometry_node started')

    # ================= Utilities =================
    @staticmethod
    def normalize_angle(a: float) -> float:
        return math.atan2(math.sin(a), math.cos(a))

    @staticmethod
    def yaw_to_quaternion(yaw: float) -> Quaternion:
        q = R.from_euler('z', yaw).as_quat()  # [x, y, z, w]
        return Quaternion(
            x=float(q[0]),
            y=float(q[1]),
            z=float(q[2]),
            w=float(q[3]),
        )

    # ================= Callback =================
    def on_ticks(self, msg: Float32MultiArray):

        # Expect: [fl, fr, rl, rr]
        if len(msg.data) < 4:
            self.get_logger().warn('wheel_ticks size < 4')
            return

        # convert cm -> m (ตามโค้ดเดิมคุณ)
        fl, fr, rl, rr = [float(v) * 0.01 for v in msg.data]

        left = 0.5 * (fl + rl)
        right = 0.5 * (fr + rr)

        # First message → just init
        if self.prev_left is None:
            self.prev_left = left
            self.prev_right = right
            return

        d_left = left - self.prev_left
        d_right = right - self.prev_right
        self.prev_left = left
        self.prev_right = right

        # ================= Odometry =================
        ds = 0.5 * (d_left + d_right)
        dtheta = (d_right - d_left) / self.wheel_sep

        self.yaw = self.normalize_angle(self.yaw + dtheta)
        self.x += ds * math.cos(self.yaw)
        self.y += ds * math.sin(self.yaw)

        # ================= Publish Odom =================
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation = self.yaw_to_quaternion(self.yaw)

        # ================= Covariance =================
        # EKF-friendly (ไม่เล็กเกิน ไม่ใหญ่เกิน)
        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0] = 0.05     # x
        odom.pose.covariance[7] = 0.05     # y
        odom.pose.covariance[35] = 0.2     # yaw

        self.pub_odom.publish(odom)


def main():
    rclpy.init()
    node = WheelOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
