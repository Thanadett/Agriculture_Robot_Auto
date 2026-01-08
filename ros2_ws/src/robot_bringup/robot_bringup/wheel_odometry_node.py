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

        self.declare_parameter('wheel_separation', 0.365)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')

        self.wheel_sep = self.get_parameter('wheel_separation').value
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.prev_left = None
        self.prev_right = None

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

    def normalize_angle(self, a):
        return math.atan2(math.sin(a), math.cos(a))

    def on_ticks(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            return

        fl, fr, rl, rr = [v * 0.01 for v in msg.data]

        left = 0.5 * (fl + rl)
        right = 0.5 * (fr + rr)

        if self.prev_left is None:
            self.prev_left = left
            self.prev_right = right
            return

        d_left = left - self.prev_left
        d_right = right - self.prev_right
        self.prev_left = left
        self.prev_right = right

        ds = 0.5 * (d_left + d_right)
        dtheta = (d_right - d_left) / self.wheel_sep

        self.yaw = self.normalize_angle(self.yaw + dtheta)
        self.x += ds * math.cos(self.yaw)
        self.y += ds * math.sin(self.yaw)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        q = R.from_euler('z', self.yaw).as_quat()
        odom.pose.pose.orientation = Quaternion(
            x=float(q[0]),
            y=float(q[1]),
            z=float(q[2]),
            w=float(q[3]),
        )

        odom.pose.covariance[0] = 0.1
        odom.pose.covariance[7] = 0.1
        odom.pose.covariance[35] = 0.5

        self.pub_odom.publish(odom)


def main():
    rclpy.init()
    node = WheelOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
