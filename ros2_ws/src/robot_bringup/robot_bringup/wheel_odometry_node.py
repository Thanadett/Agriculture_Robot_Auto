#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster


class WheelOdometryNode(Node):

    def __init__(self):
        super().__init__('wheel_odometry_node')

        self.declare_parameter('wheel_separation', 0.365)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')

        self.wheel_sep = float(self.get_parameter('wheel_separation').value)
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

        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info('wheel_odometry_node started (NO EKF, TF ENABLED)')

    @staticmethod
    def normalize_angle(a):
        return math.atan2(math.sin(a), math.cos(a))

    @staticmethod
    def yaw_to_quaternion(yaw):
        h = 0.5 * yaw
        return Quaternion(x=0.0, y=0.0, z=math.sin(h), w=math.cos(h))

    def on_ticks(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            return

        fl, fr, rl, rr = [v * 0.01 for v in msg.data]
        if not all(math.isfinite(v) for v in [fl, fr, rl, rr]):
            return

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

        q = self.yaw_to_quaternion(self.yaw)

        now = self.get_clock().now().to_msg()

        # ---------- Odometry ----------
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q

        self.pub_odom.publish(odom)

        # ---------- TF odom -> base_link ----------
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = self.frame_id
        tf.child_frame_id = self.child_frame_id

        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation = q

        self.tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = WheelOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
