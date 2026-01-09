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

        # Initialize all states properly
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

        self.get_logger().info('wheel_odometry_node started')

    @staticmethod
    def normalize_angle(a):
        """Normalize angle to [-pi, pi]"""
        return math.atan2(math.sin(a), math.cos(a))

    @staticmethod
    def yaw_to_quaternion(yaw):
        """Convert yaw to quaternion"""
        h = 0.5 * yaw
        return Quaternion(x=0.0, y=0.0, z=math.sin(h), w=math.cos(h))

    def on_ticks(self, msg: Float32MultiArray):
        # Validate input
        if len(msg.data) < 4:
            self.get_logger().warn('Invalid wheel_ticks message (< 4 values)')
            return

        # Convert to meters (assuming input is in cm)
        fl, fr, rl, rr = [v * 0.01 for v in msg.data]

        # Check for NaN/inf BEFORE any calculation
        if not all(math.isfinite(v) for v in [fl, fr, rl, rr]):
            self.get_logger().warn(
                f'Non-finite values in ticks: {[fl, fr, rl, rr]}')
            return

        # Average left and right wheels
        left = 0.5 * (fl + rl)
        right = 0.5 * (fr + rr)

        # Initialize on first message
        if self.prev_left is None:
            self.prev_left = left
            self.prev_right = right
            self.get_logger().info(
                f'Initialized: left={left:.3f}, right={right:.3f}')
            return

        # Calculate deltas
        d_left = left - self.prev_left
        d_right = right - self.prev_right

        # Sanity check on deltas (prevent huge jumps)
        max_delta = 1.0  # 1 meter max per update
        if abs(d_left) > max_delta or abs(d_right) > max_delta:
            self.get_logger().warn(
                f'Huge delta detected, skipping: d_left={d_left:.3f}, d_right={d_right:.3f}')
            self.prev_left = left
            self.prev_right = right
            return

        self.prev_left = left
        self.prev_right = right

        # Calculate odometry
        ds = 0.5 * (d_left + d_right)

        # Prevent division issues
        if abs(self.wheel_sep) < 1e-6:
            self.get_logger().error('wheel_separation is too small!')
            return

        dtheta = (d_right - d_left) / self.wheel_sep

        # Update pose
        self.yaw = self.normalize_angle(self.yaw + dtheta)
        self.x += ds * math.cos(self.yaw)
        self.y += ds * math.sin(self.yaw)

        # Final sanity check
        if not all(math.isfinite(v) for v in [self.x, self.y, self.yaw]):
            self.get_logger().error(
                f'NaN detected in pose! x={self.x}, y={self.y}, yaw={self.yaw}')
            # Reset to safe values
            self.x = 0.0
            self.y = 0.0
            self.yaw = 0.0
            return

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

        # Set pose covariance (CRITICAL for EKF!)
        # Format: [x, y, z, roll, pitch, yaw] (6x6 matrix in row-major order)
        odom.pose.covariance = [
            0.1,  0.0,  0.0,  0.0,  0.0,  0.0,   # x
            0.0,  0.1,  0.0,  0.0,  0.0,  0.0,   # y
            0.0,  0.0,  1e6,  0.0,  0.0,  0.0,   # z (not used)
            0.0,  0.0,  0.0,  1e6,  0.0,  0.0,   # roll (not used)
            0.0,  0.0,  0.0,  0.0,  1e6,  0.0,   # pitch (not used)
            0.0,  0.0,  0.0,  0.0,  0.0,  0.1    # yaw
        ]

        # Set twist covariance (even if not using velocities)
        odom.twist.covariance = [
            1e6, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1e6, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1e6
        ]

        self.pub_odom.publish(odom)

        # ---------- TF odom -> base_link ----------
        # Only publish TF if NOT using EKF (EKF will publish this TF)
        # Comment out these lines when using EKF:
        # tf = TransformStamped()
        # tf.header.stamp = now
        # tf.header.frame_id = self.frame_id
        # tf.child_frame_id = self.child_frame_id
        # tf.transform.translation.x = self.x
        # tf.transform.translation.y = self.y
        # tf.transform.translation.z = 0.0
        # tf.transform.rotation = q
        # self.tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = WheelOdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
