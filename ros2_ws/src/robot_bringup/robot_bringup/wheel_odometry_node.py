#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
from std_srvs.srv import Empty


class WheelOdometryNode(Node):

    def __init__(self):
        super().__init__('wheel_odometry_node')

        self.declare_parameter('wheel_separation', 0.365)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        # ไม่ publish TF ถ้าใช้ EKF
        self.declare_parameter('publish_tf', False)

        self.wheel_sep = float(self.get_parameter('wheel_separation').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # Initialize all states properly
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.prev_left = None
        self.prev_right = None
        self.first_valid_msg = False

        # Offset for encoder reset
        self.offset_left = 0.0
        self.offset_right = 0.0

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

        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        else:
            self.tf_broadcaster = None

        # Reset service
        self.reset_srv = self.create_service(
            Empty,
            '~/reset_odometry',
            self.reset_odometry_callback
        )

        self.get_logger().info(
            f'wheel_odometry_node started (publish_tf={self.publish_tf})'
        )

    @staticmethod
    def normalize_angle(a):
        """Normalize angle to [-pi, pi]"""
        return math.atan2(math.sin(a), math.cos(a))

    @staticmethod
    def yaw_to_quaternion(yaw):
        """Convert yaw to quaternion"""
        h = 0.5 * yaw
        return Quaternion(x=0.0, y=0.0, z=math.sin(h), w=math.cos(h))

    def reset_odometry_callback(self, request, response):
        """Reset odometry to (0, 0, 0)"""
        self.get_logger().info('Resetting odometry...')

        # Reset pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Don't reset prev_left/prev_right to allow continuous operation
        # Just mark for re-initialization
        self.first_valid_msg = False

        self.get_logger().info('Odometry reset complete')
        return response

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
        left = 0.5 * (fl + rl) - self.offset_left
        right = 0.5 * (fr + rr) - self.offset_right

        # Initialize on first message
        if self.prev_left is None:
            self.prev_left = left
            self.prev_right = right
            self.offset_left = 0.5 * (fl + rl)
            self.offset_right = 0.5 * (fr + rr)
            self.get_logger().info(
                f'Initialized with offsets: left={self.offset_left:.3f}, '
                f'right={self.offset_right:.3f}'
            )
            return

        # Calculate deltas
        d_left = left - self.prev_left
        d_right = right - self.prev_right

        # Sanity check on deltas
        # Be more lenient on first few messages
        max_delta = 10.0 if not self.first_valid_msg else 5.0

        if abs(d_left) > max_delta or abs(d_right) > max_delta:
            self.get_logger().warn(
                f'Huge delta detected (>{max_delta}m), skipping: '
                f'd_left={d_left:.3f}, d_right={d_right:.3f}'
            )
            # Reset prev values to current (skip this frame)
            self.prev_left = left
            self.prev_right = right
            return

        # Mark as valid after first successful update
        if not self.first_valid_msg:
            self.first_valid_msg = True
            self.get_logger().info('First valid odometry update received')

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
                f'NaN detected in pose! x={self.x}, y={self.y}, yaw={self.yaw}'
            )
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
        odom.pose.covariance = [
            0.1,  0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.1,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  1e6,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  1e6,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  1e6,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.1
        ]

        # Set twist covariance
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
        # Only publish TF if not using EKF
        if self.tf_broadcaster is not None:
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
