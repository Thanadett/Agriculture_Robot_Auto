#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray

import math


# =======================
# Utility
# =======================
def wrap_angle(angle):
    """Wrap angle to [-pi, pi]"""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


# =======================
# Node
# =======================
class HeadingPID(Node):

    def __init__(self):
        super().__init__('heading_pid')

        # ===== Parameters =====
        self.declare_parameter('kp',               1.8)
        self.declare_parameter('ki',               0.0)
        self.declare_parameter('kd',               0.02)
        self.declare_parameter('max_angular',      0.5)
        self.declare_parameter('enable_threshold', 0.03)

        self.kp          = self.get_parameter('kp').value
        self.ki          = self.get_parameter('ki').value
        self.kd          = self.get_parameter('kd').value
        self.max_angular = self.get_parameter('max_angular').value
        self.enable_th   = self.get_parameter('enable_threshold').value

        # ===== State =====
        self.yaw            = 0.0
        self.yaw_ref        = 0.0
        self.heading_locked = False

        self.integral   = 0.0
        self.prev_error = 0.0
        self.prev_time  = self.get_clock().now()

        self.cmd_in = Twist()

        # ===== ROS I/O =====
        self.create_subscription(Imu,   '/imu/data', self.imu_cb, 10)
        self.create_subscription(Twist, '/cmd_vel',  self.cmd_cb, 10)

        self.cmd_pub   = self.create_publisher(Twist,             '/cmd_vel_pid', 10)
        self.debug_pub = self.create_publisher(Float32MultiArray, '/pid_debug',   10)

        # 50 Hz
        self.timer = self.create_timer(0.02, self.update)

        self.get_logger().info(
            "HeadingPID started\n"
            "  subscribe: /cmd_vel  (from YoloFollower)\n"
            "  publish:   /cmd_vel_pid  (to robot)"
        )

    # =======================
    # Callbacks
    # =======================
    def imu_cb(self, msg: Imu):
        self.yaw = quaternion_to_yaw(msg.orientation)

    def cmd_cb(self, msg: Twist):
        self.cmd_in = msg

    # =======================
    # Control Loop (50 Hz)
    # =======================
    def update(self):
        now = self.get_clock().now()
        dt  = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now

        cmd_out        = Twist()
        cmd_out.linear = self.cmd_in.linear

        debug = Float32MultiArray()

        moving_forward = abs(self.cmd_in.linear.x)  > self.enable_th
        user_turning   = abs(self.cmd_in.angular.z) > self.enable_th

        # ===== CASE 1: TURNING (TRACK / ARC_APPROACH) =====
        # YoloFollower ส่ง angular.z != 0 → pass-through ทันที
        # ไม่ lock heading เพื่อไม่ขัดการควบคุมของ vision
        if user_turning:
            self.heading_locked = False
            self.integral       = 0.0
            self.prev_error     = 0.0

            cmd_out.angular.z = self.cmd_in.angular.z
            self.cmd_pub.publish(cmd_out)
            return

        # ===== CASE 2: FORWARD, NO TURNING (FINAL_APPROACH) =====
        # YoloFollower ส่ง linear.x > 0, angular.z = 0
        # → lock yaw ณ เฟรมแรก แล้วใช้ PID แก้ drift ตลอดการเดิน
        if moving_forward:
            if not self.heading_locked:
                self.yaw_ref        = self.yaw
                self.integral       = 0.0
                self.prev_error     = 0.0
                self.heading_locked = True
                self.get_logger().info(
                    f"Heading locked at {math.degrees(self.yaw_ref):.1f} deg"
                )

            error      = wrap_angle(self.yaw_ref - self.yaw)
            self.integral += error * dt
            derivative = (error - self.prev_error) / dt if dt > 0.0 else 0.0

            u = (self.kp * error +
                 self.ki * self.integral +
                 self.kd * derivative)

            u = max(-self.max_angular, min(self.max_angular, u))

            cmd_out.angular.z = u
            self.prev_error   = error

            debug.data = [error, u, derivative, float(self.yaw_ref), float(self.yaw)]

        # ===== CASE 3: STOP (SEARCH / CONFIRM) =====
        else:
            self.heading_locked = False
            self.integral       = 0.0
            self.prev_error     = 0.0
            cmd_out.angular.z   = 0.0
            debug.data          = [0.0, 0.0, 0.0, 0.0, 0.0]

        self.cmd_pub.publish(cmd_out)
        self.debug_pub.publish(debug)


# =======================
# Main
# =======================
def main(args=None):
    rclpy.init(args=args)
    node = HeadingPID()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()