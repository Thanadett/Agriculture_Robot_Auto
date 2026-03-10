#!/usr/bin/env python3
"""
Heading PID Node
============================================================
ทำหน้าที่เป็น heading stabilizer + command mux

Pipeline:

AprilTagServo      → /cmd_vel_vision
MissionController  → /cmd_vel_mission
                           ↓
                       HeadingPID
                           ↓
                       /cmd_vel_pid
                           ↓
                         robot

Logic:
  • TURNING (angular.z ≠ 0)  → pass-through
  • FORWARD (linear.x > 0)   → lock yaw + PID
  • STOP                     → reset
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def quat_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class HeadingPID(Node):

    def __init__(self):
        super().__init__('heading_pid')

        # ── parameters ─────────────────────────
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.02)
        self.declare_parameter('max_angular', 0.4)
        self.declare_parameter('enable_threshold', 0.03)

        self.declare_parameter('cmd_in_topic_vision', '/cmd_vel_vision')
        self.declare_parameter('cmd_in_topic_mission', '/cmd_vel_mission')
        self.declare_parameter('cmd_out_topic', '/cmd_vel_pid')

        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.max_angular = self.get_parameter('max_angular').value
        self.enable_th = self.get_parameter('enable_threshold').value

        topic_vision = self.get_parameter('cmd_in_topic_vision').value
        topic_mission = self.get_parameter('cmd_in_topic_mission').value
        topic_out = self.get_parameter('cmd_out_topic').value

        # ── state ─────────────────────────
        self.yaw = 0.0
        self.yaw_ref = 0.0
        self.heading_locked = False

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = self.get_clock().now()

        # command buffers
        self.cmd_vision = Twist()
        self.cmd_mission = Twist()

        # debug
        self.last_u = 0.0
        self.last_error = 0.0
        self.last_deriv = 0.0

        # ── ROS ─────────────────────────
        self.create_subscription(Imu, '/imu/data', self._imu_cb, 10)
        self.create_subscription(Twist, topic_vision, self._cmd_cb_vision, 10)
        self.create_subscription(Twist, topic_mission, self._cmd_cb_mission, 10)

        self.cmd_pub = self.create_publisher(Twist, topic_out, 10)
        self.debug_pub = self.create_publisher(Float32MultiArray, '/pid_debug', 10)

        self.timer = self.create_timer(0.02, self._update)

        self.get_logger().info(
            f"HeadingPID ready  vision:{topic_vision} mission:{topic_mission} → {topic_out}"
        )

    # ── callbacks ─────────────────────────

    def _imu_cb(self, msg: Imu):
        self.yaw = quat_to_yaw(msg.orientation)

    def _cmd_cb_vision(self, msg: Twist):
        self.cmd_vision = msg

    def _cmd_cb_mission(self, msg: Twist):
        self.cmd_mission = msg

    # ── main loop ─────────────────────────

    def _update(self):

        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now
        if dt <= 0:
            dt = 0.02

        # ── command mux ─────────────────
        cmd_src = Twist()

        mission_override = (
            abs(self.cmd_mission.linear.x) > 0.001 or
            abs(self.cmd_mission.angular.z) > 0.001
        )

        if mission_override:
            cmd_src = self.cmd_mission
        else:
            cmd_src = self.cmd_vision

        cmd_out = Twist()
        cmd_out.linear = cmd_src.linear

        dbg = Float32MultiArray()

        # ── mission override → bypass PID ─────────────
        if mission_override:

            cmd_out.angular.z = cmd_src.angular.z

            dbg.data = [
                0.0,
                float(cmd_src.angular.z),
                0.0,
                float(self.yaw_ref),
                float(self.yaw),
            ]

            self.cmd_pub.publish(cmd_out)
            self.debug_pub.publish(dbg)
            return

        moving_forward = abs(cmd_src.linear.x) > self.enable_th
        user_turning = abs(cmd_src.angular.z) > self.enable_th

        # ── CASE 1: turning ─────────────────
        if user_turning:

            self.heading_locked = False
            self.integral = 0.0
            self.prev_error = 0.0

            cmd_out.angular.z = cmd_src.angular.z
            self.yaw_ref = self.yaw

            dbg.data = [
                0.0,
                float(cmd_src.angular.z),
                0.0,
                float(self.yaw_ref),
                float(self.yaw),
            ]

            self.cmd_pub.publish(cmd_out)
            self.debug_pub.publish(dbg)
            return

        # ── CASE 2: forward ─────────────────
        if moving_forward:

            if not self.heading_locked:
                self.yaw_ref = self.yaw
                self.integral = 0.0
                self.prev_error = 0.0
                self.heading_locked = True

                self.get_logger().info(
                    f"[HeadingPID] lock @ {math.degrees(self.yaw_ref):.1f}°"
                )

            error = wrap_angle(self.yaw_ref - self.yaw)

            # deadband
            if abs(error) < 0.02:
                error = 0.0

            self.integral += error * dt
            derivative = (error - self.prev_error) / dt
            self.prev_error = error

            u = self.kp * error + self.ki * self.integral + self.kd * derivative
            u = max(-self.max_angular, min(self.max_angular, u))

            cmd_out.angular.z = u

            self.last_u = u
            self.last_error = error
            self.last_deriv = derivative

            dbg.data = [
                error,
                u,
                derivative,
                float(self.yaw_ref),
                float(self.yaw),
            ]

        # ── CASE 3: stop ─────────────────
        else:

            self.heading_locked = False
            self.integral = 0.0
            self.prev_error = 0.0

            cmd_out.angular.z = 0.0

            dbg.data = [
                0.0,
                0.0,
                0.0,
                float(self.yaw_ref),
                float(self.yaw),
            ]

        self.cmd_pub.publish(cmd_out)
        self.debug_pub.publish(dbg)


def main(args=None):

    rclpy.init(args=args)

    node = HeadingPID()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()