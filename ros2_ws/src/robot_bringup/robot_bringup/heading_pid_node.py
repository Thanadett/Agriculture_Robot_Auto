#!/usr/bin/env python3
"""
Heading PID Node
============================================================
ทำหน้าที่เป็น "heading stabilizer" ระหว่าง AprilTag Servo กับหุ่นยนต์

  /cmd_vel → [HeadingPID] → /cmd_vel_pid → robot

Logic:
  • TURNING (angular.z ≠ 0)  → pass-through ทันที, ไม่ lock heading
  • FORWARD (linear.x > 0, angular.z ≈ 0) → lock yaw ณ เฟรมแรก,
      ใช้ IMU PID แก้ drift ตลอดเวลาเดิน (ไม่ oscillate)
  • STOP    → reset lock, ส่งศูนย์

Topics subscribed:
  /imu/data   (sensor_msgs/Imu)
  /cmd_vel    (geometry_msgs/Twist)

Topics published:
  /cmd_vel_pid  (geometry_msgs/Twist)
  /pid_debug    (std_msgs/Float32MultiArray)  [error, u, deriv, yaw_ref, yaw]
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

        self.declare_parameter('kp',               1.8)
        self.declare_parameter('ki',               0.0)
        self.declare_parameter('kd',               0.02)
        self.declare_parameter('max_angular',      0.5)
        self.declare_parameter('enable_threshold', 0.03)
        self.declare_parameter('cmd_in_topic',  '/cmd_vel')
        self.declare_parameter('cmd_out_topic', '/cmd_vel_pid')

        self.kp          = self.get_parameter('kp').value
        self.ki          = self.get_parameter('ki').value
        self.kd          = self.get_parameter('kd').value
        self.max_angular = self.get_parameter('max_angular').value
        self.enable_th   = self.get_parameter('enable_threshold').value
        cmd_in  = self.get_parameter('cmd_in_topic').value
        cmd_out = self.get_parameter('cmd_out_topic').value

        self.yaw            = 0.0
        self.yaw_ref        = 0.0
        self.heading_locked = False
        self.integral       = 0.0
        self.prev_error     = 0.0
        self.prev_time      = self.get_clock().now()
        self.cmd_in         = Twist()

        # ── track last u for debug display ──────────────────────
        self.last_u     = 0.0
        self.last_error = 0.0
        self.last_deriv = 0.0

        self.create_subscription(Imu,   '/imu/data', self._imu_cb,  10)
        self.create_subscription(Twist, cmd_in,      self._cmd_cb,  10)
        self.cmd_pub   = self.create_publisher(Twist,             cmd_out,     10)
        self.debug_pub = self.create_publisher(Float32MultiArray, '/pid_debug', 10)

        self.timer = self.create_timer(0.02, self._update)   # 50 Hz
        self.get_logger().info(
            f"HeadingPID ready  {cmd_in} → {cmd_out}"
            f"  Kp={self.kp}  Ki={self.ki}  Kd={self.kd}")

    def _imu_cb(self, msg: Imu):
        self.yaw = quat_to_yaw(msg.orientation)

    def _cmd_cb(self, msg: Twist):
        self.cmd_in = msg

    def _update(self):
        now = self.get_clock().now()
        dt  = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now
        if dt <= 0: dt = 0.02

        cmd_out        = Twist()
        cmd_out.linear = self.cmd_in.linear
        dbg            = Float32MultiArray()

        moving_forward = abs(self.cmd_in.linear.x)  > self.enable_th
        user_turning   = abs(self.cmd_in.angular.z) > self.enable_th

        # ── CASE 1: Turning → pass-through (APPROACH / ALIGN) ───
        if user_turning:
            self.heading_locked = False
            self.integral = 0.0; self.prev_error = 0.0
            cmd_out.angular.z = self.cmd_in.angular.z
            # update yaw_ref while turning so lock starts from current heading
            self.yaw_ref = self.yaw
            dbg.data = [0.0, float(self.cmd_in.angular.z), 0.0,
                        float(self.yaw_ref), float(self.yaw)]
            self.cmd_pub.publish(cmd_out)
            self.debug_pub.publish(dbg)
            return

        # ── CASE 2: Forward straight (FORWARD) → lock + PID ─────
        if moving_forward:
            if not self.heading_locked:
                self.yaw_ref        = self.yaw
                self.integral       = 0.0
                self.prev_error     = 0.0
                self.heading_locked = True
                self.get_logger().info(
                    f"[HeadingPID] lock @ {math.degrees(self.yaw_ref):.1f}°")

            error      = wrap_angle(self.yaw_ref - self.yaw)
            self.integral  += error * dt
            derivative  = (error - self.prev_error) / dt
            self.prev_error = error

            u = (self.kp * error + self.ki * self.integral + self.kd * derivative)
            u = max(-self.max_angular, min(self.max_angular, u))

            cmd_out.angular.z = u
            self.last_u = u; self.last_error = error; self.last_deriv = derivative
            dbg.data = [error, u, derivative, float(self.yaw_ref), float(self.yaw)]

        # ── CASE 3: Stopped → reset ──────────────────────────────
        else:
            self.heading_locked = False
            self.integral = 0.0; self.prev_error = 0.0
            cmd_out.angular.z = 0.0
            dbg.data = [0.0, 0.0, 0.0, float(self.yaw_ref), float(self.yaw)]

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