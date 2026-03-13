#!/usr/bin/env python3
"""
Encoder Test
====================================
Forward 60 cm
Reverse 60 cm

SUB
  /wheel_ticks

PUB
  /cmd_vel_pid
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


MS_START   = 0
MS_FORWARD = 1
MS_WAIT    = 2
MS_REVERSE = 3
MS_STOP    = 4


class EncoderTest(Node):

    def __init__(self):

        super().__init__("encoder_test")

        self.create_subscription(
            Float32MultiArray,
            "/wheel_ticks",
            self.cb_ticks,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel_pid",
            10
        )

        self.timer = self.create_timer(
            0.05,
            self.tick
        )

        self.ms = MS_START

        self.ticks_now = 0.0
        self.ticks_ref = 0.0

        self.target_m = 0.60

        self.forward_vel = 0.12
        self.reverse_vel = 0.12

        self.wait_start = None

        self.get_logger().info("Encoder Forward/Reverse Test Ready")

    # ─────────────────────────────

    def cb_ticks(self, msg):

        if len(msg.data) >= 4:

            self.ticks_now = sum(abs(x) for x in msg.data[:4]) / 4 / 100.0

        elif len(msg.data) > 0:

            self.ticks_now = abs(msg.data[0]) / 100.0

    # ─────────────────────────────

    def travelled(self):

        return abs(self.ticks_now - self.ticks_ref)

    # ─────────────────────────────

    def start_move(self, dist):

        self.ticks_ref = self.ticks_now
        self.target_m = dist

        self.get_logger().info(f"[MOVE] target={dist:.2f} m")

    # ─────────────────────────────

    def stop(self):

        self.cmd_pub.publish(Twist())

    # ─────────────────────────────

    def forward(self):

        cmd = Twist()
        cmd.linear.x = self.forward_vel
        self.cmd_pub.publish(cmd)

    # ─────────────────────────────

    def reverse(self):

        cmd = Twist()
        cmd.linear.x = -self.reverse_vel
        self.cmd_pub.publish(cmd)

    # ─────────────────────────────

    def tick(self):

        if self.ms == MS_START:

            self.start_move(0.60)
            self.ms = MS_FORWARD

        # ─────────────────────────

        elif self.ms == MS_FORWARD:

            dist = self.travelled()

            self.get_logger().info(f"[FORWARD] {dist:.3f} m")

            if dist >= (self.target_m - 0.01):

                self.stop()
                self.wait_start = self.get_clock().now().nanoseconds / 1e9
                self.ms = MS_WAIT

                self.get_logger().info("[FORWARD DONE]")

            else:

                self.forward()

        # ─────────────────────────

        elif self.ms == MS_WAIT:

            now = self.get_clock().now().nanoseconds / 1e9

            if (now - self.wait_start) > 2.0:

                self.start_move(0.60)
                self.ms = MS_REVERSE

        # ─────────────────────────

        elif self.ms == MS_REVERSE:

            dist = self.travelled()

            self.get_logger().info(f"[REVERSE] {dist:.3f} m")

            if dist >= (self.target_m - 0.01):

                self.stop()
                self.ms = MS_STOP

                self.get_logger().info("[REVERSE DONE]")

            else:

                self.reverse()

        # ─────────────────────────

        elif self.ms == MS_STOP:

            self.stop()


def main(args=None):

    rclpy.init(args=args)

    node = EncoderTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()