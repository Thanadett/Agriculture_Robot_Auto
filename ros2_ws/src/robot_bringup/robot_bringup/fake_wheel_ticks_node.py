#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class FakeWheelTicks(Node):

    def __init__(self):
        super().__init__('fake_wheel_ticks')

        self.pub = self.create_publisher(
            Float32MultiArray,
            '/wheel_ticks',
            10
        )

        self.declare_parameter('rate', 10.0)
        self.declare_parameter('tick_step', 1.0)

        self.rate = self.get_parameter('rate').value
        self.tick_step = self.get_parameter('tick_step').value

        self.tick = 0.0

        self.timer = self.create_timer(
            1.0 / self.rate,
            self.timer_callback
        )

        self.get_logger().info(
            f'Fake wheel ticks started (rate={self.rate} Hz)'
        )

    def timer_callback(self):
        self.tick += self.tick_step

        msg = Float32MultiArray()
        msg.data = [
            self.tick,
            self.tick,
            self.tick,
            self.tick
        ]

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakeWheelTicks()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
