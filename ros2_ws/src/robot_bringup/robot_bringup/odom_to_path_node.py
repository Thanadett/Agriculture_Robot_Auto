#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class OdomToPathNode(Node):

    def __init__(self):
        super().__init__('odom_to_path_node')

        # ===== parameters =====
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('path_topic', '/path')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('max_length', 1000)  # ป้องกัน RAM บวม

        odom_topic = self.get_parameter('odom_topic').value
        path_topic = self.get_parameter('path_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.max_length = int(self.get_parameter('max_length').value)

        # ===== subscriber / publisher =====
        self.sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        self.pub = self.create_publisher(
            Path,
            path_topic,
            10
        )

        # ===== path message =====
        self.path = Path()
        self.path.header.frame_id = self.frame_id

        self.get_logger().info(
            f'odom_to_path_node started: {odom_topic} -> {path_topic}'
        )

    def odom_callback(self, msg: Odometry):
        # สร้าง PoseStamped จาก odometry
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self.frame_id
        pose.pose = msg.pose.pose

        self.path.header.stamp = msg.header.stamp
        self.path.poses.append(pose)

        # จำกัดความยาว path (กัน memory บวม)
        if len(self.path.poses) > self.max_length:
            self.path.poses.pop(0)

        self.pub.publish(self.path)


def main():
    rclpy.init()
    node = OdomToPathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
