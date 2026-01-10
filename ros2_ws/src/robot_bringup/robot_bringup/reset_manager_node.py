#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty


class ResetManagerNode(Node):

    def __init__(self):
        super().__init__('reset_manager_node')

        # ================= Parameters =================
        self.declare_parameter(
            'services',
            [
                '/wheel_ticks/reset',
                '/wheel_odometry_node/reset_odometry',
                '/ekf_node/reset',
                '/odom_to_path_node/reset'
            ]
        )

        self.service_names = self.get_parameter('services').value

        # ================= Service =================
        self.reset_srv = self.create_service(
            Empty,
            '/reset_all',
            self.reset_all_callback
        )

        self.get_logger().info('Reset Manager Node started')
        self.get_logger().info('Services to reset:')
        for s in self.service_names:
            self.get_logger().info(f'  - {s}')

    # ==================================================
    def reset_all_callback(self, request, response):
        self.get_logger().info('=== RESET ALL START ===')

        for service_name in self.service_names:
            if not self.call_empty_service(service_name):
                self.get_logger().warn(
                    f'Skip or failed: {service_name}'
                )

        self.get_logger().info('=== RESET ALL COMPLETE ===')
        return response

    # ==================================================
    def call_empty_service(self, service_name: str) -> bool:
        client = self.create_client(Empty, service_name)

        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                f'Service not available: {service_name}'
            )
            return False

        req = Empty.Request()
        future = client.call_async(req)

        rclpy.spin_until_future_complete(
            self, future, timeout_sec=1.0
        )

        if future.result() is None:
            self.get_logger().error(
                f'Service call failed: {service_name}'
            )
            return False

        self.get_logger().info(f'Reset OK: {service_name}')
        return True


def main():
    rclpy.init()
    node = ResetManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
