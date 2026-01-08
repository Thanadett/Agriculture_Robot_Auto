from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
# import os


def generate_launch_description():

    # pkg_share = get_package_share_directory('robot_bringup')

    # ================= Heading PID =================
    heading_pid_node = Node(
        package='robot_bringup',
        executable='heading_pid_node',
        name='heading_pid',
        output='screen'
    )

    # ================= Wheel Odometry =================
    wheel_odometry_node = Node(
        package='robot_bringup',
        executable='wheel_odometry_node',
        name='wheel_odometry_node',
        output='screen'
    )

    # ================= Static TF =================
    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_imu',
        arguments=[
            '0', '0', '0',
            '0', '0', '0',
            'base_link',
            'imu_link'
        ]
    )

    return LaunchDescription([
        heading_pid_node,
        wheel_odometry_node,
        static_tf_imu,
    ])
