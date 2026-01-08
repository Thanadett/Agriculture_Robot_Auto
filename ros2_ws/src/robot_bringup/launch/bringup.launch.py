from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory('robot_bringup')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')

    # ================= Heading PID (Yaw Control) =================
    heading_pid_node = Node(
        package='robot_bringup',
        executable='heading_pid_node',
        name='heading_pid',
        output='screen',
        parameters=[
            {
                'kp': 1.0,
                'ki': 0.0,
                'kd': 0.0,
                'max_angular': 0.5,
                'enable_threshold': 0.05
            }
        ]
    )

    # ================= EKF =================
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config]
    )

    # ================= Static TF: base_link -> imu_link =================
    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_imu',
        arguments=[
            '0', '0', '0',     # xyz
            '0', '0', '0',     # rpy
            'base_link',
            'imu_link'
        ]
    )

    # ================= Wheel Odometry =================
    wheel_odometry_node = Node(
        package='robot_bringup',
        executable='wheel_odometry_node',
        name='wheel_odometry_node',
        output='screen'
    )

    return LaunchDescription([
        heading_pid_node,
        ekf_node,
        static_tf_imu,
        wheel_odometry_node,
    ])
