from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory('robot_bringup')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')

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
        output='screen',
        parameters=[{
            'use_sim_time': False
        }]
    )

    # ================= EKF =================
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('odometry/filtered', '/odometry/filtered')
        ]
    )

    # ================= Odom -> Path =================
    odom_to_path_node = Node(
        package='robot_bringup',
        executable='odom_to_path_node',
        name='odom_to_path_node',
        output='screen',
        parameters=[
            {'odom_topic': '/odometry/filtered'},
            {'path_topic': '/path'},
            {'frame_id': 'odom'},
            {'max_length': 1000}
        ]
    )

    # ================= Reset Manager =================
    reset_manager_node = Node(
        package='robot_bringup',
        executable='reset_manager_node',
        name='reset_manager_node',
        output='screen',
        parameters=[{
            'services': [
                '/wheel_ticks/reset',
                '/wheel_odometry_node/reset_odometry',
                '/ekf_node/reset',
                '/odom_to_path_node/reset'
            ]
        }]
    )

    # ================= Static TF (IMU) =================
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
        ekf_node,
        odom_to_path_node,
        static_tf_imu,
        reset_manager_node,
    ])
