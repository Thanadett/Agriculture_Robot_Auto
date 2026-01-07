from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory('robot_bringup')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')

    use_teleop = LaunchConfiguration('use_teleop')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_use_teleop = DeclareLaunchArgument(
        'use_teleop',
        default_value='true',
        description='Enable keyboard teleoperation'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz'
    )

    # ================= Robot Localization (EKF) =================
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
            '0', '0', '0',
            '0', '0', '0',
            'base_link', 'imu_link'
        ]
    )

    # ================= Teleop (optional) =================
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        output='screen',
        condition=IfCondition(use_teleop)
    )

    # ================= RViz (optional) =================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        declare_use_teleop,
        declare_use_rviz,

        ekf_node,
        static_tf_imu,
        teleop_node,
        rviz_node
    ])
