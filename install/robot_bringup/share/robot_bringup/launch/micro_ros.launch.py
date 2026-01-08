import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    baud_rate = LaunchConfiguration('baud_rate', default='115200')

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='USB port for ESP32 micro-ROS agent'
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Baud rate for serial communication'
        ),

        # เรียกใช้ Micro-ROS Agent Node
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
                'serial',
                '--dev', serial_port,
                '-b', baud_rate
            ],
            output='screen'
        )
    ])


# ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
