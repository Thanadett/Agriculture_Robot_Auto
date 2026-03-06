import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():

    agent_usb0 = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
            'serial',
            '--dev', '/dev/ttyUSB0',
            '-b', '115200'
        ],
        output='screen'
    )

    agent_usb1 = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
            'serial',
            '--dev', '/dev/ttyUSB1',
            '-b', '115200'
        ],
        output='screen'
    )

    return LaunchDescription([
        agent_usb0,
        agent_usb1
    ])


# ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
