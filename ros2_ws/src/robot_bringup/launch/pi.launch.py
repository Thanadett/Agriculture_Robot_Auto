import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    pi_camera_node = Node(
        package='robot_bringup',
        executable='pi_camera_publisher',
        parameters=[{
            'camera_id': '/dev/webcam_EYD_2k',
            'image_width': 640,
            'image_height': 480,
            'fps': 20,
            'jpeg_quality': 80,
            'publish_raw': False,
            'use_x11_debug': False
        }]
    )

    side_camera_node = Node(
        package='robot_bringup',
        executable='cabbage_camera_publisher',
        parameters=[{
            'camera_id': '/dev/webcam_EYD_1080p',
            'image_width': 640,
            'image_height': 480,
            'fps': 10,
            'jpeg_quality': 40,
            'publish_raw': False,
            'use_x11_debug': False
        }]
    )

    return LaunchDescription([
        pi_camera_node,
        side_camera_node
    ])