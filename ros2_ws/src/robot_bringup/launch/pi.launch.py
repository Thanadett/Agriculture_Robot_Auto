import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # ── Camera 1: Pi Camera ──────────────────────────────────
    pi_camera_node = Node(
        package='robot_bringup', 
        executable='pi_camera_publisher', 
        parameters=[{
            'camera_id': '/dev/webcam_EYD_2k',
            'image_width': 320,
            'image_height': 240,
            'fps': 20,
            'jpeg_quality': 60,
            'publish_raw': False,
            'use_x11_debug': True
        }]
    )

    # ── Camera 2: Side Camera ────────────────────────────────
    side_camera_node = Node(
        package='robot_bringup',
        executable='cabbage_camera_publisher',
        parameters=[{
            'camera_id': '/dev/webcam_EYD_1080p',
            'image_width': 320,
            'image_height': 240,
            'fps': 8,
            'jpeg_quality': 40,
            'publish_raw': True,
            'use_x11_debug': True
        }]
    )

    return LaunchDescription([
        pi_camera_node,
        side_camera_node
    ])