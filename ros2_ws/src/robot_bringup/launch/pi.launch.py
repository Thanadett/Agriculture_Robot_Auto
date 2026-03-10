import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # ── Camera 1: Pi Camera ──────────────────────────────────
    pi_camera_node = Node(
        package='robot_bringup', # <-- เปลี่ยนเป็นชื่อ package ของคุณ
        executable='pi_camera_publisher', # <-- เปลี่ยนเป็นชื่อ executable ใน setup.py
        name='pi_camera_publisher',
        parameters=[{
            'camera_id': '/dev/webcam_EYD_2k',
            'image_width': 640,
            'image_height': 480,
            'fps': 10,
            'jpeg_quality': 80,
            'publish_raw': False,
            'use_x11_debug': False
        }]
    )

    # ── Camera 2: Side Camera ────────────────────────────────
    side_camera_node = Node(
        package='robot_bringup', # <-- เปลี่ยนเป็นชื่อ package ของคุณ
        executable='cabbage_camera_publisher', # <-- เปลี่ยนเป็นชื่อ executable ใน setup.py
        name='cam_side_publisher',
        parameters=[{
            'camera_id': '/dev/webcam_EYD_1080p',
            'image_width': 320,
            'image_height': 240,
            'fps': 10,
            'jpeg_quality': 60,
            'publish_raw': True,
            'use_x11_debug': False
        }]
    )

    return LaunchDescription([
        pi_camera_node,
        side_camera_node
    ])