from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Declare common arguments ─────────────────────────────────
    args = [
        DeclareLaunchArgument('yolo_model',      default_value='/home/t/392_project/ros2_ws/best.pt',
                              description='Path to YOLO .pt model'),
        DeclareLaunchArgument('target_tag_id',   default_value='-1',
                              description='AprilTag ID to track (-1 = YOLO auto)'),
        DeclareLaunchArgument('tag_size',         default_value='0.042',
                              description='AprilTag physical size in metres'),
        DeclareLaunchArgument('invert_x',         default_value='false'),
        DeclareLaunchArgument('invert_yaw',       default_value='false'),
        DeclareLaunchArgument('forward_vel',      default_value='0.10'),
        DeclareLaunchArgument('reverse_vel',      default_value='0.10'),
        DeclareLaunchArgument('camera_target_z',  default_value='0.50'),
        DeclareLaunchArgument('reverse_after_m',  default_value='0.4565'),
        DeclareLaunchArgument('wait_sec',         default_value='3.0'),
        DeclareLaunchArgument('wheel_diameter',   default_value='0.127'),
        DeclareLaunchArgument('hdg_kp',           default_value='0.8',
                              description='HeadingPID Kp'),
        DeclareLaunchArgument('hdg_kd',           default_value='0.02',
                              description='HeadingPID Kd'),
        DeclareLaunchArgument('window_name',      default_value='Robot Debug',
                              description='Debug X11 window title'),
    ]

    # ── Nodes ────────────────────────────────────────────────────

    laptop_node = Node(
        package='robot_bringup',         
        executable='laptop_node',
        name='laptop_node',
        output='screen',
        parameters=[{
            'yolo_model':      LaunchConfiguration('yolo_model'),
            'target_tag_id':   LaunchConfiguration('target_tag_id'),
            'tag_size':        LaunchConfiguration('tag_size'),
            'invert_x':        LaunchConfiguration('invert_x'),
            'invert_yaw':      LaunchConfiguration('invert_yaw'),
            'cmd_topic':       '/cmd_vel_vision',   # → heading_pid
        }],
    )

    heading_pid_node = Node(
        package='robot_bringup',
        executable='heading_pid_node',
        name='heading_pid_node',
        output='screen',
        parameters=[{
            'kp':              LaunchConfiguration('hdg_kp'),
            'kd':              LaunchConfiguration('hdg_kd'),
            'cmd_in_topic_vision':    '/cmd_vel_vision',  # ← from laptop_node
            'cmd_in_topic_mission':   '/cmd_vel_mission', # ← from mission_controller
            'cmd_out_topic':   '/cmd_vel_pid',
        }],
    )

    mission_controller = Node(
        package='robot_bringup',
        executable='mission_controller',
        name='mission_controller',
        output='screen',
        parameters=[{
            'forward_vel':     LaunchConfiguration('forward_vel'),
            'reverse_vel':     LaunchConfiguration('reverse_vel'),
            'camera_target_z': LaunchConfiguration('camera_target_z'),
            'reverse_after_m': LaunchConfiguration('reverse_after_m'),
            'wait_sec':        LaunchConfiguration('wait_sec'),
            'wheel_diameter':  LaunchConfiguration('wheel_diameter'),
            'cmd_topic':       '/cmd_vel_mission', 
        }],
    )

    debug_x11 = Node(
        package='robot_bringup',
        executable='debug_x11',
        name='debug_x11',
        output='screen',
        parameters=[{
            'window_name':     LaunchConfiguration('window_name'),
            'skip_frames':     2,
        }],
    )

    return LaunchDescription(args + [
        laptop_node,
        heading_pid_node,
        mission_controller,
        debug_x11,
    ])