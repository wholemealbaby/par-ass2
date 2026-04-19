from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Read package name from environment variable with fallback
    package_name = os.environ.get('SNC_PACKAGE_NAME', 'snc')

    # Node 1: Navigation Node
    navigation_node = Node(
        package=package_name,
        executable='navigation_node',
        name='navigation_node_ex',
        output='screen',
        parameters=[]
    )

    # Node 2: Marker Detection Node
    marker_detection_node = Node(
        package=package_name,
        executable='marker_detection_node',
        name='marker_detection_node_ex',
        output='screen',
        parameters=[]
    )

    # Node 3: Path Tracing Node
    path_tracing_node = Node(
        package=package_name,
        executable='path_tracing_node',
        name='path_tracing_node_ex',
        output='screen',
        parameters=[]
    )

    # Node 4: Twist Mux Node with Testing Mode Lock
    twist_mux_node = Node(
        package=package_name,
        executable='twist_mux',
        name='twist_mux_node',
        output='screen',
        parameters=[
            {'testing_mode': False},
            {'lock_teleop': True},
            {'lock_manual': True},
            {'cmd_vel_topic': '/cmd_vel'},
            {'cmd_vel_teleop_topic': '/cmd_vel_teleop'},
            {'cmd_vel_manual_topic': '/cmd_vel_manual'}
        ]
    )

    # SLAM Toolbox Node - Configured for ROSbot 2
    # Uses base_link as the robot base frame and odom as the odometry frame
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_node',
        output='screen',
        parameters=[{
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'scan_topic': '/scan',
            'scan_frame': 'laser',
            'use_sim_time': False,
            'queue_size': 100,
            'min_range': 0.1,
        }]
    )

    return LaunchDescription([
        navigation_node,
        marker_detection_node,
        path_tracing_node,
        twist_mux_node,
        slam_toolbox_node,
    ])
