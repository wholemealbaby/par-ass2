from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Read package name from environment variable with fallback
    package_name = os.environ.get('SNC_PACKAGE_NAME', 'snc')

    # Declare launch arguments
    testing_mode_arg = DeclareLaunchArgument(
        'testing_mode',
        default_value='false',
        description='When true, blocks all Twist commands to prevent robot movement during testing'
    )

    # Get launch configuration
    testing_mode = LaunchConfiguration('testing_mode')

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
        parameters=[
            {'testing_mode': testing_mode}
        ]
    )

    # Node 4: Twist Mux Node with Testing Mode Lock
    twist_mux_node = Node(
        package=package_name,
        executable='twist_mux',
        name='twist_mux_node',
        output='screen',
        parameters=[
            {'testing_mode': testing_mode},
            {'lock_teleop': True},
            {'lock_manual': True},
            {'cmd_vel_topic': '/cmd_vel'},
            {'cmd_vel_teleop_topic': '/cmd_vel_teleop'},
            {'cmd_vel_manual_topic': '/cmd_vel_manual'}
        ]
    )

    image_topic = '/oak/rgb/image_raw/compressed'
    image_topic_repeat = image_topic + '/repeat'

    best_effort_repeater_node = Node(
        package=package_name,
        executable='best_effort_repeater',
        name='best_effort_repeater',
        output='screen',
        parameters=[
            {'sub_topic_name': image_topic},
            {'repeat_topic_name': image_topic_repeat},
            {'use_compressed': True},   # True — topic is /compressed
        ]
    )

    return LaunchDescription([
        testing_mode_arg,
        navigation_node,
        marker_detection_node,
        path_tracing_node,
        twist_mux_node,
        best_effort_repeater_node,
        # find_object_node
    ])