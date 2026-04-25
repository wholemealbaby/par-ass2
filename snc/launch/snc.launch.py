from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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
        navigation_node,
        marker_detection_node,
        path_tracing_node,
        best_effort_repeater_node,
        # find_object_node
    ])