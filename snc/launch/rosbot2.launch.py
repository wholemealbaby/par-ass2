from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
 
objects_path = os.path.join(get_package_share_directory('snc'), 'resource', 'hazards')

def generate_launch_description():
    # Read package name from environment variable with fallback
    package_name = os.environ.get('SNC_PACKAGE_NAME', 'snc')

    image_topic = '/oak/rgb/image_raw/compressed'
    image_topic_repeat = image_topic + '/repeat'

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

    # find_object_2d
    find_object_node = Node(
        package='find_object_2d',
        executable='find_object_2d',
        name='find_object_node',
        output='screen',
        parameters=[{
            'subscribe_depth': False,
            'gui': False,
            'objects_path': objects_path,
            'settings_path': '~/.ros/find_object_2d.ini',
        }],
        remappings=[
            ('image', image_topic_repeat),
        ]
    )
 
    # Besteffort repeater
    # Bridges the camera (best_effort QoS) to find_object_2d (reliable QoS).
    # use_compressed MUST be True because image_topic is a compressed topic.
    best_effort_repeater_node = Node(
        package='snc',
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
        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '0'),
 
        # Builds map → odom → base_link TF
        slam_toolbox_node,
 
        # Application nodes
        navigation_node,
        marker_detection_node,
        path_tracing_node,
        twist_mux_node,
 
        # Object detection
        find_object_node,
        best_effort_repeater_node,
    ])