from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Node 1: Navigation Node
    navigation_node = Node(
        package='snc',
        executable='navigation_node',
        name='navigation_node',
        output='screen',
        parameters=[]
    )     

    # Node 2: Marker Detection Node
    marker_detection_node = Node(
        package='snc',
        executable='marker_detection_node',
        name='marker_detection_node',
        output='screen',
        parameters=[]
    )

    # Node 3: Path Tracing Node
    path_tracing_node = Node(
        package='snc',
        executable='path_tracing_node',
        name='path_tracing_node',
        output='screen',
        parameters=[]
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node', # Common executable for slam_toolbox
        name='slam_node',
        output='screen',
        parameters=[] 
    )

    nav2_node = Node(
        package='aiil_nav2', # Uses aiil_nav2 bringup launch file
        executable='bringup_launch', 
        name='nav2_node',
        output='screen',
        parameters=[]
    )

    find_object_node = Node(
        package='find_object_2d',
        executable='find_object_2d',
        name='find_object_node',
        output='screen'
    )

    return LaunchDescription([
        navigation_node,
        marker_detection_node,
        path_tracing_node,
        slam_toolbox_node,
        nav2_node,
        find_object_node
    ])