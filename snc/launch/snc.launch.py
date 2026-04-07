from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node', # Common executable for slam_toolbox
        name='slam_node',
        output='screen',
        parameters=[{'use_sim_time': True}] # Set to False if running on real hardware
    )

    nav2_node = Node(
        package='nav2_bringup',
        executable='bringup_launch', 
        name='nav2_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    find_object_node = Node(
        package='find_object_2d',
        executable='find_object_2d',
        name='find_object_node',
        output='screen'
    )

    return LaunchDescription([
        slam_toolbox_node,
        nav2_node,
        find_object_node
    ])