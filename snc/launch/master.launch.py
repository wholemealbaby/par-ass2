from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    package_share_dir = get_package_share_directory('snc')
    
    # Include the snc.launch.py file
    snc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_share_dir, 'launch', 'snc.launch.py'))
    )
    
    # Include the find_object.launch.py file
    find_object_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_share_dir, 'launch', 'find_object.launch.py'))
    )
    
    return LaunchDescription([
        snc_launch,
        find_object_launch
    ])