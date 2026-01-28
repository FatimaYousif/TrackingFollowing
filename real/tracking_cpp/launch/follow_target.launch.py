from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('tracking_cpp')  
    
    config_file = os.path.join(package_dir, 'config', 'follow_target_params.yaml')
    
    real1 = Node(
        package='tracking_cpp',  
        executable='real1',
        name='search_target',
        output='screen',
        parameters=[config_file]
    )

    real2 = Node(
        package='tracking_cpp',  
        executable='real2',
        name='follow_target',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        # real1
        real2
    ])