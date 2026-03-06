from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('tracking_cpp')  
    
    config_file = os.path.join(package_dir, 'config', 'real2.yaml')

    real2 = Node(
        package='tracking_cpp',  
        executable='real2',
        name='follow_target',
        output='screen',
        parameters=[config_file]
    )


    following22222 = Node(
        package='tracking_cpp',  
        executable='following22222',
        name='follow_target',
        output='screen',
        parameters=[config_file]
    )




    real2_plots = Node(
        package='tracking_cpp',  
        executable='real2_plots',
        name='follow_target',
        output='screen',
        parameters=[config_file]
    )

    
    mot = Node(
        package='tracking_cpp',  
        executable='mot',
        name='follow_target',
        output='screen',
        parameters=[config_file]
    )



    return LaunchDescription([
        # follow_target_node
        # counter_node
        # real1
        # real2,
        real2_plots
        # mot
        # following22222
        # real2_backup
        # checking
        # example_code
    ])
