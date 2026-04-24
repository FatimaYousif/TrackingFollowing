from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('tracking_cpp')  
    
    config_file = os.path.join(package_dir, 'config', 'real1.yaml')
    
    # follow_target_node = Node(
    #     package='tracking_cpp',  
    #     executable='test1',
    #     name='follow_target',
    #     output='screen',
    #     parameters=[config_file]
    # )
    
    # counter_node = Node(
    #     package='tracking_cpp',  
    #     executable='params_practice',
    #     name='follow_target',
    #     output='screen',
    #     parameters=[config_file]
    # )

    # example_code = Node(
    #     package='tracking_cpp',  
    #     executable='example_code',
    #     name='follow_target',
    #     output='screen',
    #     parameters=[config_file]
    # )


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


    real2_backup = Node(
        package='tracking_cpp',  
        executable='real2_backup',
        name='follow_target',
        output='screen',
        parameters=[config_file]
    )

    checking = Node(
        package='tracking_cpp',  
        executable='checking',
        name='follow_target',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        # follow_target_node
        # counter_node
        real1
        # real2,
        # real2_backup
        # checking
        # example_code
    ])