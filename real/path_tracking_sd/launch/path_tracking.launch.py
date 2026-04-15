from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from ament_index_python.packages import get_package_share_directory
import socket
import os

def generate_launch_description():
    params_file = os.path.join(
    get_package_share_directory('path_tracking'),
    'config',
    'params.yaml'
    )
    
    return LaunchDescription([
        # This checks if the code is running on the RPi with a hostname -> osprey
        #DeclareLaunchArgument(
        #    name='RPi_hostname',
        #    default_value='osprey',
        #    description='Change code to run on RPi'
        #    ),
        # Control node
        # Node(
        #      package='path_tracking',
        #      executable='path_control',
        #      output='screen',
        #      parameters=[params_file],
        #      shell=True,
        #     ),

        # path control and person following
        Node(
             package='path_tracking',
             executable='path_control_person_tracking',
             output='screen',
             parameters=[params_file],
             shell=True,
            ),

        # Trajectory visualization node
        Node(
             package='path_tracking',
             executable='trajectory_visualizer',
             output='screen',
             shell=True,
            ),
    ])
