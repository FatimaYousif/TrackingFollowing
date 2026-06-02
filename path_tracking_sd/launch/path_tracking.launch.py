from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from ament_index_python.packages import get_package_share_directory
import socket
import os

from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    params_file = os.path.join(
    get_package_share_directory('path_tracking'),
    'config',
    'params.yaml'
    )

    # Arguments to overwrite params file
    grasp_arg = DeclareLaunchArgument('enable_grasp_patrol', default_value='true')
    person_following_arg = DeclareLaunchArgument('enable_person_following', default_value='false')
    camera_type_arg = DeclareLaunchArgument('detection_camera_type', default_value='RGB')

    # This checks if the code is running on the RPi with a hostname -> osprey
    #DeclareLaunchArgument(
    #    name='RPi_hostname',
    #    default_value='osprey',
    #    description='Change code to run on RPi'
    #    ),

    path_tracking_node = Node(
             package='path_tracking',
             executable='int',
             output='screen',
             parameters=[
                 params_file,
                {"enable_grasp_patrol": LaunchConfiguration('enable_grasp_patrol')},
                {"enable_person_following": LaunchConfiguration('enable_person_following')},
                {"detection_camera_type": LaunchConfiguration('detection_camera_type')}
                ],
             shell=True,
            )
    
    traj_viz = Node(
             package='path_tracking',
             executable='trajectory_visualizer',
             output='screen',
             shell=True,
            )
    
    return LaunchDescription([
        grasp_arg,
        person_following_arg,
        camera_type_arg,
        path_tracking_node,
        traj_viz
    ])



# ------------------------- for aruco centering only -------------------------------------------
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import ExecuteProcess, DeclareLaunchArgument
# from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
# from ament_index_python.packages import get_package_share_directory
# import socket
# import os

# def generate_launch_description():
#     params_file = os.path.join(
#     get_package_share_directory('path_tracking'),
#     'config',
#     'params.yaml'
#     )
    
#     return LaunchDescription([
#         # This checks if the code is running on the RPi with a hostname -> osprey
#         DeclareLaunchArgument(
#             name='RPi_hostname',
#             default_value='osprey',
#             description='Change code to run on RPi'
#             ),
#         # Control node
#         Node(
#              package='path_tracking',
#              executable='aruco',
#              output='screen',
#              parameters=[params_file],
#              shell=True,
#             ),
#         # Trajectory visualization node
#         Node(
#              package='path_tracking',
#              executable='trajectory_visualizer',
#              output='screen',
#              shell=True,
#             ),
#     ])

