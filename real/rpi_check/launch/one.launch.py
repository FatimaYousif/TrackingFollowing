from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Get the current user's home directory
    home_dir = os.path.expanduser('~')
    workspace_dir = os.path.join(home_dir, 'new_ws')
    
    # Set up environment variables for the virtual environment and workspace
    env = dict(os.environ)
    env['PATH'] = f"{workspace_dir}/venv/bin:{env['PATH']}"
    
    # Source the virtual environment and workspace setup
    setup_commands = [
        f"cd {workspace_dir}",
        "source venv/bin/activate",
        "source install/setup.bash"
    ]
    
    launch_description = LaunchDescription()
    
    # 1. MicroXRCEAgent (requires sudo)
    microxrce_agent = ExecuteProcess(
        cmd=[
            'sudo', 'MicroXRCEAgent', 'serial',
            '--dev', '/dev/ttyUSB0',
            '-b', '921600'
        ],
        output='screen',
        shell=False
    )

    # 2. Camera node
    camera_node = Node(
        package='camera_ros',
        executable='camera_node',
        output='screen',
        shell=True,
        parameters=[
            {
                'width': 640,
                'height': 360
            }
        ],
        additional_env={
            'PATH': f"{workspace_dir}/venv/bin:{os.environ.get('PATH', '')}",
        }
    )
    
    # 3. Detection node (from your workspace)
    detection_node = Node(
        package='rpi_check',
        executable='detection_node',
        output='screen',
        cwd=workspace_dir,
        shell=True,
        additional_env={
            'PATH': f"{workspace_dir}/venv/bin:{os.environ.get('PATH', '')}",
        }
    )
    
    # 4. Simple Python script (alternative to detection_node)
    simple_script = ExecuteProcess(
        cmd=[
            os.path.join(workspace_dir, 'src/rpi_check/rpi_check/simple.py')
        ],
        output='screen',
        shell=True,
        cwd=workspace_dir,
        additional_env={
            'PATH': f"{workspace_dir}/venv/bin:{os.environ.get('PATH', '')}",
        }
    )
    
    # Add all components to launch description
    # launch_description.add_action(microxrce_agent)
    launch_description.add_action(camera_node)
    launch_description.add_action(detection_node)
    # Uncomment the line below if you want to run simple.py instead of detection_node
    # launch_description.add_action(simple_script)
    
    return launch_description