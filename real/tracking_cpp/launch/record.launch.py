from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import datetime

def generate_launch_description():
    # Timestamped folder name for bag
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_output = f"/tmp/rosbags/record_{timestamp}"

    topics_to_record = [
        '/detections',
        '/camera/image_raw'
    ]

    # Record process
    rosbag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_output] + topics_to_record,
        output='screen'
    )

    return LaunchDescription([
        rosbag_record
    ])