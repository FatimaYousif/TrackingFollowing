from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():
    delay = 2.0  # seconds between each launch

    micro_xrce = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
        name="micro_xrce_agent",
        output="screen",
    )

    kml_parsing = TimerAction(
        period=delay * 1,
        actions=[ExecuteProcess(
            cmd=["ros2", "launch", "kml_ros2_parsing", "polygon_parser.launch.py"],
            name="kml_ros2_parsing",
            output="screen",
        )],
    )

    waypoint_publisher = TimerAction(
        period=delay * 2,
        actions=[ExecuteProcess(
            cmd=["ros2", "launch", "waypoint_publisher", "waypoint_publisher.launch.py"],
            name="waypoint_publisher",
            output="screen",
        )],
    )

    grasp_planner = TimerAction(
        period=delay * 3,
        actions=[ExecuteProcess(
            cmd=["ros2", "launch", "grasp_path_planner", "grasp_path_planner.launch.py"],
            name="grasp_path_planner",
            output="screen",
        )],
    )

    geo_to_local = TimerAction(
        period=delay * 4,
        actions=[ExecuteProcess(
            cmd=["ros2", "launch", "geo_to_local_path", "geo_to_local_path_px4.launch.py"],
            name="geo_to_local_path",
            output="screen",
        )],
    )

    path_tracking = TimerAction(
        period=delay * 5,
        actions=[ExecuteProcess(
            cmd=["ros2", "launch", "path_tracking", "path_tracking.launch.py"],
            name="path_tracking",
            output="screen",
        )],
    )

    return LaunchDescription([
        micro_xrce,
        kml_parsing,
        waypoint_publisher,
        grasp_planner,
        geo_to_local,
        path_tracking,
    ])