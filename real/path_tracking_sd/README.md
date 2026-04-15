# Path_tracking #
This ROS 2 package provides a path tracking and control framework for autonomous path tracking. It enables smooth motion along predefined paths using pure pursuit control.

## Features ##
* Subscribes to path on topic '/navsat_utm_path'
* Publishes trajectory setpoints for downstream PX4 controllers on '/fmu/in/trajectory_setpoint'
* Implements pure pursuit controller for x/y tracking and position control for the height
* Configurable parameters for velocity, takeoff height, and acceleration
* Built-in state machine for mission flow management:
    - PREFLIGHT: Initialize parameters and wait for odometry
    - IDLE: Wait for path and wait for arming
    - TAKEOFF: Take off to target height using a controlled speed
    - CLIMB_TO_FLIGHT_HEIGHT: Climb to flight height using a controlled speed
    - MISSION: Track the path using pure pursuit control
    - HOVER: Hover at a  specific location
    - LIMBO: Silence offboard control and do nothing. Pilot has full control

## Dependencies ##
* X500 Gazebo simulation and it's dependecies: https://docs.px4.io/main/en/ros2/user_guide
* 'px4_msgs'
* 'px4_ros_com'

## Installation ##
        cd ~/ros2_ws/src
        git clone git@bitbucket.org:mechatronica/path_tracking.git
        cd ..
        colcon build --packages-select path_tracking --symlink-install
        source install/setup.bash


## Usage ##
### Simulation ###
* Start QGroundControl
* Navigate to your PX-Autopilot folder and make the sitl for gz_x500
* Open five terminals and navigate to your ros2 workspace + source the install.bash
* Terminal 1: ros2 launch f2c_ros2_planner service.launch.py
* Terminal 2: ros2 launch f2c_ros2_planner sarax_client.launch.py 
* Terminal 3: ros2 launch geo_to_local_path geo_to_local_path_px4.launch.py
* Terminal 4: ros2 launch path_tracking path_tracking.launch.py
* Terminal 5: launch rviz2
* Terminal 6: run plotjuggler (optional)

* In rviz2 you can visualize the path and the drone pose.
* Add the topic /aoi_polygon
* Add the topic /f2c_path_with_return for the path in local frame
* Add the topic /navsat_utm_path for the path in global frame
* Add the topic /vehicle_pose
* Add the topic /drone_trajectory_marker to visualize the drone trajectory

* In Plotjuggler you can plot the pure pursuit controller metrics
* Load the plot window layout "data_plotting.xml" which can be found in the path_tracking folder