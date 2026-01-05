This folder contains the simulation code for the target tracking and following. Follow the below instructions:

 ## Requirements:
Must have installed:
<ol>
  <li> Micro-XRCE-DDS-Agent </li>
  <li> PX4-Autopilot and x500_depth (or any drone model) </li>
  <li> QGroundControl </li>
  <li> ros_gz_bridge  <a href="#github_rosgz"> [1] </a> </li>
  <li> yolo_ros package <a href="#github_yoloros"> [2] </a> </li>
</ol>

<p id="github_rosgz">
[1] For the installation of ros_gz_bridge please refer to <a href="https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge">this</a> link.
</p>

<p id="github_yoloros">
[2] For the installation of yolo_ros package please refer to <a href="https://github.com/mgonzs13/yolo_ros">this</a> link.
</p>


## Running:
<!-- 
1. sudo MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
--> 

<ol>
  <li> ros2 launch sim one.launch.py </li>
  <li> ros2 launch yolo_bringup yolo.launch.py </li>
  <li> ros2 run tracking_cpp counter </li>
  or
  <li> ros2 run tracking_cpp search_only </li>
</ol>

Note: Check the one.launch.py in ``ros2_tracker > sim > launch > one.launch.py`` for more detailed understanding of why the above mentioned required packages are necessary.

<!-- 
(venv) pi@pi-desktop:~/new_ws$ ros2 run rpi_check detection_node
(or you can make it a ROS2 node in [setup.py](http://setup.py))
-->

<!-- 
4. default.sdf file (in this repo)
5. QGC offboard/takeoff
<5. ros_gz_bridge package: https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_bridge
6. ultralytics_ros package: https://github.com/Alpaca-zip/ultralytics_ros
7. ultralytics_tracker.py file to be replaced in the package. Alongside, a change required in ultralytics_ros>launch>tracker.launch.xml with the param below in pkg=ultralytics_ros <br>
``<param name="classes" value="[0]"/> ``
8. The file refactored_main.py (in this repo)


## Purpose of Each Pkg:

1. default.sdf file to be replaced in folder:  <b> ~/PX4-Autopilot/Tools/simulation/gz/worlds </b>
2. px4_ros_com package: for onboarding <br>
3. ros_gz_bridge package: for publishing gz topics to -> ros2 <br>
6. ultralytics_ros: for publishing detection and tracking results <br>
7. refactored_main.py: main node

## Running:

https://www.notion.so/ROS2-ROS2-1de8042b48ef80d0a244dae87aff3299?source=copy_link
-->
