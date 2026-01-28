This folder contains the real world code for the progress on the target tracking and following task. These packages are already uploaded on the ``RPi5`` on the ``x500_2`` so you just need the commands below to run the relevant node:

 ## Requirements:

1. Have the code on RPi5.
2. Proper physical connection (RPi5 connected to PX4).

## Running:
<!-- 
1. sudo MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
--> 

<ol>
 <!-- 
  <li> ros2 run camera_ros camera_node --ros-args -p width:=640 -p height:=360 </li>
  <li> pi@pi-desktop:~$ cd new_ws/ </li>
  <li> pi@pi-desktop:~/new_ws$ source venv/bin/activate </li>
  <li> (venv) pi@pi-desktop:~/new_ws$ source install/setup.bash </li>
  <li> (venv) pi@pi-desktop:~/new_ws/src/rpi_check/rpi_check$ ./simple.py </li>
  <li> TO BE CONTINUED ............... WITH RUNNING THE NODE ..................</li>
  -->
 
 <li> sudo MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600 </li>
 <li> cd Desktop </li>
 <li> ./launch_detections.sh </li>

 For running the nodes:
 <li> ros2 launch tracking_cpp follow_target.launch.py</li>
 
 Note:
 1. In this launch file you can change the file name of the node you are running in LaunchDescription.
 2. In the config/follow_target_params.yaml, modify the relevant parameters.
 
 (Optional) To record detection ROS2 bags:
 <li> ros2 launch tracking_cpp record.launch.py</li>
 
 Note: PX4 logs (related to UAV data) can be saved through QGC.


 In the companion computer:
 1. Connect to X500_2 hotspot.
 2. with same ROS_DOMAIN_ID (24) you should be able to see the topics running in RPi.
 3. Launch the rqt   -->    Visualize the required topics.

</ol>

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
