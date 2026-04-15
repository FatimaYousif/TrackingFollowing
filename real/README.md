<!DOCTYPE html>
<html>
<body>

<h1>GRASP Path Planning and Target Tracking and Following Integration System</h1>

<h2>Overview</h2>
<p>
This repository contains the real-world implementation for GRASP path planner and target tracking and following integration.
The packages are deployed on the onboard Jetson system of the x500_5 UAV equipped with FLIR BOSON thermal and RGB cameras mounted on the GREMSY gimbal.
</p>

<h2>System Configuration</h2>
<ul>
    <li>Platform: x500_5</li>
    <li>Onboard Computer: NVIDIA Jetson</li>
    <li>JetPack Version: 6.2.1</li>
    <li>CUDA Version: 12.6</li>
    <li>Camera: FLIR BOSON thermal and RGB cameras mounted on GREMSY gimbal.</li>
</ul>

<h2>Requirements</h2>
<ol>
    <li>Code deployed on Jetson onboard computer</li>
    <li>Proper physical connection between Jetson and PX4</li>
    <li> FLIR BOSON thermal and RGB cameras connected and detected</li>
</ol>

<h2>Running on Jetson (Onboard Computer)</h2>

<h3>1. Start RGB camera </h3>
<pre><code>ros2 run usb_cam usb_cam_node_exe --ros-args --params-file /home/x500/security_ws/src/usb_cam/config/params_1.yaml</code></pre>

<h3>2. Start Detection and Tracking (Ultralytics)</h3>
<pre><code>ros2 launch ultralytics_ros tracker.launch.xml debug:=false</code></pre>

<h3>3. Run the Launch file to Launch all GRASP related Nodes</h3>
<pre><code>ros2 launch path_tracking launch_grasp.launch.py</code></pre>

<h3>4. Run the service to plan a path</h3>
<pre><code>ros2 service call plan std_srvs/srv/Trigger {}</code></pre>


<h2>Running on Personal Computer</h2>

<ol>
    <li>Connect to x500_5 network/hotspot</li>
    <li>Ensure same ROS_DOMAIN_ID as Jetson</li>
    <li>Launch rqt or rviz to visualize required topics</li>
</ol>

<div class="note">
<p><strong>Note:</strong> PX4 logs (UAV related data) can be saved through QGroundControl.</p>
</div>

</body>
</html>
