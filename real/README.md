<!DOCTYPE html>
<html>
<body>

<h1>Target Tracking and Following System</h1>

<h2>Overview</h2>
<p>
This repository contains the real-world implementation for target tracking and following tasks.
The packages are deployed on the onboard Jetson system of the x500_5 UAV equipped with a ZED2 camera.
</p>

<h2>System Configuration</h2>
<ul>
    <li>Platform: x500_5</li>
    <li>Onboard Computer: NVIDIA Jetson</li>
    <li>JetPack Version: 6.2.1</li>
    <li>CUDA Version: 12.6</li>
    <li>Camera: ZED2</li>
</ul>

<h2>Requirements</h2>
<ol>
    <li>Code deployed on Jetson onboard computer</li>
    <li>Proper physical connection between Jetson and PX4</li>
    <li>ZED2 camera connected and detected</li>
</ol>

<h2>Running on Jetson (Onboard Computer)</h2>

<h3>1. Start MicroXRCE Agent</h3>
<pre><code>MicroXRCEAgent udp4 -p 8888</code></pre>

<h3>2. Launch ZED2 Camera</h3>
<pre><code>ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2</code></pre>

<h3>3. Start Detection and Tracking (Ultralytics)</h3>
<pre><code>ros2 launch ultralytics_ros tracker.launch.xml debug:=false</code></pre>

<h3>4. Run Main Tracking Node</h3>
<pre><code>ros2 launch tracking_cpp real1.launch.py</code></pre>

<p>OR</p>

<pre><code>ros2 launch tracking_cpp real2.launch.py</code></pre>

<div class="note">
<p><strong>Note:</strong></p>
<ol>
    <li>Choose <code>real1.launch.py</code> if you want to test the searching the target test only or <code>real2.launch.py</code> for testing the complete tracking and following pipeline</li>
    <li>Modify relevant parameters inside your configuration YAML files if needed.</li>
</ol>
</div>

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
