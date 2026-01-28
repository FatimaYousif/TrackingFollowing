<!DOCTYPE html>
<html>
<body>

<h1>Target Tracking and Following System</h1>

<h2>Overview</h2>
<p>This repository contains the real-world implementation for target tracking and following tasks. The packages are already deployed on the RPi5 of the x500_2 system.</p>

<h2>Requirements</h2>
<ol>
    <li>Code deployed on RPi5</li>
    <li>Proper physical connection (RPi5 connected to PX4)</li>
</ol>

<h2>Running on RPi5</h2>

<h3>1. Start MicroXRCE Agent</h3>
<pre><code>sudo MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600</code></pre>

<h3>2. Navigate to Desktop and Launch Detections</h3>
<pre><code>cd Desktop
./launch_detections.sh</code></pre>

<h3>3. Run Main Tracking Node</h3>
<pre><code>ros2 launch tracking_cpp follow_target.launch.py</code></pre>

<div class="note">
<p><strong>Note:</strong></p>
<ol>
    <li>In the above launch file <code>launch/follow_target.launch.py</code>, you can change the node filename in the LaunchDescription</li>
    <li>Modify relevant parameters in <code>config/follow_target_params.yaml</code></li>
</ol>
</div>

<h3>4. (Optional) Record Detection ROS2 Bags</h3>
<pre><code>ros2 launch tracking_cpp record.launch.py</code></pre>

<div class="note">
<p><strong>Note:</strong> PX4 logs (UAV related data) can be saved through QGroundControl.</p>
</div>

<h2>Running on Personal Computer</h2>

<ol>
    <li>Connect to X500_2 hotspot</li>
    <li>Ensure same ROS_DOMAIN_ID (24) to see RPi topics</li>
    <li>Launch rqt/rviz to visualize required topics</li>
</ol>

</body>
</html>
