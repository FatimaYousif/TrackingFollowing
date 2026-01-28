#!/bin/bash
cd ~/new_ws
source venv/bin/activate
source install/setup.bash
ros2 launch rpi_check one.launch.py