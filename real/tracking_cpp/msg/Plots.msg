std_msgs/Header header

# UAV pose
float32 uav_x
float32 uav_y
float32 uav_z
float32 uav_yaw

# UAV actual velocity
float32 actual_vx
float32 actual_vy
float32 actual_vz
float32 actual_yaw_rate

# Commanded velocity
float32 cmd_vx
float32 cmd_vy
float32 cmd_vz
float32 cmd_yaw_rate

# Target
float32 target_px
float32 target_py
float32 target_size

# Pixel error
float32 pixel_error_x
float32 pixel_error_y
float32 size_error

# Mode + detection
string mode
bool has_detection
int32 no_detection_counter

# Altitude hold
# float32 hold_altitude
