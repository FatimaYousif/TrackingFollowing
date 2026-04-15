cmake_minimum_required(VERSION 3.8)
project(path_tracking)

#####################
# Find dependencies #
#####################

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(px4_ros_com REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(yolo_msgs REQUIRED)


include_directories(include)

#################
# Setup targets #
#################

add_executable(path_control src/path_control.cpp src/pure_pursuit_controller.cpp)
ament_target_dependencies(path_control rclcpp px4_msgs geometry_msgs nav_msgs tf2_geometry_msgs
                          Eigen3 px4_ros_com rclcpp std_srvs sensor_msgs yolo_msgs)

add_executable(trajectory_visualizer src/trajectory_visualizer.cpp)
ament_target_dependencies(trajectory_visualizer
  rclcpp
  nav_msgs
  visualization_msgs
)

ament_export_dependencies(ament_cmake rclcpp rosidl_default_runtime eigen3_cmake_module Eigen3
                          px4_msgs geometry_msgs sensor_msgs Fields2Cover nav_msgs
                          tf2_geometry_msgs px4_ros_com yolo_msgs) 

ament_export_include_directories(include)

############
# Install ##
############

# Install path control node
install(
  TARGETS path_control 
  DESTINATION lib/${PROJECT_NAME})

# Install trajectory visualizer
install(
  TARGETS trajectory_visualizer 
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}/)

# Install config files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
