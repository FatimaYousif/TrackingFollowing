cmake_minimum_required(VERSION 3.8)
project(tracking_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ultralytics_ros REQUIRED)

# Generate interfaces first - MUST be before add_executable
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Plots.msg"
  DEPENDENCIES std_msgs
)

# Add include directory for generated headers
include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# Export dependencies
ament_export_dependencies(
  rosidl_default_runtime
  std_msgs
  rclcpp
  vision_msgs
  px4_ros2_cpp
  px4_msgs
  ultralytics_ros
)

ament_export_include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# add_executable(following src/following.cpp)
# ament_target_dependencies(
#   following
#   rclcpp
#   vision_msgs
#   px4_ros2_cpp
#   px4_msgs
# )
# target_include_directories(following PRIVATE 
#   ${EIGEN3_INCLUDE_DIRS}
#   ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
# )
# install(TARGETS following DESTINATION lib/${PROJECT_NAME})

# add_executable(search_only src/search_only.cpp)
# ament_target_dependencies(
#   search_only
#   rclcpp
#   vision_msgs
#   px4_ros2_cpp
#   px4_msgs
# )
# target_include_directories(search_only PRIVATE 
#   ${EIGEN3_INCLUDE_DIRS}
#   ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
# )
# install(TARGETS search_only DESTINATION lib/${PROJECT_NAME})

add_executable(real1 src/real1.cpp)
ament_target_dependencies(
  real1
  rclcpp
  vision_msgs
  px4_ros2_cpp
  px4_msgs
  ultralytics_ros
)
target_include_directories(real1 PRIVATE 
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
install(TARGETS real1 DESTINATION lib/${PROJECT_NAME})

add_executable(real2 src/real2.cpp)
ament_target_dependencies(
  real2
  rclcpp
  vision_msgs
  px4_ros2_cpp
  px4_msgs
  ultralytics_ros
)
# Link against the generated interfaces
rosidl_target_interfaces(real2 ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_include_directories(real2 PRIVATE 
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
install(TARGETS real2 DESTINATION lib/${PROJECT_NAME})




add_executable(offboard_rgb src/offboard_rgb.cpp)
ament_target_dependencies(
  offboard_rgb
  rclcpp
  vision_msgs
  px4_ros2_cpp
  px4_msgs
  ultralytics_ros
)
# Link against the generated interfaces
rosidl_target_interfaces(offboard_rgb ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_include_directories(offboard_rgb PRIVATE 
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
install(TARGETS offboard_rgb DESTINATION lib/${PROJECT_NAME})


add_executable(offboard_thermal src/offboard_thermal.cpp)
ament_target_dependencies(
  offboard_thermal
  rclcpp
  vision_msgs
  px4_ros2_cpp
  px4_msgs
  ultralytics_ros
)
# Link against the generated interfaces
rosidl_target_interfaces(offboard_thermal ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_include_directories(offboard_thermal PRIVATE 
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
install(TARGETS offboard_thermal DESTINATION lib/${PROJECT_NAME})








add_executable(real2_plots src/real2_plots.cpp)
ament_target_dependencies(
  real2_plots
  rclcpp
  vision_msgs
  px4_ros2_cpp
  px4_msgs
  ultralytics_ros
)
# Link against the generated interfaces
rosidl_target_interfaces(real2_plots ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_include_directories(real2_plots PRIVATE 
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
install(TARGETS real2_plots DESTINATION lib/${PROJECT_NAME})



add_executable(mot src/mot.cpp)
ament_target_dependencies(
  mot
  rclcpp
  vision_msgs
  px4_ros2_cpp
  px4_msgs
  ultralytics_ros
)
# Link against the generated interfaces
rosidl_target_interfaces(mot ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_include_directories(mot PRIVATE 
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
install(TARGETS mot DESTINATION lib/${PROJECT_NAME})




# Install Python launch and config files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
