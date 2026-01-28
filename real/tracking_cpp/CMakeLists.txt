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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


add_executable(following src/following.cpp)
ament_target_dependencies(

  following
  rclcpp
  vision_msgs
  px4_ros2_cpp
  px4_msgs
)
target_include_directories(following PRIVATE ${EIGEN3_INCLUDE_DIRS})
install(TARGETS following DESTINATION lib/${PROJECT_NAME})

add_executable(search_only src/search_only.cpp)
ament_target_dependencies(
  search_only
  rclcpp
  vision_msgs
  # yolo_msgs
  px4_ros2_cpp
  px4_msgs
)
target_include_directories(search_only PRIVATE ${EIGEN3_INCLUDE_DIRS})
install(TARGETS search_only DESTINATION lib/${PROJECT_NAME})


add_executable(real1 src/real1.cpp)
ament_target_dependencies(
  real1
  rclcpp
  vision_msgs
  px4_ros2_cpp
  px4_msgs
)
target_include_directories(real1 PRIVATE ${EIGEN3_INCLUDE_DIRS})
install(TARGETS real1 DESTINATION lib/${PROJECT_NAME})



add_executable(real2 src/real2.cpp)
ament_target_dependencies(
  real2
  rclcpp
  vision_msgs
  px4_ros2_cpp
  px4_msgs
)
target_include_directories(real2 PRIVATE ${EIGEN3_INCLUDE_DIRS})
install(TARGETS real2 DESTINATION lib/${PROJECT_NAME})





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
