#pragma once

#include <px4_ros_com/frame_transforms.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/home_position.hpp>
#include "pure_pursuit_controller.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <string>
#include "std_msgs/msg/float32_multi_array.hpp"
#include "ultralytics_ros/msg/yolo_result.hpp"

#include <Eigen/Dense>
#include <mutex>
#include <thread>
#include <array>
#include <std_msgs/msg/float64.hpp>

#include <cmath> 
#include <limits>

using namespace std::chrono_literals;

enum class State {
  PREFLIGHT,
  IDLE,
  TAKEOFF,
  CLIMB_TO_FLIGHT_HEIGHT,
  MISSION,
  FOLLOWING,
  FOLLOW_HOVER,
  FOLLOW_SEARCH,
  HOVER,
  LIMBO
};
class PathControl : public rclcpp::Node
{
public:
  PathControl();

private:
  /**
  * @brief Main control timer loop
  */
  void timer_callback();
  /**
  * @brief Service callback to start mission planning
  */
  void call_service();
  /**
   * @brief Function to load in the parameters from the params.yaml file
   */
  void declare_and_get_parameters();
  /**
  * @brief Vehicle odometry subscriber callback
  *
  */
  void vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg);
  /**
  * @brief Vehicle control mode subscriber callback
  *
  */
  void vehicle_control_mode_callback(const px4_msgs::msg::VehicleControlMode::UniquePtr msg);
  /**
  * @brief Path subscriber callback
  *
  */
  void path_callback(const nav_msgs::msg::Path msg);
  /**
  * @brief Vehicle pose subscriber callback
  *
  */
  void home_pos_callback(const px4_msgs::msg::HomePosition::UniquePtr msg);

  /**
  * @brief This function sends an OffboardControlMode message to PX4 to enable offboard control.
  *
  */
  void publish_offboard_control_mode();
  /**
  * @brief Publish a trajectory set point
  *
  * @param position - IN ROS COORDINATE SYSTEM - (x: North, y: East, z: Down)
  * @param velocity - Velocity vector at the set point position
  * @param yaw      - Desired yaw
  *
  * Reference:
  *    Coordinate frame: https://docs.px4.io/main/en/ros/ros2_comm.html#ros-2-px4-frame-conventions
  */
  void publish_trajectory_setpoint(
    std::array<float, 3UL> position, float yaw, std::array<float,
    3UL> velocity = {0.1, 0.1, 0});
  /**
  * @brief Publish vehicle commands (https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleCommand.msg)
  * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
  * @param param1    Parameter 1, as defined by MAVLink uint16 VEHICLE_CMD enum.
  * @param param2    Parameter 2, as defined by MAVLink uint16 VEHICLE_CMD enum.
  * @param param3    Parameter 3, as defined by MAVLink uint16 VEHICLE_CMD enum.
  * @param param4    Parameter 4, as defined by MAVLink uint16 VEHICLE_CMD enum.
  * @param param5    Parameter 5, as defined by MAVLink uint16 VEHICLE_CMD enum.
  * @param param6    Parameter 6, as defined by MAVLink uint16 VEHICLE_CMD enum.
  * @param param7    Parameter 7, as defined by MAVLink uint16 VEHICLE_CMD enum.
  */
  void publish_vehicle_command(
    uint16_t command, float param1 = 0.0, float param2 = 0.0,
    float param3 = 0.0, float param4 = 0.0,
    float param5 = 0.0, float param6 = 0.0, float param7 = 0.0);

  // State machine functions
  /**
  * @brief Send a command to Arm the drone
  */
  void arm();
  // Mission logic
  /**
  * @brief Non-blocking timer counter thread. This will change the flag: Flag_timer_done = True
  * @param duration Time to sleep in milliseconds
  */
  void nonBlockingWait(std::chrono::milliseconds duration);
  /**
  * @brief Check if drone is at the startup position (Within a threshold)
  *
  * @return (bool) True if drone is at startup position
  */
  bool check_drone_startup_position();
  /**
  * @brief Set take-off waypoint
  *
  * Set take-off waypoint to current drone position
  */
  void set_take_off_waypoint();
  /**
  * @brief Checks if drone is at specified setpoint and within an Euclidean tolerance
  * @param v1 First 3D point
  * @param v2 Second 3D point
  * @param tolerance Acceptable Euclidean tolerance in meters
  * @return (bool) True if inside the Euclidean tolerance
  */
  bool reached_setpoint(
    const geometry_msgs::msg::Point v1, const geometry_msgs::msg::Point v2,
    double tolerance = 1.0);
  /**
  * @brief Calculate 3D Euclidean distance
  * @param v1 First 3D point
  * @param v2 Second 3D point
  * @return Euclidean distance
  */
  double euclidean_distance(const geometry_msgs::msg::Point v1, const geometry_msgs::msg::Point v2);
  /**
   * @brief Check for a manual flight mode override by the pilot.
   *
   * If the drone is no longer in OFFBOARD mode but has switched to AUTO mode
   * (triggered manually), this function transitions the control
   * state machine into the LIMBO state to pause autonomous behavior.
   */
  void check_pilot_state_switch();
  /**
   * @brief Transition to a new state in the state machine.
   *
   * This function is used to change the current state of the state machine
   * to a new state.
   *
   * @param new_state The new state to transition to.
   */
  void transitionToState(State new_state);
  /**
   * @brief Convert the current state to a string representation.
   *
   * This function is used to convert the current state of the state machine
   * to a string representation for logging or debugging purposes.
   *
   * @param state The state to convert to a string.
   * @return A string representation of the state.
   */
  void stateToSTring(State state);

  // State machine handlers
  /**
  * @brief Handles the preflight state
  *
  * The drone will check if it is in the startup position and
  * if the take-off waypoint is set.
  */
  void handle_preflight_state();
  /**
  * @brief Handles the idle state
  *
  * In this state, the drone is waiting for a command to take off.
  * The drone will check if the take-off waypoint is set and
  * if the drone is in the startup position.
  */
  void handle_idle_state();
  /**
  * @brief Handles the take-off state
  * In this state, the drone is taking off and will hover at a specified height.
  * The drone will check if the setpoint has been reached and
  * if the timer has expired.
  */
  void handle_takeoff_state();
  /** @brief Handles the hover state
  * In this state, the drone is hovering at a specified height.
  * The drone will maintain its position and yaw.
  */
  void handle_hover_state();
  /**
  * @brief Handles the climb to flight height state
  * In this state, the drone is climbing to the flight height.
  * The drone will check if the setpoint has been reached.
  */
  void handle_climb_to_flight_height_state();
  /**
  * @brief Handles the mission state
  *
  * In this state, the drone is controlled to follow the mission path.
  */
  void handle_mission_state();
  /**
  * @brief Handles the limbo state
  *
  * In this state, the drone is in a limbo state and will not
  * follow the trajectory setpoint anymore.
  */
  void handle_limbo_state();
  /**  @brief Silence the offboard publishers to stop sending setpoints to PX4.
   * This is necessary when the pilot takes over control of the drone
   * manually and we want to avoid conflicts between offboard control
   * and manual control.
   */
  void silence_offboard_publishers();


  // New functions
  void capture_hold_position();
  void quaternion_to_yaw(const Eigen::Quaterniond & q, float & yaw);
  void publish_trajectory_setpoint_with_yawrate(
    std::array<float, 3UL> position,
    std::array<float, 3UL> velocity,
    float yaw_rate);
  void detection_callback_rgb(const ultralytics_ros::msg::YoloResult::SharedPtr msg);
  void detection_callback_thermal(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void handle_following_state();
  void handle_follow_hover_state();
  void handle_follow_search_state();

  // -------

  // State initialization
  State current_state_ = State::PREFLIGHT;

  // Flags
  bool flag_timer_done_ = false;
  bool has_executed_ = false;
  bool flag_mission_ = false;
  bool flag_vehicle_odometry_ = false;
  bool flag_take_off_position = false;
  bool pilot_takeover_ = false;
  bool offboard_publisher_silenced_ = false;
  bool stop_offboard_output_ = false;
  bool hold_position_captured_ = false;

  // Global variables
  double position_tolerance_;
  double drone_yaw_ros_;
  float drone_yaw_ned_;
  double take_off_heading_;
  float flight_height_;
  float take_off_height_;
  float take_off_velocity_;
  float ppc_velocity_;
  float max_acceleration_;
  std::array<float, 3UL> hover_position_;
  std::array<float, 3UL> hover_velocity_;
  bool logged_no_path_ = false;
  bool logged_path_received_ = false;
  bool has_replanned = false;

  // Security drone Integration
  bool enable_grasp_patrol_ = true;
  bool enable_person_following_ = false;
  
  // Person followings params

  float following_yaw_kp_;
  float following_dist_kp_;
  float following_center_x_;
  float following_center_y_;
  float following_desired_size_;
  float following_search_yaw_rate_;
  int following_hover_threshold_;
  std::string detection_topic_;
  std::string detection_camera_type_;

  // ─── Detection data ──────────────────────────────────────────
  bool has_detection_ = false;
  int no_detection_counter_ = 0;

  float det_bbox_x_;
  float det_bbox_y_;
  float det_bbox_size_;
  std::string det_target_id_;

  // ─── Follow control ──────────────────────────────────────────
  float following_yaw_rate_ = 0.0f;
  float following_velocity_x_ = 0.0f;
  float following_velocity_y_ = 0.0f;

  // ─── Hold state ──────────────────────────────────────────────
  float hold_position_x_ = 0.0f;
  float hold_position_y_ = 0.0f;
  float hold_altitude_ = 0.0f;
  float current_ned_z_ = 0.0f;

  // Create objects
  geometry_msgs::msg::Point vehicle_speed_ = geometry_msgs::msg::Point{};
  geometry_msgs::msg::Point take_off_waypoint = geometry_msgs::msg::Point{};
  geometry_msgs::msg::Point home_position_ros_ = geometry_msgs::msg::Point{};
  geometry_msgs::msg::PoseStamped vehicle_pose_px4_;
  geometry_msgs::msg::PoseStamped vehicle_pose_ros_;
  nav_msgs::msg::Path path_;
  px4_msgs::msg::VehicleControlMode vehicle_status_;
  px4_msgs::msg::HomePosition home_position_;

  // Control logic
  PurePursuitController pure_pursuit_;
  mutable std::mutex mutex_;

  // ROS interfaces
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_publisher_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_publisher_;

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr vehicle_control_mode_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::HomePosition>::SharedPtr vehicle_home_pos_subscriber_;
  rclcpp::Subscription<ultralytics_ros::msg::YoloResult>::SharedPtr detection_subscriber_rgb_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr detection_subscriber_thermal_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_mission_planning_client_;
};
