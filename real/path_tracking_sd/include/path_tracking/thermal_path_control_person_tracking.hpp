#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_ros_com/frame_transforms.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/home_position.hpp>

#include <std_srvs/srv/trigger.hpp>
// #include <yolo_msgs/msg/detection_array.hpp>
// #include "ultralytics_ros/msg/yolo_result.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "pure_pursuit_controller.hpp"

#include <Eigen/Dense>
#include <mutex>
#include <thread>
#include <array>
#include <std_msgs/msg/float64.hpp>

using namespace std::chrono_literals;

// ─── State machine ─────────────────────────────────────────────

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

// ─── Class ─────────────────────────────────────────────────────

class PathControl : public rclcpp::Node
{
public:
  PathControl();

private:

  // ─── Core ────────────────────────────────────────────────────
  void timer_callback();
  void declare_and_get_parameters();
  void call_service();

  // ─── Subscribers ─────────────────────────────────────────────
  void vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg);
  void vehicle_control_mode_callback(const px4_msgs::msg::VehicleControlMode::UniquePtr msg);
  void path_callback(const nav_msgs::msg::Path msg);
  void home_pos_callback(const px4_msgs::msg::HomePosition::UniquePtr msg);
  // void detection_callback(const yolo_msgs::msg::DetectionArray::UniquePtr msg);
  // void detection_callback(const ultralytics_ros::msg::YoloResult::SharedPtr msg);
  void detection_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
   

  // ─── Publishers ──────────────────────────────────────────────
  void publish_offboard_control_mode();

  void publish_trajectory_setpoint(
    std::array<float, 3UL> position,
    float yaw,
    std::array<float, 3UL> velocity = {0.0f, 0.0f, 0.0f});

  void publish_trajectory_setpoint_with_yawrate(
    std::array<float, 3UL> position,
    std::array<float, 3UL> velocity,
    float yaw_rate);

  void publish_vehicle_command(
    uint16_t command,
    float param1 = 0.0, float param2 = 0.0,
    float param3 = 0.0, float param4 = 0.0,
    float param5 = 0.0, float param6 = 0.0, float param7 = 0.0);

  void arm();

  // ─── Utility ─────────────────────────────────────────────────
  void nonBlockingWait(std::chrono::milliseconds duration);
  bool check_drone_startup_position();
  void set_take_off_waypoint();

  bool reached_setpoint(
    const geometry_msgs::msg::Point v1,
    const geometry_msgs::msg::Point v2,
    double tolerance = 1.0);

  double euclidean_distance(
    const geometry_msgs::msg::Point v1,
    const geometry_msgs::msg::Point v2);

  void check_pilot_state_switch();
  void silence_offboard_publishers();
  void capture_hold_position();
  void quaternion_to_yaw(const Eigen::Quaterniond & q, float & yaw);

  // ─── State machine ───────────────────────────────────────────
  void transitionToState(State new_state);
  void stateToSTring(State state);

  void handle_preflight_state();
  void handle_idle_state();
  void handle_takeoff_state();
  void handle_climb_to_flight_height_state();
  void handle_mission_state();
  void handle_hover_state();

  // Follow states
  void handle_following_state();
  void handle_follow_hover_state();
  void handle_follow_search_state();

  void handle_limbo_state();

  // ─── State ───────────────────────────────────────────────────
  State current_state_ = State::PREFLIGHT;

  // ─── Flags ───────────────────────────────────────────────────
  bool flag_timer_done_ = false;
  bool has_executed_ = false;
  bool flag_mission_ = false;
  bool flag_vehicle_odometry_ = false;
  bool flag_take_off_position = false;
  bool pilot_takeover_ = false;
  bool offboard_publisher_silenced_ = false;
  bool stop_offboard_output_ = false;
  bool hold_position_captured_ = false;

  // ─── Parameters ──────────────────────────────────────────────
  double position_tolerance_;
  double drone_yaw_ros_;
  float drone_yaw_ned_;

  double take_off_heading_;
  float take_off_height_;
  float take_off_velocity_;
  float ppc_velocity_;
  float max_acceleration_;

  // ─── Following params ────────────────────────────────────────
  float following_yaw_kp_;
  float following_dist_kp_;
  float following_center_x_;
  float following_center_y_;
  float following_desired_size_;
  float following_search_yaw_rate_;
  int following_hover_threshold_;

  std::string detection_topic_;

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

  // ─── Mission data ────────────────────────────────────────────
  std::array<float, 3UL> hover_position_;
  std::array<float, 3UL> hover_velocity_;

  bool logged_no_path_ = false;
  bool logged_path_received_ = false;
  bool has_replanned = false;

  // ─── ROS data ────────────────────────────────────────────────
  geometry_msgs::msg::Point vehicle_speed_;
  geometry_msgs::msg::Point take_off_waypoint;
  geometry_msgs::msg::Point home_position_ros_;

  geometry_msgs::msg::PoseStamped vehicle_pose_px4_;
  geometry_msgs::msg::PoseStamped vehicle_pose_ros_;

  nav_msgs::msg::Path path_;
  px4_msgs::msg::VehicleControlMode vehicle_status_;
  px4_msgs::msg::HomePosition home_position_;

  // ─── Control ─────────────────────────────────────────────────
  PurePursuitController pure_pursuit_;
  mutable std::mutex mutex_;

  // ─── ROS interfaces ──────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr timer_;
  

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_publisher_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_publisher_;

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr vehicle_control_mode_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::HomePosition>::SharedPtr vehicle_home_pos_subscriber_;
  // rclcpp::Subscription<yolo_msgs::msg::DetectionArray>::SharedPtr detection_subscriber_;
  // rclcpp::Subscription<ultralytics_ros::msg::YoloResult>::SharedPtr detection_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr detection_subscriber_;


  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_mission_planning_client_;
};

