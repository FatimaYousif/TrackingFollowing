#pragma once

#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>

#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>

#include <array>

using namespace std::chrono_literals;

// ─── State machine ─────────────────────────────────────────────

enum class State {
  PREFLIGHT,
  IDLE,
  TAKEOFF,
  ARUCO_CENTER,
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

  // ─── Subscribers ─────────────────────────────────────────────
  void vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg);
  void vehicle_control_mode_callback(const px4_msgs::msg::VehicleControlMode::UniquePtr msg);
  void aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::UniquePtr msg);

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

  void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

  // ─── Utility ─────────────────────────────────────────────────
  void set_take_off_waypoint();
  void capture_hold_position();
  void check_pilot_state_switch();

  bool reached_setpoint(
    const geometry_msgs::msg::Point v1,
    const geometry_msgs::msg::Point v2,
    double tolerance = 1.0);

  double euclidean_distance(
    const geometry_msgs::msg::Point v1,
    const geometry_msgs::msg::Point v2);

  // ─── State machine ───────────────────────────────────────────
  void transitionToState(State new_state);

  void handle_preflight_state();
  void handle_idle_state();
  void handle_takeoff_state();
  void handle_aruco_center_state();
  void handle_limbo_state();

  // ─── State ───────────────────────────────────────────────────
  State current_state_ = State::PREFLIGHT;

  // ─── Flags ───────────────────────────────────────────────────
  bool flag_timer_done_        = false;
  bool has_executed_           = false;
  bool flag_vehicle_odometry_  = false;
  bool flag_take_off_position_ = false;
  bool pilot_takeover_         = false;
  bool hold_position_captured_ = false;

  // ─── Parameters ──────────────────────────────────────────────
  double position_tolerance_  = 0.0;
  float  drone_yaw_ned_       = 0.0f;
  float  take_off_height_     = 0.0f;
  float  take_off_velocity_   = 0.0f;

  // ─── ArUco parameters ────────────────────────────────────────
  double aruco_kp_xy_                  = 0.003;
  double aruco_kd_xy_                  = 0.0005;
  double aruco_max_vel_xy_             = 0.5;
  int    aruco_deadband_px_            = 10;
  int    aruco_centered_frames_thresh_ = 30;

  // ─── ArUco state ─────────────────────────────────────────────
  bool   has_aruco_         = false;
  int    aruco_marker_id_   = -1;
  int    aruco_no_det_ctr_  = 0;

  double aruco_err_x_px_    = 0.0;
  double aruco_err_y_px_    = 0.0;
  double aruco_prev_err_x_  = 0.0;
  double aruco_prev_err_y_  = 0.0;

  float  aruco_vel_ned_x_   = 0.0f;
  float  aruco_vel_ned_y_   = 0.0f;

  int    aruco_centered_frames_ = 0;

  // ─── Hold / altitude state ───────────────────────────────────
  float hold_position_x_  = 0.0f;
  float hold_position_y_  = 0.0f;
  float hold_altitude_    = 0.0f;
  float current_ned_z_    = 0.0f;

  // ─── ROS data ────────────────────────────────────────────────
  geometry_msgs::msg::Point       take_off_waypoint_;
  geometry_msgs::msg::PoseStamped vehicle_pose_px4_;
  px4_msgs::msg::VehicleControlMode vehicle_status_;

  // ─── ROS interfaces ──────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr  offboard_control_publisher_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr   trajectory_setpoint_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr       vehicle_command_publisher_;

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr      vehicle_odometry_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr   vehicle_control_mode_subscriber_;
  rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_subscriber_;
};