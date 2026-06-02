#include "path_tracking/aruco.hpp"
#include <cmath>

PathControl::PathControl()
: Node("path_control")
{
  RCLCPP_INFO(get_logger(), "Init Node");

  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

  auto frequency_des = rcl_interfaces::msg::ParameterDescriptor{};
  frequency_des.description = "Timer callback frequency [Hz]";
  declare_parameter("frequency", 100, frequency_des);
  int frequency = get_parameter("frequency").get_parameter_value().get<int>();

  declare_and_get_parameters();

  // Publishers
  offboard_control_publisher_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
    "/fmu/in/offboard_control_mode", 10);
  trajectory_setpoint_publisher_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    "/fmu/in/trajectory_setpoint", 10);
  vehicle_command_publisher_ = create_publisher<px4_msgs::msg::VehicleCommand>(
    "/fmu/in/vehicle_command", 10);

  // Subscribers
  vehicle_odometry_subscriber_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
    "/fmu/out/vehicle_odometry", qos,
    std::bind(&PathControl::vehicle_odometry_callback, this, std::placeholders::_1));
  vehicle_control_mode_subscriber_ = create_subscription<px4_msgs::msg::VehicleControlMode>(
    "/fmu/out/vehicle_control_mode", qos,
    std::bind(&PathControl::vehicle_control_mode_callback, this, std::placeholders::_1));
  aruco_subscriber_ = create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
    "/aruco_markers",
    rclcpp::QoS(10).best_effort(),
    std::bind(&PathControl::aruco_callback, this, std::placeholders::_1));

  timer_ = create_wall_timer(
    std::chrono::milliseconds(1000 / frequency),
    std::bind(&PathControl::timer_callback, this));
}

// ─── Parameters ───────────────────────────────────────────────────────────────

void PathControl::declare_and_get_parameters()
{
  RCLCPP_INFO(get_logger(), "Loading ROS params ...");

  declare_parameter<float>("take_off_height",           0.0);
  declare_parameter<float>("take_off_velocity",         0.0);
  declare_parameter<double>("state_transition_tolerance", 0.0);

  take_off_height_    = get_parameter("take_off_height").as_double();
  take_off_velocity_  = get_parameter("take_off_velocity").as_double();
  position_tolerance_ = get_parameter("state_transition_tolerance").as_double();

  declare_parameter<double>("aruco.kp_xy",                       0.003);
  declare_parameter<double>("aruco.kd_xy",                       0.0005);
  declare_parameter<double>("aruco.max_vel_xy",                  0.5);
  declare_parameter<int>   ("aruco.deadband_px",                 10);
  declare_parameter<int>   ("aruco.centered_frames_threshold",   30);

  aruco_kp_xy_                  = get_parameter("aruco.kp_xy").as_double();
  aruco_kd_xy_                  = get_parameter("aruco.kd_xy").as_double();
  aruco_max_vel_xy_             = get_parameter("aruco.max_vel_xy").as_double();
  aruco_deadband_px_            = get_parameter("aruco.deadband_px").as_int();
  aruco_centered_frames_thresh_ = get_parameter("aruco.centered_frames_threshold").as_int();

  RCLCPP_INFO(get_logger(), "take_off_height: %f",     take_off_height_);
  RCLCPP_INFO(get_logger(), "take_off_velocity: %f",   take_off_velocity_);
  RCLCPP_INFO(get_logger(), "aruco.kp_xy: %f",         aruco_kp_xy_);
  RCLCPP_INFO(get_logger(), "aruco.kd_xy: %f",         aruco_kd_xy_);
  RCLCPP_INFO(get_logger(), "aruco.max_vel_xy: %f",    aruco_max_vel_xy_);
  RCLCPP_INFO(get_logger(), "aruco.deadband_px: %d",   aruco_deadband_px_);
}

// ─── Subscriber callbacks ─────────────────────────────────────────────────────

void PathControl::vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg)
{
  vehicle_pose_px4_.pose.position.x = static_cast<double>(msg->position[0]);
  vehicle_pose_px4_.pose.position.y = static_cast<double>(msg->position[1]);
  vehicle_pose_px4_.pose.position.z = static_cast<double>(msg->position[2]);

  current_ned_z_ = static_cast<float>(msg->position[2]);

  tf2::Quaternion q_tf;
  q_tf.setW(msg->q[0]); q_tf.setX(msg->q[1]);
  q_tf.setY(msg->q[2]); q_tf.setZ(msg->q[3]);
  double roll, pitch, yaw_ned;
  tf2::Matrix3x3(q_tf).getRPY(roll, pitch, yaw_ned);
  drone_yaw_ned_ = static_cast<float>(yaw_ned);

  flag_vehicle_odometry_ = true;
}

void PathControl::vehicle_control_mode_callback(
  const px4_msgs::msg::VehicleControlMode::UniquePtr msg)
{
  vehicle_status_ = *msg;
}

// ─── ArUco callback ───────────────────────────────────────────────────────────

void PathControl::aruco_callback(
  const ros2_aruco_interfaces::msg::ArucoMarkers::UniquePtr msg)
{
  if (msg->marker_ids.empty()) {
    has_aruco_ = false;
    aruco_no_det_ctr_++;
    return;
  }

  has_aruco_        = true;
  aruco_no_det_ctr_ = 0;
  aruco_marker_id_  = msg->marker_ids[0];

  const auto & pose = msg->poses[0];

  // Project camera-frame position to pixel error relative to image centre.
  // Camera frame: +x right, +y down, +z forward (optical convention).
  constexpr double half_w = 1280.0 / 2.0;
  constexpr double half_h =  720.0 / 2.0;

  aruco_err_x_px_ = (pose.position.x / pose.position.z) * half_w;
  aruco_err_y_px_ = (pose.position.y / pose.position.z) * half_h;

  // Deadband
  if (std::abs(aruco_err_x_px_) < aruco_deadband_px_) { aruco_err_x_px_ = 0.0; }
  if (std::abs(aruco_err_y_px_) < aruco_deadband_px_) { aruco_err_y_px_ = 0.0; }

  // PD control
  constexpr double dt = 1.0 / 100.0;
  double deriv_x = (aruco_err_x_px_ - aruco_prev_err_x_) / dt;
  double deriv_y = (aruco_err_y_px_ - aruco_prev_err_y_) / dt;
  aruco_prev_err_x_ = aruco_err_x_px_;
  aruco_prev_err_y_ = aruco_err_y_px_;

  auto clamp = [](double v, double lim) { return std::max(-lim, std::min(lim, v)); };
  double raw_vx = clamp(aruco_kp_xy_ * aruco_err_x_px_ + aruco_kd_xy_ * deriv_x, aruco_max_vel_xy_);
  double raw_vy = clamp(aruco_kp_xy_ * aruco_err_y_px_ + aruco_kd_xy_ * deriv_y, aruco_max_vel_xy_);

  // Camera +x (right) → NED +y (right),  Camera +y (down) → NED -x (backward)
  aruco_vel_ned_x_ = static_cast<float>(-raw_vy);
  aruco_vel_ned_y_ = static_cast<float>(-raw_vx);

  if (aruco_err_x_px_ == 0.0 && aruco_err_y_px_ == 0.0) {
    aruco_centered_frames_++;
  } else {
    aruco_centered_frames_ = 0;
  }

  RCLCPP_INFO(get_logger(),
    "[ARUCO] id:%d  err=(%.1f, %.1f) px  cmd_ned=(%.3f, %.3f) m/s  centred:%d/%d",
    aruco_marker_id_, aruco_err_x_px_, aruco_err_y_px_,
    aruco_vel_ned_x_, aruco_vel_ned_y_,
    aruco_centered_frames_, aruco_centered_frames_thresh_);
}

// ─── Publishers ───────────────────────────────────────────────────────────────

void PathControl::publish_offboard_control_mode()
{
  px4_msgs::msg::OffboardControlMode msg{};
  msg.position  = true;
  msg.velocity  = true;
  msg.timestamp = get_clock()->now().nanoseconds() / 1000;
  offboard_control_publisher_->publish(msg);
}

void PathControl::publish_trajectory_setpoint(
  std::array<float, 3UL> position, float yaw,
  std::array<float, 3UL> velocity)
{
  px4_msgs::msg::TrajectorySetpoint msg{};
  msg.position  = position;
  msg.velocity  = velocity;
  msg.yaw       = yaw;
  msg.timestamp = get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher_->publish(msg);
}

void PathControl::publish_trajectory_setpoint_with_yawrate(
  std::array<float, 3UL> position,
  std::array<float, 3UL> velocity,
  float yaw_rate)
{
  px4_msgs::msg::TrajectorySetpoint msg{};
  msg.position  = position;
  msg.velocity  = velocity;
  msg.yaw       = NAN;
  msg.yawspeed  = yaw_rate;
  msg.timestamp = get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher_->publish(msg);
}

void PathControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
  px4_msgs::msg::VehicleCommand msg{};
  msg.command          = command;
  msg.param1           = param1;
  msg.param2           = param2;
  msg.target_system    = 1;
  msg.target_component = 1;
  msg.source_system    = 1;
  msg.source_component = 1;
  msg.from_external    = true;
  msg.timestamp        = get_clock()->now().nanoseconds() / 1000;
  vehicle_command_publisher_->publish(msg);
}

// ─── Utility ─────────────────────────────────────────────────────────────────

double PathControl::euclidean_distance(
  const geometry_msgs::msg::Point v1,
  const geometry_msgs::msg::Point v2)
{
  return std::sqrt(
    (v2.x - v1.x) * (v2.x - v1.x) +
    (v2.y - v1.y) * (v2.y - v1.y) +
    (v2.z - v1.z) * (v2.z - v1.z));
}

bool PathControl::reached_setpoint(
  const geometry_msgs::msg::Point v1,
  const geometry_msgs::msg::Point v2,
  double tolerance)
{
  return euclidean_distance(v1, v2) <= tolerance;
}

void PathControl::capture_hold_position()
{
  if (!hold_position_captured_) {
    hold_position_x_        = static_cast<float>(vehicle_pose_px4_.pose.position.x);
    hold_position_y_        = static_cast<float>(vehicle_pose_px4_.pose.position.y);
    hold_altitude_          = current_ned_z_;
    hold_position_captured_ = true;
    RCLCPP_INFO(get_logger(),
      "[ARUCO] Hold position captured – NED (%.2f, %.2f, %.2f)",
      hold_position_x_, hold_position_y_, hold_altitude_);
  }
}

void PathControl::set_take_off_waypoint()
{
  if (!flag_take_off_position_) {
    take_off_waypoint_.x = vehicle_pose_px4_.pose.position.x;
    take_off_waypoint_.y = vehicle_pose_px4_.pose.position.y;
    take_off_waypoint_.z = vehicle_pose_px4_.pose.position.z - take_off_height_; // NED: up = negative z
    flag_take_off_position_ = true;
    RCLCPP_INFO(get_logger(), "Take-off waypoint (NED): x=%.2f y=%.2f z=%.2f",
      take_off_waypoint_.x, take_off_waypoint_.y, take_off_waypoint_.z);
  }
}

void PathControl::check_pilot_state_switch()
{
  if (!vehicle_status_.flag_control_offboard_enabled && !pilot_takeover_) {
    pilot_takeover_ = true;
    transitionToState(State::LIMBO);
    RCLCPP_WARN(get_logger(), "Pilot takeover – entering LIMBO");
  }
}

// ─── State machine ────────────────────────────────────────────────────────────

static const char * stateToString(State state)
{
  switch (state) {
    case State::PREFLIGHT:    return "PREFLIGHT";
    case State::IDLE:         return "IDLE";
    case State::TAKEOFF:      return "TAKEOFF";
    case State::ARUCO_CENTER: return "ARUCO_CENTER";
    case State::LIMBO:        return "LIMBO";
    default:                  return "LIMBO";
  }
}

void PathControl::transitionToState(State new_state)
{
  current_state_          = new_state;
  flag_timer_done_        = false;
  has_executed_           = false;
  hold_position_captured_ = false;
  aruco_prev_err_x_       = 0.0;
  aruco_prev_err_y_       = 0.0;
  aruco_centered_frames_  = 0;
  RCLCPP_INFO(get_logger(), "→ %s", stateToString(new_state));
}

void PathControl::timer_callback()
{
  switch (current_state_) {
    case State::PREFLIGHT:    handle_preflight_state();    break;
    case State::IDLE:         handle_idle_state();         break;
    case State::TAKEOFF:      handle_takeoff_state();      break;
    case State::ARUCO_CENTER: handle_aruco_center_state(); break;
    case State::LIMBO:        handle_limbo_state();        break;
    default:                  handle_limbo_state();        break;
  }
}

// ─── State handlers ───────────────────────────────────────────────────────────

void PathControl::handle_preflight_state()
{
  if (flag_vehicle_odometry_) {
    set_take_off_waypoint();
    transitionToState(State::IDLE);
  }
}

void PathControl::handle_idle_state()
{
  publish_offboard_control_mode();
  publish_trajectory_setpoint(
    {static_cast<float>(take_off_waypoint_.x),
     static_cast<float>(take_off_waypoint_.y),
     static_cast<float>(take_off_waypoint_.z)},
    drone_yaw_ned_);

  if (vehicle_status_.flag_armed) {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    if (vehicle_status_.flag_control_offboard_enabled) {
      transitionToState(State::TAKEOFF);
    }
  }
}

void PathControl::handle_takeoff_state()
{
  check_pilot_state_switch();
  publish_offboard_control_mode();

  publish_trajectory_setpoint(
    {static_cast<float>(take_off_waypoint_.x),
     static_cast<float>(take_off_waypoint_.y),
     static_cast<float>(take_off_waypoint_.z)},
    drone_yaw_ned_,
    {0.0f, 0.0f, static_cast<float>(take_off_velocity_)});

  if (reached_setpoint(take_off_waypoint_, vehicle_pose_px4_.pose.position, position_tolerance_)) {
    transitionToState(State::ARUCO_CENTER);
  }
}

void PathControl::handle_aruco_center_state()
{
  check_pilot_state_switch();
  publish_offboard_control_mode();
  capture_hold_position();

  if (!has_aruco_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "[ARUCO_CENTER] Marker lost – holding position");
    publish_trajectory_setpoint_with_yawrate(
      {hold_position_x_, hold_position_y_, hold_altitude_},
      {0.0f, 0.0f, 0.0f},
      0.0f);
    return;
  }

  // Drive x/y with PD velocity; hold altitude and yaw
  publish_trajectory_setpoint_with_yawrate(
    {NAN, NAN, hold_altitude_},
    {aruco_vel_ned_x_, aruco_vel_ned_y_, 0.0f},
    0.0f);
}

void PathControl::handle_limbo_state()
{
  // Silently hold – pilot has control
}

// ─── Main ─────────────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathControl>());
  rclcpp::shutdown();
  return 0;
}