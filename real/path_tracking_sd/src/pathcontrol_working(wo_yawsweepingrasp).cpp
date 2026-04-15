#include "path_tracking/path_control.hpp"
#include <cmath>

PathControl::PathControl()
: Node("path_control")
{
  RCLCPP_INFO_STREAM(get_logger(), "Init Node");
  RCLCPP_INFO_STREAM(get_logger(), "State transitioned to PREFLIGHT");

  // QoS settings
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

  // Parameter descriptor
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
  vehicle_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("/vehicle_pose", 10);

  // Service client
  start_mission_planning_client_ = this->create_client<std_srvs::srv::Trigger>("/plan");
  while (!start_mission_planning_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
  }

  // Subscribers
  vehicle_odometry_subscriber_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
    "/fmu/out/vehicle_odometry", qos,
    std::bind(&PathControl::vehicle_odometry_callback, this, std::placeholders::_1));
  vehicle_control_mode_subscriber_ = create_subscription<px4_msgs::msg::VehicleControlMode>(
    "/fmu/out/vehicle_control_mode", qos,
    std::bind(&PathControl::vehicle_control_mode_callback, this, std::placeholders::_1));
  path_subscriber_ = create_subscription<nav_msgs::msg::Path>(
    "/navsat_utm_path", qos,
    std::bind(&PathControl::path_callback, this, std::placeholders::_1));
  vehicle_home_pos_subscriber_ = create_subscription<px4_msgs::msg::HomePosition>(
    "/fmu/out/home_position", qos,
    std::bind(&PathControl::home_pos_callback, this, std::placeholders::_1));

  // Detection subscriber (follow-target)
  detection_subscriber_ = create_subscription<yolo_msgs::msg::DetectionArray>(
    detection_topic_,
    rclcpp::QoS(10).best_effort(),
    std::bind(&PathControl::detection_callback, this, std::placeholders::_1));

  timer_ = create_wall_timer(
    std::chrono::milliseconds(1000 / frequency),
    std::bind(&PathControl::timer_callback, this));
}

// ─── Parameters ──────────────────────────────────────────────────────────────

void PathControl::declare_and_get_parameters()
{
  RCLCPP_INFO(this->get_logger(), "-------------------------------");
  RCLCPP_INFO(this->get_logger(), "Loading ROS params ...");

  // Path-control params
  this->declare_parameter<float>("take_off_height", 0.0);
  this->declare_parameter<float>("ppc_velocity", 0.0);
  this->declare_parameter<float>("take_off_velocity", 0.0);
  this->declare_parameter<float>("max_acceleration", 0.0);
  this->declare_parameter<double>("state_transition_tolerance", 0.0);

  take_off_height_    = this->get_parameter("take_off_height").as_double();
  take_off_velocity_  = this->get_parameter("take_off_velocity").as_double();
  ppc_velocity_       = this->get_parameter("ppc_velocity").as_double();
  max_acceleration_   = this->get_parameter("max_acceleration").as_double();
  position_tolerance_ = this->get_parameter("state_transition_tolerance").as_double();

  RCLCPP_INFO(this->get_logger(), "take_off_height: %f",            take_off_height_);
  RCLCPP_INFO(this->get_logger(), "ppc_velocity: %f",               ppc_velocity_);
  RCLCPP_INFO(this->get_logger(), "take_off_velocity: %f",          take_off_velocity_);
  RCLCPP_INFO(this->get_logger(), "max_acceleration: %f",           max_acceleration_);
  RCLCPP_INFO(this->get_logger(), "state_transition_tolerance: %f", position_tolerance_);

  // Person-following params
  this->declare_parameter<std::string>("subscribers.detection_topic", "/yolo/tracking");
  this->declare_parameter<int>("detection_loss.hover_frames_threshold", 10);
  this->declare_parameter<double>("control.yaw_kp",            0.5);
  this->declare_parameter<double>("control.distance_kp",       1.5);
  this->declare_parameter<double>("camera.center_x",           960.0);
  this->declare_parameter<double>("camera.center_y",           540.0);
  this->declare_parameter<double>("camera.desired_target_size", 100.0);
  this->declare_parameter<double>("setpoint.search.yaw_rate",  0.3);

  detection_topic_           = this->get_parameter("subscribers.detection_topic").as_string();
  following_hover_threshold_ = this->get_parameter("detection_loss.hover_frames_threshold").as_int();
  following_yaw_kp_          = this->get_parameter("control.yaw_kp").as_double();
  following_dist_kp_         = this->get_parameter("control.distance_kp").as_double();
  following_center_x_        = this->get_parameter("camera.center_x").as_double();
  following_center_y_        = this->get_parameter("camera.center_y").as_double();
  following_desired_size_    = this->get_parameter("camera.desired_target_size").as_double();
  following_search_yaw_rate_ = this->get_parameter("setpoint.search.yaw_rate").as_double();

  RCLCPP_INFO(this->get_logger(), "detection_topic: %s",          detection_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "hover_frames_threshold: %d",   following_hover_threshold_);
  RCLCPP_INFO(this->get_logger(), "-------------------------------");

  pure_pursuit_.setParameters(ppc_velocity_, max_acceleration_);
}

// ─── Subscriber callbacks ─────────────────────────────────────────────────────

void PathControl::vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg)
{
  // Store raw NED pose (PX4 frame)
  vehicle_pose_px4_.header.frame_id    = "/map";
  vehicle_pose_px4_.header.stamp       = this->get_clock()->now();
  vehicle_pose_px4_.pose.position.x    = static_cast<double>(msg->position[0]);  // North
  vehicle_pose_px4_.pose.position.y    = static_cast<double>(msg->position[1]);  // East
  vehicle_pose_px4_.pose.position.z    = static_cast<double>(msg->position[2]);  // Down
  vehicle_pose_px4_.pose.orientation.w = static_cast<double>(msg->q[0]);
  vehicle_pose_px4_.pose.orientation.x = static_cast<double>(msg->q[1]);
  vehicle_pose_px4_.pose.orientation.y = static_cast<double>(msg->q[2]);
  vehicle_pose_px4_.pose.orientation.z = static_cast<double>(msg->q[3]);

  // Convert NED → ENU position
  Eigen::Vector3d ned(vehicle_pose_px4_.pose.position.x,
                      vehicle_pose_px4_.pose.position.y,
                      vehicle_pose_px4_.pose.position.z);
  const Eigen::Vector3d enu = px4_ros_com::frame_transforms::ned_to_enu_local_frame(ned);

  // Convert NED → ENU orientation
  Eigen::Quaterniond q_ned(vehicle_pose_px4_.pose.orientation.w,
                           vehicle_pose_px4_.pose.orientation.x,
                           vehicle_pose_px4_.pose.orientation.y,
                           vehicle_pose_px4_.pose.orientation.z);
  const Eigen::Quaterniond q_enu =
    px4_ros_com::frame_transforms::ned_to_enu_orientation(q_ned.normalized());

  // Extract yaw angles
  tf2::Quaternion q_tf;
  tf2::fromMsg(vehicle_pose_px4_.pose.orientation, q_tf);
  double roll, pitch, yaw_ned;
  tf2::Matrix3x3(q_tf).getRPY(roll, pitch, yaw_ned);
  drone_yaw_ros_ = -yaw_ned;                              // ENU yaw for path-control setpoints
  drone_yaw_ned_ = static_cast<float>(yaw_ned);           // NED yaw for follow-target

  // Cache NED z for altitude hold in follow states
  current_ned_z_ = static_cast<float>(msg->position[2]);

  // Publish ENU pose
  vehicle_pose_ros_.header.frame_id    = "/map";
  vehicle_pose_ros_.header.stamp       = this->get_clock()->now();
  vehicle_pose_ros_.pose.position.x    = enu.x();
  vehicle_pose_ros_.pose.position.y    = enu.y();
  vehicle_pose_ros_.pose.position.z    = enu.z();
  vehicle_pose_ros_.pose.orientation.x = q_enu.x();
  vehicle_pose_ros_.pose.orientation.y = q_enu.y();
  vehicle_pose_ros_.pose.orientation.z = q_enu.z();
  vehicle_pose_ros_.pose.orientation.w = q_enu.w();
  vehicle_pose_publisher_->publish(vehicle_pose_ros_);

  vehicle_speed_.x = static_cast<double>(msg->velocity[0]);
  vehicle_speed_.y = static_cast<double>(msg->velocity[1]);
  vehicle_speed_.z = static_cast<double>(msg->velocity[2]);

  flag_vehicle_odometry_ = true;
}

void PathControl::vehicle_control_mode_callback(
  const px4_msgs::msg::VehicleControlMode::UniquePtr msg)
{
  vehicle_status_ = *msg;
}

void PathControl::path_callback(const nav_msgs::msg::Path msg)
{
  path_         = msg;
  flag_mission_ = true;
}

void PathControl::home_pos_callback(const px4_msgs::msg::HomePosition::UniquePtr msg)
{
  home_position_ = *msg;
}

// ─── Detection callback ───────────────────────────────────────────────────────

void PathControl::detection_callback(const yolo_msgs::msg::DetectionArray::UniquePtr msg)
{
  if (msg->detections.empty()) {
    has_detection_ = false;
    no_detection_counter_++;
    return;
  }

  const auto & target = msg->detections[0];
  has_detection_     = true;
  no_detection_counter_ = 0;

  det_bbox_x_    = target.bbox.center.position.x;
  det_bbox_y_    = target.bbox.center.position.y;
  det_bbox_size_ = target.bbox.size.y;
  det_target_id_ = target.id;

  // Pre-compute velocity and yaw-rate so handle_following_state() just reads them.
  // Forward velocity: proportional to (desired_size - actual_size) / desired_size
  float vbx = following_dist_kp_ *
              (following_desired_size_ - det_bbox_size_) / following_desired_size_;

  // Yaw rate: drive horizontal bbox error to zero
  following_yaw_rate_ = -following_yaw_kp_ *
                        (following_center_x_ - det_bbox_x_) / following_center_x_;

  // Decompose forward velocity into NED x/y using current NED yaw
  following_velocity_x_ = vbx * std::cos(drone_yaw_ned_);
  following_velocity_y_ = vbx * std::sin(drone_yaw_ned_);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
    "[FOLLOW] id:%s  vbx:%.2f  yaw_rate:%.2f  tgt(%.0f,%.0f) size:%.0f",
    det_target_id_.c_str(), vbx, following_yaw_rate_,
    det_bbox_x_, det_bbox_y_, det_bbox_size_);
}

// ─── Publishers ───────────────────────────────────────────────────────────────

void PathControl::publish_offboard_control_mode()
{
  if (stop_offboard_output_) { return; }
  px4_msgs::msg::OffboardControlMode msg{};
  msg.position     = true;
  msg.velocity     = true;
  msg.acceleration = false;
  msg.attitude     = true;
  msg.body_rate    = false;
  msg.timestamp    = get_clock()->now().nanoseconds() / 1000;
  offboard_control_publisher_->publish(msg);
}

void PathControl::publish_trajectory_setpoint(
  std::array<float, 3UL> position, float yaw,
  std::array<float, 3UL> velocity)
{
  if (stop_offboard_output_) { return; }

  // ENU → NED position
  Eigen::Vector3d ros_enu;
  ros_enu << position.at(0), position.at(1), position.at(2);
  Eigen::Vector3d px4_ned = px4_ros_com::frame_transforms::enu_to_ned_local_frame(ros_enu);
  position.at(0) = px4_ned(0);
  position.at(1) = px4_ned(1);
  position.at(2) = px4_ned(2);

  // ENU → NED velocity
  Eigen::Vector3d ros_enu_vel;
  ros_enu_vel << velocity.at(0), velocity.at(1), velocity.at(2);
  Eigen::Vector3d px4_ned_vel =
    px4_ros_com::frame_transforms::enu_to_ned_local_frame(ros_enu_vel);
  velocity.at(0) = px4_ned_vel(0);
  velocity.at(1) = px4_ned_vel(1);
  velocity.at(2) = px4_ned_vel(2);

  yaw = -drone_yaw_ros_;

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
  // All values are already in NED – no frame conversion needed.
  if (stop_offboard_output_) { return; }

  px4_msgs::msg::TrajectorySetpoint msg{};
  msg.position  = position;
  msg.velocity  = velocity;
  msg.yaw       = NAN;        // yaw controlled via yawspeed
  msg.yawspeed  = yaw_rate;
  msg.timestamp = get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher_->publish(msg);
}

// ─── Vehicle commands ─────────────────────────────────────────────────────────

void PathControl::arm()
{
  publish_vehicle_command(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
  RCLCPP_INFO(get_logger(), "Arm command send");
}

void PathControl::publish_vehicle_command(
  uint16_t command, float param1, float param2,
  float param3, float param4, float param5,
  float param6, float param7)
{
  if (stop_offboard_output_) { return; }
  px4_msgs::msg::VehicleCommand msg{};
  msg.param1 = param1; msg.param2 = param2; msg.param3 = param3;
  msg.param4 = param4; msg.param5 = param5; msg.param6 = param6; msg.param7 = param7;
  msg.command          = command;
  msg.target_system    = 1;
  msg.target_component = 1;
  msg.source_system    = 1;
  msg.source_component = 1;
  msg.from_external    = true;
  msg.timestamp        = get_clock()->now().nanoseconds() / 1000;
  vehicle_command_publisher_->publish(msg);
}

// ─── Utility ─────────────────────────────────────────────────────────────────

void PathControl::nonBlockingWait(std::chrono::milliseconds duration)
{
  std::thread([this, duration]() {
    std::this_thread::sleep_for(duration);
    std::lock_guard<std::mutex> lock(mutex_);
    flag_timer_done_ = true;
  }).detach();
}

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
  return std::abs(euclidean_distance(v1, v2)) <= tolerance;
}

void PathControl::check_pilot_state_switch()
{
  if (!vehicle_status_.flag_control_offboard_enabled && !pilot_takeover_) {
    pilot_takeover_ = true;
    current_state_  = State::LIMBO;
    RCLCPP_WARN(get_logger(),
      "Pilot takeover detected. Entering LIMBO and silencing offboard publishers");
  }
}

void PathControl::silence_offboard_publishers()
{
  if (offboard_publisher_silenced_) { return; }

  px4_msgs::msg::TrajectorySetpoint sp{};
  sp.position    = {NAN, NAN, NAN};
  sp.velocity    = {NAN, NAN, NAN};
  sp.acceleration = {NAN, NAN, NAN};
  sp.yaw         = NAN;
  sp.yawspeed    = NAN;
  sp.timestamp   = get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher_->publish(sp);

  stop_offboard_output_        = true;
  offboard_publisher_silenced_ = true;
  RCLCPP_WARN(get_logger(), "Offboard publishers silenced. Pilot has full control.");
}

void PathControl::set_take_off_waypoint()
{
  if (!flag_take_off_position) {
    take_off_waypoint.x  = vehicle_pose_ros_.pose.position.x;
    take_off_waypoint.y  = vehicle_pose_ros_.pose.position.y;
    take_off_waypoint.z  = vehicle_pose_ros_.pose.position.z + take_off_height_;
    take_off_heading_    = drone_yaw_ros_;

    home_position_ros_.x = vehicle_pose_ros_.pose.position.x;
    home_position_ros_.y = vehicle_pose_ros_.pose.position.y;
    home_position_ros_.z = vehicle_pose_ros_.pose.position.z;

    RCLCPP_INFO_STREAM(get_logger(), "Home position: x: " << home_position_ros_.x
      << " y: " << home_position_ros_.y << " z: " << home_position_ros_.z);
    RCLCPP_INFO_STREAM(get_logger(), "Take-off waypoint: x: " << take_off_waypoint.x
      << " y: " << take_off_waypoint.y << " z: " << take_off_waypoint.z);
  }
  flag_take_off_position = true;
}

bool PathControl::check_drone_startup_position()
{
  float tol_xy = 5.0f;
  float tol_z  = 1.5f;
  if (vehicle_pose_ros_.pose.position.x < tol_xy &&
      vehicle_pose_ros_.pose.position.x > -tol_xy)
  {
    if (vehicle_pose_ros_.pose.position.y < tol_xy &&
        vehicle_pose_ros_.pose.position.y > -tol_xy)
    {
      if (vehicle_pose_ros_.pose.position.z < tol_z &&
          vehicle_pose_ros_.pose.position.z > -tol_z)
      {
        return true;
      }
    }
  }
  return false;
}

void PathControl::capture_hold_position()
{
  if (!hold_position_captured_) {
    hold_position_x_       = static_cast<float>(vehicle_pose_px4_.pose.position.x);
    hold_position_y_       = static_cast<float>(vehicle_pose_px4_.pose.position.y);
    hold_altitude_         = current_ned_z_;
    hold_position_captured_ = true;
    RCLCPP_INFO(get_logger(),
      "[FOLLOW] Hold position captured – NED (%.2f, %.2f, %.2f)",
      hold_position_x_, hold_position_y_, hold_altitude_);
  }
}

void PathControl::quaternion_to_yaw(const Eigen::Quaterniond & q, float & yaw)
{
  // Standard ZYX Euler extraction for yaw
  yaw = static_cast<float>(
    std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
               1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z())));
}

// ─── State machine ────────────────────────────────────────────────────────────

inline const char * stateToString(State state)
{
  switch (state) {
    case State::PREFLIGHT:              return "PREFLIGHT";
    case State::IDLE:                   return "IDLE";
    case State::TAKEOFF:                return "TAKEOFF";
    case State::CLIMB_TO_FLIGHT_HEIGHT: return "CLIMB_TO_FLIGHT_HEIGHT";
    case State::MISSION:                return "MISSION";
    case State::FOLLOWING:              return "FOLLOWING";
    case State::FOLLOW_HOVER:           return "FOLLOW_HOVER";
    case State::FOLLOW_SEARCH:          return "FOLLOW_SEARCH";
    case State::HOVER:                  return "HOVER";
    case State::LIMBO:                  return "LIMBO";
    default:                            return "LIMBO";
  }
}

void PathControl::transitionToState(State new_state)
{
  current_state_          = new_state;
  flag_timer_done_        = false;
  has_executed_           = false;
  hold_position_captured_ = false;   // reset so next state can snapshot on entry
  RCLCPP_INFO_STREAM(get_logger(), "Transitioning to state: " << stateToString(new_state));
}

void PathControl::stateToSTring(State state)
{
  RCLCPP_INFO_STREAM(get_logger(), "Current state: " << stateToString(state));
}

void PathControl::timer_callback()
{
  switch (current_state_) {
    case State::PREFLIGHT:              handle_preflight_state();              break;
    case State::IDLE:                   handle_idle_state();                   break;
    case State::TAKEOFF:                handle_takeoff_state();                break;
    case State::CLIMB_TO_FLIGHT_HEIGHT: handle_climb_to_flight_height_state(); break;
    case State::MISSION:                handle_mission_state();                break;
    case State::FOLLOWING:              handle_following_state();              break;
    case State::FOLLOW_HOVER:           handle_follow_hover_state();           break;
    case State::FOLLOW_SEARCH:          handle_follow_search_state();          break;
    case State::HOVER:                  handle_hover_state();                  break;
    case State::LIMBO:                  handle_limbo_state();                  break;
    default:                            handle_limbo_state();                  break;
  }
}

// ─── State handlers ───────────────────────────────────────────────────────────

void PathControl::handle_preflight_state()
{
  if (flag_vehicle_odometry_) {
    if (check_drone_startup_position()) {
      set_take_off_waypoint();
      transitionToState(State::IDLE);
    } else {
      RCLCPP_ERROR_STREAM(get_logger(), "Drone is not at startup position! x: "
        << vehicle_pose_ros_.pose.position.x
        << " y: " << vehicle_pose_ros_.pose.position.y
        << " z: " << vehicle_pose_ros_.pose.position.z);
    }
  }
}

void PathControl::handle_idle_state()
{
  publish_offboard_control_mode();
  publish_trajectory_setpoint(
    {static_cast<float>(take_off_waypoint.x),
     static_cast<float>(take_off_waypoint.y),
     static_cast<float>(take_off_waypoint.z)},
    static_cast<float>(take_off_heading_));

  if (path_.poses.empty()) {
    if (!logged_no_path_) {
      RCLCPP_WARN(get_logger(), "No path received yet. Waiting for path...");
      logged_no_path_ = true;
    }
    return;
  } else {
    if (!logged_path_received_) {
      RCLCPP_INFO(get_logger(), "Path received. Arm drone for take-off.");
      logged_path_received_ = true;
    }
  }
  if (vehicle_status_.flag_armed == true) {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    if (vehicle_status_.flag_armed == true &&
        vehicle_status_.flag_control_offboard_enabled == true)
    {
      logged_no_path_      = false;
      logged_path_received_ = false;
      flag_mission_         = false;
      transitionToState(State::TAKEOFF);
    }
  }
}

void PathControl::handle_takeoff_state()
{
  check_pilot_state_switch();
  publish_offboard_control_mode();

  std::array<float, 3UL> vel = {0.0f, 0.0f, static_cast<float>(take_off_velocity_)};
  publish_trajectory_setpoint(
    {static_cast<float>(take_off_waypoint.x),
     static_cast<float>(take_off_waypoint.y),
     static_cast<float>(take_off_waypoint.z)},
    static_cast<float>(take_off_heading_), vel);

  if (reached_setpoint(take_off_waypoint, vehicle_pose_ros_.pose.position, position_tolerance_)) {
    if (!has_executed_) {
      nonBlockingWait(std::chrono::seconds(5));
      has_executed_ = true;
    }
    transitionToState(State::CLIMB_TO_FLIGHT_HEIGHT);
  }
}

void PathControl::handle_climb_to_flight_height_state()
{
  check_pilot_state_switch();
  publish_offboard_control_mode();

  geometry_msgs::msg::Point flight_pos = take_off_waypoint;
  flight_pos.z = path_.poses.front().pose.position.z;

  publish_trajectory_setpoint(
    {static_cast<float>(flight_pos.x),
     static_cast<float>(flight_pos.y),
     static_cast<float>(flight_pos.z)},
    static_cast<float>(take_off_heading_));

  if (reached_setpoint(flight_pos, vehicle_pose_ros_.pose.position, position_tolerance_)) {
    if (!has_executed_) {
      nonBlockingWait(std::chrono::seconds(3));
      has_executed_ = true;
    }
    if (flag_timer_done_) {
      transitionToState(State::MISSION);
    }
  }
}

void PathControl::handle_mission_state()
{
  check_pilot_state_switch();
  publish_offboard_control_mode();

  pure_pursuit_.setData(path_.poses, vehicle_pose_ros_.pose.position, vehicle_speed_, 0);
  auto control_output = pure_pursuit_.controlLoop();

  std::array<float, 3UL> pos_arr = {
    static_cast<float>(control_output.position.x),
    static_cast<float>(control_output.position.y),
    static_cast<float>(control_output.position.z)};
  std::array<float, 3UL> vel_arr = {
    static_cast<float>(control_output.velocity.x),
    static_cast<float>(control_output.velocity.y),
    static_cast<float>(control_output.velocity.z)};
  publish_trajectory_setpoint(pos_arr, control_output.yaw, vel_arr);

  if (pure_pursuit_.isPathFinished(path_.poses)) {
    hover_position_ = {
      static_cast<float>(path_.poses.back().pose.position.x),
      static_cast<float>(path_.poses.back().pose.position.y),
      static_cast<float>(control_output.position.z)};
    hover_velocity_ = {0.0f, 0.0f, 0.0f};
    transitionToState(State::HOVER);
    pure_pursuit_.reset();
  }
  if (has_detection_) {
  transitionToState(State::FOLLOWING);
  return;
}
}

void PathControl::handle_hover_state()
{
  check_pilot_state_switch();
  publish_offboard_control_mode();
  publish_trajectory_setpoint(hover_position_, take_off_heading_, hover_velocity_);

  if (vehicle_speed_.x < 0.1 && vehicle_speed_.y < 0.1 && vehicle_speed_.z < 0.1) {
    if (!has_replanned) {
      call_service();
      has_replanned = true;
    }
  }
  if (flag_mission_ == true) {
    transitionToState(State::MISSION);
    flag_mission_  = false;
    has_replanned  = false;
  }
  if (has_detection_) {
  transitionToState(State::FOLLOWING);
  return;
}
}

// ── Follow-target states ──────────────────────────────────────────────────────

void PathControl::handle_following_state()
{
  check_pilot_state_switch();
  publish_offboard_control_mode();

  // Lock altitude once on state entry
  if (!hold_position_captured_) {
    hold_altitude_         = current_ned_z_;
    hold_position_captured_ = true;
    RCLCPP_INFO(get_logger(),
      "[FOLLOW] Following – NED altitude locked at %.2f m", hold_altitude_);
  }

  // Target lost → wait before giving up
  if (!has_detection_) {
    transitionToState(State::FOLLOW_HOVER);
    RCLCPP_INFO(get_logger(), "[FOLLOW] Target lost! → FOLLOW_HOVER");
    return;
  }

  // Publish: NED horizontal velocity + explicit altitude hold + yaw-rate
  // position[2] = hold_altitude_ keeps the drone at its locked altitude.
  // position[0/1] = NAN so PX4 uses the velocity for x/y.
  publish_trajectory_setpoint_with_yawrate(
    {NAN, NAN, hold_altitude_},
    {following_velocity_x_, following_velocity_y_, 0.0f},
    following_yaw_rate_);
}

void PathControl::handle_follow_hover_state()
{
  check_pilot_state_switch();
  publish_offboard_control_mode();

  // Snapshot position once on entry
  capture_hold_position();

  // Target reacquired → resume following
  if (has_detection_) {
    no_detection_counter_ = 0;
    transitionToState(State::FOLLOWING);
    RCLCPP_INFO(get_logger(), "[FOLLOW] Target reacquired! → FOLLOWING");
    return;
  }

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
    "[FOLLOW] Hovering - lost for %d / %d frames",
    no_detection_counter_, following_hover_threshold_);

  // Threshold exceeded → start searching
  if (no_detection_counter_ >= following_hover_threshold_) {
    // transitionToState(State::FOLLOW_SEARCH);
     transitionToState(State::HOVER);
    RCLCPP_INFO(get_logger(), "[FOLLOW] Hover timeout → FOLLOW_SEARCH");
    return;
  }

  // Hold position, no yaw rotation
  publish_trajectory_setpoint_with_yawrate(
    {hold_position_x_, hold_position_y_, hold_altitude_},
    {0.0f, 0.0f, 0.0f},
    0.0f);
}

void PathControl::handle_follow_search_state()
{
  check_pilot_state_switch();
  publish_offboard_control_mode();

  // Snapshot position once on entry
  capture_hold_position();

  // Target found → switch to following
  if (has_detection_) {
    no_detection_counter_ = 0;
    transitionToState(State::FOLLOWING);
    RCLCPP_INFO(get_logger(), "[FOLLOW] Target found during search! → FOLLOWING");
    return;
  }

  // Hold position while rotating to scan
  publish_trajectory_setpoint_with_yawrate(
    {hold_position_x_, hold_position_y_, hold_altitude_},
    {0.0f, 0.0f, 0.0f},
    following_search_yaw_rate_);
}

void PathControl::handle_limbo_state()
{
  if (!offboard_publisher_silenced_) { silence_offboard_publishers(); }
}

// ─── Service call ─────────────────────────────────────────────────────────────

void PathControl::call_service()
{
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  start_mission_planning_client_->async_send_request(
    request,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture result) {
      auto response = result.get();
      if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Service succeeded: %s", response->message.c_str());
      } else {
        RCLCPP_WARN(this->get_logger(), "Service failed: %s", response->message.c_str());
      }
    });
}

// ─── Main ────────────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathControl>());
  rclcpp::shutdown();
  return 0;
}


