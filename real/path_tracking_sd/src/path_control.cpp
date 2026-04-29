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

  // Declare default parameters value
  declare_parameter("frequency", 100, frequency_des);   // Hz for timer_callback
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

  // Create trigger
  start_mission_planning_client_ = this->create_client<std_srvs::srv::Trigger>("/plan");

  // Wait until the service is available
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

  timer_ =
    create_wall_timer(
    std::chrono::milliseconds(1000 / frequency),
    std::bind(&PathControl::timer_callback, this));
}

void PathControl::declare_and_get_parameters()
{
  RCLCPP_INFO(this->get_logger(), "-------------------------------");
  RCLCPP_INFO(this->get_logger(), "Loading ROS params ...");

  // Declare params
  this->declare_parameter<float>("take_off_height", 0.0);
  this->declare_parameter<float>("ppc_velocity", 0.0);        // [m/s] Desired velocity for the pure pursuit controller
  this->declare_parameter<float>("take_off_velocity", 0.0);   // [unitless] Fraction of max_velocity used for turning
  this->declare_parameter<float>("max_acceleration", 0.0);    // [m/s^2] Max acceleration for velocity smoothing
  this->declare_parameter<double>("state_transition_tolerance", 0.0); // [m] Distance to waypoint to consider it reached


  // Get params from .yaml file
  take_off_height_ = this->get_parameter("take_off_height").as_double();
  take_off_velocity_ = this->get_parameter("take_off_velocity").as_double();
  ppc_velocity_ = this->get_parameter("ppc_velocity").as_double();
  max_acceleration_ = this->get_parameter("max_acceleration").as_double();
  position_tolerance_ = this->get_parameter("state_transition_tolerance").as_double();

  RCLCPP_INFO(this->get_logger(), "take_off_height: %f", take_off_height_);
  RCLCPP_INFO(this->get_logger(), "ppc_velocity: %f", ppc_velocity_);
  RCLCPP_INFO(this->get_logger(), "take_off_velocity: %f", take_off_velocity_);
  RCLCPP_INFO(this->get_logger(), "max_acceleration: %f", max_acceleration_);
  RCLCPP_INFO(this->get_logger(), "state_transition_tolerance: %f", position_tolerance_);
  RCLCPP_INFO(this->get_logger(), "-------------------------------");

  pure_pursuit_.setParameters(ppc_velocity_, max_acceleration_);
}

void PathControl::vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg)
{
  // Read PX4 message (PX4 uses NED; quaternion stored as [w, x, y, z])
  vehicle_pose_px4_.header.frame_id = "/map";
  vehicle_pose_px4_.header.stamp = this->get_clock()->now();
  vehicle_pose_px4_.pose.position.x = static_cast<double>(msg->position[0]);   // North (x)
  vehicle_pose_px4_.pose.position.y = static_cast<double>(msg->position[1]);   // East  (y)
  vehicle_pose_px4_.pose.position.z = static_cast<double>(msg->position[2]);   // Down  (z)
  vehicle_pose_px4_.pose.orientation.w = static_cast<double>(msg->q[0]);       // w
  vehicle_pose_px4_.pose.orientation.x = static_cast<double>(msg->q[1]);       // x
  vehicle_pose_px4_.pose.orientation.y = static_cast<double>(msg->q[2]);       // y
  vehicle_pose_px4_.pose.orientation.z = static_cast<double>(msg->q[3]);       // z

  // Convert position (NED) to ROS (ENU) coordinates
  Eigen::Vector3d ned(vehicle_pose_px4_.pose.position.x, vehicle_pose_px4_.pose.position.y,
    vehicle_pose_px4_.pose.position.z);
  const Eigen::Vector3d enu = px4_ros_com::frame_transforms::ned_to_enu_local_frame(ned);

  // Convert orientation (NED) to ROS (ENU) coordinates
  Eigen::Quaterniond q_ned(vehicle_pose_px4_.pose.orientation.w,
    vehicle_pose_px4_.pose.orientation.x, vehicle_pose_px4_.pose.orientation.y,
    vehicle_pose_px4_.pose.orientation.z);
  const Eigen::Quaterniond q_enu = px4_ros_com::frame_transforms::ned_to_enu_orientation(
    q_ned.normalized());

  // // Convert quaternion to yaw angle
  tf2::Quaternion q_tf;
  tf2::fromMsg(vehicle_pose_px4_.pose.orientation, q_tf);
  double roll, pitch, yaw_ned;
  tf2::Matrix3x3(q_tf).getRPY(roll, pitch, yaw_ned);
  drone_yaw_ros_ = -yaw_ned;

  // Publish ROS vehicle pose
  vehicle_pose_ros_.header.frame_id = "/map";
  vehicle_pose_ros_.header.stamp = this->get_clock()->now();
  vehicle_pose_ros_.pose.position.x = enu.x();
  vehicle_pose_ros_.pose.position.y = enu.y();
  vehicle_pose_ros_.pose.position.z = enu.z();
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
  // Save vehicle status
  vehicle_status_ = *msg;
}

void PathControl::path_callback(const nav_msgs::msg::Path msg)
{
  // Copy the incoming path
  path_ = msg;
  flag_mission_ = true;
}

void PathControl::home_pos_callback(const px4_msgs::msg::HomePosition::UniquePtr msg)
{
  home_position_ = *msg;
}

void PathControl::publish_offboard_control_mode()
{
  if (stop_offboard_output_) {return;}
  px4_msgs::msg::OffboardControlMode msg{};
  msg.position = true;
  msg.velocity = true;
  msg.acceleration = false;
  msg.attitude = true;
  msg.body_rate = false;
  msg.timestamp = get_clock()->now().nanoseconds() / 1000;
  offboard_control_publisher_->publish(msg);
}

void PathControl::publish_trajectory_setpoint(
  std::array<float, 3UL> position, float yaw,
  std::array<float, 3UL> velocity)
{
  if (stop_offboard_output_) {return;}

  // Convert position and velocity from ROS (ENU) to PX4 (NED) frame
  Eigen::Vector3d ros_enu;
  ros_enu << position.at(0), position.at(1), position.at(2);
  Eigen::Vector3d px4_ned;
  px4_ned = px4_ros_com::frame_transforms::enu_to_ned_local_frame(ros_enu);
  position.at(0) = px4_ned(0);
  position.at(1) = px4_ned(1);
  position.at(2) = px4_ned(2);

  // Convert velocity from ROS (ENU) to PX4 (NED) frame
  Eigen::Vector3d ros_enu_vel;
  ros_enu_vel << velocity.at(0), velocity.at(1), velocity.at(2);
  Eigen::Vector3d px4_ned_vel;
  px4_ned_vel = px4_ros_com::frame_transforms::enu_to_ned_local_frame(ros_enu_vel);
  velocity.at(0) = px4_ned_vel(0);
  velocity.at(1) = px4_ned_vel(1);
  velocity.at(2) = px4_ned_vel(2);

  // Convert yaw from ROS (ENU) to PX4 (NED) frame
  // yaw = -drone_yaw_ros_;

  if (current_state_ == State::MISSION) 
  { float mission_yaw = -static_cast<float>(yaw) + static_cast<float>(M_PI / 2.0); yaw=mission_yaw; 
  }

  // Create trajectory setpoint message and publish
  px4_msgs::msg::TrajectorySetpoint msg{};
  msg.position = position;
  msg.velocity = velocity;
  msg.yaw = yaw;
  msg.timestamp = get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher_->publish(msg);
}

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
  if (stop_offboard_output_) {return;}
  px4_msgs::msg::VehicleCommand msg{};
  msg.param1 = param1;
  msg.param2 = param2;
  msg.param3 = param3;
  msg.param4 = param4;
  msg.param5 = param5;
  msg.param6 = param6;
  msg.param7 = param7;
  msg.command = command;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  msg.timestamp = get_clock()->now().nanoseconds() / 1000;
  vehicle_command_publisher_->publish(msg);
}

// Function to wait for a certain duration without blocking the main thread
void PathControl::nonBlockingWait(std::chrono::milliseconds duration)
{
  std::thread(
    [this, duration]() {
      std::this_thread::sleep_for(duration);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        flag_timer_done_ = true;
      }
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
  // Check if drone mode has been switched by pilot then go to limbo state
  if (!vehicle_status_.flag_control_offboard_enabled && !pilot_takeover_) {
    pilot_takeover_ = true;
    current_state_ = State::LIMBO;
    RCLCPP_WARN(
      get_logger(), "Pilot takeover detected. Entering LIMBO and silencing offboard publishers");
  }
}

void PathControl::silence_offboard_publishers()
{
  if (offboard_publisher_silenced_) {return;}

  // Tell PX4 to ignore our setpoints
  px4_msgs::msg::TrajectorySetpoint sp{};
  sp.position = {NAN, NAN, NAN};
  sp.velocity = {NAN, NAN, NAN};
  sp.acceleration = {NAN, NAN, NAN};
  sp.yaw = NAN;
  sp.yawspeed = NAN;
  sp.timestamp = get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher_->publish(sp);

  // From now on, do not publish OffboardControlMode / TrajectorySetpoint / VehicleCommand
  stop_offboard_output_ = true;
  offboard_publisher_silenced_ = true;

  RCLCPP_WARN(get_logger(), "Offboard publishers silenced. Pilot has full control.");
}

void PathControl::set_take_off_waypoint()
{
  // Only do this once
  if (!flag_take_off_position) {
    // Set take-off waypoint in ROS coordinates
    take_off_waypoint.x = vehicle_pose_ros_.pose.position.x;
    take_off_waypoint.y = vehicle_pose_ros_.pose.position.y;
    take_off_waypoint.z = vehicle_pose_ros_.pose.position.z + take_off_height_;
    take_off_heading_ = drone_yaw_ros_;   // Take-off heading is current drone heading

    // Landing pad home position in ROS coordinates
    home_position_ros_.x = vehicle_pose_ros_.pose.position.x;
    home_position_ros_.y = vehicle_pose_ros_.pose.position.y;
    home_position_ros_.z = vehicle_pose_ros_.pose.position.z;

    // Print home position
    RCLCPP_INFO_STREAM(
      get_logger(), "Home position: x: " << home_position_ros_.x <<
        " y: " << home_position_ros_.y << " z: " << home_position_ros_.z);

    // Print take-off waypoint
    RCLCPP_INFO_STREAM(
      get_logger(), "Take-off waypoint: x: " << take_off_waypoint.x <<
        " y: " << take_off_waypoint.y << " z: " << take_off_waypoint.z);
  }
  flag_take_off_position = true;
}

bool PathControl::check_drone_startup_position()
{
  // Check if drone is at startup position
  float tolerance_xy = 5.0;
  float tolerance_z = 1.5;
  if (vehicle_pose_ros_.pose.position.x < tolerance_xy &&
    vehicle_pose_ros_.pose.position.x > -tolerance_xy)
  {
    if (vehicle_pose_ros_.pose.position.y < tolerance_xy &&
      vehicle_pose_ros_.pose.position.y > -tolerance_xy)
    {
      if (vehicle_pose_ros_.pose.position.z < tolerance_z &&
        vehicle_pose_ros_.pose.position.z > -tolerance_z)
      {
        return true;
      }
    }
  }
  return false;
}

inline const char * stateToString(State state)
{
  switch (state) {
    case State::PREFLIGHT: return "PREFLIGHT";
    case State::IDLE:      return "IDLE";
    case State::TAKEOFF:   return "TAKEOFF";
    case State::CLIMB_TO_FLIGHT_HEIGHT: return "CLIMB_TO_FLIGHT_HEIGHT";
    case State::MISSION:   return "MISSION";
    case State::HOVER:     return "HOVER";
    case State::LIMBO:     return "LIMBO";
    default:               return "LIMBO";
  }
}

void PathControl::transitionToState(State new_state)
{
  // Transition to the new state
  current_state_ = new_state;

  RCLCPP_INFO_STREAM(get_logger(), "Transitioning to state: " << stateToString(new_state));
  current_state_ = new_state;

  // Reset flags
  flag_timer_done_ = false;
  has_executed_ = false;
}

void PathControl::timer_callback()
{
  // State machine
  switch (current_state_) {
    // PREFLIGHT state -> Do preflight checklist
    case State::PREFLIGHT: handle_preflight_state(); break;
    // IDLE state -> wait for take-off command and Set OFFBOARD mode
    case State::IDLE: handle_idle_state(); break;
    // Take-off state -> Take-off and hover
    case State::TAKEOFF: handle_takeoff_state(); break;
    // Climb to flight height state
    case State::CLIMB_TO_FLIGHT_HEIGHT: handle_climb_to_flight_height_state(); break;
    // Mission State
    case State::MISSION: handle_mission_state(); break;
    // Hover state
    case State::HOVER: handle_hover_state(); break;
    // Limbo state
    case State::LIMBO: handle_limbo_state(); break;
    default: handle_limbo_state(); break;
  }
}

void PathControl::handle_preflight_state()
{
  // Wait for vehicle odometry
  if (flag_vehicle_odometry_) {
    // Check if drone is at startup position
    if (check_drone_startup_position()) {
      // Set take-off waypoint
      set_take_off_waypoint();

      // Change state to IDLE
      transitionToState(State::IDLE);
    } else {
      // ROS stream error message with drone x, y, z position
      RCLCPP_ERROR_STREAM(
        get_logger(), "Drone is not at startup position! x: " <<
          vehicle_pose_ros_.pose.position.x << " y: " << vehicle_pose_ros_.pose.position.y <<
          " z: " << vehicle_pose_ros_.pose.position.z);
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
      logged_no_path_ = false;
      logged_path_received_ = false;
      flag_mission_ = false;
      transitionToState(State::TAKEOFF);
    }
  }
}

void PathControl::handle_takeoff_state()
{
  check_pilot_state_switch();
  publish_offboard_control_mode();

  std::array<float,
    3UL> take_off_velocity_vector = {0.0, 0.0, static_cast<float>(take_off_velocity_)};

  publish_trajectory_setpoint(
    {static_cast<float>(take_off_waypoint.x),
      static_cast<float>(take_off_waypoint.y),
      static_cast<float>(take_off_waypoint.z)},
    static_cast<float>(take_off_heading_), take_off_velocity_vector);


  // Check if the setpoint has been reached in a specified tolerance
  if (reached_setpoint(take_off_waypoint, vehicle_pose_ros_.pose.position, position_tolerance_)) {
    if (!has_executed_) {
      nonBlockingWait(std::chrono::seconds(5));
      has_executed_ = true;
    }
    transitionToState(State::CLIMB_TO_FLIGHT_HEIGHT);
  }
}

void PathControl::handle_mission_state()
{
  check_pilot_state_switch();
  publish_offboard_control_mode();

  pure_pursuit_.setData(path_.poses, vehicle_pose_ros_.pose.position, vehicle_speed_, 0);
  auto control_output = pure_pursuit_.controlLoop();

  std::array<float, 3UL> position_array = {static_cast<float>(control_output.position.x),
    static_cast<float>(control_output.position.y),
    static_cast<float>(control_output.position.z)};

  std::array<float, 3UL> velocity_array = {static_cast<float>(control_output.velocity.x),
    static_cast<float>(control_output.velocity.y),
    static_cast<float>(control_output.velocity.z)};
  publish_trajectory_setpoint(position_array, control_output.yaw, velocity_array);

  if (pure_pursuit_.isPathFinished(path_.poses)) {
    hover_position_ =
      std::array<float, 3UL>{static_cast<float>(path_.poses.back().pose.position.x),
      static_cast<float>(path_.poses.back().pose.position.y),
      static_cast<float>(control_output.position.z)};
    hover_velocity_ = std::array<float, 3UL>{0.0, 0.0, 0.0};
    transitionToState(State::HOVER);
    pure_pursuit_.reset();
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
    //transition to mission state if new path received
    transitionToState(State::MISSION);
    flag_mission_ = false;
    has_replanned = false;
  }
}


void PathControl::handle_climb_to_flight_height_state()
{
  check_pilot_state_switch();
  publish_offboard_control_mode();

  // Fly to same x/y but flight_height z
  geometry_msgs::msg::Point flight_pos = take_off_waypoint;
  flight_pos.z = path_.poses.front().pose.position.z;

  publish_trajectory_setpoint(
    {static_cast<float>(flight_pos.x),
      static_cast<float>(flight_pos.y),
      static_cast<float>(flight_pos.z)},
    static_cast<float>(take_off_heading_)
  );

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

void PathControl::handle_limbo_state()
{
  // Silence the offboard publisher and then do nothing
  if (!offboard_publisher_silenced_) {silence_offboard_publishers();}
}

void PathControl::call_service()
{
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto future_result = start_mission_planning_client_->async_send_request(
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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathControl>());

  rclcpp::shutdown();
  return 0;
}
