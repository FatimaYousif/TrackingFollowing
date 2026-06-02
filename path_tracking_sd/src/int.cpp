#include "path_tracking/int.hpp"

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
  if (enable_grasp_patrol_){
    while (!start_mission_planning_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }
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

    // ArUco subscriber
aruco_subscriber_ = create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
  "/aruco_markers",
  rclcpp::QoS(10).best_effort(),
  std::bind(&PathControl::aruco_callback, this, std::placeholders::_1));





  if (enable_person_following_){
    if (detection_camera_type_ == "RGB"){
      // subscriber with yolo msg type for RGB images
      detection_subscriber_rgb_ = create_subscription<ultralytics_ros::msg::YoloResult>(
        detection_topic_,
        rclcpp::QoS(10).reliable(),
        std::bind(&PathControl::detection_callback_rgb, this, std::placeholders::_1));
      
    } else if (detection_camera_type_ == "THERMAL"){
      //  subscriber with yolo msg type for thermal images
      detection_subscriber_thermal_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "/yolo/bbox_info",
        rclcpp::QoS(10).reliable(),
        std::bind(&PathControl::detection_callback_thermal, this, std::placeholders::_1));
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid detection camera type: %s", detection_camera_type_);
      // TODO: quit the node.
    }
  }

  timer_ =
    create_wall_timer(
    std::chrono::milliseconds(1000 / frequency),
    std::bind(&PathControl::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "Path Control node initialized.");
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
  this->declare_parameter<bool>("enable_grasp_patrol", false); 
  this->declare_parameter<bool>("enable_person_following", false); 
  this->declare_parameter<std::string>("detection_camera_type", ""); 

  // Get params from .yaml file
  take_off_height_    = this->get_parameter("take_off_height").as_double();
  take_off_velocity_  = this->get_parameter("take_off_velocity").as_double();
  ppc_velocity_       = this->get_parameter("ppc_velocity").as_double();
  max_acceleration_   = this->get_parameter("max_acceleration").as_double();
  position_tolerance_ = this->get_parameter("state_transition_tolerance").as_double();
  enable_grasp_patrol_      = this->get_parameter("enable_grasp_patrol").as_bool();
  enable_person_following_  = this->get_parameter("enable_person_following").as_bool();
  detection_camera_type_  = this->get_parameter("detection_camera_type").as_string();

  RCLCPP_INFO(this->get_logger(), "take_off_height: %f", take_off_height_);
  RCLCPP_INFO(this->get_logger(), "ppc_velocity: %f", ppc_velocity_);
  RCLCPP_INFO(this->get_logger(), "take_off_velocity: %f", take_off_velocity_);
  RCLCPP_INFO(this->get_logger(), "max_acceleration: %f", max_acceleration_);
  RCLCPP_INFO(this->get_logger(), "state_transition_tolerance: %f", position_tolerance_);
  RCLCPP_INFO(this->get_logger(), "enable_grasp_patrol_: %i", enable_grasp_patrol_);
  RCLCPP_INFO(this->get_logger(), "enable_person_following_: %i", enable_person_following_);
  RCLCPP_INFO(this->get_logger(), "detection_camera_type_: %s", detection_camera_type_.c_str());

  // -------------- Person-following params --------------
  
  this->declare_parameter<int>("detection_loss.hover_frames_threshold", 500);
  this->declare_parameter<double>("control.yaw_kp",            0.5);
  this->declare_parameter<double>("control.distance_kp",       0.8);
  this->declare_parameter<double>("setpoint.search.yaw_rate",  0.3);

  following_hover_threshold_ = this->get_parameter("detection_loss.hover_frames_threshold").as_int();
  following_yaw_kp_          = this->get_parameter("control.yaw_kp").as_double();
  following_dist_kp_         = this->get_parameter("control.distance_kp").as_double();
  following_search_yaw_rate_ = this->get_parameter("setpoint.search.yaw_rate").as_double();

  RCLCPP_INFO(this->get_logger(), "following_hover_threshold_: %i",   following_hover_threshold_);
  RCLCPP_INFO(this->get_logger(), "following_yaw_kp_: %f",   following_yaw_kp_);
  RCLCPP_INFO(this->get_logger(), "following_dist_kp_: %f",   following_dist_kp_);
  RCLCPP_INFO(this->get_logger(), "following_search_yaw_rate_: %f",   following_search_yaw_rate_);

  // TODO: Just load different yaml files via launch files
  if (detection_camera_type_ == "RGB"){
    this->declare_parameter<std::string>("rgb_camera.detection_topic", "/yolo_result");
    this->declare_parameter<double>("rgb_camera.center_x",           969.99036);   // cx and cy from calibration_0.3 exp on FLIR camera
    this->declare_parameter<double>("rgb_camera.center_y",           532.19128);
    this->declare_parameter<double>("rgb_camera.desired_target_size", 292.0);  

    detection_topic_           = this->get_parameter("rgb_camera.detection_topic").as_string();
    following_center_x_        = this->get_parameter("rgb_camera.center_x").as_double();
    following_center_y_        = this->get_parameter("rgb_camera.center_y").as_double();
    following_desired_size_    = this->get_parameter("rgb_camera.desired_target_size").as_double();
  } else if (detection_camera_type_ == "THERMAL"){
    this->declare_parameter<std::string>("thermal_camera.detection_topic", "/yolo/bbox_info");
    this->declare_parameter<double>("thermal_camera.center_x",           0.0);   
    this->declare_parameter<double>("thermal_camera.center_y",           0.0);
    this->declare_parameter<double>("thermal_camera.desired_target_size", 0.0);       
    
    detection_topic_           = this->get_parameter("thermal_camera.detection_topic").as_string();
    following_center_x_        = this->get_parameter("thermal_camera.center_x").as_double();
    following_center_y_        = this->get_parameter("thermal_camera.center_y").as_double();
    following_desired_size_    = this->get_parameter("thermal_camera.desired_target_size").as_double();
  }
  
  RCLCPP_INFO(this->get_logger(), "detection_topic: %s",          detection_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "hover_frames_threshold: %d",   following_hover_threshold_);
  RCLCPP_INFO(this->get_logger(), "following_center_x_: %f",   following_center_x_);
  RCLCPP_INFO(this->get_logger(), "following_center_y_: %f",   following_center_y_);
  RCLCPP_INFO(this->get_logger(), "following_desired_size_: %f",   following_desired_size_);

  RCLCPP_INFO(this->get_logger(), "-------------------------------");

  // ─── ArUco params ─────────────────────────────────────────────
    this->declare_parameter<double>("aruco.kp_xy",                     0.003);
    this->declare_parameter<double>("aruco.kd_xy",                     0.0005);
    this->declare_parameter<double>("aruco.max_vel_xy",                0.5);
    this->declare_parameter<int>   ("aruco.deadband_px",               10);
    this->declare_parameter<int>   ("aruco.centered_frames_threshold", 30);

    aruco_kp_xy_                  = this->get_parameter("aruco.kp_xy").as_double();
    aruco_kd_xy_                  = this->get_parameter("aruco.kd_xy").as_double();
    aruco_max_vel_xy_             = this->get_parameter("aruco.max_vel_xy").as_double();
    aruco_deadband_px_            = this->get_parameter("aruco.deadband_px").as_int();
    aruco_centered_frames_thresh_ = this->get_parameter("aruco.centered_frames_threshold").as_int();


  pure_pursuit_.setParameters(ppc_velocity_, max_acceleration_);
}


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

  constexpr double half_w = 1280.0 / 2.0;
  constexpr double half_h =  720.0 / 2.0;

  aruco_err_x_px_ = (pose.position.x / pose.position.z) * half_w;
  aruco_err_y_px_ = (pose.position.y / pose.position.z) * half_h;

  if (std::abs(aruco_err_x_px_) < aruco_deadband_px_) { aruco_err_x_px_ = 0.0; }
  if (std::abs(aruco_err_y_px_) < aruco_deadband_px_) { aruco_err_y_px_ = 0.0; }

  constexpr double dt = 1.0 / 100.0;
  double deriv_x = (aruco_err_x_px_ - aruco_prev_err_x_) / dt;
  double deriv_y = (aruco_err_y_px_ - aruco_prev_err_y_) / dt;
  aruco_prev_err_x_ = aruco_err_x_px_;
  aruco_prev_err_y_ = aruco_err_y_px_;

  auto clamp = [](double v, double lim) { return std::max(-lim, std::min(lim, v)); };
  double raw_vx = clamp(aruco_kp_xy_ * aruco_err_x_px_ + aruco_kd_xy_ * deriv_x, aruco_max_vel_xy_);
  double raw_vy = clamp(aruco_kp_xy_ * aruco_err_y_px_ + aruco_kd_xy_ * deriv_y, aruco_max_vel_xy_);

  // Camera +x (right) → NED +y,  Camera +y (down) → NED -x
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

void PathControl::handle_aruco_center_state()
{
  check_pilot_state_switch();
  publish_offboard_control_mode();
  capture_hold_position();   // latches NED x/y/z once on entry

  if (!has_aruco_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "[ARUCO_CENTER] Marker lost – holding position");
    publish_trajectory_setpoint_with_yawrate(
      {hold_position_x_, hold_position_y_, hold_altitude_},
      {0.0f, 0.0f, 0.0f}, 0.0f);
    return;
  }

  // Centred long enough → drop into a permanent hover at this spot
  if (aruco_centered_frames_ >= aruco_centered_frames_thresh_) {
    RCLCPP_INFO(get_logger(),
      "[ARUCO_CENTER] Marker id:%d centred – hovering at detection point", aruco_marker_id_);
    hover_position_ = {hold_position_x_, hold_position_y_, hold_altitude_};
    hover_velocity_ = {0.0f, 0.0f, 0.0f};
    transitionToState(State::HOVER);
    return;
  }

  // Active centering: velocity-only x/y, altitude hold
  publish_trajectory_setpoint_with_yawrate(
    {NAN, NAN, hold_altitude_},
    {aruco_vel_ned_x_, aruco_vel_ned_y_, 0.0f},
    0.0f);
}
// ----------------------------------------------------------------------

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
  drone_yaw_ros_ = -yaw_ned;                              // ENU yaw for path-control setpoints
  drone_yaw_ned_ = static_cast<float>(yaw_ned);           // NED yaw for follow-target
  
  // Cache NED z for altitude hold in follow states
  current_ned_z_ = static_cast<float>(msg->position[2]);

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

void PathControl::detection_callback_rgb(const ultralytics_ros::msg::YoloResult::SharedPtr msg)
{
  // if (msg->detections.empty()) {
  //   has_detection_ = false;
  //   no_detection_counter_++;
  //   return;
  // }

  // const auto & target = msg->detections[0];

  const auto &detections=msg->detections.detections;

  if (detections.empty())
  {
    has_detection_     = false;
    no_detection_counter_++;
    return;
  }

  const auto &target = detections[0];

  has_detection_     = true;
  no_detection_counter_ = 0;

  det_bbox_x_    = target.bbox.center.position.x;
  det_bbox_y_    = target.bbox.center.position.y;
  // det_bbox_size_ = target.bbox.size.y;
  det_bbox_size_ = target.bbox.size_y;
  
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

void PathControl::detection_callback_thermal(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
  if (msg->data.size() < 4)
  {
      has_detection_ = false;
      no_detection_counter_++;
      return;
  }

  has_detection_        = true;
  no_detection_counter_ = 0;

  float det_bbox_x_      = msg->data[0];  // center x
  float det_bbox_y_      = msg->data[1];  // center y
  float det_bbox_size_ = msg->data[3];  // height of bbox

  
  // det_target_id_ = target.id;

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
    "vbx:%.2f  yaw_rate:%.2f  tgt(%.0f,%.0f) size:%.0f", vbx, following_yaw_rate_,
    det_bbox_x_, det_bbox_y_, det_bbox_size_);
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
  { 
    float mission_yaw = -static_cast<float>(yaw) + static_cast<float>(M_PI / 2.0); 
    yaw=mission_yaw; 
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

  RCLCPP_INFO(get_logger(), "Arm command sent.");
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
    take_off_heading_ = -drone_yaw_ros_;   // Take-off heading is current drone heading

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

inline const char * stateToString(State state)
{
  switch (state) {
    case State::PREFLIGHT:              return "PREFLIGHT";
    case State::IDLE:                   return "IDLE";
    case State::TAKEOFF:                return "TAKEOFF";
    case State::CLIMB_TO_FLIGHT_HEIGHT: return "CLIMB_TO_FLIGHT_HEIGHT";
    case State::MISSION:                return "MISSION";                 // TODO: Rename to GRASP PPC
    case State::FOLLOWING:              return "FOLLOWING";
    case State::FOLLOW_HOVER:           return "FOLLOW_HOVER";
    case State::FOLLOW_SEARCH:          return "FOLLOW_SEARCH";
    case State::ARUCO_CENTER:           return "ARUCO_CENTER";
    case State::HOVER:                  return "HOVER";
    case State::LIMBO:                  return "LIMBO";
    default:                            return "LIMBO";
  }
}

void PathControl::transitionToState(State new_state)
{
  // Transition to the new state
  current_state_ = new_state;
  RCLCPP_INFO_STREAM(get_logger(), "Transitioning to state: " << stateToString(new_state));
  
  // Reset flags
  flag_timer_done_ = false;
  has_executed_ = false;
  hold_position_captured_ = false;

 // Reset ArUco PD integrator on every transition
  aruco_prev_err_x_      = 0.0;
  aruco_prev_err_y_      = 0.0;
  aruco_centered_frames_ = 0;

}

void PathControl::timer_callback()
{
  // State machine
  switch (current_state_) {
    case State::PREFLIGHT:              handle_preflight_state();               break; // PREFLIGHT state -> Do preflight checklist
    case State::IDLE:                   handle_idle_state();                    break; // IDLE state -> wait for take-off command and Set OFFBOARD mode
    case State::TAKEOFF:                handle_takeoff_state();                 break; // Take-off state -> Take-off and hover
    case State::CLIMB_TO_FLIGHT_HEIGHT: handle_climb_to_flight_height_state();  break; // Climb to flight height of GRASP Path
    case State::MISSION:                handle_mission_state();                 break;
    case State::FOLLOWING:              handle_following_state();               break;
    case State::FOLLOW_HOVER:           handle_follow_hover_state();            break; // Remain at current height
    case State::FOLLOW_SEARCH:          handle_follow_search_state();           break;
    case State::ARUCO_CENTER:           handle_aruco_center_state(); break;
    case State::HOVER:                  handle_hover_state();                   break;
    case State::LIMBO:                  handle_limbo_state();                   break;
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

  // Check if we got input path
  if (enable_grasp_patrol_){
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
  } else {
    RCLCPP_INFO_ONCE(get_logger(), "Not using grasp. Arm drone for take-off and person only follow mode.");
  }

  if (vehicle_status_.flag_armed == true) {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    if (vehicle_status_.flag_armed == true &&
      vehicle_status_.flag_control_offboard_enabled == true)
    {
      logged_no_path_ = false;
      logged_path_received_ = false;
      flag_mission_ = false;

      // Update take-off waypoint, as odom could have drifted between start of node and arming.
      take_off_waypoint.x = vehicle_pose_ros_.pose.position.x;
      take_off_waypoint.y = vehicle_pose_ros_.pose.position.y;
      take_off_waypoint.z = vehicle_pose_ros_.pose.position.z + take_off_height_;
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

    if (enable_grasp_patrol_){
      // Climb to flight height of GRASP Path
      transitionToState(State::CLIMB_TO_FLIGHT_HEIGHT); // TODO: rename to something grasp specific
    } else if (enable_person_following_){
      // Hover at current height (take-off height) and scan for detections
      // transitionToState(State::FOLLOW_HOVER);
      transitionToState(State::FOLLOW_SEARCH);
    } else {
      // print that both are false! 
    }
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

void PathControl::handle_mission_state()
{
  check_pilot_state_switch();
  publish_offboard_control_mode();

    // ArUco has priority over everything
    if (has_aruco_) {
    RCLCPP_INFO(get_logger(), "[...] ArUco id:%d detected → ARUCO_CENTER", aruco_marker_id_);
    transitionToState(State::ARUCO_CENTER);
    return;
    }


  if (enable_grasp_patrol_){
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

  if (enable_person_following_){
    if (has_detection_) {
      transitionToState(State::FOLLOWING);
      return;
    }
  }
}

void PathControl::handle_hover_state()
{
  check_pilot_state_switch();
  publish_offboard_control_mode();
  publish_trajectory_setpoint(hover_position_, take_off_heading_, hover_velocity_);

  if (enable_grasp_patrol_){
    // Once we have stopped moving, which should be end of the GRASP path, trigger the replan service.
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

  if (enable_person_following_){
    if (has_detection_) {
      transitionToState(State::FOLLOWING);
      return;
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


    // ArUco has priority over everything
    if (has_aruco_) {
    RCLCPP_INFO(get_logger(), "[...] ArUco id:%d detected → ARUCO_CENTER", aruco_marker_id_);
    transitionToState(State::ARUCO_CENTER);
    return;
    }


  // Target lost → wait before giving up
  if (!has_detection_) {
    hold_altitude_         = current_ned_z_;
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

    // ArUco has priority over everything
    if (has_aruco_) {
    RCLCPP_INFO(get_logger(), "[...] ArUco id:%d detected → ARUCO_CENTER", aruco_marker_id_);
    transitionToState(State::ARUCO_CENTER);
    return;
    }


  // Target re-acquired → resume following
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
    // transitionToState(State::FOLLOW_SEARCH); // TODO: Use gimbal control instead of yawing
    
    // Todo: fix this mess. Currently will only replan if grasp is enabled but will NOT yaw for searching.
    if (enable_grasp_patrol_){
      transitionToState(State::HOVER);
    } else {
      transitionToState(State::FOLLOW_SEARCH);
    }
     
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


  // ArUco has priority over everything
if (has_aruco_) {
  RCLCPP_INFO(get_logger(), "[...] ArUco id:%d detected → ARUCO_CENTER", aruco_marker_id_);
  transitionToState(State::ARUCO_CENTER);
  return;
}


  // Target found → switch to following
  if (has_detection_) {
    no_detection_counter_ = 0;
    transitionToState(State::FOLLOWING);
    RCLCPP_INFO(get_logger(), "[FOLLOW] Target found during search! → FOLLOWING");
    return;
  }

  // TODO: Use gimbal control instead of yawing
  // Hold position while rotating to scan
  publish_trajectory_setpoint_with_yawrate(
    {hold_position_x_, hold_position_y_, hold_altitude_},
    {0.0f, 0.0f, 0.0f},
    following_search_yaw_rate_);
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

// ------------------

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathControl>());

  rclcpp::shutdown();
  return 0;
}
