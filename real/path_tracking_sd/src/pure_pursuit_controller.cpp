#include "path_tracking/pure_pursuit_controller.hpp"


PurePursuitController::PurePursuitController()
: Node("pure_pursuit_controller")
{
  // Publishers
  trajectory_setpoint_publisher_ = create_publisher<geometry_msgs::msg::Point>(
    "/pure_pursuit/in/trajectory_setpoint", 10);
  current_position_publisher_ = create_publisher<geometry_msgs::msg::Point>(
    "/pure_pursuit/in/current_position", 10);
  tracking_error_publisher_ = create_publisher<std_msgs::msg::Float64>(
    "/pure_pursuit/eval/tracking_error", 10);
  yaw_command_publisher_ = create_publisher<std_msgs::msg::Float64>(
    "/pure_pursuit/out/yaw_command",
    10);
  actual_yaw_publisher_ =
    create_publisher<std_msgs::msg::Float64>("/pure_pursuit/in/actual_yaw", 10);
  yaw_error_publisher_ =
    create_publisher<std_msgs::msg::Float64>("/pure_pursuit/eval/yaw_error", 10);
  velocity_command_publisher_ = create_publisher<geometry_msgs::msg::Point>(
    "/pure_pursuit/out/velocity_command", 10);
  actual_velocity_publisher_ = create_publisher<geometry_msgs::msg::Point>(
    "/pure_pursuit/in/actual_velocity", 10);
  velocity_error_publisher_ = create_publisher<geometry_msgs::msg::Point>(
    "/pure_pursuit/eval/velocity_error", 10);
}

void PurePursuitController::setParameters(double desired_velocity, double max_acceleration)
{
  desired_velocity_ = desired_velocity;
  max_accel_ = max_acceleration;
  RCLCPP_INFO(
    this->get_logger(),
    "Pure Pursuit controller initialized with parameters: desired_velocity = %f, max_acceleration = %f",
    desired_velocity_,
    max_accel_);
}

void PurePursuitController::setData(
  const std::vector<geometry_msgs::msg::PoseStamped> & path,
  const geometry_msgs::msg::Point & current_position,
  const geometry_msgs::msg::Point & current_velocity,
  const double & current_yaw)
{
  path_ = path;
  current_position_ = current_position;
  current_velocity_ = current_velocity;
  current_yaw_ = current_yaw;
}

PurePursuitOutput PurePursuitController::controlLoop()
{
  publishLogData();
  PurePursuitOutput output;
  geometry_msgs::msg::Point lookahead_point = getLookaheadPoint();

  rclcpp::Time now = this->get_clock()->now();
  double dt_meas = last_time_set_ ? (now - last_time_).seconds() : dt_;
  last_time_ = now;
  last_time_set_ = true;

  // Compute velocity toward lookahead (constant magnitude)
  double dx = lookahead_point.x - current_position_.x;
  double dy = lookahead_point.y - current_position_.y;
  double dz = lookahead_point.z - current_position_.z;

  double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
  geometry_msgs::msg::Point v{};
  if (dist > 1e-3) {
    v.x = desired_velocity_ * dx / dist;
    v.y = desired_velocity_ * dy / dist;
    v.z = 0.0;
  }

  // Acceleration limiting for smoothness
  double dvx = v.x - last_vel_.x;
  double dvy = v.y - last_vel_.y;
  double dvz = v.z - last_vel_.z;
  double dvn = std::sqrt(dvx * dvx + dvy * dvy + dvz * dvz);
  double max_dv = max_accel_ * dt_meas;
  if (dvn > max_dv && dvn > 1e-6) {
    double s = max_dv / dvn;
    v.x = last_vel_.x + dvx * s;
    v.y = last_vel_.y + dvy * s;
    v.z = last_vel_.z + dvz * s;
  }
  last_vel_ = v;

  output.velocity = v;
  output.yaw=computeYawCommand(lookahead_point);         // for drone orientation during path tracking

  geometry_msgs::msg::Point pos{};
  pos.x = std::numeric_limits<float>::quiet_NaN();  // Don't control position in x, y (we only control velocity)
  pos.y = std::numeric_limits<float>::quiet_NaN();  // Don't control position in x, y (we only control velocity)
  pos.z = path_.front().pose.position.z;            // Control position in z to maintain altitude
  output.position = pos;

  return output;
}

geometry_msgs::msg::Point PurePursuitController::getLookaheadPoint()
{
  double accumulated_distance = 0.0;
  last_point_ = current_position_;
  lookahead_distance_ = std::clamp(1.0 * desired_velocity_, 0.1, 2.0);
  for (size_t i = global_index_; i < path_.size(); ++i) {
    next_point_ = path_[i].pose.position;
    double segment_distance = distance(last_point_, next_point_);
    accumulated_distance += segment_distance;

    if (accumulated_distance >= lookahead_distance_) {
      double overshoot_distance = accumulated_distance - lookahead_distance_;
      double ratio = 1.0 - (overshoot_distance / segment_distance);

      target_.x = last_point_.x + ratio * (next_point_.x - last_point_.x);
      target_.y = last_point_.y + ratio * (next_point_.y - last_point_.y);
      target_.z = last_point_.z;

      global_index_ = i;
      last_target_ = target_;
      return target_;
    }
    last_point_ = next_point_;
  }
  global_index_ = path_.size();
  return path_.back().pose.position;
}

double PurePursuitController::computeTrackingError(
  const geometry_msgs::msg::Point & current_position,
  const geometry_msgs::msg::Point & previous_position,
  const geometry_msgs::msg::Point & next_position)
{
  double dx = next_position.x - previous_position.x;
  double dy = next_position.y - previous_position.y;

  // General line equation: Ax + By + C = 0
  double A = dy;
  double B = -dx;
  double C = next_position.x * previous_position.y - next_position.y * previous_position.x;

  double distance = std::abs(A * current_position.x + B * current_position.y + C) / std::sqrt(
    A * A + B * B);

  // Check if the point is on the left or right side of the line
  double sign =
    (dx * (current_position.y - previous_position.y) - dy *
    (current_position.x - previous_position.x)) > 0 ? 1.0 : -1.0;

  // Return the signed distance
  return sign * distance;
}

bool PurePursuitController::isPathFinished(
  const std::vector<geometry_msgs::msg::PoseStamped> & path) const
{
  return global_index_ >= path.size();
}

void PurePursuitController::reset()
{
  global_index_ = 0;
}

double PurePursuitController::computeYawCommand(const geometry_msgs::msg::Point & target_point)
{
  double target_yaw = std::atan2(
    target_point.y - current_position_.y,
    target_point.x - current_position_.x);
  last_yaw_command_ = target_yaw;

  return target_yaw;
}

geometry_msgs::msg::Point PurePursuitController::getVelocityCommand(double yaw)
{
  target_velocity_x_ = std::cos(yaw) * desired_velocity_;
  target_velocity_y_ = std::sin(yaw) * desired_velocity_;

  geometry_msgs::msg::Point velocity_command;
  velocity_command.x = target_velocity_x_;
  velocity_command.y = target_velocity_y_;
  velocity_command.z = 0.0;                 // We don't control vertical velocity

  last_velocity_command_ = velocity_command;

  return velocity_command;
}

void PurePursuitController::publishLogData()
{
  // Yaw data
  std_msgs::msg::Float64 yaw_command_msg;
  yaw_command_msg.data = last_yaw_command_;
  yaw_command_publisher_->publish(yaw_command_msg);

  std_msgs::msg::Float64 actual_yaw_msg;
  actual_yaw_msg.data = current_yaw_;
  actual_yaw_publisher_->publish(actual_yaw_msg);

  std_msgs::msg::Float64 yaw_error_msg;
  yaw_error_msg.data = std::abs(last_yaw_command_) - std::abs(current_yaw_);
  yaw_error_publisher_->publish(yaw_error_msg);

  // Velocity data
  actual_velocity_publisher_->publish(current_velocity_);
  velocity_command_publisher_->publish(last_velocity_command_);

  geometry_msgs::msg::Point velocity_error_msg;
  velocity_error_msg.x = current_velocity_.x - last_velocity_command_.x;
  velocity_error_msg.y = current_velocity_.y - last_velocity_command_.y;
  velocity_error_msg.z = 0.0;
  velocity_error_publisher_->publish(velocity_error_msg);

  // Path tracking data
  trajectory_setpoint_publisher_->publish(target_);
  current_position_publisher_->publish(current_position_);

  double error = computeTrackingError(
    current_position_, path_[global_index_ - 1].pose.position,
    path_[global_index_].pose.position);
  std_msgs::msg::Float64 tracking_error_msg;
  tracking_error_msg.data = error;
  tracking_error_publisher_->publish(tracking_error_msg);

}

double PurePursuitController::distance(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2) const
{
  return std::sqrt(
    (p2.x - p1.x) * (p2.x - p1.x) +
    (p2.y - p1.y) * (p2.y - p1.y) +
    (p2.z - p1.z) * (p2.z - p1.z));
}
