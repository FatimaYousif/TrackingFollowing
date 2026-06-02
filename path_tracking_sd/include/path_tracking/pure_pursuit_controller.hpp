#pragma once

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64.hpp"

struct PurePursuitOutput
{
  geometry_msgs::msg::Point position;
  geometry_msgs::msg::Point velocity;
  double yaw;
};

class PurePursuitController : public rclcpp::Node
{
public:
  PurePursuitController();

  PurePursuitOutput controlLoop();

  void reset();
  void setParameters(double desired_velocity, double max_acceleration = 2.0);
  void setData(
    const std::vector<geometry_msgs::msg::PoseStamped> & path,
    const geometry_msgs::msg::Point & current_position,
    const geometry_msgs::msg::Point & current_velocity, const double & current_yaw);
  bool isPathFinished(const std::vector<geometry_msgs::msg::PoseStamped> & path) const;
  void publishLogData();

private:
  geometry_msgs::msg::Point current_position_;
  geometry_msgs::msg::Point current_velocity_;
  double current_yaw_;
  std::vector<geometry_msgs::msg::PoseStamped> path_;
  double desired_velocity_;

  double target_velocity_x_ = 0.0;
  double target_velocity_y_ = 0.0;

  double lookahead_distance_;  // meters

  geometry_msgs::msg::Point last_vel_{};
  double max_accel_;              // m/s^2 (tune)
  bool last_time_set_ = false;
  rclcpp::Time last_time_;              // for dt measurement
  double dt_;                 // control period


  size_t global_index_ = 0;
  geometry_msgs::msg::Point target_;
  geometry_msgs::msg::Point last_target_;
  geometry_msgs::msg::Point last_point_;
  geometry_msgs::msg::Point next_point_;

  double last_yaw_command_ = 0.0;
  geometry_msgs::msg::Point last_velocity_command_;

  double distance(
    const geometry_msgs::msg::Point & p1,
    const geometry_msgs::msg::Point & p2) const;

  geometry_msgs::msg::Point getLookaheadPoint();

  double computeYawCommand(const geometry_msgs::msg::Point & target_point);

  double computeTrackingError(
    const geometry_msgs::msg::Point & current_position,
    const geometry_msgs::msg::Point & previous_position,
    const geometry_msgs::msg::Point & next_position);

  geometry_msgs::msg::Point getVelocityCommand(double yaw);

  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr current_position_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr tracking_error_publisher_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_command_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr actual_yaw_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_error_publisher_;

  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr actual_velocity_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr velocity_command_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr velocity_error_publisher_;

};
