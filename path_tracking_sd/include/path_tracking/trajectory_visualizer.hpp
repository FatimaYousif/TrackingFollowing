// trajectory_visualizer.hpp
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>

class TrajectoryVisualizer : public rclcpp::Node {
public:
    TrajectoryVisualizer();

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    std::vector<geometry_msgs::msg::Point> drone_trajectory_;  // Add this to your class
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_pub_;
};
