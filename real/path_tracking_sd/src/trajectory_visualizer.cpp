#include "path_tracking/trajectory_visualizer.hpp"

TrajectoryVisualizer::TrajectoryVisualizer()
    : Node("gazebo_path_visualizer")
{
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/vehicle_pose", 10,
        std::bind(&TrajectoryVisualizer::pose_callback, this, std::placeholders::_1));

    trajectory_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/drone_trajectory_marker", 10);
}

void TrajectoryVisualizer::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    geometry_msgs::msg::Point point = msg->pose.position;
    drone_trajectory_.push_back(point);  // Accumulate position

    visualization_msgs::msg::Marker marker;
    marker.header = msg->header;
    marker.ns = "drone_trajectory";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.05;  // line width
    marker.color.r = 0.0;
    marker.color.g = 0.6;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.pose.orientation.w = 1.0;

    marker.points = drone_trajectory_;

    trajectory_pub_->publish(marker);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryVisualizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
