#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_
#include "rclcpp/rclcpp.hpp"
#include "control_core.hpp"
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class ControlNode : public rclcpp::Node {
public:
    ControlNode();

private:
    // ROS2 interfaces
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // State
    nav_msgs::msg::Odometry::SharedPtr robot_odom_;

    // Control core
    robot::ControlCore controller_;

    // Parameters
    double lookahead_distance_;
    double goal_tolerance_;
    double linear_speed_;

    // Callbacks
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void controlLoop();

    // Helper
    double quaternionToYaw(const geometry_msgs::msg::Quaternion &q) const;
};
#endif // CONTROL_CORE_HPP_