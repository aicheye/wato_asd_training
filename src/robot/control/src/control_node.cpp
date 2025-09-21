/**
 * @file control_node.cpp
 * @author Girish Munukutla Srikanth
 * @brief ROS2 node for robot control, subscribing to path and odometry,
 *        publishing velocity commands.
 * @date 2025-09-21
 */

#include "control_node.hpp"
#include <cmath>

ControlNode::ControlNode()
    : Node("control"),
      lookahead_distance_(1.0),
      goal_tolerance_(0.1),
      linear_speed_(0.5)
{

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  controller_.setParameters(lookahead_distance_, goal_tolerance_, linear_speed_);

  control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  controller_.setPath(*msg);
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_odom_ = msg;
}

void ControlNode::controlLoop()
{
  if (!robot_odom_)
    return;

  double yaw = quaternionToYaw(robot_odom_->pose.pose.orientation);
  double x = robot_odom_->pose.pose.position.x;
  double y = robot_odom_->pose.pose.position.y;

  auto cmd_vel_opt = controller_.computeCommand(x, y, yaw);
  if (cmd_vel_opt)
    cmd_vel_pub_->publish(*cmd_vel_opt);
}

double ControlNode::quaternionToYaw(const geometry_msgs::msg::Quaternion &q) const
{
  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3 m(tf_q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
