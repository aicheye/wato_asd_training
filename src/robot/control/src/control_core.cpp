/**
 * @file control_core.cpp
 * @author Girish Munukutla Srikanth
 * @brief Implementation of the ControlCore class for robot control, 
 *        including path following and obstacle avoidance.
 * @date 2025-09-21
 */

#include "control_core.hpp"
#include <cmath>

namespace robot
{

  ControlCore::ControlCore() : lookahead_distance_(1.0), goal_tolerance_(0.1), linear_speed_(1.0)
  {}

  void ControlCore::setParameters(double lookahead_distance, double goal_tolerance, double linear_speed)
  {
    lookahead_distance_ = lookahead_distance;
    goal_tolerance_ = goal_tolerance;
    linear_speed_ = linear_speed;
  }

  void ControlCore::setPath(const nav_msgs::msg::Path &path)
  {
    current_path_ = path;
  }

  std::optional<geometry_msgs::msg::Twist> ControlCore::computeCommand(double robot_x, double robot_y,
                                                                       double robot_theta) const
  {
    if (current_path_.poses.empty()) {
      geometry_msgs::msg::Twist cmd_vel = geometry_msgs::msg::Twist();
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = 0.0;
      return cmd_vel;
    }

    auto lookahead_point_opt = findLookaheadPoint(robot_x, robot_y);
    if (!lookahead_point_opt)
      return std::nullopt;

    const auto &target = *lookahead_point_opt;
    double dx = target.pose.position.x - robot_x;
    double dy = target.pose.position.y - robot_y;

    double local_x = std::cos(robot_theta) * dx + std::sin(robot_theta) * dy;
    double local_y = -std::sin(robot_theta) * dx + std::cos(robot_theta) * dy;
    double L = std::sqrt(local_x * local_x + local_y * local_y);
    if (L < 1e-6)
      return std::nullopt;

    double curvature = 2.0 * local_y / (L * L);

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = local_x > 0 ? local_x * linear_speed_ : 0;
    cmd_vel.angular.z = linear_speed_ * curvature;

    return cmd_vel;
  }

  std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint(double robot_x, double robot_y) const
  {
    for (const auto &pose_stamped : current_path_.poses)
    {
      double distance = computeDistance(robot_x, robot_y, pose_stamped.pose.position.x, pose_stamped.pose.position.y);
      if (distance >= lookahead_distance_)
        return pose_stamped;
    }
    return current_path_.poses.back();
  }

  double ControlCore::computeDistance(double x1, double y1, double x2, double y2) const
  {
    return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
  }

} // namespace robot
