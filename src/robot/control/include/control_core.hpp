#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <optional>

namespace robot
{

  class ControlCore
  {
  public:
    ControlCore();

    void setParameters(double lookahead_distance, double goal_tolerance, double linear_speed);

    void setPath(const nav_msgs::msg::Path &path);

    std::optional<geometry_msgs::msg::Twist> computeCommand(
        double robot_x, double robot_y, double robot_theta) const;

  private:
    nav_msgs::msg::Path current_path_;
    double lookahead_distance_;
    double goal_tolerance_;
    double linear_speed_;

    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint(double robot_x, double robot_y) const;
    double computeDistance(double x1, double y1, double x2, double y2) const;
  };

} // namespace robot
#endif // CONTROL_CORE_HPP_
