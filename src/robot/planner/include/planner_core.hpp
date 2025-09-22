#ifndef PLANNER_CORE_HPP
#define PLANNER_CORE_HPP

#include "rclcpp/rclcpp.hpp"
#include "planner_node.hpp"
#include <vector>
#include <unordered_map>
#include <queue>
#include <cmath>

#pragma once

namespace robot
{
  class PlannerCore {
  public:
    PlannerCore(const rclcpp::Logger &logger);

  private:
    rclcpp::Logger logger_;
  };
} // namespace robot

#endif
