/**
 * @file planner_core.cpp
 * @author Girish Munukutla Srikanth
 * @brief Implementation of the PlannerCore class for A* path planning on a 2D grid.
 * @date 2025-09-17
 */

#include "planner_core.hpp"
#include <algorithm>

robot::PlannerCore::PlannerCore(const rclcpp::Logger &logger) : logger_(logger) {}
