#ifndef PLANNER_NODE_HPP
#define PLANNER_NODE_HPP

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <mutex>
#include <vector>
#include <functional>
#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
public:
    PlannerNode();

private:
    // Node states
    enum class State { WAITING_FOR_GOAL, REACHING_GOAL };

    // ROS2 interfaces
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Data
    geometry_msgs::msg::Pose robot_pose_;
    geometry_msgs::msg::PointStamped goal_;
    nav_msgs::msg::OccupancyGrid map_;
    std::mutex map_mutex_;
    PlannerCore planner_;
    State state_;
    bool goal_received_;
    rclcpp::Time goal_start_time_;
    double goal_timeout_;
    double goal_tolerance_;

    // Callbacks
    void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void timerCallback();

    // Helpers
    void resetGoal();
    CellIndex worldToGrid(double wx, double wy);
    geometry_msgs::msg::PoseStamped gridToWorld(const CellIndex &idx);
    void planPath();
};

#endif