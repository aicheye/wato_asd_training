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

struct CellIndex {
  int x, y;
  CellIndex(int xx=0, int yy=0) : x(xx), y(yy) {}
  bool operator==(const CellIndex &other) const { return x==other.x && y==other.y; }
};

struct CellIndexHash {
  std::size_t operator()(const CellIndex &idx) const {
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y)<<1);
  }
};

struct AStarNode {
  CellIndex idx; double f;
  AStarNode(CellIndex i, double f_score) : idx(i), f(f_score) {}
};

struct CompareF {
  bool operator()(const AStarNode &a, const AStarNode &b){ return a.f > b.f; }
};

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
    State state_;
    bool goal_received_;
    rclcpp::Time goal_start_time_;
    rclcpp::Time last_path_time_;
    double goal_timeout_;
    double path_refresh_interval_;
    double goal_tolerance_;

    // Callbacks
    void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void timerCallback();

    // Helpers
    CellIndex worldToGrid(double wx, double wy);
    geometry_msgs::msg::PoseStamped gridToWorld(const CellIndex &idx);
    void resetGoal();
    void planPath();
};

#endif
