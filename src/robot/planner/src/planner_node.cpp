/**
 * @file planner_node.cpp
 * @author Girish Munukutla Srikanth
 * @brief Implementation of the PlannerNode class for publishing planned paths 
 *        based on odometry, map, and goal inputs.
 * @date 2025-09-21
 */

#include "planner_node.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

PlannerNode::PlannerNode() : Node("planner_node"), state_(State::WAITING_FOR_GOAL),
                             goal_received_(false), goal_timeout_(10.0), goal_tolerance_(0.5) {
    // declare a dummy parameter to satisfy foxglove_bridge
    this->declare_parameter("dummy_param", 0);

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10,
                std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 10,
                std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10,
                std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

    timer_ = this->create_wall_timer(500ms, std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_ = *msg;
    if(state_ == State::REACHING_GOAL) planPath();
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if(map_.data.empty()){
        RCLCPP_WARN(get_logger(), "No map yet. Cannot set goal.");
        return;
    }
    goal_ = *msg;
    goal_received_ = true;
    state_ = State::REACHING_GOAL;
    goal_start_time_ = now();
    planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback() {
    if(state_ != State::REACHING_GOAL) return;

    double elapsed = (now() - goal_start_time_).seconds();
    if(elapsed > goal_timeout_){
        RCLCPP_WARN(get_logger(), "Goal timed out. Resetting.");
        resetGoal();
        return;
    }

    double dx = goal_.point.x - robot_pose_.position.x;
    double dy = goal_.point.y - robot_pose_.position.y;
    if(std::sqrt(dx*dx + dy*dy) < goal_tolerance_){
        RCLCPP_INFO(get_logger(), "Goal reached!");
        resetGoal();
    }
}

void PlannerNode::resetGoal(){
    state_ = State::WAITING_FOR_GOAL;
    goal_received_ = false;
    nav_msgs::msg::Path empty_path;
    empty_path.header.stamp = now();
    empty_path.header.frame_id = "sim_world";
    path_pub_->publish(empty_path);
}

CellIndex PlannerNode::worldToGrid(double wx, double wy){
    std::lock_guard<std::mutex> lock(map_mutex_);
    int gx = static_cast<int>((wx - map_.info.origin.position.x)/map_.info.resolution);
    int gy = static_cast<int>((wy - map_.info.origin.position.y)/map_.info.resolution);
    return CellIndex(gx, gy);
}

geometry_msgs::msg::PoseStamped PlannerNode::gridToWorld(const CellIndex &idx){
    geometry_msgs::msg::PoseStamped pose;
    std::lock_guard<std::mutex> lock(map_mutex_);
    pose.header.frame_id = "sim_world";
    pose.header.stamp = now();
    pose.pose.position.x = (idx.x + 0.5) * map_.info.resolution + map_.info.origin.position.x;
    pose.pose.position.y = (idx.y + 0.5) * map_.info.resolution + map_.info.origin.position.y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    return pose;
}

void PlannerNode::planPath(){
    if(!goal_received_ || map_.data.empty()){
        RCLCPP_WARN(get_logger(), "Cannot plan path: missing goal or map.");
        return;
    }

    RCLCPP_INFO(get_logger(), "Planning from (%f, %f) to (%f, %f)", robot_pose_.position.x, robot_pose_.position.y, goal_.point.x, goal_.point.y);

    CellIndex start = worldToGrid(robot_pose_.position.x, robot_pose_.position.y);
    CellIndex goal_idx = worldToGrid(goal_.point.x, goal_.point.y);

    std::vector<CellIndex> grid_path;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if(!planner_.plan(map_.data, map_.info.width, map_.info.height, start, goal_idx, grid_path)){
            RCLCPP_WARN(get_logger(), "No path found!");
            return;
        }
    }

    RCLCPP_INFO(get_logger(), "Path found with %zu waypoints.", grid_path.size());

    nav_msgs::msg::Path::SharedPtr path = std::make_shared<nav_msgs::msg::Path>();
    path->header.stamp = now();
    path->header.frame_id = "map";
    for(const auto &cell : grid_path) path->poses.push_back(gridToWorld(cell));

    path_pub_->publish(*path);
    RCLCPP_INFO(get_logger(), "Published path with %zu waypoints.", path->poses.size());
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}
