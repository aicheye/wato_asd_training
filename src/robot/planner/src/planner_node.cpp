/**
 * @file planner_node.cpp
 * @author Girish Munukutla Srikanth
 * @brief Implementation of the PlannerNode class for publishing planned paths
 *        based on odometry, map, and goal inputs.
 * @date 2025-09-21
 */

#include "planner_core.hpp" // Ensure PlannerCore is included
#include "planner_node.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

PlannerNode::PlannerNode() : Node("planner_node"), state_(State::WAITING_FOR_GOAL), goal_timeout_(60.0),
                             path_refresh_interval_(0.5), goal_tolerance_(0.2)
{
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10,
          std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/goal_point", 10,
          std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10,
          std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = this->create_wall_timer(500ms, std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  map_ = *msg;
  if (state_ == State::REACHING_GOAL)
    planPath();
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  if (map_.data.empty())
  {
    RCLCPP_WARN(get_logger(), "No map yet. Cannot set goal.");
    return;
  }
  goal_ = *msg;
  goal_received_ = true;
  state_ = State::REACHING_GOAL;
  goal_start_time_ = now();
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  robot_pose_ = msg->pose.pose;
}

void PlannerNode::timerCallback()
{
  if (state_ != State::REACHING_GOAL)
    return;
  
  double dx = goal_.point.x - robot_pose_.position.x;
  double dy = goal_.point.y - robot_pose_.position.y;

  if (std::sqrt(dx * dx + dy * dy) < goal_tolerance_)
  {
    RCLCPP_INFO(get_logger(), "Goal reached!");
    resetGoal();
    return;
  }

  double elapsed = (now() - goal_start_time_).seconds();
  if (elapsed > goal_timeout_)
  {
    RCLCPP_WARN(get_logger(), "Goal timed out. Resetting.");
    resetGoal();
    return;
  }

  double since_last_path = (now() - last_path_time_).seconds();
  if (since_last_path > path_refresh_interval_) {
    planPath();
  }
}

void PlannerNode::resetGoal()
{
  state_ = State::WAITING_FOR_GOAL;
  goal_received_ = false;
  nav_msgs::msg::Path empty_path;
  empty_path.header.stamp = now();
  empty_path.header.frame_id = "sim_world";
  path_pub_->publish(empty_path);
}

CellIndex PlannerNode::worldToGrid(double wx, double wy)
{
  int gx = static_cast<int>((wx - map_.info.origin.position.x) / map_.info.resolution);
  int gy = static_cast<int>((wy - map_.info.origin.position.y) / map_.info.resolution);
  return CellIndex(gx, gy);
}

geometry_msgs::msg::PoseStamped PlannerNode::gridToWorld(const CellIndex &idx)
{
  geometry_msgs::msg::PoseStamped pose;
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

bool plan(const std::vector<int8_t> &grid, int width, int height,
                       const CellIndex &start, const CellIndex &goal,
                       std::vector<CellIndex> &out_path) {

    auto heuristic = [](const CellIndex &a, const CellIndex &b){ return std::hypot(a.x-b.x, a.y-b.y); };
    auto isFree = [&](const CellIndex &c){
        if(c.x<0 || c.x>=width || c.y<0 || c.y>=height) return false;
        int val = grid[c.y*width + c.x];
        return val >= 0 && val <= 90;
    };

    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
    std::unordered_map<CellIndex, double, CellIndexHash> g_score;

    g_score[start] = 0.0;
    open_set.emplace(start, heuristic(start, goal));

    std::vector<CellIndex> dirs{{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};

    while(!open_set.empty()){
        CellIndex current = open_set.top().idx; open_set.pop();
        if(current == goal){
            out_path.clear();
            while(came_from.find(current) != came_from.end()){
                out_path.push_back(current);
                current = came_from[current];
            }
            out_path.push_back(start);
            std::reverse(out_path.begin(), out_path.end());
            return true;
        }

        double current_g = g_score[current];
        for(auto d : dirs){
            CellIndex neighbor(current.x+d.x, current.y+d.y);
            if(!isFree(neighbor)) continue;

            double step_cost = (d.x!=0 && d.y!=0) ? std::sqrt(2.0) : 1.0;
            double penalty = grid[neighbor.y*width + neighbor.x]/25.0;
            double tentative_g = current_g + step_cost + penalty;

            if(g_score.find(neighbor)==g_score.end() || tentative_g < g_score[neighbor]){
                came_from[neighbor] = current;
                g_score[neighbor] = tentative_g;
                open_set.emplace(neighbor, tentative_g + heuristic(neighbor, goal));
            }
        }
    }
    return false;
}

void PlannerNode::planPath()
{
  last_path_time_ = now();

  if (!goal_received_ || map_.data.empty())
  {
    RCLCPP_WARN(get_logger(), "Cannot plan path: missing goal or map.");
    return;
  }

  RCLCPP_INFO(get_logger(), "Planning from (%f, %f) to (%f, %f)", robot_pose_.position.x, robot_pose_.position.y, goal_.point.x, goal_.point.y);

  CellIndex start = worldToGrid(robot_pose_.position.x, robot_pose_.position.y);
  CellIndex goal_idx = worldToGrid(goal_.point.x, goal_.point.y);

  std::vector<CellIndex> grid_path = std::vector<CellIndex>();

  if (!plan(map_.data, map_.info.width, map_.info.height, start, goal_idx, grid_path))
  {
    RCLCPP_WARN(get_logger(), "No path found!");
    return;
  }

  RCLCPP_INFO(get_logger(), "Path found with %zu waypoints.", grid_path.size());

  nav_msgs::msg::Path::SharedPtr path = std::make_shared<nav_msgs::msg::Path>();
  path->header.stamp = now();
  path->header.frame_id = "sim_world";
  for (const auto &cell : grid_path)
    path->poses.push_back(gridToWorld(cell));

  path_pub_->publish(*path);
  RCLCPP_INFO(get_logger(), "Published path with %zu waypoints.", path->poses.size());
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
