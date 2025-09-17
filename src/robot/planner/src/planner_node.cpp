#include "planner_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include <queue>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <limits>
#include <functional>
#include <iostream>
#include <chrono>
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include <chrono>

struct CellIndex
{
  int x;
  int y;
 
  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}
 
  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }
 
  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};
 
// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    // A simple hash combining x and y
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};
 
// Structure representing a node in the A* open set
struct AStarNode
{
  CellIndex index;
  double f_score;  // f = g + h
 
  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};
 
// Comparator for the priority queue (min-heap by f_score)
struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b)
  {
    // We want the node with the smallest f_score on top
    return a.f_score > b.f_score;
  }
};


using std::placeholders::_1;

using namespace std::chrono_literals;

class PlannerNode : public rclcpp::Node{
  public:
    PlannerNode()
    : Node("planner_node")
    {
      map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&PlannerNode::mapCallback, this, _1));

      goal_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, _1));

      odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, _1));

      path_publisher = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

      timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
    }

  private:
    enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };
    State state_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;
    rclcpp::TimerBase::SharedPtr timer;
 
    // Data Storage
    nav_msgs::msg::OccupancyGrid current_map_;
    geometry_msgs::msg::PointStamped goal_;
    geometry_msgs::msg::Pose robot_pose_;

    bool goal_received_ = false;

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        current_map_ = *msg;
        if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
            planPath();
        }
    }

    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        goal_ = *msg;
        goal_received_ = true;
        state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
        planPath();
    }
 
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_pose_ = msg->pose.pose;
    }
 
    void timerCallback() {
        if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
            if (goalReached()) {
                RCLCPP_INFO(this->get_logger(), "Goal reached!");
                state_ = State::WAITING_FOR_GOAL;
            } else {
                RCLCPP_INFO(this->get_logger(), "Replanning due to timeout or progress...");
                planPath();
            }
        }
    }
 
    bool goalReached() {
        double dx = goal_.point.x - robot_pose_.position.x;
        double dy = goal_.point.y - robot_pose_.position.y;
        return std::sqrt(dx * dx + dy * dy) < 0.5; // Threshold for reaching the goal
    }

    CellIndex worldToGrid(double wx, double wy, const nav_msgs::msg::OccupancyGrid &map){
      int gx = static_cast<int>((wx-map.info.origin.position.x) / map.info.resolution);
      int gy = static_cast<int>((wy-map.info.origin.position.y)/map.info.resolution);
      return CellIndex(gx, gy);
    }

    geometry_msgs::msg::PoseStamped gridToWorld(const CellIndex &idx, const nav_msgs::msg::OccupancyGrid &map){
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = map.info.origin.position.x + (idx.x + 0.5) * map.info.resolution;
        pose.pose.position.y = map.info.origin.position.y + (idx.y + 0.5) * map.info.resolution;
        pose.pose.orientation.w = 1.0;
        return pose;
    }

    bool aStar(const CellIndex &start, const CellIndex &goal, int width, int height, const std::vector<int8_t> &map_data, std::vector<CellIndex> &out_path) {

      auto heuristic = [](const CellIndex &a, const CellIndex &b){
        return std::hypot(a.x - b.x, a.y - b.y);
      };

      auto isFree = [&](const CellIndex &idx){
        int i = idx.y * width + idx.x;
        return (i >= 0 && i < (int)map_data.size() && map_data[i] == 0);
      };

      std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
      std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
      std::unordered_map<CellIndex, double, CellIndexHash> g_score;

      g_score[start] = 0.0;
      open_set.emplace(start, heuristic(start, goal));

      std::vector<CellIndex> directions = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

      while (!open_set.empty()){

        CellIndex current = open_set.top().index;
        open_set.pop();

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

        for (auto d : directions){
          CellIndex neighbor(current.x + d.x, current.y + d.y);
          if (neighbor.x < 0 || neighbor.x >= width || neighbor.y < 0 || neighbor.y >= height) continue;
          if (!isFree(neighbor)) continue;

          double tentative_g = g_score[current] + heuristic(current, neighbor);

          if (g_score.find(neighbor) == g_score.end() || tentative_g < g_score[neighbor]){
              came_from[neighbor] = current;
              g_score[neighbor] = tentative_g;
              double f = tentative_g + heuristic(neighbor, goal);
              open_set.emplace(neighbor, f);
          }
        }
      }
      return false;
    }

    void planPath() {
      if (!goal_received_ || current_map_.data.empty()) {
          RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map or goal!");
          return;
      }

      nav_msgs::msg::Path path;
      path.header.stamp = this->get_clock()->now();
      path.header.frame_id = "map";

      CellIndex start_idx = worldToGrid(robot_pose_.position.x, robot_pose_.position.y, current_map_);
      CellIndex goal_idx = worldToGrid(goal_.point.x, goal_.point.y, current_map_);

      std::vector<CellIndex> grid_path;
      bool success = aStar(start_idx, goal_idx, current_map_.info.width, current_map_.info.height, current_map_.data, grid_path);

      if (!success) {
          RCLCPP_WARN(this->get_logger(), "No valid path found!");
          return;
      }

      for (const auto &cell : grid_path) {
          path.poses.push_back(gridToWorld(cell, current_map_));
      }

      path_publisher->publish(path);
      RCLCPP_INFO(this->get_logger(), "Path published with %zu waypoints", path.poses.size());
  }
};