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
#include "../../../../../../../../private/tmp/deps/ros/humble/include/nav_msgs/nav_msgs/msg/detail/odometry__struct.hpp"
#include "../../../../../../../../private/tmp/deps/ros/humble/include/nav_msgs/nav_msgs/msg/detail/path__struct.hpp"

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
    void mapCallback() const
    {
      
    }

    void goalCallback() const
    {
      
    }

    void odomCallback() const
    {
      
    }

    void timerCallback() const
    {

    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;
    rclcpp::TimerBase::SharedPtr timer;

};