#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node
{
public:
  MapMemoryNode();

private:
  robot::MapMemoryCore map_memory_;

  // ROS2 subscribers and publisher
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

  // tf2 buffer and listener
  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // timer
  rclcpp::TimerBase::SharedPtr timer_;

  // parameters
  const double publish_frequency_ = 1.0; // Hz

  nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
  bool costmap_received_ = false;

  nav_msgs::msg::Odometry::SharedPtr latest_odom_ = nullptr;
  bool should_update_map_;
  const double distance_threshold_ = 5.0;

  const int width_ = 400;         // cells
  const int height_ = 400;        // cells
  const double resolution_ = 0.1; // meters/cell
  const double origin_x_ = -20.0; // meters
  const double origin_y_ = -20.0; // meters
  const double origin_z_ = 0.0;   // meters
  const double origin_orientation_x_ = 0.0;
  const double origin_orientation_y_ = 0.0;
  const double origin_orientation_z_ = 0.0;
  const double origin_orientation_w_ = 1.0;
  geometry_msgs::msg::Pose origin_;
  const int unknown_cost_ = -1;

  // callback functions
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap);

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);

  void timerCallback();
};

#endif
