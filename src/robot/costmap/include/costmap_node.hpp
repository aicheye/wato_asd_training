#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node
{
public:
  CostmapNode();

private:
  robot::CostmapCore costmap_;
  // ROS2 subscriber and publisher
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

  // costmap parameters
  const int width_ = 200;         // cells
  const int height_ = 200;        // cells
  const double resolution_ = 0.1; // meters/cell
  const double origin_x_ = -10.0;  // meters
  const double origin_y_ = -10.0;  // meters
  const double origin_z_ = 0.0;   // meters
  const double origin_orientation_x_ = 0.0;
  const double origin_orientation_y_ = 0.0;
  const double origin_orientation_z_ = 0.0;
  const double origin_orientation_w_ = 1.0;
  geometry_msgs::msg::Pose origin_;
  const double inflation_radius_ = 1.5; // meters
  const double obstacle_cost_ = 100.0;

  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr laserscan_msg);
};

#endif
