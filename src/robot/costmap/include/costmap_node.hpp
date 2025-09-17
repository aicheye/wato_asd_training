#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
public:
  CostmapNode();

  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr laserscan_msg);

  void publishCostmap();

private:
  robot::CostmapCore costmap_;
  // Place these constructs here
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  const int width_ = 100;
  const int height_ = 100;
  const double resolution_ = 0.1;
  const double origin_x_ = -5.0;
  const double origin_y_ = -5.0;
  const double origin_z_ = 0.0;
  const double origin_orientation_x_ = 0.0;
  const double origin_orientation_y_ = 0.0;
  const double origin_orientation_z_ = 0.0;
  const double origin_orientation_w_ = 1.0;
  geometry_msgs::msg::Pose origin_;
  const double inflation_radius_ = 0.3;
  const double obstacle_cost_ = 100.0;
};

#endif
