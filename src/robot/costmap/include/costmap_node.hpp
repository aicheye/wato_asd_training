#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
public:
  CostmapNode();

  // Place callback function here
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr laserscan_msg);

  void publishCostmap();
  
  void initializeCostmap(int rows, int cols);


private:
  robot::CostmapCore costmap_;
  // Place these constructs here
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
};

#endif
