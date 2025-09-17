/**
 * @file costmap_node.cpp
 * @author Sean Yang
 * @brief This node subscribes to LIDAR data and generates a 2D occupancy grid costmap.
 * @date 2025-09-10
 */

#include <chrono>
#include <memory>
#include <vector>
#include <cmath>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"),
                             costmap_(robot::CostmapCore(this->get_logger()))
{
  // initialize subscriber and publisher
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  // initialize costmap
  origin_.set__position(geometry_msgs::msg::Point()
                            .set__x(origin_x_)
                            .set__y(origin_y_)
                            .set__z(origin_z_));
  origin_.set__orientation(geometry_msgs::msg::Quaternion()
                               .set__x(origin_orientation_x_)
                               .set__y(origin_orientation_y_)
                               .set__z(origin_orientation_z_)
                               .set__w(origin_orientation_w_));
  costmap_.initialize(width_, height_, resolution_, origin_, inflation_radius_, obstacle_cost_);

  RCLCPP_INFO(this->get_logger(), "CostmapNode has been initialized.");
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr laserscan_msg)
{
  costmap_.update(laserscan_msg);
  RCLCPP_INFO(this->get_logger(), "Publishing occupancy grid");
  costmap_pub_->publish(*costmap_.getCostmap());
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
