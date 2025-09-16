#include <chrono>
#include <memory>
#include <vector>
#include <cmath>

#include "costmap_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

const int RESOLUTION = 0.1;
std::vector<std::vector<short>> costmap;

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger()))
{
  // Initialize the constructs and their parameters
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::initializeCostmap(int rows, int cols)
{
  costmap = std::vector<std::vector<short>>(rows, std::vector<short>(cols, 0));
}

// Define the timer to publish a message every 500ms
void CostmapNode::publishCostmap()
{
  nav_msgs::msg::OccupancyGrid occupancygrid_msg = nav_msgs::msg::OccupancyGrid();
  occupancygrid_msg.info.resolution = RESOLUTION;

  RCLCPP_INFO(this->get_logger(), "Publishing occupancy grid");
  costmap_pub_->publish(occupancygrid_msg);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr laserscan_msg)
{
  float angle_min = laserscan_msg->angle_min;
  float angle_max = laserscan_msg->angle_max;
  float angle_increment = laserscan_msg->angle_increment;
  float range_min = laserscan_msg->range_min;
  float range_max = laserscan_msg->range_max;
  std::vector<float> ranges = laserscan_msg->ranges;

  //CostmapNode::initializeCostmap((int)((range_max * sin(angle_max)) / RESOLUTION),
  //                               (int)((range_max * cos(angle_max) / RESOLUTION)));

  float angle = angle_min;
  for (float range : ranges)
  {
    RCLCPP_INFO(this->get_logger(), "Scan at %d rad returned %d m", angle, range);
    if (range < range_min || range > range_max)
    {
      angle += angle_increment;
      continue;
    }

    float x = range * cos(angle);
    float y = range * sin(angle);
    RCLCPP_INFO(this->get_logger(), "Lidar detected object at (%d, %d)", x, y);

    angle += angle_increment;
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
