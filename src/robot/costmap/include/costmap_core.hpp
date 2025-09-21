/**
 * @file costmap_core.hpp
 * @author Sean Yang
 * @brief Header file for the CostmapCore class that handles costmap generation and updating.
 * @date 2025-09-10
 */

#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace robot
{

  class CostmapCore
  {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger &logger);

    void update(const sensor_msgs::msg::LaserScan::SharedPtr laserscan_msg);

    void initialize(int height, int width, double resolution, geometry_msgs::msg::Pose origin, double inflation_radius,
                    double obstacle_cost);

    nav_msgs::msg::OccupancyGrid::SharedPtr getCostmap();

    bool has_data_;

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid::SharedPtr occupancygrid_data_;
    double inflation_radius_;
    double obstacle_cost_;

    void inflateObstacles();
  };

} // namespace robot

#endif
