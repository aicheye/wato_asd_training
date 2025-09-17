#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    void update(sensor_msgs::msg::LaserScan::SharedPtr laserscan_msg);

    void initialize(
      int height, 
      int width, 
      double resolution, 
      geometry_msgs::msg::Pose origin, 
      double inflation_radius, 
      double obstacle_cost
    );
    
    nav_msgs::msg::OccupancyGrid::SharedPtr getCostmap();

    void inflateObstacles();

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid::SharedPtr occupancygrid_data_;
    double inflation_radius_;
    double obstacle_cost_;
};

}

#endif  
