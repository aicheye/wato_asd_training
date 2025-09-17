#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    void initialize(int height, int width, double resolution, geometry_msgs::msg::Pose origin, int unknown_cost);

    void integrateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap, const nav_msgs::msg::Odometry::SharedPtr odom);

    nav_msgs::msg::OccupancyGrid getMap();

  private:
    rclcpp::Logger logger_;

    nav_msgs::msg::OccupancyGrid::SharedPtr global_map_;
};

}  

#endif  
