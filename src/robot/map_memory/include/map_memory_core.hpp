#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace robot
{

  class MapMemoryCore
  {
  public:
    explicit MapMemoryCore(const rclcpp::Logger &logger);

    void initialize(int height, int width, double resolution, geometry_msgs::msg::Pose origin, int unknown_cost);

    void integrateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap, const geometry_msgs::msg::TransformStamped::SharedPtr transform);

    nav_msgs::msg::OccupancyGrid getMap();

  private:
    rclcpp::Logger logger_;

    nav_msgs::msg::OccupancyGrid::SharedPtr global_map_;

    geometry_msgs::msg::Transform::SharedPtr net_transform_;

    int unknown_cost_;
  };

}

#endif
