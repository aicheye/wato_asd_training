#include "map_memory_core.hpp"

namespace robot
{

  MapMemoryCore::MapMemoryCore(const rclcpp::Logger &logger)
      : global_map_(std::make_shared<nav_msgs::msg::OccupancyGrid>()), logger_(logger) {}

  void MapMemoryCore::initialize(
    int height, 
    int width, 
    double resolution, 
    geometry_msgs::msg::Pose origin, 
    int unknown_cost)
  {
    global_map_->info.height = height;
    global_map_->info.width = width;
    global_map_->info.resolution = resolution;
    global_map_->info.origin = origin;
    global_map_->data = std::vector<int8_t>(height * width, unknown_cost); // unknown
    RCLCPP_INFO(logger_, "Initialized global map with size %dx%d, resolution %.2f", 
      width, height, resolution);
  }

  void MapMemoryCore::integrateMap(
    const nav_msgs::msg::OccupancyGrid::SharedPtr costmap, 
    const nav_msgs::msg::Odometry::SharedPtr odom)
  {
    // Transform costmap to global map frame using odometry
    if (!costmap || !odom) {
      RCLCPP_WARN(logger_, "Costmap or odometry is null, skipping integration");
      return;
    }

    double robot_x = odom->pose.pose.position.x;
    double robot_y = odom->pose.pose.position.y;
    double robot_yaw = costmap->info.origin.orientation.z; // assuming flat ground and yaw only
  }

  nav_msgs::msg::OccupancyGrid MapMemoryCore::getMap()
  {
    return *global_map_;
  }
}
