/**
 * @file map_memory_core.cpp
 * @author Sean Yang
 * @brief Logic for merging multiple costmaps within the robot frame into
 *        a global map in the simulation frame.
 * @date 2025-09-21
 */

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
    global_map_->header.frame_id = "sim_world";
    global_map_->info.height = height;
    global_map_->info.width = width;
    global_map_->info.resolution = resolution;
    global_map_->info.origin = origin;
    global_map_->data = std::vector<int8_t>(height * width, unknown_cost); // unknown
    RCLCPP_INFO(logger_, "Initialized global map with size %dx%d, resolution %.2f",
                width, height, resolution);
  }

  void MapMemoryCore::integrateMap(
      const nav_msgs::msg::OccupancyGrid::SharedPtr costmap, const geometry_msgs::msg::TransformStamped::SharedPtr transform)
  {
    if (!costmap)
    {
      RCLCPP_WARN(logger_, "Costmap is null, skipping integration");
      return;
    }

    if (!transform)
    {
      RCLCPP_WARN(logger_, "Transform is null, skipping integration");
      return;
    }

    for (size_t i = 0; i < costmap->data.size(); ++i)
    {
      int costmap_x_idx = i % costmap->info.width;
      int costmap_y_idx = i / costmap->info.width;
      double costmap_x = (costmap_x_idx + 0.5) * costmap->info.resolution + costmap->info.origin.position.x;
      double costmap_y = (costmap_y_idx + 0.5) * costmap->info.resolution + costmap->info.origin.position.y;

      geometry_msgs::msg::PointStamped costmap_point;
      costmap_point.header = costmap->header;
      costmap_point.point.x = costmap_x;
      costmap_point.point.y = costmap_y;
      costmap_point.point.z = 0.0;

      geometry_msgs::msg::PointStamped transformed_point;
      tf2::doTransform(costmap_point, transformed_point, *transform);

      // map transformed point to global map indices
      int global_x = static_cast<int>(std::round((transformed_point.point.x - global_map_->info.origin.position.x) / global_map_->info.resolution));
      int global_y = static_cast<int>(std::round((transformed_point.point.y - global_map_->info.origin.position.y) / global_map_->info.resolution));

      if (global_x >= 0 && global_x < static_cast<int>(global_map_->info.width) &&
          global_y >= 0 && global_y < static_cast<int>(global_map_->info.height))
      {
        size_t global_idx = global_y * global_map_->info.width + global_x;
        if (costmap->data[i] > global_map_->data[global_idx])
        {
          global_map_->data[global_idx] = costmap->data[i];
        }
      }
    }

    global_map_->header.stamp = costmap->header.stamp;
    global_map_->info.map_load_time = costmap->header.stamp;
  }

  nav_msgs::msg::OccupancyGrid::SharedPtr MapMemoryCore::getMap()
  {
    return global_map_;
  }
}
