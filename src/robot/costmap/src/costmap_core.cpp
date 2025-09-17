#include "costmap_core.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : occupancygrid_data_(
  std::make_shared<nav_msgs::msg::OccupancyGrid>()), logger_(logger) {}

void CostmapCore::update(const sensor_msgs::msg::LaserScan::SharedPtr laserscan_msg)
{
  // fill with zeros
  occupancygrid_data_->data.assign(occupancygrid_data_->info.height * occupancygrid_data_->info.width, 0);

  // access laser scan data
  float angle_min = laserscan_msg->angle_min;
  float angle_max = laserscan_msg->angle_max;
  float angle_increment = laserscan_msg->angle_increment;
  float range_min = laserscan_msg->range_min;
  float range_max = laserscan_msg->range_max;
  std::vector<float> ranges = laserscan_msg->ranges;

  // iterate over the ranges detected
  float angle = angle_min;
  for (float range : ranges)
  {
    if (range < range_min || range > range_max)
    {
      angle += angle_increment;
      continue;
    }

    float x = range * cos(angle);
    float y = range * sin(angle);
    
    // convert to grid coordinates
    int grid_x = static_cast<int>((x - occupancygrid_data_->info.origin.position.x) / 
        occupancygrid_data_->info.resolution);
    int grid_y = static_cast<int>((y - occupancygrid_data_->info.origin.position.y) /
        occupancygrid_data_->info.resolution);
    
    // check if the point is within the grid bounds
    if (grid_x >= 0 && grid_x < occupancygrid_data_->info.width &&
        grid_y >= 0 && grid_y < occupancygrid_data_->info.height)
    {
      // mark the cell as occupied (100)
      occupancygrid_data_->data[grid_y * occupancygrid_data_->info.width + grid_x] = obstacle_cost_;
    }

    angle += angle_increment;
  }
}

void CostmapCore::initialize(
  int height, 
  int width, 
  double resolution, 
  geometry_msgs::msg::Pose origin, 
  double inflation_radius, 
  double obstacle_cost)
{
  // assign member fields
  occupancygrid_data_->info.height = height;
  occupancygrid_data_->info.width = width;
  occupancygrid_data_->info.resolution = resolution;
  occupancygrid_data_->info.origin = origin;
  occupancygrid_data_->data.assign(height * width, 0);

  // private member field
  inflation_radius_ = inflation_radius;
  obstacle_cost_ = obstacle_cost;
}

nav_msgs::msg::OccupancyGrid::SharedPtr CostmapCore::getCostmap()
{
  return occupancygrid_data_;
}

void CostmapCore::inflateObstacles()
{
  int inflation_cells = static_cast<int>(inflation_radius_ / occupancygrid_data_->info.resolution);
  std::vector<int8_t> inflated_data = occupancygrid_data_->data;

  for (int y = 0; y < occupancygrid_data_->info.height; ++y)
  {
    for (int x = 0; x < occupancygrid_data_->info.width; ++x)
    {
      if (occupancygrid_data_->data[y * occupancygrid_data_->info.width + x] == obstacle_cost_)
      {
        // Inflate around the occupied cell
        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy)
        {
          for (int dx = -inflation_cells; dx <= inflation_cells; ++dx)
          {
            int nx = x + dx;
            int ny = y + dy;
            if (nx >= 0 && nx < occupancygrid_data_->info.width &&
                ny >= 0 && ny < occupancygrid_data_->info.height)
            {
              float distance = sqrt(dx * dx + dy * dy) * occupancygrid_data_->info.resolution;
              if (distance <= inflation_radius_)
              {
                int index = ny * occupancygrid_data_->info.width + nx;
                // Calculate cost based on distance
                int cost = static_cast<int>(obstacle_cost_ * (1.0 - (distance / inflation_radius_)));
                if (cost > inflated_data[index])
                {
                  inflated_data[index] = std::min(cost, static_cast<int>(obstacle_cost_));
                }
              }
            }
          }
        }
      }
    }
  }

  occupancygrid_data_->data = inflated_data;
}

}
