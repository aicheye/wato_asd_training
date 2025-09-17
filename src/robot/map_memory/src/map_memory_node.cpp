#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger()))
{
  // initialize subscribers and publisher
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // initialize timer
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / publish_frequency_)),
      std::bind(&MapMemoryNode::timerCallback, this));

  // initialize map memory
  origin_.position.x = origin_x_;
  origin_.position.y = origin_y_;
  origin_.position.z = origin_z_;
  origin_.orientation.x = origin_orientation_x_;
  origin_.orientation.y = origin_orientation_y_;
  origin_.orientation.z = origin_orientation_z_;
  origin_.orientation.w = origin_orientation_w_;
  map_memory_.initialize(height_, width_, resolution_, origin_, unknown_cost_);

  RCLCPP_INFO(this->get_logger(), "MapMemoryNode has been initialized.");
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap)
{
  latest_costmap_ = costmap;
  costmap_received_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  if (latest_odom_ == nullptr)
  {
    latest_odom_ = odom;
    return;
  }

  double current_x = odom->pose.pose.position.x;
  double current_y = odom->pose.pose.position.y;

  double last_x = latest_odom_->pose.pose.position.x;
  double last_y = latest_odom_->pose.pose.position.y;

  // compute distance moved
  double distance_moved = std::hypot(current_x - last_x, current_y - last_y);

  if (distance_moved >= distance_threshold_)
  {
    latest_odom_ = odom;
    should_update_map_ = true;
  }
}

void MapMemoryNode::timerCallback()
{
  if (costmap_received_ && should_update_map_)
  {
    map_memory_.integrateMap(latest_costmap_, latest_odom_);
    RCLCPP_INFO(this->get_logger(), "Publishing updated map");
    map_pub_->publish(map_memory_.getMap());
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
