#pragma once

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "../map/node.hpp"

namespace rmcs {

void livox_to_pcl(
    const std::vector<livox_ros_driver2::msg::CustomPoint>& livox,
    pcl::PointCloud<pcl::PointXYZ>& pcl);

void pc2_to_pcl(const sensor_msgs::msg::PointCloud2& pc2, pcl::PointCloud<pcl::PointXYZ>& pcl);

void pcl_to_pc2(const pcl::PointCloud<pcl::PointXYZ>& pcl, sensor_msgs::msg::PointCloud2& pc2);

void node_to_grid_map(ObstacleMap& nodes, nav_msgs::msg::OccupancyGrid& occupancy);

}; // namespace rmcs
