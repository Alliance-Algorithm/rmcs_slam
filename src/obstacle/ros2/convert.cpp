#include "convert.hpp"
#include "../map/node.hpp"

#include <pcl_conversions/pcl_conversions.h>

void ros2::convert::livox_to_pcl(
    const std::vector<livox_ros_driver2::msg::CustomPoint>& livox,
    pcl::PointCloud<pcl::PointXYZ>& pcl) {
    for (const auto point : livox)
        pcl.points.emplace_back(point.x, point.y, point.z);
}

void ros2::convert::pc2_to_pcl(
    const sensor_msgs::msg::PointCloud2& pc2, pcl::PointCloud<pcl::PointXYZ>& pcl) {
    pcl::fromROSMsg(pc2, pcl);
}

void ros2::convert::pcl_to_pc2(
    const pcl::PointCloud<pcl::PointXYZ>& pcl, sensor_msgs::msg::PointCloud2& pc2) {
    pcl::toROSMsg(pcl, pc2);
}

void ros2::convert::node_to_grid_map(
    type::NodeMap& node_map, nav_msgs::msg::OccupancyGrid& occupancy_map) {
    occupancy_map.info.width  = node_map.width();
    occupancy_map.info.height = node_map.length();

    occupancy_map.data = std::vector<int8_t>(node_map.width() * node_map.length());
    for (const auto& node : *node_map)
        occupancy_map.data[node.x + node.y * node_map.width()] = node.value;
};
