#include "convert.hpp"
#include "../map/node.hpp"

#include <pcl_conversions/pcl_conversions.h>

namespace rmcs {

void livox_to_pcl(
    const std::vector<livox_ros_driver2::msg::CustomPoint>& livox,
    pcl::PointCloud<pcl::PointXYZ>& pcl) {
    for (const auto point : livox)
        pcl.points.emplace_back(point.x, point.y, point.z);
}

void pc2_to_pcl(const sensor_msgs::msg::PointCloud2& pc2, pcl::PointCloud<pcl::PointXYZ>& pcl) {
    pcl::fromROSMsg(pc2, pcl);
}

void pcl_to_pc2(const pcl::PointCloud<pcl::PointXYZ>& pcl, sensor_msgs::msg::PointCloud2& pc2) {
    pcl::toROSMsg(pcl, pc2);
}

void node_to_grid_map(ObstacleMap& nodes, nav_msgs::msg::OccupancyGrid& occupancy) {
    const auto width = nodes.width();

    occupancy.info.width  = width;
    occupancy.info.height = width;

    occupancy.data = std::vector<int8_t>(width * width);
    for (auto x = 0; x < width; x++)
        for (auto y = 0; y < width; y++) {
            occupancy.data[x + y * width] = nodes(x, y).value;
        }
};
} // namespace rmcs
