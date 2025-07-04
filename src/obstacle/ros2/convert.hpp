#pragma once

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "../map/node.hpp"

namespace rmcs {
namespace convert_internal {
    template <class Point>
    concept concept_point = requires(Point point) {
        { point.x } -> std::convertible_to<float>;
        { point.y } -> std::convertible_to<float>;
        { point.z } -> std::convertible_to<float>;
    };
}

using LivoxPoint = livox_ros_driver2::msg::CustomPoint;

template <convert_internal::concept_point Point>
inline auto livox_to_pcl(const std::vector<LivoxPoint>& livox, pcl::PointCloud<Point>& pcl)
    -> void {
    for (const auto point : livox)
        pcl.points.emplace_back(point.x, point.y, point.z);
}

template <convert_internal::concept_point Point>
inline auto livox_to_pcl(const std::vector<LivoxPoint>& livox, pcl::PointCloud<Point>& pcl,
    const Eigen::Isometry3d& t) -> void {
    for (const auto point : livox) {
        Eigen::Vector3d transformed = t * Eigen::Vector3d { point.x, point.y, point.z };
        pcl.points.emplace_back(transformed.x(), transformed.y(), transformed.z());
    }
}

inline auto pc2_to_pcl(
    const sensor_msgs::msg::PointCloud2& pc2, pcl::PointCloud<pcl::PointXYZ>& pcl) -> void {
    pcl::fromROSMsg(pc2, pcl);
}

inline auto pcl_to_pc2(
    const pcl::PointCloud<pcl::PointXYZ>& pcl, sensor_msgs::msg::PointCloud2& pc2) -> void {
    pcl::toROSMsg(pcl, pc2);
}

inline auto node_to_grid_map(ObstacleMap& nodes, nav_msgs::msg::OccupancyGrid& occupancy) -> void {
    const auto width = nodes.width();

    occupancy.info.width  = width;
    occupancy.info.height = width;
    occupancy.data.resize(width * width);
    for (auto x = 0; x < width; x++)
        for (auto y = 0; y < width; y++)
            occupancy.data[x + y * width] = nodes(x, y).value;
};

};
