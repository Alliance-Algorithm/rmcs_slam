#include "process.hpp"
#include "filter.hpp"
#include "node.hpp"
#include "ros2/param.hpp"

#include <Eigen/Eigen>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <rclcpp/logging.hpp>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <type_traits>

using namespace rmcs;

namespace std {
template <> struct hash<std::pair<std::size_t, std::size_t>> {
    std::size_t operator()(const std::pair<std::size_t, std::size_t>& p) const {
        auto h1 = std::hash<std::size_t> {}(p.first);
        auto h2 = std::hash<std::size_t> {}(p.second);
        return h1 ^ h2;
    }
};
} // namespace std

struct Process::Impl {
    std::size_t points_limit;
    std::size_t side_num;
    float height_limit;
    float ground_height;
    float resolution;
    float lidar_blind;
    float map_width;
    float influence_radius;

    explicit Impl() {
        height_limit     = param::get<float>("grid.height_limit");
        ground_height    = param::get<float>("grid.ground_height");
        resolution       = param::get<float>("grid.resolution");
        lidar_blind      = param::get<float>("grid.lidar_blind");
        map_width        = param::get<float>("grid.map_width");
        points_limit     = param::get<std::size_t>("grid.points_limit");
        influence_radius = param::get<float>("grid.influence_radius");
        side_num         = static_cast<int>(map_width / resolution);
    }
};

std::unique_ptr<ObstacleMap> Process::generate_node_map(
    const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& pointcloud) {
    using Point     = std::remove_cvref<decltype((*pointcloud)[0])>::type;
    auto resolution = pimpl->resolution;
    auto map_width  = pimpl->map_width;
    auto side_num   = pimpl->side_num;

    // 加载可视域内节点的高度信息
    // 使用过的节点，用于二次更新时减少循环次数
    auto visited_node = std::unordered_set<std::pair<std::size_t, std::size_t>> {};
    // 障碍地图
    auto obstacle_map = ObstacleMap { side_num };
    for (const auto point : *pointcloud) {
        const auto f = [&](float v) -> std::size_t {
            return std::clamp(static_cast<std::size_t>((v + (map_width / 2.)) / resolution),
                std::size_t { 0 }, side_num - 1);
        };
        const auto x = f(point.x), y = f(point.y);

        const auto expand = static_cast<std::size_t>(pimpl->influence_radius / pimpl->resolution);
        obstacle_map.update_round_area(
            x, y, expand, [&](std::size_t x, std::size_t y, ObstacleMap::Node& node) {
                visited_node.insert(std::make_pair(x, y));
                node.update_height_table(point.z);
            });
    }
    // 二次更新，加载障碍物信息
    for (const auto [x, y] : visited_node) {
        auto& node = obstacle_map(x, y);
        if (node.height_table_size() < pimpl->points_limit) continue;
        if (node.maximum_height_range() < pimpl->height_limit) continue;
        obstacle_map.update_node(x, y, 100);
    }

    // 去除盲区
    auto blind_radius = pimpl->lidar_blind / 2;
    auto grid_radius  = static_cast<std::size_t>(blind_radius / pimpl->resolution);
    obstacle_map.fill_center(grid_radius, -1);

    // 三次更新，作可行域射线投射
    filter_map(obstacle_map);
    obstacle_map.ray_cast_with_infinty_unknown();

    return std::make_unique<ObstacleMap>(std::move(obstacle_map));
}

Process::Process()
    : pimpl(std::make_unique<Impl>()) { }

Process::~Process() = default;

float Process::resolution() const { return pimpl->resolution; }

float Process::map_width() const { return pimpl->map_width; }

uint Process::size_num() const { return pimpl->side_num; }
