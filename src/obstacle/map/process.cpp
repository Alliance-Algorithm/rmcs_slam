#include "process.hpp"
#include "filter.hpp"
#include "node.hpp"
#include "ros2/param.hpp"

#include <Eigen/Eigen>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
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
template <>
struct hash<std::pair<std::size_t, std::size_t>> {
    std::size_t operator()(const std::pair<std::size_t, std::size_t>& p) const {
        auto h1 = std::hash<std::size_t>{}(p.first);
        auto h2 = std::hash<std::size_t>{}(p.second);
        return h1 ^ h2;
    }
};
} // namespace std

struct Process::Impl {
    explicit Impl() {
        height_limit  = param::get<float>("grid.height_limit");
        ground_height = param::get<float>("grid.ground_height");
        resolution    = param::get<float>("grid.resolution");
        lidar_blind   = param::get<float>("grid.lidar_blind");
        map_width     = param::get<float>("grid.grid_width");
        points_limit  = param::get<std::size_t>("grid.points_limit");
        side_num      = static_cast<int>(map_width / resolution);
    }

    std::size_t points_limit;
    std::size_t side_num;
    float height_limit;
    float ground_height;
    float resolution;
    float lidar_blind;
    float map_width;
};

std::unique_ptr<ObstacleMap>
    Process::generate_node_map(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& pointcloud) {
    using PointType = std::remove_cvref<decltype((*pointcloud)[0])>::type;

    // remove blind points
    auto inside = std::make_shared<pcl::ConditionOr<PointType>>();
    inside->addComparison(
        std::make_shared<const pcl::FieldComparison<PointType>>(
            "x", pcl::ComparisonOps::LT, -pimpl->lidar_blind / 2.0));
    inside->addComparison(
        std::make_shared<const pcl::FieldComparison<PointType>>(
            "x", pcl::ComparisonOps::GT, +pimpl->lidar_blind / 2.0));
    inside->addComparison(
        std::make_shared<const pcl::FieldComparison<PointType>>(
            "y", pcl::ComparisonOps::LT, -pimpl->lidar_blind / 2.0));
    inside->addComparison(
        std::make_shared<const pcl::FieldComparison<PointType>>(
            "y", pcl::ComparisonOps::GT, +pimpl->lidar_blind / 2.0));

    auto condition = std::make_shared<pcl::ConditionAnd<PointType>>();
    condition->addCondition(inside);

    auto filter = pcl::ConditionalRemoval<PointType>{};
    filter.setCondition(condition);
    filter.setInputCloud(pointcloud);

    auto pointcloud_remove_blind = std::make_shared<pcl::PointCloud<PointType>>();
    filter.filter(*pointcloud_remove_blind);

    // 加载可视域内节点的高度信息
    // 使用过的节点，用于二次更新时减少循环次数
    auto visited_node = std::unordered_set<std::pair<std::size_t, std::size_t>>{};
    // 障碍地图
    auto obstacle_map = ObstacleMap{pimpl->side_num};
    for (const auto point : *pointcloud_remove_blind) {
        const auto f = [this](float v) -> std::size_t {
            return std::clamp(
                static_cast<std::size_t>((v + (pimpl->map_width / 2.)) / pimpl->resolution),
                std::size_t{0}, pimpl->side_num);
        };
        const auto x = f(point.x), y = f(point.y);
        visited_node.insert(std::make_pair(x, y));

        auto& node = obstacle_map(x, y);
        node.points += 1;
        node.max = std::max(node.max, point.z);
        node.min = std::min(node.min, point.z);
    }
    // 二次更新，加载障碍物信息
    for (const auto [x, y] : visited_node) {
        auto& node = obstacle_map(x, y);
        if (node.points < pimpl->points_limit)
            continue;
        if (node.max < pimpl->height_limit)
            continue;
        obstacle_map.update_node(x, y, 100);
    }
    // 三次更新，作可行域射线投射
    filter_map(obstacle_map);
    obstacle_map.ray_cast();

    return std::make_unique<ObstacleMap>(std::move(obstacle_map));
}

Process::Process()
    : pimpl(std::make_unique<Impl>()) {}

Process::~Process() = default;

float Process::resolution() const { return pimpl->resolution; }

float Process::map_width() const { return pimpl->map_width; }

uint Process::size_num() const { return pimpl->side_num; }
