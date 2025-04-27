#include "process.hpp"
#include "filter.hpp"
#include "node.hpp"

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <rclcpp/logging.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <type_traits>

template <typename RangeType, typename ValueType>
bool in_range(const ValueType& val, const RangeType& min, const RangeType& max) {
    return (val > min && val < max);
}

Process::Process() { info("map process has been constructed"); };

void Process::info(const std::string& string) const {
    static const auto logger = rclcpp::get_logger("process_for_cloud");

    if (enable_info_)
        RCLCPP_INFO(logger, "%s", string.c_str());
}

std::unique_ptr<type::NodeMap>
    Process::generate_node_map(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& pointcloud) {
    using PointType = std::remove_cvref<decltype((*pointcloud)[0])>::type;
    using namespace type;

    // remove blind points
    auto inside = std::make_shared<pcl::ConditionOr<PointType>>();
    inside->addComparison(
        std::make_shared<const pcl::FieldComparison<PointType>>(
            "x", pcl::ComparisonOps::LT, -lidar_blind_ / 2.0));
    inside->addComparison(
        std::make_shared<const pcl::FieldComparison<PointType>>(
            "x", pcl::ComparisonOps::GT, lidar_blind_ / 2.0));
    inside->addComparison(
        std::make_shared<const pcl::FieldComparison<PointType>>(
            "y", pcl::ComparisonOps::LT, -lidar_blind_ / 2.0));
    inside->addComparison(
        std::make_shared<const pcl::FieldComparison<PointType>>(
            "y", pcl::ComparisonOps::GT, lidar_blind_ / 2.0));

    auto condition = std::make_shared<pcl::ConditionAnd<PointType>>();
    condition->addCondition(inside);

    auto filter = pcl::ConditionalRemoval<PointType>{};
    filter.setCondition(condition);
    filter.setInputCloud(pointcloud);

    auto pointcloud_remove_blind = std::make_shared<pcl::PointCloud<PointType>>();
    filter.filter(*pointcloud_remove_blind);

    // load height
    auto map = std::make_unique<type::NodeMap>(grid_number_, grid_number_);
    for (const auto& point : *pointcloud_remove_blind) {
        auto x = int((point.x + (grid_width_ / 2.0)) / resolution_);
        auto y = int((point.y + (grid_width_ / 2.0)) / resolution_);

        if (!in_range(x, size_t(0), size_t(grid_number_))
            || !in_range(y, size_t(0), size_t(grid_number_)))
            continue;

        auto& node  = (*map)(x, y);
        node.height = std::max(node.height, point.z);
    }

    // load value
    for (auto& node : **map) {
        /// @note 2025.3.29 上海联盟赛记
        ///       点云太高，经过缩放后，在 int8_t 的上界处溢出至下界
        auto height = std::clamp(node.height / 0.6 * 100.0, 0., 100.);
        node.value  = std::clamp(
            static_cast<int8_t>(height), static_cast<int8_t>(0), static_cast<int8_t>(100));
    }

    filter::handle(*map);

    return map;
}

std::unique_ptr<type::NodeMap>
    Process::generate_cost_map(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& pointcloud) {
    (void)this;
    return nullptr;
}
