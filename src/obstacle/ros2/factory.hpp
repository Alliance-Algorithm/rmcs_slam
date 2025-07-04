#pragma once

#include "util/pimpl.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/node.hpp>
#include <std_msgs/msg/header.hpp>

namespace rmcs {

class Factory {
    RMCS_PIMPL_DEFINTION(Factory);

public:
    explicit Factory(rclcpp::Node& node) noexcept;

    auto set_lid_assembly_transform(const Eigen::Isometry3d& t) -> Factory&;

    auto set_imu_extrinsic_transform(const Eigen::Isometry3d& t) -> Factory&;

    auto set_lid_topic_name(const std::string& topic) -> Factory&;

    auto set_imu_topic_name(const std::string& topic) -> Factory&;

    using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
    using Callback   = std::function<void(const std::shared_ptr<PointCloud>&, //
        const std_msgs::msg::Header&)>;
    auto build_normal_mode(const Callback& process) -> void;
    auto build_unditort_mode(const Callback& process) -> void;
};

}
