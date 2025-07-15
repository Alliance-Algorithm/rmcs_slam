#pragma once

#include "util/pimpl.hpp"
#include <Eigen/Eigen>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace rmcs {

template <class Point>
concept concept_point = requires(Point point) {
    { point.x } -> std::convertible_to<float>;
    { point.y } -> std::convertible_to<float>;
    { point.z } -> std::convertible_to<float>;
};

class ImuOrthotics {
    RMCS_PIMPL_DEFINTION(ImuOrthotics);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

public:
    using ImuMsg = sensor_msgs::msg::Imu;
    using LivMsg = livox_ros_driver2::msg::CustomMsg;

    struct MessageGroup {
        std::unique_ptr<LivMsg> lid_msg;
        std::vector<std::unique_ptr<ImuMsg>> imu_msg;
    };

    using CloudXYZ = pcl::PointCloud<pcl::PointXYZ>;
    auto process(std::shared_ptr<CloudXYZ>& output, const MessageGroup& package) -> void;

    using CloudXYZI = pcl::PointCloud<pcl::PointXYZI>;
    auto process(std::shared_ptr<CloudXYZI>& output, const MessageGroup& package) -> void;

    auto set_lid_transform(const Eigen::Isometry3d& transform) -> void;
    auto set_imu_transform(const Eigen::Isometry3d& transform) -> void;

    auto reset() -> void;
};

}
