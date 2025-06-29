#pragma once

#include "sophus/se3.hpp"
#include "util/pimpl.hpp"
#include <Eigen/Eigen>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace rmcs {

class Orthotics {
    RMCS_PIMPL_DEFINTION(Orthotics);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

public:
    using LidarData  = sensor_msgs::msg::PointCloud2;
    using ImuData    = sensor_msgs::msg::Imu;
    using Point      = pcl::PointXYZI;
    using PointCloud = pcl::PointCloud<pcl::PointXYZI>;

    struct Package {
        std::unique_ptr<LidarData> pointcloud;
        std::vector<std::unique_ptr<ImuData>> imu_data;
    };

    auto update(const Package& package) -> void;
    auto reset() -> void;

    auto set_transform(const Eigen::Isometry3d& transform) -> void;

    auto undistort(std::shared_ptr<PointCloud>& pointcloud, double interval,
        const Sophus::SE3d& transform) const -> void;
};

}
