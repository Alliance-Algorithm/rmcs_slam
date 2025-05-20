#pragma once

#include "util/pimpl.hpp"
#include <Eigen/Eigen>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace rmcs {

class Dedistortion {
    RMCS_PIMPL_DEFINTION(Dedistortion);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

public:
    using LidarData  = sensor_msgs::msg::PointCloud2;
    using ImuData    = sensor_msgs::msg::Imu;
    using Point      = pcl::PointXYZI;
    using PointCloud = pcl::PointCloud<pcl::PointXYZI>;

    struct Package {
        LidarData pointcloud;
        std::vector<ImuData> imu_data;
    };
};

}
