#pragma once

#include "util/pimpl.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace rmcs {

class Dedistortion {
    RMCS_PIMPL_DEFINTION(Dedistortion);

    using LidarData = sensor_msgs::msg::PointCloud2;
    using ImuData   = sensor_msgs::msg::Imu;
};

}
