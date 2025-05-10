#pragma once

#include "util/pimpl.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/node.hpp>

namespace rmcs {

class Runtime {
    RMCS_PIMPL_DEFINTION(Runtime);

public:
    using Point      = pcl::PointXYZ;
    using PointCloud = pcl::PointCloud<Point>;

    void initialize(rclcpp::Node& node);
};

} // namespace rmcs
