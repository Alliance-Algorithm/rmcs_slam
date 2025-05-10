#pragma once

#include "util/pimpl.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace rmcs {

class DescriptorManager {
    RMCS_PIMPL_DEFINTION(DescriptorManager);

public:
    using PointT      = pcl::PointXYZ;
    using PointCloudT = pcl::PointCloud<PointT>;

    void debug() const;

    void make_context(const std::shared_ptr<PointCloudT>& pointcloud);

    std::pair<Eigen::Isometry3f, PointCloudT> query(const std::shared_ptr<PointCloudT>& pointcloud);
};

} // namespace rmcs
