#pragma once
#include "util/pimpl.hpp"
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class Segmentation {
    RMCS_PIMPL_DEFINTION(Segmentation)
public:
    using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

    void set_input_source(const std::shared_ptr<PointCloud>& source);
    std::shared_ptr<PointCloud> execute();

    void set_limit_distance(double v);
    void set_limit_max_height(double v);
    void set_distance_threshold(double v);
    void set_ground_max_height(double v);
};
