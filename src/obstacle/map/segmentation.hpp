#pragma once
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class Segmentation {
public:
    using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

    explicit Segmentation();
    ~Segmentation();
    Segmentation(const Segmentation&)            = delete;
    Segmentation& operator=(const Segmentation&) = delete;

    void set_input_source(const std::shared_ptr<PointCloud>& source);
    std::shared_ptr<PointCloud> execute();

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};
