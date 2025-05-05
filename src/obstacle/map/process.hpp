#pragma once

#include "node.hpp"
#include "util/logger.hpp"
#include "util/pimpl.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace rmcs {

class Process {
    RMCS_PIMPL_DEFINTION(Process);

public:
    std::unique_ptr<ObstacleMap>
        generate_node_map(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& pointcloud);

    float resolution() const;
    float map_width() const;
    uint size_num() const;

private:
    RMCS_INITIALIZE_LOGGER("rmcs-map");
};

} // namespace rmcs
