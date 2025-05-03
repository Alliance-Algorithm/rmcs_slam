#pragma once

#include <cstddef>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace rmcs {

template <std::size_t index>
struct pointcloud_util {
public:
    template <typename Point>
    static void publish(const pcl::PointCloud<Point>& pointcloud, const std::string& frame) {
        static auto node      = rclcpp::Node{"pointcloud_util_" + std::to_string(index)};
        static auto publisher = node.create_publisher<sensor_msgs::msg::PointCloud2>(
            "/pointcloud_util/index_" + std::to_string(index), 10);

        auto msg = sensor_msgs::msg::PointCloud2{};
        pcl::toROSMsg(pointcloud, msg);
        msg.header.frame_id = frame;
        publisher->publish(msg);
    }
};

} // namespace rmcs
