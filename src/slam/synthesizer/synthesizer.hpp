#pragma once

#include <memory>

#include <Eigen/Dense>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace rmcs {

/// @brief 双激光雷达合成器
class Synthesizer {
public:
    using LivoxMsg   = livox_ros_driver2::msg::CustomMsg;
    using ImuMsg     = sensor_msgs::msg::Imu;
    using Point      = pcl::PointXYZ;
    using PointCloud = pcl::PointCloud<Point>;

    explicit Synthesizer();
    ~Synthesizer();

    Synthesizer(const Synthesizer&)            = delete;
    Synthesizer& operator=(const Synthesizer&) = delete;

    void initialize(rclcpp::Node& node);

    void register_callback(
        const std::function<void(const std::unique_ptr<LivoxMsg>&)>& livox_callback,
        const std::function<void(const std::unique_ptr<ImuMsg>&)>& imu_callback);

    void switch_record(bool on);

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

} // namespace rmcs
