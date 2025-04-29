#pragma once

#include <memory>

#include <Eigen/Dense>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace rmcs {

/// @brief 双激光雷达合成器
class Synthesizer {
public:
    using LivoxMsg = livox_ros_driver2::msg::CustomMsg;
    using ImuMsg   = sensor_msgs::msg::Imu;

    explicit Synthesizer();
    ~Synthesizer();

    Synthesizer(const Synthesizer&)            = delete;
    Synthesizer& operator=(const Synthesizer&) = delete;

    void initialize(rclcpp::Node& node);

    void set_main_transform(
        const Eigen::Quaterniond& rotation, const Eigen::Translation3d& translation);
    void set_auxiliary_transform(
        const Eigen::Quaterniond& rotation, const Eigen::Translation3d& translation);

    void upload_main_pointcloud(const std::unique_ptr<LivoxMsg>& msg);
    void upload_main_imu(const std::unique_ptr<ImuMsg>& msg);

    void upload_auxiliary_pointcloud(const std::unique_ptr<LivoxMsg>& msg);
    void upload_auxiliary_imu(const std::unique_ptr<ImuMsg>& msg);

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

} // namespace rmcs
