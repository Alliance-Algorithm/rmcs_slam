#pragma once

#include "common/pimpl.hpp"

#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace rmcs {

class RosUtil {
    RMCS_PIMPL_DEFINTION(RosUtil)

public:
    void publish_frame_world(
        const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher);

    void publish_frame_body(
        const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher);

    void publish_effect_world(
        const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher);

    void publish_map(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher);

    void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& publisher);

    void publish_odometry(
        const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr& publisher,
        const std::unique_ptr<tf2_ros::TransformBroadcaster>& broadcaster);
};

} // namespace rmcs
