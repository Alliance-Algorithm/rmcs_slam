#pragma once

#include "../util/pimpl.hpp"

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
    void initialize(rclcpp::Node& node);

    void stop_service();

    void start_service();

    void publish_pointcloud_registerd_world(const sensor_msgs::msg::PointCloud2& msg) const;

    void publish_pointcloud_registerd_body(const sensor_msgs::msg::PointCloud2& msg) const;

    void publish_pointcloud_effect_world(const sensor_msgs::msg::PointCloud2& msg) const;

    void publish_constructed_map(const sensor_msgs::msg::PointCloud2& msg) const;

    void publish_pose(const geometry_msgs::msg::PoseStamped& msg) const;

    void publish_odometry(const nav_msgs::msg::Odometry& msg) const;

    void publish_path(const nav_msgs::msg::Path& msg) const;

    void update_transform(const geometry_msgs::msg::TransformStamped& stamp) const;

    void register_map_save_function(const std::function<void(void)>& fun);

    void register_reset_function(const std::function<void(void)>& fun);

    void register_update_function(const std::function<void(void)>& fun);

    void register_publish_map_function(const std::function<void(void)>& fun);

    void register_switch_record_function(const std::function<void(bool)>& fun);
};

} // namespace rmcs
