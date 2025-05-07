#pragma once
#include "util/logger.hpp"

#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/int32.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>

class MapNode : public rclcpp::Node {
public:
    explicit MapNode();
    ~MapNode() override;
    MapNode(const MapNode&)            = delete;
    MapNode& operator=(const MapNode&) = delete;

private:
    RMCS_INITIALIZE_LOGGER("rmcs-map");

    struct Impl;
    std::unique_ptr<Impl> pimpl;

private:
    // @brief handle the pcl type pointcloud
    void pointcloud_process(
        const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& pointcloud,
        const std_msgs::msg::Header& header);

    // @brief work as the livox subscription callback, transform the message to pcl pointcloud
    void livox_subscription_callback(const std::unique_ptr<livox_ros_driver2::msg::CustomMsg>& msg);

    // @brief work as the standard pointcloud2 subscription callback, transform the message to pcl
    // pointcloud
    void pointcloud2_subscription_callback(
        const std::unique_ptr<sensor_msgs::msg::PointCloud2>& msg);
};
