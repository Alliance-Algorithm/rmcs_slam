#include "node.hpp"
#include "convert.hpp"
#include "param.hpp"

#include "../map/process.hpp"
#include "../map/segmentation.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/common/transforms.h>
#include <rclcpp/logging.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>

#include <memory>
#include <string>

using namespace rmcs;

struct MapNode::Impl {
    // 多重点云积累生成障碍地图，适用于点云比较稀疏的情况
    int pointcloud_frame_limit = 1;
    int pointcloud_frame_index = 0;
    std::vector<pcl::PointCloud<pcl::PointXYZ>> pointcloud_frames;

    bool publish_cloud = false;
    Process process;
    Segmentation segmentation;

    std::string map_frame_id;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> segmentation_publisher;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pointcloud_publisher;

    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>> grid_map_publisher_;
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>> cost_map_publisher_;

    std::shared_ptr<rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>> livox_subscription_;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> pointcloud_subscription_;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_transform_broadcaster_;

    std::shared_ptr<rclcpp::TimerBase> test_timer_;

    using Callback = std::function<void(
        const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>&, const std_msgs::msg::Header&)>;

    void pointcloud_subscription_callback(
        const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& msg,
        const std_msgs::msg::Header& header, const Callback& process) {
        auto& pointcloud_frame = pointcloud_frames.at(pointcloud_frame_index);

        pointcloud_frame.clear();
        pointcloud_frame = *msg;

        auto pointcloud_mixed = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        for (const auto& frame : pointcloud_frames)
            *pointcloud_mixed += frame;

        process(pointcloud_mixed, header);

        if (++pointcloud_frame_index >= pointcloud_frame_limit)
            pointcloud_frame_index = 0;
    }
};

MapNode::MapNode()
    : Node(param::get<std::string>("name.node"))
    , pimpl(std::make_unique<Impl>()) {

    pimpl->grid_map_publisher_ =
        create_publisher<nav_msgs::msg::OccupancyGrid>(param::get<std::string>("name.grid"), 10);
    pimpl->cost_map_publisher_ =
        create_publisher<nav_msgs::msg::OccupancyGrid>(param::get<std::string>("name.cost"), 10);
    pimpl->pointcloud_publisher =
        create_publisher<sensor_msgs::msg::PointCloud2>("/rmcs_map/transformed_part", 10);
    pimpl->segmentation_publisher =
        create_publisher<sensor_msgs::msg::PointCloud2>("/rmcs_map/segmentation_part", 10);

    auto pointcloud_type = param::get<std::string>("switch.pointcloud_type");

    if (pointcloud_type == "livox") {
        pimpl->livox_subscription_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
            param::get<std::string>("name.lidar"), 10,
            [this](const std::unique_ptr<livox_ros_driver2::msg::CustomMsg>& msg) {
                livox_subscription_callback(msg);
            });
    } else if (pointcloud_type == "pointcloud2") {
        pimpl->pointcloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            param::get<std::string>("name.lidar"), 10,
            [this](const std::unique_ptr<sensor_msgs::msg::PointCloud2>& msg) {
                pointcloud2_subscription_callback(msg);
            });
    }

    pimpl->map_frame_id           = param::get<std::string>("name.frame.map");
    pimpl->publish_cloud          = param::get<bool>("switch.publish_cloud");
    pimpl->pointcloud_frame_limit = param::get<int>("lidar.livox_frames");

    pimpl->pointcloud_frames.resize(pimpl->pointcloud_frame_limit);
    pimpl->static_transform_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    publish_static_transform();
}

MapNode::~MapNode() = default;

void MapNode::publish_static_transform() {
    auto transform_stamp = geometry_msgs::msg::TransformStamped();

    auto quaternion                      = Eigen::Quaterniond::Identity();
    transform_stamp.transform.rotation.w = quaternion.w();
    transform_stamp.transform.rotation.x = quaternion.x();
    transform_stamp.transform.rotation.y = quaternion.y();
    transform_stamp.transform.rotation.z = quaternion.z();

    transform_stamp.header.stamp    = this->get_clock()->now();
    transform_stamp.header.frame_id = param::get<std::string>("name.frame.lidar");
    transform_stamp.child_frame_id  = param::get<std::string>("name.frame.map");
    pimpl->static_transform_broadcaster_->sendTransform(transform_stamp);
}

void MapNode::pointcloud_process(
    const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& pointcloud,
    const std_msgs::msg::Header& header) {

    // make pointcloud to publish
    auto transformed_part_pointcloud2 = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl_to_pc2(*pointcloud, *transformed_part_pointcloud2);
    transformed_part_pointcloud2->header.frame_id = pimpl->map_frame_id;
    transformed_part_pointcloud2->header.stamp    = header.stamp;

    pimpl->segmentation.set_input_source(pointcloud);
    auto segmentation_part = pimpl->segmentation.execute();

    auto segmentation_part_pointcloud2 = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl_to_pc2(*segmentation_part, *segmentation_part_pointcloud2);
    segmentation_part_pointcloud2->header.frame_id = pimpl->map_frame_id;
    segmentation_part_pointcloud2->header.stamp    = header.stamp;

    if (pimpl->publish_cloud) {
        pimpl->pointcloud_publisher->publish(*transformed_part_pointcloud2);
        pimpl->segmentation_publisher->publish(*segmentation_part_pointcloud2);
    }

    // generate grid map
    auto grid_map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    auto node_map = pimpl->process.generate_node_map(segmentation_part);
    node_to_grid_map(*node_map, *grid_map);

    grid_map->header.stamp    = header.stamp;
    grid_map->header.frame_id = pimpl->map_frame_id;
    grid_map->info.resolution = pimpl->process.resolution();
    grid_map->info.height     = pimpl->process.size_num();
    grid_map->info.width      = pimpl->process.size_num();

    grid_map->info.origin.position.x = -pimpl->process.map_width() / 2.0;
    grid_map->info.origin.position.y = -pimpl->process.map_width() / 2.0;

    pimpl->grid_map_publisher_->publish(*grid_map);
}

void MapNode::livox_subscription_callback(
    const std::unique_ptr<livox_ros_driver2::msg::CustomMsg>& msg) {

    auto pointcloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    livox_to_pcl(msg->points, *pointcloud);

    pimpl->pointcloud_subscription_callback(
        pointcloud, msg->header,
        [this](const auto& msg, const auto& header) { pointcloud_process(msg, header); });
}

void MapNode::pointcloud2_subscription_callback(
    const std::unique_ptr<sensor_msgs::msg::PointCloud2>& msg) {

    auto pointcloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pc2_to_pcl(*msg, *pointcloud);

    pimpl->pointcloud_subscription_callback(
        pointcloud, msg->header,
        [this](const auto& msg, const auto& header) { pointcloud_process(msg, header); });
}
