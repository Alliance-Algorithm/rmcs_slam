#include "obstacle/ros2/node.hpp"
#include "obstacle/map/process.hpp"
#include "obstacle/map/segmentation.hpp"
#include "obstacle/ros2/convert.hpp"
#include "obstacle/ros2/param.hpp"
#include "util/convert.hpp"
#include "util/string.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <rclcpp/logging.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

using namespace rmcs;

struct Node::Impl {
    RMCS_INITIALIZE_LOGGER("rmcs-map");

    using Point      = pcl::PointXYZ;
    using PointCloud = pcl::PointCloud<Point>;

    // 多重点云积累生成障碍地图，适用于点云比较稀疏的情况
    int pointcloud_frame_limit = 1;
    int pointcloud_frame_index = 0;
    std::vector<pcl::PointCloud<pcl::PointXYZ>> pointcloud_frames;

    bool publish_cloud = false;
    Segmentation segmentation;
    Process process;

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> segmentation_publisher;
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>> obstacle_publisher;

    std::shared_ptr<rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>> livox_subscription;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> pointcloud_subscription;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_transform_broadcaster;
    std::unique_ptr<tf2_ros::Buffer> transform_buffer;
    std::unique_ptr<tf2_ros::TransformListener> transform_listener;

    void pointcloud_subscription_callback(
        const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& pointcloud,
        const std_msgs::msg::Header& header) {

        // 去除盲区点云
        auto crop_box = pcl::CropBox<Point> {};
        auto blind    = param::get<float>("grid.lidar_blind");
        crop_box.setMin(Eigen::Vector4f { -blind / 2, -blind / 2, -1'000, 1 });
        crop_box.setMax(Eigen::Vector4f { +blind / 2, +blind / 2, +1'000, 1 });
        crop_box.setInputCloud(pointcloud);
        crop_box.setNegative(true);
        crop_box.filter(*pointcloud);

        // 补偿多帧叠加的位移偏差
        auto orientation = Eigen::Quaternionf::Identity();
        auto translation = Eigen::Translation3f::Identity();
        if (pointcloud_frame_limit > 2) {
            try {
                auto transform = transform_buffer->lookupTransform(
                    rmcs::string::slam_link, rmcs::string::robot_link, tf2::TimePointZero);
                util::convert_orientation(transform.transform.rotation, orientation);
                util::convert_translation(
                    transform.transform.translation, translation.translation());
            } catch (const tf2::TransformException& e) {
                orientation = Eigen::Quaternionf::Identity();
                translation = Eigen::Translation3f::Identity();
                rclcpp_warn("%s", e.what());
            }
        }
        auto& pointcloud_frame = pointcloud_frames.at(pointcloud_frame_index);
        pointcloud_frame.clear();
        pcl::transformPointCloud(
            *pointcloud, pointcloud_frame, Eigen::Affine3f { translation * orientation });

        auto pointcloud_mixed = std::make_shared<PointCloud>();
        for (const auto& frame : pointcloud_frames)
            *pointcloud_mixed += frame;

        // 将点云变换回云台系
        auto pointcloud_mixed_yaw_link = std::make_shared<PointCloud>();
        pcl::transformPointCloud(*pointcloud_mixed, *pointcloud_mixed_yaw_link,
            Eigen::Affine3f { translation * orientation }.inverse());

        pointcloud_process(pointcloud_mixed_yaw_link, header);

        if (++pointcloud_frame_index >= pointcloud_frame_limit) pointcloud_frame_index = 0;
    }

    void pointcloud_process(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& pointcloud,
        const std_msgs::msg::Header& header) {

        if (pointcloud->size() < 10) return;

        segmentation.set_input_source(pointcloud);
        auto segmentation_part = segmentation.execute();

        auto segmentation_part_pointcloud2 = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl_to_pc2(*segmentation_part, *segmentation_part_pointcloud2);
        segmentation_part_pointcloud2->header.frame_id = string::robot_link;
        segmentation_part_pointcloud2->header.stamp    = header.stamp;

        if (publish_cloud) segmentation_publisher->publish(*segmentation_part_pointcloud2);

        // generate grid map
        auto grid_map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
        auto node_map = process.generate_node_map(segmentation_part);
        node_to_grid_map(*node_map, *grid_map);

        grid_map->header.frame_id = string::robot_link;
        grid_map->header.stamp    = header.stamp;
        grid_map->info.resolution = process.resolution();
        grid_map->info.height     = process.size_num();
        grid_map->info.width      = process.size_num();

        grid_map->info.origin.position.x = -process.map_width() / 2.0;
        grid_map->info.origin.position.y = -process.map_width() / 2.0;

        obstacle_publisher->publish(*grid_map);
    }

    void livox_subscription_callback(
        const std::unique_ptr<livox_ros_driver2::msg::CustomMsg>& msg) {

        auto pointcloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        livox_to_pcl(msg->points, *pointcloud);

        pointcloud_subscription_callback(pointcloud, msg->header);
    }

    void pointcloud2_subscription_callback(
        const std::unique_ptr<sensor_msgs::msg::PointCloud2>& msg) {

        auto pointcloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pc2_to_pcl(*msg, *pointcloud);

        pointcloud_subscription_callback(pointcloud, msg->header);
    }
};

Node::Node()
    : rclcpp::Node(param::get<std::string>("name.node"))
    , pimpl(std::make_unique<Impl>()) {

    pimpl->obstacle_publisher =
        create_publisher<nav_msgs::msg::OccupancyGrid>(param::get<std::string>("name.grid"), 10);
    pimpl->segmentation_publisher =
        create_publisher<sensor_msgs::msg::PointCloud2>("/rmcs_map/segmentation_part", 10);

    auto pointcloud_type = param::get<std::string>("switch.pointcloud_type");

    const auto lidar_topic = param::get<std::string>("lidar.topic");
    if (pointcloud_type == "livox") {
        pimpl->livox_subscription = create_subscription<livox_ros_driver2::msg::CustomMsg>(
            lidar_topic, 10, [this](const std::unique_ptr<livox_ros_driver2::msg::CustomMsg>& msg) {
                pimpl->livox_subscription_callback(msg);
            });
    } else if (pointcloud_type == "pointcloud2") {
        pimpl->pointcloud_subscription = create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic, 10, [this](const std::unique_ptr<sensor_msgs::msg::PointCloud2>& msg) {
                pimpl->pointcloud2_subscription_callback(msg);
            });
    }

    pimpl->publish_cloud          = param::get<bool>("switch.publish_cloud");
    pimpl->pointcloud_frame_limit = param::get<int>("lidar.livox_frames");

    pimpl->pointcloud_frames.resize(pimpl->pointcloud_frame_limit);
    pimpl->static_transform_broadcaster =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    pimpl->transform_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
    pimpl->transform_listener =
        std::make_unique<tf2_ros::TransformListener>(*pimpl->transform_buffer);
}

Node::~Node() = default;
