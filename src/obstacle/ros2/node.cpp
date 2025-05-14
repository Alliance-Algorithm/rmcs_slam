#include "node.hpp"
#include "convert.hpp"
#include "param.hpp"

#include "../map/process.hpp"
#include "../map/segmentation.hpp"
#include "util/convert.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <rclcpp/logging.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <string>

using namespace rmcs;

struct MapNode::Impl {
    RMCS_INITIALIZE_LOGGER("rmcs-map");

    using Point      = pcl::PointXYZ;
    using PointCloud = pcl::PointCloud<Point>;

    // 多重点云积累生成障碍地图，适用于点云比较稀疏的情况
    int pointcloud_frame_limit = 1;
    int pointcloud_frame_index = 0;
    std::vector<pcl::PointCloud<pcl::PointXYZ>> pointcloud_frames;

    bool publish_cloud = false;
    Process process;
    Segmentation segmentation;

    std::string map_frame_id;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> segmentation_publisher;

    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>> grid_map_publisher_;
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>> cost_map_publisher_;

    std::shared_ptr<rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>> livox_subscription_;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> pointcloud_subscription_;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_transform_broadcaster;
    std::unique_ptr<tf2_ros::Buffer> transform_buffer;
    std::unique_ptr<tf2_ros::TransformListener> transform_listener;

    using Callback = std::function<void(
        const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>&, const std_msgs::msg::Header&)>;

    void pointcloud_subscription_callback(
        const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& pointcloud,
        const std_msgs::msg::Header& header, const Callback& process) {

        // 去除盲区点云
        auto crop_box = pcl::CropBox<Point> {};
        auto blind    = param::get<float>("grid.lidar_blind");
        crop_box.setMin(Eigen::Vector4f { -blind / 2, -blind / 2, -1'000 , 1});
        crop_box.setMax(Eigen::Vector4f { +blind / 2, +blind / 2, +1'000 , 1});
        crop_box.setInputCloud(pointcloud);
        crop_box.setNegative(true);
        crop_box.filter(*pointcloud);

        // 补偿多帧叠加的位移偏差
        auto orientation = Eigen::Quaternionf::Identity();
        auto translation = Eigen::Translation3f::Identity();
        try {
            auto transform =
                transform_buffer->lookupTransform("lidar_init", "lidar_link", tf2::TimePointZero);
            util::convert_orientation(transform.transform.rotation, orientation);
            util::convert_translation(transform.transform.translation, translation.translation());
        } catch (const tf2::TransformException& e) {
            orientation = Eigen::Quaternionf::Identity();
            translation = Eigen::Translation3f::Identity();
            rclcpp_warn("%s", e.what());
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

        process(pointcloud_mixed_yaw_link, header);

        if (++pointcloud_frame_index >= pointcloud_frame_limit) pointcloud_frame_index = 0;
    }

    void publish_transform(geometry_msgs::msg::TransformStamped& transform) {
        transform.header.frame_id = "lidar_link";
        transform.child_frame_id  = map_frame_id;
        static_transform_broadcaster->sendTransform(transform);
    }
};

MapNode::MapNode()
    : Node(param::get<std::string>("name.node"))
    , pimpl(std::make_unique<Impl>()) {

    pimpl->grid_map_publisher_ =
        create_publisher<nav_msgs::msg::OccupancyGrid>(param::get<std::string>("name.grid"), 10);
    pimpl->cost_map_publisher_ =
        create_publisher<nav_msgs::msg::OccupancyGrid>(param::get<std::string>("name.cost"), 10);
    pimpl->segmentation_publisher =
        create_publisher<sensor_msgs::msg::PointCloud2>("/rmcs_map/segmentation_part", 10);

    auto pointcloud_type = param::get<std::string>("switch.pointcloud_type");

    const auto lidar_topic = param::get<std::string>("name.lidar");
    if (pointcloud_type == "livox") {
        pimpl->livox_subscription_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
            lidar_topic, 10, [this](const std::unique_ptr<livox_ros_driver2::msg::CustomMsg>& msg) {
                livox_subscription_callback(msg);
            });
    } else if (pointcloud_type == "pointcloud2") {
        pimpl->pointcloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic, 10, [this](const std::unique_ptr<sensor_msgs::msg::PointCloud2>& msg) {
                pointcloud2_subscription_callback(msg);
            });
    }

    pimpl->map_frame_id           = param::get<std::string>("name.frame.map");
    pimpl->publish_cloud          = param::get<bool>("switch.publish_cloud");
    pimpl->pointcloud_frame_limit = param::get<int>("lidar.livox_frames");

    pimpl->pointcloud_frames.resize(pimpl->pointcloud_frame_limit);
    pimpl->static_transform_broadcaster =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    pimpl->transform_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
    pimpl->transform_listener =
        std::make_unique<tf2_ros::TransformListener>(*pimpl->transform_buffer);

    auto transform         = geometry_msgs::msg::TransformStamped {};
    transform.header.stamp = get_clock()->now();
    util::convert_orientation(Eigen::Quaterniond::Identity(), transform.transform.rotation);
    pimpl->publish_transform(transform);
}

MapNode::~MapNode() = default;

void MapNode::pointcloud_process(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& pointcloud,
    const std_msgs::msg::Header& header) {

    if (pointcloud->size() < 10) return;

    pimpl->segmentation.set_input_source(pointcloud);
    auto segmentation_part = pimpl->segmentation.execute();

    auto segmentation_part_pointcloud2 = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl_to_pc2(*segmentation_part, *segmentation_part_pointcloud2);
    segmentation_part_pointcloud2->header.frame_id = pimpl->map_frame_id;
    segmentation_part_pointcloud2->header.stamp    = header.stamp;

    if (pimpl->publish_cloud)
        pimpl->segmentation_publisher->publish(*segmentation_part_pointcloud2);

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

    pimpl->pointcloud_subscription_callback(pointcloud, msg->header,
        [this](const auto& msg, const auto& header) { pointcloud_process(msg, header); });
}

void MapNode::pointcloud2_subscription_callback(
    const std::unique_ptr<sensor_msgs::msg::PointCloud2>& msg) {

    auto pointcloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pc2_to_pcl(*msg, *pointcloud);

    pimpl->pointcloud_subscription_callback(pointcloud, msg->header,
        [this](const auto& msg, const auto& header) { pointcloud_process(msg, header); });
}
