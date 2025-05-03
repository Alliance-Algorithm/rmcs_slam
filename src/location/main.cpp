#include "preprocess/descriptor-manager.hpp"
#include "util/logger.hpp"
#include "util/pointcloud.hpp"

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/executor.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

struct {
    RMCS_INITIALIZE_LOGGER("debug");
} debug;

auto main(int argc, char** argv) -> int {
    using PointCloud = rmcs::DescriptorManager::PointCloudT;

    rclcpp::init(argc, argv);

    auto timestamp_first = std::chrono::high_resolution_clock::now();

    auto init_transfrom = Eigen::Affine3f{
        Eigen::Translation3f::Identity()
        * Eigen::Quaternionf{Eigen::AngleAxisf{std::numbers::pi, Eigen::Vector3f::UnitY()}}};

    auto map = std::make_shared<PointCloud>();
    if (pcl::io::loadPCDFile("/workspaces/RMCS/develop_ws/pcd/battlefiled.pcd", *map) == -1)
        rclcpp::shutdown();
    auto frame = std::make_shared<PointCloud>();
    if (pcl::io::loadPCDFile(argv[1], *frame))
        rclcpp::shutdown();

    pcl::transformPointCloud(*map, *map, init_transfrom);
    pcl::transformPointCloud(*frame, *frame, init_transfrom);

    auto descriptor_manager = rmcs::DescriptorManager{};
    descriptor_manager.make_context(map);

    auto timestamp_second = std::chrono::high_resolution_clock::now();
    auto second           = std::chrono::duration<double>(timestamp_second - timestamp_first);
    debug.rclcpp_info("load map and cost %5.2fs", second.count());

    auto [result, key_frame] = descriptor_manager.query(frame);

    auto timestamp_third = std::chrono::high_resolution_clock::now();
    auto second_third    = std::chrono::duration<double>(timestamp_second - timestamp_first);
    debug.rclcpp_info("query and cost %5.2fs", second_third.count());

    auto translation = Eigen::Translation3f{result.translation()};
    debug.rclcpp_info("query result: x[%5.2f] y[%5.2f]", translation.x(), translation.y());

    auto node = std::make_shared<rclcpp::Node>("debug");

    using PointCloud2     = sensor_msgs::msg::PointCloud2;
    auto publisher_map    = node->create_publisher<PointCloud2>("map", 10);
    auto publisher_key    = node->create_publisher<PointCloud2>("key", 10);
    auto publisher_result = node->create_publisher<PointCloud2>("result", 10);

    using Point = rmcs::DescriptorManager::PointT;
    auto filter = pcl::VoxelGrid<Point>{};
    filter.setLeafSize(0.1, 0.1, 0.1);
    filter.setInputCloud(map);
    filter.filter(*map);

    auto map_visualized = PointCloud2{};
    pcl::toROSMsg(*map, map_visualized);
    map_visualized.header.frame_id = "debug";

    pcl::transformPointCloud(*frame, *frame, result);
    auto result_visualized = PointCloud2{};
    pcl::toROSMsg(*frame, result_visualized);
    result_visualized.header.frame_id = "debug";

    auto key_translation = Eigen::Affine3f{translation * Eigen::Quaternionf::Identity()};
    pcl::transformPointCloud(key_frame, key_frame, key_translation);

    auto key_visualized = PointCloud2{};
    pcl::toROSMsg(key_frame, key_visualized);
    key_visualized.header.frame_id = "debug";

    using namespace std::chrono_literals;
    while (true) {
        rmcs::pointcloud_util<0>::publish(*map, "debug");
        rmcs::pointcloud_util<1>::publish(*frame, "debug");
        rmcs::pointcloud_util<2>::publish(key_frame, "debug");

        std::this_thread::sleep_for(1s);
    }

    rclcpp::shutdown();
    return 0;
}
