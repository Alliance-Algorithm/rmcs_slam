#include "registration/engine.hpp"
#include "util/logger.hpp"
#include "util/node.hpp"
#include "util/parameter.hpp"
#include "util/pointcloud.hpp"

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/executor.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace rmcs;

struct {
    RMCS_INITIALIZE_LOGGER("debug");
} debug;

auto main(int argc, char** argv) -> int {
    using Point      = pcl::PointXYZ;
    using PointCloud = pcl::PointCloud<Point>;

    rclcpp::init(argc, argv);
    std::signal(SIGINT, [](int) { rclcpp::shutdown(); });

    auto node = util::make_simple_node("rmcs_location");
    auto p    = util::quick_paramtetr_reader(*node);

    auto timestamp_first = std::chrono::high_resolution_clock::now();

    auto init_transfrom = Eigen::Affine3f{
        Eigen::Translation3f::Identity()
        * Eigen::Quaternionf{Eigen::AngleAxisf{std::numbers::pi, Eigen::Vector3f::UnitY()}}};

    auto map = std::make_shared<PointCloud>();
    if (pcl::io::loadPCDFile(p("map_path", std::string{}), *map) == -1)
        rclcpp::shutdown();
    auto frame = std::make_shared<PointCloud>();
    if (pcl::io::loadPCDFile(p("test_path", std::string{}), *frame))
        rclcpp::shutdown();

    pcl::transformPointCloud(*map, *map, init_transfrom);
    pcl::transformPointCloud(*frame, *frame, init_transfrom);

    // 构建 fast-gicp 实例
    auto engine = Registration{};
    engine.initialize(*node);
    engine.register_map(map);
    engine.register_scan(frame);

    auto timestamp_second = std::chrono::high_resolution_clock::now();
    auto second           = std::chrono::duration<double>(timestamp_second - timestamp_first);
    debug.rclcpp_info("load map and cost %5.2fs", second.count());

    auto align = std::make_shared<PointCloud>();
    engine.full_match(align);

    auto transfrom   = engine.transformation();
    auto translation = transfrom.translation();
    debug.rclcpp_info("query result: x[%5.2f] y[%5.2f]", translation.x(), translation.y());

    auto timestamp_third = std::chrono::high_resolution_clock::now();
    auto second_third    = std::chrono::duration<double>(timestamp_third - timestamp_second);
    debug.rclcpp_info("query and cost %5.2fs", second_third.count());

    using namespace std::chrono_literals;
    while (true) {
        rmcs::pointcloud_util<0>::publish(*map, "debug");
        rmcs::pointcloud_util<1>::publish(*frame, "debug");
        rmcs::pointcloud_util<2>::publish(*align, "debug");

        std::this_thread::sleep_for(1s);
    }

    rclcpp::shutdown();
    return 0;
}
