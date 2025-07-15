#include "obstacle/ros2/node.hpp"
#include "obstacle/map/process.hpp"
#include "ros2/convert.hpp"
#include "ros2/factory.hpp"
#include "util/parameter.hpp"
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

#include <deque>
#include <memory>
#include <string>

using namespace rmcs;

constexpr auto kLogName = [] { return "rmcs-map"; };

struct RmcsMapRuntime::Impl {
    util::Log<kLogName> log;

    using Point      = pcl::PointXYZ;
    using PointCloud = pcl::PointCloud<Point>;

    Process process;
    std::vector<std::unique_ptr<Factory>> factories;

    bool switch_publish  = false;
    bool switch_obstacle = false;
    double lidar_blind   = 0;
    double map_width     = 0;
    double map_height    = 0;

    // 多重点云积累生成障碍地图，适用于点云比较稀疏的情况
    int frame_limit = 1;
    std::deque<std::shared_ptr<PointCloud>> frames;

    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> segmentation_publisher;
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>> obstacle_publisher;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_transform_broadcaster;

    explicit Impl(rclcpp::Node& node) {
        const auto p = util::quick_paramtetr_reader(node);

        obstacle_publisher =
            node.create_publisher<nav_msgs::msg::OccupancyGrid>(p("name.grid", std::string {}), 10);
        segmentation_publisher =
            node.create_publisher<sensor_msgs::msg::PointCloud2>("/rmcs_map/segmentation_part", 10);

        switch_publish = p("switch.publish_cloud", bool {});
        frame_limit    = p("lidar.livox_frames", int {});
        lidar_blind    = p("grid.lidar_blind", double {});
        map_width      = p("grid.map_width", double {});
        map_height     = p("grid.map_height", double {});

        static_transform_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(&node);

        p("switch.undistort", bool {}) ? setup_undistortion_mode(node) : setup_normal_node(node);
    }

    auto setup_normal_node(rclcpp::Node& node) -> void {
        const auto p = util::quick_paramtetr_reader(node);
        log.info(util::title_text("Rmcs obstacle setup as normal mode").c_str());

        const auto callback = [this](const auto& p0, const auto& p1) {
            pointcloud_preprocess(p0, p1);
        };
        const auto topics = p("lidar.lid_topics", std::vector<std::string> {});
        for (auto index = 0; index < topics.size(); index++) {
            factories.emplace_back(std::make_unique<Factory>(node))
                ->set_lid_topic_name(topics.at(index))
                .set_lid_assembly_transform(get_lidar_transform(node, index))
                .build_normal_mode(callback);
        }
        log.info("Factory size: %ld", factories.size());
    }
    auto setup_undistortion_mode(rclcpp::Node& node) -> void {
        const auto p = util::quick_paramtetr_reader(node);
        log.info(util::title_text("Rmcs obstacle setup as undistortion mode").c_str());

        const auto imu_extrinsic_translation =
            p("lidar.imu_extrinsic_translation", std::vector<double> {});
        const auto imu_extrinsic_orientation =
            p("lidar.imu_extrinsic_orientation", std::vector<double> {});

        if (imu_extrinsic_translation.size() != 3 || imu_extrinsic_orientation.size() != 4)
            throw util::runtime_error("Wrong imu extrinsic transform format");

        const auto imu_extrinsic_transform = Eigen::Isometry3d { //
            Eigen::Translation3d {
                imu_extrinsic_translation[0],
                imu_extrinsic_translation[1],
                imu_extrinsic_translation[2],
            } * 
            Eigen::Quaterniond {
                imu_extrinsic_orientation[0],
                imu_extrinsic_orientation[1],
                imu_extrinsic_orientation[2],
                imu_extrinsic_orientation[3],
            }
        };

        const auto callback = [this](const auto& p0, const auto& p1) {
            if (switch_obstacle) pointcloud_preprocess(p0, p1);
        };
        const auto lid_topics = p("lidar.lid_topics", std::vector<std::string> {});
        const auto imu_topics = p("lidar.imu_topics", std::vector<std::string> {});
        for (auto index = 0; index < lid_topics.size(); index++) {
            factories.emplace_back(std::make_unique<Factory>(node))
                ->set_lid_topic_name(lid_topics[index])
                .set_imu_topic_name(imu_topics[index])
                .set_imu_extrinsic_transform(imu_extrinsic_transform)
                .set_lid_assembly_transform(get_lidar_transform(node, index))
                .build_unditort_mode(callback);
        }
    }

    auto pointcloud_preprocess(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& pointcloud,
        const std_msgs::msg::Header& header) -> void {

        const auto timestamp_begin = std::chrono::high_resolution_clock::now();

        auto crop_box = pcl::CropBox<Point> {};

        // 去除盲区点云
        const auto blind = static_cast<float>(lidar_blind) / 2;
        crop_box.setMin(Eigen::Vector4f { -blind, -blind, -1'000, 1 });
        crop_box.setMax(Eigen::Vector4f { +blind, +blind, +1'000, 1 });
        crop_box.setInputCloud(pointcloud);
        crop_box.setNegative(true);
        crop_box.filter(*pointcloud);

        // 约束点云范围
        const auto width  = static_cast<float>(map_width) / 2;
        const auto height = static_cast<float>(map_height);
        crop_box.setMin(Eigen::Vector4f { -width, -width, -1'000, 1 });
        crop_box.setMax(Eigen::Vector4f { +width, +width, +height, 1 });
        crop_box.setInputCloud(pointcloud);
        crop_box.setNegative(false);
        crop_box.filter(*pointcloud);

        // 约束点云积累数量
        frames.push_back(pointcloud);
        while (frames.size() > frame_limit)
            frames.pop_front();

        auto pointcloud_mixed = std::make_shared<PointCloud>();
        for (const auto& frame : frames)
            *pointcloud_mixed += *frame;

        pointcloud_process(pointcloud_mixed, header);

        const auto timestamp_finish = std::chrono::high_resolution_clock::now();
        const auto seconds = std::chrono::duration<double>(timestamp_finish - timestamp_begin);

        // log.info("Porcess cost seconds: %10.5fs", seconds.count());
    }

    auto pointcloud_process(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>& pointcloud,
        const std_msgs::msg::Header& header) -> void {

        /// @note 使用高程表来制作障碍地图后，地面分割没啥必要了
        // segmentation.set_input_source(pointcloud);
        // auto segmentation_part = segmentation.execute();
        const auto& segmentation_part = pointcloud;

        auto segmentation_part_pointcloud2 = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl_to_pc2(*segmentation_part, *segmentation_part_pointcloud2);
        segmentation_part_pointcloud2->header.frame_id = string::robot_link;
        segmentation_part_pointcloud2->header.stamp    = header.stamp;

        if (switch_publish) segmentation_publisher->publish(*segmentation_part_pointcloud2);

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

    auto get_lidar_transform(rclcpp::Node& node, std::size_t index) const -> Eigen::Isometry3d {
        const auto p = util::quick_paramtetr_reader(node);

        const auto radian = [](double degrees) { return degrees / 180. * std::numbers::pi; };
        const auto t_raw  = p("lidar.lidar_translations", std::vector<double>());
        const auto q_raw  = p("lidar.lidar_orientations", std::vector<double>());

        const auto t = Eigen::Translation3d {
            t_raw[index * 3 + 0],
            t_raw[index * 3 + 1],
            t_raw[index * 3 + 2],
        };
        const auto q = Eigen::Quaterniond { Eigen::Quaterniond::Identity() /* 对齐用的 */
            * Eigen::AngleAxisd(radian(q_raw[index * 3 + 0]), Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(radian(q_raw[index * 3 + 1]), Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(radian(q_raw[index * 3 + 2]), Eigen::Vector3d::UnitX()) }
                           .normalized();

        log.info("index: %d, t: %+5.2fm %+5.2fm %+5.2fm, q: %+5.2f° %+5.2f° %+5.2f°", index,
            t_raw[index * 3 + 0], t_raw[index * 3 + 1], t_raw[index * 3 + 2], q_raw[index * 3 + 0],
            q_raw[index * 3 + 1], q_raw[index * 3 + 2]);

        return Eigen::Isometry3d { t * q };
    }
};

RmcsMapRuntime::RmcsMapRuntime()
    : rclcpp::Node { "rmcs_map", util::NodeOptions {} }
    , pimpl { std::make_unique<Impl>(*this) } { }

RmcsMapRuntime::~RmcsMapRuntime() = default;
