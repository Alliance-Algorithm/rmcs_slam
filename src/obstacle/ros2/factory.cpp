#include "factory.hpp"
#include "ros2/convert.hpp"
#include "undistortion/undistortion.hpp"
#include "util/string.hpp"

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/subscription.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace rmcs;
static constexpr auto kLogName = [] { return "rmcs-map"; };

using LidMsg = livox_ros_driver2::msg::CustomMsg;
using ImuMsg = sensor_msgs::msg::Imu;
using LidSub = rclcpp::Subscription<LidMsg>;
using ImuSub = rclcpp::Subscription<ImuMsg>;
using LidPub = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>;

struct Factory::Impl {
    util::Log<kLogName> log;
    rclcpp::Node& node_reference;

    std::string lid_topic;
    std::string imu_topic;

    Undistortion undistortion_core;

    Eigen::Isometry3d lid_assembly_transform;
    Eigen::Isometry3d imu_extrinsic_transform;

    std_msgs::msg::Header newest_message_header;

    std::shared_ptr<rclcpp::TimerBase> async_callback_scheduler;
    std::shared_ptr<LidSub> lid_subscription;
    std::shared_ptr<ImuSub> imu_subscription;
    std::shared_ptr<LidPub> lid_publisher;

    bool publish_undistort_pointcloud = true;

    explicit Impl(rclcpp::Node& node)
        : node_reference { node } { }

    auto build_normal_mode(const Callback& process) -> void {
        lid_publisher = node_reference.create_publisher<sensor_msgs::msg::PointCloud2>(
            lid_topic + "/origin", 10);

        lid_subscription = node_reference.create_subscription<LidMsg>(lid_topic, 10, //
            [process, this](const std::unique_ptr<LidMsg>& msg) {
                auto pointcloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
                livox_to_pcl(msg->points, *pointcloud, lid_assembly_transform);

                if (publish_undistort_pointcloud) {
                    auto msg = sensor_msgs::msg::PointCloud2 {};
                    pcl::toROSMsg(*pointcloud, msg);
                    msg.header.stamp    = newest_message_header.stamp;
                    msg.header.frame_id = string::robot_link;
                    lid_publisher->publish(msg);
                }

                process(pointcloud, msg->header);
            });
        log.info("Create subscription %s", lid_topic.c_str());
    }

    auto build_undirtort_mode(const Callback& process) -> void {
        undistortion_core.set_lid_transform(lid_assembly_transform);
        undistortion_core.set_imu_transform(imu_extrinsic_transform);

        lid_subscription = node_reference.create_subscription<LidMsg>(
            lid_topic, 10, [this](std::unique_ptr<LidMsg> msg) {
                newest_message_header = msg->header;
                undistortion_core.push_lid_message(std::move(msg));
            });
        imu_subscription = node_reference.create_subscription<ImuMsg>(
            imu_topic, 10, [this](std::unique_ptr<ImuMsg> msg) {
                undistortion_core.push_imu_message(std::move(msg));
            });
        lid_publisher = node_reference.create_publisher<sensor_msgs::msg::PointCloud2>(
            lid_topic + "/undistort", 10);

        using namespace std::chrono_literals;
        async_callback_scheduler = node_reference.create_wall_timer(10ms, [this, process] {
            if (auto pointcloud = undistortion_core.try_query_undistort_cloud()) {
                if (publish_undistort_pointcloud) {
                    auto msg = sensor_msgs::msg::PointCloud2 {};
                    pcl::toROSMsg(*pointcloud, msg);
                    msg.header.stamp    = newest_message_header.stamp;
                    msg.header.frame_id = string::robot_link;
                    lid_publisher->publish(msg);
                }
                process(pointcloud, newest_message_header);
            }
        });
    }
};

Factory::Factory(rclcpp::Node& node) noexcept
    : pimpl { std::make_unique<Impl>(node) } { }

Factory::~Factory() = default;

auto Factory::set_lid_assembly_transform(const Eigen::Isometry3d& t) -> Factory& {
    return pimpl->lid_assembly_transform = t, *this;
}
auto Factory::set_imu_extrinsic_transform(const Eigen::Isometry3d& t) -> Factory& {
    return pimpl->imu_extrinsic_transform = t, *this;
}
auto Factory::set_lid_topic_name(const std::string& topic) -> Factory& {
    return pimpl->lid_topic = topic, *this;
}
auto Factory::set_imu_topic_name(const std::string& topic) -> Factory& {
    return pimpl->imu_topic = topic, *this;
}
auto Factory::build_normal_mode(const Callback& process) -> void {
    pimpl->build_normal_mode(process);
}
auto Factory::build_unditort_mode(const Callback& process) -> void {
    pimpl->build_undirtort_mode(process);
}
