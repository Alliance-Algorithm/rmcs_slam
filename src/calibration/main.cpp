#include "util/logger.hpp"
#include "util/node.hpp"
#include "util/parameter.hpp"
#include "util/pointcloud.hpp"

#include <fast_gicp/gicp/fast_gicp.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl/filters/voxel_grid.h>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>

using namespace rmcs;

int main(int argc, char* argv[]) {
    using namespace std::chrono_literals;
    using Point      = pcl::PointXYZ;
    using PointCloud = pcl::PointCloud<Point>;
    using Engine     = fast_gicp::FastGICP<Point, Point>;
    using LivoxMsg   = livox_ros_driver2::msg::CustomMsg;

    rclcpp::init(argc, argv);

    auto node = util::make_simple_node("rmcs_calibration");
    RMCS_MAKE_LOGGER_LAMBDA("rmcs-calibration");

    rclcpp_info("开始校准双雷达位姿，请保持雷达静止");

    auto param = util::quick_paramtetr_reader(*node);

    auto elemental_received     = std::atomic<bool> { false };
    auto elemental_pointcloud   = std::make_shared<PointCloud>();
    auto elemental_subscription = node->create_subscription<LivoxMsg>(
        param("elemental.topic", std::string {}), 10, [&](const std::unique_ptr<LivoxMsg>& msg) {
            auto pointcloud = PointCloud {};
            util::livox_to_pcl(msg->points, pointcloud);
            *elemental_pointcloud += pointcloud;
            elemental_received = true;
        });

    auto auxiliary_received     = std::atomic<bool> { false };
    auto auxiliary_pointcloud   = std::make_shared<PointCloud>();
    auto auxiliary_subscription = node->create_subscription<LivoxMsg>(
        param("auxiliary.topic", std::string {}), 10, [&](const std::unique_ptr<LivoxMsg>& msg) {
            auto pointcloud = PointCloud {};
            util::livox_to_pcl(msg->points, pointcloud);
            *auxiliary_pointcloud += pointcloud;
            auxiliary_received = true;
        });

    const auto elemental_t     = param("elemental.t", std::vector<double> {});
    const auto elemental_yaw   = param("elemental.yaw", double {});
    const auto elemental_pitch = param("elemental.pitch", double {});
    const auto elemental_roll  = param("elemental.roll", double {});
    const auto elemental_orientation =
        Eigen::Quaterniond { Eigen::AngleAxisd(
                                 elemental_yaw / 180 * std::numbers::pi, Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(elemental_pitch / 180 * std::numbers::pi, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(elemental_roll / 180 * std::numbers::pi, Eigen::Vector3d::UnitX()) }
            .normalized();
    const auto elemental_translation =
        Eigen::Translation3d { elemental_t[0], elemental_t[1], elemental_t[2] };
    const auto elemental_transform =
        Eigen::Isometry3d { elemental_translation * elemental_orientation };

    const auto auxiliary_t     = param("auxiliary.t", std::vector<double> {});
    const auto auxiliary_yaw   = param("auxiliary.yaw", double {});
    const auto auxiliary_pitch = param("auxiliary.pitch", double {});
    const auto auxiliary_roll  = param("auxiliary.roll", double {});
    const auto auxiliary_orientation =
        Eigen::Quaterniond { Eigen::AngleAxisd(
                                 auxiliary_yaw / 180 * std::numbers::pi, Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(auxiliary_pitch / 180 * std::numbers::pi, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(auxiliary_roll / 180 * std::numbers::pi, Eigen::Vector3d::UnitX()) }
            .normalized();
    const auto auxiliary_translation =
        Eigen::Translation3d { auxiliary_t[0], auxiliary_t[1], auxiliary_t[2] };
    const auto auxiliary_transform =
        Eigen::Isometry3d { auxiliary_translation * auxiliary_orientation };

    rclcpp_info("elemental translation: [%.6f, %.6f, %.6f]", elemental_t[0], elemental_t[1],
        elemental_t[2]);
    rclcpp_info("elemental orientation (quaternion): [x=%.6f, y=%.6f, z=%.6f, w=%.6f]",
        elemental_orientation.x(), elemental_orientation.y(), elemental_orientation.z(),
        elemental_orientation.w());

    rclcpp_info("auxiliary translation: [%.6f, %.6f, %.6f]", auxiliary_t[0], auxiliary_t[1],
        auxiliary_t[2]);
    rclcpp_info("auxiliary orientation (quaternion): [x=%.6f, y=%.6f, z=%.6f, w=%.6f]",
        auxiliary_orientation.x(), auxiliary_orientation.y(), auxiliary_orientation.z(),
        auxiliary_orientation.w());

    using std::chrono::steady_clock;
    auto begin_timestamp    = std::chrono::steady_clock::now();
    auto received_seconds   = std::atomic<long> { LONG_MAX };
    auto ready_registration = std::atomic<bool> { false };

    auto thread = std::thread { [&] {
        while (!(elemental_received && auxiliary_received)) {
            std::this_thread::sleep_for(1s);
            rclcpp_info("等待雷达点云消息中");
        }
        received_seconds = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - begin_timestamp)
                               .count();
        rclcpp_info("雷达已正常发送消息，请等待点云收集完毕");

        ready_registration = true;
    } };

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        rclcpp::sleep_for(10ms);

        if (ready_registration) {
            auto now      = steady_clock::now();
            auto interval = std::chrono::duration_cast<std::chrono::seconds>(now - begin_timestamp);
            if (interval.count() - received_seconds > 0) break;
        }
    }

    pcl::VoxelGrid<Point> voxel_grid;
    voxel_grid.setLeafSize(0.05, 0.05, 0.05);

    voxel_grid.setInputCloud(elemental_pointcloud);
    voxel_grid.filter(*elemental_pointcloud);

    voxel_grid.setInputCloud(auxiliary_pointcloud);
    voxel_grid.filter(*auxiliary_pointcloud);

    rclcpp_info("收集结束，开始配准，点云大小为: %zu, %zu", elemental_pointcloud->size(),
        auxiliary_pointcloud->size());

    auto engine = Engine {};
    engine.setMaxCorrespondenceDistance(1.);
    engine.setMaximumIterations(1000);
    engine.setTransformationEpsilon(0.000'001);
    engine.setEuclideanFitnessEpsilon(0.000'001);

    pcl::transformPointCloud(
        *elemental_pointcloud, *elemental_pointcloud, elemental_transform.cast<float>());
    engine.setInputTarget(elemental_pointcloud);

    pcl::transformPointCloud(
        *auxiliary_pointcloud, *auxiliary_pointcloud, auxiliary_transform.cast<float>());
    engine.setInputSource(auxiliary_pointcloud);

    auto align = PointCloud {};
    engine.align(align);

    auto result = Eigen::Isometry3f { engine.getFinalTransformation() };

    auto t = Eigen::Translation3f { result.translation() };
    auto q = Eigen::Quaternionf { result.rotation() };

    rclcpp_info("t: [%f, %f, %f]", t.x(), t.y(), t.z());
    rclcpp_info("q: [%f, %f, %f, %f]", q.w(), q.x(), q.y(), q.z());

    const auto final_auxiliary_transform = auxiliary_transform * result.cast<double>();

    const auto log_isometry = [&](const Eigen::Isometry3d& iso3d, const std::string& title) {
        const auto trans = iso3d.translation();
        const auto x     = trans.x();
        const auto y     = trans.y();
        const auto z     = trans.z();
        const auto rot   = iso3d.rotation();
        const auto euler = rot.eulerAngles(2, 1, 0);
        const auto yaw   = euler[0] * 180.0 / M_PI;
        const auto pitch = euler[1] * 180.0 / M_PI;
        const auto roll  = euler[2] * 180.0 / M_PI;
        rclcpp_info("%s transform:\n"
                    "    x: %.5f\n"
                    "    y: %.5f\n"
                    "    z: %.5f\n"
                    "    yaw: %.2f\n"
                    "    pitch: %.2f\n"
                    "    roll: %.2f",
            title.c_str(), x, y, z, yaw, pitch, roll);
    };

    log_isometry(auxiliary_transform, "origin");
    log_isometry(final_auxiliary_transform, "final");

    while (rclcpp::ok()) {
        pointcloud_util<0>::publish(*elemental_pointcloud, "calibration");
        pointcloud_util<1>::publish(align, "calibration");
        std::this_thread::sleep_for(1s);
    }

    rclcpp::shutdown();
}
