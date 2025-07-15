#include "orthotics.hpp"
#include "sophus/se3.hpp"
#include "util/imu.hpp"
#include "util/logger.hpp"

#include <pcl_conversions/pcl_conversions.h>

using namespace rmcs;
using Logger = util::Log<[] { return "orthotics"; }>;

struct ImuOrthotics::Impl {
    Logger log;
    util::Imu imu;

    Eigen::Isometry3d transform_lidar_robot { Eigen::Isometry3d::Identity() };
    Sophus::SE3d transform_lidar_imu { Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero() };
    bool receive_first { true };

    std::unique_ptr<LivMsg> last_lid;
    std::unique_ptr<ImuMsg> last_imu;

    auto reset() -> void {
        log.info("Orthotics is requested to reset");

        imu.reset(-1, std::nullopt);

        last_lid = std::make_unique<LivMsg>();
        last_imu = std::make_unique<ImuMsg>();

        receive_first = true;
    }

    template <concept_point Point>
    auto process(std::shared_ptr<pcl::PointCloud<Point>>& output, const MessageGroup& package)
        -> void {

        if (package.imu_msg.empty() || package.lid_msg == nullptr)
            throw util::runtime_error("The message group is invalid");

        if (receive_first) {
            reset();
            *last_lid     = *package.lid_msg;
            *last_imu     = *package.imu_msg.back();
            receive_first = false;
            return;
        }

        this->integrate(package.imu_msg);

        const auto transform = Sophus::SE3d { imu.rotation(), Eigen::Vector3d::Zero() };
        const auto transform_total =
            transform_lidar_imu.inverse() * transform * transform_lidar_imu;
        const auto interval_total = rclcpp::Time(package.lid_msg->header.stamp).seconds()
            - rclcpp::Time(last_lid->header.stamp).seconds();

        this->undistort(
            output, package.lid_msg, interval_total, transform_total, transform_lidar_robot);

        *last_lid = *package.lid_msg;
        *last_imu = *package.imu_msg.back();
    }

private:
    auto integrate(const std::vector<std::unique_ptr<ImuMsg>>& data) -> void {

        if (last_lid == nullptr) {
            throw util::runtime_error("Last lidar data is null");
        }

        imu.reset(rclcpp::Time { last_lid->header.stamp }.seconds(), *last_imu);

        for (const auto& imu_frame : data)
            imu.update(*imu_frame);
    }

    /// @note 事实上，源代码这里风格极其糟糕，这是不得不吐槽的事实
    template <concept_point Point>
    inline auto undistort(std::shared_ptr<pcl::PointCloud<Point>>& output,
        const std::unique_ptr<LivMsg>& source, double interval_total,
        const Sophus::SE3d& imu_transform, const Eigen::Isometry3d& lid_transform) -> void {

        const auto& translation = Eigen::Vector3d { imu_transform.translation() };
        const auto& rotate_vec  = Eigen::Vector3d { imu_transform.so3().log() };

        const auto timestamp_end = static_cast<double>(source->points.back().offset_time);

        // std::array<double, 20> ratios_point_end;
        // std::array<Eigen::Vector3d, 20> points_origin;
        // std::array<Eigen::Vector3d, 20> points_undistort;

        output->clear();
        output->resize(source->point_num);

        auto index = std::size_t { 0 };
        for (const auto& point : source->points) {

            const auto ratio_begin_point = point.offset_time / timestamp_end;
            const auto ratio_point_end   = 1. - ratio_begin_point;

            const auto rotate_vec_point_end = Eigen::Vector3d { 1. * ratio_point_end * rotate_vec };
            const auto rotation_point_end   = Sophus::SO3d::exp(rotate_vec_point_end);

            const auto translation_point_end = Eigen::Vector3d { ratio_point_end * translation };

            const auto point_current   = Eigen::Vector3d { point.x, point.y, point.z };
            const auto point_undistort = rotation_point_end.inverse()
                * Eigen::Vector3d { point_current - translation_point_end };

            const auto point_result = lid_transform * point_undistort;
            output->points[index].x = point_result.x(), output->points[index].y = point_result.y(),
            output->points[index].z = point_result.z();

            index = index + 1;

            // if (const auto array_index = index / 1'000; index % 1'000 == 0) {
            //     ratios_point_end[array_index] = ratio_point_end;
            //     points_origin[array_index]    = point_current;
            //     points_undistort[array_index] = point_undistort;
            // }
        }

        output->width    = output->size();
        output->height   = 1;
        output->is_dense = true;

        // auto ratios_string = std::string {};
        // for (const auto& ratio : ratios_point_end)
        //     ratios_string += std::to_string(ratio) + " ";
        //
        // log.info("total interval: %.4f, points interval: %.4f, ratios: %s", interval_total,
        //     interval_points, ratios_string.c_str());
        //
        // auto rotation_text = std::string {};
        // rotation_text += "X: " + std::to_string(imu_transform.angleX()) + "rad ";
        // rotation_text += "Y: " + std::to_string(imu_transform.angleY()) + "rad ";
        // rotation_text += "Z: " + std::to_string(imu_transform.angleZ()) + "rad ";
        // log.info("%s", rotation_text.c_str());
        //
        // for (auto index = 0; index < points_origin.size(); index++) {
        //     log.info("Origin:    %+.4f %+.4f %+.4f", points_origin[index].x(),
        //         points_origin[index].y(), points_origin[index].z());
        //     log.info("Undistort: %+.4f %+.4f %+.4f", points_undistort[index].x(),
        //         points_undistort[index].y(), points_undistort[index].z());
        // }
    }
};

ImuOrthotics::ImuOrthotics()
    : pimpl { std::make_unique<Impl>() } { }

ImuOrthotics::~ImuOrthotics() = default;

auto ImuOrthotics::process(std::shared_ptr<CloudXYZ>& output, const MessageGroup& package) -> void {
    pimpl->process(output, package);
}
auto ImuOrthotics::process(std::shared_ptr<CloudXYZI>& output, const MessageGroup& package)
    -> void {
    pimpl->process(output, package);
}

auto ImuOrthotics::set_imu_transform(const Eigen::Isometry3d& transform) -> void {
    pimpl->transform_lidar_imu = Sophus::SE3d { transform.rotation(), transform.translation() };
}
auto ImuOrthotics::set_lid_transform(const Eigen::Isometry3d& transform) -> void {
    pimpl->transform_lidar_robot = transform;
}

auto ImuOrthotics::reset() -> void { pimpl.reset(); }
