#include "orthotics.hpp"
#include "sophus/se3.hpp"
#include "util/imu.hpp"
#include <pcl_conversions/pcl_conversions.h>

constexpr auto kNameGetter = [] { return "orthotics"; };
constexpr auto kEnableLog  = false;
using namespace rmcs;

struct ImuOrthotics::Impl {
    util::Log<kNameGetter> log;
    util::Imu imu;

    Eigen::Isometry3d transform_lidar_robot { Eigen::Isometry3d::Identity() };
    Sophus::SE3d transform_lidar_imu { Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero() };
    bool receive_first_data { true };

    std::unique_ptr<LidData> data_last_lidar;
    std::unique_ptr<ImuData> data_last_imu;

    auto reset() -> void {
        log.info("Orthotics is requested to reset");

        imu.reset(-1, std::nullopt);

        data_last_lidar = std::make_unique<LidData>();
        data_last_imu   = std::make_unique<ImuData>();

        receive_first_data = true;
    }

    template <concept_point Point>
    auto process(std::shared_ptr<pcl::PointCloud<Point>>& output, const Package& package) -> void {
        if (package.imu_data.empty() || !package.lid_data)
            throw util::runtime_error("Package is not correct");

        if constexpr (kEnableLog) {
            const auto timestamp_lidar = util::get_time_sec(package.lid_data->timestamp);
            const auto timestamp_front = util::get_time_sec(package.imu_data.front()->header.stamp);
            const auto timestamp_back  = util::get_time_sec(package.imu_data.back()->header.stamp);
            const auto data_size       = package.imu_data.size();
            const auto output_format   = "Process: lidar[%.4f] imu[%lu][%.4f %.4f]";
            log.info(output_format, timestamp_lidar, data_size, timestamp_front, timestamp_back);
        }

        if (receive_first_data) {
            reset();
            *data_last_lidar   = *package.lid_data;
            *data_last_imu     = *package.imu_data.back();
            receive_first_data = false;
            return;
        }

        integrate(package.imu_data);

        const auto transform = Sophus::SE3d { imu.rotation(), Eigen::Vector3d::Zero() };
        const auto transform_total =
            transform_lidar_imu.inverse() * transform * transform_lidar_imu;
        const auto interval_total = util::get_time_sec(package.lid_data->timestamp)
            - util::get_time_sec(data_last_lidar->timestamp);

        undistort(output, package.lid_data, interval_total, transform_total, transform_lidar_robot);

        *data_last_lidar = *package.lid_data;
        *data_last_imu   = *package.imu_data.back();
    }

private:
    auto integrate(const std::vector<std::unique_ptr<ImuData>>& data) -> void {

        const auto second = rclcpp::Time { data_last_lidar->timestamp }.seconds();
        imu.reset(second, *data_last_imu);

        for (const auto& imu_frame : data)
            imu.update(*imu_frame);

        if constexpr (kEnableLog) {
            const auto to_degree = [](double angle) { return angle * 180. / std::numbers::pi; };

            const auto x = to_degree(imu.rotation().angleX());
            const auto y = to_degree(imu.rotation().angleY());
            const auto z = to_degree(imu.rotation().angleZ());

            const auto format = "Integrate rotation angle [x, y, z]: [%.2f, %.2f, %.2f]";
            log.info(format, x, y, z);
        }
    }

    /// @note 事实上，源代码这里风格极其糟糕，这是不得不吐槽的事实
    template <concept_point Point>
    static inline auto undistort(std::shared_ptr<pcl::PointCloud<Point>>& output,
        const std::unique_ptr<LidData>& source, double interval_total,
        const Sophus::SE3d& imu_transform, const Eigen::Isometry3d& lid_transform) -> void {

        const auto& translation = Eigen::Vector3d { imu_transform.translation() };
        const auto& rotate_vec  = Eigen::Vector3d { imu_transform.so3().log() };

        output->clear();
        output->resize(source->size());
        auto index = std::size_t { 0 };
        for (const auto& point : *source) {
            const auto ratio_begin_point = point.interval_ratio;
            const auto ratio_point_end   = 1. - ratio_begin_point;

            const auto rotate_vec_point_end = Eigen::Vector3d { ratio_point_end * rotate_vec };
            const auto rotation_point_end   = Sophus::SO3d::exp(rotate_vec_point_end);

            const auto translation_point_end = Eigen::Vector3d { ratio_point_end * translation };

            const auto point_current   = Eigen::Vector3d { point.x, point.y, point.z };
            const auto point_undistort = rotation_point_end.inverse()
                * Eigen::Vector3d { point_current - translation_point_end };

            const auto point_result = lid_transform * point_undistort;

            auto& output_point = output->points[index++];
            output_point.x     = static_cast<float>(point_result.x());
            output_point.y     = static_cast<float>(point_result.y());
            output_point.z     = static_cast<float>(point_result.z());
        }
        output->width    = output->size();
        output->height   = 1;
        output->is_dense = true;
    }
};

ImuOrthotics::ImuOrthotics()
    : pimpl { std::make_unique<Impl>() } { }

ImuOrthotics::~ImuOrthotics() = default;

auto ImuOrthotics::process(std::shared_ptr<CloudXYZ>& output, const Package& package) -> void {
    pimpl->process(output, package);
}
auto ImuOrthotics::process(std::shared_ptr<CloudXYZI>& output, const Package& package) -> void {
    pimpl->process(output, package);
}

auto ImuOrthotics::set_imu_transform(const Eigen::Isometry3d& transform) -> void {
    pimpl->transform_lidar_imu = Sophus::SE3d { transform.rotation(), transform.translation() };
}
auto ImuOrthotics::set_lid_transform(const Eigen::Isometry3d& transform) -> void {
    pimpl->transform_lidar_robot = transform;
}

auto ImuOrthotics::reset() -> void { pimpl.reset(); }
