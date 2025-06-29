#include "orthotics.hpp"
#include "util/imu.hpp"

constexpr auto kNameGetter = [] { return "orthotics"; };
constexpr auto kEnableLog  = true;
using namespace rmcs;

struct Orthotics::Impl {
    util::Log<kNameGetter> log;
    util::Imu imu;

    bool receive_first_data { true };
    double interval { 0. };

    Sophus::SE3d transform {};

    std::shared_ptr<PointCloud> pointcloud_source;
    std::shared_ptr<PointCloud> pointcloud_output;

    std::unique_ptr<LidarData> data_last_lidar;
    std::unique_ptr<ImuData> data_last_imu;

    auto reset() -> void {
        log.info("Orthotics is requested to reset");

        imu.reset(-1, std::nullopt);

        receive_first_data = true;
        data_last_lidar    = std::make_unique<LidarData>();
        data_last_imu      = std::make_unique<ImuData>();

        pointcloud_source = std::make_shared<PointCloud>();
        pointcloud_output = std::make_shared<PointCloud>();
    }

    auto process(const Package& package) -> void {
        if (package.imu_data.empty() || !package.pointcloud)
            throw util::runtime_error("Package is not correct");

        if constexpr (kEnableLog) {
            const auto timestamp_lidar = util::get_time_sec(package.pointcloud->header.stamp);
            const auto timestamp_front = util::get_time_sec(package.imu_data.front()->header.stamp);
            const auto timestamp_back  = util::get_time_sec(package.imu_data.back()->header.stamp);
            const auto data_size       = package.imu_data.size();
            const auto output_format   = "Process: lidar[%.4f] imu[%lu][%.4f %.4f]";
            log.info(output_format, timestamp_lidar, data_size, timestamp_front, timestamp_back);
        }

        if (receive_first_data) {
            receive_first_data = false;

            reset();
            *data_last_lidar = *package.pointcloud;
            *data_last_imu   = *package.imu_data.back();

            return;
        }

        integrate(package.imu_data);
        const auto rotation  = imu.rotation();
        const auto transform = Sophus::SE3d { rotation, Eigen::Vector3d::Zero() };
    }

private:
    auto integrate(const std::vector<std::unique_ptr<ImuData>>& data) -> void {

        const auto second = util::get_time_sec(data_last_lidar->header.stamp);
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

    // TODO: 这个诡异的测试用的点云记得去掉
    static inline auto pointcloud_correct = std::make_shared<PointCloud>();

    /// @note 事实上，源代码这里风格极其糟糕，这是不得不吐槽的事实
    static inline auto undistort(std::shared_ptr<PointCloud>& pointcloud, double interval_total,
        const Sophus::SE3d& transform) -> void {

        const auto& translation = Eigen::Vector3d { transform.translation() };
        const auto& rotate_vec  = Eigen::Vector3d { transform.so3().log() };

        for (auto& point : *pointcloud) {
            const auto ring     = static_cast<int>(point.intensity);
            const auto interval = point.intensity - static_cast<double>(ring);

            if (interval == 0) pointcloud_correct->push_back(point);

            const auto ratio_begin_point = interval / interval_total;
            const auto ratio_point_end   = 1. - ratio_begin_point;

            const auto rotate_vec_point_end = Eigen::Vector3d { ratio_point_end * rotate_vec };
            const auto rotation_point_end   = Sophus::SO3d::exp(rotate_vec_point_end);

            const auto translation_point_end = Eigen::Vector3d { ratio_point_end * translation };

            const auto point_current   = Eigen::Vector3d { point.x, point.y, point.z };
            const auto point_undistort = rotation_point_end.inverse()
                * Eigen::Vector3d { point_current - translation_point_end };

            point.x = static_cast<float>(point_undistort.x());
            point.y = static_cast<float>(point_undistort.y());
            point.z = static_cast<float>(point_undistort.z());
        }
    }
};

Orthotics::Orthotics()
    : pimpl { std::make_unique<Impl>() } { /* TODO:*/ }

Orthotics::~Orthotics() = default;

auto Orthotics::update(const Package& package) -> void { /* TODO:*/ }
auto Orthotics::reset() -> void { /* TODO:*/ }

auto Orthotics::set_transform(const Eigen::Isometry3d& transform) -> void { /* TODO:*/ }

auto Orthotics::undistort(std::shared_ptr<PointCloud>& pointcloud, double interval,
    const Sophus::SE3d& transform) const -> void { /* TODO:*/ }
