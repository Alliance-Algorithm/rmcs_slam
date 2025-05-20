#include "dedistortion.hpp"
#include "sophus/se3.hpp"
#include "util/imu.hpp"
#include "util/logger.hpp"

using namespace rmcs;

struct Dedistortion::Impl {
    RMCS_INITIALIZE_LOGGER("dedistortion");

    std::shared_ptr<LidarData> last_pointcloud = nullptr;
    std::shared_ptr<ImuData> last_imu_data     = nullptr;

    std::shared_ptr<PointCloud> pointcloud_source = nullptr;
    std::shared_ptr<PointCloud> pointcloud_output = nullptr;

    util::Imu imu;

    bool is_first_process = true;

    Sophus::SE3d extrinsic_transform { Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero() };

    void reset() {
        rclcpp_info("Dedistortion is reset now");

        is_first_process = true;
        last_pointcloud  = nullptr;
        last_imu_data    = nullptr;

        pointcloud_source = std::make_shared<PointCloud>();
        pointcloud_output = std::make_shared<PointCloud>();
    }

    void update_lidar(const std::unique_ptr<livox_ros_driver2::msg::CustomMsg>& msg) { }

    void integrate_gyroscope(const std::vector<std::shared_ptr<ImuData>>& imu_data) { }

    static void dedistort_pointcloud(const std::shared_ptr<PointCloud>& pointcloud_source,
        double scan_period, const Sophus::SE3d& transform) {

        if (scan_period < 1e-6) return;

        const auto& translation = transform.translation();
        const auto& orientation = transform.so3().log();

        for (auto& point : pointcloud_source->points) {
            // 获取该点环数
            const auto ring_num = static_cast<int>(point.intensity);
            // 获取时间差
            const auto time_diff = point.intensity - static_cast<float>(ring_num);
        }
    }
};

Dedistortion::Dedistortion()
    : pimpl(std::make_unique<Impl>()) { }

Dedistortion::~Dedistortion() = default;
