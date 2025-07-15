#include "undistortion.hpp"
#include "orthotics.hpp"
#include "util/logger.hpp"
#include <boost/lockfree/spsc_queue.hpp>

using namespace rmcs;

using Logger = util::Log<[] { return "undistortion"; }>;

constexpr auto kLidLoopBackMsg  = "Timestamp of lid regression occurs";
constexpr auto kImuLoopBackMsg  = "Timestamp of imu regression occurs";
constexpr auto kProcessBeginMsg = "Handle process of dedistortion starts";
constexpr auto kStopRequestMsg  = "Undistortion process is requested to stop";
constexpr auto kResetRequestMsg = "Undistortion process is requested to reset";

using lid_capacity = boost::lockfree::capacity<010>;
using imu_capacity = boost::lockfree::capacity<200>;

struct Undistortion::Impl {
    using Point      = pcl::PointXYZ;
    using PointCloud = pcl::PointCloud<Point>;

    using LivoxMsg = livox_ros_driver2::msg::CustomMsg;
    using ImuMsg   = sensor_msgs::msg::Imu;

    Logger log;
    ImuOrthotics orthotics;

    boost::lockfree::spsc_queue<LivoxMsg*, lid_capacity> livox_lid_buffer;
    std::atomic<rcl_time_point_value_t> last_lid_timestamp;

    boost::lockfree::spsc_queue<ImuMsg*, imu_capacity> livox_imu_buffer;
    std::atomic<rcl_time_point_value_t> last_imu_timestamp;

    boost::lockfree::spsc_queue<std::shared_ptr<PointCloud>, lid_capacity>
        undistort_pointcloud_buffer;

    std::jthread process_thread;

    std::atomic<bool> request_reset { false };
    std::atomic<bool> request_cancel_binding { false };

    explicit Impl() noexcept
        : process_thread { [this](const std::stop_token& stop_token) {
            log.info("Handle process of dedistortion starts");

            auto process_rate = rclcpp::Rate { 1000 };
            while (rclcpp::ok()) {
                if (stop_token.stop_requested()) {
                    log.info("Undistortion process is requested to stop");
                    break;
                }
                if (request_reset.load(std::memory_order::relaxed)) {
                    request_reset.store(false, std::memory_order::relaxed);
                    log.info("Undistortion process is requested to reset");
                    orthotics.reset();
                    continue;
                }

                if (const auto optional_package = try_bind_package()) {
                    const auto& package = optional_package.value();

                    auto output = std::make_shared<PointCloud>();
                    orthotics.process(output, package);

                    undistort_pointcloud_buffer.push(output);

                    // log.info("Imu size: %ld", package.imu_msg.size());

                    // const auto lid_timestamp =
                    //     rclcpp::Time { package.lid_msg->header.stamp }.seconds();
                    // const auto imu_timestamp_back =
                    //     rclcpp::Time { package.imu_msg.back()->header.stamp }.seconds();
                    // const auto imu_timestamp_head =
                    //     rclcpp::Time { package.imu_msg.front()->header.stamp }.seconds();
                    // log.info("Lid: %.5fs, Imu back: %.5fs, Imu head: %.5fs", lid_timestamp,
                    //     imu_timestamp_back, imu_timestamp_head);
                }

                process_rate.sleep();
            }
        } } { }

    ~Impl() noexcept {
        process_thread.request_stop();
        if (process_thread.joinable()) process_thread.join();
    }

    auto stop_process() -> void { process_thread.request_stop(); }

    auto try_query_undistort_cloud() -> std::shared_ptr<PointCloud> {
        if (undistort_pointcloud_buffer.empty()) return nullptr;

        auto result = std::make_shared<PointCloud>();
        if (undistort_pointcloud_buffer.pop(result)) {
            return result;
        }

        return nullptr;
    }

    auto push_lid_message(std::unique_ptr<LivMsg> msg) -> void {
        const auto timestamp = rclcpp::Time { msg->header.stamp };
        if (timestamp.nanoseconds() < last_lid_timestamp) {
            request_cancel_binding.store(true);
            while (livox_lid_buffer.pop()) { };
            log.warn("Timestamp of lidar loop back");
        }
        last_lid_timestamp = timestamp.nanoseconds();

        livox_lid_buffer.push(msg.release());
    }
    auto push_imu_message(std::unique_ptr<ImuMsg> msg) -> void {
        const auto timestamp = rclcpp::Time { msg->header.stamp };
        if (timestamp.nanoseconds() < last_imu_timestamp) {
            request_cancel_binding.store(true);
            while (livox_imu_buffer.pop()) { }
            log.warn("Timestamp of imu loop back");
        }
        last_imu_timestamp = timestamp.nanoseconds();

        livox_imu_buffer.push(msg.release());
    }

private:
    auto try_bind_package() -> std::optional<ImuOrthotics::MessageGroup> {

        if (livox_lid_buffer.empty()) return std::nullopt;
        if (livox_imu_buffer.empty()) return std::nullopt;

        const auto imu_timestamp_oldest = rclcpp::Time { livox_imu_buffer.front()->header.stamp };
        // 有 IMU 数据游离于最新雷达数据前无法被打包，去除
        if (last_lid_timestamp < imu_timestamp_oldest.nanoseconds()) {
            log.warn("有 IMU 数据游离于最新雷达数据前无法被打包，去除");
            while (livox_lid_buffer.pop()) { }
            return std::nullopt;
        }

        const auto lid_timestamp_oldest = rclcpp::Time { livox_lid_buffer.front()->header.stamp };
        // 还未出现新于最旧雷达数据的 IMU 数据，继续等待
        if (lid_timestamp_oldest.nanoseconds() > last_imu_timestamp) {
            log.warn("还未出现新于最旧雷达数据的 IMU 数据，继续等待");
            return std::nullopt;
        }

        auto result = ImuOrthotics::MessageGroup {};

        result.lid_msg = std::unique_ptr<LivMsg>(livox_lid_buffer.front());
        livox_lid_buffer.pop();

        const auto lid_timestamp = rclcpp::Time { result.lid_msg->header.stamp };
        while (!livox_imu_buffer.empty()) {
            const auto imu_msg       = livox_imu_buffer.front();
            const auto imu_timestamp = rclcpp::Time { imu_msg->header.stamp };

            if (imu_timestamp.nanoseconds() < lid_timestamp.nanoseconds()) {
                result.imu_msg.emplace_back(std::unique_ptr<ImuMsg>(imu_msg));
                livox_imu_buffer.pop();
            } else break;
        }

        if (result.imu_msg.empty()) return std::nullopt;

        if (request_cancel_binding.load()) {
            request_cancel_binding.store(false);
            return std::nullopt;
        }

        return result;
    }
};

Undistortion::Undistortion()
    : pimpl(std::make_unique<Impl>()) { }

Undistortion::~Undistortion() = default;

auto Undistortion::set_imu_transform(const Eigen::Isometry3d& t) -> void {
    pimpl->orthotics.set_imu_transform(t);
}

auto Undistortion::set_lid_transform(const Eigen::Isometry3d& t) -> void {
    pimpl->orthotics.set_lid_transform(t);
}

auto Undistortion::push_lid_message(std::unique_ptr<LivMsg> msg) -> void {
    pimpl->push_lid_message(std::move(msg));
}

auto Undistortion::push_imu_message(std::unique_ptr<ImuMsg> msg) -> void {
    pimpl->push_imu_message(std::move(msg));
}

auto Undistortion::stop_process() -> void { pimpl->stop_process(); }

auto Undistortion::try_query_undistort_cloud() -> std::shared_ptr<ImuOrthotics::CloudXYZ> {
    return pimpl->try_query_undistort_cloud();
}
