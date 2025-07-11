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

using lid_capacity = boost::lockfree::capacity<05>;
using imu_capacity = boost::lockfree::capacity<20>;

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

    std::condition_variable bind_action_notifiction;

    explicit Impl() noexcept {
        process_thread = std::jthread { [this](const std::stop_token& stop_token) {
            log.info(kProcessBeginMsg);

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

                auto package = try_bind_package();
                if (!package.has_value()) continue;

                auto output = std::make_shared<ImuOrthotics::CloudXYZ>();
                orthotics.process(output, package.value());

                while (!undistort_pointcloud_buffer.push(output))
                    std::this_thread::yield();

                process_rate.sleep();
            }
        } };
    }

    ~Impl() noexcept {
        process_thread.request_stop();
        if (process_thread.joinable()) process_thread.join();
    }

    auto stop_process() -> void { process_thread.request_stop(); }

    auto try_query_undistort_cloud() -> std::shared_ptr<ImuOrthotics::CloudXYZ> {
        if (undistort_pointcloud_buffer.empty()) return nullptr;

        auto result = std::make_shared<ImuOrthotics::CloudXYZ>();
        if (undistort_pointcloud_buffer.pop(result)) return result;
        else return nullptr;
    }

    auto handle_lid_message(std::unique_ptr<LivoMsg> msg) -> void {
        const auto timestamp = rclcpp::Time { msg->header.stamp };
        if (timestamp.nanoseconds() < last_lid_timestamp) {
            while (livox_lid_buffer.pop()) { };
            log.warn("Timestamp of lidar loop back");
        }
        last_lid_timestamp = timestamp.nanoseconds();

        const auto release = msg.release();
        while (!livox_lid_buffer.push(release))
            std::this_thread::yield();

        bind_action_notifiction.notify_all();
    }
    auto handle_imu_message(std::unique_ptr<ImuData> msg) -> void {
        const auto timestamp = rclcpp::Time { msg->header.stamp };
        if (timestamp.nanoseconds() < last_imu_timestamp) {
            while (livox_imu_buffer.pop()) { }
            log.warn("Timestamp of imu loop back");
        }
        last_imu_timestamp = timestamp.nanoseconds();

        const auto release = msg.release();
        while (!livox_imu_buffer.push(release))
            std::this_thread::yield();

        bind_action_notifiction.notify_all();
    }

private:
    auto try_bind_package() -> std::optional<Package> {
        if (lid_buffer.empty() || imu_buffer.empty()) return std::nullopt;

        const auto lid_timestamp_newest = rclcpp::Time { lid_buffer.back().timestamp };
        const auto imu_timestamp_oldest = rclcpp::Time { imu_buffer.front().header.stamp };
        // 有 IMU 数据游离于最新雷达数据前无法被打包，去除
        if (lid_timestamp_newest.nanoseconds() < imu_timestamp_oldest.nanoseconds()) {
            log.warn("有 IMU 数据游离于最新雷达数据前无法被打包，去除");
            return lid_buffer.clear(), std::nullopt;
        }

        const auto lid_timestamp_oldest = rclcpp::Time { lid_buffer.front().timestamp };
        const auto imu_timestamp_newest = rclcpp::Time { imu_buffer.back().header.stamp };
        // 还未出现新于最旧雷达数据的 IMU 数据，继续等待
        if (lid_timestamp_oldest.nanoseconds() > imu_timestamp_newest.nanoseconds()) {
            log.warn("还未出现新于最旧雷达数据的 IMU 数据，继续等待");
            return std::nullopt;
        }

        log.info("Data to bind, imu: %ld", imu_buffer.size());
        auto result = Package {};

        result.lid_data = std::make_unique<LidData>(lid_buffer.front());
        lid_buffer.pop_front();

        const auto lid_timestamp = result.lid_data->timestamp;
        std::erase_if(imu_buffer, [&](const ImuData& data) {
            const auto imu_timestamp = rclcpp::Time { data.header.stamp };
            if (imu_timestamp.nanoseconds() <= lid_timestamp.nanoseconds()) {
                result.imu_data.push_back(std::make_unique<ImuData>(data));
                return true;
            } else return false;
        });

        return result.imu_data.empty() ? std::nullopt : std::optional { std::move(result) };
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

auto Undistortion::handle_lid_message(std::unique_ptr<LivoMsg> msg) -> void {
    pimpl->handle_lid_message(std::move(msg));
}

auto Undistortion::handle_imu_message(std::unique_ptr<ImuData> msg) -> void {
    pimpl->handle_imu_message(std::move(msg));
}

auto Undistortion::stop_process() -> void { pimpl->stop_process(); }

auto Undistortion::try_query_undistort_cloud() -> std::shared_ptr<ImuOrthotics::CloudXYZ> {
    return pimpl->try_query_undistort_cloud();
}
