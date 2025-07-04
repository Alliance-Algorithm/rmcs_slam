#include "undistortion.hpp"
#include "orthotics.hpp"
#include "util/logger.hpp"
#include <boost/lockfree/spsc_queue.hpp>
#include <deque>

using namespace rmcs;

constexpr auto kLogName         = [] { return "undistortion"; };
constexpr auto kLidLoopBackMsg  = "Timestamp of lid regression occurs";
constexpr auto kImuLoopBackMsg  = "Timestamp of imu regression occurs";
constexpr auto kProcessBeginMsg = "Handle process of dedistortion starts";
constexpr auto kStopRequestMsg  = "Undistortion process is requested to stop";
constexpr auto kResetRequestMsg = "Undistortion process is requested to reset";

struct Undistortion::Impl {
    util::Log<kLogName> log;
    ImuOrthotics orthotics;

    std::deque<LidData> lid_buffer;
    std::deque<ImuData> imu_buffer;

    rclcpp::Time last_lid_timestamp;
    rclcpp::Time last_imu_timestamp;

    std::jthread process_thread;

    std::atomic<bool> request_reset { false };

    std::mutex buffer_mutex;
    std::condition_variable buffer_cv;

    using Object  = std::shared_ptr<ImuOrthotics::CloudXYZ>;
    using Capcity = boost::lockfree::capacity<10>;
    boost::lockfree::spsc_queue<Object, Capcity> undistort_cloud;

    explicit Impl() noexcept {
        process_thread = std::jthread { [this](const std::stop_token& stop_token) {
            log.info(kProcessBeginMsg);

            auto process_rate = rclcpp::Rate { 1000 };
            while (rclcpp::ok()) {
                auto package = Package {};
                {
                    auto unique_lock = std::unique_lock { buffer_mutex };
                    buffer_cv.wait(unique_lock, [&package, this] -> bool {
                        if (auto result = try_bind_package()) {
                            package = std::move(result.value());
                            return !process_thread.get_stop_token().stop_requested();
                        }
                        return false;
                    });
                }

                if (stop_token.stop_requested()) {
                    log.info(kStopRequestMsg);
                    break;
                }

                if (request_reset.load(std::memory_order::relaxed)) {
                    request_reset.store(false, std::memory_order::relaxed);

                    log.info(kResetRequestMsg);
                    orthotics.reset();

                    continue;
                }

                auto output = std::make_shared<ImuOrthotics::CloudXYZ>();
                orthotics.process(output, package);

                while (!undistort_cloud.push(output))
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
        if (undistort_cloud.empty()) return nullptr;

        auto result = std::make_shared<ImuOrthotics::CloudXYZ>();
        if (undistort_cloud.pop(result)) return result;
        else return nullptr;
    }

    auto handle_lid_message(const std::unique_ptr<LivoMsg>& msg) -> void {
        std::lock_guard _ { buffer_mutex };

        const auto timestamp = rclcpp::Time { msg->header.stamp };
        if (timestamp.nanoseconds() < last_lid_timestamp.nanoseconds()) {
            lid_buffer.clear();
            log.error(kLidLoopBackMsg);
        }
        last_lid_timestamp = timestamp;

        auto interval   = msg->points.back().offset_time;
        auto pointcloud = LidData { {}, timestamp };
        pointcloud.reserve(msg->point_num);

        for (const auto& livox_point : msg->points) {
            auto point = Point {};

            point.x = livox_point.x;
            point.y = livox_point.y;
            point.z = livox_point.z;

            point.interval_ratio = static_cast<double>(livox_point.offset_time) //
                / static_cast<double>(interval);

            pointcloud.points.push_back(point);
        }
        lid_buffer.push_back(pointcloud);

        buffer_cv.notify_all();
    }
    auto handle_imu_message(const std::unique_ptr<ImuData>& msg) -> void {
        std::lock_guard _ { buffer_mutex };

        const auto timestamp = rclcpp::Time { msg->header.stamp };
        if (timestamp.nanoseconds() < last_imu_timestamp.nanoseconds()) {
            imu_buffer.clear();
            log.error(kImuLoopBackMsg);
        }
        last_imu_timestamp = timestamp;

        imu_buffer.push_back(*msg);

        buffer_cv.notify_all();
    }

private:
    auto try_bind_package() -> std::optional<Package> {
        if (lid_buffer.empty() || imu_buffer.empty()) return std::nullopt;

        const auto lid_timestamp_oldest = rclcpp::Time { lid_buffer.front().timestamp };
        const auto imu_timestamp_newest = rclcpp::Time { imu_buffer.back().header.stamp };
        // 还未出现新于最旧雷达数据的 IMU 数据，继续等待
        if (lid_timestamp_oldest.nanoseconds() > imu_timestamp_newest.nanoseconds()) {
            return std::nullopt;
        }

        const auto lid_timestamp_newest = rclcpp::Time { lid_buffer.back().timestamp };
        const auto imu_timestamp_oldest = rclcpp::Time { imu_buffer.front().header.stamp };
        // 有 IMU 数据游离于最新雷达数据前无法被打包，去除
        if (lid_timestamp_newest.nanoseconds() < imu_timestamp_oldest.nanoseconds()) {
            return lid_buffer.clear(), std::nullopt;
        }

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

auto Undistortion::handle_lid_message(const std::unique_ptr<LivoMsg>& msg) -> void {
    pimpl->handle_lid_message(msg);
}

auto Undistortion::handle_imu_message(const std::unique_ptr<ImuData>& msg) -> void {
    pimpl->handle_imu_message(msg);
}

auto Undistortion::stop_process() -> void { pimpl->stop_process(); }

auto Undistortion::try_query_undistort_cloud() -> std::shared_ptr<ImuOrthotics::CloudXYZ> {
    return pimpl->try_query_undistort_cloud();
}
