#include "dedistortion.hpp"
#include "orthotics.hpp"
#include "util/imu.hpp"
#include "util/logger.hpp"
#include "util/time.hpp"
#include <deque>

using namespace rmcs;

using ImuData   = Orthotics::ImuData;
using LidarData = Orthotics::LidarData;
using Package   = Orthotics::Package;

constexpr auto log_name = [] { return "dedistortion"; };

struct Dedistortion::Impl {
    util::Log<log_name> log;
    util::Imu imu;

    std::atomic<bool> request_reset { false };
    std::condition_variable buffer_cv;
    std::mutex buffer_mutex;
    std::jthread process_thread;

    std::deque<std::unique_ptr<LidarData>> buffer_lidar;
    double timestamp_last_lidar { -1 };

    std::deque<std::unique_ptr<ImuData>> buffer_imu;
    double timestamp_last_imu { -1 };

    auto handle_process(const std::stop_token& stop_token) -> void {
        log.info("Handle process of dedistortion starts");

        auto process_rate = rclcpp::Rate { 1000 };
        while (rclcpp::ok()) {
            auto package     = Package {};
            auto unique_lock = std::unique_lock { buffer_mutex };
            buffer_cv.wait(unique_lock, [this, &package, stop_token] {
                return bool { bind_package(package) || stop_token.stop_requested() };
            });
            unique_lock.unlock();

            if (stop_token.stop_requested()) {
                log.info("Dedistortion process is requested to stop");
                break;
            }
            if (request_reset.load(std::memory_order::relaxed)) {
                log.info("Dedistortion process is requested to reset");
                // TODO:
            }

            process_rate.sleep();
        }
    }

    auto bind_package(Package& package) -> bool {
        if (buffer_lidar.empty() || buffer_imu.empty()) return false;

        if (util::get_time_sec(buffer_imu.front()->header.stamp)
            > util::get_time_sec(buffer_lidar.back()->header.stamp)) {
            buffer_lidar.clear();
            log.error("Clear lidar buffer, this status only happen at the beginning");
            return false;
        }

        if (util::get_time_sec(buffer_imu.back()->header.stamp)
            < util::get_time_sec(buffer_lidar.front()->header.stamp))
            return false;

        package.pointcloud = std::move(buffer_lidar.front());
        buffer_lidar.pop_front();

        const auto timestamp_lidar = util::get_time_sec(package.pointcloud->header.stamp);
        package.imu_data.clear();

        std::erase_if(buffer_imu, [&](std::unique_ptr<ImuData>& data) -> bool {
            const auto timestamp_imu = util::get_time_sec(data->header.stamp);
            if (timestamp_imu <= timestamp_lidar) {
                package.imu_data.emplace_back(std::move(data));
                return true;
            } else return false;
        });

        return true;
    }

    auto append_data_lidar(std::unique_ptr<LidarData> data) -> void {
        const auto timestamp = util::get_time_sec(data->header.stamp);
        buffer_mutex.lock();

        if (timestamp < timestamp_last_lidar) {
            log.error("Lidar timestamp loop back, clear buffer now");
            buffer_lidar.clear();
        }

        timestamp_last_lidar = timestamp;
        buffer_lidar.push_back(std::move(data));

        buffer_mutex.unlock();
        buffer_cv.notify_all();
    }

    auto append_data_imu(std::unique_ptr<ImuData> data) -> void {
        const auto timestamp = util::get_time_sec(data->header.stamp);
        buffer_mutex.lock();

        if (timestamp < timestamp_last_imu) {
            log.error("Imu timestamp loop back, clear buffer now");
            request_reset.store(true, std::memory_order::relaxed);
            buffer_imu.clear();
        }

        timestamp_last_imu = timestamp;
        buffer_imu.push_back(std::move(data));

        buffer_mutex.unlock();
        buffer_cv.notify_all();
    }
};

Dedistortion::Dedistortion()
    : pimpl(std::make_unique<Impl>()) { }

Dedistortion::~Dedistortion() = default;
