#pragma once

#include "util/logger.hpp"
#include "util/time.hpp"

#include <optional>
#include <sensor_msgs/msg/imu.hpp>
#include <sophus/so3.hpp>

namespace rmcs::util {

class Imu {
public:
    using Data   = sensor_msgs::msg::Imu;
    using SO3d   = Sophus::SO3d;
    using Result = std::pair<Data, SO3d>;

    void update(const Data& data) {
        if (results_.empty()) initialize(data);

        const auto [last_imu_data, last_rotation] = results_.back();

        const auto last_gyr = Eigen::Vector3d {
            last_imu_data.angular_velocity.x,
            last_imu_data.angular_velocity.y,
            last_imu_data.angular_velocity.z,
        };
        const auto last_timestamp = util::get_time_sec(last_imu_data.header.stamp);

        const auto current_gyr = Eigen::Vector3d {
            data.angular_velocity.x,
            data.angular_velocity.y,
            data.angular_velocity.z,
        };
        const auto current_timestamp = util::get_time_sec(data.header.stamp);

        const auto time_difference = current_timestamp - last_timestamp;
        const auto delta_angle     = time_difference * 0.5 * (last_gyr + current_gyr);
        const auto delta_rotation  = SO3d::exp(delta_angle);

        const auto final_rotation = last_rotation * delta_rotation;

        results_.emplace_back(data, final_rotation);
    }

    void reset(double start_timestamp, const Data& last_imu_data) {
        start_timestamp_.emplace(start_timestamp);
        last_imu_data_.emplace(last_imu_data);
        results_.clear();
    }

    SO3d rotation() const {
        if (results_.empty()) return SO3d {};
        return std::get<1>(results_.back());
    }

private:
    std::optional<double> start_timestamp_ = std::nullopt;
    std::optional<Data> last_imu_data_     = std::nullopt;

    std::vector<Result> results_;

    // 基于上一帧进行插值，权重为相隔的时间，相隔时间越短，权重越大
    void initialize(const Data& data) {
        if (!start_timestamp_.has_value() || !last_imu_data_.has_value())
            throw util::runtime_error("Wrong status of imu initialization");

        const auto last_seconds         = util::get_time_sec(last_imu_data_->header.stamp);
        const auto last_time_difference = *start_timestamp_ - last_seconds;

        const auto current_seconds         = util::get_time_sec(data.header.stamp);
        const auto current_time_difference = current_seconds - *start_timestamp_;

        if (last_time_difference < 0 || current_time_difference < 0)
            throw util::runtime_error("Wrond timestamp of imu initialization");

        const auto sum_time_difference = last_time_difference + current_time_difference + 1e-9;

        const auto last_weight = current_time_difference / sum_time_difference;
        const auto last_gyr    = last_imu_data_->angular_velocity;
        const auto last_acc    = last_imu_data_->linear_acceleration;

        const auto current_weight = last_time_difference / sum_time_difference;
        const auto current_gyr    = data.angular_velocity;
        const auto current_acc    = data.linear_acceleration;

        const auto interpolation = [&](double last, double current) {
            return last_weight * last + current_weight * current;
        };

        auto final_rotation = SO3d {};
        auto final_imu_data = Data {};

        final_imu_data.header.stamp          = util::get_ros_time(*start_timestamp_);
        final_imu_data.angular_velocity.x    = interpolation(last_gyr.x, current_gyr.x);
        final_imu_data.angular_velocity.y    = interpolation(last_gyr.y, current_gyr.y);
        final_imu_data.angular_velocity.z    = interpolation(last_gyr.z, current_gyr.z);
        final_imu_data.linear_acceleration.x = interpolation(last_acc.x, current_acc.x);
        final_imu_data.linear_acceleration.y = interpolation(last_acc.y, current_acc.y);
        final_imu_data.linear_acceleration.z = interpolation(last_acc.z, current_acc.z);

        results_.emplace_back(final_imu_data, final_rotation);
    }
};

}
