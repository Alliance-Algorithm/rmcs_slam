#pragma once

#include <builtin_interfaces/msg/time.hpp>
#include <cmath>
#include <rclcpp/time.hpp>

namespace rmcs::util {

inline double get_time_sec(const builtin_interfaces::msg::Time& time) {
    return rclcpp::Time(time).seconds();
}

inline rclcpp::Time get_ros_time(double timestamp) {
    int32_t sec    = std::floor(timestamp);
    auto nanosec_d = (timestamp - std::floor(timestamp)) * 1e9;
    auto nanosec   = static_cast<uint32_t>(nanosec_d);
    return { sec, nanosec };
}

}
