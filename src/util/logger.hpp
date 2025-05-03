#pragma once

#include <rclcpp/logging.hpp>
#include <utility>

// 转发参数到这个日志会让 clangd 报一个 warning
// 故禁用掉这个 warning
#pragma clang diagnostic ignored "-Wformat-security"

#define RMCS_INITIALIZE_LOGGER(NAME)                                           \
    rclcpp::Logger _internal_logger{rclcpp::get_logger(NAME)};                 \
                                                                               \
    inline void rclcpp_info(auto&&... args) const {                            \
        RCLCPP_INFO(_internal_logger, std::forward<decltype(args)>(args)...);  \
    }                                                                          \
    inline void rclcpp_warn(auto&&... args) const {                            \
        RCLCPP_WARN(_internal_logger, std::forward<decltype(args)>(args)...);  \
    }                                                                          \
    inline void rclcpp_error(auto&&... args) const {                           \
        RCLCPP_ERROR(_internal_logger, std::forward<decltype(args)>(args)...); \
    }

namespace internal {

struct _internal_use_include_library_to_remove_clangd_warning {
    RMCS_INITIALIZE_LOGGER("name_to_generate_logger");
};

} // namespace internal
