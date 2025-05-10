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

namespace rmcs::util {

namespace ansi {

// 基础前景色 (30-37)
constexpr auto kReset               = "\033[0m";
constexpr auto kForegroundBlack     = "\033[30m";
constexpr auto kForegroundRed       = "\033[31m";
constexpr auto kForegroundGreen     = "\033[32m";
constexpr auto kForegroundYellow    = "\033[33m";
constexpr auto kForegroundBlue      = "\033[34m";
constexpr auto kForegroundPurple    = "\033[35m";
constexpr auto kForegroundDarkGreen = "\033[36m";
constexpr auto kForegroundWhite     = "\033[37m";

// 基础背景色 (40-47)
constexpr auto kBackgroundBlack     = "\033[40m";
constexpr auto kBackgroundRed       = "\033[41m";
constexpr auto kBackgroundGreen     = "\033[42m";
constexpr auto kBackgroundYellow    = "\033[43m";
constexpr auto kBackgroundBlue      = "\033[44m";
constexpr auto kBackgroundPurple    = "\033[45m";
constexpr auto kBackgroundDarkGreen = "\033[46m";
constexpr auto kBackgroundWhite     = "\033[47m";

// 高亮前景色 (90-97)
constexpr auto kForegroundBrightBlack     = "\033[90m";
constexpr auto kForegroundBrightRed       = "\033[91m";
constexpr auto kForegroundBrightGreen     = "\033[92m";
constexpr auto kForegroundBrightYellow    = "\033[93m";
constexpr auto kForegroundBrightBlue      = "\033[94m";
constexpr auto kForegroundBrightPurple    = "\033[95m";
constexpr auto kForegroundBrightDarkGreen = "\033[96m";
constexpr auto kForegroundBrightWhite     = "\033[97m";

// 高亮背景色 (100-107)
constexpr auto kBackgroundBrightBlack     = "\033[100m";
constexpr auto kBackgroundBrightRed       = "\033[101m";
constexpr auto kBackgroundBrightGreen     = "\033[102m";
constexpr auto kBackgroundBrightYellow    = "\033[103m";
constexpr auto kBackgroundBrightBlue      = "\033[104m";
constexpr auto kBackgroundBrightPurple    = "\033[105m";
constexpr auto kBackgroundBrightDarkGreen = "\033[106m";
constexpr auto kBackgroundBrightWhite     = "\033[107m";

// 文本样式
constexpr auto kStyleBold      = "\033[1m";
constexpr auto kStyleUnderline = "\033[4m";
constexpr auto kStyleBlink     = "\033[5m";
constexpr auto kStyleReverse   = "\033[7m";
constexpr auto kStyleHidden    = "\033[8m";

// 光标控制
constexpr auto kCursorUp      = "\033[A";    // 上移一行
constexpr auto kCursorDown    = "\033[B";    // 下移一行
constexpr auto kCursorRight   = "\033[C";    // 右移一行
constexpr auto kCursorLeft    = "\033[D";    // 左移一行
constexpr auto kCursorSave    = "\033[s";    // 保存光标位置
constexpr auto kCursorRestore = "\033[u";    // 恢复光标位置
constexpr auto kCursorHide    = "\033[?25l"; // 隐藏光标
constexpr auto kCursorShow    = "\033[?25h"; // 显示光标

// 清除控制
constexpr auto kClearScreen = "\033[2J"; // 清屏
constexpr auto kClearLine   = "\033[K";  // 清除从光标到行尾的内容

} // namespace ansi

inline auto colored(const char* const color, const std::string& text) {
    return std::string{color + text + ansi::kReset};
}

inline auto error(const std::string& text) {
    return std::string{ansi::kForegroundBrightRed + text + ansi::kReset};
}

inline auto runtime_error(const std::string& text) { return std::runtime_error{error(text)}; }

} // namespace rmcs::util

namespace internal {

struct _internal_use_include_library_to_remove_clangd_warning {
    RMCS_INITIALIZE_LOGGER("name_to_generate_logger");
};

} // namespace internal
