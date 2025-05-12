#include "filter.hpp"
#include "node.hpp"

#include "../ros2/param.hpp"
#include <opencv2/imgproc.hpp>

namespace rmcs {

static inline void make_gradient_map(cv::Mat& origin) {
    static auto mat_gradient_x = origin.clone();
    static auto mat_gradient_y = origin.clone();

    static const auto rows_start = 0;
    static const auto rows_mid   = origin.cols / 2;
    static const auto rows_end   = origin.cols;

    static const auto cols_start = 0;
    static const auto cols_mid   = origin.rows / 2;
    static const auto cols_end   = origin.rows;

    auto left_up    = origin(cv::Range(rows_start, rows_mid), cv::Range(cols_start, cols_mid));
    auto left_down  = origin(cv::Range(rows_mid, rows_end), cv::Range(cols_start, cols_mid));
    auto right_up   = origin(cv::Range(rows_start, rows_mid), cv::Range(cols_mid, cols_end));
    auto right_down = origin(cv::Range(rows_mid, rows_end), cv::Range(cols_mid, cols_end));

    auto left_up_x =
        mat_gradient_x(cv::Range(rows_start, rows_mid), cv::Range(cols_start, cols_mid));
    auto left_down_x =
        mat_gradient_x(cv::Range(rows_mid, rows_end), cv::Range(cols_start, cols_mid));
    auto right_up_x =
        mat_gradient_x(cv::Range(rows_start, rows_mid), cv::Range(cols_mid, cols_end));
    auto right_down_x =
        mat_gradient_x(cv::Range(rows_mid, rows_end), cv::Range(cols_mid, cols_end));

    auto left_up_y =
        mat_gradient_y(cv::Range(rows_start, rows_mid), cv::Range(cols_start, cols_mid));
    auto left_down_y =
        mat_gradient_y(cv::Range(rows_mid, rows_end), cv::Range(cols_start, cols_mid));
    auto right_up_y =
        mat_gradient_y(cv::Range(rows_start, rows_mid), cv::Range(cols_mid, cols_end));
    auto right_down_y =
        mat_gradient_y(cv::Range(rows_mid, rows_end), cv::Range(cols_mid, cols_end));

    cv::Sobel(~left_up, left_up_x, CV_8U, 1, 0);
    cv::Sobel(~left_down, left_down_x, CV_8U, 1, 0);
    cv::Sobel(right_up, right_up_x, CV_8U, 1, 0);
    cv::Sobel(right_down, right_down_x, CV_8U, 1, 0);

    cv::Sobel(~left_up, left_up_y, CV_8U, 0, 1);
    cv::Sobel(~right_up, right_up_y, CV_8U, 0, 1);
    cv::Sobel(left_down, left_down_y, CV_8U, 0, 1);
    cv::Sobel(right_down, right_down_y, CV_8U, 0, 1);

    cv::addWeighted(mat_gradient_x, 0.5, mat_gradient_y, 0.5, 0, origin);
}

void filter_map(ObstacleMap& node_map) {
    static const auto gradient         = param::get<bool>("filter.gradient");
    static const auto threshold        = param::get<int>("filter.threshold");
    static const auto pre_dilate_size  = param::get<int>("filter.pre_dilate_size");
    static const auto pre_dilate_times = param::get<int>("filter.pre_dilate_times");
    static const auto pre_close_size   = param::get<int>("filter.pre_close_size");
    static const auto pre_close_times  = param::get<int>("filter.pre_close_times");
    static const auto dilate_size      = param::get<int>("filter.dilate_size");

    auto mat =
        cv::Mat(static_cast<int>(node_map.width()), static_cast<int>(node_map.width()), CV_8UC1);

    auto element = cv::Mat();

    for (auto x = 0; x < node_map.width(); x++)
        for (auto y = 0; y < node_map.width(); y++) {
            mat.at<int8_t>(x, y) = node_map(x, y).value;
        }

    // dilate
    if (pre_dilate_size != 0 && pre_dilate_times != 0) {
        element = cv::getStructuringElement(
            cv::MORPH_ELLIPSE, cv::Size(pre_dilate_size, pre_dilate_size), cv::Point(-1, -1));
        cv::erode(mat, mat, element, cv::Point(-1, -1), pre_dilate_times);
    }

    // close
    if (pre_close_size != 0 && pre_close_times != 0) {
        element = cv::getStructuringElement(
            cv::MORPH_ELLIPSE, cv::Size(pre_close_size, pre_close_size), cv::Point(-1, -1));
        cv::morphologyEx(mat, mat, cv::MORPH_OPEN, element, cv::Point(-1, -1), pre_close_times);
    }

    // gradient
    // 疑似再也不需要梯度
    // if (gradient) make_gradient_map(mat);

    // dilate
    if (dilate_size != 0) {
        element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(dilate_size, dilate_size));
        cv::erode(mat, mat, element);
    }

    // range
    // mat.forEach<uint8_t>([](uint8_t& pixel, const int* position) {
    //     pixel = static_cast<uint8_t>(pixel < threshold ? 0 : pixel);
    //     pixel = static_cast<uint8_t>(pixel > 100 ? 100 : pixel);
    // });

    for (auto x = 0; x < node_map.width(); x++)
        for (auto y = 0; y < node_map.width(); y++) {
            node_map(x, y).value = mat.at<int8_t>(x, y);
        }
}

} // namespace rmcs
