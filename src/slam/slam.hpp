#pragma once

#include "util/pimpl.hpp"
#include <rclcpp/rclcpp.hpp>

namespace rmcs {

class SLAM : public rclcpp::Node {
    RMCS_PIMPL_DEFINTION(SLAM)
};

} // namespace rmcs
