#pragma once
#include "util/pimpl.hpp"
#include <rclcpp/node.hpp>

class RmcsMapRuntime : public rclcpp::Node {
    RMCS_PIMPL_DEFINTION(RmcsMapRuntime);
};
