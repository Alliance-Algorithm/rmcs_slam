#pragma once

#include "util/parameter.hpp"
#include <rclcpp/node.hpp>

namespace rmcs::util {

std::shared_ptr<rclcpp::Node> make_simple_node(
    const std::string& name, const rclcpp::NodeOptions& options = util::NodeOptions {}) {
    return std::make_shared<rclcpp::Node>(name, options);
}

} // namespace rmcs::util
