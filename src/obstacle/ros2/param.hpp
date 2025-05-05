#pragma once

#include "util/parameter.hpp"

#include <rclcpp/node.hpp>

namespace param {
inline auto node = std::shared_ptr<rclcpp::Node>();

template <typename T>
inline auto get(const std::string& name) {
    // node for lazy constructing
    if (node == nullptr)
        node = std::make_shared<rclcpp::Node>("rmcs_map", rmcs::util::NodeOptions{});

    auto param = T{};
    node->get_parameter<T>(name, param);
    return param;
}

} // namespace param
