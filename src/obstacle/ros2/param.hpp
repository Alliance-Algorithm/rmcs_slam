#pragma once

#include <rclcpp/node.hpp>

namespace param {
inline auto node = std::shared_ptr<rclcpp::Node>();

template <typename T>
inline auto get(const std::string& name)
{
    // node for lazy constructing
    if (node == nullptr)
        node = std::make_shared<rclcpp::Node>(
            "param_server",
            rclcpp::NodeOptions()
                .automatically_declare_parameters_from_overrides(true));

    auto param = T {};
    node->get_parameter<T>(name, param);
    return param;
}

} // namespace param