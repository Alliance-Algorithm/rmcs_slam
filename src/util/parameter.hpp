#pragma once

#include <rclcpp/node.hpp>

namespace rmcs::util {

struct NodeOptions : rclcpp::NodeOptions {
    explicit NodeOptions() {
        automatically_declare_parameters_from_overrides(true);
        allow_undeclared_parameters(true);
    }
};

struct quick_paramtetr_reader {
public:
    explicit quick_paramtetr_reader(rclcpp::Node& node)
        : node(node) {
        auto vaild = node.get_node_options().automatically_declare_parameters_from_overrides()
            && node.get_node_options().allow_undeclared_parameters();
        if (vaild == false)
            throw std::runtime_error { "请检查 NodeOptions 配置，无法自动定义参数" };
    }

    /// @usage auto v = foo("name", int{});
    /// @note 配置若是未生效，检查一下 NodeOptions 的配置，推荐使用
    ///       rmcs::util::NodeOptions 哦
    template <typename T>
    T operator()(const std::string& name, T just_send_a_type_using_default_constructor) const {
        (void)just_send_a_type_using_default_constructor;
        return get_parameter<T>(name);
    }

    template <typename T> T get_parameter(const std::string& name) const {
        auto v = T {};
        node.get_parameter(name, v);
        return v;
    }

private:
    rclcpp::Node& node;
};

} // namespace rmcs::util
