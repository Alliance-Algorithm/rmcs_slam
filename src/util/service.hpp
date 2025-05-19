#pragma once

#include "util/string.hpp"
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace rmcs::util {

#define RMCS_SERVICE_CALLBACK(...) [__VA_ARGS__](bool success, const std::string& msg)

#define TRIGGER_CALLBACK(...)                                                                      \
    [__VA_ARGS__](const std_srvs::srv::Trigger::Request::ConstSharedPtr& request,                  \
        const std_srvs::srv::Trigger::Response::SharedPtr& response)
#define SET_BOOL_CALLBACK(...)                                                                     \
    [__VA_ARGS__](const std_srvs::srv::SetBool::Request::ConstSharedPtr& request,                  \
        const std_srvs::srv::SetBool::Response::SharedPtr& response)

struct service {
    using CommonCallback = std::function<void(bool, const std::string&)>;

    /// @param node reference
    /// @param service name
    /// @param callback void(bool, std::string)
    using ServiceContext = std::tuple<rclcpp::Node&, std::string>;

    static inline void trigger(const ServiceContext& context) {
        auto& [node, service] = context;

        auto client  = node.create_client<std_srvs::srv::Trigger>(service);
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future  = client->async_send_request(request);
    };

    static inline void set_bool(const ServiceContext& context, bool data) {
        auto& [node, service] = context;

        auto client   = node.create_client<std_srvs::srv::SetBool>(service);
        auto request  = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = data;

        auto future = client->async_send_request(request);
    }

    struct rmcs_slam {
        static inline void reset(rclcpp::Node& node) {
            trigger({ node, string::slam::reset_service_name });
        }
        static inline void save_map(rclcpp::Node& node) {
            trigger({ node, string::slam::save_map_service_name });
        }
        static inline void switch_record(rclcpp::Node& node, bool data) {
            set_bool({ node, string::slam::switch_record_service_name }, data);
        }
    };
};

} // namespace rmcs::util
