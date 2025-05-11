#pragma once

#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace rmcs::util {

#define TRIGGER_CALLBACK                                                \
    [this](                                                             \
        const std_srvs::srv::Trigger::Request::ConstSharedPtr& request, \
        const std_srvs::srv::Trigger::Response::SharedPtr& response)

struct service {
    using TriggerCallback = std::function<void(bool, std::string)>;
    static inline void
        trigger(rclcpp::Node& node, const std::string& service, const TriggerCallback& callback) {
        auto client  = node.create_client<std_srvs::srv::Trigger>(service);
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future  = client->async_send_request(request);

        if (rclcpp::spin_until_future_complete(node.get_node_base_interface(), future)
            == rclcpp::FutureReturnCode::SUCCESS) {
            const auto result = future.get();
            callback(result->success, result->message);
        } else {
            callback(false, "Service " + service + " trigger timeout");
        }
    };
    struct rmcs_slam {
        static inline void reset(rclcpp::Node& node, const TriggerCallback& callback) {
            trigger(node, "/rmcs_slam/reset", callback);
        }
        static inline void save_map(rclcpp::Node& node, const TriggerCallback& callback) {
            trigger(node, "/rmcs_slam/map_save", callback);
        }
    };
};

} // namespace rmcs::util
