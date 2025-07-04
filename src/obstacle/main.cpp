#include "ros2/node.hpp"
#include <rclcpp/executors.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<RmcsMapRuntime>();
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
