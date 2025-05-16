#include "ros2/node.hpp"
#include <rclcpp/executors.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<Node>());

    rclcpp::shutdown();

    return 0;
}
