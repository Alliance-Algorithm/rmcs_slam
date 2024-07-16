#include "slam/slam.hpp"
#include <rclcpp/executors.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<SLAM>());

    rclcpp::shutdown();

    return 0;
}