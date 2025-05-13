#include <fast_gicp/gicp/fast_gicp.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>

int main(int argc, char* argv[]) {
    using Point  = pcl::PointXYZ;
    using Engine = fast_gicp::FastGICP<Point, Point>;

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("rmcs_calibration");

    auto engine = Engine {};

    rclcpp::shutdown();
}
