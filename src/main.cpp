#include "slam/slam.hpp"

#include <rclcpp/executor.hpp>
#include <rclcpp/executors.hpp>

auto executor = std::shared_ptr<rclcpp::executors::SingleThreadedExecutor>();

class LifeCircle : public rclcpp::Node {
public:
    LifeCircle()
        : Node("_rmcs_slam")
    {
        node_ = std::make_shared<SLAM>();

        reset_trigger_ = create_service<std_srvs::srv::Trigger>(
            "/rmcs_slam/reset",
            [this](
                const std::shared_ptr<std_srvs::srv::Trigger::Request>& request,
                const std::shared_ptr<std_srvs::srv::Trigger::Response>& response) {
                reset_trigger_callback(request, response);
            });
    }

    const std::shared_ptr<SLAM>& get_node()
    {
        return node_;
    }

private:
    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> reset_trigger_;
    std::shared_ptr<SLAM> node_;

    void reset_trigger_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>& request,
        const std::shared_ptr<std_srvs::srv::Trigger::Response>& response)
    {
        executor->remove_node(node_);

        node_.reset();
        node_ = std::make_shared<SLAM>();

        executor->add_node(node_);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // executor  = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    // auto node = std::make_shared<LifeCircle>();

    // executor->add_node(node);
    // executor->add_node(node->get_node());

    // executor->spin();

    rclcpp::spin(std::make_shared<SLAM>());

    rclcpp::shutdown();

    return 0;
}