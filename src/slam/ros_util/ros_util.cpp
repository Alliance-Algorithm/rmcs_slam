#include "ros_util.hpp"
#include "util/service.hpp"
#include "util/string.hpp"

#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace rmcs;

struct RosUtil::Impl {

    using LidarMsg = livox_ros_driver2::msg::CustomMsg;
    using ImuMsg   = sensor_msgs::msg::Imu;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_registered_world_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_registered_body_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_effected_world_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr constructed_map_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;
    rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr transfromed_livox_publisher;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_trigger;
    std::function<void(void)> map_save_function;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_trigger;
    std::function<void(void)> reset_function;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr record_switch_service;
    std::function<void(bool)> record_switch_callback;

    rclcpp::TimerBase::SharedPtr update_timer;
    std::function<void(void)> update_function;

    rclcpp::TimerBase::SharedPtr publish_map_timer;
    std::function<void(void)> publish_map_function;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    void initialize(rclcpp::Node& node) {

        const auto sensor_qos =
            rclcpp::QoS { rclcpp::KeepLast(5) }.reliable().durability_volatile();

        cloud_registered_world_publisher = node.create_publisher<sensor_msgs::msg::PointCloud2>(
            string::slam::cloud_world_topic, sensor_qos);

        cloud_registered_body_publisher = node.create_publisher<sensor_msgs::msg::PointCloud2>(
            string::slam::cloud_body_topic, sensor_qos);

        cloud_effected_world_publisher = node.create_publisher<sensor_msgs::msg::PointCloud2>(
            string::slam::cloud_effected_world_topic, sensor_qos);

        constructed_map_publisher = node.create_publisher<sensor_msgs::msg::PointCloud2>(
            string::slam::constructed_map_topic, sensor_qos);

        pose_publisher = node.create_publisher<geometry_msgs::msg::PoseStamped>(
            string::slam::pose_topic, sensor_qos);

        odometry_publisher = node.create_publisher<nav_msgs::msg::Odometry>( //
            string::slam::odometry_topic, sensor_qos);

        path_publisher = node.create_publisher<nav_msgs::msg::Path>( //
            string::slam::path_topic, sensor_qos);

        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);

        using namespace std::chrono_literals;
        update_timer      = node.create_wall_timer(1ms, [this] { update_function(); });
        publish_map_timer = node.create_wall_timer(1s, [this] { publish_map_function(); });

        map_save_trigger = node.create_service<std_srvs::srv::Trigger>(
            string::slam::save_map_service_name, TRIGGER_CALLBACK(this) {
                map_save_function();
                response->success = true;
                response->message = "ros_util handled";
            });
        reset_trigger = node.create_service<std_srvs::srv::Trigger>(
            string::slam::reset_service_name, TRIGGER_CALLBACK(this) {
                reset_function();
                response->success = true;
                response->message = "ros_util handled";
            });

        using SwitchRequest   = std_srvs::srv::SetBool::Request::SharedPtr;
        using SwitchResponse  = std_srvs::srv::SetBool::Response::SharedPtr;
        record_switch_service = node.create_service<std_srvs::srv::SetBool>(
            string::slam::switch_record_service_name, SET_BOOL_CALLBACK(this) {
                record_switch_callback(request->data);
                response->success = true;
                response->message = "ros_util handled";
            });
    }
};

RosUtil::RosUtil()
    : pimpl(std::make_unique<Impl>()) { }

RosUtil::~RosUtil() = default;

void RosUtil::initialize(rclcpp::Node& node) { //
    pimpl->initialize(node);
}

void RosUtil::stop_service() {
    pimpl->update_timer->cancel();
    pimpl->publish_map_timer->cancel();
}

void RosUtil::start_service() {
    pimpl->update_timer->reset();
    pimpl->publish_map_timer->reset();
}

void RosUtil::publish_pointcloud_registerd_world(const sensor_msgs::msg::PointCloud2& msg) const {
    pimpl->cloud_registered_world_publisher->publish(msg);
}

void RosUtil::publish_pointcloud_registerd_body(const sensor_msgs::msg::PointCloud2& msg) const {
    pimpl->cloud_registered_body_publisher->publish(msg);
}

void RosUtil::publish_pointcloud_effect_world(const sensor_msgs::msg::PointCloud2& msg) const {
    pimpl->cloud_effected_world_publisher->publish(msg);
}

void RosUtil::publish_constructed_map(const sensor_msgs::msg::PointCloud2& msg) const {
    pimpl->constructed_map_publisher->publish(msg);
}

void RosUtil::publish_pose(const geometry_msgs::msg::PoseStamped& msg) const {
    pimpl->pose_publisher->publish(msg);
}

void RosUtil::publish_odometry(const nav_msgs::msg::Odometry& msg) const {
    pimpl->odometry_publisher->publish(msg);
}

void RosUtil::publish_path(const nav_msgs::msg::Path& msg) const {
    pimpl->path_publisher->publish(msg);
}

void RosUtil::update_transform(const geometry_msgs::msg::TransformStamped& stamp) const {
    pimpl->tf_broadcaster->sendTransform(stamp);
}

void RosUtil::register_map_save_function(const std::function<void(void)>& fun) {
    pimpl->map_save_function = fun;
}
void RosUtil::register_reset_function(const std::function<void(void)>& fun) {
    pimpl->reset_function = fun;
}
void RosUtil::register_update_function(const std::function<void(void)>& fun) {
    pimpl->update_function = fun;
}
void RosUtil::register_publish_map_function(const std::function<void(void)>& fun) {
    pimpl->publish_map_function = fun;
}
void RosUtil::register_switch_record_function(const std::function<void(bool)>& fun) {
    pimpl->record_switch_callback = fun;
}
