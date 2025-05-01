#include "ros_util.hpp"

#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace rmcs;

struct RosUtil::Impl {

    using LidarMsg = livox_ros_driver2::msg::CustomMsg;
    using ImuMsg   = sensor_msgs::msg::Imu;

    // publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_registered_world_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_registered_body_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_effected_world_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr constructed_map_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;
    rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr transfromed_livox_publisher;

    // service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_trigger;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_trigger;

    // tf
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    // timer
    rclcpp::TimerBase::SharedPtr main_process_timer;
    rclcpp::TimerBase::SharedPtr map_publisher_timer;

    void initialize(rclcpp::Node& node) {

        const auto sensor_qos = rclcpp::QoS{rclcpp::KeepLast(5)}.reliable().durability_volatile();

        cloud_registered_world_publisher = node.create_publisher<sensor_msgs::msg::PointCloud2>(
            topic::pointcloud_registerd_world, sensor_qos);

        cloud_registered_body_publisher = node.create_publisher<sensor_msgs::msg::PointCloud2>(
            topic::pointcloud_registerd_body, sensor_qos);

        cloud_effected_world_publisher = node.create_publisher<sensor_msgs::msg::PointCloud2>(
            topic::pointcloud_effected_world, sensor_qos);

        constructed_map_publisher = node.create_publisher<sensor_msgs::msg::PointCloud2>(
            topic::constructed_map, sensor_qos);

        pose_publisher =
            node.create_publisher<geometry_msgs::msg::PoseStamped>(topic::pose, sensor_qos);

        odometry_publisher =
            node.create_publisher<nav_msgs::msg::Odometry>(topic::odometry, sensor_qos);

        path_publisher = node.create_publisher<nav_msgs::msg::Path>(topic::path, sensor_qos);

        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);
    }
};

RosUtil::RosUtil()
    : pimpl(std::make_unique<Impl>()) {}

RosUtil::~RosUtil() = default;

void RosUtil::initialize(rclcpp::Node& node) { //
    pimpl->initialize(node);
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
