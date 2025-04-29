#include "ros_util.hpp"

#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace rmcs;

struct RosUtil::Impl {

    using LidarMsg = livox_ros_driver2::msg::CustomMsg;
    using ImuMsg   = sensor_msgs::msg::Imu;

    // publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_registered_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_registered_body_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_effected_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr laser_map_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;

    // subscription
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_a_subscription;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_b_subscription;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_a_subscription;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_b_subscription;

    std::function<void(const std::unique_ptr<LidarMsg>&)> lidar_a_subscription_callback;
    std::function<void(const std::unique_ptr<LidarMsg>&)> lidar_b_subscription_callback;
    std::function<void(const std::unique_ptr<ImuMsg>&)> imu_a_subscription_callback;
    std::function<void(const std::unique_ptr<ImuMsg>&)> imu_b_subscription_callback;

    // service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_trigger;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_trigger;

    // tf
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    // timer
    rclcpp::TimerBase::SharedPtr main_process_timer;
    rclcpp::TimerBase::SharedPtr map_publisher_timer;

    struct LidarContext {
        bool enable;
        std::string topic_lidar;
        std::string topic_imu;
    } lidar_a, lidar_b;

    void initialize(rclcpp::Node& node) {

        node.get_parameter_or("lidar_a.enable", lidar_a.enable, false);
        node.get_parameter_or<std::string>("lidar_a.lidar_topic", lidar_a.topic_lidar, "");
        node.get_parameter_or<std::string>("lidar_a.imu_topic", lidar_a.topic_imu, "");

        node.get_parameter_or("lidar_b.enable", lidar_b.enable, false);
        node.get_parameter_or<std::string>("lidar_b.lidar_topic", lidar_b.topic_lidar, "");
        node.get_parameter_or<std::string>("lidar_b.imu_topic", lidar_b.topic_imu, "");

        if (!lidar_a.enable && !lidar_b.enable)
            throw std::runtime_error{"at least one lidar was enabled"};

        const auto sensor_qos = rclcpp::QoS{rclcpp::KeepLast(5)}.reliable().durability_volatile();

        if (lidar_a.enable) {
            lidar_a_subscription = node.create_subscription<LidarMsg>(
                lidar_a.topic_lidar, sensor_qos, [this](const std::unique_ptr<LidarMsg>& msg) {
                    lidar_a_subscription_callback(msg);
                });

            imu_a_subscription = node.create_subscription<ImuMsg>(
                lidar_a.topic_imu, sensor_qos,
                [this](const std::unique_ptr<ImuMsg>& msg) { imu_a_subscription_callback(msg); });
        }

        if (lidar_b.enable) {
            lidar_b_subscription = node.create_subscription<LidarMsg>(
                lidar_b.topic_lidar, sensor_qos, [](const std::unique_ptr<LidarMsg>& msg) {});
            imu_a_subscription = node.create_subscription<ImuMsg>(
                lidar_b.topic_imu, sensor_qos, [](const std::unique_ptr<ImuMsg>& msg) {});
        }
    }
};

RosUtil::RosUtil()
    : pimpl(std::make_unique<Impl>()) {}

RosUtil::~RosUtil() = default;
