#include "synthesizer.hpp"

using namespace rmcs;

struct Synthesizer::Impl {

    std::shared_ptr<rclcpp::Subscription<LivoxMsg>> pointcloud_a_subscription;
    std::shared_ptr<rclcpp::Subscription<LivoxMsg>> pointcloud_b_subscription;

    std::shared_ptr<rclcpp::Subscription<ImuMsg>> imu_a_subscription;
    std::shared_ptr<rclcpp::Subscription<ImuMsg>> imu_b_subscription;

    std::shared_ptr<rclcpp::Publisher<LivoxMsg>> pointcloud_publisher;
    std::shared_ptr<rclcpp::Publisher<ImuMsg>> imu_publisher;

    Eigen::Quaterniond main_rotation{Eigen::Quaterniond::Identity()};
    Eigen::Translation3d main_translation{Eigen::Translation3d::Identity()};

    Eigen::Quaterniond auxiliary_rotation{Eigen::Quaterniond::Identity()};
    Eigen::Translation3d auxiliary_translation{Eigen::Translation3d::Identity()};

    void upload_pointcloud(
        const std::unique_ptr<LivoxMsg>& msg, const Eigen::Isometry3d& transform) {

        auto& points = msg->points;

#pragma omp parallel for default(none) shared(points) shared(transform) num_threads(MP_PROC_NUM)
        for (auto& point : points) {
            const auto transformed = Eigen::Vector3d{
                transform * Eigen::Vector3d{point.x, point.y, point.z}
            };
            point.x = static_cast<float>(transformed.x());
            point.y = static_cast<float>(transformed.y());
            point.z = static_cast<float>(transformed.z());
        }

        pointcloud_publisher->publish(*msg);
    }

    void update_imu(const std::unique_ptr<ImuMsg>& msg, const Eigen::Quaterniond& rotation) {
    }

    void upload_main_pointcloud(const std::unique_ptr<LivoxMsg>& msg) {
        upload_pointcloud(msg, main_translation * main_rotation);
    }

    void upload_main_imu(const std::unique_ptr<ImuMsg>& msg) {}

    void upload_auxiliary_pointcloud(const std::unique_ptr<LivoxMsg>& msg) {
        upload_pointcloud(msg, auxiliary_translation * auxiliary_rotation);
    }

    void upload_auxiliary_imu(const std::unique_ptr<ImuMsg>& msg) {}
};

Synthesizer::Synthesizer()
    : pimpl(std::make_unique<Impl>()) {}

Synthesizer::~Synthesizer() = default;

void Synthesizer::initialize(rclcpp::Node& node) {
    const auto lidar_a_topic = node.get_parameter_or<std::string>("lidar_a.lidar_topic", "");
    const auto lidar_b_topic = node.get_parameter_or<std::string>("lidar_b.lidar_topic", "");
    const auto imu_a_topic   = node.get_parameter_or<std::string>("lidar_a.imu_topic", "");
    const auto imu_b_topic   = node.get_parameter_or<std::string>("lidar_b.imu_topic", "");

    pimpl->pointcloud_a_subscription = node.create_subscription<LivoxMsg>(
        lidar_a_topic, 10, [](const std::unique_ptr<LivoxMsg>& msg) {});

    pimpl->pointcloud_b_subscription = node.create_subscription<LivoxMsg>(
        lidar_b_topic, 10, [](const std::unique_ptr<LivoxMsg>& msg) {});

    pimpl->imu_a_subscription = node.create_subscription<ImuMsg>(
        imu_a_topic, 10, [](const std::unique_ptr<ImuMsg>& msg) {});

    pimpl->imu_b_subscription = node.create_subscription<ImuMsg>(
        imu_b_topic, 10, [](const std::unique_ptr<ImuMsg>& msg) {});

    pimpl->pointcloud_publisher = node.create_publisher<LivoxMsg>("/livox/lidar", 10);
    pimpl->imu_publisher        = node.create_publisher<ImuMsg>("/livox/imu", 10);
}

void Synthesizer::set_main_transform(
    const Eigen::Quaterniond& rotation, const Eigen::Translation3d& translation) {
    pimpl->main_rotation    = rotation;
    pimpl->main_translation = translation;
}

void Synthesizer::set_auxiliary_transform(
    const Eigen::Quaterniond& rotation, const Eigen::Translation3d& translation) {
    pimpl->auxiliary_rotation    = rotation;
    pimpl->auxiliary_translation = translation;
}
