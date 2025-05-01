// for rclcpp_info on initialize function
#pragma clang diagnostic ignored "-Wformat-security"

#include "synthesizer.hpp"
#include "util/convert.hpp"

using namespace rmcs;

struct Synthesizer::Impl {
public:
    /// @brief 单个雷达相关ROS2接口的包装
    struct LidarContext {
        std::shared_ptr<rclcpp::Subscription<LivoxMsg>> pointcloud_subscription;
        std::shared_ptr<rclcpp::Subscription<ImuMsg>> imu_subscription;

        std::function<void(const std::unique_ptr<LivoxMsg>&)> livox_message_callback;
        std::function<void(const std::unique_ptr<ImuMsg>&)> imu_message_callback;

        Eigen::Quaterniond rotation;
        Eigen::Translation3d translation;

        void initialize(
            rclcpp::Node& node, const std::string& lidar_topic, const std::string& imu_topic,
            const Eigen::Quaterniond& rotation, const Eigen::Translation3d& translation) {

            this->rotation    = rotation;
            this->translation = translation;

            livox_message_callback = [](const auto&) {};
            imu_message_callback   = [](const auto&) {};

            const auto qos = rclcpp::QoS{rclcpp::KeepLast(5)}.reliable().durability_volatile();

            this->pointcloud_subscription = node.create_subscription<LivoxMsg>(
                lidar_topic, qos,
                [this](const std::unique_ptr<LivoxMsg>& msg) { _internal_update_pointcloud(msg); });

            this->imu_subscription = node.create_subscription<ImuMsg>(
                imu_topic, qos,
                [this](const std::unique_ptr<ImuMsg>& msg) { _internal_update_imu(msg); });
        }

        void register_callback(const auto& livox_callback, const auto& imu_callback) {
            livox_message_callback = livox_callback;
            imu_message_callback   = imu_callback;
        }

    private:
        /// @brief 更新一帧点云消息，实际上就是进行了从雷达系到车辆正方向的变换
        void _internal_update_pointcloud(const std::unique_ptr<LivoxMsg>& msg) const {
            auto& source_pointcloud = msg->points;
            const auto transform    = translation * rotation;

#pragma omp parallel for default(none) shared(source_pointcloud) shared(transform) \
    num_threads(MP_PROC_NUM)

            for (auto& point : source_pointcloud) {
                const auto transformed = Eigen::Vector3d{
                    transform * Eigen::Vector3d{point.x, point.y, point.z}
                };
                point.x = static_cast<float>(transformed.x());
                point.y = static_cast<float>(transformed.y());
                point.z = static_cast<float>(transformed.z());
            }

            livox_message_callback(msg);
        }

        void _internal_update_imu(const std::unique_ptr<ImuMsg>& msg) {
            auto angular_velocity = Eigen::Vector3d{
                msg->angular_velocity.x,
                msg->angular_velocity.y,
                msg->angular_velocity.z,
            };
            angular_velocity = rotation * angular_velocity;
            util::convert_vector3(angular_velocity, msg->angular_velocity);

            auto linear_acceleration = Eigen::Vector3d{
                msg->linear_acceleration.x,
                msg->linear_acceleration.y,
                msg->linear_acceleration.z,
            };
            linear_acceleration = rotation * linear_acceleration;
            util::convert_vector3(linear_acceleration, msg->linear_acceleration);

            auto orientation = Eigen::Quaterniond{
                msg->orientation.w,
                msg->orientation.x,
                msg->orientation.y,
                msg->orientation.z,
            };
            orientation = rotation * orientation;
            util::convert_orientation(orientation, msg->orientation);

            imu_message_callback(msg);
        }

    } primary_context, secondary_context;

public:
    /// @brief ros2 wrapper initialize
    void initialize(rclcpp::Node& node) {

        const auto rclcpp_info = [&node](const char* format, auto&&... args) {
            RCLCPP_INFO(node.get_logger(), format, std::forward<decltype(args)>(args)...);
        };

        const auto p = [&node](const auto& _name, const auto& _default) -> auto {
            return node.get_parameter_or<std::remove_cvref_t<decltype(_default)>>(_name, _default);
        };

        const auto primary_enbale   = p("primary_lidar.enable", false);
        const auto secondary_enbale = p("secondary_lidar.enable", false);
        if (!primary_enbale && !secondary_enbale)
            throw std::runtime_error{"at least one lidiar was enabled"};

        rclcpp_info("primary: %d, secondary: %d", primary_enbale, secondary_enbale);

        double x, y, z, yaw, pitch, roll;
        if (primary_enbale) {
            const auto lidar_topic = p("primary_lidar.lidar_topic", std::string{""});
            const auto imu_topic   = p("primary_lidar.imu_topic", std::string{""});
            const auto lidar_x     = p("primary_lidar.x", double{0});
            const auto lidar_y     = p("primary_lidar.y", double{0});
            const auto lidar_z     = p("primary_lidar.z", double{0});
            const auto lidar_yaw   = p("primary_lidar.yaw", double{0});
            const auto lidar_pitch = p("primary_lidar.pitch", double{0});
            const auto lidar_roll  = p("primary_lidar.roll", double{0});

            std::tie(x, y, z)          = std::tuple{lidar_x, lidar_yaw, lidar_z};
            std::tie(yaw, pitch, roll) = std::tuple{lidar_yaw, lidar_pitch, lidar_roll};

            const auto translation = Eigen::Translation3d{x, y, z};
            const auto rotation    = Eigen::Quaterniond{
                Eigen::AngleAxisd(yaw / 180 * std::numbers::pi, Eigen::Vector3d::UnitZ())
                * Eigen::AngleAxisd(pitch / 180 * std::numbers::pi, Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(roll / 180 * std::numbers::pi, Eigen::Vector3d::UnitX())};

            primary_context.initialize(node, lidar_topic, imu_topic, rotation, translation);

            rclcpp_info("primary lidar was initialized");
            rclcpp_info("lidar topic: %s", lidar_topic.c_str());
            rclcpp_info("imu   topic: %s", imu_topic.c_str());
            rclcpp_info("x: %5.3f, y: %5.3f, z: %5.3f", x, y, z);
            rclcpp_info("yaw: %5.2f, pitch: %5.2f, roll: %5.2f", yaw, pitch, roll);
        }

        if (secondary_enbale) {
            const auto lidar_topic = p("secondary_lidar.lidar_topic", std::string{""});
            const auto imu_topic   = p("secondary_lidar.imu_topic", std::string{""});
            const auto lidar_x     = p("secondary_lidar.x", double{0});
            const auto lidar_y     = p("secondary_lidar.y", double{0});
            const auto lidar_z     = p("secondary_lidar.z", double{0});
            const auto lidar_yaw   = p("secondary_lidar.yaw", double{0});
            const auto lidar_pitch = p("secondary_lidar.pitch", double{0});
            const auto lidar_roll  = p("secondary_lidar.roll", double{0});

            std::tie(x, y, z)          = std::tuple{lidar_x, lidar_y, lidar_z};
            std::tie(yaw, pitch, roll) = std::tuple{lidar_yaw, lidar_pitch, lidar_roll};

            const auto translation = Eigen::Translation3d{x, y, z};
            const auto rotation    = Eigen::Quaterniond{
                Eigen::AngleAxisd(yaw / 180 * std::numbers::pi, Eigen::Vector3d::UnitZ())
                * Eigen::AngleAxisd(pitch / 180 * std::numbers::pi, Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(roll / 180 * std::numbers::pi, Eigen::Vector3d::UnitX())};

            secondary_context.initialize(node, lidar_topic, imu_topic, rotation, translation);

            rclcpp_info("secondary lidar was initialized");
            rclcpp_info("lidar topic: %s", lidar_topic.c_str());
            rclcpp_info("imu   topic: %s", imu_topic.c_str());
            rclcpp_info("x: %5.3f, y: %5.3f, z: %5.3f", x, y, z);
            rclcpp_info("yaw: %5.2f, pitch: %5.2f, roll: %5.2f", yaw, pitch, roll);
        }
    }

    void register_callback(const auto& livox_callback, const auto& imu_callback) {
        primary_context.register_callback(livox_callback, imu_callback);
        secondary_context.register_callback(livox_callback, imu_callback);
    }
};

Synthesizer::Synthesizer()
    : pimpl(std::make_unique<Impl>()) {}

Synthesizer::~Synthesizer() = default;

void Synthesizer::initialize(rclcpp::Node& node) { //
    pimpl->initialize(node);
}

void Synthesizer::register_callback(
    const std::function<void(const std::unique_ptr<LivoxMsg>&)>& livox_callback,
    const std::function<void(const std::unique_ptr<ImuMsg>&)>& imu_callback) {
    pimpl->register_callback(livox_callback, imu_callback);
}
