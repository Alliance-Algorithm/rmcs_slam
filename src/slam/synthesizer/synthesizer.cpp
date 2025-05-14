// for rclcpp_info on initialize function
#pragma clang diagnostic ignored "-Wformat-security"

#include "synthesizer.hpp"
#include "util/convert.hpp"
#include "util/logger.hpp"
#include "util/parameter.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rosbag2_cpp/writer.hpp>
#include <sensor_msgs/msg/point_cloud2.h>

#include <queue>

using namespace rmcs;

struct Synthesizer::Impl {
public:
    using Point      = pcl::PointXYZ;
    using PointCloud = pcl::PointCloud<Point>;

    void initialize(rclcpp::Node& node) {
        auto p = util::quick_paramtetr_reader(node);

        enable_primary   = p("primary_lidar.enable", bool {});
        enable_secondary = p("secondary_lidar.enable", bool {});
        rclcpp_info("primary: %d, secondary: %d", enable_primary, enable_secondary);

        if (!enable_primary && !enable_secondary)
            throw util::runtime_error("at least enable one lidar");

        const auto initialize_context = [&](LidarContext& context, const std::string& index) {
            const auto lidar_topic = p(index + "_lidar.lidar_topic", std::string { "" });
            const auto imu_topic   = p(index + "_lidar.imu_topic", std::string { "" });
            const auto x           = p(index + "_lidar.x", double { 0 });
            const auto y           = p(index + "_lidar.y", double { 0 });
            const auto z           = p(index + "_lidar.z", double { 0 });
            const auto yaw         = p(index + "_lidar.yaw", double { 0 });
            const auto pitch       = p(index + "_lidar.pitch", double { 0 });
            const auto roll        = p(index + "_lidar.roll", double { 0 });

            const auto translation = Eigen::Translation3d { x, y, z };
            const auto orientation =
                Eigen::Quaterniond { Eigen::AngleAxisd(
                                         yaw / 180 * std::numbers::pi, Eigen::Vector3d::UnitZ())
                    * Eigen::AngleAxisd(pitch / 180 * std::numbers::pi, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(roll / 180 * std::numbers::pi, Eigen::Vector3d::UnitX()) }
                    .normalized();

            context.initialize(node, lidar_topic, imu_topic, orientation, translation);
            rclcpp_info("-------------------  %s ---------------------", index.c_str());
            rclcpp_info("%s context with t[%4.2f %4.2f %4.2f], q[%4.2f %4.2f %4.2f %4.2f]",
                index.c_str(), translation.x(), translation.y(), translation.z(), orientation.w(),
                orientation.x(), orientation.y(), orientation.z());
            rclcpp_info("yaw: %5.2f, pitch: %5.2f, roll: %5.2f", yaw, pitch, roll);
            rclcpp_info("lidar topic: %s", lidar_topic.c_str());
            rclcpp_info("imu   topic: %s", imu_topic.c_str());
        };

        if (enable_primary) initialize_context(primary_context, "primary");
        if (enable_secondary) initialize_context(secondary_context, "secondary");
    }

    void register_callback(const auto& update_lidar, const auto& update_imu) {
        if (enable_primary && enable_secondary)
            register_callback_with_combination(update_lidar, update_imu);
        else {
            primary_context.register_callback(update_lidar, update_imu);
            secondary_context.register_callback(update_lidar, update_imu);
        }
    }

private:
    RMCS_INITIALIZE_LOGGER("rmcs-slam");

    /// @brief 单个雷达相关ROS2接口的包装
    struct LidarContext {
        std::shared_ptr<rclcpp::Subscription<LivoxMsg>> pointcloud_subscription;
        std::shared_ptr<rclcpp::Subscription<ImuMsg>> imu_subscription;

        std::function<void(const std::unique_ptr<LivoxMsg>&)> livox_message_callback;
        std::function<void(const std::unique_ptr<ImuMsg>&)> imu_message_callback;

        Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();

        Eigen::Quaterniond rotation;
        Eigen::Translation3d translation;

        void initialize(rclcpp::Node& node, const std::string& lidar_topic,
            const std::string& imu_topic, const Eigen::Quaterniond& rotation,
            const Eigen::Translation3d& translation) {

            this->rotation    = rotation;
            this->translation = translation;

            livox_message_callback = [](const auto&) { };
            imu_message_callback   = [](const auto&) { };

            const auto qos = rclcpp::QoS { rclcpp::KeepLast(5) }.reliable().durability_volatile();

            this->pointcloud_subscription = node.create_subscription<LivoxMsg>(lidar_topic, qos,
                [this](const std::unique_ptr<LivoxMsg>& msg) { _internal_update_pointcloud(msg); });

            this->imu_subscription = node.create_subscription<ImuMsg>(imu_topic, qos,
                [this](const std::unique_ptr<ImuMsg>& msg) { _internal_update_imu(msg); });
        }

        void register_callback(const auto& livox_callback, const auto& imu_callback) {
            livox_message_callback = livox_callback;
            imu_message_callback   = imu_callback;
        }

        void update_transform(const Eigen::Isometry3f& t) { transform = t; }

    private:
        /// @brief 更新一帧点云消息，实际上就是进行了从雷达系到车辆正方向的变换
        void _internal_update_pointcloud(const std::unique_ptr<LivoxMsg>& msg) const {
            auto& source_pointcloud = msg->points;
            const auto transform    = translation * rotation;

#pragma omp parallel for default(none) shared(source_pointcloud) shared(transform)                 \
    num_threads(MP_PROC_NUM)

            for (auto& point : source_pointcloud) {
                const auto transformed =
                    Eigen::Vector3d { transform * Eigen::Vector3d { point.x, point.y, point.z } };
                point.x = static_cast<float>(transformed.x());
                point.y = static_cast<float>(transformed.y());
                point.z = static_cast<float>(transformed.z());
            }

            livox_message_callback(msg);
        }

        void _internal_update_imu(const std::unique_ptr<ImuMsg>& msg) {
            auto angular_velocity = Eigen::Vector3d {
                msg->angular_velocity.x,
                msg->angular_velocity.y,
                msg->angular_velocity.z,
            };
            angular_velocity = rotation * angular_velocity;
            util::convert_translation(angular_velocity, msg->angular_velocity);

            auto linear_acceleration = Eigen::Vector3d {
                msg->linear_acceleration.x,
                msg->linear_acceleration.y,
                msg->linear_acceleration.z,
            };
            linear_acceleration = rotation * linear_acceleration;
            util::convert_translation(linear_acceleration, msg->linear_acceleration);

            auto orientation = Eigen::Quaterniond {
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

    template <typename Msg> struct StampedMsg : public Msg {
        bool operator<(const StampedMsg& o) const {
            return (rclcpp::Time { this->header.stamp } > rclcpp::Time { o.header.stamp });
        }
    };

    bool enable_primary { false };
    bool enable_secondary { false };

    // 此方略若使用两个雷达，会造成畸变纠正的劣化
    std::array<LivoxMsg, 2> secondary_buffer;
    std::atomic<bool> secondary_write_first_buffer;
    void register_callback_with_combination(const auto& update_lidar, const auto& update_imu) {
        if (!enable_secondary) {
            primary_context.register_callback(update_lidar, update_imu);
            return;
        }

        rclcpp_info("启用辅助雷达");

        const auto primary_livox_callback = [=, this](const std::unique_ptr<LivoxMsg>& msg) {
            // 次级雷达会时刻同步点云至缓存，选取一个Free的点云进行合并
            std::size_t buffer_index =
                secondary_write_first_buffer.load(std::memory_order::relaxed) ? 1 : 0;
            auto& secondary_msg = secondary_buffer[buffer_index];
            for (const auto point : secondary_msg.points)
                msg->points.push_back(point);
            msg->point_num = msg->points.size();
            update_lidar(msg);
        };
        const auto secondary_livox_callback = [this](const std::unique_ptr<LivoxMsg>& msg) {
            auto read_first        = secondary_write_first_buffer.load(std::memory_order::relaxed);
            std::size_t read_index = read_first ? 0 : 1;
            secondary_buffer[read_index] = *msg;
            secondary_write_first_buffer.store(!read_first, std::memory_order::relaxed);
        };
        primary_context.register_callback(primary_livox_callback, update_imu);
        secondary_context.register_callback(secondary_livox_callback, [](const auto&) { });
    }

    // 雷达时间戳顺序更新
    std::priority_queue<StampedMsg<LivoxMsg>> lidar_queue;
    std::priority_queue<StampedMsg<ImuMsg>> imu_queue;

    std::size_t lidar_queue_buffer_size = 2;
    std::size_t imu_queue_buffer_size   = 2;

    void register_callback_without_combination(const auto& update_lidar, const auto& update_imu) {
        const auto lidar_callback = [this, update_lidar](const std::unique_ptr<LivoxMsg>& msg) {
            lidar_queue.emplace(StampedMsg<LivoxMsg> { *msg });
            while (lidar_queue.size() > lidar_queue_buffer_size) {
                const auto& old_msg = lidar_queue.top();
                update_lidar(std::make_unique<LivoxMsg>(old_msg));
                lidar_queue.pop();
            }
        };
        const auto imu_callback = [this, update_imu](const std::unique_ptr<ImuMsg>& msg) {
            imu_queue.emplace(StampedMsg<ImuMsg> { *msg });
            while (imu_queue.size() > imu_queue_buffer_size) {
                const auto& old_msg = imu_queue.top();
                update_imu(std::make_unique<ImuMsg>(old_msg));
                imu_queue.pop();
            }
        };

        primary_context.register_callback(lidar_callback, imu_callback);
        secondary_context.register_callback(lidar_callback, imu_callback);
    }
};

Synthesizer::Synthesizer()
    : pimpl(std::make_unique<Impl>()) { }

Synthesizer::~Synthesizer() = default;

void Synthesizer::initialize(rclcpp::Node& node) { //
    pimpl->initialize(node);
}

void Synthesizer::register_callback(
    const std::function<void(const std::unique_ptr<LivoxMsg>&)>& livox_callback,
    const std::function<void(const std::unique_ptr<ImuMsg>&)>& imu_callback) {
    pimpl->register_callback(livox_callback, imu_callback);
}

void Synthesizer::switch_record(bool on) { }
