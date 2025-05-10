#include "runtime.hpp"
#include "registration/engine.hpp"
#include "util/convert.hpp"
#include "util/logger.hpp"
#include "util/parameter.hpp"
#include "util/service.hpp"

#include <cassert>
#include <functional>
#include <memory>
#include <unordered_map>
#include <vector>

#include <Eigen/Eigen>
#include <pcl/io/pcd_io.h>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace rmcs;

struct Runtime::Impl {
    RMCS_INITIALIZE_LOGGER("rmcs-location");

    Registration registration{};

    enum class Status {
        NONE_ACTION,
        PREPARATION,
        RUNTIME,
        LOST_LOCATION,
        MISSION_SIZE,
    } runtime_status;

    using Misson = std::function<void()>;
    std::unordered_map<Status, Misson> missons;

    bool enable_record;
    bool enable_initize;
    bool enable_relocalize;

    bool map_availiable;

    // 位姿处理相关资源
    Eigen::Isometry3d initial_solution;
    Eigen::Isometry3d initial_pose;

    Eigen::Vector3d position_minimum;
    Eigen::Vector3d position_maximum;

    std::vector<Eigen::Isometry3d> pose_buffer;

    std::shared_ptr<PointCloud> standard_map{std::make_shared<PointCloud>()};
    std::shared_ptr<PointCloud> scanning_frame{std::make_shared<PointCloud>()};

    // ROS2 相关接口
    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>> odometer_subscription;
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> localization_publisher;

    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> initialize_service;

    std::shared_ptr<rclcpp::TimerBase> runtime_timer;

    void initialize(rclcpp::Node& node) {
        registration.initialize(node);

        const auto message =
            util::colored(util::ansi::kForegroundGreen, "rmcs-location runtime initializing now");
        rclcpp_info(message.c_str());

        const auto p = util::quick_paramtetr_reader{node};

        auto initial_translation = Eigen::Translation3d{
            p("initial_pose.translation.x", double{}),
            p("initial_pose.translation.y", double{}),
            p("initial_pose.translation.z", double{}),
        };
        auto initial_orientation =
            Eigen::Quaterniond{
                p("initial_pose.orientation.w", double{}),
                p("initial_pose.orientation.x", double{}),
                p("initial_pose.orientation.y", double{}),
                p("initial_pose.orientation.z", double{})}
                .normalized();
        rclcpp_info(
            "initial pose: t( %4.2f %4.2f %4.2f ) q( %4.2f %4.2f %4.2f %4.2f )",
            initial_translation.x(), initial_translation.y(), initial_translation.z(),
            initial_orientation.w(), initial_orientation.x(), initial_orientation.y(),
            initial_orientation.z());

        initial_pose = initial_translation * initial_orientation;

        enable_initize    = p("enable.relocalization_initial", bool{});
        enable_relocalize = p("enable.relocalization_automaic", bool{});
        rclcpp_info("initialize pose: %d, relocalization: %d", enable_initize, enable_relocalize);

        if (enable_initize || enable_relocalize) {
            if (pcl::io::loadPCDFile(p("map_path", std::string{}), *standard_map) == -1) {
                rclcpp_warn("can not load pcd file, running without localization");
                map_availiable = false;
            } else {
                rclcpp_info("load successfully, map size: %zu", standard_map->size());
                registration.register_map(standard_map);
            }
        }
        if (enable_relocalize) {
            position_minimum = Eigen::Vector3d{
                p("runtime.minimum_x", double{}),
                p("runtime.minimum_y", double{}),
                p("runtime.minimum_z", double{}),
            };
            position_maximum = Eigen::Vector3d{
                p("runtime.maximum_x", double{}),
                p("runtime.maximum_y", double{}),
                p("runtime.maximum_z", double{}),
            };
            rclcpp_info(
                "position limit: x ⊆ (%4.2f, %4.2f) y ⊆ (%4.2f, %4.2f) z ⊆ (%4.2f, %4.2f)",
                position_minimum.x(), position_maximum.x(), position_minimum.y(),
                position_maximum.y(), position_minimum.z(), position_maximum.z());
        }

        localization_publisher = node.create_publisher<geometry_msgs::msg::PoseStamped>(
            p("publish.robot_pose", std::string{}), 10);
        rclcpp_info("pose topic: %s", p("publish.robot_pose", std::string{}).c_str());

        // 读取 initial_pose, 加上里程计的位姿，纠正后重新发布
        odometer_subscription = node.create_subscription<geometry_msgs::msg::PoseStamped>(
            p("subscription.slam_pose", std::string{}), 10,
            [this](const std::unique_ptr<geometry_msgs::msg::PoseStamped>& msg) {
                auto orientation = Eigen::Quaterniond{};
                auto translation = Eigen::Translation3d{};
                util::convert_orientation(msg->pose.orientation, orientation);
                util::convert_vector3(msg->pose.position, translation);

                auto transformed = Eigen::Isometry3d{translation * orientation * initial_pose};
                auto transformed_orientation = Eigen::Quaterniond{transformed.rotation()};
                auto transformed_translation = Eigen::Translation3d{transformed.translation()};

                auto posestamped = geometry_msgs::msg::PoseStamped{};
                util::convert_orientation(transformed_orientation, posestamped.pose.orientation);
                util::convert_vector3(transformed_translation, posestamped.pose.position);

                localization_publisher->publish(posestamped);
            });
        rclcpp_info("slam topic: %s", p("subscription.slam_pose", std::string{}).c_str());

        initialize_service = node.create_service<std_srvs::srv::Trigger>(
            "/rmcs_location/initialize", TRIGGER_CALLBACK{});

        missons[Status::NONE_ACTION] = [] {
            // do something
        };
        missons[Status::PREPARATION] = [] {
            // do something
        };
        missons[Status::RUNTIME] = [] {
            // do something
        };
        missons[Status::LOST_LOCATION] = [] {
            // do something
        };

        // 确保所有情况被初始化，防呆防傻必备
        if (missons.size() != std::size_t(Status::MISSION_SIZE)) {
            throw util::runtime_error("[rmcs-location][runtime] not all status has mission");
        }

        using namespace std::chrono_literals;
        runtime_timer = node.create_wall_timer(100ms, [this] { update(); });
    }

    void update() { missons[runtime_status](); }

    void initialize_localization(const Eigen::Isometry3d& pose) {}

    void relocalization() {}
};

Runtime::Runtime()
    : pimpl(std::make_unique<Impl>()) {}

Runtime::~Runtime() = default;

void Runtime::initialize(rclcpp::Node& node) { pimpl->initialize(node); }
