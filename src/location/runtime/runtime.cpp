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
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace rmcs;

struct Runtime::Impl {

    void initialize(rclcpp::Node& node) {
        registration.initialize(node);

        const auto message = util::title_text("rmcs-location runtime initializing now");
        rclcpp_info(message.c_str());

        const auto p = util::quick_paramtetr_reader{node};

        /// 运行时相关参数初始化
        ///
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

        enable_initize      = p("enable.relocalization_initial", bool{});
        enable_relocalize   = p("enable.relocalization_automaic", bool{});
        enable_direct_start = p("enable.direct_start", bool{});
        rclcpp_info("initialize pose: %d, relocalization: %d", enable_initize, enable_relocalize);

        receive_size = p("registration.receive_size", std::size_t{});
        rclcpp_info("registration receive size: %zu", receive_size);

        world_link = p("link.world", std::string{});
        slam_link  = p("link.slam", std::string{});

        if (enable_initize || enable_relocalize) {
            if (pcl::io::loadPCDFile(p("map_path", std::string{}), *standard_map) == -1) {
                is_map_availiable = false;
                rclcpp_warn("can not load pcd file, running without localization");
            } else {
                registration.register_map(standard_map);
                is_map_availiable = true;
                rclcpp_info(
                    "load successfully from %s, map size: %zu",
                    p("map_path", std::string{}).c_str(), standard_map->size());
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

        /// ROS2 相关接口资源初始化
        ///
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

                auto transformed = Eigen::Isometry3d{initial_pose * translation * orientation};
                auto transformed_orientation = Eigen::Quaterniond{transformed.rotation()};
                auto transformed_translation = Eigen::Translation3d{transformed.translation()};

                auto posestamped = geometry_msgs::msg::PoseStamped{};
                util::convert_orientation(transformed_orientation, posestamped.pose.orientation);
                util::convert_vector3(transformed_translation, posestamped.pose.position);

                posestamped.header.frame_id = world_link;
                posestamped.header.stamp    = msg->header.stamp;
                localization_publisher->publish(posestamped);
            });
        rclcpp_info("slam topic: %s", p("subscription.slam_pose", std::string{}).c_str());

        pointcloud_subscription = node.create_subscription<sensor_msgs::msg::PointCloud2>(
            p("subscription.pointcloud", std::string{}), 10,
            [this](const std::unique_ptr<sensor_msgs::msg::PointCloud2>& msg) {
                if (!is_collecting_pointcloud)
                    return;

                scanning_frame = std::make_shared<PointCloud>();
                pcl::fromROSMsg(*msg, *scanning_frame);
            });
        rclcpp_info("pointcloud topic: %s", p("subscription.pointcloud", std::string{}).c_str());

        static_transform_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

        constexpr auto service_name{"/rmcs_location/initialize"};
        initialize_service = node.create_service<std_srvs::srv::Trigger>(
            service_name, TRIGGER_CALLBACK(&, this) {
                rclcpp_info("rmcs-location has handled this request");
                const auto callback = [&](bool success, const std::string& msg) {
                    response->success = success;
                };
                util::service::rmcs_slam::reset(node, callback);
            });
        rclcpp_info("custom reinitialize service ready: %s", service_name);

        /// 运行时任务分配
        ///
        missons[Status::NONE_ACTION] = [this] {
            // do something
        };
        missons[Status::PREPARATION] = [&, this] {
            const auto send_collect_pointcloud_request = [this] {
                scanning_frame           = std::make_shared<PointCloud>();
                is_collecting_pointcloud = true;
                rclcpp_info("prepare to initialize, send request to collect pointcloud");
            };
            const auto collecting_pointcloud_and_initialize = [this] {
                // 收集到的点云满足配准要求
                if (scanning_frame && scanning_frame->size() > receive_size) {
                    initialize_localization();
                    // 初始化结束
                    is_collecting_pointcloud = false;
                    is_initializing_location = true;
                } else {
                    // 点云数量不满足要求，继续收集
                    rclcpp_info(
                        "current pointcloud' size is %zu, continue collecting ...",
                        scanning_frame->size());

                    // TODO: 是否会阻塞主线程
                    using namespace std::chrono_literals;
                    rclcpp::sleep_for(1000ms);
                }
            };

            // 若未进行初始化
            if (!is_initializing_location && enable_initize && is_map_availiable) {
                // 未收集初始点云，收集点云
                if (!is_collecting_pointcloud) {
                    send_collect_pointcloud_request();
                } else {
                    collecting_pointcloud_and_initialize();
                }
            }
            // 初始化完成阶段，进入 RUNTIME 状态
            else {
                entry_mission(Status::RUNTIME);
            }
        };
        missons[Status::RUNTIME] = [this] {
            // do something
        };
        missons[Status::LOST_LOCATION] = [this] {
            // do something
        };

        // 确保所有情况被初始化，防呆防傻必备
        if (missons.size() != std::size_t(Status::MISSION_SIZE)) {
            throw util::runtime_error("[rmcs-location][runtime] not all status has mission");
        }

        publish_static_transform(initial_pose, slam_link, world_link);
        entry_mission(enable_direct_start ? Status::PREPARATION : Status::RUNTIME);

        using namespace std::chrono_literals;
        runtime_timer = node.create_wall_timer(100ms, [this] { update(); });
    }

private:
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
    bool enable_direct_start;

    std::size_t receive_size = 1000;

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
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> pointcloud_subscription;
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> localization_publisher;

    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> initialize_service;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_transform_broadcaster;

    std::shared_ptr<rclcpp::TimerBase> runtime_timer;

    std::string world_link;
    std::string slam_link;

    // 系统运行相关资源
    bool is_collecting_pointcloud = false;
    bool is_map_availiable        = false;

    // PREPARATION
    bool is_initializing_location = false;

    // RUNTIME

private:
    void update() { missons[runtime_status](); }

    void initialize_localization() {
        registration.register_scan(scanning_frame);

        auto aligned = std::make_shared<PointCloud>();
        registration.full_match(aligned, Eigen::Affine3f{initial_pose.cast<float>()});

        auto result                = registration.transformation();
        initial_pose.translation() = result.translation().cast<double>();
        initial_pose.linear()      = result.rotation().cast<double>();

        auto score = registration.fitness_score();
        rclcpp_info("initialize pose over, score: %5.4f", score);

        auto t = Eigen::Translation3d{initial_pose.translation()};
        auto q = Eigen::Quaterniond{initial_pose.rotation()};
        rclcpp_info(
            "result t(%4.2f %4.2f %4.2f ) q(%4.2f %4.2f %4.2f %4.2f)", //
            t.x(), t.y(), t.z(), q.w(), q.x(), q.y(), q.z());

        publish_static_transform(initial_pose, slam_link, world_link);
    }

    void relocalization() {}

    void entry_mission(Status status) { runtime_status = status; }

    void publish_static_transform(
        const Eigen::Isometry3d& t, const std::string& link, const std::string& child_link) {
        auto stamp = geometry_msgs::msg::TransformStamped{};
        util::convert_orientation(Eigen::Quaterniond{t.rotation()}, stamp.transform.rotation);
        util::convert_vector3(Eigen::Translation3d{t.translation()}, stamp.transform.translation);

        stamp.header.stamp    = rclcpp::Clock{RCL_SYSTEM_TIME}.now();
        stamp.header.frame_id = link;
        stamp.child_frame_id  = child_link;
        static_transform_broadcaster->sendTransform(stamp);

        rclcpp_info("publish static transform from %s to %s", link.c_str(), child_link.c_str());
    }
};

Runtime::Runtime()
    : pimpl(std::make_unique<Impl>()) {}

Runtime::~Runtime() = default;

void Runtime::initialize(rclcpp::Node& node) { pimpl->initialize(node); }
