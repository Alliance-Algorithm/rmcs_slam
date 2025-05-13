#include "runtime.hpp"
#include "registration/engine.hpp"
#include "util/convert.hpp"
#include "util/logger.hpp"
#include "util/parameter.hpp"
#include "util/segmentation.hpp"
#include "util/service.hpp"

#include <Eigen/Eigen>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <cassert>
#include <deque>
#include <functional>
#include <memory>
#include <unordered_map>
#include <vector>

using namespace rmcs;

struct Runtime::Impl {
private:
    RMCS_INITIALIZE_LOGGER("rmcs-location");

    Registration registration {};
    Segmentation segmentation {};

    enum class Status {
        NONE_ACTION,
        PREPARATION,
        RUNTIME,
        MISSION_SIZE,
    } runtime_status;

    using Misson = std::function<void()>;
    std::unordered_map<Status, Misson> missons;

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
    bool enable_record       = false;
    bool enable_initialize   = false;
    bool enable_relocalize   = false;
    bool enable_direct_start = true;

    bool is_initialized          = false;
    bool is_map_availiable       = false;
    bool need_collect_pointcloud = false;

    // 初始位姿，World 系到 SLAM 系的变换
    Eigen::Isometry3f initial_pose = Eigen::Isometry3f::Identity();

    // 用于配准的点云的最小点云数量
    std::size_t receive_pointcloud_size = 1000;
    // 用于配准的地图球形子图半径
    std::double_t registration_radius = 25.0;

    Eigen::Vector3f position_minimum = { 0, 0, 0 };
    Eigen::Vector3f position_maximum = { 0, 0, 0 };

    std::shared_ptr<PointCloud> standard_map   = std::make_shared<PointCloud>();
    std::shared_ptr<PointCloud> scanning_frame = std::make_shared<PointCloud>();

    std::size_t maximum_pose_stamped_deque_size = 100;
    std::deque<geometry_msgs::msg::PoseStamped> pose_stamped_deque;

public:
    void initialize(rclcpp::Node& node) {
        registration.initialize(node);

        const auto p = util::quick_paramtetr_reader { node };

        const auto message = util::title_text("rmcs-location runtime initializing now");
        rclcpp_info(message.c_str());

        /// 运行时相关参数初始化
        ///

        auto initial_translation = Eigen::Translation3f {
            p("initial_pose.translation.x", float {}),
            p("initial_pose.translation.y", float {}),
            p("initial_pose.translation.z", float {}),
        };
        auto initial_orientation = Eigen::Quaternionf { p("initial_pose.orientation.w", float {}),
            p("initial_pose.orientation.x", float {}), p("initial_pose.orientation.y", float {}),
            p("initial_pose.orientation.z", float {}) }
                                       .normalized();
        rclcpp_info("initial pose: t( %4.2f %4.2f %4.2f ) q( %4.2f %4.2f %4.2f %4.2f )",
            initial_translation.x(), initial_translation.y(), initial_translation.z(),
            initial_orientation.w(), initial_orientation.x(), initial_orientation.y(),
            initial_orientation.z());

        initial_pose = initial_translation * initial_orientation;

        enable_initialize   = p("enable.relocalization_initial", bool {});
        enable_relocalize   = p("enable.relocalization_automaic", bool {});
        enable_direct_start = p("enable.direct_start", bool {});
        rclcpp_info(
            "initialize pose: %d, relocalization: %d", enable_initialize, enable_relocalize);

        receive_pointcloud_size = p("registration.receive_size", std::size_t {});
        registration_radius     = p("registration.initial_map_radius", double {});
        rclcpp_info("registration receive size: %zu", receive_pointcloud_size);

        world_link = p("link.world", std::string {});
        slam_link  = p("link.slam", std::string {});

        // 需要配准时，读取点云地图
        if (enable_initialize || enable_relocalize) {
            if (pcl::io::loadPCDFile(p("map_path", std::string {}), *standard_map) == -1) {
                is_map_availiable = false;
                rclcpp_warn("can not load pcd file, running without localization");
            } else {
                is_map_availiable = true;
                rclcpp_info("load successfully from %s, map size: %zu",
                    p("map_path", std::string {}).c_str(), standard_map->size());
            }
        }
        // 需要运行时检测丢失定位进行重定位时，加载丢失定位的位置判据
        if (enable_relocalize) {
            position_minimum = Eigen::Vector3f {
                p("runtime.minimum_x", float {}),
                p("runtime.minimum_y", float {}),
                p("runtime.minimum_z", float {}),
            };
            position_maximum = Eigen::Vector3f {
                p("runtime.maximum_x", float {}),
                p("runtime.maximum_y", float {}),
                p("runtime.maximum_z", float {}),
            };
            rclcpp_info("position limit: x ⊆ (%4.2f, %4.2f) y ⊆ (%4.2f, %4.2f) z ⊆ (%4.2f, %4.2f)",
                position_minimum.x(), position_maximum.x(), position_minimum.y(),
                position_maximum.y(), position_minimum.z(), position_maximum.z());
        }

        /// ROS2 相关接口资源初始化
        ///

        // 重定位后位姿发布
        localization_publisher = node.create_publisher<geometry_msgs::msg::PoseStamped>(
            p("publish.robot_pose", std::string {}), 10);
        rclcpp_info("pose topic: %s", p("publish.robot_pose", std::string {}).c_str());

        // 读取 initial_pose, 加上里程计的位姿，纠正后重新发布
        odometer_subscription = node.create_subscription<geometry_msgs::msg::PoseStamped>(
            p("subscription.slam_pose", std::string {}), 10,
            [this](const std::unique_ptr<geometry_msgs::msg::PoseStamped>& msg) {
                auto orientation = Eigen::Quaternionf {};
                auto translation = Eigen::Translation3f {};
                util::convert_orientation(msg->pose.orientation, orientation);
                util::convert_translation(msg->pose.position, translation);

                auto posestamped = geometry_msgs::msg::PoseStamped {};
                util::convert_orientation(
                    Eigen::Quaternionf { initial_pose.rotation() * orientation },
                    posestamped.pose.orientation);
                util::convert_translation(
                    Eigen::Translation3f { initial_pose * translation.translation() },
                    posestamped.pose.position);

                posestamped.header.frame_id = world_link;
                posestamped.header.stamp    = msg->header.stamp;
                localization_publisher->publish(posestamped);

                pose_stamped_deque.push_back(posestamped);
                if (pose_stamped_deque.size() >= maximum_pose_stamped_deque_size)
                    pose_stamped_deque.pop_front();
            });
        rclcpp_info("slam topic: %s", p("subscription.slam_pose", std::string {}).c_str());

        // 接收用于重定位配准的点云，一般是 SLAM 在一定时间内建好的部分地图
        pointcloud_subscription = node.create_subscription<sensor_msgs::msg::PointCloud2>(
            p("subscription.pointcloud", std::string {}), 10,
            [this](const std::unique_ptr<sensor_msgs::msg::PointCloud2>& msg) {
                if (!need_collect_pointcloud) return;
                scanning_frame = std::make_shared<PointCloud>();
                pcl::fromROSMsg(*msg, *scanning_frame);
            });
        rclcpp_info("pointcloud topic: %s", p("subscription.pointcloud", std::string {}).c_str());

        static_transform_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

        // 用于重启和初始化整个系统的服务，该服务会自动将 SLAM 也重启
        const auto service_name { "/rmcs_location/initialize" };
        const auto service_callback = TRIGGER_CALLBACK(&, this) {
            const auto callback = [&](bool success, const std::string& msg) {
                if ((response->success = success) == true) {
                    start_collecting_pointcloud();
                    entry_mission(Status::PREPARATION);
                    const auto message = "reset initialized status and relocalize now";
                    response->message  = message;
                    rclcpp_info(message);
                } else {
                    const auto message = "try to reset slam but failed";
                    response->message  = message;
                    rclcpp_info(message);
                }
            };
            util::service::rmcs_slam::reset(node, callback);
            rclcpp_info("rmcs-location has handled this request");
        };
        using Service      = std_srvs::srv::Trigger;
        initialize_service = node.create_service<Service>(service_name, service_callback);
        rclcpp_info("custom reinitialize service ready: %s", service_name);

        /// 运行时任务分配
        ///

        missons[Status::NONE_ACTION] = [this] {
            // do something
        };
        missons[Status::PREPARATION] = [this] {
            // map 不可用，直接进入运行模式
            if (!is_map_availiable) {
                rclcpp_info("the map is unavailable, just use initial pose on config");
                entry_mission(Status::RUNTIME);
                return;
            }
            // 不需要初始化或初始化完成，进入运行模式
            if (!enable_initialize || is_initialized) {
                entry_mission(Status::RUNTIME);
                return;
            }
            // 未收集初始点云，开始收集
            if (!need_collect_pointcloud) {
                start_collecting_pointcloud();
                return;
            }
            // 尝试进行位姿初始化，成功后会将 is_initialized 设置为 true
            try_relocalization();
        };
        missons[Status::RUNTIME] = [this] {
            // 如果不启用运行时检测定位丢失，那没什么需要做的了
            if (!enable_relocalize) return;
        };

        // 确保所有情况被初始化，防呆防傻必备
        if (missons.size() != std::size_t(Status::MISSION_SIZE)) {
            throw util::runtime_error("[rmcs-location][runtime] not all status has mission");
        }

        publish_static_transform(initial_pose, slam_link, world_link);
        entry_mission(enable_direct_start ? Status::PREPARATION : Status::NONE_ACTION);

        using namespace std::chrono_literals;
        runtime_timer = node.create_wall_timer(100ms, [this] { update(); });
    }

private:
    /// @brief: 更新函数，各个阶段的任务将会在此处执行
    void update() { missons[runtime_status](); }

    /// @brief: 进行重定位，初始解是 initial_pose，地图是配置文件传入的地图
    /// @note: 会对子地图进行点云分割，去除地面，保留有效特征
    void relocalization() {
        // 没地图配什么
        if (!is_map_availiable) {
            need_collect_pointcloud = false;
            is_initialized          = true;
            return;
        }

        segmentation.set_distance_threshold(0.3);
        segmentation.set_ground_max_height(0.3);
        segmentation.set_limit_distance(50);
        segmentation.set_limit_max_height(10);

        auto initial_position   = initial_pose.translation();
        auto initial_pointcloud = extract_pointcloud(standard_map,
            Point { initial_position.x(), initial_position.y(), initial_position.z() });

        rclcpp_info("the size of pointcloud around initial pose: %zu", initial_pointcloud->size());

        registration.register_map(initial_pointcloud);

        segmentation.set_input_source(scanning_frame);
        *scanning_frame = *segmentation.execute();
        registration.register_scan(scanning_frame);

        auto aligned = std::make_shared<PointCloud>();
        registration.full_match(aligned, Eigen::Isometry3f { initial_pose });

        auto result                = registration.transformation();
        initial_pose.translation() = result.translation();
        initial_pose.linear()      = result.rotation();

        auto score = registration.fitness_score();
        rclcpp_info("initialize pose over, score: %5.4f", score);

        auto t = Eigen::Translation3f { initial_pose.translation() };
        auto q = Eigen::Quaternionf { initial_pose.rotation() };
        rclcpp_info("result t(%4.2f %4.2f %4.2f ) q(%4.2f %4.2f %4.2f %4.2f)", //
            t.x(), t.y(), t.z(), q.w(), q.x(), q.y(), q.z());

        publish_static_transform(initial_pose.inverse(), slam_link, world_link);

        is_initialized          = true;
        need_collect_pointcloud = false;
    }

    void try_relocalization() {
        if (scanning_frame->size() <= receive_pointcloud_size) {
            rclcpp_info(
                "current pointcloud' size is %zu, continue collecting ...", scanning_frame->size());
            using namespace std::chrono_literals;
            rclcpp::sleep_for(2000ms);
            return;
        }
        relocalization();
    }

    void start_collecting_pointcloud() {
        need_collect_pointcloud = true;
        scanning_frame          = std::make_shared<PointCloud>();
        rclcpp_info("prepare to initialize, send request to collect pointcloud");
    }

    void entry_mission(Status status) { runtime_status = status; }

    void publish_static_transform(
        const Eigen::Isometry3f& t, const std::string& link, const std::string& child_link) {

        auto stamp = geometry_msgs::msg::TransformStamped {};
        util::convert_orientation(Eigen::Quaternionf { t.rotation() }, stamp.transform.rotation);
        util::convert_translation(
            Eigen::Translation3f { t.translation() }, stamp.transform.translation);

        stamp.header.stamp    = rclcpp::Clock { RCL_SYSTEM_TIME }.now();
        stamp.header.frame_id = link;
        stamp.child_frame_id  = child_link;
        static_transform_broadcaster->sendTransform(stamp);

        rclcpp_info("publish static transform from %s to %s", link.c_str(), child_link.c_str());
    }

    std::shared_ptr<PointCloud> extract_pointcloud(
        const std::shared_ptr<PointCloud>& pointcloud, Point center) const {
        auto flann_kd_tree = pcl::KdTreeFLANN<Point> {};
        flann_kd_tree.setInputCloud(pointcloud);

        auto indecis   = pcl::Indices {};
        auto distances = std::vector<float> {};
        flann_kd_tree.radiusSearch(center, registration_radius, indecis, distances);

        auto search_result = std::make_shared<PointCloud>(indecis.size(), 1);
        for (const auto index : indecis) {
            search_result->points.push_back(pointcloud->at(index));
        }
        return search_result;
    }
};

Runtime::Runtime()
    : pimpl(std::make_unique<Impl>()) { }

Runtime::~Runtime() = default;

void Runtime::initialize(rclcpp::Node& node) { pimpl->initialize(node); }
