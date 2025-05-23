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
public:
    void initialize(rclcpp::Node& node) {
        registration_.initialize(node);

        const auto p = util::quick_paramtetr_reader { node };

        const auto message = util::title_text("rmcs-location runtime initializing now");
        log::info(message.c_str());

        /// 运行时相关参数初始化
        ///
        auto initialize_pose = [&](const std::string& side, Eigen::Isometry3f& pose) {
            auto initial_translation = Eigen::Translation3f {
                p(side + "_pose.translation.x", float {}),
                p(side + "_pose.translation.y", float {}),
                p(side + "_pose.translation.z", float {}),
            };
            auto initial_orientation =
                Eigen::Quaternionf {
                    p(side + "_pose.orientation.w", float {}),
                    p(side + "_pose.orientation.x", float {}),
                    p(side + "_pose.orientation.y", float {}),
                    p(side + "_pose.orientation.z", float {}),
                }
                    .normalized();
            log::info("%s pose: t( %4.2f %4.2f %4.2f ) q( %4.2f %4.2f %4.2f %4.2f )", side.c_str(),
                initial_translation.x(), initial_translation.y(), initial_translation.z(),
                initial_orientation.w(), initial_orientation.x(), initial_orientation.y(),
                initial_orientation.z());
            pose = initial_translation * initial_orientation;
        };
        initialize_pose("default", default_pose_);
        initialize_pose("opposite", opposite_pose_);

        initial_pose_ = default_pose_;

        enable_initialize_   = p("enable.relocalization_initial", bool {});
        enable_relocalize_   = p("enable.relocalization_automaic", bool {});
        enable_direct_start_ = p("enable.direct_start", bool {});
        log::info(
            "initialize pose: %d, relocalization: %d", enable_initialize_, enable_relocalize_);

        receive_pointcloud_size_ = p("registration.receive_size", std::size_t {});
        registration_radius_     = p("registration.initial_map_radius", double {});
        filter_alpha_            = p("runtime.filter_alpha", float {});

        // 需要配准时，读取点云地图
        if (enable_initialize_ || enable_relocalize_) {
            if (pcl::io::loadPCDFile(p("map_path", std::string {}), *standard_map_) == -1) {
                is_map_availiable_ = false;
                log::warn("can not load pcd file, running without localization");
            } else {
                is_map_availiable_ = true;
                log::info("load successfully from %s, map size: %zu",
                    p("map_path", std::string {}).c_str(), standard_map_->size());
            }
        }
        // 需要运行时检测丢失定位进行重定位时，加载丢失定位的位置判据
        if (enable_relocalize_) {
            position_minimum_ = Eigen::Vector3f {
                p("runtime.minimum_x", float {}),
                p("runtime.minimum_y", float {}),
                p("runtime.minimum_z", float {}),
            };
            position_maximum_ = Eigen::Vector3f {
                p("runtime.maximum_x", float {}),
                p("runtime.maximum_y", float {}),
                p("runtime.maximum_z", float {}),
            };
            log::info("position limit: x ⊆ (%4.2f, %4.2f) y ⊆ (%4.2f, %4.2f) z ⊆ (%4.2f, %4.2f)",
                position_minimum_.x(), position_maximum_.x(), position_minimum_.y(),
                position_maximum_.y(), position_minimum_.z(), position_maximum_.z());
        }

        /// ROS2 相关接口资源初始化
        ///

        // 重定位后位姿发布
        localization_publisher_ = node.create_publisher<geometry_msgs::msg::PoseStamped>(
            p("publish.robot_pose", std::string {}), 10);
        log::info("pose topic: %s", p("publish.robot_pose", std::string {}).c_str());

        // 读取 initial_pose, 加上里程计的位姿，纠正后重新发布
        odometer_subscription_ = node.create_subscription<geometry_msgs::msg::PoseStamped>(
            p("subscription.slam_pose", std::string {}), 10,
            [this](const std::unique_ptr<geometry_msgs::msg::PoseStamped>& msg) {
                auto orientation = Eigen::Quaternionf {};
                auto translation = Eigen::Translation3f {};
                util::convert_orientation(msg->pose.orientation, orientation);
                util::convert_translation(msg->pose.position, translation);

                auto current_translation =
                    Eigen::Translation3f { initial_pose_ * translation.translation() };
                auto current_orientation =
                    Eigen::Quaternionf { initial_pose_.rotation() * orientation };

                apply_ema_pose_filter(current_translation, current_orientation,
                    filtered_translation_, filtered_orientation_, filter_alpha_);

                auto posestamped = geometry_msgs::msg::PoseStamped {};
                util::convert_orientation(filtered_orientation_, posestamped.pose.orientation);
                util::convert_translation(filtered_translation_, posestamped.pose.position);

                posestamped.header.frame_id = string::world_link;
                posestamped.header.stamp    = msg->header.stamp;
                localization_publisher_->publish(posestamped);

                pose_stamped_deque_.push_back(posestamped);
                if (pose_stamped_deque_.size() >= maximum_pose_stamped_deque_size_)
                    pose_stamped_deque_.pop_front();
            });
        log::info("slam topic: %s", p("subscription.slam_pose", std::string {}).c_str());

        // 接收用于重定位配准的点云，一般是 SLAM 在一定时间内建好的部分地图
        pointcloud_subscription_ = node.create_subscription<sensor_msgs::msg::PointCloud2>(
            p("subscription.pointcloud", std::string {}), 10,
            [this](const std::unique_ptr<sensor_msgs::msg::PointCloud2>& msg) {
                if (!is_collecting_.load(std::memory_order::relaxed)) return;
                scanning_frame_ = std::make_shared<PointCloud>();
                pcl::fromROSMsg(*msg, *scanning_frame_);
            });
        log::info("pointcloud topic: %s", p("subscription.pointcloud", std::string {}).c_str());

        {
            // 用于重启和初始化整个系统的服务，该服务会自动将 SLAM 也重启
            const auto service_name { string::location::initialize_service_name };
            const auto service_callback = TRIGGER_CALLBACK(&, this) {
                util::service::rmcs_slam::reset(node);
                entry_mission(Status::PREPARATION);

                response->success = true;
                response->message = "rmcs-location has handled this request";

                log::info("rmcs-location has handled this request");
            };
            using Service       = std_srvs::srv::Trigger;
            initialize_service_ = node.create_service<Service>(service_name, service_callback);
            log::info("custom reinitialize service ready: %s", service_name);
        }
        {
            // 设置哨兵的边，true 的初始点为 (0,0) false 的初始点为另一边
            const auto service_name { string::location::update_side_service };
            const auto service_callback = SET_BOOL_CALLBACK(this) {
                response->message = "rmcs-location has handled this request";
                response->success = true;
                initial_pose_     = request->data ? default_pose_ : opposite_pose_;
                log::info("the side was set to %s", request->data ? "default" : "opposite");
                log::info("current initial pose: %s", util::to_string(initial_pose_).c_str());
            };
            using Service        = std_srvs::srv::SetBool;
            update_side_service_ = node.create_service<Service>(service_name, service_callback);
            log::info("service of updating robot side is ready: %s", service_name);
        }

        static_transform_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

        /// 运行时任务分配
        ///
        install_missions();

        using namespace std::chrono_literals;
        runtime_timer_ = node.create_wall_timer(100ms, [this] { update(); });

        transform_notification_timer_ = node.create_wall_timer(1s, [this] {
            publish_static_transform(initial_pose_, string::world_link, string::slam_link);
        });
    }

private:
    using log = util::Log<[] { return "rmcs-location"; }>;

    Registration registration_ {};
    Segmentation segmentation_ {};

    enum class Status : uint8_t {
        NONE_ACTION,
        PREPARATION,
        RUNTIME,
        MISSION_SIZE,
    };
    std::atomic<Status> runtime_status_;

    using Misson = std::function<void()>;
    std::unordered_map<Status, Misson> missons_;

    // ROS2 相关接口
    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>> odometer_subscription_;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> pointcloud_subscription_;
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> localization_publisher_;

    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> initialize_service_;
    std::shared_ptr<rclcpp::Service<std_srvs::srv::SetBool>> update_side_service_;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_transform_broadcaster_;

    std::shared_ptr<rclcpp::TimerBase> runtime_timer_;
    std::shared_ptr<rclcpp::TimerBase> transform_notification_timer_;

    // 系统运行相关资源
    bool enable_record_       = false;
    bool enable_initialize_   = false;
    bool enable_relocalize_   = false;
    bool enable_direct_start_ = true;
    bool is_map_availiable_   = false;

    std::atomic<bool> is_collecting_        = false;
    std::atomic<bool> is_relocalizing_      = false;
    std::atomic<bool> is_finish_relocalize_ = false;

    // 初始位姿，World 系到 SLAM 系的变换
    Eigen::Isometry3f initial_pose_  = Eigen::Isometry3f::Identity();
    Eigen::Isometry3f default_pose_  = Eigen::Isometry3f::Identity();
    Eigen::Isometry3f opposite_pose_ = Eigen::Isometry3f::Identity();

    // 用于配准的点云的最小点云数量
    std::size_t receive_pointcloud_size_ = 1000;
    // 用于配准的地图球形子图半径
    std::double_t registration_radius_ = 25.0;

    Eigen::Vector3f position_minimum_ = { 0, 0, 0 };
    Eigen::Vector3f position_maximum_ = { 0, 0, 0 };

    std::shared_ptr<PointCloud> standard_map_   = std::make_shared<PointCloud>();
    std::shared_ptr<PointCloud> scanning_frame_ = std::make_shared<PointCloud>();

    std::size_t maximum_pose_stamped_deque_size_ = 100;
    std::deque<geometry_msgs::msg::PoseStamped> pose_stamped_deque_;

    float filter_alpha_ = 0.1;
    Eigen::Translation3f filtered_translation_;
    Eigen::Quaternionf filtered_orientation_;

private:
    /// @brief: 更新函数，各个阶段的任务将会在此处执行
    void update() { missons_[runtime_status_](); }

    void install_missions() {
        missons_[Status::NONE_ACTION] = [this] {
            // do something
        };
        missons_[Status::PREPARATION] = [this] {
            // map 不可用，直接进入运行模式
            if (!is_map_availiable_) {
                log::info("the map is unavailable, just use initial pose on config");
                entry_mission(Status::RUNTIME);
                return;
            }
            // 不需要初始化，进入运行模式
            if (!enable_initialize_) {
                log::info("initialize pose is canceled, ignore request");
                entry_mission(Status::RUNTIME);
                return;
            }
            // 尝试进行位姿初始化，成功后会将 is_initialized 设置为 true
            update_relocalization();
        };
        missons_[Status::RUNTIME] = [this] {
            // 如果不启用运行时检测定位丢失，那没什么需要做的了
            if (!enable_relocalize_) return;
        };

        // 确保所有情况被初始化，防呆防傻必备
        if (missons_.size() != std::size_t(Status::MISSION_SIZE)) {
            throw util::runtime_error("[rmcs-location][runtime] not all status has mission");
        }

        entry_mission(enable_direct_start_ ? Status::PREPARATION : Status::NONE_ACTION);
    }

    /// @brief: 进行重定位，初始解是 initial_pose，地图是配置文件传入的地图
    /// @note: 会对子地图进行点云分割，去除地面，保留有效特征
    void relocalization() {
        segmentation_.set_distance_threshold(0.3);
        segmentation_.set_ground_max_height(0.3);
        segmentation_.set_limit_distance(50);
        segmentation_.set_limit_max_height(10);

        auto initial_position   = initial_pose_.translation();
        auto initial_pointcloud = extract_pointcloud(standard_map_,
            Point { initial_position.x(), initial_position.y(), initial_position.z() });

        log::info("the size of pointcloud around initial pose: %zu", initial_pointcloud->size());

        registration_.register_map(initial_pointcloud);

        segmentation_.set_input_source(scanning_frame_);
        *scanning_frame_ = *segmentation_.execute();
        registration_.register_scan(scanning_frame_);

        auto aligned = std::make_shared<PointCloud>();
        registration_.full_match(aligned, Eigen::Isometry3f { initial_pose_ });

        auto result                 = registration_.transformation();
        initial_pose_.translation() = result.translation();
        initial_pose_.linear()      = result.rotation();

        auto score = registration_.fitness_score();
        log::info("initialize pose over, score: %5.4f", score);

        auto t = Eigen::Translation3f { initial_pose_.translation() };
        auto q = Eigen::Quaternionf { initial_pose_.rotation() };
        log::info("result t(%4.2f %4.2f %4.2f ) q(%4.2f %4.2f %4.2f %4.2f)", //
            t.x(), t.y(), t.z(), q.w(), q.x(), q.y(), q.z());
    }

    void update_relocalization() {
        using namespace std::chrono_literals;

        // 没有地图配个什么
        if (!is_map_availiable_) {
            entry_mission(Status::RUNTIME);
            return;
        }

        // 点云收集未开始，则开始
        if (!is_collecting_.load(std::memory_order::relaxed)) {
            is_collecting_.store(true, std::memory_order::relaxed);
            scanning_frame_ = std::make_shared<PointCloud>();
            return;
        }

        // 点云未收集完毕，继续收集
        if (auto size = scanning_frame_->size(); size <= receive_pointcloud_size_) {
            log::info("continue collecting, size: %zu", size);
            rclcpp::sleep_for(2000ms);
            return;
        }

        // 重定位未开始，则开始
        if (!is_relocalizing_.load(std::memory_order::relaxed)) {
            is_finish_relocalize_.store(false, std::memory_order::relaxed);
            std::thread { [this] {
                relocalization();
                is_finish_relocalize_.store(true, std::memory_order::relaxed);
            } }.detach();
            is_relocalizing_.store(true, std::memory_order::relaxed);
            return;
        }

        // 结束重定位，进入下一阶段
        if (is_finish_relocalize_.load(std::memory_order::relaxed)) {
            entry_mission(Status::RUNTIME);
            return;
        }
    }

    void entry_mission(Status status) { runtime_status_.store(status, std::memory_order::relaxed); }

    void publish_static_transform(
        const Eigen::Isometry3f& t, const std::string& link, const std::string& child_link) {

        auto stamp = geometry_msgs::msg::TransformStamped {};
        util::convert_orientation(Eigen::Quaternionf { t.rotation() }, stamp.transform.rotation);
        util::convert_translation(
            Eigen::Translation3f { t.translation() }, stamp.transform.translation);

        stamp.header.stamp    = rclcpp::Clock { RCL_SYSTEM_TIME }.now();
        stamp.header.frame_id = link;
        stamp.child_frame_id  = child_link;
        static_transform_broadcaster_->sendTransform(stamp);
    }

    std::shared_ptr<PointCloud> extract_pointcloud(
        const std::shared_ptr<PointCloud>& pointcloud, Point center) const {
        auto flann_kd_tree = pcl::KdTreeFLANN<Point> {};
        flann_kd_tree.setInputCloud(pointcloud);

        auto indecis   = pcl::Indices {};
        auto distances = std::vector<float> {};
        flann_kd_tree.radiusSearch(center, registration_radius_, indecis, distances);

        auto search_result = std::make_shared<PointCloud>(indecis.size(), 1);
        for (const auto index : indecis) {
            search_result->points.push_back(pointcloud->at(index));
        }
        return search_result;
    }

    static void apply_ema_pose_filter(const Eigen::Translation3f& current_translation,
        const Eigen::Quaternionf& current_rotation, Eigen::Translation3f& filtered_translation,
        Eigen::Quaternionf& filtered_rotation, float alpha) {
        // 平移部分滤波
        filtered_translation.translation() = alpha * current_translation.translation()
            + (1.0 - alpha) * filtered_translation.translation();
        // 旋转部分滤波（四元数 Slerp 插值）
        filtered_rotation = filtered_rotation.slerp(alpha, current_rotation);
    }
};

Runtime::Runtime()
    : pimpl(std::make_unique<Impl>()) { }

Runtime::~Runtime() = default;

void Runtime::initialize(rclcpp::Node& node) { pimpl->initialize(node); }
