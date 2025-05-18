#include "slam.hpp"

#include "common/common_lib.hpp"
#include "common/use_ikfom.hpp"
#include "ikd_tree/ikd_tree.hpp"
#include "process/imu_processing.hpp"
#include "process/preprocess.hpp"
#include "ros_util/ros_util.hpp"
#include "synthesizer/synthesizer.hpp"
#include "util/convert.hpp"
#include "util/parameter.hpp"
#include "util/string.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <omp.h>

using namespace rmcs;

static constexpr auto initialize_interval = 0.1;
static constexpr auto lidar_point_cov     = 0.001;

static auto global_mutex  = std::mutex {};
static auto global_signal = std::condition_variable {};

// let a static c style function pointer accesses the no-static member function
using ProcessCallback = std::function<void(state_ikfom&, esekfom::dyn_share_datastruct<double>&)>;
class FuncHelperClass {
    static inline ProcessCallback object;

public:
    static void static_function(state_ikfom& a, esekfom::dyn_share_datastruct<double>& b) {
        return object(a, b);
    }
    static void bind(ProcessCallback callback) { object = std::move(callback); }
};

struct SLAM::Impl {
    void initialize(rclcpp::Node& node) {
        std::signal(SIGINT, [](int) {
            global_signal.notify_all();
            rclcpp::shutdown();
        });

        logger = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("rmcs-slam"));

        ros_utility.initialize(node);
        rclcpp_info("finish initializing ros utility");

        load_parameter_from_config(node);
        rclcpp_info("read parameters from yaml successfully");

        synthesizer.initialize(node);
        synthesizer.register_callback([this](const auto& msg) { lidar_subscription_callback(msg); },
            [this](const auto& msg) { imu_subscription_callback(msg); });
        rclcpp_info("finish initializing synthesizer");

        using namespace std::chrono_literals;
        ros_utility.register_update_function([this] { update(); });
        ros_utility.register_publish_map_function([this] {
            if (enable_publish_constructed_map) publish_constructed_map();
        });
        ros_utility.register_map_save_function([this] { save_to_pcd(); });
        ros_utility.register_reset_function([&, this] { reset_slam(node); });
        ros_utility.register_switch_record_function(
            [this](bool on) { synthesizer.switch_record(on); });

        current_path.header.stamp    = node.get_clock()->now();
        current_path.header.frame_id = string::slam_link;

        fov_degree_calculated = (fov_degree + 10.0) > 179.9 ? 179.9 : (fov_degree + 10.0);
        half_fov_cos          = std::cos((fov_degree_calculated) * 0.5 * std::numbers::pi / 180.0);

        std::memset(points_selected_surf.data(), true, sizeof(points_selected_surf));
        std::memset(residual_last.data(), -1000.0f, sizeof(residual_last));

        downsample_scan_filter.setLeafSize(
            float(filter_size_surf_min), float(filter_size_surf_min), float(filter_size_surf_min));

        std::memset(points_selected_surf.data(), true, sizeof(points_selected_surf));
        std::memset(residual_last.data(), -1000.0f, sizeof(residual_last));

        pointcloud_origin_undistort         = std::make_shared<PointCloudXYZI>();
        pointcloud_downsample_undistort_imu = std::make_shared<PointCloudXYZI>();
        pointcloud_downsample_world         = std::make_shared<PointCloudXYZI>();
        constructed_map_pointcloud          = std::make_shared<PointCloudXYZI>();
        normal_vector                       = std::make_shared<PointCloudXYZI>(100000, 1);
        origin_constructed_map              = std::make_shared<PointCloudXYZI>(100000, 1);
        normal_vector_correspondence        = std::make_shared<PointCloudXYZI>(100000, 1);

        rclcpp_info("slam source initialized successfully");

        auto translation = Eigen::Vector3d {};
        auto orientation = Eigen::Matrix3d {};

        translation << VEC_FROM_ARRAY(extrinsic_translation);
        orientation << MAT_FROM_ARRAY(extrinsic_orientation);

        imu_process->set_extrinsic(translation, orientation);
        imu_process->set_gyr_cov(
            V3D(covariance_gyroscope, covariance_gyroscope, covariance_gyroscope));
        imu_process->set_acc_cov(
            V3D(covariance_accelerometer, covariance_accelerometer, covariance_accelerometer));
        imu_process->set_gyr_bias_cov(
            V3D(covariance_gyroscpoe_bias, covariance_gyroscpoe_bias, covariance_gyroscpoe_bias));
        imu_process->set_acc_bias_cov(V3D(covariance_accelerometer_bias,
            covariance_accelerometer_bias, covariance_accelerometer_bias));

        rclcpp_info("imu process initialized successfully");

        FuncHelperClass::bind([this](state_ikfom& a, esekfom::dyn_share_datastruct<double>& b) {
            h_share_model(a, b);
        });

        std::fill(ekf_limit_array.begin(), ekf_limit_array.end(), 0.001);
        extended_kalman_filter.init_dyn_share(get_f, df_dx, df_dw, FuncHelperClass::static_function,
            ekf_max_iterations, ekf_limit_array.data());

        rclcpp_info("slam init finished");
    }

    void update() {
        if (!bind_sensor_packages(current_sensor_package)) return;

        const auto& package = current_sensor_package;

        if (first_update) {
            imu_process->first_lidar_time = first_lidar_timetamp;
            first_lidar_timetamp          = package.lidar_beg_time;
            first_update                  = false;
            return;
        }

        // 使用陀螺仪进行点云的去畸变
        imu_process->process(package, extended_kalman_filter, pointcloud_origin_undistort);

        current_ekf_prediction = extended_kalman_filter.get_x();

        current_robot_pose = current_ekf_prediction.pos
            + current_ekf_prediction.rot * current_ekf_prediction.offset_T_L_I;

        if (pointcloud_origin_undistort->empty() || (pointcloud_origin_undistort == nullptr)) {
            rclcpp_warn("no point, skip this scan!");
            return;
        }

        ekf_initialized =
            (package.lidar_beg_time - first_lidar_timetamp) < initialize_interval ? false : true;

        // segment the map in lidar fov
        lasermap_fov_segment();

        // downsample the feature points in a scan
        downsample_scan_filter.setInputCloud(pointcloud_origin_undistort);
        downsample_scan_filter.filter(*pointcloud_downsample_undistort_imu);

        feats_downsample_size =
            static_cast<int>(pointcloud_downsample_undistort_imu->points.size());

        // initialize the map kdtree
        if (ikdtree.Root_Node == nullptr) {
            rclcpp_info("initialize the map kdtree");

            if (feats_downsample_size > 5) {
                ikdtree.set_downsample_param(static_cast<float>(filter_size_map_min));
                pointcloud_downsample_world->resize(feats_downsample_size);
                for (int i = 0; i < feats_downsample_size; i++) {
                    point_lidar_to_world(pointcloud_downsample_undistort_imu->points[i],
                        pointcloud_downsample_world->points[i]);
                }

                ikdtree.Build(pointcloud_downsample_world->points);
            }
            return;
        }

        if (feats_downsample_size < 5) {
            rclcpp_warn("no point, skip this scan!");
            return;
        }

        // icp and iterated Kalman filter update
        normal_vector->resize(feats_downsample_size);
        pointcloud_downsample_world->resize(feats_downsample_size);

        V3D ext_euler = SO3ToEuler(current_ekf_prediction.offset_R_L_I);

        nearest_points.resize(feats_downsample_size);
        int rematch_num        = 0;
        bool nearest_search_en = true;

        // iterated state estimation
        double t_update_start = omp_get_wtime();
        double solve_h_time   = 0;
        extended_kalman_filter.update_iterated_dyn_share_modified(lidar_point_cov, solve_h_time);
        current_ekf_prediction = extended_kalman_filter.get_x();

        current_robot_pose = current_ekf_prediction.pos
            + current_ekf_prediction.rot * current_ekf_prediction.offset_T_L_I;

        current_orientation.x = current_ekf_prediction.rot.coeffs()[0];
        current_orientation.y = current_ekf_prediction.rot.coeffs()[1];
        current_orientation.z = current_ekf_prediction.rot.coeffs()[2];
        current_orientation.w = current_ekf_prediction.rot.coeffs()[3];

        double t_update_end = omp_get_wtime();

        // publish odometry
        publish_odometry();

        // add the feature points to map kdtree
        map_incremental();

        // publish the cloud
        if (enable_publish_path) publish_path();

        if (enable_publish_pointcloud_all) {
            publish_pointcloud_registerd_world();

            if (enable_publish_pointcloud_imu) publish_pointcloud_registerd_imu();

            if (enable_publish_pointcloud_effect_world) publish_pointcloud_effect_world();
        }
    }

    void load_current_pose_stamp(auto& stamp) {
        stamp.pose.position.x    = current_ekf_prediction.pos(0);
        stamp.pose.position.y    = current_ekf_prediction.pos(1);
        stamp.pose.position.z    = current_ekf_prediction.pos(2);
        stamp.pose.orientation.x = current_orientation.x;
        stamp.pose.orientation.y = current_orientation.y;
        stamp.pose.orientation.z = current_orientation.z;
        stamp.pose.orientation.w = current_orientation.w;
    }

    // 从机器人系变换到世界系
    void point_lidar_to_world(const PointT& src, PointT& dst) const {
        const auto result = current_ekf_prediction;

        auto point_body  = Eigen::Vector3d(src.x, src.y, src.z);
        auto point_world = Eigen::Vector3d {
            result.rot * (result.offset_R_L_I * point_body + result.offset_T_L_I) + result.pos
        };

        dst.x = static_cast<float>(point_world(0));
        dst.y = static_cast<float>(point_world(1));
        dst.z = static_cast<float>(point_world(2));

        dst.intensity = src.intensity;
    }

    void point_lidar_to_imu(const PointT& src, PointT& dst) const {
        const auto result = current_ekf_prediction;

        auto point_lidar = Eigen::Vector3d(src.x, src.y, src.z);
        auto point_imu   = Eigen::Vector3d(result.offset_R_L_I * point_lidar + result.offset_T_L_I);

        dst.x = static_cast<float>(point_imu(0));
        dst.y = static_cast<float>(point_imu(1));
        dst.z = static_cast<float>(point_imu(2));

        dst.intensity = src.intensity;
    }

    void publish_pointcloud_registerd_world() {
        auto pointcloud_use = enable_publish_pointcloud_dense ? pointcloud_origin_undistort
                                                              : pointcloud_downsample_undistort_imu;

        auto pointcloud_world = std::make_shared<PointCloudXYZI>(pointcloud_use->size(), 1);

        std::size_t index { 0 };
        for (auto& point : *pointcloud_world)
            point_lidar_to_world(pointcloud_use->at(index++), point);

        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*pointcloud_world, msg);
        msg.header.stamp    = get_ros_time(lidar_end_timestamp);
        msg.header.frame_id = string::slam_link;

        ros_utility.publish_pointcloud_registerd_world(msg);
    }

    void publish_pointcloud_registerd_imu() {
        auto pointcloud_size = pointcloud_origin_undistort->points.size();
        auto pointcloud_imu  = std::make_shared<PointCloudXYZI>(pointcloud_size, 1);

        std::size_t index { 0 };
        for (auto& point : *pointcloud_imu)
            point_lidar_to_imu(pointcloud_origin_undistort->at(index++), point);

        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*pointcloud_imu, msg);
        msg.header.stamp    = get_ros_time(lidar_end_timestamp);
        msg.header.frame_id = string::robot_link;

        ros_utility.publish_pointcloud_registerd_body(msg);
    }

    void publish_pointcloud_effect_world() {
        auto pointcloud_effect = std::make_shared<PointCloudXYZI>(pointcloud_effect_size, 1);

        std::size_t index { 0 };
        for (const auto& point : *origin_constructed_map)
            point_lidar_to_world(point, pointcloud_effect->points[index++]);

        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*pointcloud_effect, msg);
        msg.header.stamp    = get_ros_time(lidar_end_timestamp);
        msg.header.frame_id = string::slam_link;

        ros_utility.publish_pointcloud_effect_world(msg);
    }

    void publish_constructed_map() {
        const auto pointcloud_use = enable_publish_pointcloud_dense
            ? pointcloud_origin_undistort
            : pointcloud_downsample_undistort_imu;
        if (pointcloud_use->empty()) return;

        const auto pointcloud_size      = pointcloud_use->points.size();
        const auto constructed_map_part = std::make_shared<PointCloudXYZI>(pointcloud_size, 1);

        std::size_t index { 0 };
        for (auto& point : *constructed_map_part)
            point_lidar_to_world(pointcloud_use->at(index++), point);

        *constructed_map_pointcloud += *constructed_map_part;

        // 对即将发布的点云进行降采样，你也不想你的 foxglove / rviz 爆掉吧
        auto filtered_cloud = std::make_shared<pcl::PointCloud<PointT>>();
        static pcl::VoxelGrid<PointT> filter;
        filter.setLeafSize(0.2f, 0.2f, 0.2f);
        filter.setInputCloud(constructed_map_pointcloud);
        filter.filter(*filtered_cloud);

        auto msg = sensor_msgs::msg::PointCloud2 {};
        pcl::toROSMsg(*filtered_cloud, msg);

        msg.header.stamp    = get_ros_time(lidar_end_timestamp);
        msg.header.frame_id = string::slam_link;

        ros_utility.publish_constructed_map(msg);
    }

    void publish_odometry() {
        // odometry
        current_odometry.header.frame_id = string::slam_link;
        current_odometry.child_frame_id  = string::robot_link;

        current_odometry.header.stamp = get_ros_time(lidar_end_timestamp);
        load_current_pose_stamp(current_odometry.pose);

        ros_utility.publish_odometry(current_odometry);

        auto P = extended_kalman_filter.get_P();
        for (int i = 0; i < 6; i++) {
            int k = i < 3 ? i + 3 : i - 3;

            current_odometry.pose.covariance[i * 6 + 0] = P(k, 3);
            current_odometry.pose.covariance[i * 6 + 1] = P(k, 4);
            current_odometry.pose.covariance[i * 6 + 2] = P(k, 5);
            current_odometry.pose.covariance[i * 6 + 3] = P(k, 0);
            current_odometry.pose.covariance[i * 6 + 4] = P(k, 1);
            current_odometry.pose.covariance[i * 6 + 5] = P(k, 2);
        }

        // 更新 TF 树，发布机器人位姿到 SLAM 初始点的变换
        auto t = Eigen::Translation3d(                 //
            current_odometry.pose.pose.position.x,     //
            current_odometry.pose.pose.position.y,     //
            current_odometry.pose.pose.position.z);    //
        auto r = Eigen::Quaterniond(                   //
            current_odometry.pose.pose.orientation.w,  //
            current_odometry.pose.pose.orientation.x,  //
            current_odometry.pose.pose.orientation.y,  //
            current_odometry.pose.pose.orientation.z); //
        {
            geometry_msgs::msg::TransformStamped stamp;
            stamp.header.frame_id = string::slam_link;
            stamp.child_frame_id  = string::robot_link;

            util::convert_translation(t, stamp.transform.translation);
            util::convert_orientation(r, stamp.transform.rotation);

            ros_utility.update_transform(stamp);
        }

        // publish the pose only with less data
        auto pose               = geometry_msgs::msg::PoseStamped {};
        pose.header             = current_odometry.header;
        pose.pose.position.x    = current_odometry.pose.pose.position.x;
        pose.pose.position.y    = current_odometry.pose.pose.position.y;
        pose.pose.position.z    = current_odometry.pose.pose.position.z;
        pose.pose.orientation.w = current_odometry.pose.pose.orientation.w;
        pose.pose.orientation.x = current_odometry.pose.pose.orientation.x;
        pose.pose.orientation.y = current_odometry.pose.pose.orientation.y;
        pose.pose.orientation.z = current_odometry.pose.pose.orientation.z;

        ros_utility.publish_pose(pose);
    }

    void publish_path() {
        load_current_pose_stamp(current_pose_stamped);

        current_pose_stamped.header.frame_id = string::slam_link;
        current_pose_stamped.header.stamp    = get_ros_time(lidar_end_timestamp);

        current_path.poses.push_back(current_pose_stamped);

        ros_utility.publish_path(current_path);
    }

    void load_parameter_from_config(rclcpp::Node& node) {
        const auto p = [&node](const auto& _name, const auto& _default) -> auto {
            return node.get_parameter_or<std::remove_cvref_t<decltype(_default)>>(_name, _default);
        };

        enable_publish_pointcloud_all          = p("publish.pointcloud_all", false);
        enable_publish_pointcloud_dense        = p("publish.pointcloud_dense", false);
        enable_publish_pointcloud_effect_world = p("publish.pointcloud_effect_world", false);
        enable_publish_pointcloud_imu          = p("publish.pointcloud_imu", false);
        enable_publish_path                    = p("publish.path", false);
        enable_publish_constructed_map         = p("publish.constructed_map", false);

        ekf_max_iterations        = p("max_iteration", int { 4 });
        constructed_map_save_path = p("map_file_path", std::string { "" });
        filter_size_corner_min    = p("filter_size_corner", double { 0.5 });
        filter_size_surf_min      = p("filter_size_surf", double { 0.5 });
        filter_size_map_min       = p("filter_size_map", double { 0.5 });
        cube_side_length          = p("cube_side_length", double { 200 });

        enable_timestamp_sync       = p("common.enable_timestamp_sync", false);
        timestamp_diff_lidar_to_imu = p("common.time_offset_lidar_to_imu", double { 0 });

        lidar_detection_distance      = p("mapping.lidar_detection_distance", 300.f);
        fov_degree                    = p("mapping.fov_degree", 180.f);
        covariance_gyroscope          = p("mapping.gyr_cov", 0.1);
        covariance_accelerometer      = p("mapping.acc_cov", 0.1);
        covariance_gyroscpoe_bias     = p("mapping.b_gyr_cov", 0.0001);
        covariance_accelerometer_bias = p("mapping.b_acc_cov", 0.0001);
        enable_extrinsic_estimation   = p("mapping.extrinsic_est_en", true);
        extrinsic_translation         = p("mapping.extrinsic_translation", std::vector<double> {});
        extrinsic_orientation         = p("mapping.extrinsic_orientation", std::vector<double> {});

        assert(extrinsic_translation.size() == 3);
        assert(extrinsic_orientation.size() == 9);

        preprocess->blind            = p("preprocess.blind", double { 0.01 });
        preprocess->lidar_type       = p("preprocess.lidar_type", int { AVIA });
        preprocess->N_SCANS          = p("preprocess.scan_line", int { 16 });
        preprocess->time_unit        = p("preprocess.timestamp_unit", int { US });
        preprocess->SCAN_RATE        = p("preprocess.scan_rate", int { 10 });
        preprocess->point_filter_num = p("preprocess.point_filter_num", int { 2 });
        preprocess->feature_enabled  = p("preprocess.feature_extract_enable", false);
    }

    void lasermap_fov_segment() {
        point_box_need_remove.clear();

        const auto robot_pose = current_robot_pose;
        if (!local_map_initialized) {
            for (int i = 0; i < 3; i++) {
                local_map_point_box.vertex_min[i] = float(robot_pose(i) - cube_side_length / 2.0);
                local_map_point_box.vertex_max[i] = float(robot_pose(i) + cube_side_length / 2.0);
            }
            local_map_initialized = true;
            return;
        }

        float dist_to_map_edge[3][2];
        bool need_move = false;

        for (int i = 0; i < 3; i++) {
            dist_to_map_edge[i][0] = float(fabs(robot_pose(i) - local_map_point_box.vertex_min[i]));
            dist_to_map_edge[i][1] = float(fabs(robot_pose(i) - local_map_point_box.vertex_max[i]));
            if (dist_to_map_edge[i][0] <= mov_threshold * lidar_detection_distance
                || dist_to_map_edge[i][1] <= mov_threshold * lidar_detection_distance)
                need_move = true;
        }

        if (!need_move) return;

        BoxPointType new_local_map_points;
        BoxPointType temp_box_points;

        new_local_map_points = local_map_point_box;

        const auto mov_dist = std::max(
            float((cube_side_length - 2.0 * mov_threshold * lidar_detection_distance) * 0.5 * 0.9),
            float(lidar_detection_distance * (mov_threshold - 1)));

        for (int i = 0; i < 3; i++) {
            temp_box_points = local_map_point_box;

            if (dist_to_map_edge[i][0] <= mov_threshold * lidar_detection_distance) {
                new_local_map_points.vertex_max[i] -= mov_dist;
                new_local_map_points.vertex_min[i] -= mov_dist;
                temp_box_points.vertex_min[i] = local_map_point_box.vertex_max[i] - mov_dist;
                point_box_need_remove.push_back(temp_box_points);

            } else if (dist_to_map_edge[i][1] <= mov_threshold * lidar_detection_distance) {
                new_local_map_points.vertex_max[i] += mov_dist;
                new_local_map_points.vertex_min[i] += mov_dist;
                temp_box_points.vertex_max[i] = local_map_point_box.vertex_min[i] + mov_dist;
                point_box_need_remove.push_back(temp_box_points);
            }
        }

        local_map_point_box = new_local_map_points;

        PointVector points_history;
        ikdtree.acquire_removed_points(points_history);

        if (point_box_need_remove.size() > 0) ikdtree.Delete_Point_Boxes(point_box_need_remove);
    }

    bool bind_sensor_packages(MeasureGroup& meas) {
        static double lidar_mean_scan_time = 0.0;
        static int scan_number             = 0;

        if (lidar_buffer.empty() || imu_buffer.empty()) {
            return false;
        }

        // push a lidar scan
        if (!lidar_pushed) {
            meas.lidar          = lidar_buffer.front();
            meas.lidar_beg_time = timestamp_buffer.front();

            // time too little
            if (meas.lidar->points.size() <= 1) {
                lidar_end_timestamp = meas.lidar_beg_time + lidar_mean_scan_time;
                rclcpp_warn("the size of lidar frame is not enough!");
            } else if (meas.lidar->points.back().curvature / double(1000)
                < 0.5 * lidar_mean_scan_time) {
                lidar_end_timestamp = meas.lidar_beg_time + lidar_mean_scan_time;
            } else {
                scan_number++;
                lidar_end_timestamp =
                    meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
                lidar_mean_scan_time +=
                    (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scan_time)
                    / scan_number;
            }

            meas.lidar_end_time = lidar_end_timestamp;

            lidar_pushed = true;
        }

        if (last_timestamp_imu < lidar_end_timestamp) {
            return false;
        }

        // push imu data, and pop from imu buffer
        double imu_time = get_time_sec(imu_buffer.front()->header.stamp);
        meas.imu.clear();
        while ((!imu_buffer.empty()) && (imu_time < lidar_end_timestamp)) {
            imu_time = get_time_sec(imu_buffer.front()->header.stamp);
            if (imu_time > lidar_end_timestamp) break;
            meas.imu.push_back(imu_buffer.front());
            imu_buffer.pop_front();
        }

        lidar_buffer.pop_front();
        timestamp_buffer.pop_front();
        lidar_pushed = false;
        return true;
    }

    void map_incremental() {
        PointVector PointToAdd;
        PointToAdd.reserve(feats_downsample_size);

        PointVector PointNoNeedDownsample;
        PointNoNeedDownsample.reserve(feats_downsample_size);

        for (int i = 0; i < feats_downsample_size; i++) {
            /* transform to world frame */
            point_lidar_to_world(pointcloud_downsample_undistort_imu->points[i],
                pointcloud_downsample_world->points[i]);

            /* decide if need add to map */
            if (!nearest_points[i].empty() && ekf_initialized) {

                const auto& points_near = nearest_points[i];

                bool need_add = true;

                BoxPointType box_of_point;
                PointT downsample_result;
                PointT mid_point;

                mid_point.x = static_cast<float>(
                    floor(pointcloud_downsample_world->points[i].x / filter_size_map_min)
                        * filter_size_map_min
                    + 0.5 * filter_size_map_min);
                mid_point.y = static_cast<float>(
                    floor(pointcloud_downsample_world->points[i].y / filter_size_map_min)
                        * filter_size_map_min
                    + 0.5 * filter_size_map_min);
                mid_point.z = static_cast<float>(
                    floor(pointcloud_downsample_world->points[i].z / filter_size_map_min)
                        * filter_size_map_min
                    + 0.5 * filter_size_map_min);

                float dist = calc_dist(pointcloud_downsample_world->points[i], mid_point);

                if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min
                    && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min
                    && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min) {

                    PointNoNeedDownsample.push_back(pointcloud_downsample_world->points[i]);
                    continue;
                }

                for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++) {
                    if (points_near.size() < NUM_MATCH_POINTS) break;
                    if (calc_dist(points_near[readd_i], mid_point) < dist) {
                        need_add = false;
                        break;
                    }
                }

                if (need_add) PointToAdd.push_back(pointcloud_downsample_world->points[i]);

            } else {
                PointToAdd.push_back(pointcloud_downsample_world->points[i]);
            }
        }

        ikdtree.Add_Points(PointToAdd, true);
        ikdtree.Add_Points(PointNoNeedDownsample, false);
    }

    void save_to_pcd() {
        pcl::PCDWriter pcd_writer;
        pcd_writer.writeBinary(constructed_map_save_path, *constructed_map_pointcloud);
    }

    void h_share_model(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data) {
        double match_start = omp_get_wtime();
        origin_constructed_map->clear();
        normal_vector_correspondence->clear();

#pragma omp parallel for default(none) shared(s) shared(ekfom_data) num_threads(MP_PROC_NUM)
        // closest surface search and residual computation
        for (int i = 0; i < feats_downsample_size; i++) {
            auto& point_body  = pointcloud_downsample_undistort_imu->points[i];
            auto& point_world = pointcloud_downsample_world->points[i];

            /* transform to world frame */
            V3D p_body(point_body.x, point_body.y, point_body.z);
            V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);

            point_world.x = static_cast<float>(p_global(0));
            point_world.y = static_cast<float>(p_global(1));
            point_world.z = static_cast<float>(p_global(2));

            point_world.intensity = point_body.intensity;

            std::vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

            auto& points_near = nearest_points[i];

            if (ekfom_data.converge) {
                /** Find the closest surfaces in the map **/
                ikdtree.Nearest_Search(
                    point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);

                auto distance_flag = (pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5) ? false : true;

                points_selected_surf[i] = ((points_near.size() < NUM_MATCH_POINTS)
                                                  ? false
                                                  : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5)
                    ? false
                    : true;
            }

            if (!points_selected_surf[i]) continue;

            points_selected_surf[i] = false;

            auto plane = Eigen::Matrix<float, 4, 1> {};
            if (esti_plane(plane, points_near, 0.1f)) {
                float distance2 = plane(0) * point_world.x + plane(1) * point_world.y
                    + plane(2) * point_world.z + plane(3);
                auto score = 1.f - 0.9 * std::abs(distance2) / std::sqrt(p_body.norm());
                if (score > 0.9) {
                    points_selected_surf[i]            = true;
                    normal_vector->points[i].x         = plane(0);
                    normal_vector->points[i].y         = plane(1);
                    normal_vector->points[i].z         = plane(2);
                    normal_vector->points[i].intensity = distance2;
                    residual_last[i]                   = std::abs(distance2);
                }
            }
        }

        pointcloud_effect_size = 0;

        for (int i = 0; i < feats_downsample_size; i++) {
            if (points_selected_surf[i]) {
                origin_constructed_map->points[pointcloud_effect_size] =
                    pointcloud_downsample_undistort_imu->points[i];
                normal_vector_correspondence->points[pointcloud_effect_size] =
                    normal_vector->points[i];
                pointcloud_effect_size++;
            }
        }

        if (pointcloud_effect_size < 1) {
            ekfom_data.valid = false;
            rclcpp_error("no effective points");
            return;
        }

        /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
        ekfom_data.h_x = Eigen::MatrixXd::Zero(pointcloud_effect_size, 12); // 23
        ekfom_data.h.resize(pointcloud_effect_size);

        for (int i = 0; i < pointcloud_effect_size; i++) {
            const auto& laser_p = origin_constructed_map->points[i];
            V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
            M3D point_be_crossmat;
            point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
            V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
            M3D point_crossmat;
            point_crossmat << SKEW_SYM_MATRX(point_this);

            /*** get the normal vector of closest surface/corner ***/
            const auto& norm_p = normal_vector_correspondence->points[i];
            V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

            /*** calculate the Measuremnt Jacobian matrix H ***/
            V3D C(s.rot.conjugate() * norm_vec);
            V3D A(point_crossmat * C);
            if (enable_extrinsic_estimation) {
                V3D B(point_be_crossmat * s.offset_R_L_I.conjugate()
                    * C); // s.rot.conjugate()*norm_vec);
                ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z,
                    VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
            } else {
                ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z,
                    VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            }

            /*** Measuremnt: distance to the closest surface/corner ***/
            ekfom_data.h(i) = -norm_p.intensity;
        }
    }

    void reset_slam(rclcpp::Node& node) {
        rclcpp_info("reset slam process now");

        preprocess  = std::make_unique<Preprocess>();
        imu_process = std::make_unique<ImuProcess>();

        ros_utility.stop_service();

        current_path         = nav_msgs::msg::Path {};
        current_odometry     = nav_msgs::msg::Odometry {};
        current_orientation  = geometry_msgs::msg::Quaternion {};
        current_pose_stamped = geometry_msgs::msg::PoseStamped {};

        ///////// PROCESS /////////
        lidar_pushed                = false;
        ekf_initialized             = false;
        first_update                = true;
        first_receive_lidar         = true;
        enable_timestamp_sync       = false;
        enable_extrinsic_estimation = true;
        set_timestamp_diff          = false;
        local_map_initialized       = false;

        std::fill(points_selected_surf.data(), &points_selected_surf[100000 - 1], false);
        std::fill(ekf_limit_array.begin(), ekf_limit_array.end(), 0.001);
        std::fill(residual_last.data(), &residual_last[100000 - 1], 0);

        pointcloud_effect_size = 0;

        lidar_detection_distance    = 300.0f;
        timestamp_diff_lidar_to_imu = 0.0;

        last_timestamp_lidar = 0;
        last_timestamp_imu   = -1.0;

        covariance_gyroscope          = 0.1;
        covariance_accelerometer      = 0.1;
        covariance_gyroscpoe_bias     = 0.0001;
        covariance_accelerometer_bias = 0.0001;

        filter_size_corner_min = 0;
        filter_size_surf_min   = 0;
        filter_size_map_min    = 0;
        fov_degree             = 0;

        cube_side_length      = 0;
        half_fov_cos          = 0;
        fov_degree_calculated = 0;
        lidar_end_timestamp   = 0;
        first_lidar_timetamp  = 0;

        time_diff_lidar_wrt_imu = 0.0;

        feats_downsample_size = 0;
        ekf_max_iterations    = 0;

        point_box_need_remove.clear();
        nearest_points.clear();
        extrinsic_translation.clear();
        extrinsic_orientation.clear();
        timestamp_buffer.clear();
        lidar_buffer.clear();
        imu_buffer.clear();

        pointcloud_origin_undistort->clear();
        pointcloud_downsample_undistort_imu->clear();
        pointcloud_downsample_world->clear();
        normal_vector->clear();
        origin_constructed_map->clear();
        normal_vector_correspondence->clear();
        constructed_map_pointcloud->clear();

        local_map_point_box = BoxPointType {};

        ikdtree.~KD_TREE();
        new (&ikdtree) KD_TREE<pcl::PointXYZINormal> {};

        current_sensor_package = MeasureGroup {};
        current_robot_pose     = MTK::vect<3, double> {};
        extended_kalman_filter = esekfom::esekf<state_ikfom, 12, input_ikfom> {};
        current_ekf_prediction = state_ikfom {};

        ros_utility.start_service();

        rclcpp_info("parameters have reset");

        initialize(node);

        rclcpp_info("reconstruct slam context successfully");
    }

    void lidar_subscription_callback(const livox_ros_driver2::msg::CustomMsg::UniquePtr& msg) {

        global_mutex.lock();
        double cur_time              = get_time_sec(msg->header.stamp);
        double preprocess_start_time = omp_get_wtime();
        if (!first_receive_lidar && cur_time < last_timestamp_lidar) {
            rclcpp_info("雷达时间戳出现倒退，清空缓存区");
            lidar_buffer.clear();
        }
        if (first_receive_lidar) {
            first_receive_lidar = false;
        }
        last_timestamp_lidar = cur_time;

        if (!enable_timestamp_sync && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0
            && !imu_buffer.empty() && !lidar_buffer.empty()) {
            rclcpp_warn("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",
                last_timestamp_imu, last_timestamp_lidar);
        }

        if (enable_timestamp_sync && !set_timestamp_diff
            && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty()) {
            set_timestamp_diff      = true;
            time_diff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
            rclcpp_warn("Self sync IMU and LiDAR, time diff is %.10lf \n", time_diff_lidar_wrt_imu);
        }

        auto pointcloud = std::make_shared<PointCloudXYZI>();
        preprocess->process(msg, pointcloud);
        lidar_buffer.push_back(pointcloud);
        timestamp_buffer.push_back(last_timestamp_lidar);

        global_mutex.unlock();
        global_signal.notify_all();
    }

    void imu_subscription_callback(const sensor_msgs::msg::Imu::UniquePtr& msg_in) {
        auto msg = std::make_shared<sensor_msgs::msg::Imu>(*msg_in);

        msg->header.stamp =
            get_ros_time(get_time_sec(msg_in->header.stamp) - timestamp_diff_lidar_to_imu);
        if (std::abs(time_diff_lidar_wrt_imu) > 0.1 && enable_timestamp_sync) {
            msg->header.stamp = rclcpp::Time( //
                int64_t(time_diff_lidar_wrt_imu + get_time_sec(msg_in->header.stamp)));
        }

        auto timestamp = get_time_sec(msg->header.stamp);

        global_mutex.lock();

        if (timestamp < last_timestamp_imu) {
            rclcpp_warn("IMU消息时间戳出现倒退，清空缓存区");
            imu_buffer.clear();
        }

        last_timestamp_imu = timestamp;

        imu_buffer.push_back(msg);
        global_mutex.unlock();
        global_signal.notify_all();
    }

#pragma clang diagnostic ignored "-Wformat-security"
    std::shared_ptr<rclcpp::Logger> logger;

    template <typename... Args> void rclcpp_info(Args... args) {
        RCLCPP_INFO(*logger, std::forward<Args>(args)...);
    }
    template <typename... Args> void rclcpp_warn(Args... args) {
        RCLCPP_WARN(*logger, std::forward<Args>(args)...);
    }
    template <typename... Args> void rclcpp_error(Args... args) {
        RCLCPP_ERROR(*logger, std::forward<Args>(args)...);
    }

    // 双激光雷达同步器，负责同步两者变换
    Synthesizer synthesizer {};

    // ROS2 相关接口的包装器
    RosUtil ros_utility {};

    // IKD 树
    KD_TREE<PointT> ikdtree;

    std::unique_ptr<Preprocess> preprocess { std::make_unique<Preprocess>() };

    std::unique_ptr<ImuProcess> imu_process { std::make_unique<ImuProcess>() };

    pcl::VoxelGrid<PointT> downsample_scan_filter;

    std::unique_ptr<tf2_ros::TransformBroadcaster> transfrom_boardcaster;

    // some switch to change mode
    bool enable_publish_pointcloud_effect_world = false;
    bool enable_publish_constructed_map         = false;
    bool enable_publish_pointcloud_all          = false;
    bool enable_publish_pointcloud_dense        = false;
    bool enable_publish_pointcloud_imu          = false;
    bool enable_publish_path                    = false;

    bool enable_timestamp_sync       = false;
    bool enable_extrinsic_estimation = true;

    nav_msgs::msg::Path current_path;
    nav_msgs::msg::Odometry current_odometry;
    geometry_msgs::msg::Quaternion current_orientation;
    geometry_msgs::msg::PoseStamped current_pose_stamped;

    // 和配置相关的参数
    std::string constructed_map_save_path;
    double mov_threshold                 = 1.5f;
    double lidar_detection_distance      = 300.0f;
    double timestamp_diff_lidar_to_imu   = 0.0;
    double covariance_gyroscope          = 0.1;
    double covariance_accelerometer      = 0.1;
    double covariance_gyroscpoe_bias     = 0.0001;
    double covariance_accelerometer_bias = 0.0001;
    double filter_size_corner_min        = 0;
    double filter_size_surf_min          = 0;
    double filter_size_map_min           = 0;
    double fov_degree                    = 0;
    double cube_side_length              = 0;
    double half_fov_cos                  = 0;
    double fov_degree_calculated         = 0;

    int ekf_max_iterations = 0;

    std::vector<double> extrinsic_translation;
    std::vector<double> extrinsic_orientation;

    // 和过程相关标志位性质的参数
    bool lidar_pushed          = false;
    bool ekf_initialized       = false;
    bool first_update          = true;
    bool first_receive_lidar   = true;
    bool set_timestamp_diff    = false;
    bool local_map_initialized = false;

    double last_timestamp_lidar = 0;
    double last_timestamp_imu   = -1.0;

    long pointcloud_effect_size = 0;

    double lidar_end_timestamp  = 0;
    double first_lidar_timetamp = 0;

    double time_diff_lidar_wrt_imu = 0.0;

    // 参与过程迭代的参数
    MeasureGroup current_sensor_package;

    esekfom::esekf<state_ikfom, 12, input_ikfom> extended_kalman_filter;

    state_ikfom current_ekf_prediction;

    std::array<double, 23> ekf_limit_array;

    MTK::vect<3, double> current_robot_pose;

    std::array<bool, 1'000'000> points_selected_surf;
    std::array<float, 1'000'000> residual_last;

    std::size_t feats_downsample_size = 0;

    std::vector<BoxPointType> point_box_need_remove;
    std::deque<double> timestamp_buffer;
    std::deque<PointCloudXYZI::Ptr> lidar_buffer;
    std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;

    std::shared_ptr<PointCloudXYZI> pointcloud_origin_undistort;
    std::shared_ptr<PointCloudXYZI> pointcloud_downsample_undistort_imu;
    std::shared_ptr<PointCloudXYZI> pointcloud_downsample_world;
    std::shared_ptr<PointCloudXYZI> normal_vector;
    std::shared_ptr<PointCloudXYZI> normal_vector_correspondence;
    std::shared_ptr<PointCloudXYZI> origin_constructed_map;
    std::shared_ptr<PointCloudXYZI> constructed_map_pointcloud;

    BoxPointType local_map_point_box;

    // TODO: 这个变量好像看上去挺重要，为啥没被赋值
    std::vector<PointVector> nearest_points;
};

SLAM::SLAM()
    : Node("rmcs_slam", util::NodeOptions {})
    , pimpl(std::make_unique<Impl>()) {
    pimpl->initialize(*this);
}

SLAM::~SLAM() = default;
