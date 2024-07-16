#pragma once

#include "ikd_tree/ikd_tree.hpp"
#include "process/imu_processing.hpp"
#include "process/preprocess.hpp"

#include <Python.h>
#include <cmath>
#include <csignal>
#include <fstream>
#include <memory>
#include <mutex>
#include <omp.h>
#include <unistd.h>
#include <utility>

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
#include <visualization_msgs/msg/marker.hpp>

#include <Eigen/Core>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>

// some definitions
#define INIT_TIME       (0.1)
#define LASER_POINT_COV (0.001)
#define MAXN            (720000)
#define PUBFRAME_PERIOD (20)

// let a static c style function pointer accesses the no-static member function
// fuck the code style of fast_lio
using ProcessCallback = std::function<void(state_ikfom&, esekfom::dyn_share_datastruct<double>&)>;
class FuncHelperClass {
    static inline ProcessCallback object;

public:
    static void static_function(state_ikfom& a, esekfom::dyn_share_datastruct<double>& b)
    {
        return object(a, b);
    }
    static void bind(ProcessCallback callback)
    {
        object = std::move(callback);
    }
};

// global parameter
static inline std::mutex global_mutex;
static inline std::condition_variable global_signal;

// class SLAM
class SLAM : public rclcpp::Node {
public:
    explicit SLAM();
    ~SLAM();

private:
    ////////// ROS2 INTERFACE //////////
    // publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_registered_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_registered_body_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_effected_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr laser_map_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr position_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

    // subscription
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr standard_subscription_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_subscription_;

    // service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_trigger_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_trigger_;

    // tf
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

    // timer
    rclcpp::TimerBase::SharedPtr main_process_timer_;
    rclcpp::TimerBase::SharedPtr map_publisher_timer_;

    // some switch to change mode
    bool pointcloud2_       = false;
    bool publish_effect_    = false;
    bool publish_map_       = false;
    bool publish_scan_      = false;
    bool publish_dense_     = false;
    bool publish_scan_body_ = false;
    bool publish_path_      = true;

    int publish_path_count_ = 0;

    std::string lidar_topic_;
    std::string imu_topic_;

    nav_msgs::msg::Path path_;
    nav_msgs::msg::Odometry odom_after_mapped_;
    geometry_msgs::msg::Quaternion geo_quat_;
    geometry_msgs::msg::PoseStamped msg_body_pose_;

    void map_save_callback(const std_srvs::srv::Trigger::Request::ConstSharedPtr& req, const std_srvs::srv::Trigger::Response::SharedPtr& res);
    void standard_subscription_callback(const sensor_msgs::msg::PointCloud2::UniquePtr& msg);
    void livox_subscription_callback(const livox_ros_driver2::msg::CustomMsg::UniquePtr& msg);
    void imu_subscription_callback(const sensor_msgs::msg::Imu::UniquePtr& msg_in);

    void map_publish_timer_callback();

    void publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher);
    void publish_frame_body(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher);
    void publish_effect_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher);
    void publish_map(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher);
    void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& publisher);
    void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr& publisher, const std::unique_ptr<tf2_ros::TransformBroadcaster>& broadcaster);

    ///////// PROCESS /////////
    std::string root_dir = ROOT_DIR;
    std::string map_save_path_;

    bool lidar_pushed_    = false;
    bool ekf_init_        = false;
    bool first_scan_      = true;
    bool first_lidar_     = true;
    bool runtime_pos_log_ = false;
    bool save_pcd_        = false;
    bool time_sync_       = false;
    bool extrinsic_est_   = true;
    bool time_diff_set_   = false;
    bool local_map_init   = false;

    bool point_selected_surf_[100000] = { false };

    int effect_feat_number_ = 0;
    int frame_number_       = 0;

    double average_time_consu_        = 0;
    double average_time_icp_          = 0;
    double average_time_match_        = 0;
    double average_time_incre_        = 0;
    double average_time_solve_        = 0;
    double average_time_const_h_time_ = 0;

    double epsi_[23] = { 0.001 };

    int kdtree_size_st_      = 0;
    int kdtree_size_end_     = 0;
    int add_point_size_      = 0;
    int kdtree_delete_count_ = 0;

    float res_last_[100000];
    float det_range_               = 300.0f;
    const float mov_threshold_     = 1.5f;
    double time_diff_lidar_to_imu_ = 0.0;

    double res_mean_last_  = 0.05;
    double total_residual_ = 0.0;

    double last_timestamp_lidar_ = 0;
    double last_timestamp_imu_   = -1.0;

    double gyr_cov_   = 0.1;
    double acc_cov_   = 0.1;
    double b_gyr_cov_ = 0.0001;
    double b_acc_cov_ = 0.0001;

    double filter_size_corner_min_ = 0;
    double filter_size_surf_min_   = 0;
    double filter_size_map_min_    = 0;
    double fov_deg_                = 0;

    double cube_len_         = 0;
    double half_fov_cos_     = 0;
    double FOV_DEG_          = 0;
    double total_distance_   = 0;
    double lidar_end_time_   = 0;
    double first_lidar_time_ = 0;

    double time_diff_lidar_wrt_imu_ = 0.0;

    int time_log_count_ = 0;
    int scan_count_     = 0;
    int publish_count_  = 0;

    int iter_count_               = 0;
    int feats_down_size_          = 0;
    int number_max_iterations_    = 0;
    int laser_cloud_valid_number_ = 0;
    int pcd_save_interval_        = -1;
    int pcd_index_                = 0;

    vector<vector<int>> point_search_ind_surf_;
    vector<BoxPointType> cub_needrm_;
    vector<PointVector> nearest_points_;
    vector<double> extrinT_;
    vector<double> extrinR_;
    deque<double> time_buffer_;
    deque<PointCloudXYZI::Ptr> lidar_buffer_;
    deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer_;

    PointCloudXYZI::Ptr feats_from_map_;
    PointCloudXYZI::Ptr feats_undistort_;
    PointCloudXYZI::Ptr feats_down_body_;
    PointCloudXYZI::Ptr feats_down_world_;
    PointCloudXYZI::Ptr normvec_;
    PointCloudXYZI::Ptr laser_cloud_ori_;
    PointCloudXYZI::Ptr corr_normvect_;
    PointCloudXYZI::Ptr feats_array_;
    PointCloudXYZI::Ptr cloud_to_publish_;

    pcl::VoxelGrid<PointType> down_size_filter_surf_;
    pcl::VoxelGrid<PointType> down_size_filter_Map_;

    BoxPointType local_map_points_;

    KD_TREE<PointType> ikdtree_;

    V3F x_axis_point_body_;
    V3F x_axis_point_world_;
    V3D euler_cur_;
    V3D position_last_;
    V3D lidar_t_wrt_imu_;
    M3D lidar_r_wrt_imu_;

    // EKF inputs and output
    MeasureGroup measures_;
    esekfom::esekf<state_ikfom, 12, input_ikfom> kf_;
    state_ikfom state_point_;
    vect3 pos_lid_;

    std::shared_ptr<Preprocess> preprocess_;
    std::shared_ptr<ImuProcess> imu_process_;

    // log
    FILE* file_;
    ofstream fout_pre_;
    ofstream fout_out_;
    ofstream fout_dbg_;

    double kdtree_incremental_time_ = 0.0;
    double kdtree_search_time_      = 0.0;
    double kdtree_delete_time_      = 0.0;

    double t1_[MAXN];
    double s_plot1_[MAXN];
    double s_plot2_[MAXN];
    double s_plot3_[MAXN];
    double s_plot4_[MAXN];
    double s_plot5_[MAXN];
    double s_plot6_[MAXN];
    double s_plot7_[MAXN];
    double s_plot8_[MAXN];
    double s_plot9_[MAXN];
    double s_plot10_[MAXN];
    double s_plot11_[MAXN];

    double match_time_         = 0;
    double solve_time_         = 0;
    double solve_const_h_time_ = 0;

    void load_parameter();
    void initialize();

    void main_process_timer_callback();
    void reset_trigger_callback();

    void lasermap_fov_segment();
    bool sync_packages(MeasureGroup& meas);
    void map_incremental();
    void save_to_pcd();
    void h_share_model(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data);

    ////////// TOOL //////////
    static void signal_handle(int signal);
    static void point_body_to_world_ikfom(PointType const* in, PointType* out, state_ikfom& state);
    void dump_lio_state_to_log(FILE* file);
    void point_body_to_world(PointType const* pi, PointType* po);
    void rgb_point_body_to_world(PointType const* pi, PointType* po);
    void rgb_point_body_lidar_to_imu(PointType const* pi, PointType* po);
    void points_cache_collect();

    template <typename T>
    void point_body_to_world(const Matrix<T, 3, 1>& pi, Matrix<T, 3, 1>& po)
    {
        V3D p_body(pi[0], pi[1], pi[2]);
        V3D p_global(state_point_.rot * (state_point_.offset_R_L_I * p_body + state_point_.offset_T_L_I) + state_point_.pos);

        po[0] = p_global(0);
        po[1] = p_global(1);
        po[2] = p_global(2);
    }

    template <typename T>
    void set_pose_stamp(T& stamp)
    {
        stamp.pose.position.x    = state_point_.pos(0);
        stamp.pose.position.y    = state_point_.pos(1);
        stamp.pose.position.z    = state_point_.pos(2);
        stamp.pose.orientation.x = geo_quat_.x;
        stamp.pose.orientation.y = geo_quat_.y;
        stamp.pose.orientation.z = geo_quat_.z;
        stamp.pose.orientation.w = geo_quat_.w;
    }

    void hello_world();
};
