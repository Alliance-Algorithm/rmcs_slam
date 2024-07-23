#include "slam.hpp"

void SLAM::load_parameter()
{
    extrinT_ = vector<double>(3, 0.0);
    extrinR_ = vector<double>(9, 0.0);

    feats_from_map_ = std::make_shared<PointCloudXYZI>();
    feats_undistort_ = std::make_shared<PointCloudXYZI>();
    feats_down_body_ = std::make_shared<PointCloudXYZI>();
    feats_down_world_ = std::make_shared<PointCloudXYZI>();
    cloud_to_publish_ = std::make_shared<PointCloudXYZI>();
    normvec_ = std::make_shared<PointCloudXYZI>(100000, 1);
    laser_cloud_ori_ = std::make_shared<PointCloudXYZI>(100000, 1);
    corr_normvect_ = std::make_shared<PointCloudXYZI>(100000, 1);

    x_axis_point_body_ = V3F(LIDAR_SP_LEN, 0.0, 0.0);
    x_axis_point_world_ = V3F(LIDAR_SP_LEN, 0.0, 0.0);
    euler_cur_ = V3D();
    position_last_ = V3D(Zero3d);
    lidar_t_wrt_imu_ = V3D(Zero3d);
    lidar_r_wrt_imu_ = M3D(Eye3d);

    preprocess_ = std::make_shared<Preprocess>();
    imu_process_ = std::make_shared<ImuProcess>();

    // get parameter
    get_parameter_or<bool>("publish.path_en", publish_path_, true);
    get_parameter_or<bool>("publish.effect_map_en", publish_effect_, false);
    get_parameter_or<bool>("publish.map_en", publish_map_, false);
    get_parameter_or<bool>("publish.scan_publish_en", publish_scan_, true);
    get_parameter_or<bool>("publish.dense_publish_en", publish_dense_, true);
    get_parameter_or<bool>("publish.scan_bodyframe_pub_en", publish_scan_body_, true);

    get_parameter_or<int>("max_iteration", number_max_iterations_, 4);
    get_parameter_or<std::string>("map_file_path", map_save_path_, "");

    get_parameter_or<std::string>("common.lid_topic", lidar_topic_, "/livox/lidar");
    get_parameter_or<std::string>("common.imu_topic", imu_topic_, "/livox/imu");
    get_parameter_or<bool>("common.time_sync_en", time_sync_, false);
    get_parameter_or<double>("common.time_offset_lidar_to_imu", time_diff_lidar_to_imu_, 0.0);
    get_parameter_or<bool>("common.pointcloud2", pointcloud2_, false);

    get_parameter_or<double>("filter_size_corner", filter_size_corner_min_, 0.5);
    get_parameter_or<double>("filter_size_surf", filter_size_surf_min_, 0.5);
    get_parameter_or<double>("filter_size_map", filter_size_map_min_, 0.5);
    get_parameter_or<double>("cube_side_length", cube_len_, 200.f);

    get_parameter_or<float>("mapping.det_range", det_range_, 300.f);
    get_parameter_or<double>("mapping.fov_degree", fov_deg_, 180.f);
    get_parameter_or<double>("mapping.gyr_cov", gyr_cov_, 0.1);
    get_parameter_or<double>("mapping.acc_cov", acc_cov_, 0.1);
    get_parameter_or<double>("mapping.b_gyr_cov", b_gyr_cov_, 0.0001);
    get_parameter_or<double>("mapping.b_acc_cov", b_acc_cov_, 0.0001);
    get_parameter_or<bool>("mapping.extrinsic_est_en", extrinsic_est_, true);
    get_parameter_or<vector<double>>("mapping.extrinsic_T", extrinT_, vector<double>());
    get_parameter_or<vector<double>>("mapping.extrinsic_R", extrinR_, vector<double>());

    get_parameter_or<double>("preprocess.blind", preprocess_->blind, 0.01);
    get_parameter_or<int>("preprocess.lidar_type", preprocess_->lidar_type, AVIA);
    get_parameter_or<int>("preprocess.scan_line", preprocess_->N_SCANS, 16);
    get_parameter_or<int>("preprocess.timestamp_unit", preprocess_->time_unit, US);
    get_parameter_or<int>("preprocess.scan_rate", preprocess_->SCAN_RATE, 10);

    get_parameter_or<int>("point_filter_num", preprocess_->point_filter_num, 2);
    get_parameter_or<bool>("feature_extract_enable", preprocess_->feature_enabled, false);
    get_parameter_or<bool>("runtime_pos_log_enable", runtime_pos_log_, 0);

    get_parameter_or<bool>("pcd_save.pcd_save_en", save_pcd_, false);
    get_parameter_or<int>("pcd_save.interval", pcd_save_interval_, -1);

    RCLCPP_INFO(get_logger(), "lidar_type %d", preprocess_->lidar_type);
}

void SLAM::initialize()
{
    path_.header.stamp = get_clock()->now();
    path_.header.frame_id = "lidar_init";

    FOV_DEG_ = (fov_deg_ + 10.0) > 179.9 ? 179.9 : (fov_deg_ + 10.0);
    half_fov_cos_ = cos((FOV_DEG_) * 0.5 * PI_M / 180.0);

    feats_array_ = std::make_shared<PointCloudXYZI>();

    memset(point_selected_surf_, true, sizeof(point_selected_surf_));
    memset(res_last_, -1000.0f, sizeof(res_last_));

    down_size_filter_surf_.setLeafSize(float(filter_size_surf_min_), float(filter_size_surf_min_), float(filter_size_surf_min_));
    down_size_filter_Map_.setLeafSize(float(filter_size_map_min_), float(filter_size_map_min_), float(filter_size_map_min_));

    memset(point_selected_surf_, true, sizeof(point_selected_surf_));
    memset(res_last_, -1000.0f, sizeof(res_last_));

    lidar_t_wrt_imu_ << VEC_FROM_ARRAY(extrinT_);
    lidar_r_wrt_imu_ << MAT_FROM_ARRAY(extrinR_);

    imu_process_->set_extrinsic(lidar_t_wrt_imu_, lidar_r_wrt_imu_);
    imu_process_->set_gyr_cov(V3D(gyr_cov_, gyr_cov_, gyr_cov_));
    imu_process_->set_acc_cov(V3D(acc_cov_, acc_cov_, acc_cov_));
    imu_process_->set_gyr_bias_cov(V3D(b_gyr_cov_, b_gyr_cov_, b_gyr_cov_));
    imu_process_->set_acc_bias_cov(V3D(b_acc_cov_, b_acc_cov_, b_acc_cov_));

    fill(epsi_, epsi_ + 23, 0.001);

    FuncHelperClass::bind([this](state_ikfom& a, esekfom::dyn_share_datastruct<double>& b) { h_share_model(a, b); });
    kf_.init_dyn_share(get_f, df_dx, df_dw, FuncHelperClass::static_function, number_max_iterations_, epsi_);
}

SLAM::SLAM()
    : Node("rmcs_slam", rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true))
{
    signal(SIGINT, signal_handle);

    load_parameter();

    initialize();

    // debug
    std::string pos_log_dir = root_dir + "/log/pos_log.txt";

    file_ = fopen(pos_log_dir.c_str(), "w");

    // ofstream fout_pre, fout_out, fout_dbg;s
    fout_pre_.open(DEBUG_FILE_DIR("mat_pre.txt"), ios::out);
    fout_out_.open(DEBUG_FILE_DIR("mat_out.txt"), ios::out);
    fout_dbg_.open(DEBUG_FILE_DIR("dbg.txt"), ios::out);

    if (fout_pre_ && fout_out_)
        RCLCPP_INFO(get_logger(), "log file opened at: %s", ROOT_DIR);
    else
        RCLCPP_INFO(get_logger(), "log file doesn't exist at %s", ROOT_DIR);

    // ros2 interface
    if (preprocess_->lidar_type == AVIA && !pointcloud2_) {
        livox_subscription_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
            lidar_topic_, 20, [this](const livox_ros_driver2::msg::CustomMsg::UniquePtr& msg) {
                livox_subscription_callback(msg);
            });
    } else {
        standard_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic_, rclcpp::SensorDataQoS(), [this](const sensor_msgs::msg::PointCloud2::UniquePtr& msg) {
                standard_subscription_callback(msg);
            });
    }

    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, 10, [this](const sensor_msgs::msg::Imu::UniquePtr& msg) {
            imu_subscription_callback(msg);
        });

    cloud_registered_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/rmcs_slam/cloud_registered", 20);
    cloud_registered_body_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/rmcs_slam/cloud_registered_body", 20);
    cloud_effected_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/rmcs_slam/cloud_effected", 20);
    laser_map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("/rmcs_slam/laser_map", 20);
    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("/rmcs_slam/pose", 20);
    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/rmcs_slam/odometry", 20);
    path_publisher_ = create_publisher<nav_msgs::msg::Path>("/rmcs_slam/path", 20);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    using namespace std::chrono_literals;
    main_process_timer_ = rclcpp::create_timer(this, get_clock(), 10ms, [this] { main_process_timer_callback(); });
    map_publisher_timer_ = rclcpp::create_timer(this, get_clock(), 1s, [this] { map_publish_timer_callback(); });

    map_save_trigger_ = create_service<std_srvs::srv::Trigger>(
        "/rmcs_slam/map_save",
        [this](const std_srvs::srv::Trigger::Request::ConstSharedPtr& p1, const std_srvs::srv::Trigger::Response::SharedPtr& p2) {
            RCLCPP_INFO(get_logger(), "service arrived: /rmcs_slam/map_save");
            map_save_callback(p1, p2);
        });

    reset_trigger_ = create_service<std_srvs::srv::Trigger>(
        "/rmcs_slam/reset",
        [this](const std_srvs::srv::Trigger::Request::ConstSharedPtr& p1, const std_srvs::srv::Trigger::Response::SharedPtr& p2) {
            RCLCPP_INFO(get_logger(), "service arrived: /rmcs_slam/reset");
            reset_trigger_callback();
            p2->message = "reset now";
            p2->success = true;
        });

    RCLCPP_INFO(get_logger(), "node init finished");
}

SLAM::~SLAM()
{
    if (runtime_pos_log_) {

        std::vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;

        FILE* fp2;

        std::string log_dir = root_dir + "/log/rmcs_slam_time_log.csv";

        fp2 = fopen(log_dir.c_str(), "w");

        fprintf(fp2, "time_stamp, total time, scan point size, incremental time, search time, delete size, "
                     "delete time, tree size st, tree size end, add point size, preprocess time\n");

        for (int i = 0; i < time_log_count_; i++) {

            fprintf(
                fp2, "%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",
                t1_[i], s_plot1_[i], int(s_plot2_[i]), s_plot3_[i], s_plot4_[i],
                int(s_plot5_[i]), s_plot6_[i], int(s_plot7_[i]), int(s_plot8_[i]),
                int(s_plot10_[i]), s_plot11_[i]);

            t.push_back(t1_[i]);
            s_vec.push_back(s_plot9_[i]);
            s_vec2.push_back(s_plot3_[i] + s_plot6_[i]);
            s_vec3.push_back(s_plot4_[i]);
            s_vec5.push_back(s_plot1_[i]);
        }

        fclose(fp2);
    }

    fout_out_.close();
    fout_pre_.close();
    fclose(file_);
}