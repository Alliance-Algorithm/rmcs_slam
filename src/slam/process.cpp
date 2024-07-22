#include <memory>

#include "slam.hpp"

void SLAM::main_process_timer_callback()
{
    if (sync_packages(measures_)) {

        if (first_scan_) {
            first_lidar_time_ = measures_.lidar_beg_time;
            imu_process_->first_lidar_time = first_lidar_time_;
            first_scan_ = false;
            return;
        }

        double t0;
        double t1;
        double t2;
        double t3;
        double t4;
        double t5;
        double match_start;
        double solve_start;
        double svd_time;

        match_time_ = 0;
        kdtree_search_time_ = 0.0;
        solve_time_ = 0;
        solve_const_h_time_ = 0;
        svd_time = 0;
        t0 = omp_get_wtime();

        imu_process_->process(measures_, kf_, feats_undistort_);

        state_point_ = kf_.get_x();

        pos_lid_ = state_point_.pos + state_point_.rot * state_point_.offset_T_L_I;

        if (feats_undistort_->empty() || (feats_undistort_ == nullptr)) {
            RCLCPP_WARN(this->get_logger(), "no point, skip this scan!");
            return;
        }

        ekf_init_ = (measures_.lidar_beg_time - first_lidar_time_) < INIT_TIME ? false : true;

        /*** Segment the map in lidar FOV ***/
        lasermap_fov_segment();

        /*** downsample the feature points in a scan ***/
        down_size_filter_surf_.setInputCloud(feats_undistort_);
        down_size_filter_surf_.filter(*feats_down_body_);

        t1 = omp_get_wtime();

        feats_down_size_ = feats_down_body_->points.size();

        /*** initialize the map kdtree ***/
        if (ikdtree_.Root_Node == nullptr) {

            RCLCPP_INFO(this->get_logger(), "initialize the map kdtree");

            if (feats_down_size_ > 5) {
                ikdtree_.set_downsample_param(filter_size_map_min_);
                feats_down_world_->resize(feats_down_size_);
                for (int i = 0; i < feats_down_size_; i++) {
                    point_body_to_world(
                        &(feats_down_body_->points[i]), &(feats_down_world_->points[i]));
                }

                ikdtree_.Build(feats_down_world_->points);
            }
            return;
        }
        int featsFromMapNum = ikdtree_.validnum();
        kdtree_size_st_ = ikdtree_.size();

        /*** ICP and iterated Kalman filter update ***/
        if (feats_down_size_ < 5) {
            RCLCPP_WARN(this->get_logger(), "no point, skip this scan!");
            return;
        }

        normvec_->resize(feats_down_size_);
        feats_down_world_->resize(feats_down_size_);

        V3D ext_euler = SO3ToEuler(state_point_.offset_R_L_I);

        fout_pre_ << setw(20) << measures_.lidar_beg_time - first_lidar_time_
                  << " " << euler_cur_.transpose()
                  << " " << state_point_.pos.transpose()
                  << " " << ext_euler.transpose()
                  << " " << state_point_.offset_T_L_I.transpose()
                  << " " << state_point_.vel.transpose()
                  << " " << state_point_.bg.transpose()
                  << " " << state_point_.ba.transpose()
                  << " " << state_point_.grav
                  << endl;

        // If you need to see map point, change to "if(true)"
        if (false) {
            PointVector().swap(ikdtree_.PCL_Storage);
            ikdtree_.flatten(ikdtree_.Root_Node, ikdtree_.PCL_Storage, NOT_RECORD);
            feats_from_map_->clear();
            feats_from_map_->points = ikdtree_.PCL_Storage;
        }

        point_search_ind_surf_.resize(feats_down_size_);
        nearest_points_.resize(feats_down_size_);
        int rematch_num = 0;
        bool nearest_search_en = true;

        t2 = omp_get_wtime();

        /*** iterated state estimation ***/
        double t_update_start = omp_get_wtime();
        double solve_H_time = 0;
        kf_.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
        state_point_ = kf_.get_x();
        euler_cur_ = SO3ToEuler(state_point_.rot);
        pos_lid_ = state_point_.pos + state_point_.rot * state_point_.offset_T_L_I;
        geo_quat_.x = state_point_.rot.coeffs()[0];
        geo_quat_.y = state_point_.rot.coeffs()[1];
        geo_quat_.z = state_point_.rot.coeffs()[2];
        geo_quat_.w = state_point_.rot.coeffs()[3];

        double t_update_end = omp_get_wtime();

        /******* Publish odometry *******/
        publish_odometry(odometry_publisher_, tf_broadcaster_);

        /*** add the feature points to map kdtree ***/
        t3 = omp_get_wtime();

        map_incremental();

        t5 = omp_get_wtime();

        // publish the cloud
        if (publish_path_)
            publish_path(path_publisher_);

        if (publish_scan_)
            publish_frame_world(cloud_registered_publisher_);

        if (publish_scan_ && publish_scan_body_)
            publish_frame_body(cloud_registered_body_publisher_);

        if (publish_effect_)
            publish_effect_world(cloud_effected_publisher_);

        // debug message
        if (runtime_pos_log_) {
            frame_number_++;
            kdtree_size_end_ = ikdtree_.size();

            average_time_consu_ = average_time_consu_ * (frame_number_ - 1) / frame_number_ + (t5 - t0) / frame_number_;
            average_time_icp_ = average_time_icp_ * (frame_number_ - 1) / frame_number_ + (t_update_end - t_update_start) / frame_number_;
            average_time_match_ = average_time_match_ * (frame_number_ - 1) / frame_number_ + (match_time_) / frame_number_;
            average_time_incre_ = average_time_incre_ * (frame_number_ - 1) / frame_number_ + (kdtree_incremental_time_) / frame_number_;
            average_time_solve_ = average_time_solve_ * (frame_number_ - 1) / frame_number_ + (solve_time_ + solve_H_time) / frame_number_;
            average_time_const_h_time_ = average_time_const_h_time_ * (frame_number_ - 1) / frame_number_ + solve_time_ / frame_number_;

            t1_[time_log_count_] = measures_.lidar_beg_time;

            s_plot1_[time_log_count_] = t5 - t0;
            s_plot2_[time_log_count_] = feats_undistort_->points.size();
            s_plot3_[time_log_count_] = kdtree_incremental_time_;
            s_plot4_[time_log_count_] = kdtree_search_time_;
            s_plot5_[time_log_count_] = kdtree_delete_count_;
            s_plot6_[time_log_count_] = kdtree_delete_time_;
            s_plot7_[time_log_count_] = kdtree_size_st_;
            s_plot8_[time_log_count_] = kdtree_size_end_;
            s_plot9_[time_log_count_] = average_time_consu_;
            s_plot10_[time_log_count_] = add_point_size_;
            time_log_count_++;

            printf(
                "[ mapping ]: time: "
                "IMU + Map + Input Downsample: %0.6f "
                "ave match: %0.6f ave "
                "solve: %0.6f  "
                "ave ICP: %0.6f  "
                "map incre: %0.6f "
                "ave total: %0.6f "
                "icp: %0.6f "
                "construct H: %0.6f \n",
                t1 - t0,
                average_time_match_,
                average_time_solve_,
                t3 - t1, t5 - t3,
                average_time_consu_,
                average_time_icp_,
                average_time_const_h_time_);

            ext_euler = SO3ToEuler(state_point_.offset_R_L_I);

            fout_out_ << setw(20) << measures_.lidar_beg_time - first_lidar_time_
                      << " " << euler_cur_.transpose()
                      << " " << state_point_.pos.transpose()
                      << " " << ext_euler.transpose()
                      << " " << state_point_.offset_T_L_I.transpose()
                      << " " << state_point_.vel.transpose()
                      << " " << state_point_.bg.transpose()
                      << " " << state_point_.ba.transpose()
                      << " " << state_point_.grav
                      << " " << feats_undistort_->points.size() << endl;

            dump_lio_state_to_log(file_);
        }
    }
}

void SLAM::lasermap_fov_segment()
{

    cub_needrm_.clear();
    kdtree_delete_count_ = 0;
    kdtree_delete_time_ = 0.0;

    point_body_to_world(x_axis_point_body_, x_axis_point_world_);

    V3D pos_LiD = pos_lid_;

    if (!local_map_init) {
        for (int i = 0; i < 3; i++) {
            local_map_points_.vertex_min[i] = float(pos_LiD(i) - cube_len_ / 2.0);
            local_map_points_.vertex_max[i] = float(pos_LiD(i) + cube_len_ / 2.0);
        }
        local_map_init = true;
        return;
    }

    float dist_to_map_edge[3][2];
    bool need_move = false;

    for (int i = 0; i < 3; i++) {
        dist_to_map_edge[i][0] = float(fabs(pos_LiD(i) - local_map_points_.vertex_min[i]));
        dist_to_map_edge[i][1] = float(fabs(pos_LiD(i) - local_map_points_.vertex_max[i]));
        if (dist_to_map_edge[i][0] <= mov_threshold_ * det_range_
            || dist_to_map_edge[i][1] <= mov_threshold_ * det_range_)
            need_move = true;
    }

    if (!need_move)
        return;

    BoxPointType new_local_map_points;
    BoxPointType temp_box_points;

    new_local_map_points = local_map_points_;

    float mov_dist = max(
        float((cube_len_ - 2.0 * mov_threshold_ * det_range_) * 0.5 * 0.9),
        float(det_range_ * (mov_threshold_ - 1)));

    for (int i = 0; i < 3; i++) {
        temp_box_points = local_map_points_;

        if (dist_to_map_edge[i][0] <= mov_threshold_ * det_range_) {
            new_local_map_points.vertex_max[i] -= mov_dist;
            new_local_map_points.vertex_min[i] -= mov_dist;
            temp_box_points.vertex_min[i] = local_map_points_.vertex_max[i] - mov_dist;
            cub_needrm_.push_back(temp_box_points);

        } else if (dist_to_map_edge[i][1] <= mov_threshold_ * det_range_) {
            new_local_map_points.vertex_max[i] += mov_dist;
            new_local_map_points.vertex_min[i] += mov_dist;
            temp_box_points.vertex_max[i] = local_map_points_.vertex_min[i] + mov_dist;
            cub_needrm_.push_back(temp_box_points);
        }
    }

    local_map_points_ = new_local_map_points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();

    if (cub_needrm_.size() > 0)
        kdtree_delete_count_ = ikdtree_.Delete_Point_Boxes(cub_needrm_);

    kdtree_delete_time_ = omp_get_wtime() - delete_begin;
}

bool SLAM::sync_packages(MeasureGroup& meas)
{
    static double lidar_mean_scan_time = 0.0;
    static int scan_number = 0;

    if (lidar_buffer_.empty() || imu_buffer_.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed_) {
        meas.lidar = lidar_buffer_.front();
        meas.lidar_beg_time = time_buffer_.front();
        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time_ = meas.lidar_beg_time + lidar_mean_scan_time;
            std::cerr << "Too few input point cloud!\n";
        } else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scan_time) {
            lidar_end_time_ = meas.lidar_beg_time + lidar_mean_scan_time;
        } else {
            scan_number++;
            lidar_end_time_ = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scan_time += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scan_time)
                / scan_number;
        }

        meas.lidar_end_time = lidar_end_time_;

        lidar_pushed_ = true;
    }

    if (last_timestamp_imu_ < lidar_end_time_) {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = get_time_sec(imu_buffer_.front()->header.stamp);
    meas.imu.clear();
    while ((!imu_buffer_.empty()) && (imu_time < lidar_end_time_)) {
        imu_time = get_time_sec(imu_buffer_.front()->header.stamp);
        if (imu_time > lidar_end_time_)
            break;
        meas.imu.push_back(imu_buffer_.front());
        imu_buffer_.pop_front();
    }

    lidar_buffer_.pop_front();
    time_buffer_.pop_front();
    lidar_pushed_ = false;
    return true;
}

void SLAM::map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size_);
    PointNoNeedDownsample.reserve(feats_down_size_);

    for (int i = 0; i < feats_down_size_; i++) {
        /* transform to world frame */
        point_body_to_world(&(feats_down_body_->points[i]), &(feats_down_world_->points[i]));
        /* decide if need add to map */
        if (!nearest_points_[i].empty() && ekf_init_) {

            const auto& points_near = nearest_points_[i];

            bool need_add = true;

            BoxPointType box_of_point;
            PointType downsample_result;
            PointType mid_point;

            mid_point.x = static_cast<float>(
                floor(feats_down_world_->points[i].x / filter_size_map_min_) * filter_size_map_min_
                + 0.5 * filter_size_map_min_);
            mid_point.y = static_cast<float>(
                floor(feats_down_world_->points[i].y / filter_size_map_min_) * filter_size_map_min_
                + 0.5 * filter_size_map_min_);
            mid_point.z = static_cast<float>(
                floor(feats_down_world_->points[i].z / filter_size_map_min_) * filter_size_map_min_
                + 0.5 * filter_size_map_min_);

            float dist = calc_dist(feats_down_world_->points[i], mid_point);

            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min_
                && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min_
                && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min_) {

                PointNoNeedDownsample.push_back(feats_down_world_->points[i]);
                continue;
            }

            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++) {
                if (points_near.size() < NUM_MATCH_POINTS)
                    break;
                if (calc_dist(points_near[readd_i], mid_point) < dist) {
                    need_add = false;
                    break;
                }
            }

            if (need_add)
                PointToAdd.push_back(feats_down_world_->points[i]);

        } else {
            PointToAdd.push_back(feats_down_world_->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size_ = ikdtree_.Add_Points(PointToAdd, true);
    ikdtree_.Add_Points(PointNoNeedDownsample, false);
    add_point_size_ = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time_ = omp_get_wtime() - st_time;
}

void SLAM::save_to_pcd()
{
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(map_save_path_, *cloud_to_publish_);
}

void SLAM::h_share_model(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data)
{
    double match_start = omp_get_wtime();
    laser_cloud_ori_->clear();
    corr_normvect_->clear();
    total_residual_ = 0.0;

/** closest surface search and residual computation **/
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif

    for (int i = 0; i < feats_down_size_; i++) {
        PointType& point_body = feats_down_body_->points[i];
        PointType& point_world = feats_down_world_->points[i];

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);

        point_world.x = static_cast<float>(p_global(0));
        point_world.y = static_cast<float>(p_global(1));
        point_world.z = static_cast<float>(p_global(2));

        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto& points_near = nearest_points_[i];

        if (ekfom_data.converge) {
            /** Find the closest surfaces in the map **/
            ikdtree_.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);

            point_selected_surf_[i] = ((points_near.size() < NUM_MATCH_POINTS) ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5)
                ? false
                : true;
        }

        if (!point_selected_surf_[i])
            continue;

        Eigen::Matrix<float, (4), 1> pabcd;
        point_selected_surf_[i] = false;

        if (esti_plane(pabcd, points_near, 0.1f)) {

            float pd2 = pabcd(0) * point_world.x
                + pabcd(1) * point_world.y
                + pabcd(2) * point_world.z
                + pabcd(3);

            auto s = static_cast<float>(1 - 0.9 * fabs(pd2) / sqrt(p_body.norm()));

            if (s > 0.9) {
                point_selected_surf_[i] = true;
                normvec_->points[i].x = pabcd(0);
                normvec_->points[i].y = pabcd(1);
                normvec_->points[i].z = pabcd(2);
                normvec_->points[i].intensity = pd2;
                res_last_[i] = abs(pd2);
            }
        }
    }

    effect_feat_number_ = 0;

    for (int i = 0; i < feats_down_size_; i++) {
        if (point_selected_surf_[i]) {
            laser_cloud_ori_->points[effect_feat_number_] = feats_down_body_->points[i];
            corr_normvect_->points[effect_feat_number_] = normvec_->points[i];
            total_residual_ += res_last_[i];
            effect_feat_number_++;
        }
    }

    if (effect_feat_number_ < 1) {
        ekfom_data.valid = false;
        std::cerr << "No Effective Points!" << std::endl;
        // ROS_WARN("No Effective Points! \n");
        return;
    }

    res_mean_last_ = total_residual_ / effect_feat_number_;
    match_time_ += omp_get_wtime() - match_start;
    double solve_start_ = omp_get_wtime();

    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = Eigen::MatrixXd::Zero(effect_feat_number_, 12); // 23
    ekfom_data.h.resize(effect_feat_number_);

    for (int i = 0; i < effect_feat_number_; i++) {
        const PointType& laser_p = laser_cloud_ori_->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType& norm_p = corr_normvect_->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() * norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_) {
            V3D B(
                point_be_crossmat * s.offset_R_L_I.conjugate() * C); // s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A),
                VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        } else {
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A),
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    solve_time_ += omp_get_wtime() - solve_start_;
}

void SLAM::reset_trigger_callback()
{
    RCLCPP_INFO(get_logger(), "reset now");
    ////////// ROS2 INTERFACE //////////
    // timer
    main_process_timer_->cancel();
    map_publisher_timer_->cancel();

    // some switch to change mode
    pointcloud2_ = false;
    publish_effect_ = false;
    publish_map_ = false;
    publish_scan_ = false;
    publish_dense_ = false;
    publish_scan_body_ = false;
    publish_path_ = true;

    publish_path_count_ = 0;

    path_ = nav_msgs::msg::Path {};
    odom_after_mapped_ = nav_msgs::msg::Odometry {};
    geo_quat_ = geometry_msgs::msg::Quaternion {};
    msg_body_pose_ = geometry_msgs::msg::PoseStamped {};

    ///////// PROCESS /////////
    lidar_pushed_ = false;
    ekf_init_ = false;
    first_scan_ = true;
    first_lidar_ = true;
    runtime_pos_log_ = false;
    save_pcd_ = false;
    time_sync_ = false;
    extrinsic_est_ = true;
    time_diff_set_ = false;
    local_map_init = false;

    std::fill(point_selected_surf_, &point_selected_surf_[100000 - 1], false);

    effect_feat_number_ = 0;
    frame_number_ = 0;

    average_time_consu_ = 0;
    average_time_icp_ = 0;
    average_time_match_ = 0;
    average_time_incre_ = 0;
    average_time_solve_ = 0;
    average_time_const_h_time_ = 0;

    std::fill(epsi_, &epsi_[23 - 1], 0.001);

    kdtree_size_st_ = 0;
    kdtree_size_end_ = 0;
    add_point_size_ = 0;
    kdtree_delete_count_ = 0;

    std::fill(res_last_, &res_last_[100000 - 1], 0);
    det_range_ = 300.0f;
    time_diff_lidar_to_imu_ = 0.0;

    res_mean_last_ = 0.05;
    total_residual_ = 0.0;

    last_timestamp_lidar_ = 0;
    last_timestamp_imu_ = -1.0;

    gyr_cov_ = 0.1;
    acc_cov_ = 0.1;
    b_gyr_cov_ = 0.0001;
    b_acc_cov_ = 0.0001;

    filter_size_corner_min_ = 0;
    filter_size_surf_min_ = 0;
    filter_size_map_min_ = 0;
    fov_deg_ = 0;

    cube_len_ = 0;
    half_fov_cos_ = 0;
    FOV_DEG_ = 0;
    total_distance_ = 0;
    lidar_end_time_ = 0;
    first_lidar_time_ = 0;

    time_diff_lidar_wrt_imu_ = 0.0;

    time_log_count_ = 0;
    scan_count_ = 0;
    publish_count_ = 0;

    iter_count_ = 0;
    feats_down_size_ = 0;
    number_max_iterations_ = 0;
    laser_cloud_valid_number_ = 0;
    pcd_save_interval_ = -1;
    pcd_index_ = 0;

    point_search_ind_surf_.clear();
    cub_needrm_.clear();
    nearest_points_.clear();
    extrinT_.clear();
    extrinR_.clear();
    time_buffer_.clear();
    lidar_buffer_.clear();
    imu_buffer_.clear();

    feats_from_map_->clear();
    feats_undistort_->clear();
    feats_down_body_->clear();
    feats_down_world_->clear();
    normvec_->clear();
    laser_cloud_ori_->clear();
    corr_normvect_->clear();
    feats_array_->clear();
    cloud_to_publish_->clear();

    local_map_points_ = BoxPointType {};

    ikdtree_.~KD_TREE();
    new (&ikdtree_) KD_TREE<pcl::PointXYZINormal> {};

    x_axis_point_body_ = V3F {};
    x_axis_point_world_ = V3F {};
    euler_cur_ = V3D {};
    position_last_ = V3D {};
    lidar_t_wrt_imu_ = V3D {};
    lidar_r_wrt_imu_ = M3D {};

    // EKF inputs and output
    measures_ = MeasureGroup {};
    kf_ = esekfom::esekf<state_ikfom, 12, input_ikfom> {};
    state_point_ = state_ikfom {};
    pos_lid_ = vect3 {};

    preprocess_ = std::make_shared<Preprocess>();
    imu_process_ = std::make_shared<ImuProcess>();

    kdtree_incremental_time_ = 0.0;
    kdtree_search_time_ = 0.0;
    kdtree_delete_time_ = 0.0;

    match_time_ = 0;
    solve_time_ = 0;
    solve_const_h_time_ = 0;

    main_process_timer_->reset();
    map_publisher_timer_->reset();

    RCLCPP_INFO(get_logger(), "parameters have reset");

    load_parameter();

    initialize();

    RCLCPP_INFO(get_logger(), "reconstruct now");
}