#include "slam.hpp"

void SLAM::standard_subscription_callback(const sensor_msgs::msg::PointCloud2::UniquePtr& msg)
{
    global_mutex.lock();
    scan_count_++;
    double cur_time              = get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();
    if (!first_lidar_ && cur_time < last_timestamp_lidar_) {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer_.clear();
    }
    if (first_lidar_) {
        first_lidar_ = false;
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    preprocess_->process(msg, ptr);
    lidar_buffer_.push_back(ptr);
    time_buffer_.push_back(cur_time);
    last_timestamp_lidar_  = cur_time;
    s_plot11_[scan_count_] = omp_get_wtime() - preprocess_start_time;
    global_mutex.unlock();
    global_signal.notify_all();
}

void SLAM::livox_subscription_callback(const livox_ros_driver2::msg::CustomMsg::UniquePtr& msg)
{
    global_mutex.lock();
    double cur_time              = get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();
    scan_count_++;
    if (!first_lidar_ && cur_time < last_timestamp_lidar_) {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer_.clear();
    }
    if (first_lidar_) {
        first_lidar_ = false;
    }
    last_timestamp_lidar_ = cur_time;

    if (!time_sync_ && abs(last_timestamp_imu_ - last_timestamp_lidar_) > 10.0
        && !imu_buffer_.empty() && !lidar_buffer_.empty()) {
        printf(
            "IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",
            last_timestamp_imu_, last_timestamp_lidar_);
    }

    if (time_sync_ && !time_diff_set_ && abs(last_timestamp_lidar_ - last_timestamp_imu_) > 1
        && !imu_buffer_.empty()) {
        time_diff_set_           = true;
        time_diff_lidar_wrt_imu_ = last_timestamp_lidar_ + 0.1 - last_timestamp_imu_;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", time_diff_lidar_wrt_imu_);
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    preprocess_->process(msg, ptr);
    lidar_buffer_.push_back(ptr);
    time_buffer_.push_back(last_timestamp_lidar_);

    s_plot11_[scan_count_] = omp_get_wtime() - preprocess_start_time;
    global_mutex.unlock();
    global_signal.notify_all();
}

void SLAM::imu_subscription_callback(const sensor_msgs::msg::Imu::UniquePtr& msg_in)
{
    publish_count_++;
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));

    msg->header.stamp = get_ros_time(get_time_sec(msg_in->header.stamp) - time_diff_lidar_to_imu_);
    if (abs(time_diff_lidar_wrt_imu_) > 0.1 && time_sync_) {
        msg->header.stamp =
            rclcpp::Time(int64_t(time_diff_lidar_wrt_imu_ + get_time_sec(msg_in->header.stamp)));
    }

    double timestamp = get_time_sec(msg->header.stamp);

    global_mutex.lock();

    if (timestamp < last_timestamp_imu_) {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        imu_buffer_.clear();
    }

    last_timestamp_imu_ = timestamp;

    imu_buffer_.push_back(msg);
    global_mutex.unlock();
    global_signal.notify_all();
}

void SLAM::publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher)
{
    if (publish_scan_) {
        PointCloudXYZI::Ptr laserCloudFullRes(publish_dense_ ? feats_undistort_ : feats_down_body_);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++) {
            rgb_point_body_to_world(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
        }

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.stamp    = get_ros_time(lidar_end_time_);
        laserCloudmsg.header.frame_id = "lidar_init";
        publisher->publish(laserCloudmsg);
        publish_count_ -= PUBFRAME_PERIOD;
    }
}

void SLAM::publish_frame_body(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher)
{
    int size = feats_undistort_->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++) {
        rgb_point_body_lidar_to_imu(&feats_undistort_->points[i], &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp    = get_ros_time(lidar_end_time_);
    laserCloudmsg.header.frame_id = "lidar_link";
    publisher->publish(laserCloudmsg);
    publish_count_ -= PUBFRAME_PERIOD;
}

void SLAM::publish_effect_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher)
{
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effect_feat_number_, 1));
    for (int i = 0; i < effect_feat_number_; i++) {
        rgb_point_body_to_world(&laser_cloud_ori_->points[i], &laserCloudWorld->points[i]);
    }
    sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp    = get_ros_time(lidar_end_time_);
    laserCloudFullRes3.header.frame_id = "lidar_init";
    publisher->publish(laserCloudFullRes3);
}

void SLAM::publish_map(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher)
{

    PointCloudXYZI::Ptr laserCloudFullRes(publish_dense_ ? feats_undistort_ : feats_down_body_);

    int size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++) {
        rgb_point_body_to_world(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
    }
    *cloud_to_publish_ += *laserCloudWorld;

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*cloud_to_publish_, laserCloudmsg);
    laserCloudmsg.header.stamp    = get_ros_time(lidar_end_time_);
    laserCloudmsg.header.frame_id = "lidar_init";
    publisher->publish(laserCloudmsg);
}

void SLAM::publish_odometry(
    const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr& publisher,
    const std::unique_ptr<tf2_ros::TransformBroadcaster>& broadcaster)
{

    odom_after_mapped_.header.frame_id = "lidar_init";
    odom_after_mapped_.child_frame_id  = "lidar_link";

    odom_after_mapped_.header.stamp = get_ros_time(lidar_end_time_);
    set_pose_stamp(odom_after_mapped_.pose);

    publisher->publish(odom_after_mapped_);

    auto P = kf_.get_P();
    for (int i = 0; i < 6; i++) {
        int k = i < 3 ? i + 3 : i - 3;

        odom_after_mapped_.pose.covariance[i * 6 + 0] = P(k, 3);
        odom_after_mapped_.pose.covariance[i * 6 + 1] = P(k, 4);
        odom_after_mapped_.pose.covariance[i * 6 + 2] = P(k, 5);
        odom_after_mapped_.pose.covariance[i * 6 + 3] = P(k, 0);
        odom_after_mapped_.pose.covariance[i * 6 + 4] = P(k, 1);
        odom_after_mapped_.pose.covariance[i * 6 + 5] = P(k, 2);
    }

    geometry_msgs::msg::TransformStamped stamp;

    stamp.header.frame_id = "lidar_link";
    stamp.child_frame_id  = "lidar_init";

    auto t = Eigen::Affine3d(Eigen::Translation3d(
        odom_after_mapped_.pose.pose.position.x, odom_after_mapped_.pose.pose.position.y,
        odom_after_mapped_.pose.pose.position.z));
    auto r = Eigen::Affine3d(Eigen::Quaterniond(
        odom_after_mapped_.pose.pose.orientation.w, odom_after_mapped_.pose.pose.orientation.x,
        odom_after_mapped_.pose.pose.orientation.y, odom_after_mapped_.pose.pose.orientation.z));

    auto transform = (t * r).inverse();

    Eigen::Vector3d translation { transform.translation() };
    Eigen::Quaterniond rotation { transform.linear() };

    stamp.transform.translation.x = translation.x();
    stamp.transform.translation.y = translation.y();
    stamp.transform.translation.z = translation.z();
    stamp.transform.rotation.w    = rotation.w();
    stamp.transform.rotation.x    = rotation.x();
    stamp.transform.rotation.y    = rotation.y();
    stamp.transform.rotation.z    = rotation.z();

    broadcaster->sendTransform(stamp);
}

void SLAM::publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& publisher)
{
    set_pose_stamp(msg_body_pose_);

    msg_body_pose_.header.stamp    = get_ros_time(lidar_end_time_);
    msg_body_pose_.header.frame_id = "lidar_init";

    // lighten the load to keep rviz alive
    if (publish_path_count_ % 10 == 0) {
        path_.poses.push_back(msg_body_pose_);
        publisher->publish(path_);
    }

    publish_path_count_++;
}

void SLAM::map_publish_timer_callback()
{
    if (publish_map_)
        publish_map(laser_map_publisher_);
}

void SLAM::map_save_callback(
    const std_srvs::srv::Trigger::Request::ConstSharedPtr& req,
    const std_srvs::srv::Trigger::Response::SharedPtr& res)
{

    RCLCPP_INFO(this->get_logger(), "saving map to %s...", map_file_path_.c_str());

    if (save_pcd_) {
        save_to_pcd();
        res->success = true;
        res->message = "Map saved.";
    } else {
        res->success = false;
        res->message = "Map save disabled.";
    }
}