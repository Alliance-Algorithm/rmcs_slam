#include "imu_processing.hpp"
#include "common/so3_math.hpp"
#include <memory>

ImuProcess::ImuProcess()
    : is_first_frame_(true)
    , need_init_imu_(true)
    , start_timestamp_(-1) {
    init_iter_num   = 1;
    Q               = process_noise_cov();
    cov_acc         = V3D(0.1, 0.1, 0.1);
    cov_gyr         = V3D(0.1, 0.1, 0.1);
    cov_bias_gyr    = V3D(0.0001, 0.0001, 0.0001);
    cov_bias_acc    = V3D(0.0001, 0.0001, 0.0001);
    mean_acc        = V3D(0, 0, -1.0);
    mean_gyr        = V3D(0, 0, 0);
    angvel_last     = Zero3d;
    Lidar_T_wrt_IMU = Zero3d;
    Lidar_R_wrt_IMU = Eye3d;
    last_imu_       = std::make_shared<sensor_msgs::msg::Imu>();
}

ImuProcess::~ImuProcess() = default;

void ImuProcess::reset() {
    // ROS_WARN("Reset ImuProcess");
    mean_acc         = V3D(0, 0, -1.0);
    mean_gyr         = V3D(0, 0, 0);
    angvel_last      = Zero3d;
    need_init_imu_   = true;
    start_timestamp_ = -1;
    init_iter_num    = 1;
    v_imu_.clear();
    pose_imu_.clear();
    last_imu_   = std::make_shared<sensor_msgs::msg::Imu>();
    cur_pcl_un_ = std::make_shared<PointCloudXYZI>();
}

void ImuProcess::set_extrinsic(const V3D& translation, const M3D& orientation) {
    Lidar_T_wrt_IMU = translation;
    Lidar_R_wrt_IMU = orientation;
}

void ImuProcess::set_gyr_cov(const V3D& scaler) { cov_gyr_scale = scaler; }

void ImuProcess::set_acc_cov(const V3D& scaler) { cov_acc_scale = scaler; }

void ImuProcess::set_gyr_bias_cov(const V3D& b_g) { cov_bias_gyr = b_g; }

void ImuProcess::set_acc_bias_cov(const V3D& b_a) { cov_bias_acc = b_a; }

using KfState = esekfom::esekf<state_ikfom, 12, input_ikfom>;
void ImuProcess::initialize_imu(const MeasureGroup& meas, KfState& kf_state, int& N) {
    /** 1. initializing the gravity, gyro bias, acc and gyro covariance
     ** 2. normalize the acceleration measurenments to unit gravity **/

    V3D cur_acc, cur_gyr;

    if (is_first_frame_) {
        reset();
        N                   = 1;
        is_first_frame_     = false;
        const auto& imu_acc = meas.imu.front()->linear_acceleration;
        const auto& gyr_acc = meas.imu.front()->angular_velocity;
        mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
        mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
        first_lidar_time = meas.lidar_beg_time;
    }

    for (const auto& imu : meas.imu) {
        const auto& imu_acc = imu->linear_acceleration;
        const auto& gyr_acc = imu->angular_velocity;
        cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
        cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

        mean_acc += (cur_acc - mean_acc) / N;
        mean_gyr += (cur_gyr - mean_gyr) / N;

        cov_acc = cov_acc * (N - 1.0) / N
            + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
        cov_gyr = cov_gyr * (N - 1.0) / N
            + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);

        // cout<<"acc norm: "<<cur_acc.norm()<<" "<<mean_acc.norm()<<endl;

        N++;
    }
    state_ikfom init_state = kf_state.get_x();
    init_state.grav        = S2(-mean_acc / mean_acc.norm() * G_m_s2);

    // state_inout.rot = Eye3d; // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));
    init_state.bg           = mean_gyr;
    init_state.offset_T_L_I = Lidar_T_wrt_IMU;
    init_state.offset_R_L_I = Lidar_R_wrt_IMU;
    kf_state.change_x(init_state);

    esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
    init_P.setIdentity();
    init_P(6, 6) = init_P(7, 7) = init_P(8, 8) = 0.00001;
    init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.00001;
    init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.0001;
    init_P(18, 18) = init_P(19, 19) = init_P(20, 20) = 0.001;
    init_P(21, 21) = init_P(22, 22) = 0.00001;
    kf_state.change_P(init_P);
    last_imu_ = meas.imu.back();
}

void ImuProcess::undistort_pcl(
    const MeasureGroup& meas, KfState& kf_state, PointCloudXYZI& pointcloud_output) {
    /*** add the imu of the last frame-tail to the of current frame-head ***/
    auto v_imu = meas.imu;
    v_imu.push_front(last_imu_);
    const double& imu_beg_time = rclcpp::Time(v_imu.front()->header.stamp).seconds();
    const double& imu_end_time = rclcpp::Time(v_imu.back()->header.stamp).seconds();
    const double& pcl_beg_time = meas.lidar_beg_time;
    const double& pcl_end_time = meas.lidar_end_time;

    /*** sort point clouds by offset time ***/
    pointcloud_output = *(meas.lidar);
    std::sort(pointcloud_output.points.begin(), pointcloud_output.points.end(),
        [](const PointT& x, const PointT& y) { return (x.curvature < y.curvature); });

    /*** Initialize IMU pose ***/
    state_ikfom imu_state = kf_state.get_x();
    pose_imu_.clear();
    pose_imu_.push_back(set_pose6d(0.0, acc_s_last, angvel_last, imu_state.vel, imu_state.pos,
        imu_state.rot.toRotationMatrix()));

    /*** forward propagation at each imu point ***/
    V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
    M3D R_imu;

    double dt = 0;

    input_ikfom in;
    for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++) {
        auto&& head = *(it_imu);
        auto&& tail = *(it_imu + 1);

        double tail_stamp = rclcpp::Time(tail->header.stamp).seconds();
        double head_stamp = rclcpp::Time(head->header.stamp).seconds();

        if (tail_stamp < last_lidar_end_time_) continue;

        angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
            0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
            0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
        acc_avr << 0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
            0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
            0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

        // fout_imu << setw(10) << head->header.stamp.toSec() - first_lidar_time << " " <<
        // angvel_avr.transpose() << " " << acc_avr.transpose() << endl;

        acc_avr = acc_avr * G_m_s2 / mean_acc.norm(); // - state_inout.ba;

        if (head_stamp < last_lidar_end_time_) {
            dt = tail_stamp - last_lidar_end_time_;
            // dt = tail->header.stamp.toSec() - pcl_beg_time;
        } else {
            dt = tail_stamp - head_stamp;
        }

        in.acc  = acc_avr;
        in.gyro = angvel_avr;

        Q.block<3, 3>(0, 0).diagonal() = cov_gyr;
        Q.block<3, 3>(3, 3).diagonal() = cov_acc;
        Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;
        Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;

        kf_state.predict(dt, Q, in);

        /* save the poses at each IMU measurements */
        imu_state   = kf_state.get_x();
        angvel_last = angvel_avr - imu_state.bg;
        acc_s_last  = imu_state.rot * (acc_avr - imu_state.ba);
        for (int i = 0; i < 3; i++) {
            acc_s_last[i] += imu_state.grav[i];
        }
        double&& offs_t = tail_stamp - pcl_beg_time;
        pose_imu_.push_back(set_pose6d(offs_t, acc_s_last, angvel_last, imu_state.vel,
            imu_state.pos, imu_state.rot.toRotationMatrix()));
    }

    /*** calculated the pos and attitude prediction at the frame-end ***/
    double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
    dt          = note * (pcl_end_time - imu_end_time);
    kf_state.predict(dt, Q, in);

    imu_state            = kf_state.get_x();
    last_imu_            = meas.imu.back();
    last_lidar_end_time_ = pcl_end_time;

    /*** undistort each lidar point (backward propagation) ***/
    if (pointcloud_output.points.begin() == pointcloud_output.points.end()) return;
    auto it_pcl = pointcloud_output.points.end() - 1;
    for (auto it_kp = pose_imu_.end() - 1; it_kp != pose_imu_.begin(); it_kp--) {
        auto head = it_kp - 1;
        auto tail = it_kp;
        R_imu << MAT_FROM_ARRAY(head->rot);
        // cout<<"head imu acc: "<<acc_imu.transpose()<<endl;
        vel_imu << VEC_FROM_ARRAY(head->vel);
        pos_imu << VEC_FROM_ARRAY(head->pos);
        acc_imu << VEC_FROM_ARRAY(tail->acc);
        angvel_avr << VEC_FROM_ARRAY(tail->gyr);

        for (; it_pcl->curvature / double(1000) > head->offset_time; it_pcl--) {
            dt = it_pcl->curvature / double(1000) - head->offset_time;

            /* Transform to the 'end' frame, using only the rotation
             * Note: Compensation direction is INVERSE of Frame's moving direction
             * So if we want to compensate a point at timestamp-i to the frame-e
             * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global
             * frame */
            M3D R_i(R_imu * Exp(angvel_avr, dt));

            V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);
            V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);
            V3D P_compensate = imu_state.offset_R_L_I.conjugate()
                * (imu_state.rot.conjugate()
                        * (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei)
                    - imu_state.offset_T_L_I); // not accurate!

            // save Undistorted points and their rotation
            it_pcl->x = static_cast<float>(P_compensate(0));
            it_pcl->y = static_cast<float>(P_compensate(1));
            it_pcl->z = static_cast<float>(P_compensate(2));

            if (it_pcl == pointcloud_output.points.begin()) break;
        }
    }
}

void ImuProcess::process(const MeasureGroup& meas, KfState& kf_state,
    const std::shared_ptr<PointCloudXYZI>& undistrot_pointcloud) {

    if (meas.imu.empty()) return;

    assert(meas.lidar != nullptr);

    if (need_init_imu_) {
        initialize_imu(meas, kf_state, init_iter_num);

        need_init_imu_ = true;

        last_imu_ = meas.imu.back();

        state_ikfom imu_state = kf_state.get_x();
        if (init_iter_num > kImuInitCountThreshold) {
            cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
            need_init_imu_ = false;

            cov_acc = cov_acc_scale;
            cov_gyr = cov_gyr_scale;
        }
        return;
    }

    undistort_pcl(meas, kf_state, *undistrot_pointcloud);
}
