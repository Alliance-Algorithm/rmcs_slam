#pragma once

#include "common/common_lib.hpp"
#include "common/use_ikfom.hpp"
#include <common/so3_math.hpp>

#include <cmath>
#include <csignal>
#include <fstream>

#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <Eigen/Eigen>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

/// configuration
#define MAX_INI_COUNT (10)

inline bool time_list(PointT& x, PointT& y) { return (x.curvature < y.curvature); };

/// imu process and undistortion
class ImuProcess {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ImuProcess();
    ~ImuProcess();

    void reset();
    void reset(double start_timestamp, const sensor_msgs::msg::Imu::ConstSharedPtr& lastimu);
    void set_extrinsic(const V3D& transl, const M3D& rot);
    void set_extrinsic(const V3D& transl);
    void set_extrinsic(const MD(4, 4) & T);
    void set_gyr_cov(const V3D& scaler);
    void set_acc_cov(const V3D& scaler);
    void set_gyr_bias_cov(const V3D& b_g);
    void set_acc_bias_cov(const V3D& b_a);
    void process(
        const MeasureGroup& meas, esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state,
        PointCloudXYZI::Ptr undistrot_pointcloud);

    Eigen::Matrix<double, 12, 12> Q;
    std::ofstream fout_imu;
    V3D cov_acc;
    V3D cov_gyr;
    V3D cov_acc_scale;
    V3D cov_gyr_scale;
    V3D cov_bias_gyr;
    V3D cov_bias_acc;
    double first_lidar_time;

private:
    void initialize_imu(
        const MeasureGroup& meas, esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state, int& N);
    void undistort_pcl(
        const MeasureGroup& meas, esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state,
        PointCloudXYZI& pcl_in_out);

    PointCloudXYZI::Ptr cur_pcl_un_;
    sensor_msgs::msg::Imu::ConstSharedPtr last_imu_;
    std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> v_imu_;
    std::vector<Pose6D> pose_imu_;
    std::vector<M3D> v_rot_pcl_;
    M3D Lidar_R_wrt_IMU;
    V3D Lidar_T_wrt_IMU;
    V3D mean_acc;
    V3D mean_gyr;
    V3D angvel_last;
    V3D acc_s_last;
    double start_timestamp_;
    double last_lidar_end_time_;
    int init_iter_num   = 1;
    bool b_first_frame_ = true;
    bool imu_need_init_ = true;
};
