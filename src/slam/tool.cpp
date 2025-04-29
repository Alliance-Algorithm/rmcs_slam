#include "slam.hpp"

void SLAM::signal_callback(int signal) {
    std::cout << "catch signal %d" << signal << std::endl;

    global_signal.notify_all();
    rclcpp::shutdown();
}

void SLAM::dump_lio_state_to_log(FILE* file) {
    V3D rot_ang(Log(state_point_.rot.toRotationMatrix()));
    fprintf(file, "%lf ", measures_.lidar_beg_time - first_lidar_time_);
    fprintf(file, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2)); // Angle
    fprintf(
        file, "%lf %lf %lf ", state_point_.pos(0), state_point_.pos(1), state_point_.pos(2)); // Pos
    fprintf(file, "%lf %lf %lf ", 0.0, 0.0, 0.0); // omega
    fprintf(
        file, "%lf %lf %lf ", state_point_.vel(0), state_point_.vel(1), state_point_.vel(2)); // Vel
    fprintf(file, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                             // Acc
    fprintf(
        file, "%lf %lf %lf ", state_point_.bg(0), state_point_.bg(1), state_point_.bg(2)); // Bias_g
    fprintf(
        file, "%lf %lf %lf ", state_point_.ba(0), state_point_.ba(1), state_point_.ba(2)); // Bias_a
    fprintf(
        file, "%lf %lf %lf ", state_point_.grav[0], state_point_.grav[1],
        state_point_.grav[2]);                                                             // Bias_a
    fprintf(file, "\r\n");
    fflush(file);
}

void SLAM::point_body_to_world_ikfom(
    PointType const* const in, PointType* const out, state_ikfom& state) {
    V3D p_body(in->x, in->y, in->z);
    V3D p_global(state.rot * (state.offset_R_L_I * p_body + state.offset_T_L_I) + state.pos);

    out->x = static_cast<float>(p_global(0));
    out->y = static_cast<float>(p_global(1));
    out->z = static_cast<float>(p_global(2));

    out->intensity = in->intensity;
}

void SLAM::point_body_to_world(PointType const* const pi, PointType* const po) {
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(
        state_point_.rot * (state_point_.offset_R_L_I * p_body + state_point_.offset_T_L_I)
        + state_point_.pos);

    po->x = static_cast<float>(p_global(0));
    po->y = static_cast<float>(p_global(1));
    po->z = static_cast<float>(p_global(2));

    po->intensity = pi->intensity;
}

void SLAM::rgb_point_body_to_world(PointType const* const pi, PointType* const po) {
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(
        state_point_.rot * (state_point_.offset_R_L_I * p_body + state_point_.offset_T_L_I)
        + state_point_.pos);

    po->x         = static_cast<float>(p_global(0));
    po->y         = static_cast<float>(p_global(1));
    po->z         = static_cast<float>(p_global(2));
    po->intensity = pi->intensity;
}

void SLAM::rgb_point_body_lidar_to_imu(PointType const* const pi, PointType* const po) {
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point_.offset_R_L_I * p_body_lidar + state_point_.offset_T_L_I);

    po->x = static_cast<float>(p_body_imu(0));
    po->y = static_cast<float>(p_body_imu(1));
    po->z = static_cast<float>(p_body_imu(2));

    po->intensity = pi->intensity;
}

void SLAM::points_cache_collect() {
    PointVector points_history;
    ikdtree_.acquire_removed_points(points_history);
}
