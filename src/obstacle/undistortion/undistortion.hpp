#pragma once

#include "orthotics.hpp"
#include "util/pimpl.hpp"

namespace rmcs {

class Undistortion final {
    RMCS_PIMPL_DEFINTION(Undistortion);

public:
    /// 有时候，为了对齐，你不得不简写一些单词
    using LivMsg = livox_ros_driver2::msg::CustomMsg;
    using ImuMsg = ImuOrthotics::ImuMsg;

    auto set_imu_transform(const Eigen::Isometry3d&) -> void;
    auto set_lid_transform(const Eigen::Isometry3d&) -> void;

    auto push_lid_message(std::unique_ptr<LivMsg>) -> void;
    auto push_imu_message(std::unique_ptr<ImuMsg>) -> void;

    auto stop_process() -> void;
    auto try_query_undistort_cloud() -> std::shared_ptr<ImuOrthotics::CloudXYZ>;
};

}
