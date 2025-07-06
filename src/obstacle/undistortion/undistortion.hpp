#pragma once

#include "orthotics.hpp"
#include "util/pimpl.hpp"

namespace rmcs {

class Undistortion final {
    RMCS_PIMPL_DEFINTION(Undistortion);

public:
    /// 有时候，为了对齐，你不得不简写一些单词
    using LivoMsg = livox_ros_driver2::msg::CustomMsg;
    using ImuData = ImuOrthotics::ImuData;
    using LidData = ImuOrthotics::LidData;
    using Point   = ImuOrthotics::Point;
    using Package = ImuOrthotics::Package;

    auto set_imu_transform(const Eigen::Isometry3d&) -> void;
    auto set_lid_transform(const Eigen::Isometry3d&) -> void;

    auto handle_lid_message(std::unique_ptr<LivoMsg>) -> void;
    auto handle_imu_message(std::unique_ptr<ImuData>) -> void;

    auto stop_process() -> void;
    auto try_query_undistort_cloud() -> std::shared_ptr<ImuOrthotics::CloudXYZ>;
};

}
