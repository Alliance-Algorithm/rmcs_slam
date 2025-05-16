#pragma once

namespace rmcs::string {

namespace slam {
    constexpr auto reset_service_name         = "/rmcs_slam/reset";
    constexpr auto save_map_service_name      = "/rmcs_slam/map_save";
    constexpr auto switch_record_service_name = "/rmcs_slam/switch_record";

    constexpr auto cloud_effected_world_topic = "/rmcs_slam/cloud_effected";
    constexpr auto cloud_world_topic          = "/rmcs_slam/cloud_registered_world";
    constexpr auto cloud_body_topic           = "/rmcs_slam/cloud_registered_body";
    constexpr auto constructed_map_topic      = "/rmcs_slam/constructed_map";
    constexpr auto pose_topic                 = "/rmcs_slam/pose";
    constexpr auto odometry_topic             = "/rmcs_slam/odometry";
    constexpr auto path_topic                 = "/rmcs_slam/path";
}
namespace location {
    constexpr auto initialize_service_name = "/rmcs_location/initialize";
    constexpr auto pose_topic_name         = "/rmcs_location/pose";
}
namespace obtacle {
    constexpr auto obstacle_map_topic = "/rmcs_map/map/grid";
}

}
