import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory("rmcs_slam")
    default_config_path = os.path.join(package_path, "config")
    default_rviz_config_path = os.path.join(package_path, "rviz", "rmcs_slam.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    config_path = LaunchConfiguration("config_path")
    config_file = LaunchConfiguration("config_file")
    rviz_use = LaunchConfiguration("rviz")
    rviz_cfg = LaunchConfiguration("rviz_cfg")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        "config_path",
        default_value=default_config_path,
        description="Yaml config file path",
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        "config_file", default_value="online.yaml", description="Config file"
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        "rviz", default_value="false", description="Use rviz to monitor results"
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        "rviz_cfg",
        default_value=default_rviz_config_path,
        description="RViz config file path",
    )

    rmcs_slam = Node(
        package="rmcs_slam",
        executable="rmcs_slam_exe",
        parameters=[
            PathJoinSubstitution([config_path, config_file]),
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_cfg],
        condition=IfCondition(rviz_use),
    )

    description = LaunchDescription()

    description.add_action(declare_use_sim_time_cmd)
    description.add_action(declare_config_path_cmd)
    description.add_action(declare_config_file_cmd)
    description.add_action(declare_rviz_cmd)
    description.add_action(declare_rviz_config_path_cmd)

    description.add_action(rmcs_slam)
    description.add_action(rviz)

    return description
