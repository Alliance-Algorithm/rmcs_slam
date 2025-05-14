from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    location = Node(
        package="rmcs_slam",
        executable="calibration",
        parameters= [[FindPackageShare("rmcs_slam"), "/config", "/calibration.yaml"]],
        output="log",
    )
    launch = LaunchDescription()
    launch.add_action(location)

    return launch
