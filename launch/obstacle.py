from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    location = Node(
        package="rmcs_slam",
        executable="obstacle",
        parameters= [[FindPackageShare("rmcs_slam"), "/config", "/obstacle.yaml"]],
        output="log",
    )
    launch = LaunchDescription()
    launch.add_action(location)

    return launch
