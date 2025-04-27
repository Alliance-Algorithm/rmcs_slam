from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    launch = LaunchDescription()
    
    # 障碍地图生成进程
    obstacle = Node(
        package="rmcs_slam",
        executable="obstacle",
        parameters= [[FindPackageShare("rmcs_slam"), "/config", "/obstacle.yaml"]],
        output="log",
    )
    launch.add_action(obstacle)

    # SLAM 节点，单独进程
    slam = Node(
        package="rmcs_slam",
        executable="slam",
        parameters= [[FindPackageShare("rmcs_slam"), "/config", "/slam.yaml"]],
        output="log",
    )
    launch.add_action(slam)

    return launch
