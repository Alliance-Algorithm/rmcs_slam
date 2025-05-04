from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    launch = LaunchDescription()

    # 雷达驱动进程
    # 由于代码因素，有时该进程无法正确响应 sigint 信号，请使用 ctrl + \ 强制关闭
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("livox_ros_driver2"), "/launch", "/msg_MID360_launch.py"]
        )
    )
    # launch.add_action(lidar)

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
