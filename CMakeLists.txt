cmake_minimum_required(VERSION 3.17)
project(rmcs_slam)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE "Release")

# open-mp 配置
include(ProcessorCount)
processorcount(N)
message("处理器线程数: ${N}")
if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("多线程优化数: 3")
elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("多线程优化数: 2")
else()
    add_definitions(-DMP_PROC_NUM=1)
    message("多线程优化数: 1")
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

# 去除 PCL 相关警告
if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()

# ROS2 相关依赖
set(dependencies
    rclcpp
    rclcpp_components
    rosbag2_cpp
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    std_srvs
    pcl_ros
    pcl_conversions
    livox_ros_driver2
)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 第三方依赖
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

# fast-gicp 构建
add_subdirectory(src/third-party/fast_gicp)
include_directories(src/third-party/fast_gicp/include/)

include_directories(src/)
include_directories(src/slam/)
include_directories(src/obstacle/)
include_directories(src/location/)

# ROS2 自定义消息，实际上只有 SLAM 用到了
rosidl_generate_interfaces(
    ${PROJECT_NAME}
    "msg/Pose6D.msg"
)
ament_export_dependencies(
    rosidl_default_runtime
)

# rmcs-slam 进程
file(GLOB_RECURSE SLAM_SRC CONFIGURE_DEPENDS src/slam/*.cpp)
add_executable(
    slam
    ${SLAM_SRC}
)
ament_target_dependencies(
    slam
    ${dependencies}
)
target_include_directories(
    slam PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
)
rosidl_get_typesupport_target(
    cpp_typesupport_target
    ${PROJECT_NAME}
    "rosidl_typesupport_cpp"
)
target_link_libraries(
    slam
    ${PCL_LIBRARIES}
    ${cpp_typesupport_target}
)

# rmcs-map 进程
file(GLOB_RECURSE OBSTACLE_SRC CONFIGURE_DEPENDS src/obstacle/*.cpp)
add_executable(
    obstacle
    ${OBSTACLE_SRC}
)
ament_target_dependencies(
    obstacle
    ${dependencies}
)
target_link_libraries(
    obstacle
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

# rmcs-location 进程
file(GLOB_RECURSE LOCATION_SRC CONFIGURE_DEPENDS src/location/*.cpp)
add_executable(
    location
    ${LOCATION_SRC}
)
ament_target_dependencies(
    location
    ${dependencies}
)
target_link_libraries(
    location
    fast_gicp
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

# ROS2 安装配置
install(
  TARGETS slam obstacle location
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
