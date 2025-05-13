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


# ROS2 相关依赖
find_package(ament_cmake REQUIRED)
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
    rosidl_default_generators
)
foreach(dependency ${dependencies})
    find_package(${dependency} REQUIRED)
endforeach()

# 去除 PCL 相关警告
if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()

# 第三方依赖
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS imgproc)
find_package(PCL REQUIRED COMPONENTS common io segmentation filters)

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

# util 工具静态库构建
file(GLOB_RECURSE UTIL_SRC CONFIGURE_DEPENDS src/util/*.cpp)
add_library(
    util STATIC
    ${UTIL_SRC}
)
target_link_directories(
    util PRIVATE
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(
    util PRIVATE
    pcl_common
    pcl_segmentation
    pcl_filters
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
    pcl_common
    pcl_io
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
    pcl_common
    pcl_segmentation
    pcl_filters
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
    util
    fast_gicp
    pcl_io
    pcl_common
    pcl_filters
)


# Calibration 进程
file(GLOB_RECURSE CALIBRATION_SRC CONFIGURE_DEPENDS src/calibration/*.cpp)
add_executable(
    calibration
    ${CALIBRATION_SRC}
)
ament_target_dependencies(
    calibration
    ${dependencies}
)
target_link_libraries(
    calibration
    util
    fast_gicp
    pcl_common
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
