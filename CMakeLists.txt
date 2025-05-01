cmake_minimum_required(VERSION 3.8)
project(rmcs_slam)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_BUILD_TYPE "Release")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
    include(ProcessorCount)
    PROCESSORCOUNT(N)
    message("Processer number:  ${N}")

    if(N GREATER 4)
        add_definitions(-DMP_EN)
        add_definitions(-DMP_PROC_NUM=3)
        message("core for MP: 3")
    elseif(N GREATER 3)
        add_definitions(-DMP_EN)
        add_definitions(-DMP_PROC_NUM=2)
        message("core for MP: 2")
    else()
        add_definitions(-DMP_PROC_NUM=1)
    endif()
else()
    add_definitions(-DMP_PROC_NUM=1)
endif()

if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

# ROS dependencies
set(dependencies
    rclcpp
    rclcpp_components
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    std_srvs
    visualization_msgs
    pcl_ros
    pcl_conversions
    livox_ros_driver2
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Thirdparty libraries
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

include_directories(src/)
include_directories(src/slam/)
include_directories(src/obstacle/)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
    "msg/Pose6D.msg"
)

ament_export_dependencies(rosidl_default_runtime)

# Target slam exe
file(GLOB_RECURSE SLAM_SRC CONFIGURE_DEPENDS src/slam/**.cpp)

add_executable(slam ${SLAM_SRC})

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

target_include_directories(
    slam PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(slam ${dependencies})

# Target rmcs map
file(GLOB_RECURSE OBSTACLE_SRC CONFIGURE_DEPENDS src/obstacle/**.cpp)

add_executable(obstacle ${OBSTACLE_SRC})

ament_target_dependencies(obstacle ${dependencies})

target_link_libraries(
    obstacle
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

# Install
install(
  TARGETS slam
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
