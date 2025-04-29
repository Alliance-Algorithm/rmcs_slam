## RMCS SLAM For Sentry

### Introduction

### Install

#### Environment

- any `linux`
- `ros:humble` in docker
- livox mid-360 lidar

#### Dependencies

1. you can **manually** download all dependencies

- livox-sdk

    ```sh
    git clone https://github.com/Livox-SDK/Livox-SDK2.git
    cd Livox-SDK2
    mkdir build && cd build
    cmake .. && make -j12
    sudo make install
    ```

- livox-ros-driver2
    ```sh
    # entry your workspace
    git clone https://github.com/Alliance-Algorithm/livox_ros_driver2.git src/livox_ros_driver2
    # then build it
    colcon build --packages-select livox_ros_driver2
    ```

- pcl

    ```sh
    sudo apt-get install -y ros-humble-pcl-conversions ros-humble-pcl-ros libpcl-ros-dev libpcl-dev
    ```

- eigen

    ```sh
    sudo apt-get install libeigen3-dev
    ```

2. or you can use one-click dependency installation of ros2

    ```sh
    # entry you workspace
    cd /path/to/workspace
    # install
    sudo rosdep install --from-paths src --ignore-src -r -y
    ```

    but livox_ros_driver2 still needs to be installed manually

#### Configure

The config file is in `/path/to/rmcs_slam/config`, for deficiency in runtime, any point cloud msg should not be published

```yaml
/**:
    ros__parameters:
        preprocess:
            scan_line: 40 
            # default: 4

        publish:
            path_en: true 
            # false: close the path output
            scan_publish_en: true 
            # false: close all the point cloud output
            scan_bodyframe_pub_en: false 
            # true: output the point cloud scans in IMU-body-frame

        pcd_save:
            pcd_save_en: false
            interval: -1 
            # how many LiDAR frames saved in each pcd file;
            # -1 : all frames will be saved in ONE pcd file, may lead to memory crash when having too much frames.

```
#### Build

```sh
# entry your workspace/src
cd /path/to/workspace/src

# clone the source codes
git clone https://github.com/Alliance-Algorithm/rmcs_slam.git

# return to the workspace
cd ..

# build
colcon build --merge-install

# if you just want to build this package
colcon build --merge-install --packages-select rmcs_slam

# then what you want is in the workspace/install/ 
```


### Usage
#### Start up

First of all, you should set the correct ip of your wired connection and mid-360 config (192.168.1.120 and **the same nuc ip**)

```sh
# ensure that setup.bash is sourced
# terminal used by this example is zsh
source /opt/ros/humble/setup.zsh
source /path/to/livox_ws/install/setup.zsh

# then start the lidar msg publisher
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# open another terminal
# source the environment of rmcs_slam
source /opt/ros/humble/setup.zsh
source /path/to/rmcs_ws/install/setup.zsh

# start
ros2 launch rmcs_slam slam.py

# if you want to check situation in rviz
# do not forget to choose the correct frame_id
ros2 launch rmcs_slam slam.py rviz:=true
```

Now you can subscribe the topic "/position" to get position relative to point of starting

### Develop
#### 
