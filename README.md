## RMCS SLAM For Sentry

### 简介

本仓库是专门适配哨兵机器人的定位程序，由 Fast-Lio 修改而来，包含三个模块，location，slam 和 obstacle，分别负责重定位，里程计和障碍感知

### 部署

#### 环境需求

- 任意 `linux` 发行版
- `ros:humble` in docker
- livox mid-360 雷达

#### 项目依赖

- 激光雷达的驱动程序 `livox-sdk`，需要安装在全局环境变量中，雷达运行时依赖该驱动

    ```sh
    git clone https://github.com/Livox-SDK/Livox-SDK2.git
    cd Livox-SDK2
    mkdir build && cd build
    cmake .. && make -j
    sudo make install
    # 安装结束后 Livox-SDK2/ 可以删除
    ```

- 激光雷达的ROS2接口 `livox-ros-driver2`，作为 ROS2 包放入工作空间
    ```sh
    # 进入你的 ROS2 工作目录
    git clone https://github.com/Alliance-Algorithm/livox_ros_driver2.git src/livox_ros_driver2
    # 构建之
    colcon build --packages-select livox_ros_driver2
    # 然后 source 完后便可以运行这个项目，不过在连接雷达前我们还需要一些配置工作，这个留到后面来讲
    ```

- `pcl` 全程是 `Point Cloud Library`，巨大的点云处理库，里边不只封装了很多算法，还有一些可视化工具（因此引入了 VTK，Qt 什么的，导致整个库巨大无比）

    ```sh
    # 这几个包其实一部分相互嵌套，保险起见，还是全部下载了为好
    # 除了 pcl 本体，还有一些 pcl 在 ros2 的接口绑定，数据转换等模块需要使用
    sudo apt-get install -y     \
    ros-humble-pcl-conversions  \
    ros-humble-pcl-ros          \
    libpcl-ros-dev              \
    libpcl-dev
    ```

- `eigen` 只有头文件的线性代数库，使用了巨量的模板元技巧来优化性能

    ```sh
    sudo apt-get install libeigen3-dev
    ```

在面对可以使用包管理下载的依赖时，你可以用下面的指令来一键下载依赖，但有时候下载得并不全，比如 PCL 的 ROS2 绑定部分
```sh
cd /path/to/workspace
sudo rosdep install --from-paths src --ignore-src -r -y
```

#### 配置项目

首先是激光雷达配置，配置文件在 `src/livox_ros_driver2/config/MID360_config.json`。我们需要配置两个东西，一个是本机IP，一个是激光雷达的IP

本机IP便是与激光雷达连接的网卡的IP，你可以使用 `ifconfig`，`nmcli`，`ip ad` 中任意一个指令查看，一般物理网卡设备的名称是 `“eno1“` 这种 en 开头的单词

而激光雷达的 IP 则需要查看 SN 码的最后两位，比如这两位是 20，那么这个激光雷达的 IP 就是 `192.168.1.120`,修改最后两位数字即可，但我们也可以通过 `Livox Viewer2` 这个软件修改雷达的 IP，一般情况下，雷达运行的网段最好不要和调试使用的 IP 网段相同，会发生路由错误导致调试连接失败，这是 [软件下载页](https://www.livoxtech.com/cn/downloads)

下面是一个配置的示例：

```json
{
  "lidar_summary_info": {
    "lidar_type": 8
  },
  "MID360": {
    "lidar_net_info": {
      "cmd_data_port": 56100,
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    // 下面这块的 ip 都填入本机 ip
    "host_net_info": {
      "cmd_data_ip": "192.168.100.2",
      "cmd_data_port": 56101,
      "push_msg_ip": "192.168.100.2",
      "push_msg_port": 56201,
      "point_data_ip": "192.168.100.2",
      "point_data_port": 56301,
      "imu_data_ip": "192.168.100.2",
      "imu_data_port": 56401,
      "log_data_ip": "",
      "log_data_port": 56501
    }
  },
  "lidar_configs": [
    // 激光雷达 1
    {
      "ip": "192.168.100.120",
      "pcl_data_type": 1,
      "pattern_mode": 0,
      "extrinsic_parameter": {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    },
    // 激光雷达 2
    {
      "ip": "192.168.100.174",
      "pcl_data_type": 1,
      "pattern_mode": 0,
      "extrinsic_parameter": {
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
        "x": 0,
        "y": 0,
        "z": 0
      }
    }
  ]
}

```

如果使用多个雷达，还需要在 `src/livox_ros_driver2/launch/msg_MID360_launch.py` 文件中将`multi_topic` 设置为 1，让不同雷达有不同的 Topic 名字，另外，不要试图使用上面的 `extrinsic_parameter` 选项来将雷达点云信息和 IMU 信息变换到标准坐标系，我试过了，只变换点云，Imu 不动，而且 Imu 的内置偏移都没做纠正，这是外参： 

```yaml
imu_extrinsic_translation: [ -0.011, -0.02329, 0.04412 ]
imu_extrinsic_orientation: [ 1., 0., 0., 0. ]
```

随后是项目配置

`src/rmcs_slam/config` 这个目录下是项目的配置文件，必须根据雷达修改的有三处地方

第一处 `src/rmcs_slam/config/slam.yaml`，需要修改雷达的 Topic 名字和初始位姿

```yaml
rmcs_slam:
  ros__parameters:

    ...

    primary_lidar:
      enable: true
      lidar_topic: "/livox/lidar_192_168_100_120"   # 需要修改
      imu_topic: "/livox/imu_192_168_100_120"       # 需要修改
      # 雷达原点在底盘坐标系的坐标
      # 单位是m，浮点数
      x: +0.14                                      # 需要修改
      y: +0.12                                      # 需要修改
      z: +0.27                                      # 需要修改
      # 描述了雷达坐标系如何旋转到底盘坐标系
      # 单位是度，浮点数
      yaw: -49.7                                    # 需要修改
      pitch: 0.0                                    # 需要修改
      roll: -50.0                                   # 需要修改

    # 双雷达带来的同步误差和位姿误差会导致 SLAM 极容易发散，不建议开启
    secondary_lidar:
      enable: false                                 # 默认是单雷达 SLAM，关掉就好
      lidar_topic: "/livox/lidar_192_168_100_174"   # 需要修改
      imu_topic: "/livox/imu_192_168_100_174"       # 需要修改
      x: -0.06602                                   # 需要修改
      y: -0.08964                                   # 需要修改
      z: +0.25121                                   # 需要修改
      yaw: 179.33                                   # 需要修改
      pitch: -0.57                                  # 需要修改
      roll: -51.38                                  # 需要修改
```

第二处，修改障碍生成的参数，在 `src/rmcs_slam/config/obstacle.yaml` 文件中，同上，需要填写 Topic 的名字和雷达的变换，因为为了确保实时性和稳定性，该模块直接处理裸的点云数据，别问我为什么没有两个模块共享同一个配置文件，问就是 ROS2 不好做，所以乖乖把参数再复制一边吧

```yaml
rmcs_map:
  ros__parameters:

    ...

      lid_topics: [
        "/livox/lidar_192_168_100_120",
        "/livox/lidar_192_168_100_174",
      ]
      imu_topics: [
        "/livox/imu_192_168_100_120",
        "/livox/imu_192_168_100_174",
      ]      
      # 雷达原点在底盘坐标系的坐标
      # 单位是m，浮点数
      # x y z
      lidar_translations: [
        +0.14, +0.12, +0.27,
        -0.06602, -0.08964, +0.25121,
      ]
      # 描述了雷达坐标系如何旋转到底盘坐标系
      # 单位是度，浮点数
      # yaw pith roll
      lidar_orientations: [
        -49.7, 0.0, -50.0,
        179.33, -0.57, -51.38,
      ]

    ...

```

第三处，`src/rmcs_slam/config/location.yaml`，当然，平时测试的使用这个模块可以不启用，这个主要是负责初始重定位和实时变换里程计发布的坐标到世界系的，里程计的原点是启动那一刻的位姿，而世界系则是我们自己定的，为了更好地导航，我们需要将坐标做一次变换到世界系

```yaml
rmcs_location:
  ros__parameters:
    # 重定位所用的地图，地图原点需要为哨兵导航的原点
    # 格式为 pcd
    # 确保地图的位姿是正确的，地板与 xy 平面重合
    map_path: "/path/to/your/pcd"                       # 需要修改

    ...

    # 默认的初始位姿，一般是纠正后的地图的原点，在基地前一块空间，
    # 和哨兵装甲板贴合，此时哨兵的位置即为原点
    # 约定 default 为红色
    default_pose:
      translation:
        x: 0.0                                          # 需要修改
        y: 0.0                                          # 需要修改
        z: 0.0                                          # 需要修改
      orientation:
        w: 1.0                                          # 需要修改
        x: 0.0                                          # 需要修改
        y: 0.0                                          # 需要修改
        z: 0.0                                          # 需要修改
    # 地图原点的对面方的相同位置，注意绕Z轴旋转180度
    # 对面为蓝色
    opposite_pose:
      translation:
        x: 21.0                                         # 需要修改
        y: 0.0                                          # 需要修改
        z: 0.0                                          # 需要修改
      orientation:
        w: 0.0                                          # 需要修改
        x: 0.0                                          # 需要修改
        y: 0.0                                          # 需要修改
        z: 1.0                                          # 需要修改
```

#### 项目构建

```sh
# 进入工作目录的 src 目录，一般是和 livox_ros_driver2 同一个工作目录
cd /path/to/workspace/src

# 拉取项目
git clone https://github.com/Alliance-Algorithm/rmcs_slam.git

# 返回工作目录
cd ..

# 小心地 build，这句指令会不做检查地，在任何位置先拉下 build log install 三个目录
# 即使这个目录不是 ROS2 的工作目录
colcon build --merge-install

# 如果你只想单独 build 该项目
colcon build --merge-install --packages-select rmcs_slam

# 没飞出报错信息的话，大抵是构建成功了
```


### 使用方法

```sh
# 确保 ROS2 的环境和本项目的环境被成功 source
source /opt/ros/humble/setup.zsh
source /path/to/workspace/install/setup.zsh

# 启动项目
ros2 launch rmcs_slam launch.py
```

默认情况下，`src/rmcs_slam/launch/launch.py`，即上述指令所调用的 `launch` 文件，会启动所有必须的程序

如果要停下项目的话，使用 `Ctrl + C`，但一般来说 `livox_ros_driver2` 的屎山代码会阻塞线程执行，让程序无法响应 SIGINT 信号，所以你需要使用 `Ctrl + \` 来强制杀死进程

下面是可能会用上的 `Topic`：

```
- /livox/imu_192_168_100_120                # imu topic
- /livox/imu_192_168_100_174                # imu topic
- /livox/lidar_192_168_100_120              # 雷达点云
- /livox/lidar_192_168_100_120/undistort    # 去畸变后的点云
- /livox/lidar_192_168_100_174              # 雷达点云
- /livox/lidar_192_168_100_174/undistort    # 去畸变后的点云
- /rmcs_location/pose                       # 变换到世界系后的机器人位姿
- /rmcs_map/map/grid                        # 障碍栅格地图
- /rmcs_slam/cloud_registered_body          # 被注册到 ikd-tree 中的 imu 系的点云
- /rmcs_slam/cloud_registered_world         # 被注册到 ikd-tree 中的世界系的点云
- /rmcs_slam/constructed_map                # 累积的点云地图，很大
- /rmcs_slam/path                           # 机器人经过的路径
- /rmcs_slam/pose                           # 里程计的位姿，以项目启动时刻为原点
```