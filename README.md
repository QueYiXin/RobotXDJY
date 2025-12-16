# RobotXDJY
This project focuses on the development of the Xingdongjiyuan Q5Air robot.


## dependence
#### livox_ros_drivers 
https://github.com/Livox-SDK/livox_ros_driver 激光雷达驱动，机器人上已经使用，但我们需要使用它的msg
#### pointcloud_to_laserscan
https://github.com/ros-perception/pointcloud_to_laserscan 点云转激光算法
#### slam_toolbox
https://github.com/SteveMacenski/slam_toolbox/tree/humble# slam算法，注意需要对应ros版本（也可以apt安装）
#### Navigation2
http://fishros.org/doc/nav2/getting_started/index.html 导航算法（apt安装）


## build
### 关于livox_ros_driver2的编译问题
由于livox_ros_driver2在编译的时候需要传入ROS的distro，故在包含livox_ros_driver2的编译操作中，执行如下操作：
```shell
colcon build --cmake-args -DHUMBLE_ROS=humble
```
并且在使用livox_ros_driver2需要事先编译SDK源文件，详细过程参考：
https://github.com/Livox-SDK/Livox-SDK2/blob/master/README.md

### 关于分步编译（只编译修改的包）
由于有时候只改变了一个包中的内容，如果重复编译，显得很麻烦，执行如下操作：
```shell
colcon build --packages-select basic_driver
```

## 任务运行前
```shell
# 注意每个开启的终端需要先source一下工作区，即
source install/setup.bash
# 先开启机器人的active模式（默认开机时是开发者模式，机器人状态为INIT）
# 这一步会启动一些tf发布节点，必须执行
ros2 run basic_driver wr1_server                            # 可在机器人上运行
# 等上述代码正常启动后，开启底盘驱动代码
ros2 run basic_driver keyboard_ctrl                         # 可在机器人上运行
```

## 建图运行流程
```shell
# -----------------------
# 先执行 任务运行前 操作
# -----------------------

# 启动建图代码
ros2 launch basic_driver livox2map_launch.py                # 启动点云转化、tf链接、mapping算法
rviz2                                                       # 观察建图效果，可使用以配置的文件，在.rviz2下
ros2 run teleop_twist_keyboard teleop_twist_keyboard        # 实现键盘控制移动

# 地图保存
ros2 run nav2_map_server map_saver_cli -t map -f map1       # 保存/map节点内容（地图），文件名为map1（可修改）
```

## 导航运行流程
```shell
# -----------------------
# 先执行 任务运行前 操作
# -----------------------

# 一键启动导航
ros2 launch basic_driver nav2_launch.py                     # 导航launch文件
```

## 其它常用操作
```shell
# "/livox/lidar -> /cloud_in" 将livox驱动发出的非标准点云转化为标准点云消息格式
# "livox_frame <-tf-> lidar_link" 链接livox_frame和lidar_link的tf关系
ros2 launch basic_driver livox2lidar.py 

# "/cloud_in -> /scan" 将点云数据转化为激光数据
ros2 launch pointcloud_to_laserscan pointcloud2laserscan_launch.py
```

## 文件目录
```shell
xdjy_ws
├── .rviz2
│   ├──default.rviz     # 建图时使用rviz2的配置
│   ├──Nav2.rviz        # 导航时使用rviz2的配置
├── map_1765801095_origin.pgm   # 原始未滤波地图
├── map_1765801095.pgm          # 滤波后地图
├── map_1765801095.yaml         # 地图配置文件
├── nav2_params.yaml            # 导航配置文件
├── nav2_params.yaml.bak        # 官方导航配置文件（备份）
├── README.md                   
├── slam_toolbox_sync.png       # slam_toolbox_sync架构图
│   
├── src
│   ├── basic_driver            # （开发）算法驱动
│   │   ├── basic_driver        # 
│   │   │   ├── camera_viewer.py            # 摄像头测试代码
│   │   │   ├── __init__.py
│   │   │   ├── keyboard_ctrl.py            # 键盘控制（用于转换teleop_twist_keyboard这个包）
│   │   │   ├── livox2lidar.py              # livox的msg转PointCloud2
│   │   │   ├── __pycache__
│   │   │   │   ├── __init__.cpython-310.pyc
│   │   │   │   └── lidar2nt.cpython-310.pyc
│   │   │   ├── wr1_control.py              # 机器人关节控制代码
│   │   │   └── wr1_server.py               # 机器人状态服务代码（用于转换为Active模式，才可操作）
│   │   ├── config
│   │   │   └── slam_toolbox_config.yaml    # 建图配置文件（暂时没用）
│   │   ├── launch
│   │   │   ├── livox2laser_launch.py       # '原始点云'转'标准激光' 启动文件
│   │   │   ├── livox2lidar_launch.py       # '原始点云'转'标准点云' 启动文件
│   │   │   ├── livox2map_launch.py         # 建图  启动文件
│   │   │   └── nav2_launch.py              # 导航  启动文件
│   │   ├── package.xml
│   │   ├── README.md
│   │   ├── resource
│   │   │   └── basic_driver
│   │   ├── setup.cfg
│   │   └── setup.py
│   │
│   ├── livox_ros_driver2       # （开源）livox激光驱动
│   ├── official_demo           # （官方）示例代码
│   ├── pointcloud_to_laserscan # （开源）点云转激光
│   ├── slam_toolbox            # （开源）建图算法
│   └── xbot_common_interfaces  # （官方）Q5Air的ROS2消息包定义
└── tools   # 各种工具代码
    └── map_filter.py           # 原始地图滤波代码（连通区域滤波）
```