# RobotXDJY
This project focuses on the development of the Xingdongjiyuan Q5Air robot.


## dependence
#### livox_ros_drivers 
https://github.com/Livox-SDK/livox_ros_driver 激光雷达驱动，机器人上已经使用，但我们需要使用它的msg
#### pointcloud_to_laserscan
https://github.com/ros-perception/pointcloud_to_laserscan 点云转激光算法
#### slam_toolbox
https://github.com/SteveMacenski/slam_toolbox/tree/humble# slam算法，注意需要对应ros版本（也可以apt安装）


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

## 建图运行流程
```shell
# 注意每个开启的终端需要先source一下工作区，即
source install/setup.bash

# 先开启机器人的active模式（默认开机时是开发者模式，机器人状态为INIT）
# 这一步会启动一些tf发布节点，必须执行
ros2 run basic_driver wr1_server                            # 可在机器人上运行
# 等上述代码正常启动后，开启底盘驱动代码
ros2 run basic_driver keyboard_ctrl                         # 可在机器人上运行
ros2 run teleop_twist_keyboard teleop_twist_keyboard        # 实现键盘控制移动

# 启动建图代码
ros2 launch basic_driver livox2map_launch.py                # 启动点云转化、tf链接、mapping算法
rviz2                                                       # 观察建图效果，可使用以配置的文件，在.rviz2下

# 地图保存
ros2 run nav2_map_server map_saver_cli -t map -f map1       # 保存/map节点内容（地图），文件名为map1（可修改）
```

## 其它常用操作
```shell
# "/livox/lidar -> /cloud_in" 将livox驱动发出的非标准点云转化为标准点云消息格式
# "livox_frame <-tf-> lidar_link" 链接livox_frame和lidar_link的tf关系
ros2 launch basic_driver livox2lidar.py 

# "/cloud_in -> /scan" 将点云数据转化为激光数据
ros2 launch pointcloud_to_laserscan pointcloud2laserscan_launch.py
```