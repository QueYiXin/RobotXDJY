# RobotXDJY
This project focuses on the development of the Xingdongjiyuan Q5Air robot.

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
## 常用操作
```shell
# "/livox/lidar -> /cloud_in" 将livox驱动发出的非标准点云转化为标准点云消息格式
# "livox_frame <-tf-> lidar_link" 链接livox_frame和lidar_link的tf关系
ros2 launch basic_driver lidar2nt_launch.py 

# "/cloud_in -> /scan" 将点云数据转化为激光数据
ros2 launch pointcloud_to_laserscan pointcloud2laserscan_launch.py
```