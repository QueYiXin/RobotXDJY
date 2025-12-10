from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    return LaunchDescription([
        # 1. 启动 Livox 点云转换节点 + TF发布器
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('basic_driver'),
                    'launch',
                    'livox2lidar_launch.py'
                ])
            ])
        ),

        # 2. 启动点云转激光扫描节点
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('pointcloud_to_laserscan'),
                    'launch',
                    'pointcloud2laserscan_launch.py'
                ])
            ])
        ),
    ])


