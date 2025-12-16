from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取配置文件路径
    slam_toolbox_config = PathJoinSubstitution([
        FindPackageShare('basic_driver'),
        'config',
        'slam_toolbox_config.yaml'
    ])

    return LaunchDescription([
        # 1. 启动 Livox 点云转换和激光扫描
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('basic_driver'),
                    'launch',
                    'livox2laser_launch.py'
                ])
            ])
        ),
        # 2. 启动 SLAM Toolbox (异步SLAM模式)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_toolbox_config,
                {
                    'use_sim_time': False,
                    'scan_topic': '/scan',
                    'odom_frame': 'odom',
                    'base_frame': 'base_frame'
                }],
            remappings=[
                ('/scan', '/scan')
            ]
        ),
    ])
