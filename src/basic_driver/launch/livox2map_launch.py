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

        # 2. 发布静态TF: odom -> base_link (如果你的系统已经发布了动态tf，可以注释掉这一段)
        # 如果你有里程计节点在发布odom->base_link的tf，请注释掉下面这个节点
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='odom_to_base_link_publisher',
        #     arguments=['0', '0', '0', '0', '0', '0', 'odom_frame', 'base_frame'],
        #     output='screen'
        # ),

        # 3. 发布静态TF: base_link -> lidar_link
        # 这里假设雷达安装在机器人中心，如果雷达有偏移，需要修改x,y,z值
        # 2. STATIC TF: base_frame -> livox_frame
        # Connects the Lidar to the Robot Base
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_livox',
            # arguments: [x y z qx qy qz parent child]
            arguments=['0', '0', '0', '0', '0', '0', 'base_frame', 'lidar_link'],
            output='screen'
        ),

        # 3. STATIC TF: odom_frame -> base_frame
        # REQUIRED: Fakes odometry if you don't have wheel encoders.
        # This provides the transform from 'odom_frame' to 'base_frame'
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base',
            arguments=['0', '0', '0', '0', '0', '0', 'odom_frame', 'base_frame'],
            output='screen'
        ),
        # 4. 启动 SLAM Toolbox (异步SLAM模式)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_toolbox_config,
                {
                    'use_sim_time': False,
                    'scan_topic': '/scan', # <--- 强制在 Launch 里覆盖，防止 YAML 没生效
                    'odom_frame': 'odom_frame',
                    'base_frame': 'base_frame'
                }],
            remappings=[
                ('/scan', '/scan')
            ]
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             FindPackageShare('slam_toolbox'),
        #             'launch',
        #             'online_async_launch.py'
        #         ])
        #     ]),
        # ),
    ])
