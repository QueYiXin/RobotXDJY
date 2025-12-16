from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 声明 launch 参数
    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value='/home/qyx/xdjy_ws/map_1765801095.yaml',
        description='地图文件的完整路径'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value='/home/qyx/xdjy_ws/nav2_params.yaml',
        description='Nav2 参数文件的完整路径'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='是否使用仿真时间'
    )

    rviz2_config_file_arg = DeclareLaunchArgument(
        'rviz2_config',
        default_value='/home/qyx/xdjy_ws/.rviz2/Nav2.rviz',
        description='rviz2配置文件完整路径'
    )

    # 获取 launch 配置
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz2_config_file = LaunchConfiguration('rviz2_config')

    return LaunchDescription([
        # 声明参数
        map_file_arg,
        params_file_arg,
        use_sim_time_arg,
        rviz2_config_file_arg,

        # 1. 启动laser发布
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('basic_driver'),
                    'launch',
                    'livox2laser_launch.py'
                ])
            ])
        ),

        # 2. 启动 Nav2 导航
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'bringup_launch.py'
                ])
            ]),
            launch_arguments={
                'map': map_file,
                'params_file': params_file,
                'use_sim_time': use_sim_time,
            }.items()
        ),

        # 3. 启动 rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz2_config_file],
            parameters=[{
                'use_sim_time': use_sim_time
            }]
        ),
    ])
