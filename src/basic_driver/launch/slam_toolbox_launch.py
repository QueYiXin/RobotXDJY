from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 获取 slam_toolbox 的配置文件路径
    slam_params_file = PathJoinSubstitution([
        FindPackageShare('slam_toolbox'),
        'config',
        'mapper_params_online_async.yaml'
    ])

    return LaunchDescription([
        # 直接启动 slam_toolbox 节点并添加重映射
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params_file,
                {
                    'use_sim_time': False,
                    'transform_timeout': 0.5,  # 增加TF超时时间（默认0.1秒）
                }
            ],
            remappings=[
                ('/odom', '/wr1_base_drive_controller/odom')  # 重映射：让节点订阅实际话题
            ],
            arguments=['--ros-args', '--log-level', 'info']
        ),
    ])
