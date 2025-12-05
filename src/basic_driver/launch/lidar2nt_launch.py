from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Livox CustomMsg 转 PointCloud2 节点
        Node(
            package='basic_driver',
            executable='lidar2nt',
            name='lidar2nt',
            output='screen',
            parameters=[],
            remappings=[]
        ),
        # 静态坐标变换发布器：map -> livox_frame (重合)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_link_to_livox_frame',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'lidar_link', '--child-frame-id', 'livox_frame'
            ]
        )
    ])
