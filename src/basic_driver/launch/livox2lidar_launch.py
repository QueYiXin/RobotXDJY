from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 1. Livox CustomMsg 转 PointCloud2 节点
        Node(
            package='basic_driver',
            executable='livox2lidar',
            name='livox2lidar',
            output='screen',
            parameters=[],
            remappings=[]
        ),
        # 2. 静态坐标变换发布器：lidar_link -> livox_frame (重合)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_link_to_livox_frame',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'lidar_link', '--child-frame-id', 'livox_frame'
            ]
        ),
        # 3. 发布静态TF: base_link -> base_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_livox',
            # arguments: [x y z qx qy qz parent child]
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_frame'],
            output='screen'
        ),
        # 4. 发布静态TF: odom -> odom_frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'odom_frame'],
            output='screen'
        ),
    ])
