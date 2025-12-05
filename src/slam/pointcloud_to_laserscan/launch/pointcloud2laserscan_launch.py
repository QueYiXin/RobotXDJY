from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', '/cloud_in'),
                ('scan', '/scan')
            ],
            parameters=[{
                'target_frame': 'livox_frame',  # 使用实际存在的frame
                'transform_tolerance': 0.1,  # 增加容差到100ms
                'queue_size': 50,  # 增加队列大小，防止丢消息
                'min_height': 0.0,
                'max_height': 1.0,
                'angle_min': -3.14159,  # -π (360度扫描)
                'angle_max': 3.14159,   # π
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.45,
                'range_max': 4.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }]
        )
    ])
