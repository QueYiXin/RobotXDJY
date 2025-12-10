#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.time import Time
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
from std_msgs.msg import Header

class CustomMsgToPointCloud2(Node):
    def __init__(self):
        super().__init__("custommsg_to_pointcloud2")

        self.system_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
        self.time_offset = None  # <--- 之前报错就是因为缺了这行
        self.alpha = 0.01        # <--- 还有这行

        # 订阅 Livox 自定义话题
        self.sub = self.create_subscription(
            CustomMsg,
            "/livox/lidar",
            self.callback,
            10
        )

        # 发布标准 PointCloud2 话题
        self.pub = self.create_publisher(
            PointCloud2,
            "/cloud_in",
            10
        )

        # 定义 PointCloud2 的字段（x/y/z/intensity）
        self.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        self.point_step = 16  # 每个点的字节数（4x4=16）
        self.is_bigendian = False

        self.msg_count = 0
        # self.get_logger().info('CustomMsg to PointCloud2 converter started')
        # self.get_logger().info('Subscribed to: /livox/lidar')
        # self.get_logger().info('Publishing to: /cloud_in')


    def callback(self, msg):
        self.msg_count += 1

        # 检查点云数据
        if msg.point_num == 0 or len(msg.points) == 0:
            self.get_logger().warn(f'Received empty point cloud (point_num={msg.point_num})')
            return

        # 验证点数一致性
        actual_points = len(msg.points)
        if actual_points != msg.point_num:
            self.get_logger().warn(
                f'Point count mismatch: point_num={msg.point_num}, actual={actual_points}'
            )

        # 构造 PointCloud2 消息
        cloud_msg = PointCloud2()

        # 使用原始 Livox 消息的时间戳（保证时间同步）
        cloud_msg.header = msg.header
        cloud_msg.header.stamp = self.get_synced_timestamp(msg.header.stamp)
        
        if not cloud_msg.header.frame_id:
            cloud_msg.header.frame_id = 'livox_frame'

        cloud_msg.height = 1
        cloud_msg.width = actual_points  # 使用实际点数
        cloud_msg.fields = self.fields
        cloud_msg.is_bigendian = self.is_bigendian
        cloud_msg.point_step = self.point_step
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.is_dense = False

        # 拼接点云数据（x/y/z/intensity）- 使用列表推导式 + NumPy
        points_list = [[p.x, p.y, p.z, p.reflectivity / 255.0] for p in msg.points]
        points_array = np.array(points_list, dtype=np.float32)
        cloud_msg.data = points_array.tobytes()

        # 发布标准 PointCloud2 话题
        self.pub.publish(cloud_msg)

        # self.get_logger().info(f'------timestamp={cloud_msg.header.stamp}------')

        # 每 100 帧输出一次信息
        if self.msg_count % 100 == 1:
            self.get_logger().info(
                f'Published #{self.msg_count}: {actual_points} points, '
                f'frame_id={cloud_msg.header.frame_id}'
            )


    def get_synced_timestamp(self, msg_header_stamp):
        """
        核心算法：计算平滑的时间偏差 (Offset)，消除 3s 误差同时保持帧率稳定
        """
        # A. 获取“此刻”的系统时间
        sys_now = self.system_clock.now()
        sys_sec = sys_now.nanoseconds * 1e-9
        
        # B. 获取消息里的“未来”雷达时间
        msg_time = Time.from_msg(msg_header_stamp)
        msg_sec = msg_time.nanoseconds * 1e-9

        # C. 计算瞬时误差 (例如 -3.0s)
        current_diff = sys_sec - msg_sec

        # D. 低通滤波更新 Offset
        if self.time_offset is None:
            self.time_offset = current_diff
            self.get_logger().warn(f'Initial Sync Offset: {self.time_offset:.4f}s')
        else:
            # 仅微调 Offset，过滤掉系统调度的抖动
            self.time_offset = (1 - self.alpha) * self.time_offset + self.alpha * current_diff

        # E. 应用 Offset：将雷达时间平移回现在
        corrected_sec = msg_sec + self.time_offset
        
        return Time(seconds=corrected_sec).to_msg()

def main(args=None):
    rclpy.init(args=args)
    node = CustomMsgToPointCloud2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
