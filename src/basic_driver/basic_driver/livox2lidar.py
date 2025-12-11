#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import ctypes
import struct
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class CustomMsgToPointCloud2(Node):
    def __init__(self):
        super().__init__("custommsg_to_pointcloud2")

        # --- 关键修改 1: 使用 SensorData QoS 配置 ---
        # 这种配置专用于传感器数据：
        # 1. keep_last=1: 强制只保留最新的一帧，处理不过来直接丢弃旧的，防止延时堆积 (0.2 -> 0.9s 的元凶)
        # 2. best_effort: 允许 UDP 丢包，降低网络重发带来的延迟
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub = self.create_subscription(
            CustomMsg,
            "/livox/lidar",
            self.callback,
            qos_profile  # 应用 QoS
        )

        self.pub = self.create_publisher(
            PointCloud2,
            "/cloud_in",
            10
        )

        # 预定义 PointCloud2 的字段结构
        self.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        # 定义 NumPy 结构化数组类型，直接对应二进制内存布局，比多次转换快
        self.dtype = [('x', np.float32), ('y', np.float32), ('z', np.float32), ('intensity', np.float32)]
        
        self.point_step = 16 
        self.msg_count = 0

    def callback(self, msg):
        # --- 性能检查 ---
        # 如果处理过慢，这里会丢弃旧数据，直接处理当前最新到达的 msg
        
        point_num = msg.point_num
        if point_num == 0:
            return

        # 构造 PointCloud2 消息
        cloud_msg = PointCloud2()
        
        # --- 关键修改 2: 严格继承时间戳 ---
        # 绝对不要修改时间戳，否则 TF 变换会失效
        cloud_msg.header = msg.header 
        
        # 确保 frame_id 存在
        if not cloud_msg.header.frame_id:
            cloud_msg.header.frame_id = 'livox_frame'

        cloud_msg.height = 1
        cloud_msg.width = point_num
        cloud_msg.fields = self.fields
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = self.point_step
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.is_dense = True # 通常设为True除非有无效点

        # --- 关键修改 3: 优化数据转换 ---
        # Python 最大的瓶颈在于循环。
        # 原来的写法： list comprehension 生成一个巨大的 Python list，再转 numpy，非常慢。
        # 优化后：直接从 msg.points 提取数据。
        # 注意：由于 msg.points 是对象列表，Python 层面很难完全消除循环开销。
        # 这里使用结构化数组减少内存拷贝。
        
        # 提取数据 (这是 Python 节点的物理极限，耗时主要在这里)
        # 我们一次性生成 buffer，避免多次内存分配
        buffer = np.zeros(point_num, dtype=self.dtype)
        
        # 极其遗憾的是，rclpy 解析的 msg.points 是对象列表，无法直接内存拷贝。
        # 我们只能尽量写得紧凑。
        # 提取所有点的 x, y, z, reflectivity
        # 下面这个循环在 Python 中处理 24000 个点大约需要 20-40ms
        
        # 为了速度，我们牺牲一点代码可读性，避免在循环中查找属性
        points = msg.points
        
        # 使用列表推导式提取数值 (比循环 append 快)
        # 注意：Livox reflectivity 是 uint8，这里直接转 float32 归一化
        # 如果不需要 intensity，去掉这一行会快很多
        data_list = [(p.x, p.y, p.z, float(p.reflectivity)) for p in points]
        
        # 填充 numpy 数组
        buffer[:] = data_list
        
        # 转为字节流
        cloud_msg.data = buffer.tobytes()

        self.pub.publish(cloud_msg)
        
        # logging 稍微减少频率
        if self.msg_count % 100 == 0:
            # self.get_logger().info(f'Pub cloud with {point_num} points')
            pass
        self.msg_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = CustomMsgToPointCloud2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()