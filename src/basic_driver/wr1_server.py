#!/usr/bin/env python3
"""
WR1机器人服务启动程序
功能：启动关节服务、初始化、激活，并保持运行
用法：python3 wr1_server.py
说明：此程序需要保持运行，不要关闭。启动后可在另一个终端运行控制程序。
"""
import rclpy
from rclpy.node import Node
from xbot_common_interfaces.srv import DynamicLaunch
from std_srvs.srv import Trigger
from xbot_common_interfaces.action import SimpleTrajectory
from rclpy.action import ActionClient
import os
import time


class WR1Server(Node):
    def __init__(self):
        super().__init__('wr1_server')
        # 创建服务客户端
        self.dyn_launch_cli = self.create_client(DynamicLaunch, '/dynamic_launch')
        self.ready_cli = self.create_client(Trigger, '/ready_service')
        self.activate_cli = self.create_client(Trigger, '/activate_service')
        self.stop_cli = self.create_client(Trigger, '/stop_launch')
        # Action 客户端（用于复位）
        self.traj_client = ActionClient(self, SimpleTrajectory, '/simple_trajectory')

    def call_service(self, client, req):
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def startup_sequence(self):
        """启动关节服务的完整流程"""
        # 1. 启动关节服务
        self.get_logger().info('等待 dynamic_launch 服务...')
        self.dyn_launch_cli.wait_for_service()
        self.get_logger().info('启动关节服务...')
        dyn_req = DynamicLaunch.Request()
        dyn_req.app_name = ''
        dyn_req.sync_control = False
        dyn_req.launch_mode = 'pos'
        result = self.call_service(self.dyn_launch_cli, dyn_req)
        self.get_logger().info(f'关节服务启动结果: {result}')

        # 2. 初始化关节模组
        self.get_logger().info('等待 ready_service 服务...')
        self.ready_cli.wait_for_service()
        self.get_logger().info('初始化关节模组...')
        result = self.call_service(self.ready_cli, Trigger.Request())
        self.get_logger().info(f'初始化结果: {result}')

        # 3. 激活关节模组
        self.get_logger().info('等待 activate_service 服务...')
        self.activate_cli.wait_for_service()
        self.get_logger().info('激活关节模组...')
        result = self.call_service(self.activate_cli, Trigger.Request())
        self.get_logger().info(f'激活结果: {result}')

        # 4. 发送关节复位轨迹
        self.get_logger().info('等待 simple_trajectory 服务...')
        self.traj_client.wait_for_server()
        self.get_logger().info('发送复位轨迹...')
        goal = SimpleTrajectory.Goal()
        goal.traj_type = 8
        goal.duration = 4.0
        send_goal_future = self.traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('复位Action Goal未被接受')
        else:
            self.get_logger().info('复位中...')
            rclpy.spin_until_future_complete(self, goal_handle.get_result_async())
            self.get_logger().info('复位完成')
        time.sleep(1)

        self.get_logger().info('='*60)
        self.get_logger().info('WR1机器人服务已就绪！')
        self.get_logger().info('现在可以在另一个终端运行控制程序: python3 wr1_control.py')
        self.get_logger().info('按 Ctrl+C 停止服务')
        self.get_logger().info('='*60)

    def shutdown_sequence(self):
        """关闭服务（仅在程序退出时调用）"""
        print('\n' + '='*60)
        print('开始关闭关节服务...')
        print('='*60)
        try:
            if not self.context.ok():
                print('警告: ROS2 context 已失效，跳过关闭')
                return

            # 等待停止服务可用
            print('等待 stop_launch 服务...')
            service_available = self.stop_cli.wait_for_service()

            if service_available:
                print('stop_launch 服务可用，发送关闭请求...')
                result = self.call_service(self.stop_cli, Trigger.Request())
                print(f'关闭服务结果: {result}')
                print('关节服务已成功关闭')
            else:
                print('警告: stop_launch 服务超时(5秒)，尝试强制关闭...')
                # 尝试强制调用
                try:
                    result = self.call_service(self.stop_cli, Trigger.Request())
                    print(f'强制关闭结果: {result}')
                except Exception as e:
                    print(f'强制关闭失败: {e}')
        except Exception as e:
            print(f'关闭服务时出错: {e}')
            import traceback
            traceback.print_exc()
        finally:
            print('='*60)
            print('关闭流程结束')
            print('='*60)


def main(args=None):
    print("="*60)
    print("WR1机器人服务启动程序")
    print("="*60)
    print("\nROS 相关环境变量：")
    for key, value in os.environ.items():
        if "ROS" in key or "RMW" in key or "CYCLONEDDS" in key:
            print(f"{key} = {value}")
    print()

    rclpy.init(args=args)
    node = WR1Server()

    try:
        # 执行启动序列
        node.startup_sequence()
        # 保持运行
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n收到退出信号，正在安全关闭...')
        # 在 context 还有效时关闭服务
        node.shutdown_sequence()
    except Exception as e:
        print(f'运行出错: {e}')
        node.shutdown_sequence()
    finally:
        # 清理资源
        try:
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
