import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String
import subprocess
import time

class GestureDemo(Node):
    def __init__(self):
        super().__init__('gesture_demo')

        # Service clients
        self.cli_activate = self.create_client(Trigger, '/activate_service')
        self.cli_is_play = self.create_client(Trigger, '/gesture/is_play')
        self.cli_stop_play = self.create_client(Trigger, '/gesture/stop_play')

        # Publisher
        self.gesture_pub = self.create_publisher(String, '/gesture/upper_limb_play', 10)

        # 等待service接口可用
        self.get_logger().info('等待服务...')
        self.cli_activate.wait_for_service()
        self.cli_is_play.wait_for_service()
        self.cli_stop_play.wait_for_service()

    def call_service(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def run_sequence(self):
        # # 通过下层SDK启动机器人关节电机
        # # 让机器人进入零位
        # # 将机器人从吊架上放下
        # # 此处省略，请参考底层关节控制部分

        # # 开启算法控制权限
        self.get_logger().info('开启算法控制权限...')
        self.call_service(self.cli_activate, Trigger.Request())

        # # 启动MPC算法
        self.get_logger().info('启动MPC算法...')
        mpc_process = subprocess.Popen(
            ['ros2', 'launch', 'task_client', 'task_mpc.launch.py'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )

        # # 等待MPC启动完成
        self.get_logger().info('等待MPC算法启动...')
        time.sleep(3)

        # # 播放名为 hello 的动作
        self.get_logger().info("播放动作 'hello' ...")
        gesture_msg = String()
        gesture_msg.data = 'hello'
        self.gesture_pub.publish(gesture_msg)

        # # 查询rosbag是否正在播放
        self.get_logger().info("查询rosbag是否正在播放 ...")
        resp = self.call_service(self.cli_is_play, Trigger.Request())
        self.get_logger().info(f'播放状态: success={resp.success}, message="{resp.message}"')

        # # 等待动作播放
        self.get_logger().info('等待动作播放...')
        time.sleep(3)

        # # 停止rosbag播放
        self.get_logger().info('停止rosbag播放 ...')
        resp = self.call_service(self.cli_stop_play, Trigger.Request())
        self.get_logger().info(f'停止播放: success={resp.success}, message="{resp.message}"')

        # # 终止MPC算法
        self.get_logger().info('准备终止MPC算法...')
        mpc_process.terminate()
        try:
            mpc_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            mpc_process.kill()
        self.get_logger().info('MPC算法已终止')

def main(args=None):
    rclpy.init(args=args)
    node = GestureDemo()
    node.run_sequence()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()