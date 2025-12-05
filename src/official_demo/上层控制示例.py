import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import subprocess
import time
import textwrap

class MPCStartupDemo(Node):
    def __init__(self):
        super().__init__('mpc_startup_demo')
        self.cli_activate = self.create_client(Trigger, '/activate_service')
        self.cli_activate.wait_for_service()

    def call_activate_service(self):
        self.get_logger().info('开启算法控制权限...')
        req = Trigger.Request()
        future = self.cli_activate.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        if resp.success:
            self.get_logger().info('算法控制权限开启成功')
        else:
            self.get_logger().error(f'开启失败: {resp.message}')

    def launch_mpc(self):
        self.get_logger().info('启动MPC算法...')
        # 启动ros2 launch进程（注意：如需交互终端请用shell=True，或使用subprocess.Popen异步模式）
        # 这里用Popen，可Ctrl+C终止
        subprocess.run(["python", "q5_pub_client.py", "--type", "mpc", "--cmd", "start"])
        self.get_logger().info('MPC算法已启动（如需停止请手动Ctrl+C终止此脚本）')

    def publish_servo_pose_rate(self, rate_hz: int = 10):
        """以指定频率连续发布抬手位姿到 /servo_poses（默认 10Hz）"""
        message = textwrap.dedent(
            """
            {
              left_pose: {
                header: {frame_id: 'base_link'},
                pose: {position: {x: 0.425, y: 0.3, z: 0.292}, orientation: {x: 0.612, y: -0.016, z: 0.79, w: -0.006}}
              },
              right_pose: {
                header: {frame_id: 'base_link'},
                pose: {position: {x: 0.417, y: -0.146, z: 0.382}, orientation: {x: 0.521, y: 0.1, z: 0.842, w: -0.096}}
              },
              head_pose: {
                header: {frame_id: 'base_link'},
                pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}
              }
            }
            """
        ).strip()
        self.get_logger().info(f'以 {rate_hz}Hz 频率发布 /servo_poses 抬手指令...')
        subprocess.run([
            'ros2', 'topic', 'pub', '-r', str(rate_hz),
            '/servo_poses', 'xbot_common_interfaces/msg/ServoPose',
            message
        ], check=True)

def main(args=None):
    rclpy.init(args=args)
    node = MPCStartupDemo()
    node.call_activate_service()
    # 暂停1秒确保服务完全生效
    time.sleep(1)
    node.launch_mpc()
    node.publish_servo_pose_rate()
    node.destroy_node()
    rclpy.spin(node)
    rclpy.shutdown()
    subprocess.run(["python", "q5_pub_client.py", "--type", "mpc", "--cmd", "stop"])

if __name__ == '__main__':
    main()
