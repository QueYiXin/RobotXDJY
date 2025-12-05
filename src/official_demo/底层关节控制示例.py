import rclpy
from rclpy.node import Node
from xbot_common_interfaces.srv import DynamicLaunch
from std_srvs.srv import Trigger
from xbot_common_interfaces.action import SimpleTrajectory
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from xbot_common_interfaces.msg import HybridJointCommand
from std_msgs.msg import Header
from geometry_msgs.msg import TwistStamped, Twist, Vector3
import os

import time

class WR1ControlDemo(Node):
    def __init__(self):
        super().__init__('wr1_control_demo')
        # Service
        self.dyn_launch_cli = self.create_client(DynamicLaunch, '/dynamic_launch')
        self.ready_cli = self.create_client(Trigger, '/ready_service')
        self.activate_cli = self.create_client(Trigger, '/activate_service')
        self.stop_cli = self.create_client(Trigger, '/stop_launch')
        # Action
        self.traj_client = ActionClient(self, SimpleTrajectory, '/simple_trajectory')
        # Publisher
        self.wr1_pub = self.create_publisher(HybridJointCommand, '/wr1_controller/commands', 10)
        self.hand_pub = self.create_publisher(HybridJointCommand, '/hand_controller/commands', 10)
        self.base_pub = self.create_publisher(TwistStamped, '/wr1_base_drive_controller/cmd_vel', 10)
        # State
        self.last_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

    def call_service(self, client, req):
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def joint_state_callback(self, msg):
        self.last_joint_state = msg

    def wait_for_joint_state(self, joint_names, timeout=10.0):
        start = time.time()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.last_joint_state:
                # 检查包含需要的关节名
                if all(j in self.last_joint_state.name for j in joint_names):
                    return self.last_joint_state
            if time.time() - start > timeout:
                self.get_logger().warn('未及时获取到关节状态，可能数据流未发布')
                while rclpy.ok():
                    rclpy.spin_once(self, timeout_sec=0.1)
                return None
        return None

    def move_joint_safely(self, joint_names, target_positions, velocity=0.0, feedforward=0.0, kp=85.0, kd=20.0, step=0.05, wait=0.3):
        # 获取当前关节位置
        self.get_logger().info('等待获取关节状态...')
        joint_state = self.wait_for_joint_state(joint_names)
        if not joint_state:
            self.get_logger().error('未获得关节状态，无法执行关节移动！')
            return
        idx_map = {name: idx for idx, name in enumerate(joint_state.name)}
        current_pos = [joint_state.position[idx_map[name]] for name in joint_names]

        # 逐步插值逼近目标
        steps = int(max(abs(tp-cp) for tp, cp in zip(target_positions, current_pos)) // step) + 1
        for s in range(steps):
            pos = [
                cp + (tp-cp) * min((s+1)/steps, 1.0)
                for cp, tp in zip(current_pos, target_positions)
            ]
            cmd = HybridJointCommand()
            cmd.header = Header()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.joint_name = list(joint_names)
            cmd.position = pos
            cmd.velocity = [velocity] * len(joint_names)
            cmd.feedforward = [feedforward] * len(joint_names)
            cmd.kp = [kp] * len(joint_names)
            cmd.kd = [kd] * len(joint_names)
            self.wr1_pub.publish(cmd)
            self.get_logger().info(f'关节插值步进: {pos}')
            time.sleep(wait)
        self.get_logger().info('目标位置已到达。')

    def run_sequence(self):
        # 启动关节服务
        self.get_logger().info('等待 dynamic_launch 服务...')
        self.dyn_launch_cli.wait_for_service()
        self.get_logger().info('启动关节服务...')
        dyn_req = DynamicLaunch.Request()
        dyn_req.app_name = ''
        dyn_req.sync_control = False
        dyn_req.launch_mode = 'pos'
        self.call_service(self.dyn_launch_cli, dyn_req)

        # 初始化关节模组
        self.get_logger().info('等待 ready_launch 服务...')
        self.ready_cli.wait_for_service()
        self.get_logger().info('初始化关节模组...')
        self.call_service(self.ready_cli, Trigger.Request())

        # 激活关节模组
        self.get_logger().info('等待 activate_launch 服务...')
        self.activate_cli.wait_for_service()
        self.get_logger().info('激活关节模组...')
        self.call_service(self.activate_cli, Trigger.Request())

        # 发送关节复位轨迹
        self.get_logger().info('等待 simple_trajectory 服务...')
        self.traj_client.wait_for_server()
        self.get_logger().info('复位...')
        goal = SimpleTrajectory.Goal()
        goal.traj_type = 8
        goal.duration = 4.0
        self.get_logger().info('等待发送复位轨迹...')
        self.traj_client.wait_for_server()
        self.get_logger().info('发送复位轨迹...')
        send_goal_future = self.traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('复位Action Goal未被接受')
        else:
            rclpy.spin_until_future_complete(self, goal_handle.get_result_async())
        time.sleep(1)

        # 发送关节抬臂轨迹
        self.get_logger().info('抬臂...')
        goal = SimpleTrajectory.Goal()
        goal.traj_type = 2
        goal.duration = 4.0
        self.get_logger().info('等待发送抬臂轨迹...')
        self.traj_client.wait_for_server()
        self.get_logger().info('发送抬臂轨迹...')
        send_goal_future = self.traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('抬臂Action Goal未被接受')
        else:
            rclpy.spin_until_future_complete(self, goal_handle.get_result_async())
        time.sleep(1)

        # 控制机器人关节运动（例：让小臂抬起一点，实际请根据安全策略和需要调整）
        self.get_logger().info('小幅度移动小臂关节...')
        arm_joints = ['left_shoulder_pitch_joint', 'right_shoulder_pitch_joint']
        # 假设目标为当前基础上+0.05（示例），实际请按需求设定目标
        joint_state = self.wait_for_joint_state(arm_joints)
        if joint_state:
            idx_map = {name: idx for idx, name in enumerate(joint_state.name)}
            target_pos = [joint_state.position[idx_map[name]] + 0.05 for name in arm_joints]
            self.move_joint_safely(arm_joints, target_pos, velocity=0.0, feedforward=0.0, kp=85.0, kd=20.0, step=0.01, wait=0.3)
        time.sleep(1)

        # 控制机器人灵巧手动作
        self.get_logger().info('控制灵巧手动作...')
        hand_cmd = HybridJointCommand()
        hand_cmd.header = Header()
        hand_cmd.header.stamp = self.get_clock().now().to_msg()
        hand_cmd.joint_name = ['right_hand_thumb_bend_joint', 'right_hand_thumb_rota_joint2', 'right_hand_index_bend_joint']
        hand_cmd.position = [0.5, 0.5, 0.5]
        hand_cmd.velocity = [0.0, 0.0, 0.0]
        hand_cmd.feedforward = [350.0, 350.0, 350.0]
        hand_cmd.kp = [100.0, 100.0, 100.0]
        hand_cmd.kd = [0.0, 0.0, 0.0]
        self.get_logger().info('发送灵巧手控制指令...')
        self.hand_pub.publish(hand_cmd)
        self.get_logger().info('灵巧手控制指令已发送。')
        time.sleep(1)

        # 控制底盘前进和转向
        self.get_logger().info('控制底盘前进和转向...')
        twist_stamped = TwistStamped()
        twist_stamped.header = Header()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = ''
        twist_stamped.twist = Twist()
        twist_stamped.twist.linear = Vector3(x=1.0, y=0.0, z=0.0)
        twist_stamped.twist.angular = Vector3(x=0.0, y=0.0, z=0.3)
        self.get_logger().info('发送底盘运动指令...')
        self.base_pub.publish(twist_stamped)
        self.get_logger().info('底盘运动指令已发送。')
        time.sleep(1)

        # 停止关节服务
        self.get_logger().info('等待 stop_launch 服务...')
        self.stop_cli.wait_for_service()
        self.get_logger().info('停止关节服务...')
        self.call_service(self.stop_cli, Trigger.Request())
        self.get_logger().info('演示序列已完成。')
        time.sleep(1)

def main(args=None):
    print("ROS 相关环境变量：")
    for key, value in os.environ.items():
        if "ROS" in key or "RMW" in key or "CYCLONEDDS" in key:
            print(f"{key} = {value}")

    rclpy.init(args=args)
    node = WR1ControlDemo()
    node.run_sequence()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()