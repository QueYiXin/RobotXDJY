#!/usr/bin/env python3
"""
WR1机器人控制程序
功能：发送各种控制指令给机器人
前提：必须先启动 wr1_server.py 并保持运行
用法：python3 wr1_control.py
说明：可以修改此文件中的控制指令，反复运行测试不同动作
"""
import rclpy
from rclpy.node import Node
from xbot_common_interfaces.action import SimpleTrajectory
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from xbot_common_interfaces.msg import HybridJointCommand
from std_msgs.msg import Header
from geometry_msgs.msg import TwistStamped, Twist, Vector3
import time


class WR1Control(Node):
    def __init__(self):
        super().__init__('wr1_control')
        # Action 客户端
        self.traj_client = ActionClient(self, SimpleTrajectory, '/simple_trajectory')
        # Publisher
        self.wr1_pub = self.create_publisher(HybridJointCommand, '/wr1_controller/commands', 10)
        self.hand_pub = self.create_publisher(HybridJointCommand, '/hand_controller/commands', 10)
        self.base_pub = self.create_publisher(TwistStamped, '/wr1_base_drive_controller/cmd_vel', 10)
        # 订阅关节状态
        self.last_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

    def joint_state_callback(self, msg):
        """接收关节状态"""
        self.last_joint_state = msg

    def wait_for_joint_state(self, joint_names, timeout=10.0):
        """等待获取关节状态"""
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

    def move_joint_safely(self, joint_names, target_positions, velocity=0.0,
                         feedforward=0.0, kp=85.0, kd=20.0, step=0.05, wait=0.3):
        """安全地移动关节到目标位置"""
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

    def send_trajectory(self, traj_type, duration=4.0, description=""):
        """发送轨迹指令"""
        self.get_logger().info(f'{description}...')
        goal = SimpleTrajectory.Goal()
        goal.traj_type = traj_type
        goal.duration = duration
        self.traj_client.wait_for_server(timeout_sec=5.0)
        self.get_logger().info(f'发送{description}轨迹...')
        send_goal_future = self.traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'{description}Action Goal未被接受')
            return False
        else:
            rclpy.spin_until_future_complete(self, goal_handle.get_result_async())
            self.get_logger().info(f'{description}完成')
            return True

    def control_hand(self, joint_names, positions, feedforward=350.0, kp=100.0, kd=0.0):
        """控制灵巧手"""
        self.get_logger().info('控制灵巧手动作...')
        hand_cmd = HybridJointCommand()
        hand_cmd.header = Header()
        hand_cmd.header.stamp = self.get_clock().now().to_msg()
        hand_cmd.joint_name = list(joint_names)
        hand_cmd.position = list(positions)
        hand_cmd.velocity = [0.0] * len(joint_names)
        hand_cmd.feedforward = [feedforward] * len(joint_names)
        hand_cmd.kp = [kp] * len(joint_names)
        hand_cmd.kd = [kd] * len(joint_names)
        self.hand_pub.publish(hand_cmd)
        self.get_logger().info('灵巧手控制指令已发送。')

    def control_base(self, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        """控制底盘运动"""
        self.get_logger().info(f'控制底盘运动: linear_x={linear_x}, linear_y={linear_y}, angular_z={angular_z}')
        twist_stamped = TwistStamped()
        twist_stamped.header = Header()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = ''
        twist_stamped.twist = Twist()
        twist_stamped.twist.linear = Vector3(x=linear_x, y=linear_y, z=0.0)
        twist_stamped.twist.angular = Vector3(x=0.0, y=0.0, z=angular_z)
        self.base_pub.publish(twist_stamped)
        self.get_logger().info('底盘运动指令已发送。')

    def run_control_sequence(self):
        """
        执行控制序列
        ==========================================
        在这里修改你想要的控制指令
        ==========================================
        """
        # s_t = time.time()
        # # 1. 发送抬臂轨迹
        # self.send_trajectory(traj_type=2, duration=4.0, description="抬臂")
        # # time.sleep(1)
        # print(f"traj cost time:{time.time()-s_t}")

        # # 2. 小幅度移动小臂关节
        # self.get_logger().info('小幅度移动小臂关节...')
        # arm_joints = [
        #     'left_drv_wheel_joint',
        #     'right_drv_wheel_joint',
        #     'left_shoulder_pitch_joint',
        #     'left_shoulder_roll_joint',
        #     'left_arm_yaw_joint',
        #     'left_elbow_pitch_joint',
        #     'left_elbow_yaw_joint',
        #     'left_wrist_pitch_joint',
        #     'left_wrist_roll_joint',
        #     'right_shoulder_pitch_joint',
        #     'right_shoulder_roll_joint',
        #     'right_arm_yaw_joint',
        #     'right_elbow_pitch_joint',
        #     'right_elbow_yaw_joint',
        #     'right_wrist_pitch_joint',
        #     'right_wrist_roll_joint',
        #     'ankle_joint',
        #     'knee_joint',
        #     'hip_joint',
        #     'waist_yaw_joint',
        #     'neck_yaw_joint',
        #     'neck_pitch_joint'
        #     ]
        # joint_state = self.wait_for_joint_state(arm_joints)
        # if joint_state:
        #     idx_map = {name: idx for idx, name in enumerate(joint_state.name)}
        #     # 在当前位置基础上 +0.05（可修改）
        #     target_pos = [joint_state.position[idx_map[name]] - 0.05 for name in arm_joints]
        #     self.move_joint_safely(
        #         arm_joints,
        #         target_pos,
        #         velocity=0.0,
        #         feedforward=0.0,
        #         kp=85.0,
        #         kd=20.0,
        #         step=0.01,
        #         wait=0.01
        #     )
        # print(f"move cost time:{time.time()-s_t}")
        # time.sleep(1)

        # # 3. 控制灵巧手动作
        # self.control_hand(
        #     joint_names=[
        #         'left_hand_thumb_bend_joint',
        #         'left_hand_thumb_rota_joint1', 
        #         'left_hand_thumb_rota_joint2',
        #         'left_hand_index_bend_joint',
        #         'left_hand_index_joint1', 
        #         'left_hand_index_joint2',
        #         'left_hand_mid_joint1',
        #         'left_hand_mid_joint2', 
        #         'left_hand_ring_joint1',
        #         'left_hand_ring_joint2',
        #         'left_hand_pinky_joint1',
        #         'left_hand_pinky_joint2',
        #         'right_hand_thumb_bend_joint',
        #         'right_hand_thumb_rota_joint1', 
        #         'right_hand_thumb_rota_joint2',
        #         'right_hand_index_bend_joint',
        #         'right_hand_index_joint1', 
        #         'right_hand_index_joint2',
        #         'right_hand_mid_joint1',
        #         'right_hand_mid_joint2', 
        #         'right_hand_ring_joint1',
        #         'right_hand_ring_joint2',
        #         'right_hand_pinky_joint1',
        #         'right_hand_pinky_joint2'
        #         ],
        #     positions=[0.5]*24,
        #     feedforward=350.0,
        #     kp=100.0,
        #     kd=0.0
        # )
        # time.sleep(1)

        # 4. 控制底盘前进和转向
        self.control_base(linear_x=1.0, linear_y=0.0, angular_z=0.3)
        time.sleep(5)

        # self.get_logger().info('='*60)
        # self.get_logger().info('控制序列执行完成！')
        # self.get_logger().info('='*60)


def main(args=None):
    print("="*60)
    print("WR1机器人控制程序")
    print("="*60)
    print("注意：确保 wr1_server.py 已经在运行！")
    print()

    rclpy.init(args=args)
    node = WR1Control()

    try:
        # 执行控制序列
        node.run_control_sequence()
    except KeyboardInterrupt:
        node.get_logger().info('控制被中断')
    except Exception as e:
        node.get_logger().error(f'控制过程出错: {e}')
    finally:
        # 不关闭服务，仅销毁节点
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
