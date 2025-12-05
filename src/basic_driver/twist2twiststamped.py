import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Header

class TwistToTwistStamped(Node):
    def __init__(self):
        super().__init__("twist_to_twiststamped")
        # 订阅键盘控制发布的 Twist 话题（/cmd_vel_raw）
        self.sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.callback,
            10
        )
        # 发布转换后的 TwistStamped 话题（机器人需要的目标话题）
        self.pub = self.create_publisher(
            TwistStamped,
            "/wr1_base_drive_controller/cmd_vel",
            10
        )
        # 可配置坐标系（根据机器人实际情况修改）
        self.declare_parameter("frame_id", "base_link")
        self.frame_id = self.get_parameter("frame_id").value

    def callback(self, twist_msg):
        # 构造 TwistStamped 消息
        twist_stamped_msg = TwistStamped()
        # 填充 header（时间戳 + 坐标系）
        twist_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        twist_stamped_msg.header.frame_id = self.frame_id
        # 填充核心运动数据（直接复用 Twist 消息）
        twist_stamped_msg.twist = twist_msg
        # 发布消息
        self.pub.publish(twist_stamped_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistToTwistStamped()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


# Run "ros2 run teleop_twist_keyboard teleop_twist_keyboard" before this program

if __name__ == "__main__":
    main()
