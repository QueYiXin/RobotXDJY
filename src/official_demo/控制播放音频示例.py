import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from xbot_common_interfaces.srv import SetVolume
from xbot_common_interfaces.action import AudioPlay
from rclpy.action import ActionClient
import time

class AudioPlayerDemo(Node):
    def __init__(self):
        super().__init__('audio_player_demo')

        # Service clients
        self.cli_set_volume = self.create_client(SetVolume, '/audio_player/set_volume')
        self.cli_is_play = self.create_client(Trigger, '/audio_player/is_play')
        self.cli_stop_play = self.create_client(Trigger, '/audio_player/stop_play')

        # Action client
        self.audio_play_client = ActionClient(self, AudioPlay, '/audio_player/play')

        # 等待service/action可用
        self.get_logger().info('等待服务和action...')
        self.cli_set_volume.wait_for_service()
        self.cli_is_play.wait_for_service()
        self.cli_stop_play.wait_for_service()
        self.audio_play_client.wait_for_server()

    def call_service(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def play_audio(self, mode=0, force_play=False, id=5, path='', timeout=0):
        goal = AudioPlay.Goal()
        goal.mode = mode
        goal.force_play = force_play
        goal.id = id
        goal.path = path
        goal.timeout = timeout

        send_goal_future = self.audio_play_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('音频播放请求未被接受')
            return
        # 等待播放完成（也可以读取反馈，见下）
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result().result
        self.get_logger().info(f'音频播放结果: success={result.success}, message={result.message}')

    def run_sequence(self):
        # 1. 设置音量为50
        self.get_logger().info('设置音量为50...')
        vol_req = SetVolume.Request()
        vol_req.volume = 50
        resp = self.call_service(self.cli_set_volume, vol_req)
        self.get_logger().info(f'音量设置: success={resp.success}, message="{resp.message}"')

        # 2. 通过id播放音频（id=5）
        self.get_logger().info('通过id播放音频（id=5）...')
        self.play_audio(mode=0, force_play=False, id=5, timeout=0)

        # # 如需通过路径播放音频，取消下行注释
        # self.get_logger().info('通过路径播放指定音频...')
        # self.play_audio(mode=1, force_play=True, id=0, path='/xos/xos/data/audio/replay_wav/hello.wav', timeout=0)

        # 3. 查询音频是否正在播放
        self.get_logger().info('查询音频是否正在播放...')
        resp = self.call_service(self.cli_is_play, Trigger.Request())
        self.get_logger().info(f'是否正在播放: success={resp.success}, message="{resp.message}"')

        # 4. 等待音频播放
        self.get_logger().info('等待音频播放...')
        time.sleep(3)

        # 5. 停止音频播放
        self.get_logger().info('停止音频播放...')
        resp = self.call_service(self.cli_stop_play, Trigger.Request())
        self.get_logger().info(f'停止播放: success={resp.success}, message="{resp.message}"')

def main(args=None):
    rclpy.init(args=args)
    node = AudioPlayerDemo()
    node.run_sequence()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()