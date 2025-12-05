import sounddevice as sd
import numpy as np
from scipy.signal import resample

duration = 2.0  # 秒

# 根据需求调整输入和输出设备
sd.default.device = (6, 6)  # Q5默认的音频设备是6号

# 获取默认设备索引
input_device, output_device = sd.default.device

# 获取设备参数
input_info = sd.query_devices(input_device)
output_info = sd.query_devices(output_device)

input_samplerate = int(input_info['default_samplerate'])
output_samplerate = int(output_info['default_samplerate'])
channels = input_info['max_input_channels']

print(f"🎤 正在从默认输入设备（设备 {input_device}）录音 {duration} 秒 @ {input_samplerate} Hz, 通道数: {channels}")

sd.default.device = (input_device, output_device)



# 录音
recording = sd.rec(int(input_samplerate * duration), samplerate=input_samplerate, channels=channels, dtype='int16')
sd.wait()

# 音量放大，请根据实际情况调整
amplified = recording * 1
amplified = np.clip(amplified, -32768, 32767).astype(np.int16)

# 如果播放采样率和录音不同，则升/降采样
if input_samplerate != output_samplerate:
    print(f"🔁 正在将录音采样率从 {input_samplerate} Hz 转为 {output_samplerate} Hz 以便播放")
    num_samples = int(output_samplerate * duration)
    amplified = resample(amplified, num_samples).astype(np.int16)

print(f"🔊 正在从默认输出设备（设备 {output_device}）播放...")
sd.play(amplified, samplerate=output_samplerate)
sd.wait()

print("✅ 播放完成")