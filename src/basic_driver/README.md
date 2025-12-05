# WR1机器人控制程序使用说明

## 问题描述
原始的 `底层关节控制示例.py` 存在以下问题：
- 每次运行都需要启动关节服务、初始化、激活（过程较慢）
- 运行完成后会自动关闭服务
- 导致每次测试都要等待漫长的启动过程

## 解决方案
将代码拆分为两个独立的程序：

### 1. `wr1_server.py` - 服务启动程序
**功能：**
- 启动关节服务
- 初始化关节模组
- 激活关节模组
- 执行初始复位
- **保持运行状态，不主动关闭**

**特点：**
- 只需启动一次
- 持续运行在后台
- 按 Ctrl+C 才会关闭服务

### 2. `wr1_control.py` - 控制指令程序
**功能：**
- 发送各种控制指令
- 包括轨迹、关节、灵巧手、底盘控制

**特点：**
- 可以反复运行
- 随时修改控制指令
- 不会关闭服务
- 执行快速

## 使用方法

### 第一步：启动服务（一次性）
打开终端1，运行：
```bash
cd /home/qyx/xdjy_ws/dome
python3 wr1_server.py
```

等待看到以下提示：
```
============================================================
WR1机器人服务已就绪！
现在可以在另一个终端运行控制程序: python3 wr1_control.py
按 Ctrl+C 停止服务
============================================================
```

**保持这个终端运行，不要关闭！**

### 第二步：运行控制程序（可重复）
打开终端2，运行：
```bash
cd /home/qyx/xdjy_ws/dome
python3 wr1_control.py
```

控制程序会执行预设的动作序列，完成后自动退出。

### 第三步：修改控制指令
编辑 `wr1_control.py` 文件中的 `run_control_sequence()` 函数：
```python
def run_control_sequence(self):
    """
    在这里修改你想要的控制指令
    """
    # 修改这里的代码来实现不同的动作
    ...
```

保存后，再次运行：
```bash
python3 wr1_control.py
```

无需重启服务，立即执行新的控制指令！

## 可用的控制方法

### 1. 发送轨迹
```python
self.send_trajectory(
    traj_type=2,      # 轨迹类型：2=抬臂, 8=复位
    duration=4.0,     # 持续时间（秒）
    description="抬臂"
)
```

### 2. 移动关节
```python
self.move_joint_safely(
    joint_names=['left_shoulder_pitch_joint', 'right_shoulder_pitch_joint'],
    target_positions=[0.5, 0.5],  # 目标位置
    velocity=0.0,                 # 速度
    kp=85.0,                      # 比例增益
    kd=20.0,                      # 微分增益
    step=0.01,                    # 插值步长
    wait=0.3                      # 每步等待时间
)
```

### 3. 控制灵巧手
```python
self.control_hand(
    joint_names=['right_hand_thumb_bend_joint', 'right_hand_index_bend_joint'],
    positions=[0.5, 0.5],         # 关节位置
    feedforward=350.0,            # 前馈力
    kp=100.0,                     # 比例增益
    kd=0.0                        # 微分增益
)
```

### 4. 控制底盘
```python
self.control_base(
    linear_x=1.0,    # 前进速度
    linear_y=0.0,    # 侧向速度
    angular_z=0.3    # 旋转速度
)
```

## 关闭服务
当完成所有测试后，在终端1按 `Ctrl+C` 关闭服务。

## 优势对比

| 特性 | 原始程序 | 新方案 |
|-----|---------|-------|
| 启动时间 | 每次都需要 | 只需一次 |
| 修改测试 | 需要完整重启 | 立即生效 |
| 开发效率 | 低 | 高 |
| 代码结构 | 耦合 | 解耦 |

## 文件说明
- `底层关节控制示例.py` - 原始文件（保留）
- `wr1_server.py` - 服务启动程序（新）
- `wr1_control.py` - 控制指令程序（新）
- `README.md` - 本说明文档

## 注意事项
1. 必须先启动 `wr1_server.py` 才能运行 `wr1_control.py`
2. `wr1_server.py` 需要保持运行状态
3. 修改 `wr1_control.py` 后无需重启服务
4. 如果服务异常，重启 `wr1_server.py` 即可
