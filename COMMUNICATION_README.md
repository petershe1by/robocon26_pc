# 上位机 ↔ 下位机 通信协议完整说明

> 本文档统一定义上位机 ROS2 与下位机 STM32 之间的全部通信协议。
> 包含：USB CDC 虚拟遥控器 0x10 控制协议、USB 0x11 IMU 姿态协议、机械臂 UART 文本协议。

---

## 一、通信通道总览

| 通道 | 物理层 | 协议 | 方向 | 用途 |
| --- | --- | --- | --- | --- |
| 运动控制 | USB CDC 虚拟串口 | 自定义 40 字节帧，50 Hz | PC → STM32 | 运动指令、模式、IMU 姿态 |
| 机械臂控制 | UART TTL | ASCII 文本协议，115200 8N1 | PC → 机械臂控制器 | 抓放命令、吸盘控制 |
| ROS2 内部 | DDS topics/services | msg/srv | 节点间 | 导航指令、状态反馈 |

USB CDC 波特率固定 115200。

---

## 二、USB 帧结构（通用包装）

两种消息类型使用相同的 40 字节帧包装：

```
offset  size  field
──────  ────  ─────────────────────
0-1     2     magic:      A5 5A
2-3     2     msg_type:   01 10 (控制) / 01 11 (IMU)
4-5     2     reserved:   00 00
6-7     2     frame_seq:  uint16 LE, 递增
8-9     2     payload_len: 1C 00 (28, 固定)
10-37   28    payload:    消息体
38-39   2     crc16:      CRC-16/CCITT-FALSE
```

### CRC-16/CCITT-FALSE

多项式 `0x1021`，初始 `0xFFFF`，计算范围从 magic 到 payload 末尾（前 38 字节）：

```python
def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000: crc = (crc << 1) ^ 0x1021
            else: crc <<= 1
            crc &= 0xFFFF
    return crc
```

---

## 三、消息类型 0x10 — VIRTUAL_RC_SETPOINT（运动控制）

50 Hz 发送。用于向 STM32 发送运动指令、模式切换、使能/急停。

### 3.1 Payload 格式（28 字节）

python 编码：`struct.Struct("<IIBBHhhhhhHI")`

| 偏移 | 类型 | 字段 | 说明 |
| --- | --- | --- | --- |
| 0 | uint32 | session_id | USB ACQUIRE 获得的会话 ID |
| 4 | uint32 | host_time_ms | 上位机单调时钟 ms，仅诊断 |
| 8 | uint8 | main_switch | 九宫格主模式：0/1/2 |
| 9 | uint8 | sub_switch | 九宫格子模式：0/1/2 |
| 10 | uint16 | command_flags | 位标志（见 3.3） |
| 12 | int16 | yaw_permille | CH1 等价，-1000~1000，负左正右 |
| 14 | int16 | forward_permille | CH2 等价，-1000~1000，负退正进 |
| 16 | int16 | speed_permille | CH3 等价，-1000~1000，同时决定步态档和轮速 |
| 18 | int16 | arm_j0_permille | CH6 等价，机械臂 J0 |
| 20 | int16 | arm_j1_permille | CH7 等价，机械臂 J1 |
| 22 | uint16 | channel_valid_mask | 默认 0x0067（CH1/2/3/6/7） |
| 24 | uint32 | command_counter | session 内严格递增 |

### 3.2 九宫格模式

virtual_mode = main_switch × 3 + sub_switch：

| 编号 | CH5+CH8 | 枚举 | 行为 |
| --- | --- | --- | --- |
| 0 | LOW+LOW | IDLE | 无动作，轮驱 OFF |
| 1 | LOW+MID | LOW_WHEEL | 低姿态轮行 |
| 2 | LOW+HIGH | SAFE_HOLD | 腿 RX-only，轮驱 HOLD |
| 3 | MID+LOW | STAND_HOLD | MIT 站立，轮驱 HOLD |
| 4 | MID+MID | STAND_WHEEL | 站立轮行 |
| 5 | MID+HIGH | STAND_ARM | 站立 + 机械臂 jog |
| 6 | HIGH+LOW | GAIT_ONLY | 纯足步态，轮驱 HOLD |
| 7 | HIGH+MID | GAIT_WHEEL | 步态 + 轮驱同步 |
| 8 | HIGH+HIGH | RESERVED | 保留，站立 HOLD |

### 3.3 command_flags 位定义

| 位 | 名称 | 下位机行为 |
| --- | --- | --- |
| 0 | DEADMAN_HELD | 自主许可/人工持续使能，允许连续非零通道 |
| 1 | MOTION_ENABLE | 允许进入非 IDLE 模式 |
| 2 | SMOOTH_STOP | CH1/CH2 强制归零并请求平稳收步 |

MOTION_ENABLE=0 时强制 IDLE。DEADMAN=0 时通道归零。两位置位才允许非零运动。

### 3.4 上位机 ROS2 → USB 映射

```
/navigation_node 发布的 /motion_cmd (MotionCmd)：
  linear_x (-1~1)  → forward_permille (-1000~1000)
  angular_z (-1~1) → yaw_permille (-1000~1000)
  gait_mode (0/1/2) → speed_permille (-800/0/800)
  有运动? HIGH+LOW(步态) / 静止? MID+LOW(站立)
  enable=true → DEADMAN_HELD + MOTION_ENABLE
  estop=true  → emergency_stop (清零所有)

/enable_motion true  → MID+LOW + DEADMAN + MOTION_ENABLE
/enable_motion false → emergency_stop + LOW+LOW
/estop true          → emergency_stop
```

### 3.5 安全零帧

模式切换、暂停、释放时发送：

```
main_switch=0, sub_switch=0
yaw=0, forward=0, speed=-1000
flags=DEADMAN=0, MOTION_ENABLE=0, SMOOTH_STOP=0
valid_mask=0x0067
```

### 3.6 联调测试向量

```
session=0x11223344, host=0x01020304, HIGH+LOW, flags=0x0003
CH1=-250, CH2=500, CH3=-800, CH6/7=0, mask=0x0067, counter=10, frame_seq=0x1234

A5 5A 01 10 00 00 34 12 1C 00 44 33 22 11 04 03 02 01
02 00 03 00 06 FF F4 01 E0 FC 00 00 00 00 67 00 0A 00
00 00 CF 96
```

---

## 四、消息类型 0x11 — VIRTUAL_IMU（IMU 姿态数据）

25 Hz 发送，每 2 个 0x10 帧后插入 1 帧 0x11。数据来自 imu_filter 节点对 Mid-360 原始 IMU 的处理。

### 4.1 Payload 格式（28 字节）

python 编码：`struct.Struct("<6fI")`

| 偏移 | 类型 | 字段 | 单位 | 说明 |
| --- | --- | --- | --- | --- |
| 0 | float32 | roll | 度 | 横滚角 ±180°，互补滤波 |
| 4 | float32 | pitch | 度 | 俯仰角 ±90°，互补滤波 |
| 8 | float32 | yaw | 度 | 航向角 0~360°，陀螺仪积分 |
| 12 | float32 | gyro_x | rad/s | X 轴角速度 |
| 16 | float32 | gyro_y | rad/s | Y 轴角速度 |
| 20 | float32 | gyro_z | rad/s | Z 轴角速度 |
| 24 | uint32 | timestamp_ms | ms | 上位机单调时钟 |

### 4.2 滤波处理链路

```
Mid-360 IMU (200 Hz raw)
  → 4 帧滑动窗口均值（抑制气泵 20-50Hz 振动）
  → 互补滤波 alpha=0.98（加速度计定 roll/pitch，陀螺仪积分得 yaw）
  → /imu_orientation topic
  → motion_control_node 订阅
  → USB 0x11 帧 (25 Hz)
  → STM32
```

### 4.3 联调测试向量

```
roll=30.5°, pitch=-10.2°, yaw=45.0°
gyro=(0.1, -0.2, 0.3) rad/s
timestamp=0x12345678, frame_seq=1

A5 5A 01 11 00 00 01 00 1C 00 00 00 F4 42 33 33
24 C1 00 00 34 42 CD CC CC 3D 9A 99 59 BE 9A 99
99 3E 78 56 34 12 [CRC 待生成]
```

---

## 五、机械臂 UART 文本协议

115200 baud，8N1，命令以 `\n` 结尾。

| 命令 | 用途 | 触发条件 |
| --- | --- | --- |
| `HOME\n` | 回到待机位 | 初始化 / 放置完成 |
| `VISION_SCAN\n` | 视觉扫描姿态 | 导航到物块附近 |
| `VISION_AID\n` | 辅助定位姿态 | YOLO 识别到物块 |
| `GRASP\n` | 抓取姿态 | 深度确认到位 |
| `PLACE\n` | 放置姿态 | 到兑换站 |
| `SUCTION_ON\n` | 开启吸盘 | GRASP 后 |
| `SUCTION_OFF\n` | 关闭吸盘 | PLACE 后 |

---

## 六、数据流全景

```
┌────────────────────────────────────────────────────────────┐
│                        上位机 (ROS2)                        │
│                                                             │
│  navigation_node                                            │
│    └─ /motion_cmd ──→ motion_control_node                   │
│                         └─ VirtualRemoteOutput              │
│                              ├─ 0x10 CONTROL (50Hz)          │
│                              └─ 0x11 IMU (25Hz)              │
│                                   │                          │
│  imu_filter_node                                            │
│    └─ /imu_orientation ────────┘                            │
│                                                             │
│  arm_control_node                                            │
│    └─ UART text command ──→ 机械臂控制器                      │
└──────────────────────┬──────────────────────────────────────┘
                       │ USB CDC 115200
                       ▼
┌────────────────────────────────────────────────────────────┐
│                      下位机 (STM32)                         │
│                                                             │
│  UsbFrame_Process()                                         │
│    ├─ 0x10: 更新 VirtualRcSample → RemoteControlView        │
│    │   → sbus_mode_tick() / sbus_drive_input()              │
│    └─ 0x11: 更新 IMU 数据 → 可选姿态控制                    │
│                                                             │
│  九宫格状态机 / MIT 步态 / 轮腿控制                          │
└────────────────────────────────────────────────────────────┘
```

## 七、重要的设计约束

| 约束 | 说明 |
| --- | --- |
| 频率 | 0x10 帧必须稳定 50 Hz，IMU 帧 25 Hz |
| 波特率 | USB CDC 固定 115200 |
| CH9 安全 | USB 不能模拟 CH9/SD，实体遥控始终有最高优先级 |
| 急停 | 安全零帧不够，必须发 ESTOP_REQ |
| 模式切换 | 切换模式前需要先发安全零帧，等待状态确认 |
| 通道未置位 | `valid_mask` 未置位通道必须按 0 处理 |

---

## 八、ROS2 话题与节点关系

| 节点 | 话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| navigation_node | `/motion_cmd` | MotionCmd | 导航输出的运动指令 |
| task_scheduler | `/enable_motion` | Bool | 全局使能 |
| navigation_node | `/estop` | Bool | 急停 |
| imu_filter | `/livox/imu` | Imu (sub) | 雷达原始 IMU |
| imu_filter | `/imu_filtered` | Imu (pub) | 滤波后 IMU（供 EKF） |
| imu_filter | `/imu_orientation` | Float32MultiArray | [r,p,y,gx,gy,gz] |
| motion_control | `/imu_orientation` | Float32MultiArray (sub) | 接收 IMU → USB |
| motion_control | `/motion_enabled` | Bool | 反馈当前使能状态 |
