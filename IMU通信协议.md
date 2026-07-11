# 上位机 IMU 姿态数据处理与通信协议

> 本文定义上位机 Mid-360 原始 IMU 数据的处理链路，以及向下位机发送姿态信息的 USB 0x11 协议。

---

## 一、数据流全景

```
Mid-360 IMU (200 Hz)
    │ raw /livox/imu topic
    ▼
imu_filter.py
    ├── 4 帧滑动窗口均值 (200→50 Hz)   ← 抑制气泵振动
    ├── 互补滤波 (alpha=0.98)          ← 加速度计定 roll/pitch
    └── gyro_z 积分得 yaw              ← 航向死推算（180 秒可接受）
    │ published /imu_orientation
    ▼
motion_control_node
    │ VIRTUAL_IMU 0x11 帧 (25 Hz)
    ▼
USB CDC → 下位机 STM32
```

## 二、IMU 滤波处理（上位机侧）

### 2.1 滑动窗口均值

```
窗口大小:  4 帧
原始频率: 200 Hz
输出频率: 200 Hz（原样输出，但每帧都是 4 帧均值）
作用:     抑制 20-50 Hz 气泵振动，保留 1.5-2.5 Hz 步态信号
窗口时间: 20 ms（≈ 半个气泵振动周期以上）
```

### 2.2 互补滤波

```
roll  = α × (roll_prev + gyro_x × dt) + (1-α) × atan2(accel_y, accel_z)
pitch = α × (pitch_prev + gyro_y × dt) + (1-α) × atan2(-accel_x, √(accel_y²+accel_z²))
yaw   = yaw_prev + gyro_z × dt                                   — 仅陀螺仪积分

α = 0.98（信任陀螺仪 98%，加速度计 2% 修正漂移）
```

### 2.3 输出话题

| 话题 | 类型 | 频率 | 说明 |
| --- | --- | --- | --- |
| `/imu_filtered` | `sensor_msgs/Imu` | 200 Hz | 滤波后的 IMU（给 EKF 用） |
| `/imu_orientation` | `std_msgs/Float32MultiArray` | 50 Hz | 6 个 float：[roll°, pitch°, yaw°, gyro_x, gyro_y, gyro_z] |

Float32MultiArray.data 索引：

| 索引 | 字段 | 单位 | 范围 |
| --- | --- | --- | --- |
| 0 | roll | 度 | ±180 |
| 1 | pitch | 度 | ±90 |
| 2 | yaw | 度 | 0~360 |
| 3 | gyro_x | rad/s | — |
| 4 | gyro_y | rad/s | — |
| 5 | gyro_z | rad/s | — |

## 三、USB 0x11 协议 — VIRTUAL_IMU

### 3.1 协议总览

| 属性 | 值 |
| --- | --- |
| 消息类型 | `0x11` |
| 正式名称 | `VIRTUAL_IMU` |
| 方向 | 上位机 → 下位机 |
| 帧长 | 40 字节（与 0x10 相同包装） |
| payload | 28 字节 |
| 发送频率 | 25 Hz（每 2 个 0x10 帧后插入 1 帧 0x11） |
| 波特率 | 115200 |

帧格式：

```
A5 5A 01 11 00 00 [frame_seq 2B] [payload_len=1C 00] [payload 28B] [CRC16 2B]
├──magic──┤ ├type─┤ ├pad──┤ ├──seq───┤ ├──len────┤ ├──────payload──────┤ ├crc──┤
```

### 3.2 Payload 格式（28 字节）

| 偏移 | 类型 | 字段 | 说明 |
| --- | --- | --- | --- |
| 0 | float32 | roll | 横滚角，单位度，±180° |
| 4 | float32 | pitch | 俯仰角，单位度，±90° |
| 8 | float32 | yaw | 航向角，单位度，0~360° |
| 12 | float32 | gyro_x | X 轴角速度，rad/s |
| 16 | float32 | gyro_y | Y 轴角速度，rad/s |
| 20 | float32 | gyro_z | Z 轴角速度，rad/s |
| 24 | uint32 | timestamp_ms | 上位机单调时钟 ms |

Python 编码格式：

```python
IMU_FORMAT = "<6fI"
assert struct.calcsize(IMU_FORMAT) == 28   # 6×4 + 4 = 28
```

### 3.3 帧包装

与 0x10 协议共享同一 CRC 和帧序列计数器：

```python
header = MAGIC + IMU_MSG_TYPE        # A5 5A 01 11
wrapper = struct.pack("<HH", frame_seq, 28)
pre_crc = header + b"\x00\x00" + wrapper + payload
crc = crc16_ccitt(pre_crc)
frame = pre_crc + struct.pack("<H", crc)
```

### 3.4 联调测试向量

测试条件：roll=30.5°、pitch=-10.2°、yaw=45.0°、gyro=(0.1, -0.2, 0.3) rad/s、timestamp=0x12345678、frame_seq=0x0001

待生成 Python 验证代码：

```python
import struct
from virtual_remote import _crc16_ccitt, MAGIC, IMU_MSG_TYPE

payload = struct.pack("<6fI", 30.5, -10.2, 45.0, 0.1, -0.2, 0.3, 0x12345678)
header = MAGIC + IMU_MSG_TYPE
pre_crc = header + b"\x00\x00" + struct.pack("<HH", 1, 28) + payload
crc = _crc16_ccitt(pre_crc)
frame = pre_crc + struct.pack("<H", crc)
print(frame.hex())
# 输出应与联调时下位机收到的帧一致
```

### 3.5 上位机接口

```python
class VirtualRemoteOutput:
    # 在 motion_control_node 的 IMU 回调中调用
    def set_imu_orientation(self, roll_deg, pitch_deg, yaw_deg,
                             gyro_x, gyro_y, gyro_z):
        """设置 IMU 姿态数据，下一个 0x11 帧会自动发送"""
```

## 四、启动顺序

```bash
# 1. 启动雷达和 IMU
ros2 launch livox_ros2_driver mid360_launch.py

# 2. 启动 IMU 滤波节点
ros2 run robocom_localization imu_filter

# 3. 启动运动控制（自动订阅 /imu_orientation）
ros2 launch robocom_bringup all_start.launch.py
```

## 五、调参说明

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| 滤波窗口 | 4 帧 | 窗口越小延迟越低，滤波效果越弱 |
| 互补滤波 α | 0.98 | 越大越信任陀螺仪，越小越快跟踪加速度计 |
| IMU 帧频率 | 25 Hz | 每 2 个控制帧插 1 个 IMU 帧 |

> 窗口 α 0.98 适用于四足狗。若气泵振动更剧烈，可增大窗口到 6 帧或降低 α 到 0.95。
