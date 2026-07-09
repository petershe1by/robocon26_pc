# 上位机 ↔ 8DOF 四足下位机通信说明

本文档面向上位机和 STM32 下位机联调，说明 ROS2 上位机如何通过 USB CDC/SBUS 控制 DM02/8DOF 四足固件，以及机械臂、视觉和任务节点之间的数据流。

## 1. 通信通道

| 通道 | 物理层 | 协议 | 方向 | 用途 |
| --- | --- | --- | --- | --- |
| 四足运动 | USB CDC 虚拟串口 | SBUS 25 字节帧，100 Hz | PC → STM32 | 模拟 ET07/RF207S 遥控器输入 |
| 机械臂文本控制 | UART/TTL | ASCII 命令，115200 8N1 | PC → 机械臂控制器 | HOME、抓取、放置、吸盘 |
| ROS2 内部通信 | DDS topics/services | msg/srv | 节点间 | 任务调度、导航、视觉、状态显示 |

USB CDC 作为虚拟串口时，串口工具可填写常见波特率；若使用标准 SBUS 串口配置，保持 `100000` 和工程约定的数据格式。最终以 STM32 CDC 接收实现为准。

## 2. SBUS 帧格式

上位机周期发送 25 字节 SBUS 帧：

```text
byte[0]      = 0x0F
byte[1..22]  = 16 channels * 11-bit, little-endian bit packing
byte[23]     = flags
byte[24]     = 0x00
```

通道值范围：

```text
SBUS_MIN = 352
SBUS_MID = 1024
SBUS_MAX = 1695

SWITCH_LOW  = 353
SWITCH_MID  = 1024
SWITCH_HIGH = 1695
```

浮点摇杆值映射：

```text
sbus = 1024 + clamp(value, -1.0, 1.0) * (1695 - 1024)
```

## 3. 8DOF 推荐通道语义

下位机固件应按 8DOF 遥控器映射理解 SBUS 通道。注意这里的 CH1 是遥控器通道名，对应 SBUS 数组 `ch[0]`。

| 遥控器通道 | SBUS 数组 | 推荐用途 | 说明 |
| --- | --- | --- | --- |
| CH1 | `ch[0]` | 转向角速度 | 右摇杆横向；左/右转 |
| CH2 | `ch[1]` | 前进 / 后退 | 右摇杆纵向；核心运动输入 |
| CH3 | `ch[2]` | 速度档 | LOW/MID/HIGH |
| CH4 | `ch[3]` | Roll 姿态微调或保留 | 不做真实横移 |
| CH5 | `ch[4]` | 四足主模式 | LOW 停止，MID 站立，HIGH 步态 |
| CH6 | `ch[5]` | 机械臂 J0 jog 或扩展 | V1 旋钮 |
| CH7 | `ch[6]` | 机械臂 J1 jog 或扩展 | V2 波轮 |
| CH8 | `ch[7]` | 子模式 | CH5 MID + CH8 HIGH 进入机械臂 jog |
| CH9 | `ch[8]` | 安全瞬时按钮 | SD 拨到 HIGH 侧触发清故障 + 急停 |
| CH10 | `ch[9]` | 扩展 | SA 或关闭 |

8DOF 不建议使用真实横移控制。若 ROS2 `MotionCmd.linear_y` 仍存在，联调时应默认忽略，或仅作为姿态微调/保留量处理。

## 4. 四足主模式

CH5 是下位机主模式开关：

| CH5 档位 | 含义 | 下位机行为 |
| --- | --- | --- |
| LOW | 安全 / RX-only | 停止四足周期控制输出 |
| MID | 站立 / 姿态模式 | 自动选择全部 8 个腿部电机，执行 MIT foot IK 站立 |
| HIGH | 行走 / 步态模式 | 根据 CH1/CH2 启动离散步态 |

CH1/CH2 行走解析：

| 条件 | 步态命令 |
| --- | --- |
| CH2 > 进入死区 | 前进小跑 |
| CH2 < -进入死区 | 后退小跑 |
| CH2 回中，CH1 > 进入死区 | 原地右转 |
| CH2 回中，CH1 < -进入死区 | 原地左转 |
| CH1/CH2 都回中 | 停止步态并保持站立 |

建议下位机使用迟滞死区：

```text
SBUS_CENTER_DEADBAND = 5
SBUS_MOVE_ENTER_DEADBAND = 25
SBUS_MOVE_EXIT_DEADBAND = 12
ARM_JOG_DEADBAND = 8
```

## 5. 速度档和安全按钮

CH3 速度档：

| CH3 norm | 档位 |
| --- | --- |
| `<= -60` | LOW |
| `-60 ~ 60` | MID |
| `>= 60` | HIGH |

常用 8DOF 步态参数：

| 档位 | 小跑频率 | 前进/后退步幅 | 原地转向步幅 |
| --- | --- | --- | --- |
| LOW | `1.5 Hz` | `50 mm` | `22 mm` |
| MID | `2.0 Hz` | `70 mm` | `30 mm` |
| HIGH | `2.5 Hz` | `85 mm` | `38 mm` |

CH9/SD 安全瞬时按钮：

- 只在拨到 HIGH 侧的边沿触发。
- 触发动作建议为：清故障 + 急停。
- 松手回中不执行动作。

下位机还应保留 SBUS failsafe：若有效 SBUS 帧超时，应自动停止周期控制。

## 6. 上位机 ROS2 数据流

```text
task_scheduler_node
  ├─ /match_start
  └─ /enable_motion

navigation_node
  ├─ publishes /motion_cmd
  ├─ publishes /arm_command
  ├─ publishes /mission_status
  ├─ publishes /estop
  ├─ publishes /yolo_start /yolo_stop
  └─ publishes /color_vision_start /color_vision_stop

motion_control_node
  ├─ subscribes /motion_cmd
  ├─ subscribes /enable_motion
  ├─ subscribes /estop
  └─ writes SBUS frame through SBUSBridge

arm_control_node
  ├─ subscribes /arm_command
  ├─ writes UART text command
  └─ publishes /arm_current_state
```

关键话题：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/motion_cmd` | `robocom_interfaces/msg/MotionCmd` | 导航输出的运动指令 |
| `/enable_motion` | `std_msgs/msg/Bool` | 全局使能 |
| `/estop` | `std_msgs/msg/Bool` | 急停 |
| `/motion_enabled` | `std_msgs/msg/Bool` | 运动控制节点反馈 |
| `/arm_command` | `robocom_interfaces/msg/ArmCommand` | 机械臂状态和吸盘命令 |
| `/arm_current_state` | `std_msgs/msg/Int8` | 机械臂当前状态 |
| `/mission_status` | `robocom_interfaces/msg/MissionStatus` | UI 显示任务状态 |
| `/robot_state` | `robocom_interfaces/msg/RobotState` | 定位输出 |

## 7. 机械臂文本协议

若机械臂由上位机独立 UART 控制，每条命令以 `\n` 结尾：

| 命令 | 含义 |
| --- | --- |
| `HOME\n` | 回到初始/待机位 |
| `VISION_SCAN\n` | 视觉扫描姿态 |
| `VISION_AID\n` | 近距离辅助定位姿态 |
| `GRASP\n` | 抓取姿态 |
| `PLACE\n` | 放置姿态 |
| `SUCTION_ON\n` | 打开吸盘 |
| `SUCTION_OFF\n` | 关闭吸盘 |

若机械臂由 8DOF 固件内置 DM4310/EL05 控制，则推荐保留遥控器子模式：

```text
CH5 MID + CH8 HIGH -> 机械臂 jog 模式
CH6 -> J0 jog
CH7 -> J1 jog
```

此时四足不应自动启动步态。

## 8. 联调建议

上位机侧：

```bash
ros2 run robocom_motion_control motion_control_node --ros-args -p serial_port:=/dev/ttyACM0

ros2 topic pub /motion_cmd robocom_interfaces/msg/MotionCmd \
  "{linear_x: 0.5, linear_y: 0.0, angular_z: 0.0, gait_mode: 1, enable: true, estop: false}" -1

ros2 topic pub /estop std_msgs/msg/Bool "{data: true}" -1
```

下位机侧串口：

| 命令 | 作用 |
| --- | --- |
| `Y` / `y` | 打印 SBUS 诊断 |
| `8` | 选择全部 8 个腿部电机 |
| `s` | 站立 |
| `T` | 小跑测试 |
| `[` | 原地左转 |
| `]` | 原地右转 |
| `x` | idle 停止 |
| `!` | 急停全部电机 |

排查顺序：

1. 用 `Y` 确认是否收到 SBUS 帧。
2. 确认 CH5 是否进入 MID/HIGH。
3. 确认 CH1/CH2/CH3 raw/norm 是否随上位机指令变化。
4. 确认 `failsafe=0`、电机 `online=1`、`enc=1`、`fault=0`。
5. 先悬空测试站立和单步，再落地测试。

## 9. 当前仓库适配注意

当前 Python 上位机已有 SBUS 打包和 ROS2 运动链路，但旧文档曾按“侧移/步态 walk/trot/bounds/使能开关”描述。对接 8DOF 固件时，应将文档和代码统一到本文件的 CH1/CH2/CH3/CH5/CH8/CH9 语义。

重点检查：

- `sbus_bridge.py` 的通道写入是否匹配 8DOF 固件；
- `motion_control_node.py` 是否能输出 CH5 主模式、CH3 速度档、CH9 安全按钮；
- `navigation_node.py` 不应依赖真实横移；
- 下位机 `Y/y` 诊断应作为最终联调依据。
