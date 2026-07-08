 # 上位机 ↔ 下位机 通信协议说明

 > 本文档面向**下位机（STM32）开发者**，说明上位机 PC 与下位机之间的通信协议与数据流，方便双方对齐接口，打通联调。
 >
 > 协议仓库：[github.com/cha815/Robocon](https://github.com/cha815/Robocon)

---

## 一、通信通道总览

上位机与下位机之间的通信分为两条独立的物理链路：

| 通道 | 物理层 | 波特率 | 协议 | 方向 | 用途 |
|------|--------|--------|------|------|------|
| 运动控制 | USB CDC 虚拟串口 | **100000 baud** | SBUS 帧（25 字节） | PC → STM32 | 运动指令（前进/转向/侧移/使能/步态） |
| 机械臂 | UART（TTL 串口） | **115200 baud** | 文本协议（ASCII 命令） | PC → 机械臂控制器 | 机械臂状态切换 + 吸盘控制 |

---

## 二、运动控制 — SBUS 协议（USB CDC，100000 baud）

上位机通过 USB CDC 虚拟串口持续发送 SBUS 帧，模拟遥控器信号，频率 **100 Hz**（每 10 ms 一帧）。

### 2.1 帧结构（25 字节）

```
偏移   0    1                    22     23     24
      ┌─────┬────────────────────────┬──────┬─────┐
      │0x0F │  16通道 × 11-bit 打包   │ flag │0x00 │
      └─────┴────────────────────────┴──────┴─────┘
```

| 偏移 | 大小 | 值 | 说明 |
|------|------|----|------|
| 0 | 1 byte | `0x0F` | 起始字节 |
| 1~22 | 22 bytes | 16 × 11-bit 数据 | 16 个 SBUS 通道值，按位打包（详见 2.2） |
| 23 | 1 byte | `flag` | 标志位（丢失帧标志等，默认 `0x00`） |
| 24 | 1 byte | `0x00` | 结束字节 |

### 2.2 16 通道打包方式（11-bit 按位拼接）

每个通道值范围为 **352 ~ 1695**（11-bit 有效范围），按以下方式连续打包到 22 字节数据区中：

```
ch[0]   → 占据 bit[0:10]
ch[1]   → 占据 bit[11:21]
ch[2]   → 占据 bit[22:32]
...
ch[15]  → 占据 bit[165:175]
```

Python 打包实现见 `sbus_bridge.py` 中的 `pack_sbus_frame()` 函数。

### 2.3 通道映射表

| SBUS 通道 | 映射量 | 对应关系 | 备注 |
|-----------|--------|---------|------|
| **ch[0]** | `linear_y`（横向平移） | 右摇杆水平 → joystick[2] | |
| **ch[2]** | `linear_x`（前进/后退） | 左摇杆垂直 → joystick[1] | 正数前进，负数后退 |
| **ch[3]** | `angular_z`（原地转向） | 左摇杆水平 → joystick[0] | 正数右转，负数左转 |
| **ch[4]** | SB 拨杆 — 步态选择 | 见 2.4 | |
| **ch[5]** | 机械臂当前状态 | 见 2.5 | 由上位机根据机械臂反馈设置 |
| **ch[7]** | 预留 | SWITCH_LOW | |
| **ch[8]** | 预留 | SWITCH_LOW | |
| **ch[9]** | SA 拨杆 — 使能/失能 | 见 2.4 | |
| ch[1], ch[6], ch[10~15] | 未使用 | SBUS_MID (1024) | |

### 2.4 拨杆通道取值

拨杆通道使用三档值表示状态档位：

| 档位 | 通道值 | 备注 |
|------|--------|------|
| **低 (low)** | **353** | |
| **中 (mid)** | **1024** | |
| **高 (high)** | **1695** | |

| 通道 | 档位映射 | 含义 |
|------|---------|------|
| **ch[9]** (SA) | low → 失能, mid → 使能 | 失能时所有摇杆归零 |
| **ch[4]** (SB) | low → walk, mid → trot, high → bounds | 步态切换 |

### 2.5 机械臂状态 → ch[5] 映射

上位机 `arm_control_node` 将机械臂的 5 状态机值发布到 ROS2 话题 `/arm_current_state`，`motion_control_node` 收到后写入 SBUS **ch[5]**，让 STM32 实时感知机械臂当前工作阶段。

| 状态编号 | SBUS ch[5] 值 | 状态名 | 说明 |
|---------|---------------|--------|------|
| 0 | **352** | HOME | 初始/待机位 |
| 1 | **688** | VISION_SCAN | 视觉扫描姿态（寻找物块） |
| 2 | **1024** | VISION_AID | 视觉辅助定位（近距离对准） |
| 3 | **1359** | GRASP | 吸取姿态（下压吸取物块） |
| 4 | **1695** | PLACE | 放置姿态（到站释放物块） |

### 2.6 摇杆值映射（float → SBUS）

上位机内部运动指令采用浮点数 **-1.0 ~ 1.0** 表示摇杆幅度，经 `float_to_sbus()` 线性映射为 SBUS 值：

```
SBUS_value = 1024 + float_value × (1695 − 1024)

float = -1.0 → SBUS = 352
float =  0.0 → SBUS = 1024 （中位）
float =  1.0 → SBUS = 1695
```

STM32 端收到 SBUS 值后需反算为 **-1.0 ~ 1.0** 再做控制量解算：

```
float_value = (SBUS_value − 1024) / (1695 − 1024)
```

### 2.7 典型一帧示例

若机器人以 trot 步态使能前进：

| 通道 | 含义 | SBUS 值 |
|------|------|--------|
| ch[0] | linear_y = 0 | 1024 |
| ch[2] | linear_x = 0.5 | 1360 |
| ch[3] | angular_z = 0 | 1024 |
| ch[4] | SB = trot | 1024 |
| ch[9] | SA = 使能 | 1024 |

---

## 三、机械臂控制 — 文本协议（UART，115200 baud）

上位机通过 TTL 串口（`/dev/ttyUSB0`，115200 baud，8N1）向机械臂控制器发送 ASCII 命令。

### 3.1 指令列表

| 指令 | 含义 | 上位机触发条件 |
|------|------|---------------|
| `HOME\n` | 回到待机/初始位置 | 初始化 / 抓取完成回到待机 |
| `VISION_SCAN\n` | 视觉扫描姿态 | 导航到达物块附近，开始 YOLO 识别 |
| `VISION_AID\n` | 视觉辅助定位姿态 | YOLO 识别到物块，需要深度相机近距离对准 |
| `GRASP\n` | 吸取姿态 | 深度相机确认位置，下压吸取 |
| `PLACE\n` | 放置姿态 | 导航到达兑换站，释放物块 |
| `SUCTION_ON\n` | 打开吸盘 | GRASP 姿态到位 |
| `SUCTION_OFF\n` | 关闭吸盘 | PLACE 姿态到位 |

### 3.2 协议约定

- 每条命令以 **换行符 `\n`（0x0A）** 结尾
- 命令字符串均为大写 ASCII
- 上位机发送命令后 sleep 100 ms，然后读一行回应（可选，超时 0.5s）
- 机械臂控制器无需应答也可正常工作，超时不影响状态机流转

---

## 四、数据流全景

```
┌────────────────────────────────────────────────────────────────────────────┐
│                              上位机 (PC, ROS2)                              │
│                                                                              │
│  navigation_node                                                             │
│      │ 发布 /motion_cmd (MotionCmd)                                         │
│      ▼                                                                       │
│  motion_control_node                                                         │
│      │ 订阅 /motion_cmd + /arm_current_state + /enable_motion + /estop      │
│      │ 调用 SBUSBridge API                                                   │
│      ▼                                                                       │
│  SBUSBridge  ──USB CDC 100000→  STM32   （SBUS 25 字节帧, 100 Hz）           │
│                                                                              │
│  navigation_node                                                             │
│      │ 发布 /arm_command (ArmCommand) + 订阅 /arm_at_target                  │
│      ▼                                                                       │
│  arm_control_node                                                            │
│      │ 发布 /arm_current_state (Int8)  ← 反馈给 motion_control_node         │
│      ▼                                                                       │
│  UART  ─────────115200──────→  机械臂控制器   （文本协议）                    │
└────────────────────────────────────────────────────────────────────────────┘
```

### 4.1 运动控制链路（PC → USB CDC → STM32）

```
navigation_node  ──/motion_cmd──→  motion_control_node  ──SBUSBridge.send_frame()──→  USB CDC ──→  STM32
```

1. `navigation_node` 根据当前目标坐标计算摇杆量，发布 `MotionCmd` 到 `/motion_cmd`
2. `motion_control_node` 收到后将浮点值映射为 SBUS 通道值
3. `SBUSBridge` 在独立线程中以 100 Hz 持续发送 SBUS 帧

### 4.2 机械臂控制链路（PC → UART → 机械臂）

```
navigation_node  ──/arm_command──→  arm_control_node  ──UART.write()──→  机械臂控制器
                                       │
                                       │ 发布 /arm_current_state
                                       ▼
                                  motion_control_node  ──SBUS ch[5]──→  STM32
```

机械臂状态也会通过 SBUS ch[5] 转发至 STM32，方便下位机根据机械臂阶段协调动作（如行走时收起机械臂，抓取时保持机身稳定）。

### 4.3 ROS2 节点间话题图

| 话题 | 类型 | 发布者 | 订阅者 | 说明 |
|------|------|--------|--------|------|
| `/motion_cmd` | MotionCmd | navigation | motion_control | 运动控制指令 |
| `/arm_command` | ArmCommand | navigation | arm_control | 机械臂指令 |
| `/arm_current_state` | Int8 | arm_control | motion_control | 机械臂当前状态 → SBUS ch[5] |
| `/arm_at_target` | Bool | arm_control | navigation | 机械臂到位信号 |
| `/enable_motion` | Bool | task_scheduler | motion, navigation | 全局使能 |
| `/estop` | Bool | navigation | motion_control | 急停 |
| `/motion_enabled` | Bool | motion_control | — | 当前使能状态 |
| `/mission_status` | MissionStatus | navigation | UI | 任务阶段状态 |
| `/robot_state` | RobotState | localization | navigation, UI | 机器人位姿 |

---

## 五、时序与安全机制

### 5.1 SBUS 发送时序

- **帧率**: 100 Hz（每帧间隔 10 ms）
- **发送线程**: 独立后台线程，不阻塞主逻辑
- **断线重连**: `SBUSBridge.connect()` 支持 `auto` 模式自动扫描 `/dev/ttyACM*` 或 `USB*` 设备

### 5.2 运动看门狗

- 若连续 **5 秒** 未收到新的 `/motion_cmd`，`motion_control_node` 自动将所有摇杆通道置为 **1024（中位/停止）**
- 失能状态下看门狗不生效

### 5.3 速度与安全限制（上位机侧）

| 条件 | 行为 |
|------|------|
| 速度 > 1.5 m/s | 报警限速，控制量降额输出 |
| 速度 > 2.5 m/s | 发布 `/estop`，立即失能 |
| 超出电子围栏 | 发布 `/estop`，停止运动 |

---

## 六、下位机（STM32）职责说明

### 6.1 USB CDC 接收

1. 实现 USB CDC 虚拟串口接收（STM32 USB_OTG_FS 或 FS）
2. 以 **100000 baud** 接收 25 字节 SBUS 帧
3. 校验起始字节 `0x0F` 和结束字节 `0x00`
4. 若连续超过 **20 ms** 未收到有效帧，应自动失能进入安全状态
5. 解包 16 × 11-bit 通道值，按 2.3~2.5 节映射解读控制量
6. 失能（ch[9] = low，即 353）时所有摇杆按中位处理，电机不输出

### 6.2 串口约定

| 参数 | 值 |
|------|-----|
| 设备 | `/dev/ttyACM0`（或自动扫描 ACM/USB 类设备） |
| 波特率 | **100000** |
| 数据位 | 8 |
| 校验位 | 无 |
| 停止位 | 1 |
| 流控 | 无 |

### 6.3 机械臂控制器串口约定

| 参数 | 值 |
|------|-----|
| 设备 | `/dev/ttyUSB0` |
| 波特率 | **115200** |
| 数据位 | 8 |
| 校验位 | 无 |
| 停止位 | 1 |
| 流控 | 无 |
| 命令编码 | ASCII, 大写字符串 + `\n` |

### 6.4 通道值范围速查

```
SBUS 最小值:  352  （对应 float -1.0）
SBUS 中位值:  1024 （对应 float  0.0）
SBUS 最大值:  1695 （对应 float  1.0）

拨杆低:  353
拨杆中:  1024
拨杆高:  1695
```

---

## 七、调试与联调建议

### 7.1 验证 USB CDC 连通性

上位机侧（Ubuntu）：
```bash
# 查看设备节点
ls -la /dev/ttyACM*

# 检查 udev 权限
ls -la /dev/ttyACM0   # 应为 crw-rw-rw-

# 用 minicom/cutecom 查看原始数据
sudo apt install minicom
stty -F /dev/ttyACM0 100000 cs8 -cstopb -parenb raw
cat /dev/ttyACM0 | xxd   # 观察是否收到 25 字节 SBUS 帧
```

### 7.2 验证 SBUS 帧内容

启动 `motion_control_node`（模拟模式）后观察发送的字节流：
```bash
# 运行运动控制节点
ros2 run robocom_motion_control motion_control_node --ros-args -p serial_port:=/dev/ttyACM0

# 另一终端，监视原始帧
cat /dev/ttyACM0 | xxd -c 25
```

### 7.3 单通道测试

通过 ROS2 话题手动发布指令验证每个通道：
```bash
# 测试前进 (linear_x = 0.5 → ch[2] ≈ 1360)
ros2 topic pub /motion_cmd robocom_interfaces/msg/MotionCmd \
  "{linear_x: 0.5, linear_y: 0.0, angular_z: 0.0, gait_mode: 1, enable: true, estop: false}" -1

# 测试转向 (angular_z = 0.3 → ch[3] ≈ 1225)
ros2 topic pub /motion_cmd robocom_interfaces/msg/MotionCmd \
  "{linear_x: 0.0, linear_y: 0.0, angular_z: 0.3, gait_mode: 1, enable: true, estop: false}" -1

# 测试机械臂状态映射 (→ ch[5] = 688)
ros2 topic pub /arm_current_state std_msgs/msg/Int8 "{data: 1}" -1

# 测试使能/失能
ros2 topic pub /enable_motion std_msgs/msg/Bool "{data: false}" -1   # 失能
ros2 topic pub /enable_motion std_msgs/msg/Bool "{data: true}" -1    # 使能
```

### 7.4 常见问题

| 现象 | 可能原因 | 解决 |
|------|---------|------|
| 串口打不开 | 设备权限不足 | `sudo chmod 666 /dev/ttyACM0`，或执行 `setup_udev_rules.sh` |
| 收不到 SBUS 帧 | 上位机未启动 / 串口名不对 | 检查 `motion_control_node` 日志，确认 `serial_port` 参数 |
| SBUS 帧校验失败 | 波特率或打包方式不一致 | 确认 STM32 端使用 **100000 baud**，按 2.2 节解包 |
| 机器人不运动 | ch[9] 为 low 处于失能状态 | 检查上位机是否发布了 `enable: true` 指令 |
| 机械臂无动作 | 串口线松动 / 波特率不匹配 | 确认 115200 baud，检查接线 |

---

## 附录：上位机相关代码文件

| 文件 | 功能 |
|------|------|
| `robocom_motion_control/sbus_bridge.py` | SBUS 帧打包 + USB CDC 串口收发 + 100Hz 发送线程 |
| `robocom_motion_control/motion_control_node.py` | ROS2 节点：订阅运动指令 → 调用 SBUSBridge 输出 |
| `robocom_motion_control/config/motion_params.yaml` | 运动控制参数配置 |
| `robocom_arm_control/arm_control_node.py` | ROS2 节点：订阅机械臂指令 → UART 文本协议输出 |
| `robocom_interfaces/msg/MotionCmd.msg` | 运动控制 ROS2 消息定义 |
| `robocom_interfaces/msg/ArmCommand.msg` | 机械臂控制 ROS2 消息定义 |
| `scripts/setup_udev_rules.sh` | USB 设备 udev 权限规则 |

---

*如有疑问或协议需要调整，请联系上位机负责人。*
