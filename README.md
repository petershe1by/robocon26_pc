# ROBOCON 仿生足式机器人挑战赛 - 任务赛上位机代码

> 第二十五届全国大学生机器人大赛 ROBOCON
> 8-DOF 点足足式机器人 · Mid-360 激光雷达 · D435 深度相机 · R7 5700ESU 机载电脑

## 项目概述

本项目是为 ROBOCON 2025 仿生足式机器人挑战赛任务赛设计的完整上位机 ROS2 软件框架。

**核心任务**：机器人在 180 秒内识别 8 个随机分布的物资箱（食品/工具/仪器/药品），通过视觉识别类型并移送至对应颜色归位区。每轮比赛有一道四则运算题，答案对 4 取模指定高分归位区。

## 硬件平台

| 组件 | 规格 | 用途 |
|------|------|------|
| 机器人本体 | 8-DOF 点足四足机器人 | 行走搬运 |
| 机载电脑 | AMD R7 5700ESU | 上位机 ROS2 运行 |
| 激光雷达 | Livox Mid-360 | 全局定位 |
| 深度相机 | Intel RealSense D435 | 物块识别+吸取验证（机械臂末端） |
| USB 摄像头 ×2 | 通用 USB 相机 | 数学题拍摄 + 颜色识别 |
| 机械臂 | 5-DOF 轻型机械臂 | 吸取/放置 |
| 下位机 | STM32F407 | 运控执行 + SBUS 解析 |

**机器人尺寸约束**：≤800×600×600mm · ≤35kg · ≤48V · 足部接地直径≤80mm

## 软件架构（10 个 ROS2 功能包）

| 包名 | 功能 | 关键节点 | 语言 |
|------|------|---------|------|
| robocom_interfaces | 自定义 msg/srv | MathResult, RobotState, MotionCmd 等 | CMake |
| robocom_math_solver | 数学题 OCR 求解（20s 看门狗） | math_solver_node | Python |
| robocom_localization | 全局定位 + 坐标管理器 | localization_node, CoordinateManager | Python |
| robocom_motion_control | 运动控制 + USB CDC SBUS 桥接 | motion_control_node, SBUSBridge | Python |
| robocom_navigation | 导航 + 安全 + 任务循环 | navigation_node | Python |
| robocom_vision | YOLO + 颜色掩码 + D435 深度 | yolo_block_detector, color_mask_detector, depth_helper | Python |
| robocom_arm_control | 机械臂 5 状态机 | arm_control_node | Python |
| robocom_task_scheduler | 任务编排调度 | task_scheduler_node | Python |
| robocom_ui | PySide6 触摸屏界面 | ui_main | Python |
| robocom_bringup | 启动文件 + 开机自启 | autostart, launch files | Python |

## 坐标系定义

- **雷达坐标系**：前 x+ / 左 y+ / 上 z+，一键启动时刻的机器人位置为原点
- **物资箱坐标**：左下角物资箱中心为 (x0, y0)，8 个坐标依次为 (x0+850/0, y0-0/850/1700/2550)
- **兑换站坐标**：最左侧兑换站中心为 (x1, y1)，4 个坐标 (x1, y1-0/800/1600/2400)
- **电子围栏**：入口中心 (x3, y3)，矩形范围 [x3, x3+6000] × [y3-2000, y3+2000]
- **不可达区域**：每个物块中心 700mm 圆形避障区
- **兑换触发线**：机器人 x ≥ x3+3500 时启动颜色识别

预留了方便的 SetCoordinate 服务和 CoordinateManager API 给 x0/y0/x1/y1/x3/y3 赋值。

## 通信协议

### USB CDC → STM32（SBUS）

上位机通过 USB CDC 串口发送 25 字节 SBUS 帧模拟遥控器信号。

**帧格式**：[0x0F] + [16 通道 × 11-bit 打包] + [flag] + [0x00]

**通道映射**：
- angular_z(yaw) → ch[3] 左摇杆水平 → joystick[0]
- linear_x(前进) → ch[2] 左摇杆垂直 → joystick[1]（差分→加速度）
- linear_y(侧移) → ch[0] 右摇杆水平 → joystick[2]
- SA 拨杆(ch[9])：使能/失能；SB 拨杆(ch[4])：步态 walk/trot/bounds

**值范围**：352（低）→ 1024（中）→ 1695（高）

### 机械臂 UART（文本协议，115200 baud）

命令：HOME / VISION_SCAN / VISION_AID / GRASP / PLACE / SUCTION_ON / SUCTION_OFF

## 任务执行流程

1. 开机自启动 → UI + 雷达驱动 + 所有 ROS2 节点
2. 操作员在触摸屏点击一键启动
3. TaskScheduler 发布 /match_start 信号
   - Localization 重置雷达坐标原点
   - MathSolver 启动 20 秒 OCR 求解（最高优先级，挂起其他任务）
     - USB 相机 1600×900 → PaddleOCR → 正则清洗 → SymPy 求解 → result%4 → 高分区
4. Navigation 启动主任务循环（8 个物资箱依次执行）：
   a. 大步流星模式：导航到目标物块附近（x±500, y±500），全程电子围栏+避障+限速
   b. 到达目标区域 → 启动 YOLO 视觉识别物块类型
   c. 视野中心在物块矩形内 → D435 深度不变确认吸取成功
   d. Kill YOLO → 导航到兑换站
   e. x ≥ x3+3500 → 启动颜色掩码识别，依次判定 4 个兑换区
   f. 匹配 → 放置物块；不匹配 → 判断下一个
   g. 循环直到 8 个全部送达

## 安全机制

| 机制 | 说明 |
|------|------|
| 电子围栏 | 4 点矩形边界，超出立即急停 |
| 运动看门狗 | 5 秒收不到新坐标变化或指令 → 归零 |
| 速度限制 | >1.5m/s 报警限速，>2.5m/s 失能 |
| 避障 | 物块中心 700mm 不可达圆绕行 |
| 全局超时 | 180 秒比赛限时 |
| 数学题超时 | 20s 未完成直接跳过 |

## 快速开始

```bash
# 1. 安装 ROS2 Humble 和依赖
sudo apt install ros-humble-desktop
chmod +x scripts/install_deps.sh && ./scripts/install_deps.sh

# 2. 编译
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash

# 3. USB 设备权限
sudo scripts/setup_udev_rules.sh

# 4. 运行（调试终端）
ros2 launch robocom_bringup all_start.launch.py &
ros2 run robocom_ui robocom_ui

# 5. 开机自启动（部署后）
sudo scripts/setup_autostart.sh
```

## 调试命令

```bash
ros2 topic echo /robot_state      # 查看定位
ros2 topic echo /mission_status   # 查看任务进度
ros2 topic echo /math_result      # 查看数学题结果
ros2 service call /set_coordinate ...  # 标定坐标系（调试用）
journalctl -u robocom-autostart -f     # 查看自启日志
```

所有节点均内置模拟模式，无实际硬件时自动降级运行，方便开发调试。

下位机 STM32 协议仓库：[github.com/cha815/Robocon](https://github.com/cha815/Robocon)