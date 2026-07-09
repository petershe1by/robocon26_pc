# ROBOCON 8DOF 四足上位机

> ROBOCON 任务赛上位机 ROS2 框架
> 适配 DM02/8DOF 点足四足控制固件、Livox Mid-360、RealSense D435、USB 相机和机械臂

## 项目概述

本仓库负责比赛流程调度、定位、视觉识别、机械臂动作编排，以及向 8DOF 四足下位机发送运动控制指令。

机器人本体为每条腿 2 自由度的 8DOF 点足四足。该结构主要在足端 `x/z` 平面运动，适合前进、后退、原地转向、弧线转弯、站立保持和姿态微调。不建议把 `linear_y` 当成真实横移使用，因为 8DOF 没有髋外展/内收自由度，强行横移通常依赖脚底打滑，稳定性和可控性都不好。

## 硬件平台

| 组件 | 规格 | 用途 |
| --- | --- | --- |
| 机器人本体 | 8DOF 点足四足，4 腿 × HIP/KNEE | 站立、行走、转向 |
| 下位机 | STM32 + DM 电机控制固件 | CAN 电机控制、SBUS 解码、MIT 步态 |
| 机载电脑 | AMD R7 5700U / Intel N100 及以上 | ROS2 上位机 |
| 激光雷达 | Livox Mid-360 | 全局定位 |
| 深度相机 | Intel RealSense D435 | 物块识别、吸取验证 |
| USB 摄像头 | 通用 USB Camera | 数学题 OCR、颜色识别 |
| 机械臂 | DM4310/EL05 或独立控制器 | 物块抓取/放置 |
| 遥控链路 | ET07 + RF207S，W.BUS/S.BUS | 手动调试与下位机模式参考 |

## ROS2 软件架构

| 包名 | 功能 | 关键节点 |
| --- | --- | --- |
| `robocom_interfaces` | 自定义 msg/srv | `MotionCmd`, `ArmCommand`, `MathResult` |
| `robocom_task_scheduler` | 一键启动、比赛计时 | `task_scheduler_node` |
| `robocom_localization` | 雷达/里程计定位、坐标标定 | `localization_node` |
| `robocom_navigation` | 任务导航、安全边界、阶段切换 | `navigation_node` |
| `robocom_motion_control` | ROS2 运动指令转 SBUS | `motion_control_node`, `SBUSBridge` |
| `robocom_arm_control` | 机械臂状态机和串口命令 | `arm_control_node` |
| `robocom_vision` | YOLO、颜色掩码、D435 深度辅助 | `yolo_block_detector`, `color_mask_detector`, `depth_helper` |
| `robocom_math_solver` | 数学题 OCR 和求解 | `math_solver_node` |
| `robocom_ui` | PySide6 触摸屏 UI | `robocom_ui` |
| `robocom_bringup` | launch、自启动配置 | `all_start.launch.py` |

## 8DOF 控制口径

下位机 8DOF 固件以遥控器通道为核心语义。上位机通过 USB CDC 发送 SBUS 帧来模拟遥控器输入。

| SBUS 通道 | 8DOF 固件用途 | 上位机含义 |
| --- | --- | --- |
| CH1 | 转向输入 | `angular_z`，左/右原地转向 |
| CH2 | 前进/后退输入 | `linear_x`，正数前进，负数后退 |
| CH3 | 速度档 | LOW/MID/HIGH 三档步态速度 |
| CH4 | 姿态微调或保留 | 不作为真实横移 |
| CH5 | 主模式 | LOW 停止，MID 站立，HIGH 步态 |
| CH6 | 机械臂 J0 jog | 机械臂扩展 |
| CH7 | 机械臂 J1 jog | 机械臂扩展 |
| CH8 | 子模式 | CH5 MID + CH8 HIGH 进入机械臂 jog 模式 |
| CH9 | 安全瞬时按钮 | HIGH 边沿触发清故障 + 急停 |
| CH10 | 扩展 | 保留 |

四足主模式：

| CH5 档位 | 行为 |
| --- | --- |
| LOW | RX-only / 安全停止，不发送周期控制 |
| MID | 选择全部 8 个腿部电机，执行 MIT foot IK 站立 |
| HIGH | 根据 CH1/CH2 启动离散步态，摇杆回中后停止并保持站立 |

CH1/CH2 行走逻辑：

| 输入 | 行为 |
| --- | --- |
| CH2 向前 | 前进小跑 |
| CH2 向后 | 后退小跑 |
| CH2 回中且 CH1 向左 | 原地左转 |
| CH2 回中且 CH1 向右 | 原地右转 |
| CH1/CH2 同时输入 | 当前建议 CH2 前后优先 |
| CH1/CH2 都回中 | 停止步态并保持站立 |

CH3 速度档以 STM32 当前固件参数为准。常用配置为 LOW=`1.5 Hz/50 mm/22 mm`，MID=`2.0 Hz/70 mm/30 mm`，HIGH=`2.5 Hz/85 mm/38 mm`，分别对应小跑频率、前进步幅和转向步幅。

## 任务执行流程

1. 开机启动 ROS2 节点和 UI。
2. 操作员在 UI 点击一键启动。
3. `task_scheduler_node` 发布 `/match_start` 和 `/enable_motion`。
4. `math_solver_node` 启动 20 秒数学题 OCR；超时则发布失败结果，让导航继续。
5. `navigation_node` 进入任务循环：导航到物资箱、启动视觉识别、机械臂抓取、前往兑换站、颜色识别并放置。
6. `motion_control_node` 将 `/motion_cmd` 转成 SBUS 帧，持续发送给 STM32。

## 坐标与安全

坐标单位默认使用 `mm`。

| 项目 | 说明 |
| --- | --- |
| 物资箱坐标 | 以左下角物资箱中心 `(x0, y0)` 为基准，间距 850 mm |
| 兑换站坐标 | 以最左侧兑换站中心 `(x1, y1)` 为基准，间距 800 mm |
| 入口中心 | `(x3, y3)` |
| 电子围栏 | `[x3, x3+6000] × [y3-2000, y3+2000]` |
| 物块避障 | 每个物块中心 700 mm 圆形不可达区 |

上位机安全机制包括 `/estop` 急停、`/enable_motion` 全局使能、运动指令看门狗、电子围栏越界急停、180 秒比赛总超时和 20 秒数学题超时。

下位机侧仍应保留独立安全：SBUS failsafe、CH5 LOW 安全档、CH9/SD 急停、编码器异常保护、MIT 目标误差保护。

## 快速开始

```bash
sudo apt install ros-humble-desktop python3-colcon-common-extensions
chmod +x scripts/install_deps.sh
./scripts/install_deps.sh

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

sudo bash src/robocom_pc/scripts/setup_udev_rules.sh
ros2 launch robocom_bringup all_start.launch.py
```

UI 或命令行启动任务：

```bash
ros2 service call /start_mission robocom_interfaces/srv/StartMission "{}"
```

## 常用调试命令

```bash
ros2 topic echo /robot_state
ros2 topic echo /mission_status
ros2 topic echo /motion_cmd
ros2 topic echo /motion_enabled

ros2 topic pub /motion_cmd robocom_interfaces/msg/MotionCmd \
  "{linear_x: 0.5, linear_y: 0.0, angular_z: 0.0, gait_mode: 1, enable: true, estop: false}" -1

ros2 topic pub /estop std_msgs/msg/Bool "{data: true}" -1
```

下位机串口侧建议保留：

| 命令 | 用途 |
| --- | --- |
| `Y` / `y` | 打印 SBUS 诊断：帧计数、CH1/CH2/CH3/CH5/CH8/CH9 raw/norm、failsafe、当前步态命令 |
| `8` | 选择全部 8 个腿部电机 |
| `s` | 站立 |
| `T` | 小跑/前进步态测试 |
| `[` / `]` | 原地左转 / 右转 |
| `x` | idle 并停止控制 |
| `!` | 全部电机急停 |

## 重要适配提醒

当前仓库的运动控制代码需要和 8DOF 固件通道语义保持一致。若下位机以 CH1/CH2/CH3/CH5/CH8/CH9 解码，请重点检查：

- `robocom_motion_control/sbus_bridge.py` 的 ROS 摇杆量到 SBUS 通道映射；
- `robocom_motion_control/motion_control_node.py` 的主模式、速度档、急停通道写入；
- 是否仍在使用旧版 `linear_y` 侧移通道；
- 是否需要新增 CH3 速度档、CH5 主模式、CH8 子模式、CH9 安全按钮输出。

文档以 8DOF 四足控制使用方式为准；代码联调时应以 STM32 当前固件的实际 SBUS 解码为最终依据。
