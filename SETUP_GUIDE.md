# ROBOCON 上位机 — 从零搭建教程（Ubuntu 22.04）

> 本文档假设你有一台**全新安装的 Ubuntu 22.04 LTS** 系统，未安装过 ROS 或 Python 依赖。
> 机载电脑推荐 AMD R7 5700U / Intel N100 及以上性能。

---

## 目录

1. [系统准备](#1-系统准备)
2. [安装 ROS2 Humble](#2-安装-ros2-humble)
3. [克隆仓库](#3-克隆仓库)
4. [安装系统依赖](#4-安装系统依赖)
5. [安装 Python 依赖](#5-安装-python-依赖)
6. [Livox 雷达驱动](#6-livox-雷达驱动)
7. [编译工作空间](#7-编译工作空间)
8. [USB 设备权限](#8-usb-设备权限)
9. [下载模型文件](#9-下载模型文件)
10. [验证安装](#10-验证安装)
11. [开机自启动配置](#11-开机自启动配置)
12. [坐标系标定](#12-坐标系标定)
13. [常见问题](#13-常见问题)

---

## 1. 系统准备

```bash
# 更新系统
sudo apt update && sudo apt upgrade -y

# 安装基础开发工具
sudo apt install -y \
    git \
    curl \
    wget \
    vim \
    net-tools \
    htop \
    cmake \
    build-essential \
    python3-pip \
    python3-venv \
    python3-dev

# 确认 Python 版本 (>=3.10)
python3 --version
```

---

## 2. 安装 ROS2 Humble

按照 ROS 官方文档安装 Humble：

```bash
# 设置 locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 添加 ROS2 源
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

# 添加 ROS2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装 ROS2 Humble
sudo apt update
sudo apt install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool

# 初始化 rosdep
sudo rosdep init
rosdep update

# 添加环境变量（追加到 ~/.bashrc）
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

验证安装：

```bash
ros2 --version
# 输出类似: ros2 humble
```

---

## 3. 克隆仓库

```bash
# 创建工作空间
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# 克隆上位机代码
git clone https://github.com/petershe1by/robocon26_pc.git src/robocom_pc

# 确认目录结构
ls src/robocom_pc/src/
# 应看到 10 个 robocom_* 包
```

---

## 4. 安装系统依赖

```bash
cd ~/ros2_ws/src/robocom_pc

# 运行官方安装脚本（ROS2 相关依赖）
sudo bash scripts/install_deps.sh
```

如果 `install_deps.sh` 执行失败，可以手动安装：

```bash
sudo apt install -y \
    python3-rclpy \
    ros-humble-sensor-msgs \
    ros-humble-nav-msgs \
    ros-humble-tf2-ros \
    ros-humble-launch-ros \
    ros-humble-cv-bridge \
    ros-humble-rosidl-default-generators \
    python3-rosidl-generator-py
```

---

## 5. 安装 Python 依赖

```bash
# 升级 pip
pip3 install --upgrade pip

# 安装 Python 依赖包
pip3 install \
    sympy \
    paddleocr \
    paddlepaddle \
    pyttsx3 \
    pyserial \
    PySide6 \
    ultralytics \
    opencv-python \
    pyrealsense2 \
    numpy \
    transforms3d
```

> **注意**：`paddlepaddle` 和 `paddleocr` 体积较大（~500MB），安装可能需要较长时间。
> 如果不需要 OCR 功能（数学题识别），可以跳过这两个包，`math_solver_node` 会自动降级为模拟模式。

各包用途：

| 包名 | 用途 | 必需 |
|------|------|------|
| sympy | 数学题安全计算 | 是 |
| pyserial | USB CDC SBUS 通信 | 是 |
| PySide6 | 触摸屏 UI 界面 | 是 |
| opencv-python | 摄像头图像处理 | 是 |
| numpy | 数值计算 | 是 |
| ultralytics | YOLO 物块检测 | 是 |
| pyrealsense2 | D435 深度相机 | 是 |
| paddleocr / paddlepaddle | OCR 数学题识别 | 否 |
| pyttsx3 | TTS 语音播报 | 否 |
| transforms3d | 坐标变换工具 | 否 |

---

## 6. Livox 雷达驱动

Mid-360 激光雷达需要单独安装 Livox SDK：

```bash
# 克隆 Livox ROS2 Driver
cd ~
git clone https://github.com/Livox-SDK/livox_ros_driver2.git

# 编译
cd livox_ros_driver2
./build.sh humble

# 此时会生成 livox_ros2_driver 包并安装到工作空间
# 如果 build.sh 失败，手动编译：
cd ~/ros2_ws
colcon build --symlink-install --packages-select livox_ros2_driver
source ~/ros2_ws/install/setup.bash
```

> 如果手边没有 Mid-360 雷达，定位节点会降级运行，不影响其他模块调试。

---

## 7. 编译工作空间

```bash
cd ~/ros2_ws

# 安装 robocom_interfaces（msg/srv 定义，需要 CMake）
colcon build --symlink-install --packages-select robocom_interfaces

# 编译所有 Python 包
colcon build --symlink-install

# 刷新环境
source ~/ros2_ws/install/setup.bash
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
```

如果编译时遇到 `ament_python` 相关错误，确认已安装：

```bash
sudo apt install -y python3-ament-package python3-colcon-core
```

---

## 8. USB 设备权限

连接硬件后，配置 udev 规则：

```bash
cd ~/ros2_ws/src/robocom_pc
sudo bash scripts/setup_udev_rules.sh
```

这条命令会写入 `/etc/udev/rules.d/99-robocom.rules`，规则内容：

| 设备 | VID | PID | 用途 |
|------|-----|-----|------|
| STM32 USB CDC | 0483 | 5740 | SBUS 串口通信 |
| Intel D435 | 8086 | 0b07 | 深度相机 |

执行后**重新插拔** USB 设备使规则生效。

如果手边没有硬件，所有节点会自动降级为**模拟模式**（打印模拟日志代替实际操作），方便在没有机器人的情况下调试代码逻辑。

---

## 9. 下载模型文件

YOLO 物块检测模型文件未包含在 git 仓库中（`.gitignore` 已排除），需要单独下载：

**方式一：直接下载**

将 `block_detector.pt` 放置到：

```bash
# 创建目录（如果不存在）
mkdir -p ~/ros2_ws/src/robocom_pc/src/robocom_vision/models

# 放入模型文件（从 U 盘或网盘复制）
cp /path/to/block_detector.pt \
   ~/ros2_ws/src/robocom_pc/src/robocom_vision/models/
```

**方式二：使用软链接**

如果模型存储在共享位置：

```bash
ln -s /data/models/block_detector.pt \
   ~/ros2_ws/src/robocom_pc/src/robocom_vision/models/block_detector.pt
```

> **注意**：模型路径已通过 `ament_index_python` 自动解析，编译安装后会在共享目录查找。如果你将模型放在源目录中，`colcon build --symlink-install` 会通过符号链接自动关联。

---

## 10. 验证安装

### 10.1 测试所有 Python 模块语法

```bash
source ~/ros2_ws/install/setup.bash

# 逐一测试各节点能否导入
python3 -c "from robocom_interfaces.msg import MotionCmd; print('interfaces OK')"
python3 -c "from robocom_localization.coordinate_manager import CoordinateManager; print('localization OK')"
python3 -c "from robocom_motion_control.sbus_bridge import SBUSBridge; print('motion OK')"
python3 -c "from robocom_vision.yolo_block_detector import YOLOBlockDetector; print('vision OK')"
python3 -c "from robocom_arm_control.arm_control_node import ArmControlNode; print('arm OK')"
python3 -c "from robocom_math_solver.math_solver_node import MathSolverNode; print('math OK')"
python3 -c "from robocom_navigation.navigation_node import NavigationNode; print('nav OK')"
python3 -c "from robocom_task_scheduler.task_scheduler_node import TaskSchedulerNode; print('scheduler OK')"
```

### 10.2 启动所有节点（无硬件模拟模式）

```bash
# 终端 1：启动所有 ROS2 节点 + UI
ros2 launch robocom_bringup all_start.launch.py
```

你应该能在输出中看到每个节点的启动日志，例如：

```
[task_scheduler]: TaskSchedulerNode 已启动，等待一键启动...
[navigation]: NavigationNode 已启动
[motion_control]: MotionControlNode 已启动
[arm_control]: ArmControlNode 已启动, 状态: HOME
...
```

同时在屏幕上看到 PySide6 触摸屏界面：

- 标题：**ROBOCON 仿生足式机器人挑战赛 - 任务赛上位机**
- 一键启动按钮（绿色）
- 雷达坐标 / 数学题识别 / 任务进度 三个信息面板
- 运行进程列表

> **注意**：在没有硬件的模拟模式下，所有节点的 `try/except` 会捕获硬件访问错误并自动降级。

### 10.3 检查话题通信

```bash
# 终端 2：查看话题列表
ros2 topic list

# 应看到：
# /arm_at_target
# /arm_command
# /arm_current_state
# /block_info
# /color_vision_start
# /detected_block_type
# /detected_exchange_id
# /enable_motion
# /estop
# /grasp_complete
# /grasp_verified
# /match_start
# /math_result
# /mission_status
# /motion_cmd
# /motion_enabled
# /place_complete
# /robot_state
# /yolo_start
```

### 10.4 一键启动模拟比赛

在 UI 界面上点击「一键启动」按钮，或通过命令行：

```bash
# 终端 2：调用启动服务
ros2 service call /start_mission robocom_interfaces/srv/StartMission "{}"
```

观察终端输出，查看任务调度流程：

```
[task_scheduler]: === 一键启动！===
[task_scheduler]: 已发布 match_start 和 enable_motion
[localization]: 雷达原点已重置, yaw=0.00
[math_solver]: === 数学题识别启动 ===
[math_solver]: [模拟] 检测到物块，发送抓取确认
...
```

---

## 11. 开机自启动配置

部署到机器人后，配置开机自动运行：

```bash
cd ~/ros2_ws/src/robocom_pc
sudo bash scripts/setup_autostart.sh
```

该脚本会：

1. 创建 systemd 服务 `/etc/systemd/system/robocom-autostart.service`
2. 配置 USB 设备 udev 规则
3. 启用服务并启动

查看启动状态：

```bash
journalctl -u robocom-autostart -f
```

停止自启服务：

```bash
sudo systemctl stop robocom-autostart
sudo systemctl disable robocom-autostart
```

---

## 12. 坐标系标定

在比赛场地中，需要通过 `SetCoordinate` 服务标定三个关键坐标系原点：

```bash
# 设置左下角物资箱中心 (x0, y0)
ros2 service call /set_coordinate robocom_interfaces/srv/SetCoordinate \
  "{coordinate_name: 'block_origin', x: 1000.0, y: 2000.0}"

# 设置最左侧兑换站中心 (x1, y1)
ros2 service call /set_coordinate robocom_interfaces/srv/SetCoordinate \
  "{coordinate_name: 'exchange_origin', x: 5000.0, y: 1500.0}"

# 设置场地入口中心 (x3, y3)
ros2 service call /set_coordinate robocom_interfaces/srv/SetCoordinate \
  "{coordinate_name: 'entrance_center', x: 0.0, y: 0.0}"
```

也可以在 `src/robocom_bringup/config/bringup_params.yaml` 中预设这些值。

---

## 13. 常见问题

### Q: 编译时 `rosidl_generate_interfaces` 报错

```bash
# 确保安装了 rosidl 工具
sudo apt install -y ros-humble-rosidl-default-generators
```

### Q: `colcon build` 找不到 `ament_python`

```bash
sudo apt install -y python3-ament-package python3-colcon-common-extensions
```

### Q: USB 串口打不开

```bash
# 检查设备
ls -la /dev/ttyACM*

# 确认 udev 规则已加载
sudo udevadm control --reload-rules
sudo udevadm trigger

# 重新插拔 USB
# 如果仍不行，临时授予权限：
sudo chmod 666 /dev/ttyACM0
```

### Q: D435 相机无法识别

```bash
# 检查 USB 连接
lsusb | grep 8086

# 安装 realsense SDK（如果 pip 包不够用）
sudo apt install -y ros-humble-librealsense2*
```

### Q: UI 界面无法显示

```bash
# 检查 PySide6 安装
python3 -c "from PySide6.QtWidgets import QApplication; print('PySide6 OK')"

# 如果无显示器（SSH 连接），UI 节点会自动跳过
# 可以通过设置 QT_QPA_PLATFORM=offscreen 测试
export QT_QPA_PLATFORM=offscreen
```

### Q: 数学题识别一直超时

检查摄像头：

```bash
# 查看 USB 摄像头设备
ls -la /dev/video*

# 测试摄像头
python3 -c "
import cv2
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
print(f'读取摄像头0: {ret}')
cap.release()
"
```

### Q: 如何只调试导航逻辑而不启动视觉节点？

修改 `all_start.launch.py`，注释掉视觉相关节点即可。所有视觉节点在无硬件时也会自动降级为模拟模式。

### Q: 仓库更新后如何同步？

```bash
cd ~/ros2_ws/src/robocom_pc
git pull origin main
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

---

## 附录：架构总览

```
┌─────────────────────────────────────────────────────┐
│                   UI (PySide6)                       │
│   /start_mission (srv) → task_scheduler              │
└────────────────────┬────────────────────────────────┘
                     │ /match_start
                     ▼
┌─────────────────────────────────────────────────────┐
│               task_scheduler_node                     │
│   ├─ 比赛计时 180s 看门狗                             │
│   ├─ 数学题 20s 超时看门狗                            │
│   └─ 发布 /enable_motion                             │
└────────┬────────┬────────┬────────┬──────────────────┘
         │        │        │        │
         ▼        ▼        ▼        ▼
   localization  math_solver  navigation  arm_control
       │                          │            │
       │                     motion_control     │
       │                     (SBUS → STM32)     │
       │                          │            │
       └────────── vision ────────┘            │
                 (yolo + color + depth)         │
                                                │
                                          (UART → 机械臂)
```

### 通信主题（Topics）

| 发布者 | 话题 | 订阅者 |
|--------|------|--------|
| task_scheduler | /match_start, /enable_motion | localization, math_solver, navigation |
| localization | /robot_state, /block_info | navigation, UI |
| math_solver | /math_result | navigation, task_scheduler, UI |
| navigation | /motion_cmd, /arm_command, /mission_status, /estop, /yolo_start, /color_vision_start | motion_control, arm_control, UI, vision |
| motion_control | /motion_enabled | - |
| arm_control | /arm_current_state | motion_control（→ SBUS ch[5]） |
| yolo_detector | /grasp_complete, /detected_block_type, /arm_command | navigation, depth_helper |
| color_mask | /place_complete, /detected_exchange_id | navigation |
| depth_helper | /grasp_verified | - |

### SBUS 通道映射（USB CDC → STM32）

| SBUS 通道 | 映射 | 值范围 |
|-----------|------|--------|
| ch[0] | linear_y (侧移) | 352~1695 |
| ch[2] | linear_x (前进) | 352~1695 |
| ch[3] | angular_z (转向) | 352~1695 |
| ch[4] | SB 拨杆：步态选择 | 353/1024/1695 |
| ch[5] | 机械臂状态 | 352/688/1024/1359/1695 |
| ch[9] | SA 拨杆：使能/失能 | 353=失能, 1024=使能 |
