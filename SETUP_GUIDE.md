# ROBOCON 8DOF 上位机部署与联调指南

本文档说明如何在 Ubuntu 22.04 / ROS2 Humble 上部署本仓库，并与 DM02/8DOF 四足控制固件联调。

## 1. 系统准备

```bash
sudo apt update
sudo apt install -y \
  git curl wget vim htop net-tools \
  cmake build-essential \
  python3-pip python3-venv python3-dev
```

Python 建议 `>= 3.10`：

```bash
python3 --version
```

## 2. 安装 ROS2 Humble

```bash
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install -y software-properties-common
sudo add-apt-repository -y universe

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool

sudo rosdep init
rosdep update

source /opt/ros/humble/setup.bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
```

## 3. 获取和编译上位机

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/petershe1by/robocon26_pc.git robocom_pc

cd ~/ros2_ws/src/robocom_pc
sudo bash scripts/install_deps.sh

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
```

如果 `ament_python` 或接口生成报错：

```bash
sudo apt install -y \
  python3-ament-package \
  python3-colcon-common-extensions \
  ros-humble-rosidl-default-generators
```

## 4. Python 依赖

```bash
pip3 install --upgrade pip
pip3 install \
  sympy \
  pyserial \
  PySide6 \
  opencv-python \
  numpy \
  ultralytics \
  pyrealsense2 \
  transforms3d
```

可选：

```bash
pip3 install paddleocr paddlepaddle pyttsx3
```

OCR 包较大；未安装时数学题节点可按模拟/失败路径继续运行。

## 5. USB 设备权限

```bash
cd ~/ros2_ws/src/robocom_pc
sudo bash scripts/setup_udev_rules.sh
sudo udevadm control --reload-rules
sudo udevadm trigger
```

重新插拔 USB 设备后检查：

```bash
ls -la /dev/ttyACM*
ls -la /dev/ttyUSB*
ls -la /dev/video*
```

常见设备：

| 设备 | 典型节点 | 用途 |
| --- | --- | --- |
| STM32 USB CDC | `/dev/ttyACM0` | 8DOF 四足 SBUS/调试串口 |
| 机械臂 UART | `/dev/ttyUSB0` | 文本命令控制 |
| USB 相机 | `/dev/video*` | OCR / 颜色识别 |
| RealSense D435 | USB 3.0 | 深度辅助 |

## 6. 下位机 8DOF 固件确认

在接入上位机前，建议先用下位机 USB CDC 串口确认四足固件工作正常。

串口工具可填 `115200 8N1`。USB CDC 实际不依赖该波特率。命令为单字符，通常不需要回车。

基础检查：

```text
p        打印当前状态
Y        打印 SBUS 诊断
8        选择全部 8 个腿部电机
s        站立
x        idle 并停止控制
!        全部电机急停
```

单腿 MIT 调试参考流程：

```text
L        选择 LF
2        选择单腿 HIP+KNEE
r        RX-only，停止周期控制
e        清错
o        probe，确认闭环和编码器
p        查看 online/loop/enc/fault
z        可选：设置用户零点
b        MIT boot，打开 PID
a/d      HIP -/+60 deg
j/l      KNEE -/+60 deg
x        idle 并停止控制
```

安全建议：

- 首次调试让腿悬空或可靠支撑。
- 每次只发一个动作命令，等 `user` 接近 `tgt` 后再继续。
- 看到 `ENC jump`、`SAFETY`、`enc=0` 时先 `x` 或 `r` 停止。
- 不要从高负载折叠姿态直接硬撑起整机。

## 7. 8DOF 遥控器 / SBUS 映射确认

推荐 ET07 + RF207S 设置：

| 通道 | 控件 | 用途 |
| --- | --- | --- |
| CH1 | 右摇杆横向 | 转向 |
| CH2 | 右摇杆纵向 | 前进/后退 |
| CH3 | 左摇杆纵向 | 速度档 |
| CH5 | SB 三挡 | 主模式：LOW 停止，MID 站立，HIGH 步态 |
| CH6 | V1 | 机械臂 J0 |
| CH7 | V2 | 机械臂 J1 |
| CH8 | SC 三挡 | 子模式 |
| CH9 | SD 弹簧回中 | 清故障 + 急停 |

ET07 中进入 `通用功能 -> 辅助通道`，确认：

| 通道 | 推荐设置 |
| --- | --- |
| CH5 | SB |
| CH6 | V1 |
| CH7 | V2 |
| CH8 | SC |
| CH9 | SD |

若 CH8 未分配给 SC，机械臂子模式无法进入。

## 8. 启动上位机

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch robocom_bringup all_start.launch.py
```

也可以单独启动运动控制节点：

```bash
ros2 run robocom_motion_control motion_control_node --ros-args \
  -p serial_port:=/dev/ttyACM0
```

UI 或命令行启动任务：

```bash
ros2 service call /start_mission robocom_interfaces/srv/StartMission "{}"
```

## 9. 上位机到四足联调

先确认上位机能发布运动指令：

```bash
ros2 topic echo /motion_cmd
ros2 topic echo /motion_enabled
```

手动发布前进：

```bash
ros2 topic pub /motion_cmd robocom_interfaces/msg/MotionCmd \
  "{linear_x: 0.5, linear_y: 0.0, angular_z: 0.0, gait_mode: 1, enable: true, estop: false}" -1
```

手动发布原地转向：

```bash
ros2 topic pub /motion_cmd robocom_interfaces/msg/MotionCmd \
  "{linear_x: 0.0, linear_y: 0.0, angular_z: 0.4, gait_mode: 1, enable: true, estop: false}" -1
```

急停：

```bash
ros2 topic pub /estop std_msgs/msg/Bool "{data: true}" -1
```

下位机串口发送 `Y`，检查：

- SBUS 帧计数是否增长；
- `failsafe` 是否为 0；
- CH1/CH2/CH3/CH5/CH8/CH9 raw/norm 是否变化；
- CH5 是否进入 MID/HIGH；
- 解码出的步态命令是否符合预期。

如果上位机指令发出但下位机无动作，优先检查通道语义是否匹配 8DOF 固件。尤其注意：8DOF 固件不以真实横移为核心，`linear_y` 不应作为侧移主输入。

## 10. 视觉和模型

YOLO 模型文件不放入 git。需要将模型放到：

```bash
mkdir -p ~/ros2_ws/src/robocom_pc/src/robocom_vision/models
cp /path/to/block_detector.pt \
  ~/ros2_ws/src/robocom_pc/src/robocom_vision/models/block_detector.pt
```

检查相机：

```bash
ls -la /dev/video*
python3 - <<'PY'
import cv2
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
print("camera0:", ret, None if frame is None else frame.shape)
cap.release()
PY
```

D435：

```bash
python3 -c "import pyrealsense2 as rs; print('realsense OK')"
```

## 11. 坐标标定

比赛场地中需要标定：

```bash
ros2 service call /set_coordinate robocom_interfaces/srv/SetCoordinate \
  "{coordinate_name: 'block_origin', x: 1000.0, y: 2000.0}"

ros2 service call /set_coordinate robocom_interfaces/srv/SetCoordinate \
  "{coordinate_name: 'exchange_origin', x: 5000.0, y: 1500.0}"

ros2 service call /set_coordinate robocom_interfaces/srv/SetCoordinate \
  "{coordinate_name: 'entrance_center', x: 0.0, y: 0.0}"
```

注意：若定位和导航分别运行在独立 ROS2 进程中，不能依赖 Python 单例跨进程共享坐标。联调时应确认导航节点实际使用的坐标已经更新。

## 12. 开机自启动

```bash
cd ~/ros2_ws/src/robocom_pc
sudo bash scripts/setup_autostart.sh

journalctl -u robocom-autostart -f
```

停止自启动：

```bash
sudo systemctl stop robocom-autostart
sudo systemctl disable robocom-autostart
```

## 13. 常见问题

| 现象 | 处理 |
| --- | --- |
| `/dev/ttyACM0` 不存在 | 检查 USB 线、STM32 是否枚举、`dmesg -w` |
| 串口权限不足 | 执行 `setup_udev_rules.sh`，或临时 `sudo chmod 666 /dev/ttyACM0` |
| 下位机 `Y` 无 SBUS 帧 | 检查 `motion_control_node` 是否连接到正确串口 |
| CH5 未进入 MID/HIGH | 检查上位机是否输出 8DOF 主模式通道 |
| 有 SBUS 但不走 | 检查电机 online/enc/fault，先 `8`、`s`、`x` 手动验证 |
| 只有前进没有转向 | 检查 CH1/CH2 映射是否和固件一致 |
| 机器人试图侧移 | 不符合 8DOF 能力边界，应改为转向/弧线转弯策略 |
| OCR 超时 | 检查相机编号、PaddleOCR 安装；超时不会阻塞任务继续 |
| UI 无法显示 | 本地显示器运行；SSH 下可设置 `QT_QPA_PLATFORM=offscreen` 测试 |

## 14. 快速检查清单

```text
1. 下位机串口 p 正常，online=1 enc=1 fault=0
2. 下位机 Y 能看到 SBUS 帧
3. CH5 LOW/MID/HIGH 主模式正确
4. CH1/CH2 能触发转向和前进/后退
5. CH3 速度档能被识别
6. CH9 HIGH 边沿能触发安全动作
7. ROS2 /motion_cmd、/estop、/mission_status 正常
8. UI 一键启动能发布 /match_start
9. 视觉节点在无硬件时能降级，不阻塞整机调试
```
