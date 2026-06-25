#!/bin/bash
# install_deps.sh — 安装 ROBOCON 上位机依赖
# 在 R7 5700ESU 小电脑上运行

set -e

echo "=== 安装 ROS2 依赖 ==="
sudo apt update
sudo apt install -y \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool \
    ros-humble-rclpy \
    ros-humble-sensor-msgs \
    ros-humble-nav-msgs \
    ros-humble-tf2-ros \
    ros-humble-launch-ros \
    ros-humble-cv-bridge

echo "=== 安装 Python 依赖 ==="
pip install --upgrade pip
pip install \
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

echo "=== 安装 Livox SDK (Mid-360) ==="
# 参考: https://github.com/Livox-SDK/livox_ros_driver2
cd ~
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd livox_ros_driver2
./build.sh humble

echo "=== 编译工作区 ==="
cd ~/ros2_ws  # 将 robocom_pc 链接/复制到 src/
colcon build --symlink-install
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

echo "=== 完成 ==="
echo "运行: ros2 launch robocom_bringup all_start.launch.py"
echo "UI:   ros2 run robocom_ui robocom_ui"
echo "自启动: ros2 run robocom_bringup autostart"
