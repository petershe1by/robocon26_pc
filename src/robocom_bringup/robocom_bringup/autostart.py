#!/usr/bin/env python3
"""autostart.py - 开机自启动入口（任务 10）"""

import subprocess
import sys
import time


def main():
    """开机自动启动雷达和 ROS2 节点（UI 已包含在 all_start.launch.py 中）"""
    print('[autostart] ROBOCON 上位机自启动...')
    procs = []

    # 1. 启动雷达驱动
    radar_cmd = [
        'ros2', 'launch', 'livox_ros2_driver', 'mid360_launch.py'
    ]
    try:
        proc_radar = subprocess.Popen(
            radar_cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        procs.append(proc_radar)
        print(f'[autostart] 雷达驱动已启动 (PID: {proc_radar.pid})')
    except FileNotFoundError:
        print('[autostart] livox_ros2_driver 未安装，跳过雷达启动')

    # 等待雷达初始化
    time.sleep(5)

    # 2. 启动所有 ROS2 节点（含 UI）
    bringup_cmd = ['ros2', 'launch', 'robocom_bringup', 'all_start.launch.py']
    try:
        proc_ros = subprocess.Popen(
            bringup_cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        procs.append(proc_ros)
        print(f'[autostart] ROS2 节点已启动 (PID: {proc_ros.pid})')
    except FileNotFoundError:
        print('[autostart] robocom_bringup 未安装')

    # 3. 保持运行，等待 Ctrl+C
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        print('[autostart] 正在关闭...')
        for p in procs:
            try:
                p.terminate()
                p.wait(timeout=3)
            except Exception:
                pass


if __name__ == '__main__':
    main()
