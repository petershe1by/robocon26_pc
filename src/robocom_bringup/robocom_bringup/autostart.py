#!/usr/bin/env python3
"""autostart.py - 开机自启动入口（任务 10）"""

import subprocess
import sys
import time
import os


def _source_setup(path: str):
    """Source 一个 bash setup 脚本并将环境变量合并到当前进程"""
    if not os.path.exists(path):
        print(f'[autostart] 文件不存在，跳过: {path}')
        return
    try:
        result = subprocess.run(
            ['bash', '-c', f'source {path} && env'],
            capture_output=True, text=True, check=True
        )
        for line in result.stdout.splitlines():
            if '=' not in line:
                continue
            k, v = line.split('=', 1)
            if k in ('SHLVL', 'PS1', '_'):
                continue
            os.environ[k] = v
        print(f'[autostart] 已加载环境: {path}')
    except (subprocess.CalledProcessError, FileNotFoundError) as e:
        print(f'[autostart] 加载环境失败 {path}: {e}')


def main():
    """开机自动启动雷达和 ROS2 节点（UI 已包含在 all_start.launch.py 中）"""
    print('[autostart] ROBOCON 上位机自启动...')
    sys.stdout.flush()

    # 0. 加载 ROS2 环境（systemd 不自带 bashrc）
    ros_setup = '/opt/ros/humble/setup.bash'
    ws_setup = os.path.expanduser('~/ros2_ws/install/setup.bash')
    _source_setup(ros_setup)
    _source_setup(ws_setup)

    procs = []

    # 1. 启动雷达驱动
    radar_cmd = [
        'ros2', 'launch', 'livox_ros2_driver', 'mid360_launch.py'
    ]
    try:
        proc_radar = subprocess.Popen(
            radar_cmd,
            stdout=sys.stdout,
            stderr=sys.stderr
        )
        procs.append(proc_radar)
        print(f'[autostart] 雷达驱动已启动 (PID: {proc_radar.pid})')
        sys.stdout.flush()
    except FileNotFoundError as e:
        print(f'[autostart] 雷达驱动启动失败 (livox_ros2_driver 未安装?): {e}')
        sys.stdout.flush()

    # 等待雷达初始化
    time.sleep(5)

    # 2. 启动所有 ROS2 节点（含 UI）
    bringup_cmd = ['ros2', 'launch', 'robocom_bringup', 'all_start.launch.py']
    try:
        proc_ros = subprocess.Popen(
            bringup_cmd,
            stdout=sys.stdout,
            stderr=sys.stderr
        )
        procs.append(proc_ros)
        print(f'[autostart] ROS2 节点已启动 (PID: {proc_ros.pid})')
        sys.stdout.flush()
    except FileNotFoundError as e:
        print(f'[autostart] ROS2 节点启动失败 (robocom_bringup 未编译?): {e}')
        sys.stdout.flush()

    # 3. 保持运行，等待 Ctrl+C
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        print('[autostart] 正在关闭...')
        sys.stdout.flush()
        for p in procs:
            try:
                p.terminate()
                p.wait(timeout=3)
            except Exception:
                pass


if __name__ == '__main__':
    main()
