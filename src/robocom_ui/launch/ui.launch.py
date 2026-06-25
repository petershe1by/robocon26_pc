#!/usr/bin/env python3
"""
ui.launch.py — 启动 UI 节点
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robocom_ui',
            executable='robocom_ui',
            name='robocom_ui',
            output='screen',
        ),
    ])
