#!/usr/bin/env python3
"""radar.launch.py — 启动 Mid-360 激光雷达驱动"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='livox_ros2_driver',
            executable='livox_ros2_driver_node',
            name='livox_lidar',
            output='screen',
            parameters=[{
                'xfer_format': 0,
                'multi_topic': 0,
                'data_src': 0,
                'publish_freq': 10.0,
                'output_data_type': 0,
            }]
        ),
    ])
