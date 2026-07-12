#!/usr/bin/env python3
"""all_start.launch.py - 启动所有 ROS2 节点"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
import os


def generate_launch_description():
    # 录包: 记录所有话题到带时间戳的目录
    bag_dir = os.path.join(
        os.path.expanduser('~'),
        'ros2_ws',
        'bags',
        'robocon_' + os.popen('date +%Y%m%d_%H%M%S').read().strip()
    )

    return LaunchDescription([
        # === 雷达驱动 (Livox Mid-360) ===
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(os.path.expanduser('~'), 'livox_ros_driver2',
                             'launch_ROS2', 'msg_MID360_launch.py')
            ),
        ),

        # IMU 滤波
        Node(package='robocom_localization', executable='imu_filter',
             name='imu_filter', output='screen'),

        # EKF 融合定位 (IMU + 雷达里程计)
        Node(package='robot_localization', executable='ekf_node',
             name='ekf_filter_node', output='screen',
             parameters=['src/robocom_localization/config/ekf.yaml']),

        # === 任务调度器 ===
        # 任务调度器（必须先启动，提供 /start_mission 服务）
        Node(package='robocom_task_scheduler', executable='task_scheduler_node',
             name='task_scheduler', output='screen'),

        # 定位
        Node(package='robocom_localization', executable='localization_node',
             name='localization', output='screen',
             parameters=[{'odom_topic': '/odometry/filtered'}]),

        # 运动控制 + USB CDC
        Node(package='robocom_motion_control', executable='motion_control_node',
             name='motion_control', output='screen',
             parameters=[{'serial_port': 'auto'}]),

        # 导航 + 安全 + 任务循环
        Node(package='robocom_navigation', executable='navigation_node',
             name='navigation', output='screen'),

        # 数学题求解（低功耗等待 match_start）
        Node(package='robocom_math_solver', executable='math_solver_node',
             name='math_solver', output='screen'),

        # 机械臂控制
        Node(package='robocom_arm_control', executable='arm_control_node',
             name='arm_control', output='screen'),

        # 视觉节点（延迟启动，等待 /yolo_start 等触发）
        Node(package='robocom_vision', executable='yolo_block_detector',
             name='yolo_block_detector', output='screen'),
        Node(package='robocom_vision', executable='color_mask_detector',
             name='color_mask_detector', output='screen'),
        Node(package='robocom_vision', executable='depth_helper',
             name='depth_helper', output='screen'),

        # UI 界面（如果启动文件已包含 UI，autostart.py 中则不再重复拉起）
        Node(package='robocom_ui', executable='robocom_ui',
             name='robocom_ui', output='screen'),

        # === 录包（任务 12）===
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a',
                 '-o', bag_dir],
            name='rosbag_record',
            output='screen',
        ),
    ])
