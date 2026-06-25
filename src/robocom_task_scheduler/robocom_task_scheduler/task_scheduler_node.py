#!/usr/bin/env python3
"""
task_scheduler_node.py — 任务编排调度主节点

核心职责（任务 11）：
  1. 一键启动 → 重置雷达坐标 + 开启数学题识别
  2. 等待数学题完成 → 启动导航循环
  3. 协调各节点的生命周期
  4. 监控比赛时间（180 秒总限时）
  5. 开机自启动入口

流程：
  IDLE → [一键启动] → RADAR_RESET + MATH_SOLVING
     → [20s timeout / OCR done] → NAVIGATING
     → [8 blocks delivered] → COMPLETED
"""

import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from robocom_interfaces.msg import MathResult, MissionStatus
from robocom_interfaces.srv import StartMission
from std_msgs.msg import String, Bool


class TaskSchedulerNode(Node):
    """任务编排调度主节点"""

    def __init__(self):
        super().__init__('task_scheduler_node')

        self.declare_parameter('match_duration', 180.0)   # 任务赛 180 秒
        self._match_duration = self.get_parameter('match_duration').value

        # ---------- 状态 ----------
        self._match_started = False
        self._match_start_time = 0.0
        self._math_solved = False
        self._math_timeout = 20.0
        self._mission_complete = False

        # ---------- 发布 ----------
        self.pub_start = self.create_publisher(String, '/match_start', 10)
        self.pub_enable = self.create_publisher(Bool, '/enable_motion', 10)
        self.pub_estop = self.create_publisher(Bool, '/estop', 10)

        # ---------- 订阅 ----------
        self.create_subscription(MathResult, '/math_result', self._math_result_cb, 10)
        self.create_subscription(MissionStatus, '/mission_status', self._mission_status_cb, 10)

        # ---------- 服务 ----------
        self.srv_start = self.create_service(
            StartMission, '/start_mission', self._start_mission_cb
        )

        # ---------- 定时器 ----------
        self.create_timer(0.5, self._watchdog)

        self.get_logger().info('TaskSchedulerNode 已启动，等待一键启动...')

    # ------------------------------------------------------------------
    def _start_mission_cb(self, request, response):
        """一键启动服务（由 UI 调用）"""
        if self._match_started:
            response.success = False
            response.message = '比赛已开始'
            return response

        self.get_logger().info('=== 一键启动！===')
        self._match_started = True
        self._match_start_time = time.time()
        self._math_solved = False
        self._mission_complete = False

        # 1. 发布 match_start 信号 → 重置雷达原点 + 启动数学题识别
        start_msg = String()
        start_msg.data = 'match_start'
        self.pub_start.publish(start_msg)

        # 2. 使能运动
        self.pub_enable.publish(Bool(data=True))

        response.success = True
        response.message = '比赛已启动'
        self.get_logger().info('已发布 match_start 和 enable_motion')

        # 3. 启动数学题超时看门狗（后台线程）
        thread = threading.Thread(target=self._math_watchdog, daemon=True)
        thread.start()

        return response

    # ------------------------------------------------------------------
    def _math_result_cb(self, msg: MathResult):
        """数学题完成回调"""
        if msg.success:
            self._math_solved = True
            self.get_logger().info(
                f'数学题已完成: {msg.expression} = {msg.result}, '
                f'高分区: {msg.high_zone_id}'
            )

    # ------------------------------------------------------------------
    def _mission_status_cb(self, msg: MissionStatus):
        """任务状态更新"""
        if msg.status == 'COMPLETED':
            self._mission_complete = True
            elapsed = time.time() - self._match_start_time
            self.get_logger().info(
                f'=== 比赛完成！用时: {elapsed:.1f}s, 得分: {msg.score} ==='
            )

    # ------------------------------------------------------------------
    def _math_watchdog(self):
        """数学题超时看门狗：20 秒超时"""
        time.sleep(self._math_timeout)
        if not self._math_solved:
            self.get_logger().warn(
                f'数学题超时 ({self._math_timeout}s)，跳过继续比赛'
            )
            # 超时也视为"已处理"，导航节点将继续

    # ------------------------------------------------------------------
    def _watchdog(self):
        """比赛全局看门狗"""
        if not self._match_started:
            return

        elapsed = time.time() - self._match_start_time

        # 比赛限时
        if elapsed > self._match_duration:
            self.get_logger().warn(f'比赛超时 ({self._match_duration}s)')
            self.pub_estop.publish(Bool(data=True))
            self._match_started = False


def main(args=None):
    rclpy.init(args=args)
    node = TaskSchedulerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
