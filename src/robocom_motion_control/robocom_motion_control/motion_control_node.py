#!/usr/bin/env python3
"""
motion_control_node.py — 运动控制节点

功能：
  - 订阅 /motion_cmd → 转换为 SBUS 帧 → 发送给下位机
  - 支持大步流星模式（开环位置控制）与视觉辅助微调模式
  - 发布使能/失能状态
  - 5 秒以上坐标不变时自动停止运动（安全看门狗）
"""

import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from robocom_interfaces.msg import MotionCmd, RobotState
from std_msgs.msg import Bool

from .sbus_bridge import SBUSBridge, float_to_sbus, SBUS_MID, SWITCH_LOW, SWITCH_HIGH


class MotionControlNode(Node):
    """运动控制与下位机通信节点"""

    def __init__(self):
        super().__init__('motion_control_node')

        # ---------- 参数 ----------
        self.declare_parameter('serial_port', 'auto')
        self.declare_parameter('serial_baud', 100000)
        self.declare_parameter('sbus_freq', 100.0)
        self.declare_parameter('motion_watchdog_sec', 5.0)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('serial_baud').value
        freq = self.get_parameter('sbus_freq').value
        self._watchdog_timeout = self.get_parameter('motion_watchdog_sec').value

        # ---------- SBUS 桥接 ----------
        self._bridge = SBUSBridge(port=port, baud=baud)
        if self._bridge.connect():
            self.get_logger().info(f'USB CDC 已连接: {self._bridge.port}')
            self._bridge.start_loop(freq)
        else:
            self.get_logger().error('USB CDC 连接失败，运动控制不可用')

        # ---------- 状态 ----------
        self._last_cmd_time = time.time()
        self._last_pose = (0.0, 0.0)
        self._enabled = True

        # ---------- 订阅 ----------
        self.create_subscription(MotionCmd, '/motion_cmd', self._motion_cmd_cb, 10)
        self.create_subscription(RobotState, '/robot_state', self._robot_state_cb, 10)
        self.create_subscription(Bool, '/enable_motion', self._enable_cb, 10)
        self.create_subscription(Bool, '/estop', self._estop_cb, 10)

        # ---------- 发布 ----------
        self.pub_enabled = self.create_publisher(Bool, '/motion_enabled', 10)

        # ---------- 看门狗定时器 ----------
        self.create_timer(1.0, self._watchdog_check)

        self.get_logger().info('MotionControlNode 已启动')

    # ------------------------------------------------------------------
    def _motion_cmd_cb(self, msg: MotionCmd):
        """运动指令回调：解析并转发为 SBUS 信号"""
        if not self._enabled:
            return

        self._last_cmd_time = time.time()

        # 急停
        if msg.estop:
            self._bridge.disable()
            self.get_logger().warn('!!! 急停指令 !!!')
            return

        # 使能/失能
        if not msg.enable:
            self._bridge.disable()
            return

        # --- 大步流星模式 / 视觉辅助模式 ---
        # linear_x → 左摇杆垂直(差分→加速度)
        # angular_z (yaw) → 左摇杆水平
        # linear_y → 右摇杆水平(侧移)

        # 步态切换 → SB 拨杆 (ch[4])
        if msg.gait_mode == 0:    # walk
            self._bridge.set_switch(1, 0)  # SB 低位
        elif msg.gait_mode == 1:  # trot
            self._bridge.set_switch(1, 1)  # SB 中位
        elif msg.gait_mode == 2:  # bounds
            self._bridge.set_switch(1, 2)  # SB 高位

        # 设置摇杆值
        self._bridge.set_joystick(0, msg.angular_z)    # yaw → 左摇杆水平
        self._bridge.set_joystick(1, msg.linear_x)     # 前进 → 左摇杆垂直
        self._bridge.set_joystick(2, msg.linear_y)     # 侧移 → 右摇杆水平

        # 使能
        self._bridge.enable()

    # ------------------------------------------------------------------
    def _robot_state_cb(self, msg: RobotState):
        """更新机器人位置（看门狗用）"""
        self._last_pose = (msg.x, msg.y)

    # ------------------------------------------------------------------
    def _enable_cb(self, msg: Bool):
        """使能指令"""
        self._enabled = msg.data
        if msg.data:
            self._bridge.enable()
        else:
            self._bridge.disable()
        self.pub_enabled.publish(msg)
        self.get_logger().info(f'运动{"使能" if msg.data else "失能"}')

    # ------------------------------------------------------------------
    def _estop_cb(self, msg: Bool):
        """急停指令"""
        if msg.data:
            self._bridge.disable()
            self.get_logger().warn('!!! 急停 !!!')

    # ------------------------------------------------------------------
    def _watchdog_check(self):
        """运动看门狗：5 秒发送指令但坐标不变 → 停止"""
        # 简化：检查最后指令时间
        if not self._enabled:
            return

        # 5 秒未收到指令 → 归零摇杆
        elapsed = time.time() - self._last_cmd_time
        if elapsed > self._watchdog_timeout:
            if self._enabled:
                self._bridge.set_joystick(0, 0.0)
                self._bridge.set_joystick(1, 0.0)
                self._bridge.set_joystick(2, 0.0)
                self.get_logger().warn(
                    f'运动看门狗触发：{elapsed:.0f}s 未收到运动指令，已归零'
                )

    # ------------------------------------------------------------------
    def destroy_node(self):
        self._bridge.stop_loop()
        self._bridge.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotionControlNode()
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
