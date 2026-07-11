#!/usr/bin/env python3
"""motion_control_node.py — 运动控制 + USB 虚拟遥控器"""

import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from robocom_interfaces.msg import MotionCmd
from std_msgs.msg import Bool
from .virtual_remote import (
    VirtualRemoteOutput,
    MODE_STAND_HOLD, MODE_GAIT_ONLY, MODE_IDLE,
    MODE_STAND_ARM, MODE_STAND_WHEEL,
    SPEED_LOW, SPEED_MID, SPEED_HIGH,
    DEADMAN_HELD, MOTION_ENABLE, SMOOTH_STOP,
)


class MotionControlNode(Node):
    def __init__(self):
        super().__init__("motion_control_node")
        self.declare_parameter("serial_port", "auto")
        self.declare_parameter("motion_watchdog_sec", 5.0)

        port = self.get_parameter("serial_port").value
        self._watchdog_timeout = self.get_parameter("motion_watchdog_sec").value

        self._vremote = VirtualRemoteOutput(port=port)
        if self._vremote.connect():
            self.get_logger().info(f"USB 虚拟遥控器已连接: {self._vremote.port}")
            self._vremote.start_loop(50.0)
        else:
            self.get_logger().error("USB 虚拟遥控器连接失败")

        self._last_cmd_time = time.time()
        self._enabled = False

        self.create_subscription(MotionCmd, "/motion_cmd", self._motion_cmd_cb, 10)
        self.create_subscription(Bool, "/enable_motion", self._enable_cb, 10)
        self.create_subscription(Bool, "/estop", self._estop_cb, 10)

        self.pub_enabled = self.create_publisher(Bool, "/motion_enabled", 10)
        self.create_timer(1.0, self._watchdog_check)
        self.get_logger().info("MotionControlNode 已启动")

    def _motion_cmd_cb(self, msg: MotionCmd):
        if not self._enabled:
            return
        self._last_cmd_time = time.time()

        if msg.estop:
            self._vremote.emergency_stop()
            return
        if not msg.enable:
            self._vremote.send_safety_zero()
            return

        # 速度档: gait_mode → speed_permille
        speed_map = {0: SPEED_LOW, 1: SPEED_MID, 2: SPEED_HIGH}
        self._vremote.set_speed_axis(speed_map.get(msg.gait_mode, SPEED_MID) / 1000.0)

        # 运动: angular_z(转向), linear_x(前进/后退)
        self._vremote.set_motion(msg.linear_x, msg.angular_z)

        # 九宫格模式: 有运动→HIGH+LOW(步态), 静止→MID+LOW(站立)
        has_motion = abs(msg.angular_z) > 0.01 or abs(msg.linear_x) > 0.01
        if has_motion:
            self._vremote.set_mode(2, 0)   # HIGH+LOW → 纯步态
        else:
            self._vremote.set_mode(1, 0)   # MID+LOW → 站立

        self._vremote.refresh_autonomy_permit()

    def _enable_cb(self, msg: Bool):
        self._enabled = msg.data
        if msg.data:
            self._vremote.set_mode(1, 0)   # MID+LOW = 站立待命
            self._vremote.refresh_autonomy_permit()
        else:
            self._vremote.send_safety_zero()
            self._vremote = VirtualRemoteOutput()  # 强制新 session
        self.pub_enabled.publish(msg)

    def _estop_cb(self, msg: Bool):
        if msg.data:
            self._vremote.emergency_stop()

    def _watchdog_check(self):
        if not self._enabled:
            return
        elapsed = time.time() - self._last_cmd_time
        if elapsed > self._watchdog_timeout:
            self._vremote.smooth_stop()
            self.get_logger().warn(f"看门狗: 无指令 ({elapsed:.0f}s)")

    def destroy_node(self):
        self._vremote.stop_loop()
        self._vremote.disconnect()
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

if __name__ == "__main__":
    main()
