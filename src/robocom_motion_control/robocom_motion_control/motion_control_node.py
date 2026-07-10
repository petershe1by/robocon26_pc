#!/usr/bin/env python3
"""motion_control_node.py - 运动控制 + SBUS 通信"""

import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from robocom_interfaces.msg import MotionCmd
from std_msgs.msg import Bool
from .sbus_bridge import SBUSBridge, SWITCH_LOW, SWITCH_MID, SWITCH_HIGH


class MotionControlNode(Node):
    def __init__(self):
        super().__init__("motion_control_node")
        self.declare_parameter("serial_port", "auto")
        self.declare_parameter("serial_baud", 100000)
        self.declare_parameter("sbus_freq", 100.0)
        self.declare_parameter("motion_watchdog_sec", 5.0)

        port = self.get_parameter("serial_port").value
        baud = self.get_parameter("serial_baud").value
        freq = self.get_parameter("sbus_freq").value
        self._watchdog_timeout = self.get_parameter("motion_watchdog_sec").value

        self._bridge = SBUSBridge(port=port, baud=baud)
        if self._bridge.connect():
            self.get_logger().info(f"USB CDC 已连接: {self._bridge.port}")
            self._bridge.start_loop(freq)
        else:
            self.get_logger().error("USB CDC 连接失败")

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
            self._bridge.set_channel(8, SWITCH_HIGH)  # CH9 = 安全瞬时触发
            self._bridge.set_channel(4, SWITCH_LOW)   # CH5 = LOW = 停止
            return
        if not msg.enable:
            self._bridge.set_channel(4, SWITCH_LOW)   # CH5 = LOW = 停止
            self._bridge.set_joystick(0, 0.0)
            self._bridge.set_joystick(1, 0.0)
            return

        # CH3 = 速度档（基于 gait_mode）
        speed_map = {0: SWITCH_LOW, 1: SWITCH_MID, 2: SWITCH_HIGH}
        self._bridge.set_channel(2, speed_map.get(msg.gait_mode, SWITCH_MID))

        # CH1 = angular_z, CH2 = linear_x（8DOF 无横移）
        self._bridge.set_joystick(0, msg.angular_z)
        self._bridge.set_joystick(1, msg.linear_x)

        # CH5 主模式：有运动→HIGH(步态), 静止→MID(站立)
        has_motion = abs(msg.angular_z) > 0.01 or abs(msg.linear_x) > 0.01
        if has_motion:
            self._bridge.set_channel(4, SWITCH_HIGH)
        else:
            self._bridge.set_channel(4, SWITCH_MID)


    def _enable_cb(self, msg: Bool):
        self._enabled = msg.data
        if msg.data:
            self._bridge.set_channel(4, SWITCH_MID)  # CH5 = MID = 站立待命
        else:
            self._bridge.set_channel(4, SWITCH_LOW)   # CH5 = LOW = 停止
            self._bridge.set_joystick(0, 0.0)
            self._bridge.set_joystick(1, 0.0)
        self.pub_enabled.publish(msg)

    def _estop_cb(self, msg: Bool):
        if msg.data:
            self._bridge.set_channel(8, SWITCH_HIGH)  # CH9 = 安全瞬时触发
            self._bridge.set_channel(4, SWITCH_LOW)   # CH5 = LOW = 停止

    def _watchdog_check(self):
        if not self._enabled:
            return
        elapsed = time.time() - self._last_cmd_time
        if elapsed > self._watchdog_timeout:
            self._bridge.set_joystick(0, 0.0)
            self._bridge.set_joystick(1, 0.0)
            self._bridge.set_channel(4, SWITCH_MID)  # CH5 = MID = 站立保持
            self.get_logger().warn(f"看门狗: 无指令 ({elapsed:.0f}s)")

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

if __name__ == "__main__":
    main()
