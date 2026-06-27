#!/usr/bin/env python3
"""motion_control_node.py - 运动控制 + SBUS 通信"""

import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from robocom_interfaces.msg import MotionCmd
from std_msgs.msg import Bool, Int8
from .sbus_bridge import SBUSBridge


# 机械臂状态 → SBUS ch[5] 值映射
ARM_SBUS_MAP = {
    0: 352,    # HOME
    1: 688,    # VISION_SCAN
    2: 1024,   # VISION_AID
    3: 1359,   # GRASP
    4: 1695,   # PLACE
}


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
        self._enabled = True

        self.create_subscription(MotionCmd, "/motion_cmd", self._motion_cmd_cb, 10)
        self.create_subscription(Bool, "/enable_motion", self._enable_cb, 10)
        self.create_subscription(Bool, "/estop", self._estop_cb, 10)
        # 机械臂状态 → SBUS ch[5]
        self.create_subscription(Int8, "/arm_current_state", self._arm_state_cb, 10)

        self.pub_enabled = self.create_publisher(Bool, "/motion_enabled", 10)
        self.create_timer(1.0, self._watchdog_check)
        self.get_logger().info("MotionControlNode 已启动")

    def _motion_cmd_cb(self, msg: MotionCmd):
        if not self._enabled:
            return
        self._last_cmd_time = time.time()
        if msg.estop:
            self._bridge.disable()
            return
        if not msg.enable:
            self._bridge.disable()
            return

        gait_cfg = {0: (1, 0), 1: (1, 1), 2: (1, 2)}
        if msg.gait_mode in gait_cfg:
            self._bridge.set_switch(*gait_cfg[msg.gait_mode])

        self._bridge.set_joystick(0, msg.angular_z)
        self._bridge.set_joystick(1, msg.linear_x)
        self._bridge.set_joystick(2, msg.linear_y)
        self._bridge.enable()

    def _arm_state_cb(self, msg: Int8):
        """机械臂状态 → SBUS ch[5], 让 STM32 知道当前机械臂在做什么"""
        sbus_val = ARM_SBUS_MAP.get(msg.data, 1024)
        self._bridge.set_channel(5, sbus_val)

    def _enable_cb(self, msg: Bool):
        self._enabled = msg.data
        if msg.data:
            self._bridge.enable()
        else:
            self._bridge.disable()
        self.pub_enabled.publish(msg)

    def _estop_cb(self, msg: Bool):
        if msg.data:
            self._bridge.disable()

    def _watchdog_check(self):
        if not self._enabled:
            return
        if time.time() - self._last_cmd_time > self._watchdog_timeout:
            for ch in range(4):
                self._bridge.set_joystick(ch, 0.0)
            self.get_logger().warn(f"看门狗: 无指令")

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