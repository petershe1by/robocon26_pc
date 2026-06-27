#!/usr/bin/env python3
"""arm_control_node.py — 机械臂 5 状态机控制节点"""

import time
import rclpy
from rclpy.node import Node
from robocom_interfaces.msg import ArmCommand
from std_msgs.msg import Bool, Int8


ARM_STATE_NAMES = ["HOME", "VISION_SCAN", "VISION_AID", "GRASP", "PLACE"]

try:
    import serial
except ImportError:
    serial = None


class ArmControlNode(Node):
    """机械臂 5 状态机, 发布 /arm_current_state 给 SBUS 通道"""

    def __init__(self):
        super().__init__("arm_control_node")
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("serial_baud", 115200)

        port = self.get_parameter("serial_port").value
        baud = self.get_parameter("serial_baud").value

        self._serial = None
        if serial is not None:
            try:
                self._serial = serial.Serial(port, baud, timeout=0.5)
                self.get_logger().info(f"机械臂串口已连接: {port}")
            except Exception as e:
                self.get_logger().warn(f"机械臂串口连接失败: {e}")

        self._current_state = 0
        self._suction_on = False

        self._state_poses = {
            0: "HOME\n", 1: "VISION_SCAN\n", 2: "VISION_AID\n",
            3: "GRASP\n", 4: "PLACE\n",
        }

        self.create_subscription(ArmCommand, "/arm_command", self._arm_cmd_cb, 10)

        # 发布机械臂当前状态（供 motion_control 转发到 SBUS）
        self.pub_arm_state = self.create_publisher(Int8, "/arm_current_state", 10)
        self.pub_at_target = self.create_publisher(Bool, "/arm_at_target", 10)

        self.get_logger().info("ArmControlNode 已启动, 状态: HOME")

    def _arm_cmd_cb(self, msg: ArmCommand):
        if not msg.command_valid:
            return
        if not (0 <= msg.state <= 4):
            return
        if msg.suction_on != self._suction_on:
            self._set_suction(msg.suction_on)
        if msg.state != self._current_state:
            self._transition_to(msg.state)

    def _transition_to(self, new_state: int):
        old = ARM_STATE_NAMES[self._current_state]
        new = ARM_STATE_NAMES[new_state]
        self.get_logger().info(f"机械臂: {old} → {new}")

        if self._serial and self._serial.is_open:
            try:
                self._serial.write(self._state_poses[new_state].encode())
                time.sleep(0.1)
                resp = self._serial.readline().decode().strip()
            except Exception as e:
                self.get_logger().error(f"串口指令失败: {e}")
        else:
            time.sleep(0.5)

        self._current_state = new_state

        # 发布当前状态 → motion_control_node 收到后写入 SBUS ch[5]
        self.pub_arm_state.publish(Int8(data=new_state))
        self.pub_at_target.publish(Bool(data=True))

    def _set_suction(self, on: bool):
        self._suction_on = on
        cmd = "SUCTION_ON\n" if on else "SUCTION_OFF\n"
        if self._serial and self._serial.is_open:
            try:
                self._serial.write(cmd.encode())
            except Exception:
                pass
        self.get_logger().info(f"吸盘 {'开' if on else '关'}")

    def destroy_node(self):
        if self._serial and self._serial.is_open:
            self._serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()