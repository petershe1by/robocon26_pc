#!/usr/bin/env python3
"""
arm_control_node.py — 机械臂 5 状态机控制节点（任务 9）

五个状态：
  0 — 初始位置 (HOME)
  1 — 视觉识别姿态 (VISION_SCAN)
  2 — 视觉辅助定位姿态 (VISION_AID)
  3 — 抓取姿态 (GRASP)
  4 — 放置姿态 (PLACE)

通信：
  - 订阅 /arm_command → 执行状态切换
  - 向机械臂串口发送控制指令
  - 吸盘开关控制
"""

import time

import rclpy
from rclpy.node import Node
from robocom_interfaces.msg import ArmCommand
from std_msgs.msg import Bool

try:
    import serial
except ImportError:
    serial = None


class ArmControlNode(Node):
    """机械臂 5 状态机控制节点"""

    # 状态名称
    STATE_NAMES = ['HOME', 'VISION_SCAN', 'VISION_AID', 'GRASP', 'PLACE']

    def __init__(self):
        super().__init__('arm_control_node')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('serial_baud', 115200)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('serial_baud').value

        # ---------- 串口 ----------
        self._serial = None
        if serial is not None:
            try:
                self._serial = serial.Serial(port, baud, timeout=0.5)
                self.get_logger().info(f'机械臂串口已连接: {port}')
            except Exception as e:
                self.get_logger().warn(f'机械臂串口连接失败: {e}')

        # ---------- 状态 ----------
        self._current_state = 0    # HOME
        self._suction_on = False

        # ---------- 预定义关节位置 ----------
        # 格式: [关节1, 关节2, 关节3, ..., 关节N] 或 姿态描述
        self._state_poses = {
            0: {'name': 'HOME',          'cmd': 'HOME\n'},
            1: {'name': 'VISION_SCAN',   'cmd': 'VISION_SCAN\n'},
            2: {'name': 'VISION_AID',    'cmd': 'VISION_AID\n'},
            3: {'name': 'GRASP',         'cmd': 'GRASP\n'},
            4: {'name': 'PLACE',         'cmd': 'PLACE\n'},
        }

        # ---------- 订阅 ----------
        self.create_subscription(ArmCommand, '/arm_command', self._arm_cmd_cb, 10)

        # ---------- 发布 ----------
        self.pub_state = self.create_publisher(Bool, '/arm_at_target', 10)

        self.get_logger().info('ArmControlNode 已启动，当前状态: HOME')

    # ------------------------------------------------------------------
    def _arm_cmd_cb(self, msg: ArmCommand):
        """接收机械臂控制指令"""
        if not msg.command_valid:
            return

        state = msg.state
        if state < 0 or state > 4:
            self.get_logger().warn(f'无效状态: {state}')
            return

        # 更新吸盘状态
        if msg.suction_on != self._suction_on:
            self._set_suction(msg.suction_on)

        # 状态切换
        if state != self._current_state:
            self._transition_to(state)

    # ------------------------------------------------------------------
    def _transition_to(self, new_state: int):
        """执行状态切换"""
        old_name = self.STATE_NAMES[self._current_state]
        new_name = self.STATE_NAMES[new_state]

        self.get_logger().info(
            f'机械臂状态: {old_name} → {new_name}'
        )

        # 发送串口指令
        if self._serial and self._serial.is_open:
            cmd = self._state_poses[new_state]['cmd']
            try:
                self._serial.write(cmd.encode())
                time.sleep(0.1)
                # 等待机械臂到达目标
                response = self._serial.readline().decode().strip()
                self.get_logger().debug(f'机械臂响应: {response}')
            except Exception as e:
                self.get_logger().error(f'机械臂指令发送失败: {e}')
        else:
            # 无串口时模拟延迟
            time.sleep(0.5)

        self._current_state = new_state

        # 通知到达
        self.pub_state.publish(Bool(data=True))

    # ------------------------------------------------------------------
    def _set_suction(self, on: bool):
        """吸盘控制"""
        self._suction_on = on
        action = '开启' if on else '关闭'

        if self._serial and self._serial.is_open:
            cmd = 'SUCTION_ON\n' if on else 'SUCTION_OFF\n'
            try:
                self._serial.write(cmd.encode())
            except Exception as e:
                self.get_logger().error(f'吸盘控制失败: {e}')

        self.get_logger().info(f'吸盘 {action}')

    # ------------------------------------------------------------------
    def get_current_state(self) -> int:
        """获取当前状态"""
        return self._current_state

    # ------------------------------------------------------------------
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


if __name__ == '__main__':
    main()
