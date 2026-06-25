#!/usr/bin/env python3
"""ui_main.py - PySide6 上位机主界面（任务 7）"""
import sys, threading, time
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String, Bool
from robocom_interfaces.msg import RobotState, MathResult, MissionStatus
from robocom_interfaces.srv import StartMission
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QGroupBox
)
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QFont


class UINode(Node):
    """UI ROS2 通信节点"""
    def __init__(self):
        super().__init__('ui_node')
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.robot_vel = 0.0
        self.math_expression = ''
        self.math_result = 0
        self.math_success = False
        self.high_zone = -1
        self.mission_status = 'IDLE'
        self.blocks_delivered = 0
        self.blocks_remaining = 8
        self.match_time = 0.0
        self.match_started = False
        self.create_subscription(RobotState, '/robot_state', self._robot_cb, 10)
        self.create_subscription(MathResult, '/math_result', self._math_cb, 10)
        self.create_subscription(MissionStatus, '/mission_status', self._mission_cb, 10)
        self._start_client = self.create_client(StartMission, '/start_mission')
        while not self._start_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('等待 /start_mission 服务...')

    def _robot_cb(self, msg):
        self.robot_x = msg.x
        self.robot_y = msg.y
        self.robot_yaw = msg.yaw
        self.robot_vel = msg.velocity
        self.match_time = msg.mission_time.sec + msg.mission_time.nanosec * 1e-9

    def _math_cb(self, msg):
        self.math_expression = msg.expression
        self.math_result = msg.result
        self.math_success = msg.success
        self.high_zone = msg.high_zone_id

    def _mission_cb(self, msg):
        self.mission_status = msg.status
        self.blocks_delivered = msg.blocks_delivered
        self.blocks_remaining = msg.blocks_remaining

    def start_mission(self):
        req = StartMission.Request()
        future = self._start_client.call_async(req)
        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.result() and future.result().success:
                self.match_started = True
                return True
        except Exception:
            pass
        return False


class MainWindow(QMainWindow):
    """ROBOCON 上位机主界面"""
    update_signal = Signal()

    def __init__(self, ui_node):
        super().__init__()
        self._node = ui_node
        self.setWindowTitle('ROBOCON 仿生足式机器人 - 任务赛上位机')
        self.setMinimumSize(1024, 768)
        self.setStyleSheet("""
            QMainWindow { background-color: #1a1a2e; }
            QGroupBox { color: #e0e0e0; font-weight: bold; border: 1px solid #333; border-radius: 6px; margin-top: 10px; padding: 12px 8px 8px 8px; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
            QLabel { color: #e0e0e0; }
        """)
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setSpacing(12)
        layout.setContentsMargins(16, 16, 16, 16)

        # 标题
        title = QLabel('ROBOCON 仿生足式机器人挑战赛 - 任务赛上位机')
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont('Microsoft YaHei', 18, QFont.Bold))
        layout.addWidget(title)

        # 一键启动
        g_start = QGroupBox('比赛控制')
        l_start = QHBoxLayout(g_start)
        self.btn_start = QPushButton('▶ 一键启动')
        self.btn_start.setMinimumHeight(60)
        self.btn_start.setFont(QFont('Microsoft YaHei', 14, QFont.Bold))
        self.btn_start.setStyleSheet("QPushButton { background-color: #27ae60; color: white; border-radius: 8px; padding: 12px 32px; } QPushButton:hover { background-color: #2ecc71; } QPushButton:disabled { background-color: #555; color: #888; }")
        self.btn_start.clicked.connect(self._on_start)
        l_start.addWidget(self.btn_start, 3)
        self.lbl_status = QLabel('状态: 待机中')
        self.lbl_status.setFont(QFont('Microsoft YaHei', 12))
        l_start.addWidget(self.lbl_status, 2)
        self.lbl_timer = QLabel('⏱ 00:00')
        self.lbl_timer.setFont(QFont('Consolas', 20, QFont.Bold))
        l_start.addWidget(self.lbl_timer, 1)
        layout.addWidget(g_start)

        # 信息面板
        info = QHBoxLayout()
        g_coord = QGroupBox('雷达坐标')
        l_coord = QVBoxLayout(g_coord)
        self.lbl_coord = QLabel('X: 0.00 mm\nY: 0.00 mm\nYaw: 0.00 rad')
        self.lbl_coord.setFont(QFont('Consolas', 14))
        l_coord.addWidget(self.lbl_coord)
        self.lbl_vel = QLabel('速度: 0.00 m/s')
        self.lbl_vel.setFont(QFont('Consolas', 12))
        l_coord.addWidget(self.lbl_vel)
        info.addWidget(g_coord, 1)

        g_math = QGroupBox('数学题识别')
        l_math = QVBoxLayout(g_math)
        self.lbl_math = QLabel('等待比赛开始...')
        self.lbl_math.setFont(QFont('Consolas', 14))
        self.lbl_math.setWordWrap(True)
        l_math.addWidget(self.lbl_math)
        self.lbl_hz = QLabel('高分区: -')
        self.lbl_hz.setFont(QFont('Consolas', 12))
        l_math.addWidget(self.lbl_hz)
        info.addWidget(g_math, 1)

        g_prog = QGroupBox('任务进度')
        l_prog = QVBoxLayout(g_prog)
        self.lbl_mission = QLabel('已送达: 0 / 8')
        self.lbl_mission.setFont(QFont('Consolas', 14))
        l_prog.addWidget(self.lbl_mission)
        self.lbl_phase = QLabel('阶段: IDLE')
        self.lbl_phase.setFont(QFont('Consolas', 12))
        l_prog.addWidget(self.lbl_phase)
        info.addWidget(g_prog, 1)
        layout.addLayout(info)

        # 进程监控
        g_log = QGroupBox('运行进程')
        l_log = QVBoxLayout(g_log)
        self.lbl_proc = QLabel(
            '● task_scheduler_node\n○ math_solver_node\n○ localization_node\n'
            '○ navigation_node\n○ motion_control_node\n○ yolo_block_detector\n'
            '○ color_mask_detector\n○ arm_control_node\n○ depth_helper'
        )
        self.lbl_proc.setFont(QFont('Consolas', 10))
        l_log.addWidget(self.lbl_proc)
        layout.addWidget(g_log)

        self._timer = QTimer()
        self._timer.timeout.connect(self._update)
        self._timer.start(100)

    def _on_start(self):
        self.btn_start.setEnabled(False)
        self.btn_start.setText('启动中...')

        def _do():
            ok = self._node.start_mission()
            if ok:
                self.lbl_status.setText('状态: 运行中')
            else:
                self.lbl_status.setText('状态: 启动失败')
                self.btn_start.setEnabled(True)
                self.btn_start.setText('▶ 一键启动')
        threading.Thread(target=_do, daemon=True).start()

    def _update(self):
        n = self._node
        self.lbl_coord.setText(f'X:   {n.robot_x:.1f} mm\nY:   {n.robot_y:.1f} mm\nYaw: {n.robot_yaw:.2f} rad')
        self.lbl_vel.setText(f'速度: {n.robot_vel:.2f} m/s')
        if n.math_success:
            self.lbl_math.setText(f'算式: {n.math_expression}\n结果: {n.math_result}')
            self.lbl_hz.setText(f'高分区: 编号 {n.high_zone}')
        elif n.match_started:
            self.lbl_math.setText('正在识别数学题...')
        self.lbl_mission.setText(f'已送达: {n.blocks_delivered} / 8\n剩余: {n.blocks_remaining}')
        self.lbl_phase.setText(f'阶段: {n.mission_status}')
        smap = {'IDLE':'待机','MATH_SOLVING':'解数学题','NAVIGATING':'导航中','GRASPING':'抓取中','PLACING':'放置中','COMPLETED':'完成','ERROR':'错误'}
        if n.mission_status in smap:
            self.lbl_status.setText(f'状态: {smap[n.mission_status]}')
        if n.match_started:
            e = int(n.match_time)
            self.lbl_timer.setText(f'⏱ {e//60:02d}:{e%60:02d}')


def main(args=None):
    rclpy.init(args=args)
    ui_node = UINode()
    executor = SingleThreadedExecutor()
    executor.add_node(ui_node)
    threading.Thread(target=executor.spin, daemon=True).start()
    app = QApplication(sys.argv)
    w = MainWindow(ui_node)
    w.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
