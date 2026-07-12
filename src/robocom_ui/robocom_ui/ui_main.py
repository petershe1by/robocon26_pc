#!/usr/bin/env python3
"""ui_main.py - PySide6 上位机主界面（任务 7）"""
import sys, threading, time, json, os

try:
    import cv2
    import numpy as np
except ImportError:
try:
    import pyrealsense2 as rs
except ImportError:
    rs = None

    cv2 = None
    np = None

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String, Bool
from robocom_interfaces.msg import RobotState, MathResult, MissionStatus
from robocom_interfaces.srv import StartMission
from robocom_interfaces.srv import SetCoordinate
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QGroupBox, QDoubleSpinBox, QGridLayout,
    QMessageBox, QDialog, QComboBox
)
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QFont, QPixmap, QImage


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
        self._set_coord_client = self.create_client(SetCoordinate, '/set_coordinate')
        while not self._start_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('等待 /start_mission 服务...')
        if not self._set_coord_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('等待 /set_coordinate 服务...')

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

    def set_coordinate(self, name: str, x: float, y: float) -> bool:
        """调用 /set_coordinate 服务设定坐标原点"""
        req = SetCoordinate.Request()
        req.coordinate_name = name
        req.x = x
        req.y = y
        future = self._set_coord_client.call_async(req)
        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.result() and future.result().success:
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
        self._coords_file = os.path.join(os.path.expanduser('~'), '.robocom_coords.json')
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
        self.btn_cam = QPushButton('📷 摄像头')
        self.btn_cam.setMinimumHeight(40)
        self.btn_cam.setStyleSheet("QPushButton { background-color: #8e44ad; color: white; border-radius: 6px; padding: 8px 16px; font-size: 12pt; } QPushButton:hover { background-color: #9b59b6; }")
        self.btn_cam.clicked.connect(self._open_camera_preview)
        l_start.addWidget(self.btn_cam, 1)
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

        # === 坐标标定（x0/y0, x1/y1, x3/y3）===
        g_calib = QGroupBox('场地坐标标定')
        calib_layout = QGridLayout(g_calib)
        calib_layout.setSpacing(8)

        # 表头
        calib_layout.addWidget(QLabel('坐标系'), 0, 0)
        calib_layout.addWidget(QLabel('X (mm)'), 0, 1)
        calib_layout.addWidget(QLabel('Y (mm)'), 0, 2)
        calib_layout.addWidget(QLabel(''), 0, 3)

        # x0, y0 — 物资箱原点（左下角物块中心）
        calib_layout.addWidget(QLabel('物资箱 (x₀,y₀)'), 1, 0)
        self.spin_x0 = QDoubleSpinBox(); self.spin_x0.setRange(-99999, 99999); self.spin_x0.setDecimals(1); self.spin_x0.setValue(0.0)
        self.spin_y0 = QDoubleSpinBox(); self.spin_y0.setRange(-99999, 99999); self.spin_y0.setDecimals(1); self.spin_y0.setValue(0.0)
        calib_layout.addWidget(self.spin_x0, 1, 1)
        calib_layout.addWidget(self.spin_y0, 1, 2)
        btn_apply0 = QPushButton('应用'); btn_apply0.setStyleSheet("QPushButton { background-color: #2980b9; color: white; border-radius: 4px; padding: 4px 16px; } QPushButton:hover { background-color: #3498db; }")
        btn_apply0.clicked.connect(lambda: self._on_apply_coord('block_origin', self.spin_x0, self.spin_y0))
        calib_layout.addWidget(btn_apply0, 1, 3)

        # x1, y1 — 兑换站原点（最左侧兑换站中心）
        calib_layout.addWidget(QLabel('兑换站 (x₁,y₁)'), 2, 0)
        self.spin_x1 = QDoubleSpinBox(); self.spin_x1.setRange(-99999, 99999); self.spin_x1.setDecimals(1); self.spin_x1.setValue(0.0)
        self.spin_y1 = QDoubleSpinBox(); self.spin_y1.setRange(-99999, 99999); self.spin_y1.setDecimals(1); self.spin_y1.setValue(0.0)
        calib_layout.addWidget(self.spin_x1, 2, 1)
        calib_layout.addWidget(self.spin_y1, 2, 2)
        btn_apply1 = QPushButton('应用'); btn_apply1.setStyleSheet("QPushButton { background-color: #2980b9; color: white; border-radius: 4px; padding: 4px 16px; } QPushButton:hover { background-color: #3498db; }")
        btn_apply1.clicked.connect(lambda: self._on_apply_coord('exchange_origin', self.spin_x1, self.spin_y1))
        calib_layout.addWidget(btn_apply1, 2, 3)

        # x3, y3 — 场地入口中心
        calib_layout.addWidget(QLabel('入口中心 (x₃,y₃)'), 3, 0)
        self.spin_x3 = QDoubleSpinBox(); self.spin_x3.setRange(-99999, 99999); self.spin_x3.setDecimals(1); self.spin_x3.setValue(0.0)
        self.spin_y3 = QDoubleSpinBox(); self.spin_y3.setRange(-99999, 99999); self.spin_y3.setDecimals(1); self.spin_y3.setValue(0.0)
        calib_layout.addWidget(self.spin_x3, 3, 1)
        calib_layout.addWidget(self.spin_y3, 3, 2)
        btn_apply3 = QPushButton('应用'); btn_apply3.setStyleSheet("QPushButton { background-color: #2980b9; color: white; border-radius: 4px; padding: 4px 16px; } QPushButton:hover { background-color: #3498db; }")
        btn_apply3.clicked.connect(lambda: self._on_apply_coord('entrance_center', self.spin_x3, self.spin_y3))
        calib_layout.addWidget(btn_apply3, 3, 3)

        # 加载上次保存的坐标值
        self._load_coords()

        layout.addWidget(g_calib)

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

    def _open_camera_preview(self):
        """打开摄像头预览对话框"""
        dlg = CameraPreviewDialog(self)
        dlg.exec()

    def _on_apply_coord(self, name: str, spin_x: QDoubleSpinBox, spin_y: QDoubleSpinBox):
        """应用坐标到 /set_coordinate 服务"""
        x = spin_x.value(); y = spin_y.value()
        name_labels = {
            'block_origin':     '物资箱原点',
            'exchange_origin':  '兑换站原点',
            'entrance_center':  '入口中心',
        }
        label = name_labels.get(name, name)

        def _do():
            ok = self._node.set_coordinate(name, x, y)
            if ok:
                self._save_coords()
                self.lbl_status.setText(f'✓ {label} → ({x:.1f}, {y:.1f})')
            else:
                self.lbl_status.setText(f'✗ {label} 设置失败')
        threading.Thread(target=_do, daemon=True).start()

    def _save_coords(self):
        """将当前 spinbox 数值持久化到 JSON 文件"""
        data = {
            'block_origin':    {'x': self.spin_x0.value(), 'y': self.spin_y0.value()},
            'exchange_origin': {'x': self.spin_x1.value(), 'y': self.spin_y1.value()},
            'entrance_center': {'x': self.spin_x3.value(), 'y': self.spin_y3.value()},
        }
        try:
            with open(self._coords_file, 'w') as f:
                json.dump(data, f, indent=2)
        except Exception:
            pass

    def _load_coords(self):
        """从 JSON 文件恢复上次保存的坐标值"""
        if not os.path.exists(self._coords_file):
            return
        try:
            with open(self._coords_file) as f:
                data = json.load(f)
            if 'block_origin' in data:
                self.spin_x0.setValue(data['block_origin'].get('x', 0.0))
                self.spin_y0.setValue(data['block_origin'].get('y', 0.0))
            if 'exchange_origin' in data:
                self.spin_x1.setValue(data['exchange_origin'].get('x', 0.0))
                self.spin_y1.setValue(data['exchange_origin'].get('y', 0.0))
            if 'entrance_center' in data:
                self.spin_x3.setValue(data['entrance_center'].get('x', 0.0))
                self.spin_y3.setValue(data['entrance_center'].get('y', 0.0))
        except Exception:
            pass

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


class CameraPreviewDialog(QDialog):
    """摄像头实时预览对话框（支持 D435 pyrealsense2 + USB cv2）"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle('摄像头实时预览')
        self.setMinimumSize(720, 560)
        self.setStyleSheet("""
            QDialog { background-color: #1a1a2e; }
            QLabel { color: #e0e0e0; }
            QComboBox { color: #e0e0e0; background-color: #2c2c4e; border: 1px solid #555; border-radius: 4px; padding: 4px 8px; }
        """)

        layout = QVBoxLayout(self)

        top_bar = QHBoxLayout()
        top_bar.addWidget(QLabel('选择摄像头:'))
        self.cam_combo = QComboBox()
        self.cam_combo.addItems(['D435 (RealSense) [默认]', 'USB 0', 'USB 1', 'USB 2', 'USB 3'])
        self.cam_combo.currentIndexChanged.connect(self._switch_camera)
        top_bar.addWidget(self.cam_combo)
        top_bar.addStretch()
        layout.addLayout(top_bar)

        self.video_label = QLabel()
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setMinimumSize(640, 480)
        self.video_label.setStyleSheet("background-color: #000; border: 1px solid #555; border-radius: 4px;")
        self.video_label.setText('正在打开摄像头...')
        layout.addWidget(self.video_label)

        self.lbl_cam_status = QLabel('状态: 等待打开')
        self.lbl_cam_status.setFont(QFont('Consolas', 10))
        layout.addWidget(self.lbl_cam_status)

        self._cap = None
        self._pipeline = None
        self._using_rs = False

        self._open_d435()
        self._timer = QTimer()
        self._timer.timeout.connect(self._update_frame)
        self._timer.start(33)

    def _close_camera(self):
        if self._pipeline:
            try: self._pipeline.stop()
            except: pass
            self._pipeline = None
        if self._cap:
            self._cap.release()
            self._cap = None
        self._using_rs = False

    def _open_d435(self):
        self._close_camera()
        if rs is None:
            self.video_label.setText('pyrealsense2 未安装\n无法打开 D435')
            self.lbl_cam_status.setText('状态: pyrealsense2 不可用')
            return
        try:
            self._pipeline = rs.pipeline()
            cfg = rs.config()
            cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self._pipeline.start(cfg)
            self._using_rs = True
            self.lbl_cam_status.setText('状态: D435 (RealSense) 640x480')
        except Exception as e:
            self._close_camera()
            self.video_label.setText(f'D435 打开失败: {e}')
            self.lbl_cam_status.setText('状态: D435 连接失败')

    def _open_usb(self, cam_id):
        self._close_camera()
        if cv2 is None:
            self.video_label.setText('OpenCV 未安装')
            return
        try:
            self._cap = cv2.VideoCapture(cam_id, cv2.CAP_V4L2)
            if not self._cap.isOpened():
                self._cap = cv2.VideoCapture(cam_id)
            if self._cap.isOpened():
                self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                self.lbl_cam_status.setText(f'状态: USB {cam_id} 640x480')
            else:
                self.video_label.setText(f'无法打开 USB {cam_id}')
        except Exception as e:
            self.video_label.setText(f'摄像头错误: {e}')

    def _switch_camera(self, index):
        if index == 0:
            self._open_d435()
        else:
            self._open_usb(index - 1)

    def _update_frame(self):
        try:
            if self._using_rs and self._pipeline:
                frames = self._pipeline.wait_for_frames(timeout_ms=5000)
                c = frames.get_color_frame()
                if not c: return
                frame = np.asanyarray(c.get_data())
            elif self._cap and self._cap.isOpened():
                ret, frame = self._cap.read()
                if not ret or frame is None: return
            else: return
            h, w = frame.shape[:2]
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            qi = QImage(rgb.data, w, rgb.strides[0], QImage.Format_RGB888)
            s = qi.scaled(self.video_label.width(), self.video_label.height(),
                          Qt.KeepAspectRatio, Qt.SmoothTransformation)
            self.video_label.setPixmap(QPixmap.fromImage(s))
        except Exception:
            pass

    def closeEvent(self, event):
        self._timer.stop()
        self._close_camera()
        super().closeEvent(event)

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
