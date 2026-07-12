#!/usr/bin/env python3
"""
debug_ui.py — 调试遥控器 UI（虚拟摇杆 + 触屏控制）

运行: ros2 run robocom_ui debug_ui
"""

import sys, math, threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from robocom_interfaces.msg import MotionCmd
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QGroupBox, QSlider, QGridLayout
)
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QFont, QPainter, QColor, QPen
from PySide6.QtCore import QPointF


# ================== 虚拟摇杆控件 ==================

class VirtualJoystick(QWidget):
    """触摸拖拽式虚拟摇杆。发出 (forward, yaw) 信号，范围 -1~1。"""
    valueChanged = Signal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(220, 220)
        self._fx = 0.0
        self._fy = 0.0
        self._dragging = False

    @property
    def _cx(self):
        return self.width() / 2.0

    @property
    def _cy(self):
        return self.height() / 2.0

    @property
    def _r(self):
        return min(self.width(), self.height()) / 2.0 - 15

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(p.Antialiasing)
        cx, cy, r = self._cx, self._cy, self._r

        # 外圆
        p.setBrush(QColor("#2c2c4e"))
        p.setPen(QPen(QColor("#555"), 2))
        p.drawEllipse(QPointF(cx, cy), r, r)

        # 十字准星
        p.setPen(QPen(QColor("#444"), 1))
        p.drawLine(int(cx - r), int(cy), int(cx + r), int(cy))
        p.drawLine(int(cx), int(cy - r), int(cx), int(cy + r))

        # 摇杆球
        tx = cx + self._fx * r
        ty = cy - self._fy * r  # Y 翻转：上为正
        p.setBrush(QColor("#3498db"))
        p.setPen(QPen(QColor("#5dade2"), 2))
        p.drawEllipse(QPointF(tx, ty), 22, 22)

        # 中心点
        p.setBrush(QColor("#888"))
        p.setPen(Qt.NoPen)
        p.drawEllipse(QPointF(cx, cy), 4, 4)

    def mousePressEvent(self, e):
        self._dragging = True
        self._update_pos(e.position())

    def mouseMoveEvent(self, e):
        if self._dragging:
            self._update_pos(e.position())

    def mouseReleaseEvent(self, e):
        self._dragging = False
        self._fx = 0.0
        self._fy = 0.0
        self.update()
        self.valueChanged.emit(0.0, 0.0)

    def _update_pos(self, pos):
        dx = pos.x() - self._cx
        dy = self._cy - pos.y()  # Y 翻转：向上拖动 = 正向
        r = self._r
        dist = math.hypot(dx, dy)
        if dist > r:
            dx = dx / dist * r
            dy = dy / dist * r
        self._fx = dx / r
        self._fy = dy / r
        self.update()
        self.valueChanged.emit(self._fy, self._fx)


# ================== ROS2 节点 ==================

class DebugUINode(Node):
    def __init__(self):
        super().__init__("debug_ui_node")
        self.pub_cmd = self.create_publisher(MotionCmd, "/motion_cmd", 10)
        self.pub_enable = self.create_publisher(Bool, "/enable_motion", 10)
        self.pub_estop = self.create_publisher(Bool, "/estop", 10)
        self._motion_enabled = False
        self.create_subscription(Bool, "/motion_enabled", self._cb, 10)
        self.get_logger().info("DebugUINode 已启动")

    def _cb(self, msg: Bool):
        self._motion_enabled = msg.data

    def send_motion(self, fwd=0.0, yaw=0.0, gait=1, enable=True, estop=False):
        cmd = MotionCmd()
        cmd.linear_x = max(-1.0, min(1.0, fwd))
        cmd.linear_y = 0.0
        cmd.angular_z = max(-1.0, min(1.0, yaw))
        cmd.gait_mode = gait
        cmd.enable = enable
        cmd.estop = estop
        self.pub_cmd.publish(cmd)

    def send_enable(self, on: bool):
        self.pub_enable.publish(Bool(data=on))

    def send_estop(self):
        self.pub_estop.publish(Bool(data=True))


# ================== 主界面 ==================

class DebugWindow(QMainWindow):
    def __init__(self, node: DebugUINode):
        super().__init__()
        self._node = node
        self._gait = 1
        self._joystick_fwd = 0.0
        self._joystick_yaw = 0.0

        self.setWindowTitle("ROBOCON 调试遥控器")
        self.setMinimumSize(520, 720)
        self.setStyleSheet("""
            QMainWindow { background-color: #1a1a2e; }
            QGroupBox { color: #e0e0e0; font-weight: bold; border: 1px solid #444;
                        border-radius: 8px; margin-top: 12px; padding: 12px 8px 8px; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
            QLabel { color: #e0e0e0; }
        """)

        cw = QWidget()
        self.setCentralWidget(cw)
        lo = QVBoxLayout(cw)
        lo.setSpacing(8)
        lo.setContentsMargins(12, 12, 12, 12)

        # 标题
        tl = QLabel("ROBOCON 调试遥控器 — 虚拟摇杆")
        tl.setFont(QFont("Microsoft YaHei", 14, QFont.Bold))
        tl.setAlignment(Qt.AlignCenter)
        lo.addWidget(tl)

        # ===== 摇杆区域 =====
        g_stick = QGroupBox("方向控制（触摸拖拽）")
        sg = QVBoxLayout(g_stick)
        self.joystick = VirtualJoystick()
        self.joystick.valueChanged.connect(self._on_joystick)
        sg.addWidget(self.joystick, alignment=Qt.AlignCenter)
        lo.addWidget(g_stick)

        # 数值显示
        val_row = QHBoxLayout()
        self.lbl_fwd = QLabel("前进/后退: 0.00")
        self.lbl_yaw = QLabel("转向: 0.00")
        for lb in [self.lbl_fwd, self.lbl_yaw]:
            lb.setFont(QFont("Consolas", 12))
            lb.setAlignment(Qt.AlignCenter)
            val_row.addWidget(lb)
        lo.addLayout(val_row)

        # ===== 速度档 =====
        g_spd = QGroupBox("速度档位")
        sr = QHBoxLayout(g_spd)
        self._speed_btns = {}
        for i, (label, clr, gm) in enumerate([
            ("LOW", "#e67e22", 0), ("MID", "#f39c12", 1), ("HIGH", "#e74c3c", 2)
        ]):
            btn = QPushButton(label)
            btn.setCheckable(True)
            if i == 1:
                btn.setChecked(True)
            btn.setStyleSheet(
                f"QPushButton {{ background-color: {clr}; color: white; "
                f"border-radius: 6px; padding: 10px; font-weight: bold; }}"
            )
            btn.clicked.connect(lambda checked, m=gm: self._set_gait(m))
            self._speed_btns[gm] = btn
            sr.addWidget(btn)
        lo.addWidget(g_spd)

        # ===== 模式 =====
        g_mode = QGroupBox("九宫格模式")
        mr = QHBoxLayout(g_mode)
        for label, clr, cb in [
            ("IDLE", "#555", self._mode_idle),
            ("STAND", "#27ae60", self._mode_stand),
            ("GAIT", "#2980b9", self._mode_gait),
            ("ARM", "#8e44ad", self._mode_arm),
        ]:
            btn = QPushButton(label)
            btn.setMinimumHeight(44)
            btn.setStyleSheet(
                f"QPushButton {{ background-color: {clr}; color: white; "
                f"border-radius: 6px; font-weight: bold; font-size: 11pt; }}"
            )
            btn.clicked.connect(cb)
            mr.addWidget(btn)
        lo.addWidget(g_mode)

        # ===== 机械臂微调 =====
        g_arm = QGroupBox("机械臂微调")
        al = QVBoxLayout(g_arm)
        for axis, label in [("j0", "J0"), ("j1", "J1")]:
            hr = QHBoxLayout()
            hr.addWidget(QLabel(label))
            sld = QSlider(Qt.Horizontal)
            sld.setRange(-1000, 1000)
            sld.setValue(0)
            sld.setTickPosition(QSlider.TicksBelow)
            sld.valueChanged.connect(
                lambda v, a=axis: self._arm_jog(a, v)
            )
            setattr(self, f"_sld_{axis}", sld)
            hr.addWidget(sld)
            lv = QLabel("0")
            lv.setFont(QFont("Consolas", 10))
            lv.setMinimumWidth(50)
            lv.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            setattr(self, f"_lbl_{axis}", lv)
            hr.addWidget(lv)
            al.addLayout(hr)
        lo.addWidget(g_arm)

        # ===== 控制按钮 =====
        g_ctrl = QGroupBox("系统控制")
        cr = QHBoxLayout(g_ctrl)
        for label, clr, cb in [
            ("🟢 使能", "#27ae60", lambda: self._node.send_enable(True)),
            ("🔴 失能", "#e74c3c", lambda: self._node.send_enable(False)),
            ("⛔ 急停", "#c0392b", self._node.send_estop),
        ]:
            btn = QPushButton(label)
            btn.setMinimumHeight(48)
            btn.setStyleSheet(
                f"QPushButton {{ background-color: {clr}; color: white; "
                f"border-radius: 8px; font-weight: bold; font-size: 12pt; }}"
            )
            btn.clicked.connect(cb)
            cr.addWidget(btn)
        lo.addWidget(g_ctrl)

        # ===== 状态 =====
        g_info = QGroupBox("状态")
        ir = QHBoxLayout(g_info)
        self.lbl_info = QLabel("forward=0.00  yaw=0.00  speed=MID  使能: 否")
        self.lbl_info.setFont(QFont("Consolas", 11))
        ir.addWidget(self.lbl_info)
        lo.addWidget(g_info)

        # ===== 定时发布摇杆值 (20Hz) =====
        self._pub_timer = QTimer()
        self._pub_timer.timeout.connect(self._publish_stick)
        self._pub_timer.start(50)

        # ===== 状态刷新 (5Hz) =====
        self._ui_timer = QTimer()
        self._ui_timer.timeout.connect(self._refresh_ui)
        self._ui_timer.start(200)

    # ---------- 摇杆 ----------
    def _on_joystick(self, fwd, yaw):
        self._joystick_fwd = fwd
        self._joystick_yaw = yaw

    def _publish_stick(self):
        f, y = self._joystick_fwd, self._joystick_yaw
        self._node.send_motion(f, y, self._gait, enable=(abs(f) > 0.01 or abs(y) > 0.01))

    def _refresh_ui(self):
        en = self._node._motion_enabled
        spd = {0: "LOW", 1: "MID", 2: "HIGH"}.get(self._gait, "?")
        self.lbl_fwd.setText(f"前进/后退: {self._joystick_fwd:+.2f}")
        self.lbl_yaw.setText(f"转向: {self._joystick_yaw:+.2f}")
        self.lbl_info.setText(
            f"forward={self._joystick_fwd:+.2f}  "
            f"yaw={self._joystick_yaw:+.2f}  "
            f"speed={spd}  使能: {'是' if en else '否'}"
        )

    # ---------- 模式 ----------
    def _set_gait(self, gm):
        self._gait = gm
        for g, btn in self._speed_btns.items():
            btn.setChecked(g == gm)

    def _mode_idle(self):
        self._node.send_enable(False)
        self._node.send_motion(0, 0, self._gait, enable=False)

    def _mode_stand(self):
        self._node.send_enable(True)
        self._node.send_motion(0, 0, self._gait, enable=True)

    def _mode_gait(self):
        self._node.send_enable(True)
        self._node.send_motion(0.01, 0, self._gait, enable=True)

    def _mode_arm(self):
        self._node.send_enable(True)
        self._node.send_motion(0, 0, self._gait, enable=True)

    def _arm_jog(self, axis, val):
        getattr(self, f"_lbl_{axis}").setText(f"{val}")
        self._node.send_motion(
            angular_z=val / 3000.0,
            gait=self._gait, enable=True
        )


# ================== 启动 ==================

def main(args=None):
    rclpy.init(args=args)
    node = DebugUINode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    app = QApplication(sys.argv)
    w = DebugWindow(node)
    w.show()
    rc = app.exec()
    rclpy.shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()
