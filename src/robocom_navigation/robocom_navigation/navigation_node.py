#!/usr/bin/env python3
\"\"\"
navigation_node.py - 导航、安全与任务循环主节点

核心职责（任务 4、5、6）：
  1. 电子围栏 - 超出矩形边界则急停
  2. 避障 - 物资箱周围 700mm 不可到达圆
  3. 限速 - 雷达融合速度 <= 1.5 m/s, > 2.5 m/s 失能
  4. 任务循环 - 目标点导航 -> YOLO 视觉 -> 吸取 -> 兑换 -> 循环
  5. 安全看门狗 - 5s 指令不变则停止

通信接口：
  - 订阅：/robot_state, /block_info, /mission_status
  - 发布：/motion_cmd（纯摇杆值 -1.0~1.0）, /arm_command, /enable_motion
  - 服务：/set_coordinate

注意：MotionCmd 的 linear_x/linear_y/angular_z 都是纯摇杆信号（-1.0~1.0），
      狗的移动位移完全由下位机 STM32 根据摇杆值自行控制。
\"\"\"

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from robocom_interfaces.msg import (
    MotionCmd, RobotState, BlockInfo, MathResult,
    MissionStatus, ArmCommand
)
from robocom_interfaces.srv import SetCoordinate
from robocom_localization.coordinate_manager import CoordinateManager
from std_msgs.msg import String, Bool


# 常量
BLOCK_OBSTACLE_RADIUS = 700.0      # mm
SPEED_LIMIT = 1.5                   # m/s
SPEED_ESTOP = 2.5                   # m/s
POSITION_TOLERANCE = 500.0          # mm（到达目标点容差）
EXCHANGE_TRIGGER_X_OFFSET = 3500.0  # mm
JOYSTICK_MAX = 1.0                  # 最大摇杆值


class NavigationNode(Node):
    \"\"\"导航、安全与任务循环主节点\"\"\"

    def __init__(self):
        super().__init__('navigation_node')

        self.coord = CoordinateManager()

        # ---------- 参数 ----------
        self.declare_parameter('position_tolerance', POSITION_TOLERANCE)
        self._pos_tol = self.get_parameter('position_tolerance').value

        # ---------- 状态 ----------
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._velocity = 0.0
        self._enabled = True
        self._exchange_x_trigger = 0.0

        # 任务状态
        self._mission_status = 'IDLE'
        self._current_block_target = 0
        self._blocks_delivered = 0
        self._blocks_data = {}
        self._high_zone_id = -1
        self._high_zone_known = False

        # 目标队列
        self._block_targets = list(range(8))
        self._mission_phase = 'nav_to_block'

        # ---------- 回调组 ----------
        self._cb_group_mutex = MutuallyExclusiveCallbackGroup()
        self._cb_group_reent = ReentrantCallbackGroup()

        # ---------- 订阅 ----------
        self.create_subscription(RobotState, '/robot_state', self._robot_state_cb, 10)
        self.create_subscription(BlockInfo, '/block_info', self._block_info_cb, 10)
        self.create_subscription(MathResult, '/math_result', self._math_result_cb, 10)
        self.create_subscription(String, '/match_start', self._on_match_start, 10)
        self.create_subscription(Bool, '/enable_motion', self._enable_cb, 10)
        self.create_subscription(Bool, '/grasp_complete', self._grasp_complete_cb, 10)
        self.create_subscription(Bool, '/place_complete', self._place_complete_cb, 10)

        # ---------- 发布 ----------
        self.pub_cmd = self.create_publisher(MotionCmd, '/motion_cmd', 10)
        self.pub_arm = self.create_publisher(ArmCommand, '/arm_command', 10)
        self.pub_mission = self.create_publisher(MissionStatus, '/mission_status', 10)
        self.pub_estop = self.create_publisher(Bool, '/estop', 10)
        self.pub_yolo_start = self.create_publisher(String, '/yolo_start', 10)
        self.pub_yolo_stop = self.create_publisher(String, '/yolo_stop', 10)
        self.pub_vision_start = self.create_publisher(String, '/color_vision_start', 10)

        # ---------- 定时器 ----------
        self.create_timer(0.1, self._control_loop, callback_group=self._cb_group_mutex)
        self.create_timer(1.0, self._publish_mission_status)

        self.get_logger().info('NavigationNode 已启动')

    # ------------------------------------------------------------------
    def _robot_state_cb(self, msg: RobotState):
        self._x = msg.x
        self._y = msg.y
        self._yaw = msg.yaw
        self._velocity = msg.velocity

    def _block_info_cb(self, msg: BlockInfo):
        self._blocks_data[msg.block_id] = msg

    def _math_result_cb(self, msg: MathResult):
        if msg.success:
            self._high_zone_id = msg.high_zone_id
            self._high_zone_known = True
            self.get_logger().info(f'接收高分区编号: {self._high_zone_id}')

    def _enable_cb(self, msg: Bool):
        self._enabled = msg.data

    def _grasp_complete_cb(self, msg: Bool):
        if msg.data and self._mission_phase == 'grasp':
            self._mission_phase = 'nav_to_exchange'
            self.get_logger().info(f'物资箱 {self._current_block_target} 吸取完成，前往兑换站')

    def _place_complete_cb(self, msg: Bool):
        if msg.data and self._mission_phase == 'place':
            self._blocks_delivered += 1
            if self._blocks_delivered >= 8:
                self._mission_status = 'COMPLETED'
                self.get_logger().info('=== 所有物资箱已送达！===')
                return
            self._current_block_target = self._block_targets[self._blocks_delivered]
            self._mission_phase = 'nav_to_block'
            self.get_logger().info(f'前往第 {self._current_block_target} 号物资箱')

    # ------------------------------------------------------------------
    def _on_match_start(self, msg: String):
        self._mission_status = 'MATH_SOLVING'
        self._block_targets = list(range(8))
        self._current_block_target = self._block_targets[0]
        self._exchange_x_trigger = self.coord.x3 + EXCHANGE_TRIGGER_X_OFFSET
        self.get_logger().info('=== 任务循环启动 ===')

    # ------------------------------------------------------------------
    def _control_loop(self):
        \"\"\"10 Hz 控制循环：安全 + 导航 + 任务流转\"\"\"
        if not self._enabled:
            return

        if self._check_fence():
            return

        self._check_speed_limit()

        if self._mission_status in ('IDLE', 'MATH_SOLVING'):
            return

        target = self._get_current_target()
        if target is None:
            return

        tx, ty, phase_info = target

        # 路径规划输出纯摇杆值（-1.0~1.0）
        cmd = self._gen_joystick_cmd(tx, ty)
        if cmd:
            self.pub_cmd.publish(cmd)

        # 到达判断
        dist = math.hypot(tx - self._x, ty - self._y)
        if dist < self._pos_tol:
            self._handle_arrival(target)

    # ------------------------------------------------------------------
    def _check_fence(self) -> bool:
        x_min, x_max, y_min, y_max = self.coord.get_fence_bounds()
        if not (x_min <= self._x <= x_max and y_min <= self._y <= y_max):
            self.get_logger().error(
                f'!!! 电子围栏触发！({self._x:.0f}, {self._y:.0f}) '
                f'超出 [{x_min:.0f}, {x_max:.0f}]x[{y_min:.0f}, {y_max:.0f}]'
            )
            estop = Bool(data=True)
            self.pub_estop.publish(estop)
            cmd = MotionCmd()
            cmd.estop = True
            self.pub_cmd.publish(cmd)
            self._mission_status = 'ERROR'
            return True
        return False

    def _check_speed_limit(self):
        if self._velocity > SPEED_ESTOP:
            self.get_logger().error(
                f'!!! 超速！当前 {self._velocity:.2f} m/s > {SPEED_ESTOP} m/s，失能!'
            )
            estop = Bool(data=True)
            self.pub_estop.publish(estop)
        elif self._velocity > SPEED_LIMIT:
            self.get_logger().warn(
                f'速度过高 {self._velocity:.2f} m/s > {SPEED_LIMIT} m/s，限速中'
            )

    # ------------------------------------------------------------------
    def _get_current_target(self) -> tuple | None:
        if self._mission_phase == 'nav_to_block':
            blocks = self.coord.get_block_coordinates()
            if self._current_block_target < len(blocks):
                bx, by, _ = blocks[self._current_block_target]
                return (bx, by, 'block')
        elif self._mission_phase == 'nav_to_exchange':
            stations = self.coord.get_exchange_coordinates()
            exchange_id = self._current_block_target % 4
            if exchange_id < len(stations):
                ex, ey, _ = stations[exchange_id]
                return (ex, ey, 'exchange')
        return None

    # ------------------------------------------------------------------
    def _gen_joystick_cmd(self, tx: float, ty: float) -> MotionCmd | None:
        \"\"\"
        生成纯摇杆指令（-1.0 ~ 1.0），不涉及任何速度/位移计算。
        下位机 STM32 根据摇杆值自行决定步态和行走速度。

        逻辑：
          - 计算目标方向与当前朝向的夹角 → angular_z（转向摇杆值）
          - 距离远则推满前进摇杆，距离近则松摇杆
          - 避障时叠加一个横向摇杆分量
        \"\"\"
        cmd = MotionCmd()
        dx = tx - self._x
        dy = ty - self._y
        dist = math.hypot(dx, dy)

        if dist < 50.0:
            return None  # 已到达，不发送运动指令

        # 期望朝向（弧度）
        target_yaw = math.atan2(dy, dx)
        yaw_diff = self._normalize_angle(target_yaw - self._yaw)

        # 避障检测
        avoidance = self._check_obstacles(tx, ty)

        # --- 转向摇杆值（angular_z = -1.0 ~ 1.0） ---
        # yaw_diff 范围 -pi ~ pi, 映射到 -1.0 ~ 1.0
        cmd.angular_z = max(-1.0, min(1.0, yaw_diff / math.pi))

        # --- 前进摇杆值（linear_x = -1.0 ~ 1.0） ---
        # 远距离推满杆，接近目标逐渐回中
        if abs(yaw_diff) > 0.3:
            # 偏角太大时先转向不走动
            cmd.linear_x = 0.0
        else:
            cmd.linear_x = max(-1.0, min(1.0, dist / 2000.0))

        # --- 横向摇杆值（避障偏移） ---
        cmd.linear_y = 0.0
        if avoidance is not None:
            ax, ay = avoidance
            # 避障方向叠加到侧移摇杆
            cmd.linear_y = max(-1.0, min(1.0, ax))

        cmd.gait_mode = 1  # trot
        cmd.enable = True
        cmd.estop = False

        return cmd

    # ------------------------------------------------------------------
    def _check_obstacles(self, tx: float, ty: float) -> tuple | None:
        \"\"\"检查路径上的障碍物，返回避障偏移方向\"\"\"
        zones = self.coord.get_block_obstacle_zones()
        path_dx = tx - self._x
        path_dy = ty - self._y
        path_dist = math.hypot(path_dx, path_dy)

        if path_dist < 0.001:
            return None

        for ox, oy, radius, bid in zones:
            dx = ox - self._x
            dy = oy - self._y
            t = (dx * path_dx + dy * path_dy) / (path_dist * path_dist)
            t = max(0.0, min(1.0, t))

            closest_x = self._x + t * path_dx
            closest_y = self._y + t * path_dy
            closest_dist = math.hypot(closest_x - ox, closest_y - oy)

            if closest_dist < radius:
                perp_x = -(oy - closest_y)
                perp_y = ox - closest_x
                perp_len = math.hypot(perp_x, perp_y)
                if perp_len > 0.001:
                    return (perp_x / perp_len, perp_y / perp_len)
        return None

    # ------------------------------------------------------------------
    def _handle_arrival(self, target: tuple):
        tx, ty, phase = target

        if phase == 'block':
            self._mission_phase = 'grasp'
            self._mission_status = 'GRASPING'

            msg = String()
            msg.data = f'block_{self._current_block_target}'
            self.pub_yolo_start.publish(msg)
            self.get_logger().info(
                f'到达目标物资箱 {self._current_block_target}，启动 YOLO 视觉'
            )

            arm_cmd = ArmCommand()
            arm_cmd.state = 1
            arm_cmd.command_valid = True
            self.pub_arm.publish(arm_cmd)

        elif phase == 'exchange':
            self._mission_phase = 'place'
            self._mission_status = 'PLACING'

            msg = String()
            msg.data = f'exchange_{self._current_block_target % 4}'
            self.pub_vision_start.publish(msg)
            self.get_logger().info(
                f'到达兑换站 {self._current_block_target % 4}，启动颜色识别'
            )

            arm_cmd = ArmCommand()
            arm_cmd.state = 4
            arm_cmd.command_valid = True
            self.pub_arm.publish(arm_cmd)

    # ------------------------------------------------------------------
    def _publish_mission_status(self):
        msg = MissionStatus()
        msg.status = self._mission_status
        msg.current_target = self._current_block_target
        msg.blocks_delivered = self._blocks_delivered
        msg.blocks_remaining = 8 - self._blocks_delivered
        msg.score = self._blocks_delivered * 100
        msg.high_zone_known = self._high_zone_known
        self.pub_mission.publish(msg)

    # ------------------------------------------------------------------
    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
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