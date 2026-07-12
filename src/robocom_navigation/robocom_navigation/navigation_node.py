#!/usr/bin/env python3
"""navigation_node.py - 导航、安全与任务循环"""

import math
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from robocom_interfaces.msg import MotionCmd, RobotState, BlockInfo, MathResult, MissionStatus, ArmCommand
from robocom_localization.coordinate_manager import CoordinateManager
from std_msgs.msg import String, Bool


BLOCK_OBSTACLE_RADIUS = 700.0
SPEED_LIMIT = 1.5
SPEED_ESTOP = 2.5
POSITION_TOLERANCE = 500.0


class NavigationNode(Node):
    def __init__(self):
        super().__init__("navigation_node")
        self.coord = CoordinateManager()
        self.declare_parameter("position_tolerance", POSITION_TOLERANCE)
        self._pos_tol = self.get_parameter("position_tolerance").value

        self._x = 0.0; self._y = 0.0; self._yaw = 0.0; self._velocity = 0.0
        self._enabled = False; self._mission_status = "IDLE"
        self._current_block_target = 0; self._blocks_delivered = 0
        self._blocks_data = {}; self._high_zone_id = -1; self._high_zone_known = False; self._exchange_try_count = 0
        self._detected_exchange_id = 0
        self._block_targets = list(range(8)); self._mission_phase = "nav_to_block"

        cg = MutuallyExclusiveCallbackGroup()
        self.create_subscription(RobotState, "/robot_state", self._robot_state_cb, 10)
        self.create_subscription(BlockInfo, "/block_info", self._block_info_cb, 10)
        self.create_subscription(MathResult, "/math_result", self._math_result_cb, 10)
        self.create_subscription(String, "/match_start", self._on_match_start, 10)
        self.create_subscription(Bool, "/enable_motion", self._enable_cb, 10)
        self.create_subscription(Bool, "/grasp_verified", self._grasp_verified_cb, 10)
        self.create_subscription(String, "/detected_block_type", self._detected_type_cb, 10)
        self.create_subscription(String, "/exchange_mismatch", self._exchange_mismatch_cb, 10)
        self.create_subscription(Bool, "/place_complete", self._place_complete_cb, 10)

        self.pub_cmd = self.create_publisher(MotionCmd, "/motion_cmd", 10)
        self.pub_arm = self.create_publisher(ArmCommand, "/arm_command", 10)
        self.pub_mission = self.create_publisher(MissionStatus, "/mission_status", 10)
        self.pub_estop = self.create_publisher(Bool, "/estop", 10)
        self.pub_yolo_start = self.create_publisher(String, "/yolo_start", 10)
        self.pub_yolo_stop = self.create_publisher(String, "/yolo_stop", 10)
        self.pub_vision_start = self.create_publisher(String, "/color_vision_start", 10)
        self.pub_vision_stop = self.create_publisher(String, "/color_vision_stop", 10)

        self.create_timer(0.1, self._control_loop, callback_group=cg)
        self.create_timer(1.0, self._publish_mission_status)
        self.get_logger().info("NavigationNode 已启动")

    def _robot_state_cb(self, msg):
        self._x = msg.x; self._y = msg.y; self._yaw = msg.yaw; self._velocity = msg.velocity

    def _block_info_cb(self, msg):
        self._blocks_data[msg.block_id] = msg

    def _math_result_cb(self, msg):
        if msg.success:
            self._high_zone_id = msg.high_zone_id; self._high_zone_known = True
        # 数学题完成/超时 → 解锁导航
        if self._mission_status in ("MATH_SOLVING", "IDLE"):
            self._mission_status = "NAVIGATING"
            self.get_logger().info("数学题阶段结束, 开始导航")

    def _enable_cb(self, msg):
        self._enabled = msg.data

    def _grasp_verified_cb(self, msg):
        if not msg.data or self._mission_phase != "grasp_wait_verify":
            return
        self.get_logger().info("深度验证通过，开始前往兑换站")
        self._mission_phase = "nav_to_exchange"
        self._mission_status = "NAVIGATING"
        self._exchange_try_count = 0
        # 停止 YOLO 视觉
        self.pub_yolo_stop.publish(String(data="done"))
        # 吸盘已吸住, 机械臂回到待机位
        arm = ArmCommand(); arm.state = 0; arm.command_valid = True
        self.pub_arm.publish(arm)

    def _detected_type_cb(self, msg):
        """YOLO 检测到的物块类型 → 决定去哪个兑换站"""
        try:
            cls_id = int(msg.data)
            if 0 <= cls_id <= 3:
                self._detected_exchange_id = cls_id
                self.get_logger().info(f"YOLO 检测物块类型 {cls_id}, → 兑换站 {cls_id}")
        except ValueError:
            pass

    def _exchange_mismatch_cb(self, msg):
        """兑换区颜色不匹配，尝试下一个兑换站"""
        self._exchange_try_count += 1
        if self._exchange_try_count >= 4:
            self.get_logger().error("所有兑换站都不匹配，任务出错")
            self._mission_status = "ERROR"
            return
        next_id = (self._detected_exchange_id + self._exchange_try_count) % 4
        self.get_logger().info(f"兑换区颜色不匹配，尝试下一个: station {next_id}")
        self._mission_phase = "nav_to_exchange"
        self._mission_status = "NAVIGATING"

    def _place_complete_cb(self, msg):
        if msg.data and self._mission_phase == "place":
            # 停止颜色视觉
            self.pub_vision_stop.publish(String(data="done"))
            self._blocks_delivered += 1
            if self._blocks_delivered >= 8:
                self._mission_status = "COMPLETED"
                return
            self._current_block_target = self._block_targets[self._blocks_delivered]
            self._mission_phase = "nav_to_block"
            self._mission_status = "NAVIGATING"

    def _on_match_start(self, msg):
        self._mission_status = "MATH_SOLVING"
        self._current_block_target = self._block_targets[0]

    def _control_loop(self):
        if not self._enabled:
            return
        if self._check_fence():
            return
        self._check_speed_limit()
        # 数学题阶段不导航; IDLE 不导航
        if self._mission_status in ("IDLE", "MATH_SOLVING"):
            return
        target = self._get_current_target()
        if not target:
            return
        tx, ty, target_id, phase = target
        exclude_id = target_id if phase == 'block' else None
        cmd = self._gen_joystick_cmd(tx, ty, exclude_block_id=exclude_id)
        if cmd:
            self.pub_cmd.publish(cmd)
        if math.hypot(tx - self._x, ty - self._y) < self._pos_tol:
            self._handle_arrival(tx, ty, phase)

    def _publish_stop_cmd(self):
        stop = MotionCmd()
        stop.linear_x = 0.0
        stop.linear_y = 0.0
        stop.angular_z = 0.0
        stop.gait_mode = 1
        stop.enable = True
        self.pub_cmd.publish(stop)

    def _check_fence(self) -> bool:
        x_min, x_max, y_min, y_max = self.coord.get_fence_bounds()
        if not (x_min <= self._x <= x_max and y_min <= self._y <= y_max):
            self.pub_estop.publish(Bool(data=True))
            cmd = MotionCmd(); cmd.estop = True
            self.pub_cmd.publish(cmd)
            self._mission_status = "ERROR"
            return True
        return False

    def _check_speed_limit(self):
        if self._velocity > SPEED_ESTOP:
            self.pub_estop.publish(Bool(data=True))

    def _get_current_target(self):
        if self._mission_phase == "nav_to_block":
            blocks = self.coord.get_block_coordinates()
            if self._current_block_target < len(blocks):
                return (*blocks[self._current_block_target], "block")
        elif self._mission_phase == "nav_to_exchange":
            stations = self.coord.get_exchange_coordinates()
            eid = (self._detected_exchange_id + self._exchange_try_count) % 4
            if eid < len(stations):
                return (*stations[eid], "exchange")
        return None

    def _gen_joystick_cmd(self, tx: float, ty: float, exclude_block_id=None):
        cmd = MotionCmd()
        dx = tx - self._x; dy = ty - self._y
        dist = math.hypot(dx, dy)
        if dist < 50.0:
            return None
        target_yaw = math.atan2(dy, dx)
        yaw_diff = self._normalize_angle(target_yaw - self._yaw)
        avoid = self._check_obstacles(tx, ty, exclude_block_id=exclude_block_id)
        cmd.angular_z = max(-1.0, min(1.0, yaw_diff / math.pi))
        cmd.linear_x = 0.0 if abs(yaw_diff) > 0.3 else max(-1.0, min(1.0, dist / 2000.0))
        # 8DOF: 无横移能力, linear_y 置零; 避障通过调整航向实现
        cmd.linear_y = 0.0
        cmd.gait_mode = 1; cmd.enable = True
        return cmd

    def _check_obstacles(self, tx, ty, exclude_block_id=None):
        zones = self.coord.get_block_obstacle_zones()
        path_dx = tx - self._x; path_dy = ty - self._y
        path_dist = math.hypot(path_dx, path_dy)
        if path_dist < 0.001:
            return None
        for ox, oy, radius, bid in zones:
            if exclude_block_id is not None and bid == exclude_block_id:
                continue
            t = max(0.0, min(1.0, ((ox - self._x) * path_dx + (oy - self._y) * path_dy) / (path_dist * path_dist)))
            cx = self._x + t * path_dx; cy = self._y + t * path_dy
            if math.hypot(cx - ox, cy - oy) < radius:
                perp_x = -(oy - cy); perp_y = ox - cx
                pl = math.hypot(perp_x, perp_y)
                if pl > 0.001:
                    return (perp_x / pl, perp_y / pl)
        return None

    def _handle_arrival(self, tx: float, ty: float, phase: str):
        self._publish_stop_cmd()
        if phase == "block":
            self._mission_phase = "grasp_wait_verify"; self._mission_status = "GRASPING"
            self.pub_yolo_start.publish(String(data=f"block_{self._current_block_target}"))
            arm = ArmCommand(); arm.state = 1; arm.command_valid = True
            arm.suction_on = False  # 先视觉识别, 不吸
            self.pub_arm.publish(arm)
        elif phase == "exchange":
            self._mission_phase = "place"; self._mission_status = "PLACING"
            target_exchange = (self._detected_exchange_id + self._exchange_try_count) % 4
            self.pub_vision_start.publish(String(data=f"exchange_{target_exchange}"))
            arm = ArmCommand(); arm.state = 4; arm.command_valid = True
            arm.suction_on = False  # 到站释放物块
            self.pub_arm.publish(arm)

    def _publish_mission_status(self):
        msg = MissionStatus()
        msg.status = self._mission_status
        msg.current_target = self._current_block_target
        msg.blocks_delivered = self._blocks_delivered
        msg.blocks_remaining = 8 - self._blocks_delivered
        msg.score = self._blocks_delivered * 100
        msg.high_zone_known = self._high_zone_known
        self.pub_mission.publish(msg)

    @staticmethod
    def _normalize_angle(a):
        while a > math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a


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

if __name__ == "__main__":
    main()
