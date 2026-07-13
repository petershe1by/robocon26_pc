#!/usr/bin/env python3
"""localization_node.py - 全局定位节点"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from robocom_interfaces.msg import RobotState, BlockInfo
from robocom_interfaces.srv import SetCoordinate
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from robocom_interfaces.msg import MotionCmd
from builtin_interfaces.msg import Duration as RosDuration

from .coordinate_manager import CoordinateManager


class LocalizationNode(Node):
    def __init__(self):
        super().__init__("localization_node")
        self.coord = CoordinateManager()

        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("block_origin_x", 0.0)
        self.declare_parameter("block_origin_y", 0.0)
        self.declare_parameter("exchange_origin_x", 0.0)
        self.declare_parameter("exchange_origin_y", 0.0)
        self.declare_parameter("entrance_x", 0.0)
        self.declare_parameter("entrance_y", 0.0)

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._velocity = 0.0
        self._odom_received = False
        self._match_start_time = 0.0
        self._no_odom_warn_count = 0
        self._last_motion_time = time.time()

        odom_topic = self.get_parameter("odom_topic").value
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.create_subscription(MotionCmd, "/motion_cmd", self._motion_cmd_cb, 10)

        self.pub_state = self.create_publisher(RobotState, "/robot_state", 10)
        self.pub_blocks = self.create_publisher(BlockInfo, "/block_info", 10)

        self.create_service(SetCoordinate, "/set_coordinate", self._set_coordinate_cb)
        self.create_subscription(String, "/match_start", self._on_match_start, 10)

        rate = self.get_parameter("publish_rate").value
        self.create_timer(1.0 / rate, self._publish_state)
        self.create_timer(1.0, self._publish_block_info)

        self.coord.x0 = self.get_parameter("block_origin_x").value
        self.coord.y0 = self.get_parameter("block_origin_y").value
        self.coord.x1 = self.get_parameter("exchange_origin_x").value
        self.coord.y1 = self.get_parameter("exchange_origin_y").value
        self.coord.x3 = self.get_parameter("entrance_x").value
        self.coord.y3 = self.get_parameter("entrance_y").value

        self.get_logger().info(f"LocalizationNode 已启动")

    def _odom_cb(self, msg: Odometry):
        self._x = msg.pose.pose.position.x * 1000.0
        self._y = msg.pose.pose.position.y * 1000.0
        self._velocity = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._yaw = math.atan2(siny, cosy)
        self._odom_received = True

    def _motion_cmd_cb(self, msg: MotionCmd):
        """无雷达时的简易航迹推算：从运动指令积分出坐标变化"""
        if self._odom_received:
            return  # 有雷达时不用推算
        dt = time.time() - self._last_motion_time
        self._last_motion_time = time.time()
        if dt <= 0 or dt > 0.5:
            return
        MAX_SPEED = 0.5    # m/s (全摇杆速度)
        MAX_YAW = 2.0       # rad/s (全摇杆转向)
        speed = msg.linear_x * MAX_SPEED
        yaw_rate = msg.angular_z * MAX_YAW
        self._yaw += yaw_rate * dt
        self._x += speed * math.cos(self._yaw) * dt * 1000.0
        self._y += speed * math.sin(self._yaw) * dt * 1000.0
        self._velocity = abs(speed)

    def _on_match_start(self, msg: String):
        if self._odom_received:
            self.coord.reset_radar_origin(self._x, self._y, self._yaw)
            self.get_logger().info(f"雷达原点已重置, yaw={self._yaw:.2f}")
        else:
            self.coord.reset_radar_origin(0.0, 0.0, 0.0)
            self.get_logger().warn("未收到里程计，原点设为 (0,0,0) — 开发模式")
        self._match_start_time = time.time()

    def _publish_state(self):
        if not self._odom_received:
            self._no_odom_warn_count += 1
            if self._no_odom_warn_count % 250 == 0:
                self.get_logger().warn(
                    "等待里程计... 确认雷达驱动已启动并发布 /odom"
                )
        msg = RobotState()
        msg.x = self._x
        msg.y = self._y
        msg.yaw = self._yaw
        msg.velocity = self._velocity
        msg.armed = True
        msg.on_ground = True
        if self._match_start_time > 0:
            elapsed = time.time() - self._match_start_time
            msg.mission_time = RosDuration(sec=int(elapsed), nanosec=int((elapsed % 1) * 1e9))
        self.pub_state.publish(msg)

    def _publish_block_info(self):
        if hasattr(self, "_blocks_published") and self._blocks_published:
            return
        block_types = ["food", "tool", "instrument", "medicine"]
        zone_colors = ["green", "gray", "blue", "red"]
        for bx, by, bid in self.coord.get_block_coordinates():
            msg = BlockInfo()
            msg.block_id = bid
            msg.block_type = block_types[bid % 4]
            msg.x = bx
            msg.y = by
            msg.zone_color = zone_colors[bid % 4]
            msg.grasped = False
            self.pub_blocks.publish(msg)
        self._blocks_published = True
        self.get_logger().info(f"已发布物资箱坐标")

    def _set_coordinate_cb(self, request, response):
        name_map = {
            "block_origin": ("x0", "y0"),
            "exchange_origin": ("x1", "y1"),
            "entrance_center": ("x3", "y3"),
        }
        if request.coordinate_name in name_map:
            setattr(self.coord, name_map[request.coordinate_name][0], request.x)
            setattr(self.coord, name_map[request.coordinate_name][1], request.y)
            if request.coordinate_name == "block_origin":
                self._blocks_published = False
            response.success = True
            response.message = f"{request.coordinate_name} 设为 ({request.x}, {request.y})"
        else:
            response.success = False
            response.message = f"未知坐标系: {request.coordinate_name}"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
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
