#!/usr/bin/env python3
"""
localization_node.py — 全局定位节点

功能：
  - 订阅 Mid-360 LiDAR /scan 或 /odom 话题，获取机器人实时位置
  - 维护雷达坐标系（前 x+ / 左 y+ / 上 z+）
  - 以一键启动时刻为坐标原点
  - 提供块坐标、兑换站坐标的查询服务
  - 定时发布机器人状态 /robot_state

依赖：
  - Mid-360 → livox_ros2_driver / laser_scan_matcher → /odom
  - 或直接使用 cartographer / slam_toolbox 输出的 /odom /map
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from robocom_interfaces.msg import RobotState, BlockInfo
from robocom_interfaces.srv import SetCoordinate
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from .coordinate_manager import CoordinateManager


class LocalizationNode(Node):
    """基于 LiDAR 的全局定位节点"""

    def __init__(self):
        super().__init__('localization_node')

        # ---------- 坐标系管理器 ----------
        self.coord = CoordinateManager()

        # ---------- 参数 ----------
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('block_origin_x', 0.0)
        self.declare_parameter('block_origin_y', 0.0)
        self.declare_parameter('exchange_origin_x', 0.0)
        self.declare_parameter('exchange_origin_y', 0.0)
        self.declare_parameter('entrance_x', 0.0)
        self.declare_parameter('entrance_y', 0.0)

        # ---------- 当前位姿 (雷达坐标系) ----------
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._velocity = 0.0
        self._odom_received = False

        # ---------- 雷达坐标系重置 ----------
        self._origin_reset = False  # 一键启动后设为 True

        # ---------- 订阅 ----------
        odom_topic = self.get_parameter('odom_topic').value
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)

        # ---------- 发布 ----------
        self.pub_state = self.create_publisher(RobotState, '/robot_state', 10)
        self.pub_blocks = self.create_publisher(BlockInfo, '/block_info', 10)

        # ---------- 服务 ----------
        self.srv_set_coord = self.create_service(
            SetCoordinate, '/set_coordinate', self._set_coordinate_cb
        )

        # ---------- 订阅（重置指令） ----------
        self.create_subscription(String, '/match_start', self._on_match_start, 10)

        # ---------- 定时发布 ----------
        rate = self.get_parameter('publish_rate').value
        self.create_timer(1.0 / rate, self._publish_state)
        self.create_timer(1.0, self._publish_block_info)

        # ---------- 初始化坐标系参数 ----------
        self.coord.x0 = self.get_parameter('block_origin_x').value
        self.coord.y0 = self.get_parameter('block_origin_y').value
        self.coord.x1 = self.get_parameter('exchange_origin_x').value
        self.coord.y1 = self.get_parameter('exchange_origin_y').value
        self.coord.x3 = self.get_parameter('entrance_x').value
        self.coord.y3 = self.get_parameter('entrance_y').value

        self.get_logger().info(
            f'LocalizationNode 已启动。坐标原点: '
            f'块区({self.coord.x0},{self.coord.y0}) '
            f'兑换区({self.coord.x1},{self.coord.y1}) '
            f'入口({self.coord.x3},{self.coord.y3})'
        )

    # ------------------------------------------------------------------
    def _odom_cb(self, msg: Odometry):
        """里程计回调（雷达坐标系：前 x+ / 左 y+ / 上 z+）"""
        self._x = msg.pose.pose.position.x * 1000.0   # m → mm
        self._y = msg.pose.pose.position.y * 1000.0
        self._velocity = math.hypot(
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y
        )

        # 从四元数提取 yaw
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._yaw = math.atan2(siny, cosy)

        self._odom_received = True

    # ------------------------------------------------------------------
    def _on_match_start(self, msg: String):
        """一键启动时重置雷达坐标原点为当前位置"""
        if not self._odom_received:
            self.get_logger().warn('未收到里程计数据，无法重置原点')
            return

        self.coord.reset_radar_origin(self._x, self._y, self._yaw)
        self._origin_reset = True
        self.get_logger().info(
            f'✓ 雷达坐标原点已重置为 ({self._x:.1f}, {self._y:.1f}), '
            f'yaw={self._yaw:.2f}'
        )

    # ------------------------------------------------------------------
    def _publish_state(self):
        """定时发布机器人状态"""
        if not self._odom_received:
            return

        msg = RobotState()
        msg.x = self._x
        msg.y = self._y
        msg.yaw = self._yaw
        msg.velocity = self._velocity
        msg.armed = True
        msg.on_ground = True
        self.pub_state.publish(msg)

    # ------------------------------------------------------------------
    def _publish_block_info(self):
        """发布物资箱坐标（仅发布一次）"""
        if hasattr(self, '_blocks_published') and self._blocks_published:
            return

        block_types = ['food', 'tool', 'instrument', 'medicine']
        zone_colors = ['green', 'gray', 'blue', 'red']
        blocks = self.coord.get_block_coordinates()

        for bx, by, bid in blocks:
            msg = BlockInfo()
            msg.block_id = bid
            msg.block_type = block_types[bid % 4]
            msg.x = bx
            msg.y = by
            msg.zone_color = zone_colors[bid % 4]
            msg.grasped = False
            self.pub_blocks.publish(msg)

        self._blocks_published = True
        self.get_logger().info(f'已发布 {len(blocks)} 个物资箱坐标')

    # ------------------------------------------------------------------
    def _set_coordinate_cb(self, request, response):
        """设置坐标系原点（调试/标定用）"""
        if request.coordinate_name == 'block_origin':
            self.coord.x0 = request.x
            self.coord.y0 = request.y
            self._blocks_published = False
            response.success = True
            response.message = f'块区原点设为 ({request.x}, {request.y})'
        elif request.coordinate_name == 'exchange_origin':
            self.coord.x1 = request.x
            self.coord.y1 = request.y
            response.success = True
            response.message = f'兑换区原点设为 ({request.x}, {request.y})'
        elif request.coordinate_name == 'entrance_center':
            self.coord.x3 = request.x
            self.coord.y3 = request.y
            response.success = True
            response.message = f'入口中心设为 ({request.x}, {request.y})'
        else:
            response.success = False
            response.message = f'未知坐标系: {request.coordinate_name}'
        return response

    # ------------------------------------------------------------------
    def get_pose(self):
        """外部获取当前位姿"""
        return (self._x, self._y, self._yaw)


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


if __name__ == '__main__':
    main()
