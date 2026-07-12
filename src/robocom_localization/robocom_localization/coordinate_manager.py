#!/usr/bin/env python3
"""
coordinate_manager.py — 坐标系管理器

管理三类坐标系原点（供全局访问）：
  1. block_origin (x0, y0)  — 左下物资箱中心
  2. exchange_origin (x1, y1) — 最左侧兑换站中心
  3. entrance_center (x3, y3) — 场地入口中心
"""

from typing import Tuple


class CoordinateManager:
    """单例式坐标系管理器"""

    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return

        # ---- 原点坐标 (mm) ----
        self.x0: float = 0.0   # 左下物资箱中心
        self.y0: float = 0.0
        self.x1: float = 0.0   # 最左侧兑换站中心
        self.y1: float = 0.0
        self.x3: float = 0.0   # 场地入口中心
        self.y3: float = 0.0

        # ---- 雷达原点（一键启动时重置） ----
        self.radar_origin_x: float = 0.0
        self.radar_origin_y: float = 0.0
        self.radar_origin_yaw: float = 0.0
        self.radar_initialized: bool = False

        # ---- 物块颜色赋值 (红/蓝/灰/绿) ----
        self._block_colors = ['red', 'blue', 'green', 'grey',
                              'red', 'blue', 'green', 'grey']

        self._initialized = True

    def set_block_colors(self, colors: list):
        """设置 8 个物块的盘面颜色"""
        if len(colors) >= 8:
            self._block_colors = list(colors[:8])

    def get_block_color(self, block_id: int) -> str:
        """获取指定物块的盘面颜色"""
        if 0 <= block_id < 8:
            return self._block_colors[block_id]
        return 'red'

    def get_block_colors(self) -> list:
        """获取全部 8 个物块颜色"""
        return list(self._block_colors)

    def get_exchange_id_for_block(self, block_id: int) -> int:
        """根据物块盘面颜色返回目标兑换站 ID"""
        mapping = {'blue': 0, 'green': 1, 'grey': 2, 'red': 3}
        return mapping.get(self.get_block_color(block_id), 0)

    # ------------------------------------------------------------------
    # 物资箱坐标（8个，相对 x0/y0）
    # ------------------------------------------------------------------
    # 物资箱坐标（8个，相对 x0/y0）
    # ------------------------------------------------------------------
    def get_block_coordinates(self) -> list:
        """返回 8 个物资箱的 (x, y, id) 列表，单位 mm"""
        blocks = [
            (self.x0 + 850,   self.y0 - 0,     0),
            (self.x0 + 850,   self.y0 - 850,   1),
            (self.x0 + 850,   self.y0 - 1700,  2),
            (self.x0 + 850,   self.y0 - 2550,  3),
            (self.x0 + 0,     self.y0 - 0,     4),
            (self.x0 + 0,     self.y0 - 850,   5),
            (self.x0 + 0,     self.y0 - 1700,  6),
            (self.x0 + 0,     self.y0 - 2550,  7),
        ]
        return blocks

    # ------------------------------------------------------------------
    # 兑换站坐标（4个，相对 x1/y1）
    # ------------------------------------------------------------------
    def get_exchange_coordinates(self) -> list:
        """返回 4 个兑换站的 (x, y, id) 列表，单位 mm"""
        stations = [
            (self.x1,          self.y1,           0),
            (self.x1,          self.y1 - 800,     1),
            (self.x1,          self.y1 - 1600,    2),
            (self.x1,          self.y1 - 2400,    3),
        ]
        return stations

    # ------------------------------------------------------------------
    # 电子围栏（相对 x3/y3）
    # ------------------------------------------------------------------
    def get_fence_bounds(self) -> Tuple[float, float, float, float]:
        """返回电子围栏 (x_min, x_max, y_min, y_max)，单位 mm"""
        return (
            self.x3,               # x_min
            self.x3 + 6000,        # x_max
            self.y3 - 2000,        # y_min
            self.y3 + 2000,        # y_max
        )

    # ------------------------------------------------------------------
    # 交换区触发线
    # ------------------------------------------------------------------
    def get_exchange_trigger_x(self) -> float:
        """机器人 x >= 该值时触发兑换区识别"""
        return self.x3 + 3500.0

    # ------------------------------------------------------------------
    # 障碍区（不可到达区域）
    # ------------------------------------------------------------------
    def get_block_obstacle_zones(self) -> list:
        """返回 8 个物资箱周边 700mm 不可到达圆的 (x, y, radius)"""
        zones = []
        for bx, by, bid in self.get_block_coordinates():
            zones.append((bx, by, 700.0, bid))
        return zones

    # ------------------------------------------------------------------
    # 重置雷达原点
    # ------------------------------------------------------------------
    def reset_radar_origin(self, current_x: float, current_y: float,
                           current_yaw: float = 0.0):
        """以当前雷达坐标作为新原点"""
        self.radar_origin_x = current_x
        self.radar_origin_y = current_y
        self.radar_origin_yaw = current_yaw
        self.radar_initialized = True

    def world_to_local(self, wx: float, wy: float) -> Tuple[float, float]:
        """世界坐标 → 本地坐标（相对雷达原点）"""
        return (wx - self.radar_origin_x, wy - self.radar_origin_y)

    def local_to_world(self, lx: float, ly: float) -> Tuple[float, float]:
        """本地坐标 → 世界坐标"""
        return (lx + self.radar_origin_x, ly + self.radar_origin_y)
