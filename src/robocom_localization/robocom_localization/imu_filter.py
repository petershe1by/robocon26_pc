#!/usr/bin/env python3
"""
imu_filter.py — Mid-360 IMU 姿态解算与滤波

处理链路：
  /livox/imu (200Hz raw)
    → 4帧滑动均值（去气泵振动）
    → 互补滤波（加速度计定 roll/pitch，陀螺仪积分得 yaw）
    → /imu_orientation (Float32MultiArray[6], 50Hz)
"""

import math, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray


class OrientationFilter:
    """互补滤波器：加速度计+陀螺仪 → roll/pitch/yaw"""

    def __init__(self, alpha: float = 0.98):
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._last_time = None
        self._alpha = alpha

    def update(self, ax, ay, az, gx, gy, gz, stamp_ns: int):
        stamp_s = stamp_ns * 1e-9
        if self._last_time is None:
            self._last_time = stamp_s
            return 0.0, 0.0, 0.0

        dt = stamp_s - self._last_time
        if dt <= 0 or dt > 0.1:
            self._last_time = stamp_s
            return math.degrees(self._roll), math.degrees(self._pitch), math.degrees(self._yaw)

        # 加速度计推算 roll/pitch
        accel_roll = math.atan2(ay, az)
        accel_pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))

        # 陀螺仪积分
        gyro_roll = self._roll + gx * dt
        gyro_pitch = self._pitch + gy * dt
        gyro_yaw = self._yaw + gz * dt

        # 互补融合
        a = self._alpha
        self._roll = a * gyro_roll + (1 - a) * accel_roll
        self._pitch = a * gyro_pitch + (1 - a) * accel_pitch
        self._yaw = gyro_yaw  # 纯陀螺仪积分，无磁力计修正

        self._last_time = stamp_s
        return math.degrees(self._roll), math.degrees(self._pitch), math.degrees(self._yaw)


class ImuFilterNode(Node):
    """IMU 滤波与方位解算节点"""

    def __init__(self):
        super().__init__("imu_filter")
        self.declare_parameter("filter_window", 4)
        self.declare_parameter("complementary_alpha", 0.98)

        win = self.get_parameter("filter_window").value
        self._alpha = self.get_parameter("complementary_alpha").value

        # 滑动窗口
        self._accel_buf = []   # [ax, ay, az]
        self._gyro_buf = []    # [gx, gy, gz]
        self._window = win

        self._orient = OrientationFilter(alpha=self._alpha)
        self._pub_imu = self.create_publisher(Imu, "/imu_filtered", 10)
        self._pub_angle = self.create_publisher(Float32MultiArray, "/imu_orientation", 10)
        self.create_subscription(Imu, "/livox/imu", self._cb, 10)

        self.get_logger().info(
            f"IMU 滤波器启动: 窗口={win}帧, α={self._alpha}"
        )

    def _cb(self, msg: Imu):
        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z

        # ===== 1. 滑动窗口均值 =====
        self._accel_buf.append([ax, ay, az])
        self._gyro_buf.append([gx, gy, gz])
        if len(self._accel_buf) > self._window:
            self._accel_buf.pop(0)
            self._gyro_buf.pop(0)

        import numpy as np
        a_mean = [ax, ay, az] if len(self._accel_buf) < self._window else \
                 np.mean(self._accel_buf, axis=0).tolist()
        g_mean = [gx, gy, gz] if len(self._gyro_buf) < self._window else \
                 np.mean(self._gyro_buf, axis=0).tolist()

        # ===== 2. 发布滤波后 IMU（供 EKF 使用）=====
        filtered = Imu()
        filtered.header = msg.header
        filtered.header.frame_id = "base_link"
        filtered.linear_acceleration.x, filtered.linear_acceleration.y, filtered.linear_acceleration.z = a_mean
        filtered.angular_velocity.x, filtered.angular_velocity.y, filtered.angular_velocity.z = g_mean
        filtered.orientation_covariance = msg.orientation_covariance
        self._pub_imu.publish(filtered)

        # ===== 3. 互补滤波 → 欧拉角 =====
        roll, pitch, yaw = self._orient.update(
            a_mean[0], a_mean[1], a_mean[2],
            g_mean[0], g_mean[1], g_mean[2],
            msg.header.stamp.nanosec + msg.header.stamp.sec * 1e9
        )

        # ===== 4. 发布 /imu_orientation =====
        arr = Float32MultiArray()
        arr.data = [roll, pitch, yaw, g_mean[0], g_mean[1], g_mean[2]]
        self._pub_angle.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = ImuFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
