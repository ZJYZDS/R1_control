#!/usr/bin/env python3
"""lidar2center 标定脚本 — 多组 180° 配对取中值.

用法:
  rosrun r1_control calibrate_lidar2center.py [--blue|--red]

原理:
  底盘绕车体中心旋转, IMU 绕中心画圆.
  对于任意 yaw 角 θ, IMU 在 θ 和 θ+π 时的坐标对称分布在圆两侧,
  lidar2center = (pos_θ + pos_θ+π) / 2

  采集多组 180° 配对, 取中值, 比单次更鲁棒.

操作:
  1. 上电后 yaw≈0
  2. 运行脚本
  3. 遥控机器人绕车体中心旋转 > 360° (圈数越多数据越多)
  4. 按 Ctrl+C 输出结果
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rospy
import numpy as np
import math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

ANGLE_TOL = math.radians(5.0)   # 配对角度容差


class Lidar2CenterCalibrator:
    def __init__(self):
        self.records = []  # [(x, y, yaw), ...]

        rospy.init_node("calibrate_lidar2center", anonymous=True)
        rospy.Subscriber("/aft_mapped_to_init", Odometry, self._on_odom)
        rospy.loginfo("[Calib] 已启动, 遥控机器人绕车体中心旋转 > 360°...")
        rospy.loginfo("[Calib] 多转几圈数据更准, Ctrl+C 结束")

    def _on_odom(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        self.records.append((pos.x, pos.y, yaw))

        rospy.loginfo_throttle(
            1.0,
            f"[Calib] 已采样 {len(self.records)} 点  "
            f"yaw={np.degrees(yaw):.1f}°  pos=({pos.x:.4f}, {pos.y:.4f})"
        )

    def report(self):
        if len(self.records) < 20:
            rospy.logerr("[Calib] 采样点太少")
            return

        rec = np.array(self.records)  # (N, 3): x, y, yaw

        # 每个点找其 180° 对偶点
        estimates = []
        yaws = rec[:, 2]
        for i in range(len(rec)):
            xi, yi, yi_i = rec[i]
            # 目标 yaw: θ + π (或 θ - π), 归一化
            target = yi_i + math.pi
            target = math.atan2(math.sin(target), math.cos(target))

            # 找 yaw 最接近 target 的点
            diff = np.abs(yaws - target)
            # 处理跨越 ±π 的情况
            diff = np.minimum(diff, 2 * math.pi - diff)

            j = np.argmin(diff)
            if diff[j] > ANGLE_TOL or j == i:
                continue

            xj, yj, _ = rec[j]
            lc_x = (xi + xj) / 2.0
            lc_y = (yi + yj) / 2.0
            estimates.append((lc_x, lc_y))

        if len(estimates) < 5:
            rospy.logerr(f"[Calib] 有效配对太少 ({len(estimates)} 组), 请旋转更大角度")
            return

        est = np.array(estimates)
        lc_x_med = np.median(est[:, 0])
        lc_y_med = np.median(est[:, 1])
        lc_x_std = np.std(est[:, 0])
        lc_y_std = np.std(est[:, 1])
        lc_x_mean = np.mean(est[:, 0])
        lc_y_mean = np.mean(est[:, 1])
        radius = np.median(np.sqrt(est[:, 0]**2 + est[:, 1]**2))

        rospy.loginfo("=" * 55)
        rospy.loginfo("[Calib] 标定结果")
        rospy.loginfo("=" * 55)
        rospy.loginfo(f"  采样点数:      {len(rec)}")
        rospy.loginfo(f"  有效配对:      {len(estimates)} 组 (180°±{np.degrees(ANGLE_TOL):.1f}°)")
        rospy.loginfo("")
        rospy.loginfo(f"  lidar2center_x  mean={lc_x_mean:.4f}  median={lc_x_med:.4f}  std={lc_x_std:.4f} m")
        rospy.loginfo(f"  lidar2center_y  mean={lc_y_mean:.4f}  median={lc_y_med:.4f}  std={lc_y_std:.4f} m")
        rospy.loginfo(f"  |lidar2center|  ≈ {radius:.4f} m")
        rospy.loginfo("")
        rospy.loginfo("[Calib] 建议填入 r1_graph_config.yaml (使用中值):")
        rospy.loginfo(f"  lidar2center_offset_x: {lc_x_med:.4f}")
        rospy.loginfo(f"  lidar2center_offset_y: {lc_y_med:.4f}")
        rospy.loginfo("=" * 55)


def main():
    calib = Lidar2CenterCalibrator()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    calib.report()


if __name__ == "__main__":
    main()
