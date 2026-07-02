#!/usr/bin/env python3
"""R1 航点执行节点 — 集成 pipeline + 底盘控制.

用法:
  rosrun r1_control waypoint_executor.py [--red|--blue] [--test-scene=N]

从 r1_graph_config.yaml 加载全部配置, 直接调用 pipeline 生成航点,
无需中间 JSON 文件.
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import struct
import serial
import time
from tf.transformations import euler_from_quaternion
import math
import threading

from lib.config import get_config, AppConfig
from pipeline.astar_planner import astar_plan, select_r1_ids, get_crossed_r1_positions
from pipeline.graph_planner import GraphMapper, supplement_ids, route_to_waypoints
from pipeline.kfs_reader import KfsSerialReader
from pipeline.commands_generator import generate_commands, commands_to_binary, fmt_hex
from pipeline.orchestrator import (process_grid, route_to_absolute_waypoints,
                                   DEFAULT_REQUIRED_R2, DEFAULT_REQUIRED_R1)


PHASE_ROTATE = 0
PHASE_TRANSLATE = 1


class WaypointExecutor:
    """KFS → pipeline → 绝对航点 → 两阶段底盘控制."""

    def __init__(self, side="blue", test_scene=None):
        self.cfg = get_config(side)
        self.side = side
        self.test_scene = test_scene

        # ── 状态 ──
        self.target_poses = []
        self.wp_meta = []
        self.waypoint_count = 0
        self.waypoint_index = 0
        self.current_wp_name = ""
        self.current_wp_height = None

        self.current_absolute = None
        self.current_yaw = 0.0
        self.target = None

        self.motion_phase = PHASE_ROTATE
        self.hold_start = None
        self.waypoint_start_time = None
        self.active = False  # 是否有航点在执行

        # ── 底盘串口 (延迟到 KFS 触发后初始化, 共享同一物理口) ──
        self.chassis_serial = None
        self.chassis_fault = False
        self.chassis_lock = threading.Lock()
        self.chassis_data = bytearray(self.cfg.chassis_send_bytes)
        self._chassis_ready = False

        # ── ROS ──
        self.pub_fault = rospy.Publisher("/area12/fault", String, queue_size=10)
        rospy.Subscriber("/aft_mapped_to_init", Odometry, self._on_odom)

        # ── 数据源: test scene 或 KFS 串口 ──
        if self.test_scene is not None:
            self._init_test_scene()
        else:
            self._kfs_reader = KfsSerialReader(
                self.cfg.serial_port, baud=self.cfg.serial_baud, on_grid=self._on_kfs_grid)
            self._kfs_reader.start()

        rospy.on_shutdown(self._cleanup)
        rospy.loginfo(f"[Executor] 已启动, side={side}, "
                      f"test_scene={test_scene}, "
                      f"KFS={self.cfg.serial_port}, "
                      f"cmd_serial={self.cfg.command_port}, "
                      f"chassis={self.cfg.chassis_port}")

    # ==================== KFS → pipeline ====================

    def _on_kfs_grid(self, grid_4x3):
        """KFS 串口收到网格 → 运行 pipeline → 加载航点."""
        rospy.loginfo("[Executor] 收到 KFS 网格, 运行 pipeline...")

        result = process_grid(grid_4x3, self.side, verbose=False)
        if "error" in result:
            rospy.logerr(f"[Executor] pipeline 失败: {result['error']}")
            return

        # KFS 已完成使命, 关闭 KFS 串口, 释放给底盘使用
        if hasattr(self, '_kfs_reader') and self._kfs_reader:
            self._kfs_reader.stop()
            rospy.loginfo("[Executor] KFS 串口已关闭, 切换到底盘模式")

        # 初始化底盘串口 (与 KFS 共用同一物理口)
        self._chassis_init()
        self._chassis_ready = True

        abs_wps = result.get("absolute_waypoints")
        if not abs_wps:
            rospy.logerr("[Executor] pipeline 未返回 absolute_waypoints")
            return

        self._load_absolute_waypoints(abs_wps)

        # 打印命令 (调试)
        cfg = self.cfg
        r1_positions = []
        for r in range(4):
            for c in range(3):
                if int(grid_4x3[r, c]) == 1:
                    r1_positions.append((c, r + 1))
        cmd_str = generate_commands(
            result["astar_path"], [], r1_positions, DEFAULT_REQUIRED_R2, cfg.get_height_map())
        binary_cmds = commands_to_binary(cmd_str)
        rospy.loginfo(f"[Executor] /path/commands: {cmd_str}")
        rospy.loginfo(f"[Executor] Binary ({len(binary_cmds)} bytes): {fmt_hex(binary_cmds)}")

        # 通过命令串口发送二进制指令
        self._send_command_serial(binary_cmds)

    def _send_command_serial(self, payload):
        """发送命令二进制帧: [header(1B) | commands(N bytes) | tail(1B)].

        header: red→0x11, blue→0x22
        """
        c = self.cfg
        hdr = c.command_header[self.side]
        frame = bytes([hdr]) + bytes(payload) + bytes([c.command_tail])
        try:
            ser = serial.Serial(c.command_port, c.command_baud, timeout=c.command_timeout)
            ser.write(frame)
            ser.close()
            rospy.loginfo(f"[Executor] 命令帧已发送到 {c.command_port}: "
                          f"{fmt_hex(frame)}")
        except serial.SerialException as e:
            rospy.logerr(f"[Executor] 命令串口 {c.command_port} 发送失败: {e}")

    # ==================== 测试场景 ====================

    def _init_test_scene(self):
        """从测试场景加载航点, 跳过 KFS 串口."""
        from pipeline.orchestrator import process_test_scene

        rospy.loginfo(f"[Executor] 加载 test scene {self.test_scene}...")
        result = process_test_scene(self.test_scene, self.side, verbose=True)
        if "error" in result:
            rospy.logerr(f"[Executor] test scene 失败: {result['error']}")
            return

        abs_wps = result.get("absolute_waypoints")
        if not abs_wps:
            rospy.logerr("[Executor] test scene 未返回 absolute_waypoints")
            return

        self._load_absolute_waypoints(abs_wps)
        self._chassis_init()
        self._chassis_ready = True
        rospy.loginfo(f"[Executor] test scene 航点已加载, 开始执行")

    # ==================== 航点加载 (共享) ====================

    def _load_absolute_waypoints(self, abs_wps):
        """将绝对航点列表加载到执行器, 开始执行第一个航点."""
        self.target_poses = [(wp["x"], wp["y"], wp["theta"]) for wp in abs_wps]
        self.wp_meta = [{"name": wp["name"], "height": wp.get("height")} for wp in abs_wps]
        self.waypoint_count = len(abs_wps)
        self.waypoint_index = 0

        self.target = self.target_poses[0]
        self.current_wp_name = self.wp_meta[0]["name"]
        self.current_wp_height = self.wp_meta[0].get("height")
        self.waypoint_index = 1
        self.motion_phase = PHASE_TRANSLATE
        self.hold_start = None
        self.waypoint_start_time = time.time()
        self.active = True

        rospy.loginfo(f"[Executor] {self.waypoint_count} 个绝对航点已加载, "
                      f"首个: {self.current_wp_name} → "
                      f"({self.target[0]:.3f},{self.target[1]:.3f},{self.target[2]:.3f})")

    # ==================== 底盘串口 ====================

    def _chassis_init(self):
        c = self.cfg
        try:
            self.chassis_serial = serial.Serial(
                c.chassis_port, c.chassis_baud, timeout=c.chassis_timeout)
            self.chassis_fault = False
            rospy.loginfo(f"[Executor] 底盘串口 {c.chassis_port} 已打开")
        except serial.SerialException as e:
            rospy.logerr(f"[Executor] 底盘串口打开失败: {e}")
            self.chassis_fault = True

    def _chassis_reconnect(self):
        c = self.cfg
        delay = c.chassis_reconnect_base
        while not rospy.is_shutdown():
            if self.chassis_serial and self.chassis_serial.is_open:
                self.chassis_fault = False
                return True
            rospy.logwarn(f"重连底盘串口 {c.chassis_port}... ({delay:.1f}s)")
            time.sleep(delay)
            try:
                self.chassis_serial = serial.Serial(
                    c.chassis_port, c.chassis_baud, timeout=c.chassis_timeout)
                self.chassis_fault = False
                return True
            except serial.SerialException:
                pass
            delay = min(delay * 2, c.chassis_reconnect_max)
        return False

    def _chassis_send(self):
        """两阶段串口发送: ROTATE 只发 dtheta, TRANSLATE 发 (dx, dy, dtheta)."""
        if not self.active or self.target is None:
            return
        if not self._chassis_ready:
            return
        if self.chassis_fault:
            self._chassis_reconnect()
            if self.chassis_fault:
                return
        if self.current_absolute is None:
            return

        c = self.cfg
        with self.chassis_lock:
            if not self.chassis_serial or not self.chassis_serial.is_open:
                self.chassis_fault = True
                return
            try:
                tar_x, tar_y, tar_theta = self.target
                cur_x = self.current_absolute[0]
                cur_y = self.current_absolute[1]
                cur_yaw = self.current_yaw

                if self.motion_phase == PHASE_ROTATE:
                    dx_cm = 0.0
                    dy_cm = 0.0
                    dtheta = (tar_theta - cur_yaw + math.pi) % (2 * math.pi) - math.pi
                else:
                    dx_cm = (tar_x - cur_x) * 100.0
                    dy_cm = (tar_y - cur_y) * 100.0
                    dtheta = (tar_theta - cur_yaw + math.pi) % (2 * math.pi) - math.pi

                # Format: header(1B) + cmd(1B) + 3 floats(12B LE) + tail(1B)
                self.chassis_data[0] = c.chassis_send_header
                if self.hold_start is not None and self.motion_phase == PHASE_TRANSLATE and self.current_wp_name.startswith("TGT"):
                    h = self.current_wp_height
                    cmd = {1: 0x01, 2: 0x02, 3: 0x03}.get(h, 0x00)
                else:
                    cmd = 0x00
                self.chassis_data[1] = cmd
                pack = struct.pack('<3f', dx_cm, dy_cm, dtheta)
                self.chassis_data[2:c.chassis_send_bytes - 1] = pack
                self.chassis_data[c.chassis_send_bytes - 1] = c.chassis_send_tail
                self.chassis_serial.write(self.chassis_data)
                rospy.loginfo(f"dx={dx_cm:.1f}, dy={dy_cm:.1f}, dtheta={dtheta:.3f}, cmd=0x{cmd:02X}")
            except serial.SerialException as e:
                rospy.logerr(f"底盘发送失败: {e}")
                self.chassis_fault = True

    # ==================== odom → 控制 ====================

    def _on_odom(self, msg: Odometry):
        if not self.active or self.target is None:
            return

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])

        c = self.cfg
        # position(IMU) → lidar（zero_point2lidar 不随旋转变化）
        cx = position.x + c.zero_point2lidar["x"]
        cy = position.y + c.zero_point2lidar["y"]
        # lidar → 车体中心（lidar2center 是车体坐标系下的固定偏置，需随 yaw 旋转）
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        cx += c.lidar2center["x"] * cos_yaw - c.lidar2center["y"] * sin_yaw
        cy += c.lidar2center["x"] * sin_yaw + c.lidar2center["y"] * cos_yaw



        self.current_absolute = [cx, cy, 0.0]
        self.current_yaw = yaw

        tar_x, tar_y, tar_theta = self.target

        angle_err = abs(math.atan2(math.sin(tar_theta - yaw),
                                   math.cos(tar_theta - yaw)))
        dist_err = math.hypot(tar_x - cx, tar_y - cy)

        phase_str = "ROT" if self.motion_phase == PHASE_ROTATE else "TRANS"
        rospy.loginfo(f"[{self.current_wp_name}] target=({tar_x:.4f},{tar_y:.4f},{tar_theta:.4f}) "
                      f"odom=({cx:.4f},{cy:.4f},{yaw:.4f}) "
                      f"[{phase_str}] err_d={dist_err:.4f} err_a={angle_err:.4f}")

        # ── 两阶段 ──
        # 首个航点: TRANSLATE → ROTATE → advance
        # 后续航点: ROTATE → TRANSLATE → advance
        if self.motion_phase == PHASE_ROTATE:
            if angle_err < c.angle_thre:
                if self.waypoint_index == 1:
                    rospy.loginfo("旋转到位 → 首个航点完成")
                    self.hold_start = None
                    self._advance()
                    return
                else:
                    rospy.loginfo("旋转到位 → 平移阶段")
                    self.motion_phase = PHASE_TRANSLATE
                    self.hold_start = None
            else:
                self.hold_start = None
        else:
            if dist_err < c.dist_thre:
                if self.hold_start is None:
                    self.hold_start = time.time()
                elif time.time() - self.hold_start >= c.hold_duration:
                    if self.waypoint_index == 1:
                        rospy.loginfo("平移到位 → 旋转阶段")
                        self.motion_phase = PHASE_ROTATE
                        self.hold_start = None
                    else:
                        rospy.loginfo(f"航点到达: #{self.waypoint_index - 1} {self.current_wp_name}")
                        self.hold_start = None
                        self._advance()
                        return
            else:
                self.hold_start = None

        # ── 超时 ──
        if (self.waypoint_start_time and
                time.time() - self.waypoint_start_time > c.waypoint_timeout):
            rospy.logerr(f"航点超时 ({c.waypoint_timeout}s): d={dist_err:.4f} a={angle_err:.4f}")
            self.pub_fault.publish("waypoint_timeout")
            self._advance()
            return

        self._chassis_send()

    # ==================== 航点推进 ====================

    def _advance(self):
        if self.waypoint_index >= self.waypoint_count:
            rospy.loginfo("[Executor] 所有航点已完成")
            self.active = False
            return
        idx = self.waypoint_index
        self.target = self.target_poses[idx]
        self.current_wp_name = self.wp_meta[idx]["name"]
        self.current_wp_height = self.wp_meta[idx].get("height")
        self.waypoint_index += 1
        self.waypoint_start_time = time.time()
        self.hold_start = None
        self.motion_phase = PHASE_ROTATE

        ht = f" h={self.current_wp_height}" if self.current_wp_height else ""
        rospy.loginfo(f"[Executor] 航点 #{idx}/{self.waypoint_count}: "
                      f"{self.current_wp_name}{ht} "
                      f"→ ({self.target[0]:.3f},{self.target[1]:.3f},{self.target[2]:.3f})")
        self._chassis_send()

    # ==================== 清理 ====================

    def _cleanup(self):
        if hasattr(self, '_kfs_reader') and self._kfs_reader:
            self._kfs_reader.stop()
        if self.chassis_serial and self.chassis_serial.is_open:
            self.chassis_serial.close()
        rospy.loginfo("[Executor] 已清理")


if __name__ == '__main__':
    import sys
    side = "blue"
    test_scene = None
    for a in sys.argv:
        if a == "--red":
            side = "red"
        elif a == "--blue":
            side = "blue"
        elif a.startswith("--test-scene="):
            test_scene = int(a.split("=")[1])
    rospy.init_node("waypoint_executor", anonymous=True)
    ex = WaypointExecutor(side=side, test_scene=test_scene)
    rospy.spin()
