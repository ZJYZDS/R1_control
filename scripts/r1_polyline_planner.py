#!/usr/bin/env python3
"""
流程:
  1. 订阅 /R1_grap_pos, 确认目标编号后 unsubscribe.
  2. GraphMapper 构建无向图, 搜索 SELF → TGT1 → TGT2 的最短路径.
  3. 将路径分解为原子动作 (平移/原地旋转), 逐个作为航点执行:
     - 平移动作 (含 TGT): 仅检查距离, 串口输出 (dx, dy, 0)
     - 旋转动作 (仅 K 节点): 仅检查角度, 串口输出 (0, 0, dtheta)
     - TGT 到达: 通过独立串口发送到达帧, 等待下位机回复后继续

订阅:
  /R1_grap_pos         (Int32MultiArray)  [id1, id2]
  /aft_mapped_to_init  (Odometry)         当前位姿
  /R1_grap_pos/reset   (Empty)            重置状态

发布:
  /global_path         (Path)             折线路径 (可视化)
  /r1_waypoints_viz    (Path)             图节点 + 路线节点
  /arrive_tgt          (Int32)            到达 TGT 时发布对应 ID
"""

import sys, os
sys.path.insert(0, os.path.dirname(__file__))

import rospy
import numpy as np
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, Int32MultiArray, Empty, String
import tf.transformations
import threading
import time
import struct
import serial
import math

from r1_config import Config
from r1_graph_mapper import GraphMapper


# ── R1 Sequential Runner ──────────────────────────────────────────────
class R1SequentialRunner:
    def __init__(self):
        rospy.init_node("r1_polyline_planner")

        # ── 位姿状态 ──
        self.origin_pose = None
        self.initial_origin = None
        self.current_absolute = None
        self.current_pose = [0.0, 0.0, 0.0]
        self.header = None

        # ── 航点执行状态 ──
        self.target = None
        self.waypoints = []          # [(dx, dy, dyaw), ...] 预计算 relative target
        self.waypoint_types = []     # ['translate' | 'rotate', ...]
        self.waypoint_type = None    # 当前航点类型
        self.waypoint_names = []
        self.waypoint_heights = []   # 每航点的 height 字节 (0x00 或 0x01-0x03)
        self.waypoint_height = 0x00  # 当前航点 height
        self.tgt_ids = {}
        self.wp_index = 0
        self.finished = False

        # ── TGT 串口握手状态 ──
        self.waiting_tgt_response = False
        self.current_tgt_name = None
        self.tgt_serial_port = None
        self.tgt_serial_fault = False
        self.tgt_receive_buffer = bytearray()
        self._tgt_serial_init()

        # ── /R1_grap_pos 确认机制 ──
        self.confirm_buffer = []
        self.confirmed_ids = None
        self.grap_sub = None

        # ── 规划结果 (可视化) ──
        self.graph_nodes = []
        self.route_nodes = []
        self.full_path_world = []

        # ── 串口 ──
        self.cmd = 0x11
        self.arrived = False
        self.serial_port = None
        self.serial_fault = False
        self.serial_lock = threading.Lock()
        self.serial_data = bytearray(Config.SEND_BYTES)
        self.receive_data = bytearray(Config.RECEIVE_BYTES)
        self.receive_buffer = bytearray()
        self.serial_init()

        # ── 航点超时 ──
        self.waypoint_start_time = None

        # ── Publishers ──
        self.pub_path = rospy.Publisher("/global_path", Path,
                                         queue_size=1, latch=True)
        self.pub_wp   = rospy.Publisher("/r1_waypoints_viz", Path,
                                         queue_size=1, latch=True)
        self.pub_fault = rospy.Publisher("/area12/fault", String, queue_size=10)
        self.pub_arrive_tgt = rospy.Publisher("/arrive_tgt", Int32, queue_size=1, latch=True)

        # ── Subscribers ──
        rospy.Subscriber("/aft_mapped_to_init",  Odometry,        self.cb_odom)
        self.grap_sub = rospy.Subscriber("/R1_grap_pos", Int32MultiArray, self.cb_grap_pos)
        rospy.Subscriber("/R1_grap_pos/reset",   Empty,           self.cb_reset)

        # ── Timers ──
        rospy.Timer(rospy.Duration(1.0 / Config.PATH_PUB_RATE),
                     lambda _: self._publish_paths())

        # ── 独立线程 ──
        self.receive_thread = threading.Thread(target=self.receive_listener, daemon=True)
        self.receive_thread.start()
        self.tgt_receive_thread = threading.Thread(target=self._tgt_receive_listener, daemon=True)
        self.tgt_receive_thread.start()

        # 串口未连接时后台定时重连
        if self.serial_fault:
            threading.Thread(target=self.serial_reconnect, daemon=True).start()
        if self.tgt_serial_fault:
            threading.Thread(target=self._tgt_serial_reconnect, daemon=True).start()

        rospy.on_shutdown(self.shutdown_cleanup)

        rospy.loginfo("R1SequentialRunner ready.\n"
                      "  /aft_mapped_to_init → robot pose\n"
                      "  /R1_grap_pos (Int32MultiArray) → [id1, id2]\n"
                      f"  Confirm threshold: {Config.N_CONFIRM} times\n"
                      "  Graph → route nodes → sequential waypoint execution")

    # ── Odom ────────────────────────────────────────────────────────

    def cb_odom(self, msg: Odometry):
        if self.waiting_tgt_response:
            return

        self.header = msg.header
        pos = msg.pose.pose.position
        quat = msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion(
            [quat.x, quat.y, quat.z, quat.w])

        self.current_absolute = [pos.x, pos.y, yaw]
        if self.origin_pose is None:
            self.origin_pose = [pos.x, pos.y, yaw]
        if self.initial_origin is None:
            self.initial_origin = [pos.x, pos.y, yaw]
            rospy.loginfo(f"Initial origin: ({pos.x:.3f}, {pos.y:.3f}, {yaw:.3f})")

        origin_x, origin_y, origin_yaw = self.origin_pose
        self.current_pose = [
            pos.x - origin_x,
            pos.y - origin_y,
            np.arctan2(np.sin(yaw - origin_yaw), np.cos(yaw - origin_yaw))
        ]

        if self.target is None or self.finished:
            return

        curr_x, curr_y, curr_yaw = self.current_pose
        tar_x, tar_y, tar_yaw = self.target
        rospy.logwarn_throttle(1.0, f"位姿: ({curr_x:.4f},{curr_y:.4f},{curr_yaw:.4f}) "
                               f"→ target ({tar_x:.4f},{tar_y:.4f},{tar_yaw:.4f})")

        angle_error = abs(np.arctan2(np.sin(tar_yaw - curr_yaw),
                                     np.cos(tar_yaw - curr_yaw)))
        dist_error = np.hypot(tar_x - curr_x, tar_y - curr_y)

        if self.waypoint_type == 'translate':
            arrived = dist_error < Config.DIST_THRE
        elif self.waypoint_type == 'rotate':
            arrived = angle_error < Config.ANGLE_THRE
        else:
            arrived = (angle_error < Config.ANGLE_THRE
                       and dist_error < Config.DIST_THRE)

        if arrived:
            wp_name = (self.waypoint_names[self.wp_index]
                       if self.wp_index < len(self.waypoint_names) else None)

            if wp_name and wp_name.startswith("TGT"):
                tgt_id = self.tgt_ids.get(wp_name)
                if tgt_id is not None:
                    self.pub_arrive_tgt.publish(tgt_id)
                    rospy.loginfo(f"Arrived at {wp_name}, published /arrive_tgt id={tgt_id}")
                if not self.waiting_tgt_response:
                    # Send one chassis frame with height byte set
                    self.waypoint_height = self.waypoint_heights[self.wp_index]
                    self.serial_send()
                    # Then TGT serial handshake
                    self._send_tgt_arrival(wp_name)
                    self.waiting_tgt_response = True
                    self.current_tgt_name = wp_name
            else:
                # 非 TGT 航点 → 直接前进
                rospy.loginfo(f"Arrived at non-TGT waypoint {wp_name}, auto-advancing")
                self.wp_index += 1
                self._start_waypoint(self.wp_index)

        if (not self.waiting_tgt_response
                and self.waypoint_start_time is not None
                and time.time() - self.waypoint_start_time > Config.WAYPOINT_TIMEOUT):
            rospy.logerr(f"航点超时 ({Config.WAYPOINT_TIMEOUT}s), "
                         f"dist={dist_error:.4f} angle={angle_error:.4f}")
            self.pub_fault.publish("waypoint_timeout")
            self.wp_index += 1
            self._start_waypoint(self.wp_index)

        if not self.waiting_tgt_response:
            self.serial_send()

    # ── /R1_grap_pos ────────────────────────────────────────────────

    def cb_grap_pos(self, msg: Int32MultiArray):
        if self.confirmed_ids is not None:
            return

        data = list(msg.data)
        if len(data) != 2:
            rospy.logwarn(f"/R1_grap_pos expects 2 ints, got {len(data)}")
            return

        self.confirm_buffer.append(data)
        rospy.loginfo(f"Received #{len(self.confirm_buffer)}/{Config.N_CONFIRM}: {data}")

        if len(self.confirm_buffer) >= Config.N_CONFIRM:
            self._finalize_ids()

    def _finalize_ids(self):
        first = self.confirm_buffer[0]
        for i, d in enumerate(self.confirm_buffer[1:], start=2):
            if d != first:
                rospy.logwarn(f"Mismatch at #{i}: {d} != {first}, resetting buffer")
                self.confirm_buffer = []
                return

        self.confirmed_ids = tuple(first)
        rospy.loginfo(f"IDs confirmed: {self.confirmed_ids} (after {Config.N_CONFIRM} checks)")

        if self.grap_sub is not None:
            self.grap_sub.unregister()
            self.grap_sub = None
            rospy.loginfo("Unsubscribed from /R1_grap_pos")

        self._map_and_plan()

    def cb_reset(self, _):
        self.confirm_buffer = []
        self.confirmed_ids = None
        self.waypoints = []
        self.waypoint_types = []
        self.waypoint_type = None
        self.waypoint_names = []
        self.waypoint_heights = []
        self.waypoint_height = 0x00
        self.tgt_ids = {}
        self.wp_index = 0
        self.target = None
        self.full_path_world = []
        self.graph_nodes = []
        self.route_nodes = []
        self.waiting_tgt_response = False
        self.finished = False

        if self.grap_sub is None:
            self.grap_sub = rospy.Subscriber("/R1_grap_pos", Int32MultiArray, self.cb_grap_pos)

        rospy.loginfo("Reset complete, re-subscribed /R1_grap_pos")

    # ── Mapping & Planning ──────────────────────────────────────────

    def _map_and_plan(self):
        id1, id2 = self.confirmed_ids
        mapper = GraphMapper()

        c1 = mapper.id_to_coord(id1)
        c2 = mapper.id_to_coord(id2)
        if c1 is None or c2 is None:
            rospy.logerr("ID→coord failed")
            return

        self.tgt_ids = {"TGT1": id1, "TGT2": id2}

        rospy.loginfo(f"Target coords: TGT1(id={id1})=({c1[0]:.3f},{c1[1]:.3f},{c1[2]:.3f}), "
                       f"TGT2(id={id2})=({c2[0]:.3f},{c2[1]:.3f},{c2[2]:.3f})")

        if self.current_absolute is None:
            rospy.logerr("No odom — cannot plan. Is FAST-LIVO running?")
            return

        nodes, adj = mapper.build_graph([c1, c2])
        self.graph_nodes = nodes

        start_idx = Config.K_INDEX[Config.START_K] + Config.K_GRAPH_OFFSET
        end_idx   = Config.K_INDEX[Config.END_K]   + Config.K_GRAPH_OFFSET
        rospy.loginfo(f"Scenario: {Config.SCENARIO}, "
                       f"Start: {Config.START_K}(idx={start_idx}), "
                       f"End: {Config.END_K}(idx={end_idx})")

        route = mapper.find_path(nodes, adj, start_idx, end_idx)
        if not route:
            return
        self.route_nodes = [n for n, _ in route]

        wp_list = self._route_to_waypoints(route, self.tgt_ids)
        self.waypoints = []
        self.waypoint_types = []
        self.waypoint_names = []
        self.waypoint_heights = []
        for wp_type, dx, dy, dyaw, name in wp_list:
            self.waypoints.append((dx, dy, dyaw))
            self.waypoint_types.append(wp_type)
            self.waypoint_names.append(name)
            if name.startswith('TGT'):
                tgt_id = self.tgt_ids.get(name)
                if tgt_id is not None:
                    coord = mapper.id_to_coord(tgt_id)
                    self.waypoint_heights.append(int(coord[2]) if coord is not None else 0x00)
                else:
                    self.waypoint_heights.append(0x00)
            else:
                self.waypoint_heights.append(0x00)
        rospy.loginfo(f"Waypoints ({len(self.waypoints)}): "
                       f"{list(zip(self.waypoint_names, self.waypoints, self.waypoint_heights))}")

        self._build_polyline(route)

        self.wp_index = 0
        self._start_waypoint(0)

    def _route_to_waypoints(self, route, tgt_ids):
        """Decompose route into atomic translate/rotate waypoints.

        Returns list of (type, dx, dy, dyaw, name):
          - 'translate': pure translation, target_yaw=0
          - 'rotate':    pure rotation at K node or TGT, dx=dy=0

        K nodes get rotate-before-first-segment and rotate-at-each-turn
        actions; at K nodes before a TGT, rotate to face the TGT's inward direction.
        """
        n = len(route)
        headings = []
        for i in range(n - 1):
            a, b = route[i][1], route[i + 1][1]
            headings.append(math.atan2(b[1] - a[1], b[0] - a[0]))

        dir_c = Config.DIR_CORRECT
        wp = []

        for i in range(n):
            name = route[i][0]
            coord = route[i][1]
            is_k = name.startswith('K')
            is_tgt = name.startswith('TGT')

            if i == 0:
                if is_k:
                    if n >= 2 and route[1][0].startswith('TGT'):
                        tgt_id = tgt_ids.get(route[1][0])
                        if tgt_id is not None and tgt_id in Config.TGT_FACING:
                            target_facing = Config.TGT_FACING[tgt_id]
                            dtheta = (target_facing - Config.START_YAW) * dir_c
                        else:
                            dtheta = (headings[0] - Config.START_YAW) * dir_c
                    else:
                        dtheta = (headings[0] - Config.START_YAW) * dir_c
                    dtheta = math.atan2(math.sin(dtheta), math.cos(dtheta))
                    if abs(dtheta) > 0.001:
                        wp.append(('rotate', 0.0, 0.0, dtheta, name))
            else:
                prev = route[i - 1][1]
                dx = coord[0] - prev[0]
                dy = coord[1] - prev[1]
                wp.append(('translate', dx, dy, 0.0, name))

                if is_k and i < n - 1:
                    next_name = route[i + 1][0]
                    if next_name.startswith('TGT'):
                        tgt_id = tgt_ids.get(next_name)
                        if tgt_id is not None and tgt_id in Config.TGT_FACING:
                            target_facing = Config.TGT_FACING[tgt_id]
                            dtheta = (target_facing - headings[i - 1]) * dir_c
                        else:
                            dtheta = (headings[i] - headings[i - 1]) * dir_c
                    else:
                        dtheta = (headings[i] - headings[i - 1]) * dir_c
                    dtheta = math.atan2(math.sin(dtheta), math.cos(dtheta))
                    if abs(dtheta) > 0.001:
                        wp.append(('rotate', 0.0, 0.0, dtheta, name))
                elif is_k:
                    dtheta = (Config.TARGET_END_YAW - headings[-1]) * dir_c
                    dtheta = math.atan2(math.sin(dtheta), math.cos(dtheta))
                    if abs(dtheta) > 0.001:
                        wp.append(('rotate', 0.0, 0.0, dtheta, name))

        return wp

    def _build_polyline(self, route):
        full = []
        for i in range(len(route) - 1):
            _, pa = route[i]
            _, pb = route[i + 1]
            seg = self._interpolate_segment(pa[0], pa[1], pb[0], pb[1])
            full.extend(seg if not full else seg[1:])
        self.full_path_world = full
        rospy.loginfo(f"Polyline: {len(route)} nodes → {len(full)} pts")
        self._publish_paths()

    def _interpolate_segment(self, ax, ay, bx, by):
        dist = np.hypot(bx - ax, by - ay)
        n = max(2, int(dist / Config.PATH_INTERP_RES) + 1)
        xs = np.linspace(ax, bx, n)
        ys = np.linspace(ay, by, n)
        return [(float(x), float(y)) for x, y in zip(xs, ys)]

    # ── 航点执行 ────────────────────────────────────────────────────

    def _start_waypoint(self, idx):
        if idx >= len(self.waypoints):
            rospy.loginfo("All waypoints completed")
            self.finished = True
            self.target = None
            return

        if self.current_absolute is not None:
            self.origin_pose = self.current_absolute.copy()

        dx, dy, dyaw = self.waypoints[idx]
        self.target = (dx, dy, dyaw)
        self.waypoint_type = self.waypoint_types[idx]
        self.waypoint_height = 0x00

        self.waiting_tgt_response = False
        self.waypoint_start_time = time.time()
        rospy.loginfo(f"Waypoint {idx} [{self.waypoint_type}]: "
                       f"delta ({dx:.3f},{dy:.3f},{dyaw:.3f}) "
                       f"height=0x{self.waypoint_height:02X}")

    # ── TGT 串口握手 ──────────────────────────────────────────────────

    def _tgt_serial_init(self):
        try:
            self.tgt_serial_port = serial.Serial(
                Config.TGT_SERIAL_PORT, Config.TGT_BAUD_RATE,
                timeout=Config.TGT_SERIAL_TIMEOUT)
            self.tgt_serial_fault = False
            rospy.loginfo(f"TGT serial {Config.TGT_SERIAL_PORT} opened")
        except serial.SerialException as e:
            rospy.logerr(f"TGT serial open failed: {e}")
            self.tgt_serial_fault = True

    def _tgt_serial_reconnect(self):
        delay = Config.TGT_RECONNECT_BASE_DELAY
        while not rospy.is_shutdown():
            if self.tgt_serial_port and self.tgt_serial_port.is_open:
                self.tgt_serial_fault = False
                return True
            rospy.logwarn(f"Reconnecting TGT serial {Config.TGT_SERIAL_PORT}... ({delay:.1f}s)")
            time.sleep(delay)
            self._tgt_serial_init()
            if not self.tgt_serial_fault:
                rospy.loginfo("TGT serial reconnected")
                return True
            delay = min(delay * 2, Config.TGT_RECONNECT_MAX_DELAY)
        return False

    def _send_tgt_arrival(self, wp_name):
        if self.tgt_serial_fault:
            self._tgt_serial_reconnect()
            if self.tgt_serial_fault:
                rospy.logerr("TGT serial unavailable, cannot send arrival frame")
                return

        try:
            cmd = Config.TGT1_CMD if wp_name == "TGT1" else Config.TGT2_CMD
            frame = bytearray()
            h = Config.TGT_SEND_HEADER
            frame.extend(struct.pack('>H' if h > 0xFF else '>B', h))
            frame.append(cmd)
            t = Config.TGT_SEND_TAIL
            frame.extend(struct.pack('>H' if t > 0xFF else '>B', t))
            self.tgt_serial_port.write(frame)
            rospy.loginfo(f"Sent TGT arrival frame for {wp_name}: "
                          f"header=0x{h:04X} cmd=0x{cmd:02X} tail=0x{t:04X}")
        except serial.SerialException as e:
            rospy.logerr(f"TGT serial send failed: {e}")
            self.tgt_serial_fault = True

    def _tgt_receive_listener(self):
        HEADER = Config.TGT_RECEIVE_HEADER
        TAIL = Config.TGT_RECEIVE_TAIL
        FRAME_LEN = Config.TGT_RECEIVE_BYTES

        while not rospy.is_shutdown():
            if (self.tgt_serial_fault or not self.tgt_serial_port
                    or not self.tgt_serial_port.is_open):
                time.sleep(0.5)
                continue

            if not self.waiting_tgt_response:
                time.sleep(0.1)
                continue

            try:
                waiting = self.tgt_serial_port.in_waiting
                if waiting > 0:
                    raw = self.tgt_serial_port.read(waiting)
                    self.tgt_receive_buffer.extend(raw)

                buf = self.tgt_receive_buffer
                while len(buf) >= FRAME_LEN:
                    head_val = HEADER.to_bytes(1 if HEADER <= 0xFF else 2, 'big')
                    head_pos = buf.find(head_val)
                    if head_pos < 0:
                        buf.clear()
                        break
                    if head_pos > 0:
                        del buf[:head_pos]
                    if len(buf) < FRAME_LEN:
                        break
                    if buf[FRAME_LEN - 1] == TAIL & 0xFF:
                        status = buf[1]
                        rospy.loginfo(f"TGT response: 0x{status:02X}")
                        if status == Config.TGT_RECEIVE_OK:
                            rospy.loginfo(f"TGT {self.current_tgt_name} acknowledged, advancing")
                            del buf[:FRAME_LEN]
                            self.waiting_tgt_response = False
                            self.current_tgt_name = None
                            self.wp_index += 1
                            self._start_waypoint(self.wp_index)
                        else:
                            rospy.logwarn(f"TGT unexpected response: 0x{status:02X}")
                            del buf[:FRAME_LEN]
                    else:
                        del buf[0]

                if len(buf) > 1024:
                    rospy.logwarn(f"TGT receive buffer overflow ({len(buf)} bytes), clearing")
                    buf.clear()

            except serial.SerialException as e:
                rospy.logwarn(f"TGT serial receive error: {e}")
                self.tgt_serial_fault = True
            time.sleep(0.01)

    # ── 串口 ────────────────────────────────────────────────────────

    def serial_init(self):
        try:
            self.serial_port = serial.Serial(
                Config.SERIAL_PORT, Config.BAUD_RATE, timeout=Config.SERIAL_TIMEOUT)
            self.serial_fault = False
            rospy.loginfo("串口初始化成功")
            return True
        except serial.SerialException as e:
            rospy.logerr(f"串口打开失败: {e}")
            self.serial_fault = True
            return False

    def serial_reconnect(self):
        delay = Config.RECONNECT_BASE_DELAY
        while not rospy.is_shutdown():
            if self.serial_port and self.serial_port.is_open:
                self.serial_fault = False
                return True
            rospy.logwarn(f"尝试重连串口 {Config.SERIAL_PORT}... ({delay:.1f}s)")
            time.sleep(delay)
            if self.serial_init():
                rospy.loginfo("串口重连成功")
                return True
            delay = min(delay * 2, Config.RECONNECT_MAX_DELAY)
        return False

    def serial_send(self):
        if self.target is None:
            return
        if self.serial_fault:
            self.serial_reconnect()
            if self.serial_fault:
                return

        with self.serial_lock:
            if not self.serial_port or not self.serial_port.is_open:
                self.serial_fault = True
                rospy.logwarn_throttle(5, "串口已断开, 下次调用触发重连")
                return
            try:
                self.serial_data[0:2] = struct.pack('>H', Config.SEND_HEADER)
                self.serial_data[2] = self.cmd
                self.serial_data[-2:] = struct.pack('>H', Config.SEND_TAIL)

                if self.waypoint_type == 'translate':
                    dx = (self.target[0] - self.current_pose[0]) * 100
                    dy = (self.target[1] - self.current_pose[1]) * 100
                    thi = 0.0
                elif self.waypoint_type == 'rotate':
                    dx = 0.0
                    dy = 0.0
                    thi = math.atan2(
                        math.sin(self.target[2] - self.current_pose[2]),
                        math.cos(self.target[2] - self.current_pose[2]))
                else:
                    dx = (self.target[0] - self.current_pose[0]) * 100
                    dy = (self.target[1] - self.current_pose[1]) * 100
                    thi = math.atan2(
                        math.sin(self.target[2] - self.current_pose[2]),
                        math.cos(self.target[2] - self.current_pose[2]))
                pack_data = struct.pack('<3f', dx, dy, thi)
                self.serial_data[3:15] = pack_data               # 3 floats = 12 bytes
                self.serial_data[15] = self.waypoint_height       # height byte
                self.serial_port.write(self.serial_data)
            except serial.SerialException as e:
                rospy.logerr(f"串口发送失败: {e}")
                self.serial_fault = True

    def receive_listener(self):
        HEADER = Config.RECEIVE_HEADER
        TAIL   = Config.RECEIVE_TAIL
        FRAME_LEN = Config.RECEIVE_BYTES

        while not rospy.is_shutdown():
            if self.serial_fault or not self.serial_port or not self.serial_port.is_open:
                time.sleep(0.5)
                continue
            try:
                waiting = self.serial_port.in_waiting
                if waiting > 0:
                    raw = self.serial_port.read(waiting)
                    self.receive_buffer.extend(raw)

                buf = self.receive_buffer
                while len(buf) >= FRAME_LEN:
                    head_pos = buf.find(HEADER)
                    if head_pos < 0:
                        buf.clear()
                        break
                    if head_pos > 0:
                        del buf[:head_pos]
                    if len(buf) < FRAME_LEN:
                        break
                    if buf[FRAME_LEN - 1] == TAIL:
                        status = buf[1]
                        rospy.logdebug(f"下位机应答: 0x{status:02x}")
                        del buf[:FRAME_LEN]
                    else:
                        del buf[0]

                if len(buf) > 1024:
                    rospy.logwarn(f"接收缓冲区溢出 ({len(buf)} 字节), 全清")
                    buf.clear()

            except serial.SerialException as e:
                rospy.logwarn(f"串口接收异常: {e}")
                self.serial_fault = True
            time.sleep(0.01)

    # ── 可视化发布 ──────────────────────────────────────────────────

    def _publish_paths(self):
        self._pub_path_msg(self.pub_path, self.full_path_world)
        viz_pts = []
        if self.graph_nodes:
            viz_pts.extend([(c[0], c[1]) for _, c in self.graph_nodes])
        self._pub_path_msg(self.pub_wp, viz_pts)

    def _pub_path_msg(self, pub, pts):
        if not pts:
            return
        msg = Path()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "camera_init"
        for x, y in pts:
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.position.z = 0.0
            p.pose.orientation.w = 1.0
            msg.poses.append(p)
        pub.publish(msg)

    def shutdown_cleanup(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        if self.tgt_serial_port and self.tgt_serial_port.is_open:
            self.tgt_serial_port.close()
        rospy.loginfo("串口已关闭, 资源释放完成")


if __name__ == "__main__":
    try:
        R1SequentialRunner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
