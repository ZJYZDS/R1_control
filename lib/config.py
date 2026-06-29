"""统一配置加载 — 无 ROS 依赖."""
import os
import math
import yaml


_SCRIPT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def _eval_num(val, **ctx):
    if isinstance(val, (int, float)):
        return float(val)
    if isinstance(val, str):
        s = val.replace('（', '(').replace('）', ')').replace('＊', '*').replace('－', '-')
        try:
            return float(eval(s, {"__builtins__": {}, "math": math}, ctx))
        except Exception:
            return float(val) if _is_numeric(val) else 0.0
    return float(val)


def _is_numeric(s):
    try:
        float(str(s).replace('（', '').replace('）', '').strip())
        return True
    except ValueError:
        return False


class AppConfig:
    """加载 r1_graph_config.yaml + path_planner/config.yaml, 暴露所有参数."""

    def __init__(self, side=None):
        if side is None:
            side = os.environ.get("R1_SCENARIO", "blue")
        self.side = side

        # ── 图规划配置 ──
        self.graph_cfg = _load_yaml(os.path.join(_SCRIPT_DIR, "config", "r1_graph_config.yaml"))
        sc = self.graph_cfg.get(side, self.graph_cfg.get("red", {}))

        self.scenario = side
        self.zero_point = sc.get("zero_point", {"x": 0, "y": 0, "z": 0, "yaw": 0})
        self.relative_positions = sc.get("relative_positions", {})
        self.id_to_coord = sc.get("id_to_coord", {})
        self.start_k = sc.get("start_k", "k3")
        self.end_k = sc.get("end_k", "k4")

        self.ref_yaw = float(sc.get("ref_yaw", 0.0))
        self.dir_correct = int(sc.get("dir_correct", 1))
        angle_ctx = {"ref_yaw": self.ref_yaw, "dir_correct": self.dir_correct}
        self.start_yaw = _eval_num(sc.get("start_yaw", 0.0), **angle_ctx)
        self.target_end_yaw = _eval_num(sc.get("target_end_yaw", 0.0), **angle_ctx)

        raw_facing = sc.get("tgt_facing", {})
        self.tgt_facing = {}
        for k, v in raw_facing.items():
            if isinstance(v, list):
                self.tgt_facing[int(k)] = [_eval_num(x, **angle_ctx) for x in v]
            else:
                self.tgt_facing[int(k)] = _eval_num(v, **angle_ctx)

        # 阈值
        th = self.graph_cfg.get("thresholds", {})
        self.dist_thre = th.get("dist_thre", 0.03)
        self.angle_thre = th.get("angle_thre", 0.06)
        self.hold_duration = th.get("hold_duration", 1.0)

        # KFS 串口
        ser = self.graph_cfg.get("serial", {})
        self.serial_port = ser.get("port", "/dev/ttyUSB0")
        self.serial_baud = ser.get("baud_rate", 115200)
        self.send_header = ser.get("send_header", 0x0CAA)
        self.send_tail = ser.get("send_tail", 0xAC0A)

        # 底盘串口 (waypoint 执行)
        ch = self.graph_cfg.get("chassis_serial", {})
        self.chassis_port = ch.get("port", "/dev/ttyCHASSIS")
        self.chassis_baud = ch.get("baud_rate", 115200)
        self.chassis_timeout = ch.get("timeout", 1.0)
        self.chassis_send_bytes = ch.get("send_bytes", 17)
        self.chassis_send_header = ch.get("send_header", 0x0CAA)
        self.chassis_send_tail = ch.get("send_tail", 0xAC0A)
        self.chassis_reconnect_base = ch.get("reconnect_base_delay", 0.5)
        self.chassis_reconnect_max = ch.get("reconnect_max_delay", 3.0)

        # 偏置
        off = self.graph_cfg.get("offsets", {})
        self.offset_imu = off.get("imu", [0.011, -0.02329, 0.04412])
        self.lidar_xoffset = off.get("lidar_x", 0.28)
        self.lidar_yoffset = off.get("lidar_y", -0.04542)

        # 其他
        self.n_confirm = self.graph_cfg.get("n_confirm", 3)
        self.turn_90_penalty = self.graph_cfg.get("turn_90_penalty", 2.0)
        self.path_interp_res = self.graph_cfg.get("path_interp_res", 0.1)
        self.waypoint_timeout = self.graph_cfg.get("waypoint_timeout", 10.0)

        # ── A* 规划配置 ──
        # 整合到本项目内，不再依赖外部 path_planner 目录
        _pp_config = os.path.join(_SCRIPT_DIR, "config", "path_planner_config.yaml")
        try:
            self.astar_cfg = _load_yaml(_pp_config)
        except FileNotFoundError:
            self.astar_cfg = {}

        self.height_maps = self.astar_cfg.get("height_maps", {})
        self.test_scenes = self.astar_cfg.get("test_scenes", {})
        astar = self.astar_cfg.get("a_star", {})
        self.astar_params = astar
        rosd = self.astar_cfg.get("ros_defaults", {})
        self.default_start = tuple(rosd.get("start", [1, 5]))
        # 优先使用 height_maps 中阵营特定的 target_point_coord
        hm_side = self.height_maps.get(side, {})
        self.default_target = tuple(hm_side.get("target_point_coord",
                                                rosd.get("target", [2, 0])))

        # ── K 节点坐标 (经 zero_point 变换) ──
        self.known_coords = self._transform_coords()
        self.k_index = {"k1": 1, "k2": 2, "k3": 3, "k4": 4}

    def _transform_coords(self):
        zp = self.zero_point
        yaw = zp.get("yaw", 0.0)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        def _transform(local_x, local_y, local_z):
            tx = zp["x"] + local_x
            ty = zp["y"] + local_y
            gx = tx * cos_yaw - ty * sin_yaw
            gy = tx * sin_yaw + ty * cos_yaw
            gz = zp["z"] + local_z
            return [gx, gy, gz]

        coords = [_transform(0.0, 0.0, 0.0)]  # origin
        for key in ["k1", "k2", "k3", "k4"]:
            dx, dy, dz = self.relative_positions.get(key, [0, 0, 0])
            coords.append(_transform(dx, dy, dz))
        return coords

    def get_height_map(self):
        hm = self.height_maps.get(self.side, self.height_maps.get("blue", {}))
        return [list(row) for row in hm.get("data", [])]

    def get_test_scene(self, scene_id):
        s = self.test_scenes.get(int(scene_id), {})
        if not s:
            return None
        return {
            "start": tuple(s["start"]),
            "target": tuple(s["target"]),
            "fake": tuple(s["fake"]),
            "r1": [tuple(p) for p in s.get("r1", [])],
            "r2": [tuple(p) for p in s.get("r2", [])],
        }

    def get_id_coord(self, kfs_id):
        return self.id_to_coord.get(int(kfs_id))

    def get_tgt_facing(self, tgt_id, coord=None):
        """Return facing for a TGT ID. If multi-facing (list), match by coord index."""
        facing = self.tgt_facing.get(int(tgt_id))
        if facing is None:
            return 0.0
        if not isinstance(facing, list):
            return facing
        if coord is None:
            return facing[0]
        coords = self.id_to_coord.get(int(tgt_id))
        if coords is None or not isinstance(coords[0], list):
            return facing[0]
        for i, c in enumerate(coords):
            if abs(c[0] - float(coord[0])) < 0.01 and abs(c[1] - float(coord[1])) < 0.01:
                return facing[i] if i < len(facing) else facing[0]
        return facing[0]


# 全局实例
_cfg_instance = None


def get_config(side=None):
    global _cfg_instance
    if _cfg_instance is None or (side is not None and _cfg_instance.side != side):
        _cfg_instance = AppConfig(side)
    return _cfg_instance
