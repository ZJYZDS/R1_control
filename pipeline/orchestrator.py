"""R1 总管道 — 从 KFS 网格到航点列表。

串联: KFS → A* → graph → waypoints, 无 ROS 通信。
"""

import json
import math
import os
import time
import numpy as np

from lib.config import get_config
from pipeline.astar_planner import astar_plan, select_r1_ids, get_crossed_r1_positions
from pipeline.graph_planner import GraphMapper, supplement_ids, route_to_waypoints
from pipeline.kfs_reader import KfsSerialReader
from pipeline.commands_generator import generate_commands, commands_to_binary, fmt_hex
from lib.kfs_id_map import load_kfs_id_map


DEFAULT_WAYPOINTS_FILE = "/tmp/pipeline_waypoints.json"


DEFAULT_REQUIRED_R2 = 2
DEFAULT_REQUIRED_R1 = 2


def route_to_absolute_waypoints(route, tgt_ids_dict, cfg):
    """将图规划 route 转为 zero_point 坐标系下的绝对航点.

    route: [(name, np.array([x, y, z])), ...]
    返回: [{"x": ..., "y": ..., "theta": ..., "name": ..., "height"?}, ...]

    theta = (heading_rad - ref_yaw) * dir_correct
    其中 heading_rad 以 +y 轴为 0 rad（与 r1_graph_config.yaml 约定一致）.
    """
    ref_yaw = cfg.ref_yaw
    dir_c = cfg.dir_correct

    result = []
    for i, (name, coord) in enumerate(route):
        x, y, z = float(coord[0]), float(coord[1]), float(coord[2])

        # ── 计算该节点的目标朝向 (map heading, +y=0) ──
        if name.startswith("TGT"):
            tgt_id = tgt_ids_dict.get(name)
            if tgt_id is not None and tgt_id in cfg.tgt_facing:
                heading_rad = cfg.tgt_facing[tgt_id]
            else:
                heading_rad = 0.0
        elif i == len(route) - 1:
            # 最后一个节点: 直接用 target_end_yaw (已经是 theta)
            heading_rad = None
        else:
            next_name, next_coord = route[i + 1]
            nx, ny = float(next_coord[0]), float(next_coord[1])
            if next_name.startswith("TGT"):
                tgt_id = tgt_ids_dict.get(next_name)
                if tgt_id is not None and tgt_id in cfg.tgt_facing:
                    # K→TGT: 朝向用 TGT 的 tgt_facing
                    heading_rad = cfg.tgt_facing[tgt_id]
                else:
                    heading_rad = math.atan2(nx - x, ny - y)
            else:
                # K→K: 计算两点间方向 (atan2(dx, dy), 0=+y 轴)
                heading_rad = math.atan2(nx - x, ny - y)

        # ── 转为 theta ──
        if heading_rad is not None:
            theta = (heading_rad - ref_yaw) * dir_c
        else:
            theta = cfg.target_end_yaw  # 已经是 theta
        theta = math.atan2(math.sin(theta), math.cos(theta))  # 归一化到 [-pi, pi]

        wp = {"x": x, "y": y, "theta": theta, "name": name}
        if name.startswith("TGT"):
            wp["height"] = int(z)
        result.append(wp)

    return result


def save_waypoints_json(absolute_waypoints, filepath=DEFAULT_WAYPOINTS_FILE):
    """将绝对航点列表保存为 pao.py 可读的 JSON 文件.

    输入: [{"x": ..., "y": ..., "theta": ..., "name": ..., "height": ...?}, ...]
    输出 JSON: 同格式
    """
    os.makedirs(os.path.dirname(filepath) or ".", exist_ok=True)
    with open(filepath, "w") as f:
        json.dump(absolute_waypoints, f, indent=2)
    print(f"[Pipeline] Waypoints ({len(absolute_waypoints)} 个, 绝对坐标) 已保存 → {filepath}")


def process_grid(grid_4x3, side="blue", verbose=True):
    """处理一个 4×3 KFS 网格 → 输出航点列表.

    Args:
        grid_4x3: 4×3 numpy array (uint8: 0=空, 1=R1, 2=R2, 3=Fake)
        side:     "red" 或 "blue"
        verbose:  是否打印详细信息

    Returns:
        dict with: waypoints, route_nodes, tgt_coords, r1_ids, astar_path, or error
    """
    cfg = get_config(side)
    base_height_map = cfg.get_height_map()
    start = cfg.default_start
    target = cfg.default_target

    # ── 1. 构建 working height map: 基础高度 + KFS 网格/10 ──
    kfs_cfg = cfg.astar_cfg.get("kfs_scoring", {})
    R2_FRAC = kfs_cfg.get("r2_frac", 0.1)
    R1_FRAC = kfs_cfg.get("r1_frac", 0.2)
    FAKE_VAL = kfs_cfg.get("fake_val", 4.0)
    OBSTACLE_VAL = cfg.astar_params.get("obstacle_val", 4)

    working_map = np.array(base_height_map, dtype=float)

    r1_positions = []
    r2_positions = []
    fake_position = None

    for r in range(4):
        for c in range(3):
            val = int(grid_4x3[r, c])
            ay = r + 1  # height map row y = 1..4
            if val == 0:
                continue
            elif val == 3:  # Fake → 设为障碍值
                working_map[ay][c] = OBSTACLE_VAL
                fake_position = (c, ay)
            elif val == 2:  # R2 → +0.1 (r2_frac)
                working_map[ay][c] += R2_FRAC
                r2_positions.append((c, ay))
            elif val == 1:  # R1 → +0.2 (r1_frac)
                working_map[ay][c] += R1_FRAC
                r1_positions.append((c, ay))

    if verbose:
        print(f"[Pipeline] Grid: R1={len(r1_positions)}, R2={len(r2_positions)}, "
              f"Fake={fake_position}")
        print(f"  R1: {r1_positions}")
        print(f"  R2: {r2_positions}")
        print(f"  Side={side}, Start={start}, Target={target}")
        print(f"  Base height map:\n{np.array(base_height_map)}")
        print(f"  Working map (with KFS):\n{working_map}")

    # ── 2. A* 规划 (使用叠加 KFS 后的 working_map) ──
    entry_row = 4 if start[1] == 5 else start[1]
    path, _, _, r2_mask, r1_mask, cost = astar_plan(
        working_map, start, target,
        fake=fake_position, r1=r1_positions, r2=r2_positions,
        required_r2=DEFAULT_REQUIRED_R2, required_r1=DEFAULT_REQUIRED_R1,
        entry_row=entry_row,
    )

    if not path:
        return {"error": "A* NO PATH FOUND"}

    if verbose:
        print(f"\n[Pipeline] A* 规划成功! 路径长度={len(path)}, 代价={cost:.1f}")
        print(f"  Path: {path}")

    # ── 3. 生成 /path/commands ──
    cmd_str = generate_commands(path, r2_positions, r1_positions,
                                DEFAULT_REQUIRED_R2, base_height_map)
    binary_cmds = commands_to_binary(cmd_str)
    if verbose:
        print(f"\n[Pipeline] /path/commands:")
        print(f"  {cmd_str}")
        print(f"  Binary ({len(binary_cmds)} bytes): {fmt_hex(binary_cmds)}")

    # ── 4. R1 ID 选择 ──
    crossed_r1 = get_crossed_r1_positions(path, r1_positions, r1_mask, len(r2_positions))
    if verbose:
        print(f"\n[Pipeline] 跨越 R1: {crossed_r1}")

    kfs_id_map = load_kfs_id_map(side)
    r1_ids = select_r1_ids(crossed_r1, r1_positions, kfs_id_map)

    if verbose:
        print(f"[Pipeline] 初步 R1 IDs: {r1_ids}")
        print(f"  KFS ID map (cell_id→kfs_id): {kfs_id_map}")

    # ── 5. ID 补充 ──
    mapper = GraphMapper(cfg)
    valid_ids = sorted(cfg.id_to_coord.keys())
    id1, id2 = supplement_ids(mapper, r1_ids[:2], valid_ids)
    if id1 is None or id2 is None:
        return {"error": f"ID supplementation failed: ids={r1_ids}"}

    final_ids = [id1, id2]
    if verbose:
        print(f"[Pipeline] 最终 IDs (补充后): {final_ids}")

    # ── 6. 图规划 ──
    c1 = mapper.id_to_coord(id1)
    c2 = mapper.id_to_coord(id2)
    if c1 is None or c2 is None:
        return {"error": f"ID→coord failed"}

    tgt_ids_dict = {"TGT1": id1, "TGT2": id2}

    if verbose:
        print(f"\n[Pipeline] 图规划:")
        print(f"  TGT1(id={id1})=({c1[0]:.3f},{c1[1]:.3f},{c1[2]:.3f})")
        print(f"  TGT2(id={id2})=({c2[0]:.3f},{c2[1]:.3f},{c2[2]:.3f})")
        print(f"  Start: {cfg.start_k}, End: {cfg.end_k}")

    nodes, adj = mapper.build_graph([c1, c2])
    start_idx = cfg.k_index[cfg.start_k] + 1
    end_idx = cfg.k_index[cfg.end_k] + 1

    route = mapper.find_path(nodes, adj, start_idx, end_idx)
    if not route:
        return {"error": "No graph route found"}

    route_nodes = [n for n, _ in route]
    wps = route_to_waypoints(route, tgt_ids_dict)

    # 生成绝对坐标航点 (zero_point 框架), 保存供 pao.py 读取
    abs_wps = route_to_absolute_waypoints(route, tgt_ids_dict, cfg)
    save_waypoints_json(abs_wps)

    if verbose:
        print(f"\n[Pipeline] 图规划结果:")
        print(f"  Route: {route_nodes}")
        print(f"  Waypoints ({len(wps)}):")
        for i, (wp_type, dx, dy, dyaw, name) in enumerate(wps):
            h = ""
            if name.startswith("TGT"):
                tgt_id = tgt_ids_dict.get(name)
                if tgt_id is not None:
                    coord = mapper.id_to_coord(tgt_id)
                    h = f" height={int(coord[2])}" if coord is not None else ""
            print(f"    {i}: [{wp_type}] ({dx:+.3f}, {dy:+.3f}, {dyaw:+.3f}) {name}{h}")

    return {
        "waypoints": wps,
        "absolute_waypoints": abs_wps,
        "route_nodes": route_nodes,
        "tgt_coords": {
            "TGT1": (float(c1[0]), float(c1[1]), float(c1[2])),
            "TGT2": (float(c2[0]), float(c2[1]), float(c2[2])),
        },
        "r1_ids": final_ids,
        "astar_path": path,
        "cost": cost,
        "commands": cmd_str,
        "binary_commands": binary_cmds,
        "start_k": cfg.start_k,
        "end_k": cfg.end_k,
    }


def process_test_scene(scene_id, side="blue", verbose=True):
    """使用测试场景运行管道."""
    cfg = get_config(side)
    scene = cfg.get_test_scene(scene_id)
    if scene is None:
        return {"error": f"Scene {scene_id} not found"}

    # 将场景数据转为 4×3 网格
    grid = np.zeros((4, 3), dtype=np.int32)
    for x, y in scene["r2"]:
        if 1 <= y <= 4:
            grid[y - 1, x] = 2
    for x, y in scene["r1"]:
        if 1 <= y <= 4:
            grid[y - 1, x] = 1
    fx, fy = scene["fake"]
    if 1 <= fy <= 4:
        grid[fy - 1, fx] = 3

    if verbose:
        print(f"[Test] Scene {scene_id} ({side}):")
        print(f"  Start={scene['start']}, Target={scene['target']}")
        print(f"  Grid:\n{grid}")

    return process_grid(grid, side, verbose)


def run_serial_pipeline(serial_port, side="blue"):
    """从串口持续读取 KFS 帧并运行管道."""
    def on_grid(grid):
        result = process_grid(grid, side, verbose=True)
        if "error" in result:
            print(f"[ERROR] {result['error']}")
        else:
            print(f"\n[RESULT] 航点数: {len(result['waypoints'])}")
            print(f"  Route: {result['route_nodes']}")
            for i, (wp_type, dx, dy, dyaw, name) in enumerate(result['waypoints']):
                print(f"    {i}: [{wp_type}] ({dx:+.3f}, {dy:+.3f}, "
                      f"{dyaw:+.3f}) {name}")

    reader = KfsSerialReader(serial_port, on_grid=on_grid)
    reader.start()
    print(f"[Pipeline] 串口管道已启动: {serial_port}, side={side}")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        reader.stop()
        print("[Pipeline] 已停止")
