#!/usr/bin/env python3
"""Full pipeline test: sender → /R1_grap_pos → supplement → plan → waypoints → serial frames.

Scene 1:
  r1: [[1,3], [2,3], [2,1]]
  cell_id mapping: (1,3)→7(no map), (2,3)→8→ID8, (2,1)→14→ID14
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

import math
import struct
import numpy as np
from r1_config import Config
from r1_graph_mapper import GraphMapper, supplement_ids
from r1_graph.r1_id_sender import _cell_id, _coord_to_r1_id, load_kfs_id_map, _supplement_on_sender


def route_to_waypoints(route, tgt_ids):
    """Replicates planner's _route_to_waypoints."""
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


def height_byte(coord):
    z = coord[2]
    if abs(z - 1.0) < 0.01:
        return 0x01
    elif abs(z - 2.0) < 0.01:
        return 0x02
    elif abs(z - 3.0) < 0.01:
        return 0x03
    return 0x00


def fmt_serial_frame(wp_type, dx, dy, dtheta, height):
    data = bytearray(18)
    data[0:2] = struct.pack('>H', Config.SEND_HEADER)
    data[2] = 0x11
    if wp_type == 'translate':
        pack = struct.pack('<3f', dx * 100, dy * 100, 0.0)
    else:
        pack = struct.pack('<3f', 0.0, 0.0, dtheta)
    data[3:15] = pack
    data[15] = height
    data[16:18] = struct.pack('>H', Config.SEND_TAIL)
    return ' '.join(f'{b:02X}' for b in data)


def simulate_planner_flow(id1, id2):
    """Simulate full flow from IDs → supplement → plan → waypoints → frames."""
    # Validate
    for idx, rid in enumerate([id1, id2], start=1):
        if rid != 0 and rid not in Config.ID_TO_COORD:
            return f"Unknown ID{idx}={rid}"

    required = [i for i in [id1, id2] if i in Config.ID_TO_COORD]
    mapper = GraphMapper()

    id1_sup, id2_sup = supplement_ids(mapper, required)
    if id1_sup is None:
        return "Supplement failed"

    c1 = mapper.id_to_coord(id1_sup)
    c2 = mapper.id_to_coord(id2_sup)
    if c1 is None or c2 is None:
        return "ID to coord failed"

    nodes, adj = mapper.build_graph([c1, c2])
    start_idx = Config.K_INDEX[Config.START_K] + Config.K_GRAPH_OFFSET
    end_idx   = Config.K_INDEX[Config.END_K]   + Config.K_GRAPH_OFFSET

    route = mapper.find_path(nodes, adj, start_idx, end_idx)
    if not route:
        return "No route found"

    # Build tgt_ids
    tgt_ids = {}
    tgt_idx = 0
    for name, _ in route:
        if name.startswith('TGT'):
            tgt_idx += 1
            tgt_ids[name] = id1_sup if tgt_idx == 1 else id2_sup

    waypoints = route_to_waypoints(route, tgt_ids)
    cost = sum(np.linalg.norm(route[i][1][:2] - route[i + 1][1][:2])
               for i in range(len(route) - 1))

    return {
        'required': required,
        'supplemented': (id1_sup, id2_sup),
        'c1': c1, 'c2': c2,
        'route': route,
        'waypoints': waypoints,
        'cost': cost,
        'tgt_ids': tgt_ids,
    }


def main():
    print("=" * 60)
    print("  FULL PIPELINE TEST — Scene 1  (ROS topic mode)")
    print("=" * 60)

    # ── Scene 1 config ──
    r1_positions = [(0, 3), (2, 3), (2, 1)]
    side = "red"
    id_map = load_kfs_id_map(side)

    print(f"\nScene 1 R1 positions: {r1_positions}")
    print(f"Side: {side}")
    print(f"Cell ID formula: cell_id = 3*(5-y) + x")
    print(f"kfs_id_map ({side}): {id_map}")

    print(f"\nPosition -> cell_id -> R1 ID:")
    for x, y in r1_positions:
        cid = _cell_id(x, y)
        rid = _coord_to_r1_id(x, y, id_map)
        print(f"  ({x},{y}) -> cell_id={cid} -> {'ID ' + str(rid) if rid else 'NO MAPPING'}")

    scene_ids = []
    for x, y in r1_positions:
        rid = _coord_to_r1_id(x, y, id_map)
        if rid is not None and rid > 0 and rid not in scene_ids:
            scene_ids.append(rid)
    print(f"\nScene candidate IDs: {scene_ids}")

    print(f"\nGraph config:")
    print(f"  Scenario: {Config.SCENARIO}")
    print(f"  Start K: {Config.START_K}, End K: {Config.END_K}")
    print(f"  START_YAW: {Config.START_YAW:.4f}, TARGET_END_YAW: {Config.TARGET_END_YAW:.4f}")
    print(f"  TGT_FACING: {Config.TGT_FACING}")
    print(f"  Valid IDs: {sorted(Config.ID_TO_COORD.keys())}")

    test_cases = [
        ("(0,3)+(2,3)+(2,1) all crossed", [(0, 3), (2, 3), (2, 1)]),
        ("Only (0,3) crossed",            [(0, 3)]),
        ("(0,3)+(2,1) crossed",           [(0, 3), (2, 1)]),
        ("No positions crossed",          []),
    ]

    for label, crossed_positions in test_cases:
        print(f"\n{'─' * 60}")
        print(f"  CASE: {label}")
        print(f"{'─' * 60}")

        # ── Sender: select IDs, publish to /R1_grap_pos (simulated) ──
        crossed_ids = []
        for x, y in crossed_positions:
            rid = _coord_to_r1_id(x, y, id_map)
            if rid is not None and rid > 0 and rid not in crossed_ids:
                crossed_ids.append(rid)

        id_num = len(crossed_ids)
        if id_num >= 2:
            ids = crossed_ids[:2]
            how = f"use first 2"
        elif id_num == 1 and len(scene_ids) >= 2:
            ids = _supplement_on_sender(crossed_ids, scene_ids, verbose=False)
            how = "supplemented"
        elif id_num == 0 and len(scene_ids) >= 2:
            ids = _supplement_on_sender([], scene_ids, verbose=False)
            how = "supplemented from scene"
        else:
            ids = list(crossed_ids)
            while len(ids) < 2:
                ids.append(0x00)
            how = "fallback"

        ids = ids[:2]
        print(f"\n  [Sender] Crossed: {crossed_positions} -> IDs: {crossed_ids}")
        print(f"  [Sender] id_num={id_num} -> {how}: {ids}")
        print(f"  [Sender] publish /R1_grap_pos: Int32MultiArray({ids}) x 3")

        # ── Receiver: supplement + plan + waypoints ──
        result = simulate_planner_flow(ids[0], ids[1])
        if isinstance(result, str):
            print(f"  [Planner] ERROR: {result}")
            continue

        print(f"\n  [Planner] Valid required: {result['required']}")
        print(f"  [Planner] Supplemented:  [{result['supplemented'][0]}, {result['supplemented'][1]}]")
        print(f"  [Planner] TGT1(id={result['supplemented'][0]}): "
              f"({result['c1'][0]:.2f}, {result['c1'][1]:.2f}, {result['c1'][2]:.2f})")
        print(f"  [Planner] TGT2(id={result['supplemented'][1]}): "
              f"({result['c2'][0]:.2f}, {result['c2'][1]:.2f}, {result['c2'][2]:.2f})")
        print(f"  [Planner] Path: {' -> '.join(n for n, _ in result['route'])}")
        print(f"  [Planner] Cost: {result['cost']:.2f} m")

        # ── Waypoints + Serial frames ──
        wp = result['waypoints']
        route = result['route']
        print(f"\n  [Serial] Waypoints ({len(wp)}):")
        print(f"  {'#':3s} {'type':10s} {'name':6s} {'dx(m)':>8s} {'dy(m)':>8s} {'dtheta(rad)':>11s} {'h':4s}  FRAME (hex)")
        print(f"  {'-'*3} {'-'*10} {'-'*6} {'-'*8} {'-'*8} {'-'*11} {'-'*4}  {'-'*50}")

        for idx, (wp_type, dx, dy, dtheta, name) in enumerate(wp):
            is_tgt_wp = name.startswith('TGT')
            approach_h = 0x00
            frame_hex = fmt_serial_frame(wp_type, dx, dy, dtheta, approach_h)
            print(f"  {idx:3d} {wp_type:10s} {name:6s} {dx:8.2f} {dy:8.2f} {dtheta:11.4f} 0x{approach_h:02X}  {frame_hex}")

            if is_tgt_wp:
                for rname, rcoord in route:
                    if rname == name:
                        arr_h = height_byte(rcoord)
                        break
                arr_frame = fmt_serial_frame('translate', 0.0, 0.0, 0.0, arr_h)
                print(f"       {'(arrive)':10s} {'':6s} {'':>8s} {'':>8s} {'':>11s} 0x{arr_h:02X}  {arr_frame}")

    print(f"\n{'=' * 60}")
    print("  TEST COMPLETE")
    print(f"{'=' * 60}")


if __name__ == "__main__":
    main()
