#!/usr/bin/env python3
"""Test GraphMapper with random ID arrays + simulated serial frame output.

Uses GraphMapper + route_to_waypoints logic — no ROS required.
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

import math
import struct
import numpy as np
import random
from r1_config import Config
from r1_graph_mapper import GraphMapper, supplement_ids


def route_to_waypoints(route, tgt_ids):
    """Replicates planner's _route_to_waypoints. Returns list of (type, dx, dy, dtheta, name)."""
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
    """Map z-coordinate to height byte: 1.0→0x01, 2.0→0x02, 3.0→0x03, else 0x00."""
    z = coord[2]
    if abs(z - 1.0) < 0.01:
        return 0x01
    elif abs(z - 2.0) < 0.01:
        return 0x02
    elif abs(z - 3.0) < 0.01:
        return 0x03
    return 0x00


def fmt_serial_frame(wp_type, dx, dy, dtheta, height):
    """Build and format a serial frame for display."""
    data = bytearray(18)
    data[0:2] = struct.pack('>H', Config.SEND_HEADER)
    data[2] = 0x11  # CMD
    # dtheta in radians, dx/dy in cm (from meters)
    if wp_type == 'translate':
        pack = struct.pack('<3f', dx * 100, dy * 100, 0.0)
    else:
        pack = struct.pack('<3f', 0.0, 0.0, dtheta)
    data[3:15] = pack
    data[15] = height
    data[16:18] = struct.pack('>H', Config.SEND_TAIL)
    return ' '.join(f'{b:02X}' for b in data)


def main():
    mapper = GraphMapper()

    valid_ids = [3, 4, 5, 6, 8, 9, 11, 12, 13, 14]
    zp = Config.KNOWN_COORDS[0]
    print(f"Scenario: {Config.SCENARIO}")
    print(f"Valid IDs: {valid_ids}")
    print(f"Zero point: ({zp[0]:.2f}, {zp[1]:.2f}, {zp[2]:.2f})")
    print(f"Start K: {Config.START_K}, End K: {Config.END_K}")
    print(f"START_YAW: {Config.START_YAW:.4f}, TARGET_END_YAW: {Config.TARGET_END_YAW:.4f}")
    print(f"ref_yaw: {Config.REF_YAW:.2f}, dir_correct: {Config.DIR_CORRECT}")
    print(f"TGT_FACING: {Config.TGT_FACING}")

    test_cases = [
        ("id_num=0", []),
        ("id_num=1", [random.choice(valid_ids)]),
        ("id_num=2", random.sample(valid_ids, 2)),
    ]

    for label, required in test_cases:
        id1, id2 = supplement_ids(mapper, required, valid_ids)

        print(f"\n{'='*60}")
        print(f"Test: {label}")
        print(f"  Required: {required}  ->  Supplemented: [{id1}, {id2}]")

        c1 = mapper.id_to_coord(id1)
        c2 = mapper.id_to_coord(id2)
        if c1 is None or c2 is None:
            print(f"  FAILED: ID->coord mapping failed")
            continue

        print(f"  TGT1(id={id1}) = ({c1[0]:.2f}, {c1[1]:.2f}, {c1[2]:.2f})")
        print(f"  TGT2(id={id2}) = ({c2[0]:.2f}, {c2[1]:.2f}, {c2[2]:.2f})")

        nodes, adj = mapper.build_graph([c1, c2])

        start_idx = Config.K_INDEX[Config.START_K] + Config.K_GRAPH_OFFSET
        end_idx   = Config.K_INDEX[Config.END_K]   + Config.K_GRAPH_OFFSET

        route = mapper.find_path(nodes, adj, start_idx, end_idx)
        if not route:
            print("  No route found")
            continue

        path_str = ' -> '.join(n for n, _ in route)
        print(f"  Path: {path_str}")

        cost = sum(np.linalg.norm(route[i][1][:2] - route[i + 1][1][:2])
                   for i in range(len(route) - 1))
        print(f"  Cost: {cost:.2f} m")

        # Build tgt_ids dict (TGT1, TGT2 -> id)
        tgt_ids = {}
        tgt_idx = 0
        for name, coord in route:
            if name.startswith('TGT'):
                tgt_idx += 1
                if tgt_idx == 1:
                    tgt_ids[name] = id1
                else:
                    tgt_ids[name] = id2

        waypoints = route_to_waypoints(route, tgt_ids)
        print(f"\n  Waypoints ({len(waypoints)}):")
        print(f"  {'#':3s} {'type':10s} {'name':6s} {'dx(m)':>8s} {'dy(m)':>8s} {'dtheta(rad)':>11s} {'h':4s}  SERIAL FRAME (hex)")
        print(f"  {'-'*3} {'-'*10} {'-'*6} {'-'*8} {'-'*8} {'-'*11} {'-'*4}  {'-'*50}")

        for idx, (wp_type, dx, dy, dtheta, name) in enumerate(waypoints):
            is_tgt_wp = name.startswith('TGT')
            # During approach: height=0x00. Only set at TGT arrival.
            approach_h = 0x00
            frame_hex = fmt_serial_frame(wp_type, dx, dy, dtheta, approach_h)
            print(f"  {idx:3d} {wp_type:10s} {name:6s} {dx:8.2f} {dy:8.2f} {dtheta:11.4f} 0x{approach_h:02X}  {frame_hex}")

            # If TGT waypoint, show arrival frame with height byte
            if is_tgt_wp:
                for rname, rcoord in route:
                    if rname == name:
                        arr_h = height_byte(rcoord)
                        break
                arr_frame = fmt_serial_frame('translate', 0.0, 0.0, 0.0, arr_h)
                print(f"       {'(arrive)':10s} {'':6s} {'':>8s} {'':>8s} {'':>11s} 0x{arr_h:02X}  {arr_frame}")

        print(f"\n  Graph ({len(nodes)} nodes, {sum(len(v) for v in adj.values()) // 2} edges):")
        for name, coord in nodes:
            print(f"    {name:6s}  ({coord[0]:7.2f}, {coord[1]:7.2f}, {coord[2]:6.2f})")
        for u, nbrs in adj.items():
            for v, edge_cost in nbrs.items():
                if u < v:
                    print(f"    {nodes[u][0]:6s} -> {nodes[v][0]:6s}  cost={edge_cost:.2f}")


if __name__ == "__main__":
    main()
