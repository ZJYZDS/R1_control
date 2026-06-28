#!/usr/bin/env python3
"""Test the full receiving pipeline: frame bytes → parse → supplement → plan.

Simulates MapSecReceive frame parsing + GraphMapperNode planning.
No ROS or serial required.
"""

import sys, os
_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_dir, '..'))   # scripts/
sys.path.insert(0, _dir)                          # scripts/tools/ (for sibling imports)

import random
import numpy as np
from r1_config import Config
from r1_graph_mapper import GraphMapper, supplement_ids
from test_random_ids import path_to_actions


def simulate_receive_flow(id1, id2, mapper):
    """Simulate MapSecReceive._parse_frame + GraphMapperNode.cb_token_id.

    Returns: (valid_ids, supplemented_pair, route, actions, error_msg)
    """
    # ── Step 1: simulate _parse_frame — validate non-zero IDs ──
    for idx, rid in enumerate([id1, id2], start=1):
        if rid != 0 and rid not in Config.ID_TO_COORD:
            return None, None, None, None, f"Unknown ID{idx}={rid}"

    # ── Step 2: simulate cb_token_id — filter valid, supplement, plan ──
    ids = [id1, id2]
    required = [i for i in ids if i in Config.ID_TO_COORD]
    id_num = len(required)

    id1_sup, id2_sup = supplement_ids(mapper, required)
    if id1_sup is None:
        return required, None, None, None, "Supplement failed"

    c1 = mapper.id_to_coord(id1_sup)
    c2 = mapper.id_to_coord(id2_sup)
    if c1 is None or c2 is None:
        return required, (id1_sup, id2_sup), None, None, "ID→coord mapping failed"

    nodes, adj = mapper.build_graph([c1, c2])
    start_idx = Config.K_INDEX[Config.START_K] + Config.K_GRAPH_OFFSET
    end_idx   = Config.K_INDEX[Config.END_K]   + Config.K_GRAPH_OFFSET

    route = mapper.find_path(nodes, adj, start_idx, end_idx)
    if not route:
        return required, (id1_sup, id2_sup), None, None, "No route found"

    actions = path_to_actions(route)
    return required, (id1_sup, id2_sup), route, actions, None


def main():
    mapper = GraphMapper()
    valid_ids = sorted(Config.ID_TO_COORD.keys())
    zp = Config.KNOWN_COORDS[0]
    print(f"Scenario: {Config.SCENARIO}")
    print(f"Valid IDs: {valid_ids}")
    print(f"Start K: {Config.START_K}, End K: {Config.END_K}")
    print(f"ref_yaw: {Config.REF_YAW:.2f}, dir_correct: {Config.DIR_CORRECT}")
    print(f"Zero point: ({zp[0]:.2f}, {zp[1]:.2f}, {zp[2]:.2f})")

    # Build test frames covering all cases
    test_frames = [
        # (label, id1, id2)
        ("2 valid IDs",                random.choice(valid_ids), random.choice(valid_ids)),
        ("1 valid ID  + 0x00 (id_num=1)", random.choice(valid_ids), 0x00),
        ("0x00 + 1 valid ID (id_num=1)", 0x00, random.choice(valid_ids)),
        ("0x00 + 0x00 (id_num=0)",     0x00, 0x00),
        ("invalid ID → should fail",   0xFF, 0x00),
    ]

    for label, id1, id2 in test_frames:
        frame_hex = f"5A {id1:02X} {id2:02X} A5"
        print(f"\n{'='*60}")
        print(f"Frame: [{frame_hex}]  —  {label}")
        print(f"{'='*60}")

        required, pair, route, actions, error = simulate_receive_flow(id1, id2, mapper)

        if error:
            print(f"  ERROR: {error}")
            continue

        print(f"  Raw IDs: [{id1}, {id2}]")
        print(f"  Valid required: {required}")
        print(f"  Supplemented → [{pair[0]}, {pair[1]}]")

        c1 = mapper.id_to_coord(pair[0])
        c2 = mapper.id_to_coord(pair[1])
        print(f"  TGT1(id={pair[0]}) = ({c1[0]:.2f}, {c1[1]:.2f}, {c1[2]:.2f})")
        print(f"  TGT2(id={pair[1]}) = ({c2[0]:.2f}, {c2[1]:.2f}, {c2[2]:.2f})")

        print(f"  Path: {' → '.join(n for n, _ in route)}")
        cost = sum(np.linalg.norm(route[i][1][:2] - route[i + 1][1][:2])
                   for i in range(len(route) - 1))
        print(f"  Cost: {cost:.2f} m")

        print(f"  Actions ({len(actions)}): [")
        for act in actions:
            print(f"    [{act[0]:6.2f}, {act[1]:6.2f}, {act[2]:6.2f}],")
        print(f"  ]")


if __name__ == "__main__":
    main()
