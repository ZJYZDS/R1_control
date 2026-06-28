"""R1 ID selection and transmission.

Given crossed R1 positions and the full scene R1 list, selects the optimal
2 R1 IDs (supplementing from scene candidates when needed) and publishes
them via ROS topic /R1_grap_pos (preferred) or serial as fallback.
"""

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from common.serial_utils import send_via_serial
from r1_graph.mapper import GraphMapper, supplement_ids


# ── Config loading (reads from path_planner's config.yaml) ──

_SCRIPT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_PATH_PLANNER_CONFIG = os.path.join(_SCRIPT_DIR, "config", "path_planner_config.yaml")


def _load_planner_yaml():
    import yaml
    with open(_PATH_PLANNER_CONFIG, 'r') as f:
        return yaml.safe_load(f) or {}


def _r1_serial_cfg(key, default):
    cfg = _load_planner_yaml().get("r1_serial", {})
    return cfg.get(key, default)


def _cell_id(x, y):
    """Convert A* grid coordinate to linear cell ID."""
    return (5 - y) * 3 + x


def load_kfs_id_map(side="blue"):
    """Load cell_id → R1 ID mapping for the given side."""
    raw = _load_planner_yaml().get("kfs_id_map", {}).get(side, {})
    return {int(k): int(v) for k, v in raw.items()}


def _coord_to_r1_id(x, y, id_map):
    """(x, y) → cell_id → R1 ID.  Returns None if no mapping exists."""
    return id_map.get(_cell_id(x, y))


def _supplement_on_sender(required_ids, scene_ids, verbose=True):
    """Supplement IDs to exactly 2 using only scene R1 candidates."""
    mapper = GraphMapper()
    id1, id2 = supplement_ids(mapper, required_ids, valid_ids=scene_ids)
    if id1 is None:
        result = list(required_ids)
        while len(result) < 2:
            result.append(0x00)
        return result
    return [id1, id2]


def _publish_via_ros(ids, n_confirm=3):
    """Publish [id1, id2] to /R1_grap_pos with confirmation repeats."""
    import rospy
    from std_msgs.msg import Int32MultiArray

    try:
        rospy.get_master().getPid()
    except Exception:
        raise RuntimeError("ROS master not reachable")

    if not rospy.core.get_node_uri():
        rospy.init_node("r1_id_sender", anonymous=True)

    pub = rospy.Publisher("/R1_grap_pos", Int32MultiArray, queue_size=10, latch=True)
    time.sleep(0.1)

    msg = Int32MultiArray(data=ids)
    for i in range(n_confirm):
        pub.publish(msg)
        if i < n_confirm - 1:
            time.sleep(0.05)
    return True


def send_r1_ids(crossed_r1_positions, all_r1_positions, side="blue",
                serial_port=None, serial_baud=None, verbose=True):
    """Select optimal 2 R1 IDs from scene context and publish via ROS.

    Args:
        crossed_r1_positions: R1 positions crossed in A* path (from r1_mask)
        all_r1_positions:     ALL R1 positions in the current scene
        side:                 "blue" | "red"
        serial_port:          (deprecated) kept for API compatibility
        serial_baud:          (deprecated) kept for API compatibility
        verbose:              Print diagnostic output

    Publishes Int32MultiArray [id1, id2] to /R1_grap_pos (× N_CONFIRM).
    """
    id_map = load_kfs_id_map(side)

    # Crossed R1 → IDs  (via cell_id = 3*(5-y)+x)
    crossed_ids = []
    for x, y in crossed_r1_positions:
        rid = _coord_to_r1_id(x, y, id_map)
        if rid is not None and rid > 0 and rid not in crossed_ids:
            crossed_ids.append(rid)

    # All scene R1 → IDs (candidate pool)
    scene_ids = []
    for x, y in all_r1_positions:
        rid = _coord_to_r1_id(x, y, id_map)
        if rid is not None and rid > 0 and rid not in scene_ids:
            scene_ids.append(rid)

    id_num = len(crossed_ids)

    if id_num >= 2:
        ids = crossed_ids[:2]
    elif id_num == 1 and len(scene_ids) >= 2:
        ids = _supplement_on_sender(crossed_ids, scene_ids, verbose=verbose)
    elif id_num == 0 and len(scene_ids) >= 2:
        ids = _supplement_on_sender([], scene_ids, verbose=verbose)
    else:
        ids = list(crossed_ids)
        while len(ids) < 2:
            ids.append(0x00)

    ids = ids[:2]

    if verbose:
        print(f"[R1-sender] Crossed R1: {crossed_r1_positions} → IDs: {crossed_ids}")
        print(f"[R1-sender] Scene R1 candidates: {scene_ids}")
        print(f"[R1-sender] Final IDs: {ids}")

    # Try ROS first
    try:
        import rospy
        rospy.init_node("r1_id_sender", anonymous=True, disable_signals=True)
        _publish_via_ros(ids)
        if verbose:
            print(f"[R1-sender] Published /R1_grap_pos: {ids}")
        return ids
    except (ImportError, Exception) as e:
        if verbose:
            print(f"[R1-sender] ROS unavailable ({e}), falling back to serial")

    # Serial fallback (deprecated — kept for backwards compatibility)
    header = _r1_serial_cfg("frame_header", 0x5A)
    tail   = _r1_serial_cfg("frame_tail", 0xA5)

    frame = bytearray([header, ids[0], ids[1], tail])

    if verbose:
        print(f"[R1-sender] Frame: {' '.join(f'{b:02X}' for b in frame)}")

    if serial_port is None:
        try:
            import rospy
            serial_port = rospy.get_param("~r1_serial_port",
                                          _r1_serial_cfg("port", "/dev/ttyR1"))
        except Exception:
            serial_port = _r1_serial_cfg("port", "/dev/ttyR1")
    if serial_baud is None:
        serial_baud = _r1_serial_cfg("baudrate", 115200)

    send_via_serial(frame, serial_port, serial_baud, verbose=verbose)
    return ids
