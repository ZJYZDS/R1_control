"""Configuration loader for R1 graph planning.

Reads r1_graph_config.yaml and exposes scenario-specific parameters.
"""

import os
import math
import yaml


def _load_config(yaml_path=None):
    if yaml_path is None:
        yaml_path = os.path.join(os.path.dirname(__file__), "..", "..",
                                 "config", "r1_graph_config.yaml")
    with open(yaml_path, 'r') as f:
        return yaml.safe_load(f)


_cfg = _load_config()

_SCENARIO = os.environ.get("R1_SCENARIO", _cfg.get("scenario", "red"))
_sc = _cfg.get(_SCENARIO, _cfg.get("red", {}))


def _eval_num(val, **ctx):
    """Evaluate a YAML value as a numeric expression if it's a string.

    Supports expressions like '(0.0 - ref_yaw) * dir_correct' with
    the given context variables.  Full-width parens are normalized.
    """
    if isinstance(val, list):
        return [_eval_num(v, **ctx) for v in val]
    if isinstance(val, (int, float)):
        return float(val)
    if isinstance(val, str):
        s = val.replace('（', '(').replace('）', ')')
        s = s.replace('＊', '*').replace('－', '-')
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


_zero = _eval_num(_sc.get("ref_yaw", 0.0))
_dir  = int(_sc.get("dir_correct", 1))
_angle_ctx = {"ref_yaw": _zero, "dir_correct": _dir}


def _transform_coord(local_x, local_y, local_z, zp):
    """Apply zero_point offset (dx, dy, yaw) to a local coordinate."""
    yaw = zp['yaw']
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    tx = zp['x'] + local_x
    ty = zp['y'] + local_y
    gx = tx * cos_yaw - ty * sin_yaw
    gy = tx * sin_yaw + ty * cos_yaw
    gz = zp['z'] + local_z
    return (gx, gy, gz)


def _load_known_coords():
    zp = _sc['zero_point']
    coords = [_transform_coord(0.0, 0.0, 0.0, zp)]
    for key in ['k1', 'k2', 'k3', 'k4']:
        dx, dy, dz = _sc['relative_positions'][key]
        coords.append(_transform_coord(dx, dy, dz, zp))
    return coords


def _load_id_coords():
    zp = _sc['zero_point']
    result = {}
    for k, v in _sc['id_to_coord'].items():
        # 货架可能有多个坐标, 取第一个
        coord = v[0] if isinstance(v[0], list) else v
        result[int(k)] = list(_transform_coord(coord[0], coord[1], coord[2], zp))
    return result


class Config:
    SCENARIO = _SCENARIO

    START_K = _sc.get("start_k", "k3")
    END_K = _sc.get("end_k", "k4")

    REF_YAW = _zero
    DIR_CORRECT = _dir
    START_YAW = _eval_num(_sc.get("start_yaw", 0.0), **_angle_ctx)
    TARGET_END_YAW = _eval_num(_sc.get("target_end_yaw", 0.0), **_angle_ctx)

    _raw_facing = _sc.get("tgt_facing", {})
    TGT_FACING = {int(k): _eval_num(v, **_angle_ctx)
                  for k, v in _raw_facing.items()}

    DIST_THRE = _cfg['thresholds']['dist_thre']
    ANGLE_THRE = _cfg['thresholds']['angle_thre']

    SERIAL_PORT = _cfg['serial']['port']
    BAUD_RATE = _cfg['serial']['baud_rate']
    SEND_BYTES = _cfg['serial']['send_bytes']
    SEND_HEADER = _cfg['serial']['send_header']
    SEND_TAIL = _cfg['serial']['send_tail']
    RECEIVE_BYTES = _cfg['serial']['receive_bytes']
    RECEIVE_HEADER = _cfg['serial']['receive_header']
    RECEIVE_TAIL = _cfg['serial']['receive_tail']
    RECEIVE_OK = _cfg['serial'].get('receive_ok', 0x01)

    SERIAL_TIMEOUT = _cfg['serial']['timeout']
    RECONNECT_BASE_DELAY = _cfg['reconnect']['base_delay']
    RECONNECT_MAX_DELAY = _cfg['reconnect']['max_delay']
    WAYPOINT_TIMEOUT = _cfg['waypoint_timeout']

    N_CONFIRM = _cfg['n_confirm']

    TURN_90_PENALTY = _cfg['turn_90_penalty']

    PATH_INTERP_RES = _cfg['path_interp_res']

    ID_TO_COORD = _load_id_coords()

    KNOWN_COORDS = _load_known_coords()

    K_INDEX = {"k1": 1, "k2": 2, "k3": 3, "k4": 4}
    K_GRAPH_OFFSET = 1

    PATH_PUB_RATE = _cfg['path_pub_rate']

