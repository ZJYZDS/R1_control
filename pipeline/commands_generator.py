"""A* 路径 → /path/commands 命令生成 + 二进制映射.

无 ROS 依赖。核心逻辑移植自 test_A_star_no_r2_cost.py。
"""

# ========== 动作→二进制映射 (来自 map.py) ==========

ACTION_MAP = {
    "move_up_1":              b'\xA1',
    "move_down_1":            b'\xA2',
    "move_up_2":              b'\xA3',
    "move_down_2":            b'\xA4',
    "take_front_r2":          b'\xAA',
    "take_front_r2_up200":    b'\xD1',
    "take_front_r2_up400":    b'\xD2',
    "take_front_r2_down200":  b'\xD3',
    "move_left":              b'\xB1',
    "move_right":             b'\xB2',
    "face_up":                b'\xC1',
    "face_left":              b'\xC2',
    "face_right":             b'\xC3',
}

NEIGHBOR_OFFSETS = [(0, -1), (0, 1), (-1, 0), (1, 0)]


def _dir_name(dx, dy):
    if (dx, dy) == (0, -1):
        return "face_up"
    if (dx, dy) == (0, 1):
        return "face_down"
    if (dx, dy) == (-1, 0):
        return "face_left"
    if (dx, dy) == (1, 0):
        return "face_right"
    return "face_up"


def _step_desc(cur, nxt):
    dx = nxt[0] - cur[0]
    dy = nxt[1] - cur[1]
    if dx == 1 and dy == 0:
        return "right"
    if dx == -1 and dy == 0:
        return "left"
    if dx == 0 and dy == -1:
        return "up"
    if dx == 0 and dy == 1:
        return "down"
    raise ValueError(f"path error: from {cur} -> {nxt}")


def generate_commands(path, r2_list, r1_list, required_r2, height_map):
    """沿 A* 路径生成 /path/commands 格式的命令字符串.

    Args:
        path:        A* 路径 [(x,y), ...]
        r2_list:     R2 位置列表 [(x,y), ...]
        r1_list:     R1 位置列表 [(x,y), ...]
        required_r2: 需收集的 R2 数量
        height_map:  高度地图 (2D numpy array)

    Returns:
        命令字符串, 如 "face_right+take_front_r2+move_right"
    """
    kfs_list = [(x, y, 'R2') for x, y in r2_list] + [(x, y, 'R1') for x, y in r1_list]
    pos_to_idx = {(x, y): i for i, (x, y, _) in enumerate(kfs_list)}

    sim_r2 = 0
    all_actions = []

    for i in range(len(path)):
        x, y = path[i]

        if i == len(path) - 1:
            break

        nxt = path[i + 1]
        direction = _step_desc((x, y), nxt)

        face_dx, face_dy = nxt[0] - x, nxt[1] - y

        h_from = int(height_map[y][x])
        h_to = int(height_map[nxt[1]][nxt[0]])
        h_diff = h_to - h_from

        new_face_dx, new_face_dy = face_dx, face_dy
        take_action = None
        turned_for_r2 = False

        for adx, ady in NEIGHBOR_OFFSETS:
            kx, ky = x + adx, y + ady
            if (kx, ky) not in pos_to_idx:
                continue
            kidx = pos_to_idx[(kx, ky)]
            if kfs_list[kidx][2] == 'R2' and not ((sim_r2 >> kidx) & 1) \
                    and bin(sim_r2).count('1') < required_r2:
                sim_r2 |= (1 << kidx)
                r2_h = int(height_map[ky][kx])
                r2_h_diff = r2_h - h_from
                if r2_h_diff == 2:
                    take_str = "take_front_r2_up400"
                elif r2_h_diff == 1:
                    take_str = "take_front_r2_up200"
                elif r2_h_diff == -1:
                    take_str = "take_front_r2_down200"
                elif r2_h_diff == -2:
                    take_str = "take_front_r2_down200"
                else:
                    take_str = "take_front_r2"

                if (adx, ady) == (face_dx, face_dy):
                    take_action = take_str
                elif (adx, ady) == (face_dy, -face_dx):
                    new_face_dx, new_face_dy = face_dy, -face_dx
                    take_action = take_str
                    turned_for_r2 = True
                elif (adx, ady) == (-face_dy, face_dx):
                    new_face_dx, new_face_dy = -face_dy, face_dx
                    take_action = take_str
                    turned_for_r2 = True

        face_action = _dir_name(new_face_dx, new_face_dy)

        if h_diff > 0:
            move_action = f"move_up_{h_diff}"
        elif h_diff < 0:
            move_action = f"move_down_{abs(h_diff)}"
        else:
            move_action = f"move_{direction}"

        # 无高度差: 全向轮可任意方向平移, 只有侧向取 R2 才需转向
        # 有高度差: 必须正向行驶, face 不可省略
        if turned_for_r2 or h_diff != 0:
            step_actions = [face_action]
        else:
            step_actions = []
        if take_action:
            step_actions.append(take_action)
        step_actions.append(move_action)
        all_actions.append("+".join(step_actions))

    return "+".join(all_actions)


def commands_to_binary(cmd_str):
    """将命令字符串转换为二进制 bytearray.

    Args:
        cmd_str: 如 "face_right+take_front_r2+move_right"

    Returns:
        bytearray
    """
    buf = bytearray()
    for act in cmd_str.split("+"):
        if not act:
            continue
        val = ACTION_MAP.get(act)
        if val is None:
            raise KeyError(f"unknown action '{act}' in command string")
        buf.extend(val)
    return buf


def fmt_hex(buf):
    """将 bytearray 格式化为 0xXX 空格分隔的字符串."""
    return " ".join(f"0x{b:02X}" for b in buf)
