"""A* 路径规划 — 从 KFS 4×3 网格到 A* 路径 + R1 KFS IDs.

无 ROS 依赖。核心算法从 path_planner/planner.py 提取。
"""

import heapq
import numpy as np

from lib.config import get_config

# ========== A* 常量 ==========

OBSTACLE_VAL = 4
TURN_PENALTY = 2
PICKUP_FRONT_COST = 1.0
PICKUP_SIDE_COST = 2.0
EXTRA_R2_PENALTY = 0.0
DEFAULT_REQUIRED_R2 = 2
DEFAULT_REQUIRED_R1 = 2

DIR_VEC = {1: (0, -1), 2: (0, 1), 3: (-1, 0), 4: (1, 0)}

DIR_OPTIONS = {
    0: [((0, -1), 1), ((-1, 0), 3), ((1, 0), 4)],
    1: [((0, -1), 1), ((-1, 0), 3), ((1, 0), 4)],
    2: [((0, 1), 2), ((-1, 0), 3), ((1, 0), 4)],
    3: [((-1, 0), 3), ((0, -1), 1), ((0, 1), 2)],
    4: [((1, 0), 4), ((0, -1), 1), ((0, 1), 2)],
}

NEIGHBOR_OFFSETS = [(0, -1), (0, 1), (-1, 0), (1, 0)]


def _facing_vec(dir_id):
    if dir_id == 0:
        return (0, -1)
    return DIR_VEC[dir_id]


def _pickup_cost(dx, dy, face_dx, face_dy):
    if (dx, dy) == (face_dx, face_dy):
        return 'front', PICKUP_FRONT_COST
    if (dx, dy) == (face_dy, -face_dx):
        return 'left', PICKUP_SIDE_COST
    if (dx, dy) == (-face_dy, face_dx):
        return 'right', PICKUP_SIDE_COST
    return None


def _heuristic(x, y, tx, ty):
    return abs(x - tx) + abs(y - ty)


def _r2_count(mask, r2_indices):
    return sum(1 for i in r2_indices if (mask >> i) & 1)


def _r1_count(mask, r1_indices):
    return sum(1 for i in r1_indices if (mask >> i) & 1)


class _Node:
    __slots__ = ('x', 'y', 'in_direction', 'r2_mask', 'r1_mask', 'g', 'h', 'f', 'parent')

    def __init__(self, x, y, in_direction, r2_mask=0, r1_mask=0, g=0, h=0.0):
        self.x, self.y = x, y
        self.in_direction = in_direction
        self.r2_mask, self.r1_mask = r2_mask, r1_mask
        self.g, self.h = g, h
        self.f = g + h
        self.parent = None

    def __lt__(self, other):
        return self.f < other.f


def astar_plan(height_map, start, target, fake=None, r1=None, r2=None,
               mandatory_r2=None, required_r2=None, required_r1=None, entry_row=None):
    """A* 路径规划.

    Returns:
        (path, r1_positions_out, r2_positions_out, r2_mask, r1_mask, total_cost)
        失败时返回 (None,)*6
    """
    if r1 is None:
        r1 = []
    if r2 is None:
        r2 = []
    if required_r2 is None:
        required_r2 = DEFAULT_REQUIRED_R2
    if required_r1 is None:
        required_r1 = DEFAULT_REQUIRED_R1

    start_x, start_y = start
    target_x, target_y = target
    amap = np.array(height_map, dtype=float)
    rows, cols = amap.shape

    if entry_row is None:
        entry_row = 4 if start_y == 5 else start_y

    work_map = amap.copy()
    obx, oby = (fake if fake else (None, None))
    if fake:
        work_map[oby][obx] = OBSTACLE_VAL

    kfs_list = [(x, y, 'R2') for x, y in r2] + [(x, y, 'R1') for x, y in r1]
    kfs_pos_to_idx = {(x, y): i for i, (x, y, _) in enumerate(kfs_list)}
    r2_indices = [i for i, (_, _, t) in enumerate(kfs_list) if t == 'R2']
    r1_indices = [i for i, (_, _, t) in enumerate(kfs_list) if t == 'R1']

    mandatory_set = set()
    if mandatory_r2:
        for mx, my in mandatory_r2:
            if (mx, my) in kfs_pos_to_idx:
                mandatory_set.add(kfs_pos_to_idx[(mx, my)])

    def _mandatory_satisfied(r2_mask):
        if not mandatory_set:
            return True
        return bool(mandatory_set & {i for i in r2_indices if (r2_mask >> i) & 1})

    def is_cell_blocked(x, y, r2_mask, r1_mask=0):
        if not (0 <= x < cols and 0 <= y < rows):
            return True
        if work_map[y][x] == OBSTACLE_VAL:
            return True
        if (x, y) in kfs_pos_to_idx:
            idx = kfs_pos_to_idx[(x, y)]
            ktype = kfs_list[idx][2]
            if ktype == 'R2' and not ((r2_mask >> idx) & 1):
                return True
            elif ktype == 'R1' and not ((r1_mask >> idx) & 1):
                if _r1_count(r1_mask, r1_indices) >= required_r1:
                    return True
                if y == entry_row and mandatory_set and not _mandatory_satisfied(r2_mask):
                    return True
            elif y == entry_row and mandatory_set and not _mandatory_satisfied(r2_mask):
                return True
        elif y == entry_row and mandatory_set and not _mandatory_satisfied(r2_mask):
            return True
        return False

    def compute_pickup_cost(x, y, face_dir_id, r2_mask):
        fx, fy = _facing_vec(face_dir_id)
        added_cost = 0.0
        new_r2 = r2_mask
        for adx, ady in NEIGHBOR_OFFSETS:
            kx, ky = x + adx, y + ady
            if (kx, ky) not in kfs_pos_to_idx:
                continue
            kidx = kfs_pos_to_idx[(kx, ky)]
            if kfs_list[kidx][2] != 'R2' or ((new_r2 >> kidx) & 1):
                continue
            already_have = bin(new_r2).count('1')
            is_mandatory = kidx in mandatory_set
            extra = EXTRA_R2_PENALTY if already_have >= required_r2 and not is_mandatory else 0.0
            rel = _pickup_cost(adx, ady, fx, fy)
            if rel is not None:
                _, cost = rel
                added_cost += cost + extra
                new_r2 |= (1 << kidx)
        return added_cost, new_r2

    open_heap = []
    closed_set = set()

    start_node = _Node(start_x, start_y, 0,
                       h=_heuristic(start_x, start_y, target_x, target_y))
    heapq.heappush(open_heap, (start_node.f, id(start_node), start_node))

    while open_heap:
        _, _, current = heapq.heappop(open_heap)

        if (current.x == target_x and current.y == target_y
                and _r2_count(current.r2_mask, r2_indices) >= required_r2
                and (not mandatory_set or
                     mandatory_set & {i for i in r2_indices if (current.r2_mask >> i) & 1})):
            path = []
            node = current
            while node:
                path.append((node.x, node.y))
                node = node.parent
            return (path[::-1], r1, r2, current.r2_mask, current.r1_mask, current.g)

        state_key = (current.x, current.y, current.in_direction,
                     current.r2_mask, current.r1_mask)
        if state_key in closed_set:
            continue
        closed_set.add(state_key)

        pickup_add, new_r2 = compute_pickup_cost(
            current.x, current.y, current.in_direction, current.r2_mask)

        for (dx, dy), new_dir in DIR_OPTIONS[current.in_direction]:
            nx, ny = current.x + dx, current.y + dy
            if not (0 <= nx < cols and 0 <= ny < rows):
                continue
            if is_cell_blocked(nx, ny, new_r2, current.r1_mask):
                continue

            terrain_from = int(work_map[current.y][current.x])
            terrain_to = int(work_map[ny][nx])
            height_diff = abs(terrain_to - terrain_from)
            road_diff = 1 + (height_diff if height_diff <= 1 else 1)
            if ny == 0 or ny == 5:
                road_diff = road_diff / 2.0

            new_r1 = current.r1_mask
            if (nx, ny) in kfs_pos_to_idx:
                kidx = kfs_pos_to_idx[(nx, ny)]
                if kfs_list[kidx][2] == 'R1' and not ((new_r1 >> kidx) & 1):
                    new_r1 |= (1 << kidx)

            turn_cost = TURN_PENALTY if current.in_direction != 0 and current.in_direction != new_dir else 0
            new_g = current.g + road_diff + turn_cost + pickup_add
            new_h = _heuristic(nx, ny, target_x, target_y)

            new_node = _Node(nx, ny, new_dir, new_r2, new_r1, new_g, new_h)
            new_node.parent = current

            new_state = (nx, ny, new_dir, new_r2, new_r1)
            if new_state not in closed_set:
                heapq.heappush(open_heap, (new_node.f, id(new_node), new_node))

    return None, None, None, None, None, float('inf')


# ========== R1 KFS ID 选择 ==========

def _cell_id(x, y):
    """A* 坐标 (x,y) → cell_id (0..14)."""
    return (5 - y) * 3 + x


def select_r1_ids(crossed_r1, all_r1):
    """根据跨越的 R1 位置，选择 2 个 KFS ID.

    Args:
        crossed_r1: 实际跨越的 R1 位置列表 [(x,y), ...]
        all_r1:     所有 R1 位置列表 [(x,y), ...]

    Returns:
        [id1, id2]: 选中的 2 个 KFS ID (cell_id 即 KFS ID)
    """
    def _cid(xy):
        return _cell_id(xy[0], xy[1])

    n = len(crossed_r1)

    if n == 0:
        # 0 个跨越: 在同列或同行中找 >= 2 个 R1
        for col in range(3):
            col_r1 = [xy for xy in all_r1 if xy[0] == col]
            ids = [_cid(xy) for xy in col_r1]
            if len(ids) >= 2:
                return sorted(ids)[:2]
        for row in range(1, 5):
            row_r1 = [xy for xy in all_r1 if xy[1] == row]
            ids = [_cid(xy) for xy in row_r1]
            if len(ids) >= 2:
                return sorted(ids)[:2]
        return []

    elif n == 1:
        # 1 个跨越: 以该 R1 为基准，找同列或同行的另一个
        cx, cy = crossed_r1[0]
        col_r1 = [xy for xy in all_r1 if xy[0] == cx and xy != (cx, cy)]
        ids = [_cid(xy) for xy in col_r1]
        if ids:
            base_id = _cid((cx, cy))
            if base_id:
                return sorted([base_id, ids[0]])
        row_r1 = [xy for xy in all_r1 if xy[1] == cy and xy != (cx, cy)]
        ids = [_cid(xy) for xy in row_r1]
        if ids:
            base_id = _cid((cx, cy))
            if base_id:
                return sorted([base_id, ids[0]])
        return []

    else:  # n >= 2
        ids = [_cid(xy) for xy in crossed_r1]
        return sorted(ids)[:2]


def get_crossed_r1_positions(path, r1_list, r1_mask, r2_offset=0):
    """获取路径中实际跨越的 R1 位置.

    r1_mask 的位索引 = r2_offset + i (i 为 r1_list 中的序号).
    """
    if r1_mask is None:
        return []
    result = []
    for i, (x, y) in enumerate(r1_list):
        if (r1_mask >> (r2_offset + i)) & 1:
            result.append((x, y))
    return result


def plan_astar_for_grid(grid_4x3, side="blue"):
    """对 KFS 4×3 网格执行完整 A* 规划.

    Args:
        grid_4x3: 4×3 numpy array (uint8: 0=空, 1=R1, 2=R2, 3=Fake)
        side:     "red" 或 "blue"

    Returns:
        dict with keys: path, r1_ids, r2_positions, r1_positions, fake_position, r2_mask, r1_mask, cost
    """
    cfg = get_config(side)
    height_map = cfg.get_height_map()
    start = cfg.default_start
    target = cfg.default_target

    # 解析 4×3 网格
    r1_positions = []
    r2_positions = []
    fake_position = None
    for r in range(4):
        for c in range(3):
            val = int(grid_4x3[r, c])
            ax, ay = c, r + 1  # 4×3(row) → A* (x, y), y 从 1 开始
            if val == 0:
                continue
            elif val == 2:
                r2_positions.append((ax, ay))
            elif val == 1:
                r1_positions.append((ax, ay))
            elif val == 3:
                fake_position = (ax, ay)

    entry_row = 4 if start[1] == 5 else start[1]

    # A* 规划
    path, _, _, r2_mask, r1_mask, cost = astar_plan(
        height_map, start, target,
        fake=fake_position, r1=r1_positions, r2=r2_positions,
        required_r2=DEFAULT_REQUIRED_R2, required_r1=DEFAULT_REQUIRED_R1,
        entry_row=entry_row,
    )

    if not path:
        return {"error": "NO PATH FOUND"}

    # 获取跨越的 R1
    crossed_r1 = get_crossed_r1_positions(path, r1_positions, r1_mask, len(r2_positions))

    r1_ids = select_r1_ids(crossed_r1, r1_positions)

    return {
        "path": path,
        "r1_ids": r1_ids,
        "crossed_r1": crossed_r1,
        "r2_positions": r2_positions,
        "r1_positions": r1_positions,
        "fake_position": fake_position,
        "r2_mask": r2_mask,
        "r1_mask": r1_mask,
        "cost": cost,
    }
