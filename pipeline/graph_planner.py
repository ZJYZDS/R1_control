"""R1 图规划 — 从 2 个 KFS ID 到航点列表.

无 ROS 依赖。核心算法从 r1_graph/mapper.py + r1_polyline_planner.py 提取。
"""

import math
import numpy as np

from lib.config import get_config


# ========== GraphMapper (从 r1_graph/mapper.py) ==========

class GraphMapper:
    def __init__(self, cfg=None):
        self.cfg = cfg or get_config()
        self.id_map = self.cfg.id_to_coord
        self.known = [np.array(c) for c in self.cfg.known_coords]
        self._base_nodes, self._base_adj = self._build_base_graph()

    def id_to_coord(self, idx):
        coord = self.id_map.get(idx)
        if coord is None:
            return None
        if isinstance(coord[0], list):
            return np.array(coord[0], dtype=np.float64)
        return np.array(coord, dtype=np.float64)

    def id_to_coords(self, idx):
        """Return all coordinate options for a KFS ID as np.array list."""
        coord = self.id_map.get(idx)
        if coord is None:
            return []
        if isinstance(coord[0], list):
            return [np.array(c, dtype=np.float64) for c in coord]
        return [np.array(coord, dtype=np.float64)]

    def _build_base_graph(self):
        nodes = [
            ("K1", self.known[1]),
            ("K2", self.known[2]),
            ("K3", self.known[3]),
            ("K4", self.known[4]),
        ]
        return self._make_adj(nodes)

    def build_graph(self, target_coords):
        nodes = [("TGT1", target_coords[0]), ("TGT2", target_coords[1])]
        nodes.extend(self._base_nodes)
        adj = self._offset_adj(self._base_adj, 2)

        for tgt_i in [0, 1]:
            adj.setdefault(tgt_i, {})
            for j in range(2, len(nodes)):
                if not self._is_axis_aligned(nodes[tgt_i][1], nodes[j][1]):
                    continue
                cost = self._edge_cost(nodes[tgt_i][1], nodes[j][1])
                adj[tgt_i][j] = cost
                adj.setdefault(j, {})[tgt_i] = cost

        tgt0_k = set(adj[0].keys())
        tgt1_k = set(adj[1].keys())
        if tgt0_k & tgt1_k:
            cost = self._edge_cost(nodes[0][1], nodes[1][1])
            adj[0][1] = cost
            adj[1][0] = cost

        return nodes, adj

    def find_path(self, nodes, adj, start_idx, end_idx=None):
        orders = [(0, 1), (1, 0)]
        best_path, best_cost = None, float("inf")
        for a, b in orders:
            seg1, c1 = self._dijkstra(adj, start_idx, a)
            if seg1 is None:
                continue
            seg2, c2 = self._dijkstra(adj, a, b)
            if seg2 is None:
                continue
            if end_idx is not None:
                seg3, c3 = self._dijkstra(adj, b, end_idx)
                if seg3 is None:
                    continue
                total = c1 + c2 + c3
                if total < best_cost:
                    best_cost = total
                    best_path = [nodes[i] for i in seg1 + seg2[1:] + seg3[1:]]
            else:
                total = c1 + c2
                if total < best_cost:
                    best_cost = total
                    best_path = [nodes[i] for i in seg1 + seg2[1:]]
        if best_path is None:
            return []
        return best_path

    @staticmethod
    def _is_axis_aligned(pi, pj, tol=0.01):
        return abs(pi[0] - pj[0]) < tol or abs(pi[1] - pj[1]) < tol

    @staticmethod
    def _offset_adj(adj, offset):
        return {i + offset: {j + offset: c for j, c in nbrs.items()}
                for i, nbrs in adj.items()}

    def _make_adj(self, nodes):
        n = len(nodes)
        adj = {i: {} for i in range(n)}
        for i in range(n):
            for j in range(i + 1, n):
                if not self._is_axis_aligned(nodes[i][1], nodes[j][1]):
                    continue
                cost = self._edge_cost(nodes[i][1], nodes[j][1])
                adj[i][j] = cost
                adj[j][i] = cost
        return nodes, adj

    def _edge_cost(self, pi, pj):
        dx = abs(pj[0] - pi[0])
        dy = abs(pj[1] - pi[1])
        dz = abs(pj[2] - pi[2])
        dist = np.sqrt(dx**2 + dy**2 + dz**2)
        if dx > 0.6 and dy > 0.6:
            return dist + self.cfg.turn_90_penalty
        return dist

    def _dijkstra(self, adj, start, goal):
        n = len(adj)
        dist = {i: float("inf") for i in range(n)}
        prev = {}
        dist[start] = 0.0
        unvisited = set(range(n))

        while unvisited:
            u = min(unvisited, key=lambda i: dist[i])
            if dist[u] == float("inf"):
                break
            unvisited.remove(u)
            if u == goal:
                break
            for v, cost in adj[u].items():
                if v not in unvisited:
                    continue
                alt = dist[u] + cost
                if alt < dist[v]:
                    dist[v] = alt
                    prev[v] = u

        if dist[goal] == float("inf"):
            return None, float("inf")

        path = [goal]
        curr = goal
        while curr != start:
            curr = prev[curr]
            path.append(curr)
        return path[::-1], dist[goal]


# ========== ID 补充 (从 mapper.py supplement_ids) ==========

def supplement_ids(mapper, required_ids, valid_ids=None):
    if valid_ids is None:
        valid_ids = sorted(mapper.id_map.keys())

    id_num = len(required_ids)
    if id_num >= 2:
        return required_ids[0], required_ids[1]

    cfg = get_config()
    start_idx = cfg.k_index[cfg.start_k] + 1  # K_GRAPH_OFFSET
    end_idx = cfg.k_index[cfg.end_k] + 1

    def _pair_cost(a, b):
        coords_a = mapper.id_to_coords(a)
        coords_b = mapper.id_to_coords(b)
        if not coords_a or not coords_b:
            return float('inf')
        best = float('inf')
        for c1 in coords_a:
            for c2 in coords_b:
                nodes, adj = mapper.build_graph([c1, c2])
                route = mapper.find_path(nodes, adj, start_idx, end_idx)
                if not route:
                    continue
                cost = sum(np.linalg.norm(route[i][1][:2] - route[i + 1][1][:2])
                           for i in range(len(route) - 1))
                if cost < best:
                    best = cost
        return best

    if id_num == 0:
        best_pair = None
        best_cost = float('inf')
        for i, a in enumerate(valid_ids):
            for b in valid_ids[i + 1:]:
                cost = _pair_cost(a, b)
                if cost < best_cost:
                    best_cost = cost
                    best_pair = (a, b)
        return best_pair if best_pair else (None, None)

    if id_num == 1:
        req = required_ids[0]
        best_id = None
        best_cost = float('inf')
        for b in valid_ids:
            if b == req:
                continue
            cost = _pair_cost(req, b)
            if cost < best_cost:
                best_cost = cost
                best_id = b
        return (req, best_id) if best_id else (None, None)

    return None, None


# ========== 航点生成 (从 r1_polyline_planner.py _route_to_waypoints) ==========

def route_to_waypoints(route, tgt_ids=None, cfg=None):
    """将图规划 route → 航点列表 [(type, dx, dy, dyaw, name), ...].

    type: 'translate' 或 'rotate'
    """
    if cfg is None:
        cfg = get_config()
    tgt_ids = tgt_ids or {}
    dir_c = cfg.dir_correct

    n = len(route)
    headings = []
    for i in range(n - 1):
        a, b = route[i][1], route[i + 1][1]
        headings.append(math.atan2(b[1] - a[1], b[0] - a[0]))

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
                    if tgt_id is not None and tgt_id in cfg.tgt_facing:
                        target_facing = cfg.get_tgt_facing(tgt_id, route[1][1])
                        dtheta = (target_facing - cfg.start_yaw) * dir_c
                    else:
                        dtheta = (headings[0] - cfg.start_yaw) * dir_c
                else:
                    dtheta = (headings[0] - cfg.start_yaw) * dir_c
                dtheta = math.atan2(math.sin(dtheta), math.cos(dtheta))
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
                    if tgt_id is not None and tgt_id in cfg.tgt_facing:
                        target_facing = cfg.get_tgt_facing(tgt_id, route[i + 1][1])
                        dtheta = (target_facing - headings[i - 1]) * dir_c
                    else:
                        dtheta = (headings[i] - headings[i - 1]) * dir_c
                else:
                    dtheta = (headings[i] - headings[i - 1]) * dir_c
                dtheta = math.atan2(math.sin(dtheta), math.cos(dtheta))
                if abs(dtheta) > 0.001:
                    wp.append(('rotate', 0.0, 0.0, dtheta, name))
            elif is_k:
                dtheta = (cfg.target_end_yaw - headings[-1]) * dir_c
                dtheta = math.atan2(math.sin(dtheta), math.cos(dtheta))
                if abs(dtheta) > 0.001:
                    wp.append(('rotate', 0.0, 0.0, dtheta, name))

    return wp


# ========== 完整图规划 (对外接口) ==========

def plan_graph(r1_ids, side="blue"):
    """从 2 个 KFS ID → 图规划 → 航点列表.

    Args:
        r1_ids: [id1, id2] — 2 个目标 R1 KFS ID
        side:   "red" 或 "blue"

    Returns:
        dict with keys: waypoints, route_nodes, tgt_coords
        waypoints: [(type, dx, dy, dyaw, name), ...]
    """
    cfg = get_config(side)
    mapper = GraphMapper(cfg)

    id1, id2 = r1_ids[0], r1_ids[1]
    coords1 = mapper.id_to_coords(id1)
    coords2 = mapper.id_to_coords(id2)
    if not coords1 or not coords2:
        return {"error": f"ID→coord failed: id1={id1}, id2={id2}"}

    tgt_ids = {"TGT1": id1, "TGT2": id2}
    start_idx = cfg.k_index[cfg.start_k] + 1  # K_GRAPH_OFFSET=1
    end_idx = cfg.k_index[cfg.end_k] + 1

    best_result = None
    best_cost = float('inf')

    for c1 in coords1:
        for c2 in coords2:
            nodes, adj = mapper.build_graph([c1, c2])
            route = mapper.find_path(nodes, adj, start_idx, end_idx)
            if not route:
                continue
            route_nodes = [n for n, _ in route]
            wps = route_to_waypoints(route, tgt_ids, cfg)
            cost = sum(np.linalg.norm(route[i][1][:2] - route[i + 1][1][:2])
                       for i in range(len(route) - 1))
            if cost < best_cost:
                best_cost = cost
                best_result = {
                    "waypoints": wps,
                    "route_nodes": route_nodes,
                    "tgt_coords": {
                        "TGT1": (float(c1[0]), float(c1[1]), float(c1[2])),
                        "TGT2": (float(c2[0]), float(c2[1]), float(c2[2])),
                    },
                    "start_k": cfg.start_k,
                    "end_k": cfg.end_k,
                }

    if best_result is None:
        return {"error": "No graph route found"}
    return best_result
