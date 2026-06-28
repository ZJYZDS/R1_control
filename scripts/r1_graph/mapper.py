"""R1 graph-based path planning — GraphMapper + ID supplementation.

Pure planning library — no ROS dependencies.
"""

import numpy as np

try:
    import rospy
except ImportError:
    rospy = None

from .config import Config


class GraphMapper:
    def __init__(self):
        self.id_map = Config.ID_TO_COORD
        self.known = [np.array(c) for c in Config.KNOWN_COORDS]
        self._base_nodes, self._base_adj = self._build_base_graph()

    def id_to_coord(self, idx):
        coord = self.id_map.get(idx)
        if coord is None:
            if rospy:
                rospy.logerr(f"Unknown ID: {idx}, available: {list(self.id_map.keys())}")
            return None
        return np.array(coord, dtype=np.float64)

    # ── Base graph: K1-K4, fully connected by rows/cols ──

    def _build_base_graph(self):
        nodes = [
            ("K1", self.known[1]),
            ("K2", self.known[2]),
            ("K3", self.known[3]),
            ("K4", self.known[4]),
        ]
        return self._make_adj(nodes)

    # ── Dual-target planning ──

    def build_graph(self, target_coords):
        nodes = [
            ("TGT1", target_coords[0]),
            ("TGT2", target_coords[1]),
        ]
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

        # TGT-TGT 直连: 仅当在同一条 K 边上 (共享至少一个 K 邻居)
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
            if rospy:
                rospy.logerr("No path through both targets")
            return []
        return best_path

    # ── Single-target planning ──

    def build_graph_single(self, target_coord):
        nodes = [("TGT", target_coord)]
        nodes.extend(self._base_nodes)

        adj = self._offset_adj(self._base_adj, 1)

        adj[0] = {}
        for j in range(1, len(nodes)):
            if not self._is_axis_aligned(nodes[0][1], nodes[j][1]):
                continue
            cost = self._edge_cost(nodes[0][1], nodes[j][1])
            adj[0][j] = cost
            adj.setdefault(j, {})[0] = cost

        return nodes, adj

    def find_path_to_target(self, nodes, adj, start_idx):
        path_idx, cost = self._dijkstra(adj, start_idx, 0)
        if path_idx is None:
            return []
        return [nodes[i] for i in path_idx], cost

    # ── Internal ──

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
            return dist + Config.TURN_90_PENALTY
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


# ── ID supplementation: fill to exactly 2 with minimum-cost pair ──

def supplement_ids(mapper, required_ids, valid_ids=None):
    """Supplement required_ids to exactly 2, minimizing total path cost.

    Args:
        mapper:       GraphMapper instance
        required_ids: list of already-known IDs (length 0, 1, or 2)
        valid_ids:    candidate pool (default: all IDs in Config.ID_TO_COORD)

    Returns:
        (id1, id2): supplemented pair, or (None, None) on failure
    """
    if valid_ids is None:
        valid_ids = sorted(Config.ID_TO_COORD.keys())

    id_num = len(required_ids)
    if id_num == 2:
        return required_ids[0], required_ids[1]

    start_idx = Config.K_INDEX[Config.START_K] + Config.K_GRAPH_OFFSET
    end_idx   = Config.K_INDEX[Config.END_K]   + Config.K_GRAPH_OFFSET

    def _pair_cost(a, b):
        c1 = mapper.id_to_coord(a)
        c2 = mapper.id_to_coord(b)
        if c1 is None or c2 is None:
            return float('inf')
        nodes, adj = mapper.build_graph([c1, c2])
        route = mapper.find_path(nodes, adj, start_idx, end_idx)
        if not route:
            return float('inf')
        return sum(np.linalg.norm(route[i][1][:2] - route[i + 1][1][:2])
                   for i in range(len(route) - 1))

    if id_num == 0:
        best_pair = None
        best_cost = float('inf')
        for i, a in enumerate(valid_ids):
            for b in valid_ids[i + 1:]:
                cost = _pair_cost(a, b)
                if cost < best_cost:
                    best_cost = cost
                    best_pair = (a, b)
        return best_pair

    if id_num == 1:
        req = required_ids[0]
        best_pair = None
        best_cost = float('inf')
        for b in valid_ids:
            if b == req:
                continue
            cost = _pair_cost(req, b)
            if cost < best_cost:
                best_cost = cost
                best_pair = (req, b)
        return best_pair

    return None, None
