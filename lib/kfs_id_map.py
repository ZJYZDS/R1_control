"""KFS cell_id → KFS ID 映射.

cell_id = (5 - y) * 3 + x  (A* 坐标 → 0..14)
直接从 path_planner/config.yaml 的 kfs_id_map 读取预定义映射。
"""

from lib.config import get_config


def load_kfs_id_map(side="blue"):
    """返回 {cell_id: kfs_id} 映射."""
    return get_config(side).kfs_id_map
