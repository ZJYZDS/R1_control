#!/usr/bin/env python3
"""发布测试场景 KFS 网格到 /total_map topic, 替代串口输入。

每个场景定义与 test_serial_sender.py 一致。
Usage:
  python3 publish_test_scene.py 1          # 场景1 持续发布
  python3 publish_test_scene.py 2 --once   # 场景2 单次发布
"""

import sys
import time

import rospy
from std_msgs.msg import Float32MultiArray

# 场景定义: items = {type: [(x, y), ...]}
SCENES = {
    1: {
        'r1':  [(0, 3), (2, 3), (2, 1)],
        'r2':  [(0, 4), (1, 3), (1, 1), (0, 2)],
        'fake': [(0, 1)],
    },
    2: {
        'r1':  [(2, 1), (0, 2), (2, 4)],
        'r2':  [(1, 1), (1, 3), (2, 3), (1, 4)],
        'fake': [(0, 3)],
    },
}

TYPE_CODE = {'r2': 2.0, 'r1': 1.0, 'fake': 3.0}


def build_grid(scene_id):
    items = SCENES[scene_id]
    grid = [[0.0] * 3 for _ in range(4)]
    for x, y in items['r2']:
        grid[y - 1][x] = 2.0
    for x, y in items['r1']:
        grid[y - 1][x] = 1.0
    for x, y in items['fake']:
        grid[y - 1][x] = 3.0
    # 展平为单行: row0, row1, row2, row3
    flat = []
    for row in grid:
        flat.extend(row)
    return flat


def main():
    if len(sys.argv) < 2:
        print("Usage: publish_test_scene.py <scene_id> [--once]", file=sys.stderr)
        sys.exit(1)

    scene_id = int(sys.argv[1])
    once = "--once" in sys.argv

    if scene_id not in SCENES:
        print(f"Unknown scene {scene_id}, available: {list(SCENES.keys())}", file=sys.stderr)
        sys.exit(1)

    grid = build_grid(scene_id)

    rospy.init_node("test_scene_publisher", anonymous=True)
    pub = rospy.Publisher("/total_map", Float32MultiArray, queue_size=5, latch=True)

    # 先发 3 帧让 path_planner 3帧一致性检查通过
    msg = Float32MultiArray(data=grid)
    rate = rospy.Rate(2)

    print(f"Scene {scene_id} grid: {grid}")
    print(f"Publishing to /total_map @ 2Hz...")

    if once:
        for _ in range(3):
            pub.publish(msg)
            rospy.sleep(0.1)
        print("Published 3 frames (once)")
    else:
        while not rospy.is_shutdown():
            pub.publish(msg)
            rate.sleep()


if __name__ == "__main__":
    main()
