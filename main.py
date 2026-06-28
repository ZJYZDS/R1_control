#!/usr/bin/env python3
"""R1 总管道入口 — 从 KFS 网格到航点列表.

Usage:
  python3 main.py --test-scene 1 --side blue        # 测试场景
  python3 main.py --test-scene 1 --side red          # 红方测试
  python3 main.py --serial /dev/ttyUSB0 --side blue  # 串口实时模式

Output: 航点列表 [(type, dx, dy, dtheta, name), ...]
"""

import sys
import os

# 确保脚本目录在 sys.path 中
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pipeline.orchestrator import process_test_scene, run_serial_pipeline


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="R1 总管道: KFS 网格 → A* → 图规划 → 航点")
    parser.add_argument("--test-scene", type=int, default=None,
                        help="测试场景编号 (1 或 2)")
    parser.add_argument("--serial", type=str, default=None,
                        help="KFS 串口设备路径 (如 /dev/ttyUSB0)")
    parser.add_argument("--side", type=str, default="blue",
                        choices=["red", "blue"],
                        help="场景方向 (默认 blue)")
    args = parser.parse_args()

    if args.test_scene is not None:
        result = process_test_scene(args.test_scene, args.side, verbose=True)
        if "error" in result:
            print(f"\n[ERROR] {result['error']}", file=sys.stderr)
            sys.exit(1)
        print(f"\n[OK] 测试完成, {len(result['waypoints'])} 个航点")
        return 0

    if args.serial is not None:
        run_serial_pipeline(args.serial, args.side)
        return 0

    parser.print_help()
    return 1


if __name__ == "__main__":
    sys.exit(main())
