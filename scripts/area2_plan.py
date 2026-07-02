#!/usr/bin/env python3
"""2区 KFS 串口读取 + 路径规划.

用法:
  python3 scripts/area2_plan.py --serial /dev/ttyUSB0 [--side blue|red]

流程:
  1. 打开串口, 等待 KFS 帧 (14 字节: 0x5A + 12×uint8 + 0xA5)
  2. 连续 3 帧一致 → 触发 2区规划
  3. 输出航点列表和 /path/commands
  4. 航点保存到 /tmp/area2_waypoints.json

无 ROS 依赖, 可脱离 waypoint_executor 独立运行.
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import time
import argparse
import numpy as np

from pipeline.kfs_reader import KfsSerialReader
from pipeline.orchestrator import process_grid
from pipeline.commands_generator import generate_commands, commands_to_binary, fmt_hex


def main():
    parser = argparse.ArgumentParser(description="2区 KFS 串口读取 + 路径规划")
    parser.add_argument("--serial", type=str, required=True,
                        help="KFS 串口设备路径 (如 /dev/ttyUSB0)")
    parser.add_argument("--side", type=str, default="blue",
                        choices=["red", "blue"], help="红/蓝方 (默认 blue)")
    parser.add_argument("--baud", type=int, default=115200,
                        help="串口波特率 (默认 115200)")
    args = parser.parse_args()

    print(f"[Area2] 串口: {args.serial}, 波特率: {args.baud}, 阵营: {args.side}")
    print(f"[Area2] 等待 KFS 帧 (连续 3 帧一致触发)...")

    last_result = {}

    def on_grid(grid_4x3):
        """KFS 帧回调: 运行 pipeline 并输出结果."""
        print(f"\n[Area2] 收到 KFS 网格 (连续 3 帧一致):")
        print(grid_4x3)

        result = process_grid(grid_4x3, args.side, verbose=True)
        last_result.clear()
        last_result.update(result)

        if "error" in result:
            print(f"\n[Area2] 规划失败: {result['error']}")
            return

        # 输出航点
        wps = result.get("absolute_waypoints", [])
        print(f"\n[Area2] 航点 ({len(wps)} 个):")
        for i, wp in enumerate(wps):
            h = f" height={wp['height']}" if 'height' in wp else ""
            print(f"  {i}: {wp['name']} → "
                  f"({wp['x']:.3f}, {wp['y']:.3f}, {wp['theta']:.3f}){h}")

        # 输出 /path/commands
        cmd_str = result.get("commands", "")
        binary = result.get("binary_commands", b"")
        print(f"\n[Area2] /path/commands: {cmd_str}")
        print(f"[Area2] Binary ({len(binary)} bytes): {fmt_hex(binary)}")

        print(f"\n[Area2] 航点已保存到 /tmp/area2_waypoints.json")
        print(f"[Area2] 等待下一次 KFS 帧...")

    reader = KfsSerialReader(args.serial, baud=args.baud, on_grid=on_grid)
    reader.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[Area2] 停止")
        reader.stop()


if __name__ == "__main__":
    main()
