#!/usr/bin/env python3
"""KFS 分布矩阵串口发送器 — 运行在另一台设备上，模拟相机检测结果。

4×3 网格结构 (与 order.py _assemble_integrated_grid 一致):
  Row 0 (A* y=1): 远后行 — rear_two 检测 row2
  Row 1 (A* y=2): 近后行 — rear_two 检测 row1
  Row 2 (A* y=3): 近前行 — first_two 检测 row1
  Row 3 (A* y=4): 入口行 — first_two 检测 row0

每个 cell 编码 (1 byte):
  0x00 = 空
  0x01 = R1 KFS (需跨越, 共3个)
  0x02 = R2 KFS (需收集, 共4个)
  0x03 = Fake/障碍 (0-1个)

发送端不关心高度地图, 仅发送 type code. 高度地图由接收端合并.

Frame: [HEADER(0x5A) | 12×uint8 | TAIL(0xA5)] = 14 bytes

Usage:
  python3 test_serial_sender.py /dev/ttyUSB0                  # 默认场景0
  python3 test_serial_sender.py /dev/ttyUSB0 --scene 1        # 场景1
  python3 test_serial_sender.py /dev/ttyUSB0 --rate 2         # 2Hz 持续
  python3 test_serial_sender.py --list                        # 列出场景
"""

import time
import argparse
import sys
import random

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run: pip install pyserial", file=sys.stderr)
    sys.exit(1)


HEADER = 0x5A
TAIL = 0xA5

# Cell type codes
EMPTY = 0x00
R1 = 0x01
R2 = 0x02
FAKE = 0x03

# 预设场景: items = [(x, y, type), ...]
# y 使用实际地图坐标 (1..4), type in ('r2','r1','fake')
SCENES = {
    1: {
        'r1': [(0, 3), (2, 3), (2, 1)],
        'r2': [(0, 4), (1, 3), (1, 1), (0, 2)],
        'fake': [(0, 1)],
    },
    2: {
        'r1': [(2, 1), (0, 2), (2, 4)],
        'r2': [(1, 1), (1, 3), (2, 3), (1, 4)],
        'fake': [(0, 3)],
    },
}

TYPE_TO_CODE = {'r2': R2, 'r1': R1, 'fake': FAKE}
CODE_TO_NAME = {EMPTY: ' · ', R1: ' R1 ', R2: ' R2 ', FAKE: 'FAKE'}


def make_grid(items):
    """根据 KFS 位置列表生成 4x3 矩阵 (uint8)."""
    grid = [[EMPTY] * 3 for _ in range(4)]
    for x, y, ktype in items:
        if not (0 <= x <= 2 and 1 <= y <= 4):
            raise ValueError(f"({x},{y}) 超出 KFS 区域")
        if grid[y - 1][x] != EMPTY:
            raise ValueError(f"({x},{y}) 重复分配 {CODE_TO_NAME[grid[y-1][x]]} → {ktype}")
        grid[y - 1][x] = TYPE_TO_CODE[ktype]
    return grid


def build_frame(grid):
    """[HEADER(1) | 12×uint8 | TAIL(1)] = 14 bytes"""
    buf = bytearray([HEADER])
    for row in grid:
        buf.extend(row)
    buf.append(TAIL)
    return bytes(buf)


def print_grid(grid):
    print(f"\nKFS 分布矩阵 (4×3):")
    print(f"  y\\x │ x=0     x=1     x=2")
    print(f"  ────┼──────────────────────")
    counts = {R1: 0, R2: 0, FAKE: 0}
    for yi in range(3, -1, -1):
        y = yi + 1
        parts = [CODE_TO_NAME.get(v, f' ?({v}) ') for v in grid[yi]]
        for v in grid[yi]:
            if v in counts:
                counts[v] += 1
        print(f"  y={y} │ {' '.join(parts)}")
    print(f"  R2={counts[R2]}(需4)  R1={counts[R1]}(需3)  Fake={counts[FAKE]}")


def main():
    parser = argparse.ArgumentParser(description="KFS 分布矩阵串口发送器")
    parser.add_argument("port", help="串口设备，如 /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--rate", type=float, default=1.0, help="发送频率 Hz")
    parser.add_argument("--count", type=int, default=0, help="发送次数 (0=无限)")
    parser.add_argument("--scene", type=int, default=0, help="场景编号")
    parser.add_argument("--list", action="store_true", help="列出所有场景")
    args = parser.parse_args()

    if args.list:
        for sid, items in SCENES.items():
            all_items = [(x, y, 'r2') for x, y in items['r2']] + \
                        [(x, y, 'r1') for x, y in items['r1']] + \
                        [(x, y, 'fake') for x, y in items['fake']]
            print(f"\nscene {sid}:")
            g = make_grid(all_items)
            print_grid(g)
        return

    if args.scene not in SCENES:
        print(f"未知场景 {args.scene}", file=sys.stderr)
        print(f"可用: {list(SCENES.keys())}", file=sys.stderr)
        sys.exit(1)

    items = SCENES[args.scene]
    all_items = [(x, y, 'r2') for x, y in items['r2']] + \
                [(x, y, 'r1') for x, y in items['r1']] + \
                [(x, y, 'fake') for x, y in items['fake']]
    grid = make_grid(all_items)
    print(f"场景 {args.scene}:")
    print_grid(grid)

    buf = build_frame(grid)
    print(f"\nFrame ({len(buf)}B): {' '.join(f'{b:02X}' for b in buf)}")
    print(f"Hex array: {{{', '.join('0x%02X' % b for b in buf)}}}")

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1.0)
        print(f"\n{args.port} 已打开 @ {args.baud}")
    except Exception as e:
        print(f"\nERROR: {args.port}: {e}", file=sys.stderr)
        sys.exit(1)

    try:
        period = 1.0 / args.rate
        cnt = 0
        print(f"发送中 @ {args.rate}Hz (Ctrl+C 停止)...")
        while args.count == 0 or cnt < args.count:
            ser.write(buf)
            cnt += 1
            print(f"  [{cnt}]", end="\r")
            if args.count > 0 and cnt >= args.count:
                break
            time.sleep(period)
        print(f"\n完成, 共 {cnt} 帧")
    except KeyboardInterrupt:
        print("\n中断")
    finally:
        ser.close()
        print(f"{args.port} 已关闭")


if __name__ == "__main__":
    main()
