#!/usr/bin/env python3
"""模拟 KFS 串口发送器 — 通过 PTY 虚拟串口发送 4×3 网格数据.

用法:
  Terminal 1: python3 scripts/mock_kfs_sender.py
              → 创建虚拟串口 /tmp/kfs_virtual

  Terminal 2: python3 scripts/area2_plan.py --serial /tmp/kfs_virtual --side red
              → 接收并规划

网格示例 (对应 test scene 1 red):
  row 1-4, col 0-2: 0=空 1=R1 2=R2 3=Fake
"""

import sys
import os
import time
import signal
import threading

# 默认测试网格 (test scene 1 red):
# y=1: [0, 2, 0]  (R2 at col1)
# y=2: [1, 0, 0]  (R1 at col0)
# y=3: [2, 0, 0]  (R2 at col0)
# y=4: [0, 0, 3]  (Fake at col2)
DEFAULT_GRID = [
    [0, 2, 0],
    [1, 0, 0],
    [2, 0, 0],
    [0, 0, 3],
]

FRAME_HEADER = 0x5A
FRAME_TAIL   = 0xA5
N_CONSECUTIVE = 4  # 多发送一帧确保稳定触发


def build_frame(grid_4x3):
    """4×3 网格 → 14 字节 KFS 帧."""
    flat = []
    for row in grid_4x3:
        flat.extend(row)
    return bytes([FRAME_HEADER] + flat + [FRAME_TAIL])


def main():
    import argparse
    parser = argparse.ArgumentParser(description="模拟 KFS 串口发送器")
    parser.add_argument("--port", type=str, default="/tmp/kfs_virtual",
                        help="虚拟串口路径 (默认 /tmp/kfs_virtual)")
    parser.add_argument("--scene", type=int, default=1,
                        help="使用 test scene 编号 (默认 1)")
    parser.add_argument("--side", type=str, default="red",
                        choices=["red", "blue"])
    args = parser.parse_args()

    # 加载场景网格
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from lib.config import get_config
    cfg = get_config(args.side)
    scene = cfg.get_test_scene(args.scene)
    if scene is None:
        print(f"[Mock] Scene {args.scene} not found, 使用默认网格")
        grid = DEFAULT_GRID
    else:
        # 用 test scene 的 r1/r2/fake 构建 4×3 网格
        grid = [[0]*3 for _ in range(4)]
        for x, y in scene.get("r2", []):
            if 1 <= y <= 4:
                grid[y-1][x] = 2
        for x, y in scene.get("r1", []):
            if 1 <= y <= 4:
                grid[y-1][x] = 1
        fx, fy = scene.get("fake", (0, 0))
        if 1 <= fy <= 4:
            grid[fy-1][fx] = 3

    frame = build_frame(grid)
    print(f"[Mock] 网格:\n{grid}")
    print(f"[Mock] 帧 ({len(frame)} bytes): {' '.join(f'{b:02X}' for b in frame)}")

    # 用 pty 创建虚拟串口对
    import pty
    master_fd, slave_fd = pty.openpty()
    slave_name = os.ttyname(slave_fd)

    # 创建符号链接方便访问
    try:
        os.unlink(args.port)
    except OSError:
        pass
    os.symlink(slave_name, args.port)

    print(f"[Mock] 虚拟串口: {args.port} → {slave_name}")
    print(f"[Mock] 在另一个终端运行:")
    print(f"  python3 scripts/area2_plan.py --serial {args.port} --side {args.side}")
    print(f"[Mock] 按 Enter 发送 KFS 帧, Ctrl+C 退出...")

    running = [True]

    def sender_loop():
        """间隔发送 KFS 帧."""
        while running[0]:
            try:
                os.write(master_fd, frame)
                time.sleep(0.05)
            except OSError:
                break

    def keyboard():
        try:
            input()
            # 按 Enter 后持续发送
            t = threading.Thread(target=sender_loop, daemon=True)
            t.start()
            input("按 Enter 停止发送...")
        except (EOFError, KeyboardInterrupt):
            pass
        running[0] = False

    keyboard()

    os.close(master_fd)
    os.unlink(args.port)
    print(f"[Mock] 已清理, 退出")


if __name__ == "__main__":
    main()
