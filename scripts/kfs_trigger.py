#!/usr/bin/env python3
"""KFS 串口触发规划 — 等待串口 KFS 信号 → 输出 R1 货架编码 + R2 指令 → 退出.

用法:
  ./scripts/kfs_trigger.py --port /dev/ttyUSB0 --side red [--frame-len 32]

流程:
  1. 打开串口, 等待 14 字节 KFS 帧 (连续 3 帧一致)
  2. 运行完整 pipeline (A* 规划 + 图规划 + 指令生成)
  3. 打印至终端:
     - R1 要取的货架 KFS 位置编码 (2 个 ID)
     - R2 command 二进制数组 (hex 格式)
  4. 通过同一串口发回 R2 指令帧
  5. 自动退出
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import time
import argparse
import threading

from pipeline.kfs_reader import KfsSerialReader
from pipeline.orchestrator import process_grid
from pipeline.commands_generator import fmt_hex


def main():
    parser = argparse.ArgumentParser(description="KFS 串口触发规划 (单次)")
    parser.add_argument("--port", default="/dev/ttyUSB0",
                        help="KFS 串口设备路径 (默认 /dev/ttyUSB0)")
    parser.add_argument("--side",  choices=["red", "blue"],
                        help="阵营 (无默认, 必须强制指定)")
    parser.add_argument("--baud", type=int, default=115200,
                        help="串口波特率 (默认 115200)")
    parser.add_argument("--frame-len", type=int, default=(25 + 1),
                        help="固定帧长度, 不足补最后一个字节 (默认 0=不补齐)")
    # 数据位 + 帧尾0x99
    args = parser.parse_args()

    print(f"[KFS] 等待串口 {args.port} KFS 信号 (阵营: {args.side})...")

    received_grid = [None]
    event = threading.Event()

    def on_grid(grid_4x3):
        """KFS 回调: 保存网格, 通知主线程. (在 reader 线程中执行)"""
        if event.is_set():
            return
        received_grid[0] = grid_4x3
        event.set()

    reader = KfsSerialReader(args.port, baud=args.baud, on_grid=on_grid)
    reader.start()

    # 等待 KFS 触发 (主线程等待)
    event.wait()

    grid_4x3 = received_grid[0]
    print(f"\n[KFS] 收到网格 (连续 3 帧一致):")
    print(grid_4x3)

    # 关闭 KFS 读取器 (释放串口)
    reader.stop()

    # 运行 pipeline
    result = process_grid(grid_4x3, args.side, verbose=False)

    if "error" in result:
        print(f"[ERROR] 规划失败: {result['error']}")
        sys.exit(1)

    # ── 输出 1: R1 货架位置编码 ──
    r1_ids = result.get("r1_ids", [])
    print(f"\n===== R1 货架编码 =====")
    print(f"{r1_ids}")

    # ── 输出 2: R2 command 二进制数组 ──
    binary = result.get("binary_commands", b"")

    # 构建发送帧: [header(1B) | payload(N bytes) | tail(1B)]
    header = 0x11 if args.side == "red" else 0x22
    tail = 0x99
    frame = bytearray([header]) + bytearray(binary)
    # 补齐到指定长度 (用最后一个字节填充)
    if args.frame_len > 0 and len(frame) < args.frame_len:
        pad_byte = frame[-1]
        frame += bytearray([pad_byte]) * (args.frame_len - len(frame))
        frame[-1] = tail

    print(f"\n===== R2 指令二进制 =====")
    print(f"{fmt_hex(frame)}")
    pad_info = f" (补齐至 {args.frame_len} 字节)" if args.frame_len > 0 else ""
    print(f"(帧共 {len(frame)} 字节, payload {len(binary)} 字节{pad_info})")

    # 复用同一串口发送
    try:
        import serial
        ser = serial.Serial(args.port, args.baud, timeout=1.0)
        ser.write(frame)
        ser.close()
        print(f"[KFS] 已通过 {args.port} 发送 R2 指令帧")
    except Exception as e:
        print(f"[KFS] 串口发送失败: {e}")

    print(f"\n[KFS] 完成, 退出")


if __name__ == "__main__":
    main()
