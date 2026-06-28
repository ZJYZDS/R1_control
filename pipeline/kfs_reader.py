"""KFS 串口帧读取器 — 从串口读取 14 字节帧, 3 帧一致后输出 4×3 网格."""

import time
import threading
import numpy as np

try:
    import serial
except ImportError:
    serial = None


MAP_FRAME_HEADER = 0x5A
MAP_FRAME_TAIL = 0xA5
MAP_FRAME_LEN = 14  # header(1) + 12×uint8 + tail(1)


def parse_frame(frame_bytes):
    """解析 14 字节 KFS 帧 → 12 个 uint8 值列表.

    Frame: [HEADER(0x5A) | 12×uint8 | TAIL(0xA5)]
    """
    if len(frame_bytes) != MAP_FRAME_LEN:
        return None
    if frame_bytes[0] != MAP_FRAME_HEADER:
        return None
    if frame_bytes[-1] != MAP_FRAME_TAIL:
        return None
    return list(frame_bytes[1:13])


def raw_to_grid(raw_12):
    """12 个 uint8 → 4×3 numpy 数组."""
    return np.array(raw_12, dtype=np.int32).reshape(4, 3)


class KfsSerialReader:
    """串口 KFS 帧读取器 (后台线程).

    连续 3 帧一致后, 回调 on_grid(grid_4x3).
    """

    def __init__(self, port, baud=115200, on_grid=None):
        self.port = port
        self.baud = baud
        self.on_grid = on_grid
        self._ser = None
        self._buffer = bytearray()
        self._last_raw = None
        self._consistent_count = 0
        self._running = False
        self._thread = None

    def start(self):
        if serial is None:
            raise ImportError("pyserial not installed")
        self._running = True
        self._connect()
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._ser and self._ser.is_open:
            self._ser.close()

    def _connect(self):
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=1.0)
            print(f"[KFS] 串口 {self.port} 已打开")
        except Exception as e:
            print(f"[KFS] 串口 {self.port} 打开失败: {e}")
            self._ser = None

    def _recv_loop(self):
        while self._running:
            if not self._ser or not self._ser.is_open:
                time.sleep(0.5)
                self._connect()
                continue
            try:
                waiting = self._ser.in_waiting
                if waiting > 0:
                    raw = self._ser.read(waiting)
                    self._buffer.extend(raw)

                while len(self._buffer) >= MAP_FRAME_LEN:
                    pos = self._buffer.find(bytes([MAP_FRAME_HEADER]))
                    if pos < 0:
                        self._buffer.clear()
                        break
                    if pos > 0:
                        del self._buffer[:pos]
                    if len(self._buffer) < MAP_FRAME_LEN:
                        break
                    if self._buffer[MAP_FRAME_LEN - 1] == MAP_FRAME_TAIL:
                        self._process_frame(bytes(self._buffer[:MAP_FRAME_LEN]))
                        del self._buffer[:MAP_FRAME_LEN]
                    else:
                        del self._buffer[0]
            except Exception as e:
                print(f"[KFS] 读错误: {e}")
                self._ser = None
            time.sleep(0.005)

    def _process_frame(self, frame):
        raw = parse_frame(frame)
        if raw is None:
            return

        if self._last_raw is not None and raw == self._last_raw:
            self._consistent_count += 1
        else:
            self._consistent_count = 1
        self._last_raw = raw

        if self._consistent_count >= 3:
            self._consistent_count = 0
            grid = raw_to_grid(raw)
            print(f"[KFS] 连续3帧一致, 触发规划\n{grid}")
            if self.on_grid:
                self.on_grid(grid)
