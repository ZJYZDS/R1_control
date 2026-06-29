#!/bin/bash
# R1 控制总管道启动脚本
#
# Usage:
#   ./start.sh                          # 默认 --serial /dev/ttyUSB0
#   ./start.sh --serial /dev/ttyUSB1    # 指定串口
#   ./start.sh --test-scene 1           # 测试场景 1
#   ./start.sh --test-scene 2 --side red
#
# Options:
#   --serial      KFS 串口设备路径 (默认 /dev/ttyUSB0)
#   --test-scene  测试场景编号 (1 或 2)
#   --side        阵营: blue / red (默认 blue)

set -e

# ── 默认值 ──
MODE="serial"
SERIAL_PORT="/dev/ttyUSB0"
TEST_SCENE=""
SIDE="red"

# ── 参数解析 ──
while [[ $# -gt 0 ]]; do
    case "$1" in
        --serial)
            MODE="serial"
            SERIAL_PORT="$2"
            shift 2
            ;;
        --test-scene)
            MODE="test"
            TEST_SCENE="$2"
            shift 2
            ;;
        blue|red)
            SIDE="$1"
            shift
            ;;
        *)
            shift
            ;;
    esac
done

# ── 确定脚本目录 ──
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

echo "========================================"
echo "  R1 Control Pipeline"
echo "  Mode:      ${MODE}"
echo "  Side:      ${SIDE}"
if [ "$MODE" = "test" ]; then
    echo "  Scene:     ${TEST_SCENE}"
else
    echo "  Serial:    ${SERIAL_PORT}"
fi
echo "========================================"

if [ "$MODE" = "test" ]; then
    if [ -z "$TEST_SCENE" ]; then
        echo "ERROR: --test-scene requires a scene number (1 or 2)"
        exit 1
    fi
    echo "[start.sh] 启动 waypoint executor (test scene ${TEST_SCENE})..."
    python3 executor/waypoint_executor.py --test-scene="$TEST_SCENE" --"$SIDE"
else
    echo "[start.sh] 启动 waypoint executor (KFS serial)..."
    python3 executor/waypoint_executor.py --"$SIDE"
fi
