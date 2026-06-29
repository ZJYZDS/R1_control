#!/bin/bash
# R1 控制总管道启动脚本
#
# Usage:
#   ./start.sh                          # 默认串口模式, 红方
#   ./start.sh --test-scene 1 --blue    # 测试场景 1, 蓝方

set -e

# ── 默认值 ──
MODE="serial"
SERIAL_PORT="/dev/ttyUSB0"
TEST_SCENE=""
SIDE="blue"

# ── 参数解析 (放在最前面) ──
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
        --red)
            SIDE="red"
            shift
            ;;
        --blue)
            SIDE="blue"
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

# ── Ctrl+C 关闭所有窗口 ──
cleanup() {
    echo ""
    echo "[start.sh] 正在关闭所有窗口..."
    # 杀掉子终端里的 bash 进程 → 窗口自动关闭
    for f in /tmp/r1_livox.pid /tmp/r1_mapping.pid; do
        [[ -f "$f" ]] && kill $(cat "$f") 2>/dev/null
        rm -f "$f"
    done
    kill $(jobs -p) 2>/dev/null || true
    exit 0
}
trap cleanup SIGINT SIGTERM

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

# ── 启动 livox_ros_driver2 ──
echo "[start.sh] 启动 livox_ros_driver2 ..."
gnome-terminal -- bash -c "echo \$\$ > /tmp/r1_livox.pid; roslaunch livox_ros_driver2 msg_MID360.launch; exec bash" &
sleep 2

# ── 启动 fast_livo mapping ──
echo "[start.sh] 启动 fast_livo mapping ..."
gnome-terminal -- bash -c "echo \$\$ > /tmp/r1_mapping.pid; roslaunch fast_livo mapping_mid360.launch; exec bash" &
sleep 1

# ── 启动 waypoint executor (前台, Ctrl+C 终止) ──
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
