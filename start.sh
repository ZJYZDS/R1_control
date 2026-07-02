#!/bin/bash
# R1 控制总管道启动脚本
#
# Usage:
#   ./start.sh --serial --red              # 串口模式 (端口从 YAML 读取)
#   ./start.sh --test-scene 1 --blue       # 测试场景 1, 蓝方
#
# 串口 (均由 r1_graph_config.yaml 配置):
#   KFS 串口:     接收 4×3 货架网格 → 触发 pipeline 规划
#   命令发送串口:  规划完成后发送 /path/commands 二进制数组 (header: red→0x11, blue→0x22)
#   底盘控制串口:  航点执行时发送 dx/dy/dtheta 运动指令

set -e

# ── 默认值 ──
MODE="serial"
TEST_SCENE=""
SIDE="blue"

# ── 参数解析 ──
while [[ $# -gt 0 ]]; do
    case "$1" in
        --serial)
            MODE="serial"
            shift
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

# ── 读取串口配置 ──
KFS_PORT=$(python3 -c "
import yaml
with open('config/r1_graph_config.yaml') as f:
    cfg = yaml.safe_load(f)
print(cfg.get('kfs_serial', {}).get('port', '/dev/ttyACM0'))
")
CMD_PORT=$(python3 -c "
import yaml
with open('config/r1_graph_config.yaml') as f:
    cfg = yaml.safe_load(f)
print(cfg.get('command_serial', {}).get('port', '/dev/ttyUSB1'))
")
CHASSIS_PORT=$(python3 -c "
import yaml
with open('config/r1_graph_config.yaml') as f:
    cfg = yaml.safe_load(f)
print(cfg.get('chassis_serial', {}).get('port', '/dev/ttyACM0'))
")

# ── Ctrl+C 关闭所有窗口 ──
cleanup() {
    echo ""
    echo "[start.sh] 正在关闭所有窗口..."
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
    echo "  KFS serial:    ${KFS_PORT}"
    echo "  Cmd serial:    ${CMD_PORT}"
    echo "  Chassis:       ${CHASSIS_PORT}"
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
    echo "[start.sh] 启动 waypoint executor (串口模式)..."
    python3 executor/waypoint_executor.py --"$SIDE"
fi
