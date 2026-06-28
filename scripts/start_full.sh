#!/bin/bash
# One-click launch: path_planner + r1_control
# Prerequisite: Fast-LIVO already running (publishes /aft_mapped_to_init)
#
# Usage:
#   ./start_full.sh red                        # red side, /total_map topic
#   ./start_full.sh blue /dev/ttyUSB0           # blue side, serial KFS input
#   ./start_full.sh red --test-scene 1          # red side, test scene → /total_map

SIDE="blue"
MAP_SERIAL=""
TEST_SCENE=""

# ── 参数解析 ──
while [[ $# -gt 0 ]]; do
    case "$1" in
        --test-scene)
            TEST_SCENE="$2"
            shift 2
            ;;
        red|blue)
            SIDE="$1"
            shift
            ;;
        /dev/*)
            MAP_SERIAL="$1"
            shift
            ;;
        *)
            shift
            ;;
    esac
done

echo "SIDE=$SIDE  MAP_SERIAL=${MAP_SERIAL:-/total_map}  TEST_SCENE=${TEST_SCENE:-off}"

# ── 清理旧进程 ──
pkill -f "path_planner_main" 2>/dev/null
pkill -f "r1_polyline_planner" 2>/dev/null
pkill -f "publish_test_scene" 2>/dev/null
sleep 1

SETUP="source /opt/ros/noetic/setup.bash && source ~/roboncon2025_ws/devel/setup.bash"

# ── 测试场景: 持续发布 KFS 网格到 /total_map ──
if [ -n "$TEST_SCENE" ]; then
    echo "===== 测试场景 scene=$TEST_SCENE (发布到 /total_map) ====="

    PUBLISHER_SCRIPT="/home/zjy/R1_control/scripts/tools/publish_test_scene.py"
    gnome-terminal --title="TEST_SCENE_PUB [scene=$TEST_SCENE]" -- \
        bash -c "$SETUP && echo '发布场景 $TEST_SCENE → /total_map @ 2Hz'; python3 '$PUBLISHER_SCRIPT' '$TEST_SCENE'; exec bash" &
    sleep 2

    # 测试场景走 /total_map topic, 不传 map_serial_port
    MAP_SERIAL=""
fi

# ── 窗口 1: path_planner ──
EXTRA_ARG=""
[ -n "$MAP_SERIAL" ] && EXTRA_ARG="map_serial_port:=$MAP_SERIAL"

gnome-terminal --title="PATH_PLANNER [$SIDE]" -- \
    bash -c "$SETUP && roslaunch yolo_depth_pkg a_star_planner.launch side:=$SIDE $EXTRA_ARG; exec bash" &
sleep 3

# ── 窗口 2: r1_control ──
gnome-terminal --title="R1_CONTROL [$SIDE]" -- \
    bash -c "export R1_SCENARIO=$SIDE && $SETUP && roslaunch r1_control r1_full_pipeline.launch; exec bash" &

echo ""
echo "PATH_PLANNER + R1_CONTROL 已启动 (side=$SIDE, input=${MAP_SERIAL:-/total_map})"
if [ -n "$TEST_SCENE" ]; then
    echo "测试场景 $TEST_SCENE: /total_map ← publish_test_scene → path_planner → /R1_grap_pos → r1_control → /dev/ttyCHASSIS"
fi
