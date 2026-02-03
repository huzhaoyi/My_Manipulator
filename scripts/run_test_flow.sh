#!/bin/bash
# 运行「发布缆绳坐标 + 监听 /grasp_state」测试，观察抓取流程。
# 用法: 先在一个终端运行 ./start_robot.sh，再在本脚本所在目录或工作空间根目录执行:
#   ./scripts/run_test_flow.sh [--once]
# 或从工作空间根目录: bash scripts/run_test_flow.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$WS_ROOT"

if [ -f install/setup.bash ]; then
    source install/setup.bash
else
    echo "请先在工作空间根目录执行: colcon build && source install/setup.bash"
    exit 1
fi

echo "=========================================="
echo "测试: 发布缆绳坐标并观察 /grasp_state 流程"
echo "=========================================="
echo "请确保已在另一终端运行: ./start_robot.sh"
echo "默认将每 3 秒发布一次 (0.25, 0, 0.25) yaw=0，按 Ctrl+C 停止"
echo ""

python3 "$SCRIPT_DIR/publish_cable_pose_test.py" "$@"
