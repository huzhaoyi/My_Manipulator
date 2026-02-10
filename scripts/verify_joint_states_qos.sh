#!/usr/bin/env bash
# 验证 /joint_states QoS：sealien_payload_grasp 订阅应为 BEST_EFFORT + VOLATILE，与 MoveIt 一致。
# 使用：先启动机器人 (./start_robot.sh)，再在另一终端运行本脚本。

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$WS_DIR/install/setup.bash" 2>/dev/null || true

echo "=== 1. /joint_states 话题信息 (QoS) ==="
if ! ros2 topic info /joint_states -v 2>/dev/null; then
  echo "错误: /joint_states 未发布。请先启动机器人: ./start_robot.sh"
  exit 1
fi

echo ""
echo "=== 2. 检查 sealien_payload_grasp 订阅端 QoS 是否为 BEST_EFFORT + VOLATILE ==="
OUT=$(ros2 topic info /joint_states -v 2>/dev/null)
if echo "$OUT" | grep -A 20 "sealien_payload_grasp" | grep -q "BEST_EFFORT" && echo "$OUT" | grep -A 20 "sealien_payload_grasp" | grep -q "VOLATILE"; then
  echo "通过: sealien_payload_grasp 订阅为 BEST_EFFORT + VOLATILE"
else
  echo "未通过或未找到 sealien_payload_grasp 订阅。请确认 sealien_payload_grasp 已启动并订阅 /joint_states。"
  echo "$OUT" | grep -A 15 "sealien_payload_grasp" || true
fi

echo ""
echo "=== 3. /joint_states 发布频率 (约 5 秒) ==="
timeout 5 ros2 topic hz /joint_states 2>/dev/null || true
echo "验证结束。"
