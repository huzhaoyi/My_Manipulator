#!/bin/bash
# 验证 joint_states QoS 配置是否生效
# 使用方法：在系统启动后运行此脚本

echo "=== 验证 joint_states QoS 配置 ==="
echo ""
echo "检查 /joint_states 话题的 QoS 配置："
echo ""

ros2 topic info /joint_states -v

echo ""
echo "=== 预期结果 ==="
echo "发布者应该使用：BEST_EFFORT + VOLATILE + KEEP_LAST"
echo "订阅者应该使用：BEST_EFFORT + VOLATILE（已匹配）"
echo ""
echo "如果发布者仍显示 RELIABLE + TRANSIENT_LOCAL，说明配置未生效"
