#!/bin/bash

# S3机械臂模拟器启动脚本

echo "=========================================="
echo "S3机械臂模拟器"
echo "=========================================="

cd "$(dirname "$0")"

# 检查Python3
if ! command -v python3 &> /dev/null; then
    echo "错误: 未找到python3，请先安装Python3"
    exit 1
fi

# 启动模拟器
echo "正在启动模拟器..."
echo ""
python3 robot_simulator.py
