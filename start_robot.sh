#!/bin/bash

# M5机械臂启动脚本
# 用于启动MoveIt和硬件接口，连接到S3机械臂

echo "=========================================="
echo "M5机械臂启动脚本"
echo "=========================================="

# 检查工作空间是否已编译
if [ ! -d "install" ]; then
    echo "错误: 工作空间未编译，请先运行: colcon build"
    exit 1
fi

# Source工作空间
echo "正在加载工作空间环境..."
source install/setup.bash

# 检查S3机械臂连接（可选）
echo "提示: 确保S3机械臂已启动并监听 192.168.100.50:7001"
echo ""

# 启动MoveIt和RViz
echo "正在启动MoveIt和RViz..."
echo "提示: 硬件接口将自动连接到S3机械臂"
echo ""

ros2 launch m5_configure demo.launch.py
