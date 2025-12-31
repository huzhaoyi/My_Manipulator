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

# 禁用 FastDDS 共享内存传输以避免警告（可选）
# 如果遇到共享内存端口冲突警告，取消下面的注释
# export FASTRTPS_DEFAULT_PROFILES_FILE=""
# export RMW_FASTRTPS_USE_QOS_FROM_XML=0

# 检查S3机械臂连接（可选）
echo "提示: 确保S3机械臂已启动并监听 192.168.32.150:7001"
echo ""

# 根据参数选择启动哪个launch文件
LAUNCH_FILE="demo.launch.py"  # 默认启动demo.launch.py

if [ "$1" == "demo_moveit" ] || [ "$1" == "moveit" ]; then
    LAUNCH_FILE="demo_moveit.launch.py"
    echo "启动模式: demo_moveit (包含demo_moveit节点)"
elif [ "$1" == "demo" ] || [ "$1" == "" ]; then
    LAUNCH_FILE="demo.launch.py"
    echo "启动模式: demo (包含RViz)"
else
    echo "用法: $0 [demo|demo_moveit]"
    echo "  demo        - 启动demo.launch.py (包含RViz，默认)"
    echo "  demo_moveit - 启动demo_moveit.launch.py (包含demo_moveit节点)"
    exit 1
fi

# 启动MoveIt
echo "正在启动MoveIt ($LAUNCH_FILE)..."
echo "提示: 硬件接口将自动连接到S3机械臂"
echo ""

ros2 launch m5_configure $LAUNCH_FILE
