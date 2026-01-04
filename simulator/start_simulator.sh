#!/bin/bash

# S3机械臂上位机启动脚本

echo "=========================================="
echo "S3机械臂上位机"
echo "=========================================="

cd "$(dirname "$0")"

# 检查Python3
if ! command -v python3 &> /dev/null; then
    echo "错误: 未找到python3，请先安装Python3"
    exit 1
fi

# 检查ROS2环境（可选）
if [ -f "../install/setup.bash" ]; then
    echo "检测到ROS2环境，正在加载..."
    source ../install/setup.bash
fi

# 检查端口是否被占用
check_port() {
    local port=$1
    if lsof -Pi :$port -sTCP:LISTEN -t >/dev/null 2>&1 ; then
        echo "警告: 端口 $port 已被占用"
        return 1
    fi
    return 0
}

echo "检查端口..."
check_port 7001 || echo "  - UDP端口7001可能被占用"
check_port 8081 || echo "  - Web端口8081可能被占用"

# 获取本机IP地址
get_local_ip() {
    local ip=$(hostname -I | awk '{print $1}')
    if [ -z "$ip" ]; then
        ip=$(ip route get 8.8.8.8 2>/dev/null | awk '{print $7; exit}')
    fi
    echo "$ip"
}

LOCAL_IP=$(get_local_ip)
echo ""
echo "本机IP地址: $LOCAL_IP"
echo ""

# 启动上位机
echo "正在启动上位机..."
echo ""
python3 robot_simulator.py
