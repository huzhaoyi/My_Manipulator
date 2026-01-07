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

# 清理函数
cleanup() {
    echo ""
    echo "正在清理资源..."
    
    # 查找并终止robot_simulator.py进程
    PIDS=$(pgrep -f "robot_simulator.py")
    if [ -n "$PIDS" ]; then
        echo "  终止robot_simulator.py进程: $PIDS"
        kill -TERM $PIDS 2>/dev/null
        sleep 2
        # 如果还没停止，强制终止
        PIDS=$(pgrep -f "robot_simulator.py")
        if [ -n "$PIDS" ]; then
            echo "  强制终止进程: $PIDS"
            kill -9 $PIDS 2>/dev/null
        fi
    fi
    
    # 释放端口
    echo "  释放端口 7001 和 8081..."
    lsof -ti:7001 | xargs kill -9 2>/dev/null
    lsof -ti:8081 | xargs kill -9 2>/dev/null
    
    # 清理ROS2节点（如果存在）
    if command -v ros2 &> /dev/null && [ -f "../install/setup.bash" ]; then
        source ../install/setup.bash 2>/dev/null
        ROS2_NODES=$(ros2 node list 2>/dev/null | grep -E "(robot_web_interface|m5_)" || true)
        if [ -n "$ROS2_NODES" ]; then
            echo "  检测到ROS2节点，请手动检查:"
            echo "$ROS2_NODES"
        fi
    fi
    
    echo "清理完成"
    exit 0
}

# 注册信号处理
trap cleanup SIGINT SIGTERM EXIT

# 启动上位机
echo "正在启动上位机..."
echo ""
python3 robot_simulator.py

# 如果正常退出，也执行清理
cleanup
