#!/bin/bash

# M5机械臂启动脚本
# 用于启动MoveIt和缆绳抓取节点，连接到M5机械臂

echo "=========================================="
echo "M5机械臂启动脚本"
echo "=========================================="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 清理函数
cleanup() {
    echo ""
    echo "正在清理资源..."
    
    # 清理所有相关的ROS2节点（如果存在）
    if command -v ros2 &> /dev/null && [ -f "install/setup.bash" ]; then
        source install/setup.bash 2>/dev/null
        ROS2_NODES=$(ros2 node list 2>/dev/null | grep -E "(m5_grasp|move_group|ros2_control)" || true)
        if [ -n "$ROS2_NODES" ]; then
            echo "  检测到以下ROS2节点仍在运行:"
            echo "$ROS2_NODES" | sed 's/^/    /'
            echo "  提示: ros2 launch 会自动清理其启动的节点"
        fi
    fi
    
    echo "清理完成"
    exit 0
}

# 注册信号处理和退出清理
trap cleanup SIGINT SIGTERM EXIT

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    if [ -f /opt/ros/humble/setup.bash ]; then
        echo "正在加载ROS2 Humble环境..."
        source /opt/ros/humble/setup.bash
    else
        echo -e "${RED}错误: 未检测到ROS2环境${NC}"
        exit 1
    fi
fi

# ========== 关键修复：限制 FastDDS 只使用指定网卡 ==========
# 问题：多网卡环境下，action 回包可能走错网卡，导致客户端收不到响应
# 解决：通过 FastDDS 配置文件限制只使用 ens33 (192.168.100.44) 和 lo 网卡
# 注意：如果遇到 UDP 通信问题，可以设置 DISABLE_FASTDDS_WHITELIST=1 来禁用接口限制
if [ "${DISABLE_FASTDDS_WHITELIST:-0}" = "1" ]; then
    echo -e "${YELLOW}已禁用 FastDDS 接口限制（使用所有网卡）${NC}"
    unset FASTRTPS_DEFAULT_PROFILES_FILE
elif [ -f "$(dirname "$0")/fastdds_profile.xml" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$(dirname "$0")/fastdds_profile.xml"
    echo -e "${GREEN}已设置 FastDDS 配置文件，限制使用 ens33 和 lo 网卡${NC}"
else
    echo -e "${YELLOW}警告: 未找到 fastdds_profile.xml，使用默认网络配置${NC}"
fi

# 检查工作空间是否已编译
if [ ! -d "install" ]; then
    echo -e "${RED}错误: 工作空间未编译${NC}"
    echo "请先运行: ./install_deps.sh 或 colcon build"
    exit 1
fi

# Source工作空间
echo "正在加载工作空间环境..."
source install/setup.bash

# 禁用 FastDDS 共享内存传输以避免警告（可选）
# 如果遇到共享内存端口冲突警告，取消下面的注释
# export FASTRTPS_DEFAULT_PROFILES_FILE=""
# export RMW_FASTRTPS_USE_QOS_FROM_XML=0

# 显示帮助
show_help() {
    echo ""
    echo "用法: $0 [命令]"
    echo ""
    echo "命令:"
    echo "  grasp     启动m5_grasp抓取节点 (默认)"
    echo "  rviz      启动RViz可视化演示"
    echo "  sim       启动模拟器 (Web界面)"
    echo "  all       启动模拟器 + 抓取节点"
    echo "  help      显示此帮助"
    echo ""
    echo "示例:"
    echo "  $0              # 启动抓取节点"
    echo "  $0 grasp        # 启动抓取节点"
    echo "  $0 rviz         # 启动RViz演示"
    echo "  $0 sim          # 启动模拟器"
    echo "  $0 all          # 启动模拟器和抓取节点"
    echo ""
}

# 根据参数选择启动模式
case "${1:-grasp}" in
    grasp)
        LAUNCH_FILE="m5_grasp.launch.py"
        echo -e "${GREEN}启动模式: grasp (缆绳抓取)${NC}"
        echo ""
        echo "提示: 确保M5机械臂已启动并监听 192.168.100.38:7001"
        echo "提示: 接收话题: /cable_pose_with_yaw"
        echo "提示: 按 Ctrl+C 停止"
        echo ""
        ros2 launch m5_bringup $LAUNCH_FILE
        EXIT_CODE=$?
        if [ $EXIT_CODE -ne 0 ]; then
            echo -e "${YELLOW}警告: ros2 launch 退出，退出码: $EXIT_CODE${NC}"
        fi
        ;;
    
    rviz)
        LAUNCH_FILE="rviz.launch.py"
        echo -e "${GREEN}启动模式: rviz (RViz可视化)${NC}"
        echo ""
        echo "提示: 按 Ctrl+C 停止"
        echo ""
        ros2 launch m5_bringup $LAUNCH_FILE
        EXIT_CODE=$?
        if [ $EXIT_CODE -ne 0 ]; then
            echo -e "${YELLOW}警告: ros2 launch 退出，退出码: $EXIT_CODE${NC}"
        fi
        ;;
    
    sim)
        echo -e "${GREEN}启动模式: sim (模拟器)${NC}"
        echo ""
        echo "提示: Web界面地址: http://localhost:8080"
        echo "提示: 按 Ctrl+C 停止"
        echo ""
        cd simulator
        python3 robot_simulator.py
        ;;
    
    all)
        echo -e "${GREEN}启动模式: all (模拟器 + 抓取节点)${NC}"
        echo ""
        echo "步骤1: 启动模拟器 (后台)..."
        cd simulator
        python3 robot_simulator.py &
        SIM_PID=$!
        cd ..
        sleep 2
        
        if ! kill -0 $SIM_PID 2>/dev/null; then
            echo -e "${RED}错误: 模拟器启动失败${NC}"
            exit 1
        fi
        
        echo "步骤2: 启动抓取节点..."
        echo ""
        echo "提示: Web界面地址: http://localhost:8080"
        echo "提示: 按 Ctrl+C 停止所有进程"
        echo ""
        
        # 重新定义cleanup以清理模拟器
        trap "kill $SIM_PID 2>/dev/null; cleanup" SIGINT SIGTERM EXIT
        
        ros2 launch m5_bringup m5_grasp.launch.py
        ;;
    
    help|-h|--help)
        show_help
        exit 0
        ;;
    
    *)
        echo -e "${RED}错误: 未知命令 '$1'${NC}"
        show_help
        exit 1
        ;;
esac
