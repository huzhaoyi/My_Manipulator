#!/bin/bash

# M5机械臂启动脚本
# 用于启动MoveIt和缆绳抓取节点，连接到M5机械臂

# 切换到脚本所在目录，保证 install/ 等相对路径正确（任意目录执行 ./start_robot.sh 均可）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR" || exit 1

echo "=========================================="
echo "M5机械臂启动脚本"
echo "=========================================="
echo "工作目录: $SCRIPT_DIR"

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
        ROS2_NODES=$(ros2 node list 2>/dev/null | grep -E "(sealien_payload_grasp|move_group|ros2_control)" || true)
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

#export ROS_DOMAIN_ID=42

# 检查工作空间是否已编译
if [ ! -d "install" ]; then
    echo -e "${RED}错误: 工作空间未编译${NC}"
    echo "请先运行: ./install_deps.sh 或 colcon build"
    exit 1
fi

# Source工作空间
echo "正在加载工作空间环境..."
source install/setup.bash

# 显示帮助
show_help() {
    echo ""
    echo "用法: $0 [命令]"
    echo ""
    echo "命令:"
    echo "  grasp     启动sealien_payload_grasp抓取节点 (默认)"
    echo "  rviz      启动RViz可视化演示"
    echo "  sim       启动模拟器 (Web界面)"
    echo "  all       启动模拟器 + 抓取节点"
    echo "  tf2_echo  用本脚本相同环境查看 world_link->sonar_link（需在另一终端先运行 grasp）"
    echo "  help      显示此帮助"
    echo ""
    echo "示例:"
    echo "  $0              # 启动抓取节点"
    echo "  $0 grasp        # 启动抓取节点"
    echo "  $0 tf2_echo     # 查看手眼 TF（先另开终端运行 $0 grasp）"
    echo "  $0 rviz         # 启动RViz演示"
    echo "  $0 sim          # 启动模拟器"
    echo "  $0 all          # 启动模拟器和抓取节点"
    echo ""
}

# 根据参数选择启动模式
case "${1:-grasp}" in
    grasp)
        LAUNCH_FILE="sealien_payload_grasp.launch.py"
        echo -e "${GREEN}启动模式: grasp (缆绳抓取)${NC}"
        echo ""
        echo "提示: 确保M5机械臂已启动并监听配置的 robot_ip:robot_port（见 src/m5_moveit_config/config/m5.ros2_control.xacro）"
        echo "提示: 接收话题: /cable_pose_with_yaw"
        echo "提示: 按 Ctrl+C 停止"
        echo ""
        ros2 launch sealien_payload_bringup $LAUNCH_FILE
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
        ros2 launch sealien_payload_bringup $LAUNCH_FILE
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
    
    tf2_echo)
        echo -e "${GREEN}用本脚本相同环境查看 world_link -> sonar_link${NC}"
        echo "请确保已在另一终端运行: $0 grasp"
        echo ""
        ros2 run tf2_ros tf2_echo world_link sonar_link
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
        
        ros2 launch sealien_payload_bringup m5_grasp.launch.py
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
