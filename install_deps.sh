#!/bin/bash
#
# Sealien CtrlPilot Payload - 一键安装依赖脚本
#
# 使用方法:
#   chmod +x install_deps.sh
#   ./install_deps.sh
#

set -e

echo "=========================================="
echo "Sealien CtrlPilot Payload 依赖安装脚本"
echo "=========================================="
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查是否以root运行
if [ "$EUID" -eq 0 ]; then
    echo -e "${RED}请不要以root用户运行此脚本${NC}"
    exit 1
fi

# 检查ROS2 Humble是否安装
if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo -e "${RED}错误: 未检测到ROS2 Humble${NC}"
    echo "请先安装ROS2 Humble: https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

echo -e "${GREEN}✓ 检测到ROS2 Humble${NC}"
echo ""

# 更新包列表
echo ">>> 更新包列表..."
sudo apt update

echo ""
echo ">>> 安装ROS2包依赖..."
sudo apt install -y \
    ros-humble-moveit \
    ros-humble-moveit-task-constructor-core \
    ros-humble-moveit-task-constructor-msgs \
    ros-humble-controller-manager \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-rviz2 \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers

echo -e "${GREEN}✓ ROS2包安装完成${NC}"
echo ""

echo ">>> 安装系统库依赖..."
sudo apt install -y \
    libspdlog-dev \
    libnlopt-cxx-dev \
    build-essential \
    cmake

echo -e "${GREEN}✓ 系统库安装完成${NC}"
echo ""

echo ">>> 安装Python依赖..."
pip3 install --user -r requirements.txt 2>/dev/null || {
    echo -e "${YELLOW}提示: pip安装失败，尝试使用apt安装...${NC}"
    sudo apt install -y python3-numpy python3-matplotlib
}

echo -e "${GREEN}✓ Python依赖安装完成${NC}"
echo ""

# 编译工作空间
echo ">>> 编译工作空间..."
source /opt/ros/humble/setup.bash

# 检查是否在正确的目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

if [ ! -f "src/sealien_payload_grasp/package.xml" ]; then
    echo -e "${RED}错误: 请在项目根目录运行此脚本${NC}"
    exit 1
fi

colcon build

echo -e "${GREEN}✓ 编译完成${NC}"
echo ""

echo "=========================================="
echo -e "${GREEN}安装完成!${NC}"
echo "=========================================="
echo ""
echo "使用方法:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source install/setup.bash"
echo "  ros2 launch sealien_payload_bringup sealien_payload_grasp.launch.py"
echo ""
echo "或使用启动脚本:"
echo "  ./start_robot.sh grasp"
echo ""
