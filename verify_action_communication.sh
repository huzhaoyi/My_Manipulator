#!/bin/bash

# Action 通信验证脚本
# 用于诊断 action client 回包丢失问题

echo "=========================================="
echo "Action 通信验证脚本"
echo "=========================================="
echo ""

# 检查 ROS2 环境
if [ -z "$ROS_DISTRO" ]; then
    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    else
        echo "错误: 未检测到 ROS2 环境"
        exit 1
    fi
fi

export ROS_DOMAIN_ID=42

# 加载工作空间
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
fi

# 加载 FastDDS 配置（如果存在）
if [ -f "fastdds_profile.xml" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$(pwd)/fastdds_profile.xml"
    echo "✓ 已加载 FastDDS 配置文件"
fi

echo ""
echo "A. 验证 Action 名称"
echo "----------------------------------------"
ACTION_NAME="execute_task_solution"
echo "查找 action: $ACTION_NAME"
ros2 action list | grep -i execute || echo "  ⚠️ 未找到 execute_task_solution action"
echo ""

echo "B. 检查 Action Server 信息"
echo "----------------------------------------"
if ros2 action list | grep -q execute_task_solution; then
    echo "执行: ros2 action info $ACTION_NAME"
    ros2 action info $ACTION_NAME 2>&1 | head -30
    echo ""
    echo "检查 server 和 client 连接状态:"
    ros2 action info $ACTION_NAME 2>&1 | grep -E "Action|Server|Client" || echo "  (无详细信息)"
else
    echo "  ⚠️ Action server 未运行，请先启动系统"
    echo "  提示: 运行 ./start_robot.sh 启动系统"
fi
echo ""

echo "C. 检查网络接口"
echo "----------------------------------------"
echo "当前网络接口:"
ip a | grep -E "inet |^[0-9]+:" | grep -v "127.0.0.1"
echo ""

echo "D. 检查 DDS 配置"
echo "----------------------------------------"
echo "RMW 实现: ${RMW_IMPLEMENTATION:-未设置（默认: rmw_fastrtps_cpp）}"
if [ -n "$FASTRTPS_DEFAULT_PROFILES_FILE" ]; then
    echo "FastDDS 配置文件: $FASTRTPS_DEFAULT_PROFILES_FILE"
    if [ -f "$FASTRTPS_DEFAULT_PROFILES_FILE" ]; then
        echo "  ✓ 配置文件存在"
    else
        echo "  ⚠️ 配置文件不存在"
    fi
else
    echo "FastDDS 配置文件: 未设置"
fi
echo ""

echo "E. 检查 Action Server 运行状态"
echo "----------------------------------------"
ACTION_INFO=$(ros2 action info /execute_task_solution 2>&1)
echo "$ACTION_INFO"
echo ""

if echo "$ACTION_INFO" | grep -q "Action servers: 0"; then
    echo "  ⚠️ Action server 未运行！"
    echo "  请先启动系统: ./start_robot.sh"
    echo ""
    echo "  如果系统已启动但仍显示 servers: 0，可能原因："
    echo "    1. move_group 节点未启动"
    echo "    2. ROS_DOMAIN_ID 不匹配"
    echo "    3. DDS 通信问题（多网卡/防火墙）"
else
    echo "  ✓ Action server 正在运行"
    echo ""
    echo "F. 测试 Action 通信"
    echo "----------------------------------------"
    echo "提示: 可以尝试发送测试 goal："
    echo "  ros2 action send_goal /execute_task_solution moveit_task_constructor_msgs/action/ExecuteTaskSolution \"{}\" --feedback"
    echo ""
    echo "如果 CLI 测试也卡住，说明是 DDS 网络配置问题"
fi
echo ""

echo "=========================================="
echo "验证完成"
echo "=========================================="
