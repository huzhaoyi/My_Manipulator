#!/bin/bash

# M5机械臂启动脚本
# 用于启动MoveIt和硬件接口，连接到S3机械臂

echo "=========================================="
echo "M5机械臂启动脚本"
echo "=========================================="

# 清理函数
cleanup() {
    echo ""
    echo "正在清理资源..."
    
    # 如果是在all模式下，清理后台进程
    if [ -n "$PLANNING_PID" ] && kill -0 $PLANNING_PID 2>/dev/null; then
        echo "  停止 m5_planning 节点 (PID: $PLANNING_PID)..."
        kill -TERM $PLANNING_PID 2>/dev/null
        sleep 2
        if kill -0 $PLANNING_PID 2>/dev/null; then
            kill -9 $PLANNING_PID 2>/dev/null
        fi
    fi
    
    if [ -n "$GRASP_PID" ] && kill -0 $GRASP_PID 2>/dev/null; then
        echo "  停止 m5_grasp 节点 (PID: $GRASP_PID)..."
        kill -TERM $GRASP_PID 2>/dev/null
        sleep 2
        if kill -0 $GRASP_PID 2>/dev/null; then
            kill -9 $GRASP_PID 2>/dev/null
        fi
    fi
    
    # 清理所有相关的ROS2节点（如果存在）
    if command -v ros2 &> /dev/null && [ -f "install/setup.bash" ]; then
        source install/setup.bash 2>/dev/null
        ROS2_NODES=$(ros2 node list 2>/dev/null | grep -E "(m5_planning|m5_grasp|move_group|ros2_control)" || true)
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
if [ "$1" == "planning" ] || [ "$1" == "moveit" ]; then
    LAUNCH_FILE="m5_planning.launch.py"
    echo "启动模式: planning (包含m5_planning节点，接收/target_pose话题)"
    echo "正在启动MoveIt ($LAUNCH_FILE)..."
    echo "提示: 硬件接口将自动连接到S3机械臂"
    echo "提示: 按 Ctrl+C 停止"
    echo ""
    ros2 launch m5_bringup $LAUNCH_FILE
    EXIT_CODE=$?
    if [ $EXIT_CODE -ne 0 ]; then
        echo "警告: ros2 launch 退出，退出码: $EXIT_CODE"
    fi
elif [ "$1" == "grasp" ]; then
    LAUNCH_FILE="m5_grasp.launch.py"
    echo "启动模式: grasp (包含m5_grasp节点，接收/cable_pose话题，缆绳抓取)"
    echo "正在启动MoveIt ($LAUNCH_FILE)..."
    echo "提示: 硬件接口将自动连接到S3机械臂"
    echo "提示: 按 Ctrl+C 停止"
    echo ""
    ros2 launch m5_bringup $LAUNCH_FILE
    EXIT_CODE=$?
    if [ $EXIT_CODE -ne 0 ]; then
        echo "警告: ros2 launch 退出，退出码: $EXIT_CODE"
    fi
elif [ "$1" == "all" ] || [ "$1" == "both" ]; then
    echo "启动模式: all (同时启动planning和grasp节点)"
    echo "提示: 这将启动两个后台进程，分别运行planning和grasp节点"
    echo "提示: 硬件接口将自动连接到S3机械臂"
    echo ""
    echo "正在启动 m5_planning 节点（后台运行）..."
    nohup bash -c "source install/setup.bash && ros2 launch m5_bringup m5_planning.launch.py" > /tmp/m5_planning.log 2>&1 &
    PLANNING_PID=$!
    echo "  m5_planning 进程ID: $PLANNING_PID"
    echo "  日志文件: /tmp/m5_planning.log"
    
    sleep 3
    
    echo "正在启动 m5_grasp 节点（后台运行）..."
    nohup bash -c "source install/setup.bash && ros2 launch m5_bringup m5_grasp.launch.py" > /tmp/m5_grasp.log 2>&1 &
    GRASP_PID=$!
    echo "  m5_grasp 进程ID: $GRASP_PID"
    echo "  日志文件: /tmp/m5_grasp.log"
    
    echo ""
    echo "提示: 两个节点已启动，可以同时接收 /target_pose 和 /cable_pose 话题"
    echo "提示: 查看日志: tail -f /tmp/m5_planning.log 或 tail -f /tmp/m5_grasp.log"
    echo "提示: 停止节点: kill $PLANNING_PID $GRASP_PID"
    echo "提示: 按 Ctrl+C 停止当前脚本（节点将继续在后台运行）"
    echo ""
    # 保持脚本运行，直到用户按Ctrl+C
    while true; do 
        # 检查进程是否还在运行
        if ! kill -0 $PLANNING_PID 2>/dev/null; then
            echo "警告: m5_planning 节点已停止 (PID: $PLANNING_PID)"
            PLANNING_PID=""  # 清除已停止的PID
        fi
        if ! kill -0 $GRASP_PID 2>/dev/null; then
            echo "警告: m5_grasp 节点已停止 (PID: $GRASP_PID)"
            GRASP_PID=""  # 清除已停止的PID
        fi
        
        # 如果两个节点都停止了，退出循环
        if [ -z "$PLANNING_PID" ] && [ -z "$GRASP_PID" ]; then
            echo "所有节点已停止，退出脚本"
            break
        fi
        
        sleep 5
    done
elif [ "$1" == "rviz" ] || [ "$1" == "" ]; then
    LAUNCH_FILE="rviz.launch.py"
    echo "启动模式: rviz (包含RViz可视化)"
    echo "正在启动MoveIt ($LAUNCH_FILE)..."
    echo "提示: 硬件接口将自动连接到S3机械臂"
    echo "提示: 按 Ctrl+C 停止"
    echo ""
    ros2 launch m5_bringup $LAUNCH_FILE
    EXIT_CODE=$?
    if [ $EXIT_CODE -ne 0 ]; then
        echo "警告: ros2 launch 退出，退出码: $EXIT_CODE"
    fi
else
    echo "用法: $0 [rviz|planning|grasp|all]"
    echo "  rviz     - 启动rviz.launch.py (包含RViz可视化，默认)"
    echo "  planning - 启动m5_planning.launch.py (包含m5_planning节点，接收/target_pose)"
    echo "  grasp    - 启动m5_grasp.launch.py (包含m5_grasp节点，接收/cable_pose，缆绳抓取)"
    echo "  all      - 同时启动planning和grasp节点（两个终端窗口）"
    exit 1
fi
