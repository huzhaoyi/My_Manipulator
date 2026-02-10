# Sealien_CtrlPilot_Payload 项目结构说明

## 包结构

本项目（Sealien CtrlPilot Payload）已按 ROS2 最佳实践进行模块化重构，各包职责清晰：

### 核心包

#### `m5` - 机器人描述包
**职责**: 机器人URDF描述和mesh文件
- `urdf/` - URDF文件
- `meshes/` - 3D模型文件

#### `m5_hardware` - 硬件接口包
**职责**: 硬件抽象层，与物理机器人 UDP 通信
- `src/` - 硬件接口实现（UDP 收发、JSON 解析）
- `include/` - 头文件
- **机器人 IP/端口**：在 `m5_moveit_config/config/m5.ros2_control.xacro` 中配置 `robot_ip`、`robot_port`（机械臂）、`local_ip`、`local_port`（本机），修改后需重新编译。
- **夹爪直接目标**：读取 `/tmp/gripper_direct.txt`（JointGL/JointGR 弧度），合并到每周期 UDP 命令中发送
- **UDP 反馈落地**：每收到 UDP 反馈即把夹爪 (JointGL, JointGR) 写入 `/tmp/udp_feedback.txt`，供 sealien_payload_grasp 直接读

#### `m5_moveit_config` - MoveIt配置包
**职责**: MoveIt运动规划框架的配置
- `config/` - MoveIt配置文件
  - `m5.srdf` - SRDF配置
  - `kinematics.yaml` - 运动学求解器配置
  - `ompl_planning.yaml` - OMPL规划器配置
  - `joint_limits.yaml` - 关节限制
  - `moveit_controllers.yaml` - 控制器配置
  - `sensors_3d.yaml` - 传感器配置
  - `moveit.rviz` - RViz配置

### 功能包

#### `sealien_payload_grasp` - 抓取节点包（模块化架构）
**职责**: 基于MTC的抓取功能实现

**模块架构**:
```
sealien_payload_grasp_node (ROS2 Node, 450行)
    ├── TargetTracker      # 目标跟踪与稳定判定
    ├── GraspFSM           # 抓取状态机
    │   ├── ITaskRunner    # MTC任务执行接口
    │   └── IGripper       # 夹爪控制接口
    ├── TaskRunner         # MTC任务执行实现
    │   └── TaskFactory    # MTC任务构建
    └── GripperController  # 夹爪控制（直接 UDP：写目标到文件、读反馈文件，无轨迹；两阶段判稳）
```

**目录结构**:
```
sealien_payload_grasp/
  include/sealien_payload_grasp/
    fsm/                   # 状态机模块
      target_tracker.hpp   # 目标跟踪
      grasp_fsm.hpp        # 抓取FSM
    mtc/                   # MTC任务模块
      task_context.hpp     # 任务上下文
      task_factory.hpp     # 任务构建
      task_runner.hpp      # 任务执行
    hw/                    # 硬件接口
      gripper_interface.hpp
    execution/             # 执行模块
    planning/              # 规划模块
    utils/                 # 工具模块
    visualization/         # 可视化模块
    logging/               # 日志模块
  src/
    sealien_payload_grasp.cpp  # 主节点（450行）
    fsm/
    mtc/
    execution/
    planning/
    utils/
    visualization/
    logging/
```

### 系统包

#### `sealien_payload_bringup` - 启动文件包
**职责**: 系统启动和集成
- `launch/` - 所有 launch 文件
  - `sealien_payload_grasp.launch.py` - 抓取节点启动
  - `rviz.launch.py` - RViz可视化启动
  - `move_group.launch.py` - MoveGroup启动
  - 其他系统启动文件

#### `m5_msgs` - 消息定义包
**职责**: 自定义ROS2消息
- `msg/CablePoseWithYaw.msg` - 缆绳位置+yaw消息

## 依赖关系

```
sealien_payload_bringup
  ├── sealien_payload_grasp
  ├── m5_moveit_config
  ├── m5_hardware
  ├── m5_msgs
  └── m5

m5_grasp
  ├── m5_moveit_config
  ├── m5_msgs
  ├── moveit_task_constructor
  └── m5

m5_moveit_config
  └── m5
```

## 使用方式

### 启动抓取节点
```bash
./start_robot.sh grasp
# 或
ros2 launch sealien_payload_bringup sealien_payload_grasp.launch.py
```

### 启动RViz演示
```bash
./start_robot.sh demo
# 或
ros2 launch sealien_payload_bringup rviz.launch.py
```

### 启动模拟器
```bash
cd simulator
python3 robot_simulator.py
# 访问 http://localhost:8080
```

## 依赖

### 系统要求

- Ubuntu 22.04
- ROS2 Humble

### ROS2 包依赖

```bash
# MoveIt2
ros-humble-moveit
ros-humble-moveit-task-constructor-core
ros-humble-moveit-task-constructor-msgs

# 控制器
ros-humble-controller-manager
ros-humble-joint-state-publisher
ros-humble-joint-state-publisher-gui
ros-humble-robot-state-publisher

# 工具
ros-humble-xacro
ros-humble-rviz2
```

### 系统库依赖

```bash
# 日志库
libspdlog-dev

# TRAC-IK（可选）
libnlopt-cxx-dev
```

### 安装所有依赖

```bash
sudo apt update
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
    libspdlog-dev \
    libnlopt-cxx-dev
```

## 优势

1. **模块化**: 每个包职责单一，易于维护
2. **可扩展**: 新功能可以独立添加新包
3. **可重用**: 配置和节点可以在不同项目间复用
4. **可测试**: 模块化设计便于单元测试
5. **符合ROS2最佳实践**: 遵循ROS2包组织规范
