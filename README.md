# M5 Grasp Perception

M5 机械臂抓取系统 - 基于 ROS2 Humble + MoveIt2 + MoveIt Task Constructor

## 系统要求

- Ubuntu 22.04
- ROS2 Humble
- MoveIt2
- MoveIt Task Constructor

## 安装

### 方式1：一键安装（推荐）

```bash
cd ~/
git clone <repository_url> grasp_perception
cd grasp_perception

# 一键安装所有依赖并编译
./install_deps.sh

# Source 工作空间
source install/setup.bash
```

### 方式2：手动安装

#### 2.1 安装系统依赖

```bash
# ROS2 基础依赖
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
    ros-humble-ros2-control \
    ros-humble-ros2-controllers

# TRAC-IK 依赖（可选，推荐用于4DOF机械臂）
sudo apt install -y libnlopt-cxx-dev

# 日志库
sudo apt install -y libspdlog-dev

# Python 依赖
pip3 install -r requirements.txt
```

#### 2.2 编译

```bash
cd grasp_perception
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 安装 TRAC-IK（可选）

详见 [TRAC_IK_INSTALL.md](TRAC_IK_INSTALL.md)

## 包结构

| 包名 | 说明 |
|------|------|
| `m5` | 机器人URDF描述和mesh文件 |
| `m5_hardware` | 硬件接口（UDP通信） |
| `m5_moveit_config` | MoveIt配置 |
| `m5_grasp` | 抓取节点（MTC + FSM） |
| `m5_msgs` | 自定义消息 |
| `m5_bringup` | Launch文件 |
| `trac_ik` | TRAC-IK求解器 |

详细说明见 [PROJECT_STRUCTURE.md](PROJECT_STRUCTURE.md)

## 依赖关系

### ROS2 包依赖

```
rclcpp                          # ROS2 C++ 客户端库
rclcpp_action                   # ROS2 Action 支持
moveit_ros_planning_interface   # MoveIt 规划接口
moveit_core                     # MoveIt 核心库
moveit_task_constructor_core    # MTC 核心库
moveit_task_constructor_msgs    # MTC 消息
geometry_msgs                   # 几何消息
std_msgs                        # 标准消息
sensor_msgs                     # 传感器消息
control_msgs                    # 控制消息
trajectory_msgs                 # 轨迹消息
visualization_msgs              # 可视化消息
nav_msgs                        # 导航消息
shape_msgs                      # 形状消息
tf2_ros                         # TF2 ROS 接口
tf2_geometry_msgs               # TF2 几何消息转换
hardware_interface              # 硬件接口
pluginlib                       # 插件库
```

### 系统库依赖

```
spdlog                          # 高性能日志库
nlopt                           # 非线性优化（TRAC-IK）
```

### 内部库依赖

```
m5_grasp_node
    ├── m5_grasp_logging_lib     # 日志（spdlog封装）
    ├── m5_grasp_utils_lib       # 工具（参数管理、场景管理等）
    ├── m5_grasp_fsm_lib         # FSM（状态机、目标跟踪）
    │   └── m5_grasp_mtc_lib     # MTC（任务构建、执行）
    ├── m5_grasp_planning_lib    # 规划（IK工具、MTC任务创建）
    ├── m5_grasp_execution_lib   # 执行（夹爪、运动执行）
    └── m5_grasp_visualization_lib # 可视化（网页3D）
```

## 使用

### 启动抓取系统

```bash
# 方式1：使用启动脚本
./start_robot.sh grasp

# 方式2：使用launch文件
ros2 launch m5_bringup m5_grasp.launch.py
```

### 启动模拟器

```bash
cd simulator
python3 robot_simulator.py
# 访问 http://localhost:8080
```

### 启动RViz演示

```bash
ros2 launch m5_bringup rviz.launch.py
```

## 话题

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/cable_pose_with_yaw` | `m5_msgs/CablePoseWithYaw` | 输入 | 缆绳位置+yaw |
| `/emergency_stop` | `std_msgs/String` | 输入 | 急停信号 |
| `/grasp_state` | `std_msgs/String` | 输出 | 抓取状态 |
| `/joint_states` | `sensor_msgs/JointState` | 输出 | 关节状态 |

## 配置文件

| 文件 | 说明 |
|------|------|
| `src/m5_grasp/config/cable_grasp.yaml` | 抓取参数（含夹爪、FSM、轨迹等） |
| `src/m5_moveit_config/config/kinematics.yaml` | 运动学求解器 |
| `src/m5_moveit_config/config/ompl_planning.yaml` | OMPL规划器 |
| `src/m5_moveit_config/config/joint_limits.yaml` | 关节限制 |

## 夹爪控制与运行时文件

夹爪采用**直接 UDP** 方式（无轨迹、不经过 MoveGroup 轨迹执行）：

- **命令**：m5_grasp 将目标 (JointGL, JointGR) 写入 `/tmp/gripper_direct.txt`，m5_hardware 通信线程合并后发 UDP（axis5：0=闭合，-1100=全开）。
- **反馈**：m5_hardware 每收到 UDP 反馈即把夹爪关节写入 `/tmp/udp_feedback.txt`，m5_grasp 优先读该文件作为夹爪状态，不依赖 `/joint_states`，避免 MoveGroup 时间戳问题。
- **判稳**：先等「数值在变化」（运动开始），再等「数值不变」即判到位/夹到；参数见 `cable_grasp.yaml` 中 `gripper.*`。

| 临时文件 | 写入方 | 读取方 | 说明 |
|----------|--------|--------|------|
| `/tmp/gripper_direct.txt` | m5_grasp | m5_hardware | 夹爪目标 (JointGL JointGR) 弧度，合并进 UDP 发送 |
| `/tmp/udp_feedback.txt` | m5_hardware | m5_grasp | UDP 反馈中的夹爪 (JointGL JointGR)，供夹爪判稳 |

## 文档

| 文档 | 说明 |
|------|------|
| [PROJECT_STRUCTURE.md](PROJECT_STRUCTURE.md) | 项目结构说明 |
| [MOVEIT_CONFIG_GUIDE.md](MOVEIT_CONFIG_GUIDE.md) | MoveIt配置指南 |
| [TRAC_IK_INSTALL.md](TRAC_IK_INSTALL.md) | TRAC-IK安装说明 |
| [src/m5_grasp/README.md](src/m5_grasp/README.md) | m5_grasp包说明 |
| [simulator/README.md](simulator/README.md) | 模拟器说明 |

## 开发

### 单独编译某个包

```bash
colcon build --packages-select m5_grasp
```

### 运行测试

```bash
colcon test --packages-select m5_grasp
```

## 许可证

BSD License
