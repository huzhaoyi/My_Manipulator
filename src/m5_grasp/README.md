# M5 Grasp

M5 机械臂抓取节点 - 基于 MoveIt Task Constructor 的模块化实现。

## 架构

```
m5_grasp_node (ROS2 Node, 454行)
    ├── TargetTracker      # 目标跟踪与稳定判定
    ├── GraspFSM           # 抓取状态机
    │   ├── ITaskRunner    # MTC任务执行接口
    │   └── IGripper       # 夹爪控制接口
    ├── TaskRunner         # MTC任务执行实现
    │   └── TaskFactory    # MTC任务构建
    └── GripperController  # 夹爪控制实现
```

## 模块说明

| 模块 | 路径 | 职责 |
|------|------|------|
| GraspNode | `src/m5_grasp.cpp` | ROS2资源管理、模块协调 |
| TargetTracker | `fsm/target_tracker.*` | 目标滤波、稳定判定、跳变检测 |
| GraspFSM | `fsm/grasp_fsm.*` | 状态机逻辑、重试回退 |
| TaskFactory | `mtc/task_factory.*` | 构建MTC任务 |
| TaskRunner | `mtc/task_runner.*` | 执行MTC任务 |
| TaskContext | `mtc/task_context.*` | MTC共享配置 |
| IGripper | `hw/gripper_interface.hpp` | 夹爪抽象接口 |
| GripperController | `execution/gripper_controller.*` | 夹爪控制（直接 UDP，无轨迹） |
| ParameterManager | `utils/parameter_manager.*` | 参数管理 |
| WebVisualizer | `visualization/web_visualizer.*` | 网页可视化 |

## FSM状态

```
IDLE → WAIT_TARGET_STABLE → OPEN_GRIPPER → PREGRASP → ALIGN → DESCEND → CLOSE_GRIPPER → LIFT → DONE
                ↑                                                                              │
                └──────────────────────────── 失败重试 ←───────────────────────────────────────┘
```

| 状态 | 说明 |
|------|------|
| IDLE | 空闲，等待目标 |
| WAIT_TARGET_STABLE | 等待目标稳定（5帧内位置<1cm，yaw<5°） |
| OPEN_GRIPPER | 打开夹爪 |
| PREGRASP | 移动到预抓取位置 |
| ALIGN | 对齐姿态（Joint4对齐缆绳yaw） |
| DESCEND | 笛卡尔下探（工具系-Z方向） |
| CLOSE_GRIPPER | 闭合夹爪 |
| LIFT | 笛卡尔抬升 |
| DONE | 完成 |
| ABORT_SAFE | 安全中止 |

## 话题

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/cable_pose_with_yaw` | `m5_msgs/CablePoseWithYaw` | 订阅 | 缆绳位置+yaw |
| `/emergency_stop` | `std_msgs/String` | 订阅 | 急停信号 |
| `/grasp_state` | `std_msgs/String` | 发布 | 抓取状态 |
| `/grasp_markers` | `visualization_msgs/MarkerArray` | 发布 | 可视化标记 |
| `/cable_pose_visualization` | `geometry_msgs/PoseStamped` | 发布 | 缆绳位姿可视化 |

## 夹爪控制（直接 UDP）

夹爪**不走轨迹**，通过文件与 m5_hardware 配合：

- **发命令**：将目标 (JointGL, JointGR) 弧度写入 `/tmp/gripper_direct.txt`，硬件合并后发 UDP。axis5：**0=闭合**，**-1100=全开**。
- **读反馈**：优先从 `/tmp/udp_feedback.txt` 读取夹爪状态（硬件每收到 UDP 即写），不依赖 `/joint_states`，避免 MoveGroup “latest state time 0” 报错。
- **判稳**：先等「数值在变化」（运动开始），再等「数值不变」即判到位/夹到。

详见 `config/cable_grasp.yaml` 中 `gripper.*` 与 FSM 相关参数。

**真机**：需保证 `m5_moveit_config/config/m5.ros2_control.xacro` 中 `robot_ip`/`robot_port` 与机械臂一致，修改后重新编译。

## 参数

详见 `config/cable_grasp.yaml`

### 夹爪参数（gripper.*）
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `gripper.open_width` | 0.030 | 张开宽度 (m) |
| `gripper.close_width` | 0.0 | 闭合宽度 (m) |
| `gripper.angle_min` / `angle_max` | 1.01 / -1.01 | 闭合/张开对应弧度（axis5 0/-1100） |
| `gripper.stable_samples` | 10 | 判稳连续采样次数 |
| `gripper.stable_threshold_rad` | 0.008 | 相邻采样变化小于此视为不变 (rad) |
| `gripper.motion_start_threshold_rad` | 0.008 | 判定「开始运动」的最小变化 (rad) |
| `gripper.motion_start_timeout_ms` | 3000 | 等待运动开始最大 ms，超时仍进入判稳 |
| `gripper.stable_timeout_ms` | 8000 | 等待数值不变最大 ms，闭合超时=失败 |

### FSM 参数
| 参数 | 默认值 | 说明 |
|------|--------|------|
| `fsm.stable_position_threshold` | 0.01 | 位置稳定阈值 (m) |
| `fsm.stable_yaw_threshold` | 0.087 | yaw稳定阈值 (rad, 约5°) |
| `fsm.target_stale_timeout` | 0.5 | 目标过期超时 (s) |
| `fsm.jump_position_threshold` | 0.03 | 位置跳变阈值 (m) |
| `fsm.jump_yaw_threshold` | 0.175 | yaw跳变阈值 (rad, 约10°) |
| `fsm.stable_window_size` | 5 | 稳定性滑窗大小 |
| `fsm.gripper_wait_ms` | 8000 | 夹爪阶段最大等待 (ms) |

## 使用

```bash
# 启动节点
ros2 run m5_grasp m5_grasp_node

# 或使用launch文件
ros2 launch m5_bringup m5_grasp.launch.py
```

## 编译

```bash
cd /home/huzy/grasp_perception
source /opt/ros/humble/setup.bash
colcon build --packages-select m5_grasp
```

## 依赖

### ROS2 包依赖

| 包名 | 说明 |
|------|------|
| `rclcpp` | ROS2 C++ 客户端库 |
| `rclcpp_action` | ROS2 Action 支持 |
| `moveit_ros_planning_interface` | MoveIt 规划接口 |
| `moveit_core` | MoveIt 核心（时间参数化等） |
| `moveit_task_constructor_core` | MTC 核心库 |
| `moveit_task_constructor_msgs` | MTC 消息定义 |
| `geometry_msgs` | 几何消息 |
| `std_msgs` | 标准消息 |
| `control_msgs` | 控制消息 |
| `trajectory_msgs` | 轨迹消息 |
| `visualization_msgs` | 可视化消息 |
| `nav_msgs` | 导航消息 |
| `shape_msgs` | 形状消息 |
| `tf2_ros` | TF2 ROS 接口 |
| `tf2_geometry_msgs` | TF2 几何消息转换 |
| `m5_msgs` | 自定义消息（CablePoseWithYaw） |

### 系统库依赖

| 库名 | 说明 |
|------|------|
| `spdlog` | 高性能异步日志库 |

### 运行时依赖

| 包名 | 说明 |
|------|------|
| `moveit_ros_move_group` | MoveGroup 节点 |
| `moveit_kinematics` | 运动学插件 |
| `moveit_planners` | 运动规划器 |
| `m5` | 机器人描述 |
| `m5_hardware` | 硬件接口 |

## 库结构

```
m5_grasp_node
    │
    ├── m5_grasp_logging_lib        # 日志库（spdlog封装）
    │       └── spdlog
    │
    ├── m5_grasp_utils_lib          # 工具库
    │       ├── trajectory_planner  # 轨迹规划
    │       ├── scene_manager       # 场景管理
    │       └── parameter_manager   # 参数管理
    │
    ├── m5_grasp_fsm_lib            # FSM库
    │       ├── target_tracker      # 目标跟踪
    │       ├── grasp_fsm           # 抓取状态机
    │       └── m5_grasp_mtc_lib    # 依赖MTC库
    │
    ├── m5_grasp_mtc_lib            # MTC库
    │       ├── task_context        # 任务上下文
    │       ├── task_factory        # 任务构建
    │       └── task_runner         # 任务执行
    │
    ├── m5_grasp_planning_lib       # 规划库
    │       ├── kinematics_utils    # IK工具
    │       └── mtc_task_creator    # MTC任务创建
    │
    ├── m5_grasp_execution_lib      # 执行库
    │       ├── gripper_controller  # 夹爪控制（直接 UDP，读/写 tmp 文件）
    │       ├── motion_executor     # 运动执行
    │       └── cartesian_executor  # 笛卡尔执行
    │
    └── m5_grasp_visualization_lib  # 可视化库
            └── web_visualizer      # 网页3D可视化
```

## 安装依赖

```bash
# ROS2 依赖
sudo apt install -y \
    ros-humble-moveit \
    ros-humble-moveit-task-constructor-core \
    ros-humble-moveit-task-constructor-msgs

# 系统库
sudo apt install -y libspdlog-dev
```
