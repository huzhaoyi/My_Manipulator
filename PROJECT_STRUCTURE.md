# M5项目结构说明

## 包结构

项目已按ROS2最佳实践进行模块化重构，各包职责清晰：

### 核心包

#### `m5` - 机器人描述包
**职责**: 机器人URDF描述和mesh文件
- `urdf/` - URDF文件
- `meshes/` - 3D模型文件

#### `m5_hardware` - 硬件接口包
**职责**: 硬件抽象层，与物理机器人通信
- `src/` - 硬件接口实现
- `include/` - 头文件

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

#### `m5_planning` - 规划节点包
**职责**: 运动规划节点实现
- `src/m5_planning.cpp` - 主规划节点
  - 订阅 `/target_pose` 话题
  - 执行运动规划
  - 发布 `/robot_state` 状态

#### `m5_control` - 控制节点包
**职责**: 控制相关节点
- `src/joint_state_publisher_node.cpp` - 关节状态发布节点

### 系统包

#### `m5_bringup` - 启动文件包
**职责**: 系统启动和集成
- `launch/` - 所有launch文件
  - `m5_planning.launch.py` - 规划节点启动
  - `rviz.launch.py` - RViz可视化启动
  - `move_group.launch.py` - MoveGroup启动
  - 其他系统启动文件

#### `m5_tools` - 工具脚本包
**职责**: 开发和调试工具
- `scripts/` - Python工具脚本
  - `publish_target_pose.py` - 目标位姿发布工具

## 依赖关系

```
m5_bringup
  ├── m5_planning
  ├── m5_control
  ├── m5_moveit_config
  ├── m5_hardware
  └── m5

m5_planning
  ├── m5_moveit_config
  ├── m5_hardware
  └── m5

m5_control
  └── m5

m5_moveit_config
  └── m5
```

## 使用方式

### 启动规划节点
```bash
./start_robot.sh planning
# 或
ros2 launch m5_bringup m5_planning.launch.py
```

### 启动RViz演示
```bash
./start_robot.sh demo
# 或
ros2 launch m5_bringup rviz.launch.py
```

### 使用工具
```bash
ros2 run m5_tools publish_target_pose.py <x> <y> <z>
```

## 优势

1. **模块化**: 每个包职责单一，易于维护
2. **可扩展**: 新功能可以独立添加新包
3. **可重用**: 配置和节点可以在不同项目间复用
4. **符合ROS2最佳实践**: 遵循ROS2包组织规范
