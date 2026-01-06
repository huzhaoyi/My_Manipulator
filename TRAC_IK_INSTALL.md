# TRAC-IK安装说明

## 概述

根据原厂代码分析，M5机械臂需要使用TRAC-IK求解器而不是KDL求解器。TRAC-IK对4DOF机械臂更鲁棒，特别是在工作空间边缘位置。

## 安装步骤

### 1. 安装依赖

```bash
sudo apt-get update
sudo apt-get install -y libnlopt-cxx-dev
```

或者使用提供的安装脚本：

```bash
cd /home/huzy/grasp_perception
./install_trac_ik_deps.sh
```

### 2. 修复ROS2兼容性问题

TRAC-IK源码已经克隆到 `src/trac_ik/` 目录，并且已经修复了ROS2兼容性问题：

- ✅ 修复了 `urdf/model.hpp` → `urdf/model.h`
- ✅ 修复了 `moveit/kinematics_base/kinematics_base.hpp` → `moveit/kinematics_base/kinematics_base.h`
- ✅ 修复了 `moveit/robot_model/robot_model.hpp` → `moveit/robot_model/robot_model.h`
- ✅ 修复了 `moveit/robot_state/robot_state.hpp` → `moveit/robot_state/robot_state.h`

### 3. 构建TRAC-IK

```bash
cd /home/huzy/grasp_perception
colcon build --packages-select trac_ik_lib trac_ik_kinematics_plugin
```

### 4. 构建整个工作空间

```bash
cd /home/huzy/grasp_perception
colcon build
```

### 5. 配置环境

构建完成后，source工作空间：

```bash
source install/setup.bash
```

### 6. 验证安装

检查TRAC-IK插件是否可用：

```bash
ros2 pkg list | grep trac_ik
```

应该看到：
- `trac_ik_lib`
- `trac_ik_kinematics_plugin`

## 配置说明

配置文件已经更新为使用TRAC-IK：

- 文件：`src/m5_configure/config/kinematics.yaml`
- 配置：
  ```yaml
  arm_group:
    kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
    kinematics_solver_search_resolution: 0.005
    kinematics_solver_timeout: 0.005
    position_only_ik: true
  ```

## 测试

安装完成后，重新启动demo：

```bash
ros2 launch m5_bringup m5_planning.launch.py
```

然后使用目标位置 (0.184, 0.081, 0.398) 进行测试，应该能够成功求解IK。

## 参考

- 原厂代码位置：`/home/huzy/M5_kuaji/ws_m5_test1/src/m5_moveit_config/config/kinematics.yaml`
- TRAC-IK仓库：https://bitbucket.org/traclabs/trac_ik.git
