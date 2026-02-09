# M5机械臂模拟器

这是一个用于测试M5机械臂通信的Web模拟器，包含UDP服务器和可视化界面。

## 功能特性

- ✅ UDP服务器模拟M5机械臂（端口7001）
- ✅ 接收JSON格式的控制命令
- ✅ 返回JSON格式的反馈数据
- ✅ 3D可视化机械臂模型
- ✅ 实时状态显示
- ✅ 手动控制滑块
- ✅ FSM状态机状态显示
- ✅ MTC任务执行状态显示

## 使用方法

### 1. 启动模拟器

```bash
cd /home/huzy/grasp_perception/simulator
python3 robot_simulator.py
```

### 2. 打开Web界面

在浏览器中访问：
```
http://localhost:8080
```

### 3. 测试ROS2硬件接口

在另一个终端中启动ROS2：

```bash
cd /home/huzy/grasp_perception
source install/setup.bash
ros2 launch m5_bringup rviz.launch.py
```

## 协议说明

### 轴与夹爪约定

- **num 1～4**：机械臂关节（度）
- **num 5（axis5）**：夹爪。**0 = 闭合**，**-1100 = 完全打开**（与实机一致）

### 发送命令格式（ROS2 → 模拟器）

```json
[
  {"num": 1, "value": 45.0},
  {"num": 2, "value": -30.0},
  {"num": 3, "value": -45.0},
  {"num": 4, "value": 0.0},
  {"num": 5, "value": 0.0}
]
```

（num 5：0=夹爪闭合，-1100=夹爪全开）

### 接收反馈格式（模拟器 → ROS2）

```json
[
  {"num": 1, "value": 45.0},
  {"num": 2, "value": -30.0},
  {"num": 3, "value": -45.0},
  {"num": 4, "value": 0.0},
  {"num": 5, "value": 0.0},
  {"status": 0}
]
```

## 配置

可以在 `robot_simulator.py` 中修改：

- `udp_port`: UDP服务器端口（默认7001）
- `web_port`: Web服务器端口（默认8080）
- `host`: 监听地址（默认0.0.0.0）

## 注意事项

- 确保端口7001和8080未被占用
- 模拟器会平滑移动机械臂（最大速度10度/秒）
- Web界面每100ms更新一次状态
- **真机 IP/端口**：实机机器人地址在 `src/m5_moveit_config/config/m5.ros2_control.xacro` 中配置（`robot_ip`/`robot_port`，默认示例 192.168.100.38:7003）；模拟器默认监听 7001，与真机端口可不同。
