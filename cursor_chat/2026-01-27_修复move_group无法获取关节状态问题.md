# 修复 move_group 无法获取关节状态问题

## 问题描述

系统在运行时出现大量错误：
```
[ERROR] [move_group_interface]: Failed to fetch current robot state
Didn't receive robot state (joint angles) with recent timestamp within 1.000000 seconds.
```

`move_group` 的 `current_state_monitor` 无法获取最新的关节状态，导致任务执行时反复失败。

## 根本原因

1. `move_group` 的 `current_state_monitor` 有时间戳检查，要求接收到的关节状态时间戳必须是"最近的"（默认1秒内）
2. 虽然 `joint_states` 话题以 100Hz 正常发布，但 `current_state_monitor` 的时间戳检查过于严格
3. 当时间戳检查失败时，`getCurrentJointValues()` 和 `getCurrentState()` 都会返回空或失败

## 修复方案

### 1. **关键修复：修改 joint_states QoS 配置** (`ros2_controllers.yaml`)

**这是最重要的修复！** 之前的提交 `408bd30` 已经修复过这个问题，但后来又被改回去了。

将 `joint_states` 的 QoS 从 `reliable` 改为 `best_effort`：

```yaml
joint_states:
  publisher:
    reliability: best_effort  # 使用 best_effort 避免时间戳检查过严导致的问题
    durability: volatile
    history: keep_last
    depth: 10
```

**原因**：`reliable` QoS 要求消息必须按顺序、无丢失地传递，但在某些情况下（如网络延迟、消息队列满等）会导致时间戳检查失败。`best_effort` 允许更灵活的消息传递，更适合高频更新的状态消息。

### 2. 增加 move_group 配置参数 (`m5_grasp.launch.py`)

添加了以下参数来放宽时间戳检查容忍度：

```python
"planning_scene_monitor.wait_for_initial_state_timeout": 10.0,  # 等待初始状态的超时时间（秒）
"robot_state_topic_timeout": 5.0,  # 从默认1.0秒增加到5.0秒，放宽时间戳检查容忍度
```

### 2. 改进获取关节状态的重试逻辑 (`task_factory.cpp`)

在 `makePregrasp`、`makeAlign` 等函数中增强了重试机制：

- **增加超时时间**：`getCurrentState()` 的超时从 2 秒逐步增加到 5 秒（2s → 3s → 5s）
- **多次重试**：`getCurrentState()` 失败后自动重试 3 次，每次重试间隔 500ms
- **更好的错误处理**：改进日志输出，记录每次重试的详细信息

### 主要改进点

1. **更长的超时时间**：从 2 秒增加到最多 5 秒
2. **多次重试**：`getCurrentState()` 失败后自动重试 3 次
3. **逐步增加超时**：每次重试增加超时时间
4. **更好的日志**：记录每次重试的详细信息

## 修改的文件

1. **`src/m5_moveit_config/config/ros2_controllers.yaml`** - **关键修复**：将 `joint_states` QoS 从 `reliable` 改为 `best_effort`
2. `src/m5_bringup/launch/m5_grasp.launch.py` - 添加了 `robot_state_topic_timeout` 参数
3. `src/m5_grasp/src/mtc/task_factory.cpp` - 改进了 `addCurrentStateStage`、`makePregrasp`、`makeAlign` 中的重试逻辑，使用 `std::unique_lock` 避免在锁内阻塞
4. `src/m5_grasp/src/mtc/task_context.cpp` - 改进了 `getCurrentStateSafe()` 的 fallback 逻辑，匹配 607e1b8 版本

## 关键修复点

### 1. QoS 配置（最重要）
参考 607e1b8 和 408bd30 版本，将 `joint_states` QoS 从 `reliable` 改为 `best_effort`：
- `reliable` QoS 要求消息必须按顺序、无丢失地传递，在某些情况下会导致时间戳检查失败
- `best_effort` 允许更灵活的消息传递，更适合高频更新的状态消息（100Hz）

### 2. 锁的使用改进
在 `task_factory.cpp` 中使用 `std::unique_lock` 替代 `std::lock_guard`，以便在调用可能阻塞的 `getCurrentState()` 时释放锁。

### 3. Fallback 逻辑优化
参考 607e1b8 版本，优化了 `getCurrentStateSafe()` 的 fallback 逻辑，使用 3.0 秒超时。

## 参考

- 提交 `408bd30` - "解决Failed to fetch current robot state报错问题"
  - 主要修改：将 `joint_states` QoS 从 `reliable` 改为 `best_effort`
- 提交 `607e1b8` - "重构，位置规划成功，夹爪成功控制"
  - 这是能正常工作的版本，所有流程都能完成
  - 关键实现：`getCurrentStateSafe()` 优先使用 `getCurrentJointValues()` 绕过时间戳检查

## 测试建议

重新编译并测试：

```bash
cd /home/huzy/grasp_perception
colcon build --packages-select m5_grasp m5_bringup
./start_robot.sh
```

观察日志中是否还有 "Failed to fetch current robot state" 错误。

## 注意事项

- `joint_states` 话题确实在正常发布（100Hz），问题在于 `current_state_monitor` 的时间戳检查
- 如果问题仍然存在，可能需要进一步检查 `move_group` 和 `m5_grasp` 节点的 `MoveGroupInterface` 实例是否正确订阅了 `joint_states` 话题
