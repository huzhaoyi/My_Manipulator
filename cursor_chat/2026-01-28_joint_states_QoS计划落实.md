# joint_states QoS 计划落实

## 日期
2026-01-28

## 计划来源
`.cursor/plans/joint_states_qos_说明与可选对齐_1d350fc6.plan.md`（joint_states QoS 现状与可选对齐）

## 已落实内容

### 1. m5_grasp 的 /joint_states 订阅（与 MoveIt 一致）

- **文件**：`src/m5_grasp/src/m5_grasp.cpp`
- **修改**：将诊断订阅的 QoS 从 `reliable().transient_local()` 改为 `best_effort().durability_volatile()`（sensor_data 风格），与 MoveIt、robot_state_publisher 一致。
- **注释**：说明与 MoveIt 一致用 best_effort+volatile，避免 QoS 不匹配导致 MoveIt 收不到 joint_states、报 "latest received state time 0"。
- **LOG**：改为 "✓ /joint_states 诊断订阅已创建 (best_effort/volatile，与 MoveIt 一致)"。

### 2. ros2_controllers.yaml 注释（发布端说明）

- **文件**：`src/m5_moveit_config/config/ros2_controllers.yaml`
- **修改**：在 `qos_overrides` 前增加注释，说明上游 joint_state_broadcaster 使用 `SystemDefaultsQoS()` 创建 publisher，未读取此处 override；需从源码构建 ros2_controllers 并将 broadcaster 改为 `SensorDataQoS()` 后，发布端才会变为 VOLATILE+BEST_EFFORT。
- **保留**：`qos_overrides.joint_states.publisher` 仍为 reliability: best_effort、durability: volatile，供未来支持 override 的版本或补丁后生效。

## 当前状态小结

| 角色 | 节点 | Reliability | Durability |
|------|------|-------------|------------|
| Publisher | joint_state_broadcaster | RELIABLE | TRANSIENT_LOCAL（上游默认，yaml override 未生效） |
| Subscriber | move_group_private_* | BEST_EFFORT | VOLATILE |
| Subscriber | robot_state_publisher | BEST_EFFORT | VOLATILE |
| Subscriber | m5_grasp | BEST_EFFORT | VOLATILE（已与 MoveIt 一致） |

- m5_grasp 订阅已与 MoveIt、robot_state_publisher 一致；若发布端后续改为 VOLATILE+BEST_EFFORT（如通过 joint_state_broadcaster 源码补丁），三者将统一。
- 实际 joint_states 发布仍为 RELIABLE+TRANSIENT_LOCAL，以 `ros2 topic info /joint_states -v` 为准。

## 测试验证

1. **编译**：`colcon build --packages-select m5_grasp --symlink-install`（已通过）。
2. **启动机器人**：`./start_robot.sh`。
3. **在另一终端验证 QoS**：
   ```bash
   source install/setup.bash
   bash scripts/verify_joint_states_qos.sh
   ```
   脚本会检查：`/joint_states` 存在、m5_grasp 订阅为 BEST_EFFORT + VOLATILE、发布频率约 100 Hz。
4. **手动检查**（可选）：
   ```bash
   ros2 topic info /joint_states -v   # 看 m5_grasp 的 QoS 是否为 BEST_EFFORT + VOLATILE
   ros2 topic hz /joint_states       # 看约 100 Hz
   ros2 topic echo /joint_states --once  # 看 header.stamp 是否非 0
   ```
5. **运行抓取流程**：观察日志中 "latest received state time 0" / "Failed to fetch current robot state" 是否减少（发布端仍为 RELIABLE+TRANSIENT_LOCAL 时，MoveIt 可能仍偶发；发布端改为 SensorDataQoS 后效果最佳）。

## 可选后续（未在本仓库执行）

- 从源码构建 ros2_controllers，在 `joint_state_broadcaster.cpp` 的 `on_configure()` 中，将创建 joint_states publisher 的 `SystemDefaultsQoS()` 改为 `rclcpp::SensorDataQoS()`，使发布端变为 VOLATILE+BEST_EFFORT，根因可彻底消除。
