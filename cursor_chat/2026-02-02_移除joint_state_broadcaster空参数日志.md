# 移除 joint_state_broadcaster 空参数日志

## 修改

- **文件**: `src/ros2_controllers/joint_state_broadcaster/src/joint_state_broadcaster.cpp`
- **内容**: 在 `on_configure()` 中当 `use_all_available_interfaces()` 为真时，删除了原来的 `RCLCPP_DEBUG(..., "'joints' or 'interfaces' parameter is empty. All available state interfaces will be published.")` 日志，改为仅保留注释说明：未配置 joints/interfaces 时发布所有可用 state interfaces，关节顺序由 `init_joint_data` 内排序保证。
- **效果**: 该条提示在任何日志级别下都不会再出现；行为不变（仍发布全部接口并按 Joint1..Joint4, JointGL, JointGR 排序）。
