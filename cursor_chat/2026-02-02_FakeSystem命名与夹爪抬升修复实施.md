# FakeSystem 命名、夹爪时序与 LIFT 抬升修复实施

## 实施内容

### 1) LIFT 无抬升（轨迹 1 点、duration=0）修复

**文件**: `src/m5_grasp/src/mtc/task_factory.cpp`

- 在得到 `lift_joint_map` 后、决定使用关节抬升或笛卡尔抬升之前，增加**抬升目标与起始差异检查**：
  - 若提供了 `start_joints`，逐关节计算 `max_i |lift_i - start_i|`；
  - 若 `max_diff < 0.0087 rad`（约 0.5°），认为无有效抬升，将 `use_joint_lift = false`，走现有笛卡尔抬升分支 `makeMoveRelativeZ`；
  - 打日志：`[makeLift] 抬升关节目标与起始差异过小 (max_diff=...)，改用笛卡尔抬升`。

### 2) 夹爪“还没动就下一阶段”修复

**文件**: `src/m5_grasp/src/execution/gripper_controller.cpp`

- **execute 后最小等待**（open 与 close 共用）：
  - `execute()` 返回 SUCCESS 后，由原来的 4×50ms=200ms 改为**固定 400ms** 再开始 `isFeedbackStable()` 采样，让实机起动、反馈更新。
- **打开路径增加“到位”判据**：
  - 在 `isFeedbackStable()` 通过后、设置 `last_open_verified_stable_ = true` 之前，增加“反馈接近打开目标”的检查：
  - 取当前反馈 `(JointGL, JointGR)`，要求 `|pos_gl - angle_max| < 0.02 rad` 且 `|pos_gr - (-angle_max)| < 0.02 rad`；
  - 不满足则按失败返回，不打“打开验证通过”，避免“未动但稳定”被当成打开完成。

### 3) FakeSystem 命名为 m5_robot

**文件**:
- `src/m5_moveit_config/config/m5.urdf.xacro`：调用处由 `name="FakeSystem"` 改为 `name="m5_robot"`。
- `src/m5_moveit_config/config/m5.ros2_control.xacro`：在 macro 上方增加注释，说明 `name` 为 ros2_control 硬件名，日志中 "Loading hardware 'xxx'" 即此名，传 `m5_robot` 表示 M5 实机 UDP 接口。

## 编译

- `colcon build --packages-select m5_grasp m5_moveit_config` 已通过。

## 建议验证

1. 运行抓取流程，确认 LIFT 阶段有实际抬升轨迹（笛卡尔或关节），无 `points=1 duration=0`。
2. 观察夹爪打开：应在 execute 后约 400ms 再判稳，且只有反馈接近张开目标时才进入下一阶段。
3. 启动时日志中应出现 `Loading hardware 'm5_robot'` 而非 `FakeSystem`。
