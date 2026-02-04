# 启动前 joint_states 就绪门槛（消除“先跑再回”）

## 问题
机械臂会先动到一个地方，然后突然回到原位，再按抓取流程执行。常见原因之一是 MoveIt 的 current_state 在启动/切换时为全 0 或旧值，规划从错误起点出发，等 joint_states 正常后下一条轨迹又拉回。

## 方案
在 FSM 开始执行任何 task 之前增加「就绪门槛」：只有连续收到足够次数的 `/joint_states`、时间戳未断流、且关节非全零时，才允许从 IDLE 进入 OPEN_GRIPPER 开始抓取。

## 实现摘要

1. **参数（ParameterManager）**
   - `fsm.joint_states_ready_enabled`：是否启用就绪门槛（默认 true）
   - `fsm.joint_states_ready_min_count`：至少收到的 joint_states 次数（默认 10）
   - `fsm.joint_states_ready_max_age_s`：最后一帧不得超过此时间(s)（默认 0.3）
   - `fsm.joint_states_ready_nonzero_eps`：sum(|position|) 需大于此值才认为非全零（默认 0.01）

2. **节点逻辑（m5_grasp.cpp）**
   - 启用时：`cablePoseCallback` 收到目标后只设置 `pending_target_` 和 `pending_start_grasp_`，不立刻 `triggerStartGrasp()`。
   - `isJointStatesReady()`：根据 `task_context_.joint_state_diag` 判断 js_count、last_js_wall 新鲜度、以及 last_position 非全零。
   - `fsmTick()` 中：若当前为 IDLE 且 `pending_start_grasp_` 且 `isJointStatesReady()`，则 `setCurrentTarget(pending_target_)`、`triggerStartGrasp()`，并清除 pending。

3. **配置**
   - `config/cable_grasp.yaml` 的 `fsm` 下增加上述四个参数，便于调参。

关闭就绪门槛：将 `fsm.joint_states_ready_enabled` 设为 `false` 即可恢复“接受目标即立即启动”的旧行为。
