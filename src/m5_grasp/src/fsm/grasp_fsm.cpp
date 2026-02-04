#include "m5_grasp/fsm/grasp_fsm.hpp"
#include "m5_grasp/logging/logger.hpp"
#include "m5_grasp/mtc/task_runner.hpp"

// 前向声明，避免循环依赖
namespace m5_grasp
{
class TaskRunner;
}

#include <chrono>
#include <thread>

namespace m5_grasp
{

const char* grasp_state_to_string(GraspState state)
{
    switch (state)
    {
    case GraspState::IDLE:
        return "IDLE";
    case GraspState::OPEN_GRIPPER:
        return "OPEN_GRIPPER";
    case GraspState::WAIT_GRIPPER:
        return "WAIT_GRIPPER";
    case GraspState::PREGRASP:
        return "PREGRASP";
    case GraspState::WAIT_EXECUTION_RESULT:
        return "WAIT_EXECUTION_RESULT";
    case GraspState::WAIT_ARM_STABLE:
        return "WAIT_ARM_STABLE";
    case GraspState::ALIGN:
        return "ALIGN";
    case GraspState::DESCEND:
        return "DESCEND";
    case GraspState::CLOSE_GRIPPER:
        return "CLOSE_GRIPPER";
    case GraspState::LIFT:
        return "LIFT";
    case GraspState::DONE:
        return "DONE";
    case GraspState::ABORT_SAFE:
        return "ABORT_SAFE";
    case GraspState::RETRY_BACKOFF:
        return "RETRY_BACKOFF";
    default:
        return "UNKNOWN";
    }
}

GraspFSM::GraspFSM(std::shared_ptr<ITaskRunner> task_runner, std::shared_ptr<IGripper> gripper)
    : task_runner_(std::move(task_runner)), gripper_(std::move(gripper)),
      current_target_(std::make_unique<TaskTarget>())
{
}

void GraspFSM::set_current_target(const TaskTarget& target)
{
    *current_target_ = target;
}

void GraspFSM::trigger_start_grasp()
{
    if (state_.load() == GraspState::IDLE)
    {
        transitionTo(GraspState::OPEN_GRIPPER);
    }
}

void GraspFSM::set_config(const Config& config)
{
    config_ = config;
}

void GraspFSM::set_state_change_callback(StateChangeCallback callback)
{
    state_change_callback_ = std::move(callback);
}

void GraspFSM::transitionTo(GraspState new_state)
{
    GraspState old_state = state_.exchange(new_state);
    if (old_state != new_state)
    {
        LOG_NAMED_INFO("grasp_fsm", "[FSM] 状态转换: {} -> {}", grasp_state_to_string(old_state),
                       grasp_state_to_string(new_state));
        if (state_change_callback_)
        {
            state_change_callback_(old_state, new_state);
        }
    }
}

void GraspFSM::handleFailure(GraspState fallback_state)
{
    int retry = plan_retry_count_.fetch_add(1);
    if (retry < config_.plan_retry_max)
    {
        LOG_NAMED_WARN("grasp_fsm", "[FSM] 失败，重试 {}/{}", retry + 1, config_.plan_retry_max);
        transitionTo(fallback_state);
    }
    else
    {
        LOG_NAMED_ERROR("grasp_fsm", "[FSM] 重试次数耗尽，进入ABORT_SAFE");
        transitionTo(GraspState::ABORT_SAFE);
    }
}

void GraspFSM::doAbortSafe(const rclcpp::Time& now)
{
    LOG_NAMED_INFO("grasp_fsm", "[FSM] 执行安全中止（非阻塞）");
    if (gripper_)
    {
        gripper_->open();
        gripper_wait_start_ = now;
        wait_gripper_abort_ = true;
        wait_gripper_open_ = false; // 中止路径只等超时回 IDLE，不参与“打开→PREGRASP”
        transitionTo(GraspState::WAIT_GRIPPER);
        LOG_NAMED_INFO("grasp_fsm", "[FSM] 已发打开夹爪，进入WAIT_GRIPPER（安全中止）");
    }
    else
    {
        if (current_target_)
        {
            current_target_->valid = false;
        }
        emergency_stop_.store(false);
        if (task_runner_)
        {
            task_runner_->set_emergency_stop(false);
        }
        transitionTo(GraspState::IDLE);
        LOG_NAMED_INFO("grasp_fsm", "[FSM] 安全中止完成（无夹爪）");
    }
}

void GraspFSM::tick(const rclcpp::Time& now)
{
    // 检查急停：非 IDLE/ABORT_SAFE 时先转 ABORT_SAFE；已在 ABORT_SAFE 时继续执行 handleAbortSafe
    // 以便清除急停并回到 IDLE
    if (emergency_stop_.load())
    {
        GraspState current = state_.load();
        if (current != GraspState::IDLE && current != GraspState::ABORT_SAFE)
        {
            LOG_NAMED_WARN("grasp_fsm", "[FSM] 急停触发，进入ABORT_SAFE");
            transitionTo(GraspState::ABORT_SAFE);
            return;
        }
        if (current == GraspState::IDLE)
        {
            return;
        }
        // current == ABORT_SAFE：不 return，继续到 switch 中执行
        // handleAbortSafe，完成后清除急停并回到 IDLE
    }

    GraspState current = state_.load();

    switch (current)
    {
    case GraspState::IDLE:
        break;

    case GraspState::OPEN_GRIPPER:
        handleOpenGripper(now);
        break;

    case GraspState::WAIT_GRIPPER:
        handleWaitGripper(now);
        break;

    case GraspState::PREGRASP:
        handlePregrasp(now);
        break;

    case GraspState::WAIT_EXECUTION_RESULT:
        handleWaitExecutionResult(now);
        break;

    case GraspState::WAIT_ARM_STABLE:
        handleWaitArmStable(now);
        break;

    case GraspState::ALIGN:
        handleAlign(now);
        break;

    case GraspState::DESCEND:
        handleDescend(now);
        break;

    case GraspState::CLOSE_GRIPPER:
        handleCloseGripper(now);
        break;

    case GraspState::LIFT:
        handleLift();
        break;

    case GraspState::DONE:
        handleDone();
        break;

    case GraspState::ABORT_SAFE:
        handleAbortSafe(now);
        break;

    default:
        break;
    }
}

void GraspFSM::handleOpenGripper(const rclcpp::Time& now)
{
    if (!gripper_)
    {
        LOG_NAMED_ERROR("grasp_fsm", "[FSM] 夹爪接口未设置");
        transitionTo(GraspState::ABORT_SAFE);
        return;
    }

    LOG_NAMED_INFO("grasp_fsm", "[FSM] 开始打开夹爪");
    if (gripper_->open())
    {
        gripper_wait_start_ = now;
        wait_gripper_open_ = true;
        wait_gripper_abort_ = false; // 正常打开入口，清除中止标志
        transitionTo(GraspState::WAIT_GRIPPER);
        LOG_NAMED_INFO("grasp_fsm", "[FSM] 已发打开命令，进入WAIT_GRIPPER（非阻塞）");
    }
    else
    {
        LOG_NAMED_ERROR("grasp_fsm", "[FSM] 打开夹爪失败");
        transitionTo(GraspState::ABORT_SAFE);
    }
}

void GraspFSM::handleWaitGripper(const rclcpp::Time& now)
{
    if (gripper_)
    {
        // 仅“正常打开→预抓”时夹爪打开后进
        // PREGRASP；需满足到位且打开最小停留，避免臂动时夹爪仍在张开
        if (wait_gripper_open_ && !wait_gripper_abort_ && gripper_->is_open_to_target())
        {
            double elapsed_s = 0.0;
            try
            {
                elapsed_s = (now - gripper_wait_start_).seconds();
            }
            catch (...)
            {
                elapsed_s = 0.0;
            }
            if (elapsed_s < 0.0)
                elapsed_s = 0.0;
            double open_min_dwell_s = config_.gripper_open_min_dwell_ms > 0
                                          ? (config_.gripper_open_min_dwell_ms / 1000.0)
                                          : 0.0;
            if (elapsed_s >= open_min_dwell_s)
            {
                transitionTo(GraspState::PREGRASP);
                LOG_NAMED_INFO("grasp_fsm", "[FSM] 夹爪已到位（打开），进入PREGRASP");
                return;
            }
        }
        if (!wait_gripper_open_ && !wait_gripper_abort_ && gripper_->is_closed_to_target())
        {
            // 闭合后最小停留，再允许转 LIFT，避免夹爪还在动就抬升
            double elapsed_s = 0.0;
            try
            {
                elapsed_s = (now - gripper_wait_start_).seconds();
            }
            catch (...)
            {
                elapsed_s = 0.0;
            }
            if (elapsed_s < 0.0)
                elapsed_s = 0.0;
            double min_dwell_s = config_.gripper_close_min_dwell_ms > 0
                                     ? (config_.gripper_close_min_dwell_ms / 1000.0)
                                     : 0.0;
            if (elapsed_s >= min_dwell_s)
            {
                transitionTo(GraspState::LIFT);
                LOG_NAMED_INFO("grasp_fsm", "[FSM] 夹爪已到位（闭合），进入LIFT");
                return;
            }
        }
    }
    double elapsed_s = 0.0;
    try
    {
        elapsed_s = (now - gripper_wait_start_).seconds();
    }
    catch (...)
    {
        elapsed_s = 0.0;
    }
    if (elapsed_s < 0.0)
    {
        elapsed_s = 0.0;
    }
    double timeout_s = config_.gripper_wait_ms / 1000.0;
    if (elapsed_s >= timeout_s)
    {
        if (wait_gripper_abort_)
        {
            wait_gripper_abort_ = false;
            if (current_target_)
            {
                current_target_->valid = false;
            }
            emergency_stop_.store(false);
            if (task_runner_)
            {
                task_runner_->set_emergency_stop(false);
            }
            transitionTo(GraspState::IDLE);
            LOG_NAMED_INFO("grasp_fsm", "[FSM] 安全中止完成（非阻塞，超时 {:.1f}s）", elapsed_s);
        }
        else if (wait_gripper_open_)
        {
            transitionTo(GraspState::PREGRASP);
            LOG_NAMED_INFO("grasp_fsm", "[FSM] 夹爪等待超时({:.1f}s)，进入PREGRASP", elapsed_s);
        }
        else
        {
            // 闭合等待超时：未在超时内得到“闭合验证通过”或未满足 min_dwell（到位/抓取成功均由
            // gripper 稳定判定）
            transitionTo(GraspState::ABORT_SAFE);
            LOG_NAMED_WARN(
                "grasp_fsm",
                "[FSM] 夹爪闭合等待超时({:.1f}s)，未在超时内满足闭合验证+min_dwell，进入ABORT_SAFE",
                elapsed_s);
        }
    }
    // 未超时且未到位则本 tick 直接返回，不阻塞，executor 可调度 /joint_states 等回调
}

void GraspFSM::handlePregrasp(const rclcpp::Time& /*now*/)
{
    // 执行阶段不再检查目标过期/跳变，避免因夹爪操作延时导致循环
    // 如果需要实时跟踪，应该在更高层实现"目标跟踪模式"

    if (!current_target_ || !current_target_->valid)
    {
        LOG_NAMED_ERROR("grasp_fsm", "[FSM] PREGRASP: 目标无效");
        transitionTo(GraspState::ABORT_SAFE);
        return;
    }

    LOG_NAMED_INFO("grasp_fsm", "[FSM] 开始执行预抓取");
    if (task_runner_->run_pregrasp(*current_target_))
    {
        // 已启动异步执行，进入等待状态
        transitionTo(GraspState::WAIT_EXECUTION_RESULT);
        LOG_NAMED_INFO("grasp_fsm", "[FSM] 预抓取已启动，进入WAIT_EXECUTION_RESULT");
    }
    else
    {
        handleFailure(GraspState::IDLE);
    }
}

void GraspFSM::handleWaitExecutionResult(const rclcpp::Time& /*now*/)
{
    // 检查执行状态（需要将 checkExecutionStatus 添加到 ITaskRunner 接口）
    // 暂时使用类型转换访问 TaskRunner
    auto task_runner = std::dynamic_pointer_cast<TaskRunner>(task_runner_);
    if (!task_runner)
    {
        LOG_NAMED_ERROR("grasp_fsm", "[FSM] WAIT_EXECUTION_RESULT: TaskRunner 类型错误");
        transitionTo(GraspState::ABORT_SAFE);
        return;
    }

    int status = task_runner->check_execution_status();

    // 根据任务名称决定下一步转移
    std::string task_name = task_runner->get_current_task_name();
    GraspState next_state = GraspState::IDLE;
    GraspState fallback_state = GraspState::IDLE; // 失败回退到 IDLE，接受新目标

    if (task_name == "pregrasp")
    {
        next_state = GraspState::ALIGN;
        fallback_state = GraspState::IDLE;
    }
    else if (task_name == "align")
    {
        next_state = GraspState::DESCEND;
        fallback_state = GraspState::PREGRASP;
    }
    else if (task_name == "descend")
    {
        next_state = GraspState::CLOSE_GRIPPER;
        fallback_state = GraspState::ALIGN;
    }
    else if (task_name == "lift")
    {
        next_state = GraspState::DONE;
        fallback_state = GraspState::ABORT_SAFE;
    }
    else
    {
        LOG_NAMED_WARN("grasp_fsm", "[FSM] WAIT_EXECUTION_RESULT: 未知任务名称 '{}'，使用默认逻辑",
                       task_name);
        next_state = GraspState::ALIGN;
        fallback_state = GraspState::IDLE;
    }

    if (status == 2)
    { // 成功
        // 清理执行状态（先清，再决定进 WAIT_ARM_STABLE 或直接 next_state）
        task_runner->clear_execution_state();
        // 机械臂每步完成后非阻塞等待关节稳定再进入下一步
        if (config_.arm_stable_timeout_ms > 0 && config_.arm_stable_window > 0)
        {
            next_state_after_stable_ = next_state;
            task_runner->start_arm_stable_check(
                config_.arm_stable_timeout_ms, config_.arm_stable_threshold_rad,
                config_.arm_stable_window, config_.arm_stable_velocity_eps_rad_s,
                config_.arm_stable_min_dwell_ms);
            transitionTo(GraspState::WAIT_ARM_STABLE);
            LOG_NAMED_INFO("grasp_fsm", "[FSM] 任务 '{}' 执行成功，进入WAIT_ARM_STABLE（非阻塞）",
                           task_name);
        }
        else
        {
            transitionTo(next_state);
            LOG_NAMED_INFO("grasp_fsm", "[FSM] 任务 '{}' 执行成功，进入{}", task_name,
                           grasp_state_to_string(next_state));
        }
    }
    else if (status == 3 || status == 4)
    { // 失败或超时
        LOG_NAMED_WARN("grasp_fsm", "[FSM] 任务 '{}' 执行{}（status={}）", task_name,
                       status == 3 ? "失败" : "超时", status);
        // 清理执行状态
        task_runner->clear_execution_state();
        handleFailure(fallback_state);
    }
    // status == 1 (执行中) 或 0 (未开始)：继续等待，不做任何操作
}

void GraspFSM::handleWaitArmStable(const rclcpp::Time& now)
{
    auto task_runner = std::dynamic_pointer_cast<TaskRunner>(task_runner_);
    if (!task_runner)
    {
        LOG_NAMED_ERROR("grasp_fsm", "[FSM] WAIT_ARM_STABLE: TaskRunner 类型错误");
        transitionTo(GraspState::ABORT_SAFE);
        return;
    }
    auto [done, stable] = task_runner->tick_arm_stable_check(now);
    if (!done)
    {
        return;
    }
    if (!stable)
    {
        // 超时 = 未达到稳定阈值，禁止继续下一步，避免“流程结束但实机还在动”
        LOG_NAMED_ERROR("grasp_fsm",
                        "[FSM] 关节稳定等待超时，动作未稳定，禁止继续（进入 ABORT_SAFE）");
        transitionTo(GraspState::ABORT_SAFE);
        return;
    }
    transitionTo(next_state_after_stable_);
    LOG_NAMED_INFO("grasp_fsm", "[FSM] 进入{}", grasp_state_to_string(next_state_after_stable_));
}

void GraspFSM::handleAlign(const rclcpp::Time& /*now*/)
{
    // 目标已锁定，不再检查过期/跳变

    if (!current_target_ || !current_target_->valid)
    {
        LOG_NAMED_ERROR("grasp_fsm", "[FSM] ALIGN: 目标无效");
        transitionTo(GraspState::ABORT_SAFE);
        return;
    }

    LOG_NAMED_INFO("grasp_fsm", "[FSM] 开始执行对齐");
    if (task_runner_->run_align(*current_target_))
    {
        // 已启动异步执行，进入等待状态
        transitionTo(GraspState::WAIT_EXECUTION_RESULT);
        LOG_NAMED_INFO("grasp_fsm", "[FSM] 对齐已启动，进入WAIT_EXECUTION_RESULT");
    }
    else
    {
        handleFailure(GraspState::PREGRASP);
    }
}

void GraspFSM::handleDescend(const rclcpp::Time& /*now*/)
{
    // 目标已锁定，不再检查过期/跳变
    if (!current_target_ || !current_target_->valid)
    {
        LOG_NAMED_ERROR("grasp_fsm", "[FSM] DESCEND: 目标无效");
        transitionTo(GraspState::ABORT_SAFE);
        return;
    }

    LOG_NAMED_INFO("grasp_fsm", "[FSM] 开始执行下探（移到 grasp_pose）");
    if (task_runner_->run_descend(*current_target_))
    {
        // 已启动异步执行，进入等待状态
        transitionTo(GraspState::WAIT_EXECUTION_RESULT);
        LOG_NAMED_INFO("grasp_fsm", "[FSM] 下探已启动，进入WAIT_EXECUTION_RESULT");
    }
    else
    {
        handleFailure(GraspState::ALIGN);
    }
}

void GraspFSM::handleCloseGripper(const rclcpp::Time& now)
{
    if (!gripper_)
    {
        LOG_NAMED_ERROR("grasp_fsm", "[FSM] 夹爪接口未设置");
        transitionTo(GraspState::ABORT_SAFE);
        return;
    }

    LOG_NAMED_INFO("grasp_fsm", "[FSM] 开始闭合夹爪");
    if (gripper_->close())
    {
        gripper_wait_start_ = now;
        wait_gripper_open_ = false;
        wait_gripper_abort_ = false; // 正常闭合入口，清除中止标志，避免超时误走“安全中止完成”
        transitionTo(GraspState::WAIT_GRIPPER);
        LOG_NAMED_INFO("grasp_fsm", "[FSM] 已发闭合命令，进入WAIT_GRIPPER（非阻塞）");
    }
    else
    {
        LOG_NAMED_ERROR("grasp_fsm", "[FSM] 闭合夹爪失败");
        transitionTo(GraspState::ABORT_SAFE);
    }
}

void GraspFSM::handleLift()
{
    if (!current_target_ || !current_target_->valid)
    {
        LOG_NAMED_ERROR("grasp_fsm", "[FSM] LIFT: 目标无效");
        transitionTo(GraspState::ABORT_SAFE);
        return;
    }
    LOG_NAMED_INFO("grasp_fsm", "[FSM] 开始执行抬升");
    if (task_runner_->run_lift(*current_target_))
    {
        // 已启动异步执行，进入等待状态
        transitionTo(GraspState::WAIT_EXECUTION_RESULT);
        LOG_NAMED_INFO("grasp_fsm", "[FSM] 抬升已启动，进入WAIT_EXECUTION_RESULT");
    }
    else
    {
        LOG_NAMED_ERROR("grasp_fsm", "[FSM] 抬升启动失败");
        transitionTo(GraspState::ABORT_SAFE);
    }
}

void GraspFSM::handleDone()
{
    LOG_NAMED_INFO("grasp_fsm", "[FSM] 抓取任务完成！");
    // 清除当前目标，准备接收新目标
    if (current_target_)
    {
        current_target_->valid = false;
    }
    transitionTo(GraspState::IDLE);
}

void GraspFSM::handleAbortSafe(const rclcpp::Time& now)
{
    doAbortSafe(now);
    // 若有夹爪则已转入 WAIT_GRIPPER，清理与回 IDLE 在 handleWaitGripper 超时后做；无夹爪时
    // doAbortSafe 内部已清理并转 IDLE
}

void GraspFSM::emergency_stop()
{
    emergency_stop_.store(true);
    if (task_runner_)
    {
        task_runner_->set_emergency_stop(true);
    }
}

void GraspFSM::reset()
{
    emergency_stop_.store(false);
    plan_retry_count_.store(0);
    exec_retry_count_.store(0);
    state_.store(GraspState::IDLE);
    if (task_runner_)
    {
        task_runner_->set_emergency_stop(false);
    }
}

GraspState GraspFSM::get_current_state() const
{
    return state_.load();
}

std::string GraspFSM::get_current_state_string() const
{
    return grasp_state_to_string(state_.load());
}

bool GraspFSM::is_executing() const
{
    GraspState current = state_.load();
    return current != GraspState::IDLE && current != GraspState::DONE;
}

} // namespace m5_grasp
