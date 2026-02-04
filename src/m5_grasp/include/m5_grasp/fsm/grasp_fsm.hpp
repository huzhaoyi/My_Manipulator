#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <m5_msgs/msg/cable_pose_with_yaw.hpp>
#include <rclcpp/rclcpp.hpp>

#include "m5_grasp/hw/gripper_interface.hpp"

namespace m5_grasp
{

struct TaskTarget;
class ITaskRunner;

enum class GraspState
{
    IDLE,
    OPEN_GRIPPER,
    WAIT_GRIPPER,          // 非阻塞等待夹爪完成（超时或 result 后进 PREGRASP/LIFT）
    PREGRASP,              // 移动到预抓取位置
    WAIT_EXECUTION_RESULT, // 等待执行结果（异步执行）
    WAIT_ARM_STABLE,       // 非阻塞等待机械臂关节稳定后再进下一步
    ALIGN,                 // 对齐姿态
    DESCEND,               // 下探
    CLOSE_GRIPPER,         // 闭合夹爪（发动作后立刻切 WAIT_GRIPPER）
    LIFT,                  // 抬升
    DONE,                  // 完成
    ABORT_SAFE,            // 安全中止
    RETRY_BACKOFF          // 重试回退
};

/**
 * @brief 状态枚举转字符串
 */
const char* grasp_state_to_string(GraspState state);

/**
 * @brief 状态变化回调类型
 */
using StateChangeCallback = std::function<void(GraspState old_state, GraspState new_state)>;

/**
 * @brief 抓取状态机 - 纯逻辑，不依赖MTC/MoveIt细节
 *
 * 职责：
 * - 管理状态转移
 * - 重试和回退逻辑
 * - 调用抽象接口（ITaskRunner, IGripper）
 *
 * 不做：
 * - 不直接操作MTC/MoveIt
 * - 不处理ROS订阅/发布
 * - 不管理具体硬件
 */
class GraspFSM
{
  public:
    /**
     * @brief 配置参数
     */
    struct Config
    {
        int plan_retry_max{3};     // 规划重试次数
        int exec_retry_max{2};     // 执行重试次数
        int gripper_wait_ms{8000}; // 夹爪等待时间 (ms)，应 ≥ 夹爪全行程 7.5s
        int gripper_close_min_dwell_ms{2000}; // 闭合后最小停留 (ms)，2~4s 更保守，避免未夹紧就抬
        int gripper_open_min_dwell_ms{
            1500}; // 打开后最小停留 (ms)，再允许转 PREGRASP，避免臂动时夹爪仍在张开
        int arm_stable_timeout_ms{15000}; // 机械臂稳定等待最大超时 (ms)，超时=失败
        double arm_stable_threshold_rad{0.004}; // 关节角度变化阈值 (rad)，小于此值视为稳定
        int arm_stable_window{8}; // 连续满足 pos+vel 的采样次数（8~10 更易达稳）
        double arm_stable_velocity_eps_rad_s{0.012}; // 速度判稳阈值 (rad/s)，<=0 不检查
        int arm_stable_min_dwell_ms{800};            // 稳定后最小停留 (ms)，<=0 不要求
    };

    GraspFSM(std::shared_ptr<ITaskRunner> task_runner, std::shared_ptr<IGripper> gripper);

    /**
     * @brief 设置配置
     */
    void set_config(const Config& config);

    /**
     * @brief 设置状态变化回调
     */
    void set_state_change_callback(StateChangeCallback callback);

    /**
     * @brief FSM tick（定时调用）
     * @param now 当前时间
     */
    void tick(const rclcpp::Time& now);

    /**
     * @brief 触发急停
     */
    void emergency_stop();

    /**
     * @brief 重置状态机
     */
    void reset();

    /**
     * @brief 获取当前状态
     */
    GraspState get_current_state() const;

    /**
     * @brief 获取当前状态字符串
     */
    std::string get_current_state_string() const;

    /**
     * @brief 检查是否正在执行
     */
    bool is_executing() const;

    /**
     * @brief 设置当前任务目标（由外部Node计算后注入）
     */
    void set_current_target(const TaskTarget& target);

    /**
     * @brief 获取当前任务目标（用于外部访问）
     */
    const TaskTarget* get_current_target() const
    {
        return current_target_.get();
    }

    void trigger_start_grasp();

  private:
    /**
     * @brief 状态转移
     */
    void transitionTo(GraspState new_state);

    /**
     * @brief 处理失败，进入重试或回退
     * @param fallback_state 回退状态
     */
    void handleFailure(GraspState fallback_state);

    /**
     * @brief 执行安全中止（非阻塞：发 open 后进 WAIT_GRIPPER，超时后回 IDLE）
     * @param now 当前时间（用于 WAIT_GRIPPER 超时）
     */
    void doAbortSafe(const rclcpp::Time& now);

    void handleOpenGripper(const rclcpp::Time& now);
    void handleWaitGripper(const rclcpp::Time& now);
    void handlePregrasp(const rclcpp::Time& now);
    void handleWaitExecutionResult(const rclcpp::Time& now);
    void handleWaitArmStable(const rclcpp::Time& now);
    void handleAlign(const rclcpp::Time& now);
    void handleDescend(const rclcpp::Time& now);
    void handleCloseGripper(const rclcpp::Time& now);
    void handleLift();
    void handleDone();
    void handleAbortSafe(const rclcpp::Time& now);

    std::shared_ptr<ITaskRunner> task_runner_;
    std::shared_ptr<IGripper> gripper_;

    // 配置
    Config config_;

    // 状态
    std::atomic<GraspState> state_{GraspState::IDLE};
    std::atomic<bool> emergency_stop_{false};

    // 重试计数
    std::atomic<int> plan_retry_count_{0};
    std::atomic<int> exec_retry_count_{0};

    // 当前任务目标（使用unique_ptr避免包含完整定义）
    std::unique_ptr<TaskTarget> current_target_;

    // WAIT_GRIPPER 非阻塞：记录起始时间与等待类型（true=等打开→PREGRASP，false=等闭合→LIFT）
    rclcpp::Time gripper_wait_start_{0, 0};
    bool wait_gripper_open_{true};
    bool wait_gripper_abort_{false}; // true=安全中止：等夹爪打开超时后回 IDLE

    // WAIT_ARM_STABLE 非阻塞：稳定后要进入的下一个状态
    GraspState next_state_after_stable_{GraspState::IDLE};

    // 回调
    StateChangeCallback state_change_callback_;
};

} // namespace m5_grasp
