#pragma once

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>

#include "m5_grasp/mtc/task_context.hpp"
#include "m5_grasp/mtc/task_factory.hpp"
#include "m5_grasp/hw/gripper_interface.hpp"

namespace mtc = moveit::task_constructor;

namespace m5_grasp {

/**
 * @brief 执行等待状态（堆上共享状态，避免回调悬空引用）
 * 
 * 问题：回调按引用捕获栈变量，函数返回后变量析构，回调访问悬空引用导致崩溃
 * 解决：使用 shared_ptr<ExecWaitState>，确保回调执行时状态仍然有效
 */
struct ExecWaitState {
  std::mutex m;
  std::condition_variable cv_goal;
  std::condition_variable cv_result;
  
  std::atomic<bool> goal_accepted{false};
  std::atomic<bool> execution_completed{false};
  std::atomic<bool> execution_succeeded{false};
  
  rclcpp_action::ClientGoalHandle<moveit_task_constructor_msgs::action::ExecuteTaskSolution>::SharedPtr goal_handle;
};

/**
 * @brief 任务执行器抽象接口
 * 
 * FSM 只依赖此接口，不关心具体的MTC实现细节
 */
class ITaskRunner {
public:
  virtual ~ITaskRunner() = default;

  /**
   * @brief 执行预抓取任务
   * @param target 任务目标
   * @return true 如果执行成功
   */
  virtual bool runPregrasp(const TaskTarget& target) = 0;

  /**
   * @brief 执行对齐任务
   * @param target 任务目标
   * @return true 如果执行成功
   */
  virtual bool runAlign(const TaskTarget& target) = 0;

  /**
   * @brief 执行下探任务（移到 grasp_pose，用 OMPL，不用笛卡尔）
   * @param target 任务目标（含 grasp_pose）
   * @return true 如果执行成功
   */
  virtual bool runDescend(const TaskTarget& target) = 0;

  /**
   * @brief 执行抬升任务（抬升时固定 Joint4 为 target.joint4_target，与 align/descend 一致）
   * @param target 任务目标（提供 joint4_target）
   * @return true 如果执行成功
   */
  virtual bool runLift(const TaskTarget& target) = 0;

  /**
   * @brief 设置急停标志
   */
  virtual void setEmergencyStop(bool stop) = 0;

  /**
   * @brief 检查是否处于急停状态
   */
  virtual bool isEmergencyStopped() const = 0;
};

/**
 * @brief MTC任务执行器 - 负责规划和执行MTC任务
 */
class TaskRunner : public ITaskRunner, public std::enable_shared_from_this<TaskRunner> {
public:
  /**
   * @brief 构造函数
   * @param ctx 任务上下文
   */
  explicit TaskRunner(const TaskContext& ctx);

  // ITaskRunner 接口实现
  bool runPregrasp(const TaskTarget& target) override;
  bool runAlign(const TaskTarget& target) override;
  bool runDescend(const TaskTarget& target) override;
  bool runLift(const TaskTarget& target) override;
  void setEmergencyStop(bool stop) override;
  bool isEmergencyStopped() const override;

  /**
   * @brief 执行通用MTC任务
   * @param task MTC任务
   * @param task_name 任务名称（用于日志）
   * @return true 如果执行成功
   */
  bool executeTask(mtc::Task& task, const std::string& task_name);

  /**
   * @brief 初始化 action client（延迟初始化，确保 ctx_ 已准备好）
   * @return true 如果初始化成功
   */
  bool initializeActionClient();

  /**
   * @brief 检查当前执行状态（供 FSM 调用）
   * @return 0=未开始, 1=执行中, 2=成功, 3=失败, 4=超时
   */
  int checkExecutionStatus() const;

  /**
   * @brief 清理当前执行状态（供 FSM 调用）
   */
  void clearExecutionState();

  /**
   * @brief 获取当前执行的任务名称（供 FSM 调用）
   * @return 任务名称，如果没有在执行则返回空字符串
   */
  std::string getCurrentTaskName() const;

  /**
   * @brief 等待机械臂关节角度稳定（阻塞，会占用 executor）
   * @deprecated 改用 startArmStableCheck + tickArmStableCheck 非阻塞流程
   */
  bool waitForArmStable(int timeout_ms, double threshold_rad, int stable_window);

  /**
   * @brief 启动非阻塞机械臂稳定检查（每次 tick 调用 tickArmStableCheck 一次）
   * @param timeout_ms 最大等待时间 (ms)
   * @param threshold_rad 位置稳定判定阈值 (rad)
   * @param stable_window 连续满足阈值的采样次数
   * @param velocity_eps_rad_s 速度判稳阈值 (rad/s)，max(|vel|) 需小于此值；<=0 表示不检查速度
   * @param min_dwell_ms 满足稳定后至少再停留 (ms) 才返回 done；<=0 表示不要求最小停留
   */
  void startArmStableCheck(int timeout_ms, double threshold_rad, int stable_window,
                           double velocity_eps_rad_s = 0.0, int min_dwell_ms = 0);

  /**
   * @brief 非阻塞轮询一次稳定检查（不 sleep，供 FSM 每 tick 调用）
   * @param now 当前时间（与 startArmStableCheck 同源时钟）
   * @return (done, stable)：done 为 true 表示可进入下一步，stable 表示是否达到稳定（false 表示超时）
   */
  std::pair<bool, bool> tickArmStableCheck(const rclcpp::Time& now);

  /**
   * @brief 设置“下发前”回调：在修复并设置 controller 后的 solution 发送给 ExecuteTaskSolution 前调用，用于将轨迹发布到网页等
   */
  void setOnSolutionBeforeSend(std::function<void(const moveit_task_constructor_msgs::msg::Solution&)> cb) {
    on_solution_before_send_ = std::move(cb);
  }

private:
  /**
   * @brief 规划任务
   * @param task MTC任务
   * @return true 如果规划成功
   */
  bool planTask(mtc::Task& task);

  /**
   * @brief 执行已规划的任务解（异步执行）
   * @param task MTC任务
   * @param task_name 任务名称（用于日志和状态管理）
   * @return true 如果成功启动异步执行
   */
  bool executePlannedTask(mtc::Task& task, const std::string& task_name);

  /**
   * @brief 清理MoveGroup状态（避免残留约束）
   */
  void cleanupMoveGroupState();

  /**
   * @brief 修复轨迹时间戳
   * 
   * MTC生成的轨迹可能存在时间戳问题：
   * - 第一个点和第二个点的time_from_start都是0
   * - 时间戳不严格递增
   * 这会导致ros2_control的joint_trajectory_controller拒绝执行
   * 
   * @param sol_msg 需要修复的解决方案消息
   */
  void fixTrajectoryTimestamps(moveit_task_constructor_msgs::msg::Solution& sol_msg);

  /**
   * @brief 为 Solution 中每个 sub_trajectory 设置 execution_info.controller_names
   * 消除 "stage does not have any controllers specified" 警告，明确 arm/gripper 归属
   */
  void setControllerNamesForSolution(moveit_task_constructor_msgs::msg::Solution& sol_msg);

  std::function<void(const moveit_task_constructor_msgs::msg::Solution&)> on_solution_before_send_;

  TaskContext ctx_;
  std::unique_ptr<TaskFactory> factory_;
  std::atomic<bool> emergency_stop_{false};
  
  // ========== Action Client 成员变量（确保生命周期）==========
  // 问题：如果 action client 是局部变量，函数返回后会被析构，导致回调无法触发
  // 解决：将 action client 提升为成员变量，确保在整个 TaskRunner 生命周期内存在
  rclcpp_action::Client<moveit_task_constructor_msgs::action::ExecuteTaskSolution>::SharedPtr execute_client_;
  std::atomic<bool> execute_client_initialized_{false};
  
  // ========== 异步执行状态（供 FSM 检查）==========
  // 问题：在 FSM callback 中阻塞等待会占用 executor worker 线程，导致回调无法及时调度
  // 解决：executePlannedTask() 异步化，FSM 通过定期检查状态推进
  struct AsyncExecState {
    std::mutex m;
    std::atomic<bool> goal_sent{false};
    std::atomic<bool> goal_accepted{false};
    std::atomic<bool> execution_completed{false};
    std::atomic<bool> execution_succeeded{false};
    std::string task_name;
    std::chrono::steady_clock::time_point start_time;
    std::chrono::seconds timeout;
  };
  
  std::shared_ptr<AsyncExecState> current_exec_state_;  // 当前执行状态（nullptr 表示没有在执行）

  // 非阻塞机械臂稳定检查（供 FSM WAIT_ARM_STABLE 每 tick 调用）
  bool arm_stable_check_active_{false};
  rclcpp::Time arm_stable_start_{0, 0};
  std::vector<double> arm_stable_prev_joints_;
  int arm_stable_count_{0};
  int arm_stable_timeout_ms_{0};
  double arm_stable_threshold_rad_{0.0};
  int arm_stable_window_{0};
  double arm_stable_velocity_eps_rad_s_{0.0};  // 速度判稳阈值，<=0 不检查
  int arm_stable_min_dwell_ms_{0};             // 稳定后最小停留 ms，<=0 不要求
  std::shared_ptr<ExecWaitState> current_wait_state_;  // 当前等待状态（用于回调）
};

}  // namespace m5_grasp
