#include "m5_grasp/mtc/task_runner.hpp"
#include "m5_grasp/logging/logger.hpp"

#include <thread>
#include <sstream>
#include <future>
#include <condition_variable>
#include <mutex>
#include <chrono>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/executors.hpp>

namespace m5_grasp {

TaskRunner::TaskRunner(const TaskContext& ctx)
  : ctx_(ctx)
  , factory_(std::make_unique<TaskFactory>(ctx))
  , current_exec_state_(nullptr)
  , current_wait_state_(nullptr)
{
  // Action client 延迟初始化，在 initializeActionClient() 中创建
  // 因为此时 ctx_ 可能还没有完全准备好（node 和 callback_group_fast）
}

bool TaskRunner::initializeActionClient() {
  // ========== 关键修复：延迟初始化 action client，确保生命周期 ==========
  // 问题：如果 action client 是局部变量，函数返回后会被析构，导致回调无法触发
  // 解决：将 action client 提升为成员变量，在初始化时创建一次，确保在整个生命周期内存在
  
  if (execute_client_initialized_.load()) {
    LOG_NAMED_INFO("task_runner", "Action client 已初始化，跳过重复初始化");
    return true;
  }
  
  if (!ctx_.node) {
    LOG_NAMED_ERROR("task_runner", "ctx_.node 为空，无法初始化 action client");
    return false;
  }
  
  if (!ctx_.callback_group_fast) {
    LOG_NAMED_ERROR("task_runner", "ctx_.callback_group_fast 为空，无法初始化 action client");
    return false;
  }
  
  // 打印 node 指针地址，验证是否是同一个 node 实例
  void* node_ptr = ctx_.node.get();
  void* node_base_ptr = ctx_.node->get_node_base_interface().get();
  LOG_NAMED_INFO("task_runner", "初始化 action client: node={}, node_ptr={}, node_base_ptr={}, callback_group={} (Reentrant)",
                 ctx_.node->get_name(), 
                 node_ptr,
                 node_base_ptr,
                 ctx_.callback_group_fast ? "已设置" : "未设置");
  
  // 验证 callback group 类型
  if (ctx_.callback_group_fast) {
    auto group_type = ctx_.callback_group_fast->type();
    LOG_NAMED_INFO("task_runner", "  callback_group 类型: {} (0=MutuallyExclusive, 1=Reentrant)",
                   static_cast<int>(group_type));
    if (group_type != rclcpp::CallbackGroupType::Reentrant) {
      LOG_NAMED_WARN("task_runner", "  ⚠️ callback_group 不是 Reentrant！这可能导致回调无法并发执行");
    }
  }
  
  // 创建 action client 并保存为成员变量
  execute_client_ = rclcpp_action::create_client<moveit_task_constructor_msgs::action::ExecuteTaskSolution>(
      ctx_.node, "execute_task_solution", ctx_.callback_group_fast);
  
  if (!execute_client_) {
    LOG_NAMED_ERROR("task_runner", "创建 action client 失败");
    return false;
  }
  
  // 注意：use_count=1 是正常的（成员变量本身就是一个引用），不代表会析构
  LOG_NAMED_INFO("task_runner", "  action client 创建完成，use_count={}", execute_client_.use_count());
  
  // 等待 action server 可用
  LOG_NAMED_INFO("task_runner", "  等待 execute_task_solution action server 可用...");
  if (!execute_client_->wait_for_action_server(std::chrono::seconds(30))) {
    LOG_NAMED_ERROR("task_runner", "execute_task_solution action server不可用（30秒内未响应）");
    execute_client_.reset();
    return false;
  }
  
  LOG_NAMED_INFO("task_runner", "  ✓ action server 已可用，action client 初始化完成");
  execute_client_initialized_.store(true);
  return true;
}

void TaskRunner::setEmergencyStop(bool stop) {
  emergency_stop_.store(stop);
}

bool TaskRunner::isEmergencyStopped() const {
  return emergency_stop_.load();
}

bool TaskRunner::runPregrasp(const TaskTarget& target) {
  LOG_NAMED_INFO("task_runner", "[runPregrasp] 开始执行预抓取任务");
  mtc::Task task = factory_->makePregrasp(target);
  return executeTask(task, "pregrasp");
}

bool TaskRunner::runAlign(const TaskTarget& target) {
  LOG_NAMED_INFO("task_runner", "[runAlign] 开始执行对齐任务");
  mtc::Task task = factory_->makeAlign(target);
  return executeTask(task, "align");
}

bool TaskRunner::runDescend(const TaskTarget& target) {
  LOG_NAMED_INFO("task_runner", "[runDescend] 开始执行下探任务（移到 grasp_pose，OMPL）");
  mtc::Task task = factory_->makeDescend(target);
  return executeTask(task, "descend");
}

bool TaskRunner::runLift(const TaskTarget& target) {
  LOG_NAMED_INFO("task_runner", "[runLift] 开始执行抬升任务（固定 Joint4 = {:.1f}°）",
                 target.joint4_target * 180.0 / M_PI);
  const std::map<std::string, double>* start_joints =
      !target.grasp_joint_map.empty() ? &target.grasp_joint_map : nullptr;
  mtc::Task task = factory_->makeLift(-1.0, target.joint4_target, start_joints);
  return executeTask(task, "lift");
}

namespace {
  // 范围执行时间（全行程）：见 cable_grasp.yaml trajectory.range_execution_time
  // Joint1: 30s | Joint2: 13s | Joint3: 13s | Joint4: 21s | 夹爪: 7.5s

  /**
   * @brief 根据关节名称获取最大速度
   * 
   * 基于实际测试数据：
   * - Joint1: -355° 到 355° (710° = 12.39 rad), 30s -> 0.413 rad/s
   * - Joint2: -170° 到 0° (170° = 2.97 rad), 13s -> 0.228 rad/s
   * - Joint3: -170° 到 0° (170° = 2.97 rad), 13s -> 0.228 rad/s
   * - Joint4: -355° 到 355° (710° = 12.39 rad), 21s -> 0.590 rad/s
   * - 夹爪: -1100 到 0 (范围值，厂家实际), 7.5s 全行程
   * 
   * 注意：夹爪不是弧度，而是范围值（可能是编码器值），不同驱动可能对应不同范围
   */
  // 全行程时间→估算最大角速度（与 cable_grasp.yaml range_execution_time 一致）
  // J1 12.4rad/30s≈0.41, J2/J3 2.97rad/13s≈0.23, J4 12.4rad/21s≈0.59, 夹爪单侧 2.02rad/7.5s≈0.27
  double getMaxJointVelocity(const std::string& joint_name) {
    if (joint_name == "Joint1") {
      return 0.41;  // rad/s (12.4 rad / 30 s)
    } else if (joint_name == "Joint2" || joint_name == "Joint3") {
      return 0.23;  // rad/s (2.97 rad / 13 s)
    } else if (joint_name == "Joint4") {
      return 0.59;  // rad/s (12.4 rad / 21 s)
    } else if (joint_name == "JointGL" || joint_name == "JointGR") {
      return 0.27;  // rad/s (单侧 2.02 rad / 7.5 s)
    }
    return 0.3;  // rad/s 默认
  }
  
  /**
   * @brief 计算轨迹的总执行时间（duration）
   * 
   * 从Solution消息中提取所有子轨迹的最后一个点的time_from_start
   * 返回最大的time_from_start作为总duration
   * 
   * @param sol_msg 解决方案消息
   * @return 总执行时间（秒）
   */
  double calculateTrajectoryDuration(const moveit_task_constructor_msgs::msg::Solution& sol_msg) {
    double max_duration = 0.0;
    
    for (const auto& sub_traj : sol_msg.sub_trajectory) {
      const auto& joint_traj = sub_traj.trajectory.joint_trajectory;
      if (!joint_traj.points.empty()) {
        const auto& last_point = joint_traj.points.back();
        double duration = last_point.time_from_start.sec + 
                         last_point.time_from_start.nanosec * 1e-9;
        if (duration > max_duration) {
          max_duration = duration;
        }
      }
    }
    
    return max_duration;
  }
  
  /**
   * @brief 根据关节运动量估算执行时间上限
   * 
   * 基于关节最大速度和实际运动量，估算最坏情况下的执行时间
   * 用于设置超时时间的上限
   * 
   * @param sol_msg 解决方案消息
   * @return 估算的执行时间上限（秒）
   */
  double estimateMaxExecutionTime(const moveit_task_constructor_msgs::msg::Solution& sol_msg) {
    double max_time = 0.0;
    
    for (const auto& sub_traj : sol_msg.sub_trajectory) {
      const auto& joint_traj = sub_traj.trajectory.joint_trajectory;
      if (joint_traj.points.size() < 2) continue;
      
      // 计算每个关节的最大运动量
      std::map<std::string, double> joint_max_diff;
      
      for (size_t i = 0; i < joint_traj.joint_names.size(); ++i) {
        const std::string& joint_name = joint_traj.joint_names[i];
        double min_val = joint_traj.points[0].positions[i];
        double max_val = joint_traj.points[0].positions[i];
        
        for (const auto& point : joint_traj.points) {
          if (i < point.positions.size()) {
            min_val = std::min(min_val, point.positions[i]);
            max_val = std::max(max_val, point.positions[i]);
          }
        }
        
        double diff = max_val - min_val;
        if (diff > 0) {
          joint_max_diff[joint_name] = diff;
        }
      }
      
      // 计算每个关节所需时间，取最大值
      double stage_max_time = 0.0;
      for (const auto& [joint_name, diff] : joint_max_diff) {
        double max_vel = getMaxJointVelocity(joint_name);
        if (max_vel > 0) {
          double joint_time = diff / max_vel;
          if (joint_time > stage_max_time) {
            stage_max_time = joint_time;
          }
        }
      }
      
      if (stage_max_time > max_time) {
        max_time = stage_max_time;
      }
    }
    
    return max_time;
  }

  /**
   * @brief 调参用：打印 Solution 的 plan/轨迹修复后摘要（子轨迹数、点数、时长、首末关节位置）
   */
  void logSolutionForTuning(const char* label,
                            const moveit_task_constructor_msgs::msg::Solution& sol_msg) {
    LOG_NAMED_INFO("task_runner", "========== {} ==========", label);
    int idx = 0;
    for (const auto& sub_traj : sol_msg.sub_trajectory) {
      const auto& jt = sub_traj.trajectory.joint_trajectory;
      idx++;
      if (jt.points.empty()) {
        LOG_NAMED_INFO("task_runner", "  sub#{} empty", idx);
        continue;
      }
      bool is_gripper = false;
      for (const auto& n : jt.joint_names) {
        if (n == "JointGL" || n == "JointGR") { is_gripper = true; break; }
      }
      double duration = 0.0;
      {
        const auto& last = jt.points.back();
        duration = last.time_from_start.sec + last.time_from_start.nanosec * 1e-9;
      }
      // 首、末点关节位置（便于核对夹爪是否闭合、机械臂各轴）
      std::string first_pos, last_pos;
      const auto& fp = jt.points.front().positions;
      const auto& lp = jt.points.back().positions;
      for (size_t i = 0; i < jt.joint_names.size() && i < fp.size(); ++i) {
        if (i > 0) { first_pos += ","; last_pos += ","; }
        double f = std::round(fp[i] * 1000.0) / 1000.0;
        double l = std::round(lp[i] * 1000.0) / 1000.0;
        first_pos += std::to_string(f);
        last_pos += std::to_string(l);
      }
      LOG_NAMED_INFO("task_runner",
                     "  sub#{} {} points={} duration={:.3f}s first=[{}] last=[{}]",
                     idx, is_gripper ? "夹爪" : "机械臂", jt.points.size(), duration,
                     first_pos.empty() ? "-" : first_pos.c_str(),
                     last_pos.empty() ? "-" : last_pos.c_str());
    }
    LOG_NAMED_INFO("task_runner", "========================================");
  }
}  // namespace

void TaskRunner::fixTrajectoryTimestamps(moveit_task_constructor_msgs::msg::Solution& sol_msg) {
  // 轨迹时间修复参数（范围执行时间见 trajectory.range_execution_time）
  const double MIN_SEGMENT_TIME = 0.05;    // 最小段时间50ms（提高平滑度）
  const double MIN_GRIPPER_TRAJECTORY_TIME = 7.5;   // 夹爪 7.5s 全行程
  const double MIN_ARM_TRAJECTORY_TIME = 1.0;       // 机械臂轨迹最小 1s（仅防 0 时长；执行超时由 MIN_TIMEOUT 35s 保证）
  
  int traj_index = 0;
  for (auto& sub_traj : sol_msg.sub_trajectory) {
    auto& joint_traj = sub_traj.trajectory.joint_trajectory;
    traj_index++;
    
    if (joint_traj.points.size() < 2) {
      continue;
    }
    
    // 检测是否是夹爪轨迹（通过关节名称判断）
    bool is_gripper_traj = false;
    for (const auto& name : joint_traj.joint_names) {
      if (name == "JointGL" || name == "JointGR") {
        is_gripper_traj = true;
        break;
      }
    }
    
    // 获取原始轨迹时间
    double original_last_time = joint_traj.points.back().time_from_start.sec + 
                               joint_traj.points.back().time_from_start.nanosec * 1e-9;
    
    // 确定最小轨迹时间
    double min_traj_time = is_gripper_traj ? MIN_GRIPPER_TRAJECTORY_TIME : MIN_ARM_TRAJECTORY_TIME;
    
    // 检查是否需要修复：时间太短或不递增
    bool needs_fix = (original_last_time < min_traj_time);
    
    if (!needs_fix) {
      for (size_t i = 1; i < joint_traj.points.size(); ++i) {
        auto& prev_time = joint_traj.points[i-1].time_from_start;
        auto& curr_time = joint_traj.points[i].time_from_start;
        double prev_sec = prev_time.sec + prev_time.nanosec * 1e-9;
        double curr_sec = curr_time.sec + curr_time.nanosec * 1e-9;
        if (curr_sec <= prev_sec) {
          needs_fix = true;
          break;
        }
      }
    }
    
    LOG_NAMED_DEBUG("task_runner", "[轨迹诊断] 子轨迹#{}: {}, {}个点, 原始时长={:.3f}s, 需要修复={}",
                   traj_index, is_gripper_traj ? "夹爪" : "机械臂",
                   joint_traj.points.size(), original_last_time, needs_fix ? "是" : "否");
    
    if (needs_fix) {
      // 重新计算所有点的时间戳，基于关节运动量和每个关节的实际最大速度
      double accumulated_time = 0.0;
      joint_traj.points[0].time_from_start.sec = 0;
      joint_traj.points[0].time_from_start.nanosec = 0;
      
      for (size_t i = 1; i < joint_traj.points.size(); ++i) {
        // 计算从上一点到当前点，每个关节所需的时间，取最大值
        double max_segment_time = 0.0;
        const auto& prev_positions = joint_traj.points[i-1].positions;
        const auto& curr_positions = joint_traj.points[i].positions;
        
        for (size_t j = 0; j < std::min(prev_positions.size(), curr_positions.size()) && 
                          j < joint_traj.joint_names.size(); ++j) {
          double diff = std::abs(curr_positions[j] - prev_positions[j]);
          if (diff > 1e-6) {  // 忽略微小变化
            // 获取该关节的最大速度
            double max_vel = getMaxJointVelocity(joint_traj.joint_names[j]);
            // 计算该关节完成这段运动所需的时间
            double joint_time = diff / max_vel;
            if (joint_time > max_segment_time) {
              max_segment_time = joint_time;
            }
          }
        }
        
        // 使用最大时间（最慢的关节决定整段的时间）
        double segment_time = max_segment_time;
        if (segment_time < MIN_SEGMENT_TIME) {
          segment_time = MIN_SEGMENT_TIME;
        }
        
        accumulated_time += segment_time;
        auto& curr_time = joint_traj.points[i].time_from_start;
        curr_time.sec = static_cast<int32_t>(accumulated_time);
        curr_time.nanosec = static_cast<uint32_t>((accumulated_time - curr_time.sec) * 1e9);
      }
      
      // 如果计算出的时间仍然小于最小时间，进行缩放
      if (accumulated_time < min_traj_time) {
        double scale = min_traj_time / accumulated_time;
        for (size_t i = 1; i < joint_traj.points.size(); ++i) {
          double old_time = joint_traj.points[i].time_from_start.sec + 
                           joint_traj.points[i].time_from_start.nanosec * 1e-9;
          double new_time = old_time * scale;
          joint_traj.points[i].time_from_start.sec = static_cast<int32_t>(new_time);
          joint_traj.points[i].time_from_start.nanosec = static_cast<uint32_t>((new_time - static_cast<int32_t>(new_time)) * 1e9);
        }
        accumulated_time = min_traj_time;
      }
      
      LOG_NAMED_INFO("task_runner", "[轨迹修复] 子轨迹#{} ({}): {}个点, 修复后时长={:.2f}s",
                    traj_index, is_gripper_traj ? "夹爪" : "机械臂",
                    joint_traj.points.size(), accumulated_time);
    }

    // 厂家建议：所有关节轨迹（机械臂 + 夹爪）步长不要太短(如1度)，约10度发一点，点要散一点减少抖动
    if (ctx_.min_joint_step_rad > 0 && joint_traj.points.size() > 2) {
      std::vector<size_t> keep = {0};
      size_t last = 0;
      const size_t n_joints = joint_traj.points[0].positions.size();
      for (size_t i = 1; i < joint_traj.points.size() - 1; ++i) {
        double max_diff = 0.0;
        for (size_t j = 0; j < n_joints && j < joint_traj.points[i].positions.size() && j < joint_traj.points[last].positions.size(); ++j) {
          double d = std::abs(joint_traj.points[i].positions[j] - joint_traj.points[last].positions[j]);
          if (d > max_diff) max_diff = d;
        }
        if (max_diff >= ctx_.min_joint_step_rad) {
          keep.push_back(i);
          last = i;
        }
      }
      keep.push_back(joint_traj.points.size() - 1);
      const size_t original_count = joint_traj.points.size();
      if (keep.size() < original_count) {
        std::vector<trajectory_msgs::msg::JointTrajectoryPoint> new_points;
        new_points.reserve(keep.size());
        for (size_t k : keep) {
          new_points.push_back(joint_traj.points[k]);
        }
        joint_traj.points = std::move(new_points);
        LOG_NAMED_INFO("task_runner", "[轨迹稀疏化] 子轨迹#{} ({}): {} -> {} 个点（最小步长 {:.1f}°）",
                       traj_index, is_gripper_traj ? "夹爪" : "机械臂",
                       static_cast<int>(original_count), static_cast<int>(joint_traj.points.size()),
                       ctx_.min_joint_step_rad * 180.0 / M_PI);
      }
    }

    // 无运动轨迹检测：首尾关节位置差小于阈值则视为无运动，缩短总时长为 0.1s，避免 5s 空跑
    const double NO_MOTION_THRESHOLD_RAD = 1e-4;  // 约 0.006°
    const double NO_MOTION_DURATION_SEC = 0.1;
    if (joint_traj.points.size() >= 2) {
      const auto& first_pos = joint_traj.points.front().positions;
      const auto& last_pos = joint_traj.points.back().positions;
      double max_diff = 0.0;
      for (size_t j = 0; j < first_pos.size() && j < last_pos.size(); ++j) {
        double d = std::abs(first_pos[j] - last_pos[j]);
        if (d > max_diff) max_diff = d;
      }
      if (max_diff < NO_MOTION_THRESHOLD_RAD) {
        // 无运动：只保留首尾两点，末点时间设为 0.1s
        trajectory_msgs::msg::JointTrajectoryPoint first_pt = joint_traj.points.front();
        trajectory_msgs::msg::JointTrajectoryPoint last_pt = joint_traj.points.back();
        first_pt.time_from_start.sec = 0;
        first_pt.time_from_start.nanosec = 0;
        last_pt.time_from_start.sec = static_cast<int32_t>(NO_MOTION_DURATION_SEC);
        last_pt.time_from_start.nanosec = static_cast<uint32_t>(
            (NO_MOTION_DURATION_SEC - std::floor(NO_MOTION_DURATION_SEC)) * 1e9);
        joint_traj.points = {first_pt, last_pt};
        LOG_NAMED_INFO("task_runner", "[无运动轨迹] 子轨迹#{} ({}): 首尾差 max={:.2e} rad < {:.2e}，时长缩短为 {:.2f}s",
                       traj_index, is_gripper_traj ? "夹爪" : "机械臂", max_diff, NO_MOTION_THRESHOLD_RAD, NO_MOTION_DURATION_SEC);
      }
    }
  }
}

void TaskRunner::setControllerNamesForSolution(moveit_task_constructor_msgs::msg::Solution& sol_msg) {
  const std::string ARM_CONTROLLER = "arm_group_controller";
  const std::string GRIPPER_CONTROLLER = "gripper_group_controller";

  for (auto& sub_traj : sol_msg.sub_trajectory) {
    auto& joint_traj = sub_traj.trajectory.joint_trajectory;
    bool is_gripper = false;
    for (const auto& name : joint_traj.joint_names) {
      if (name == "JointGL" || name == "JointGR") {
        is_gripper = true;
        break;
      }
    }
    if (sub_traj.execution_info.controller_names.empty()) {
      sub_traj.execution_info.controller_names.resize(1);
      sub_traj.execution_info.controller_names[0] = is_gripper ? GRIPPER_CONTROLLER : ARM_CONTROLLER;
      LOG_NAMED_DEBUG("task_runner", "stage controller 指定: {} -> {}",
                      is_gripper ? "夹爪" : "机械臂",
                      sub_traj.execution_info.controller_names[0]);
    }
  }
}

void TaskRunner::cleanupMoveGroupState() {
  if (!ctx_.moveit_mutex || !ctx_.move_group) {
    return;
  }
  
  std::lock_guard<std::mutex> lock(*ctx_.moveit_mutex);
  ctx_.move_group->clearPathConstraints();
  ctx_.move_group->clearPoseTargets();
  
  const auto& robot_model = ctx_.move_group->getRobotModel();
  if (!robot_model) {
    ctx_.move_group->setStartStateToCurrentState();
    return;
  }
  
  auto start_state = std::make_shared<moveit::core::RobotState>(robot_model);
  start_state->setToDefaultValues();
  bool arm_ok = false;
  bool gripper_ok = false;
  
  // 与夹爪一致：优先 diag、新鲜度 15s，再 MoveIt。open_gripper 等待 8s 期间 executor 可能未处理 joint_states，last_js_wall 会超过 5s；放宽到 15s 避免退化为 setStartStateToCurrentState（MoveIt 拿不到有效状态时 MTC 起点错误）
  rclcpp::Time now = ctx_.node ? ctx_.node->get_clock()->now() : rclcpp::Time(0, 0);
  constexpr double kDiagMaxAgeSec = 15.0;

  const auto* arm_jmg = robot_model->getJointModelGroup(ctx_.arm_group);
  if (arm_jmg) {
    std::vector<double> arm_joint_values;
    const auto& arm_names = arm_jmg->getActiveJointModelNames();
    if (ctx_.joint_state_diag && ctx_.node &&
        fillJointPositionsFromDiag(ctx_.joint_state_diag, arm_names, arm_joint_values, now, kDiagMaxAgeSec) &&
        arm_joint_values.size() == arm_names.size()) {
      start_state->setJointGroupPositions(arm_jmg, arm_joint_values);
      arm_ok = true;
    }
    if (!arm_ok) {
      try {
        arm_joint_values = ctx_.move_group->getCurrentJointValues();
      } catch (const std::exception& e) {
        (void)e;
        arm_joint_values.clear();
      }
      if (arm_joint_values.size() != arm_names.size() && ctx_.joint_state_diag && ctx_.node &&
          fillJointPositionsFromDiag(ctx_.joint_state_diag, arm_names, arm_joint_values, now, kDiagMaxAgeSec)) {
        LOG_NAMED_INFO("task_runner", "cleanupMoveGroupState: arm 使用 joint_state_diag 后备（getCurrentJointValues 不可用）");
      }
      if (arm_joint_values.size() == arm_names.size()) {
        start_state->setJointGroupPositions(arm_jmg, arm_joint_values);
        arm_ok = true;
      }
    }
  }

  if (ctx_.gripper_group_interface) {
    const auto* gripper_jmg = robot_model->getJointModelGroup(ctx_.gripper_group);
    if (gripper_jmg) {
      std::vector<double> gripper_joint_values;
      const auto& grip_names = gripper_jmg->getActiveJointModelNames();
      if (ctx_.joint_state_diag && ctx_.node &&
          fillJointPositionsFromDiag(ctx_.joint_state_diag, grip_names, gripper_joint_values, now, kDiagMaxAgeSec) &&
          gripper_joint_values.size() == grip_names.size()) {
        start_state->setJointGroupPositions(gripper_jmg, gripper_joint_values);
        gripper_ok = true;
      }
      if (!gripper_ok) {
        try {
          gripper_joint_values = ctx_.gripper_group_interface->getCurrentJointValues();
        } catch (const std::exception& e) {
          (void)e;
          gripper_joint_values.clear();
        }
        if (gripper_joint_values.size() != grip_names.size() && ctx_.joint_state_diag && ctx_.node &&
            fillJointPositionsFromDiag(ctx_.joint_state_diag, grip_names, gripper_joint_values, now, kDiagMaxAgeSec)) {
          LOG_NAMED_INFO("task_runner", "cleanupMoveGroupState: gripper 使用 joint_state_diag 后备（getCurrentJointValues 不可用）");
        }
        if (gripper_joint_values.size() == grip_names.size()) {
          start_state->setJointGroupPositions(gripper_jmg, gripper_joint_values);
          gripper_ok = true;
        }
      }
    }
  }
  
  if (arm_ok) {
    start_state->update();
    start_state->enforceBounds();
    ctx_.move_group->setStartState(*start_state);
    LOG_NAMED_DEBUG("task_runner", "已清理MoveGroup状态");
  } else {
    LOG_NAMED_WARN("task_runner", "cleanupMoveGroupState: 无法取得 arm 关节（含 joint_state_diag），退化为 setStartStateToCurrentState");
    ctx_.move_group->setStartStateToCurrentState();
  }
}

bool TaskRunner::planTask(mtc::Task& task) {
  LOG_NAMED_INFO("task_runner", "[planTask] 进入");
  // ========== 关键修复：初始化Task ==========
  // 注意：task.init() 在 introspection 已禁用的情况下不应该创建新的 node/publisher
  // 我们在 task_factory 中已经禁用了 introspection (task.enableIntrospection(false))
  // 这样可以避免创建新的 publisher，防止 "Publisher already registered" 警告
  // task.init() 不需要 node 参数，它会使用 Task 创建时设置的 robot model
  try {
    LOG_NAMED_INFO("task_runner", "[planTask] 即将 task.init()");
    // ========== 关键诊断：task.init() 可能创建新的 publisher ==========
    // 注意：即使禁用了 introspection，task.init() 可能仍然会创建内部 publisher
    // 这会导致 "Publisher already registered" 警告
    // 但这不是致命问题，只是警告
    task.init();
    LOG_NAMED_INFO("task_runner", "MTC Task初始化成功");
  } catch (const std::exception& e) {
    LOG_NAMED_ERROR("task_runner", "MTC Task初始化失败: {}", e.what());
    return false;
  } catch (...) {
    LOG_NAMED_ERROR("task_runner", "MTC Task初始化失败: 未知异常");
    return false;
  }
  
  // ========== 关键修复：在独立线程中执行 task.plan()，用 wait_for 实现真正超时 ==========
  // 原先“规划超时检查”在 plan() 返回之后，若 plan() 内部阻塞（如 descend 的 CurrentState/Cartesian 等 planning scene）则永远不触发。
  // 改为 std::async + future.wait_for，超时则 preempt 并返回，避免整机卡死。
  if (emergency_stop_) {
    LOG_NAMED_WARN("task_runner", "急停触发，停止Task规划");
    return false;
  }
  
  constexpr auto planning_timeout = std::chrono::seconds(90);
  
  try {
    LOG_NAMED_INFO("task_runner", "开始执行task.plan()...（超时{}秒）", planning_timeout.count());
    auto future = std::async(std::launch::async, [&task]() { task.plan(); });
    std::future_status status = future.wait_for(planning_timeout);
    
    if (status == std::future_status::timeout) {
      LOG_NAMED_ERROR("task_runner", "task.plan() 超时（{}秒），可能被 CurrentState/Cartesian 等阻塞，调用 preempt 并中止",
                      planning_timeout.count());
      task.preempt();
      return false;
    }
    future.get();
    
    // ========== 关键修复：访问 task.solutions() 时添加异常捕获 ==========
    try {
      size_t solution_count = task.solutions().size();
      LOG_NAMED_INFO("task_runner", "task.plan()完成，找到 {} 个解", solution_count);
      
      if (solution_count == 0) {
        LOG_NAMED_ERROR("task_runner", "MTC Task规划失败：没有找到解");
        
        // 输出Task状态用于调试
        try {
          std::stringstream ss;
          std::streambuf* old_cout = std::cout.rdbuf(ss.rdbuf());
          task.printState();
          std::cout.rdbuf(old_cout);
          
          std::string state_output = ss.str();
          if (!state_output.empty()) {
            std::istringstream iss(state_output);
            std::string line;
            while (std::getline(iss, line)) {
              LOG_NAMED_ERROR("task_runner", "Task状态: {}", line.c_str());
            }
          }
        } catch (...) {
          // 忽略
        }
        
        return false;
      }
    } catch (const std::exception& e) {
      LOG_NAMED_ERROR("task_runner", "访问 task.solutions() 时异常: {}", e.what());
      return false;
    } catch (...) {
      LOG_NAMED_ERROR("task_runner", "访问 task.solutions() 时未知异常");
      return false;
    }
  } catch (const std::exception& e) {
    LOG_NAMED_ERROR("task_runner", "task.plan()异常: {}", e.what());
    return false;
  } catch (...) {
    LOG_NAMED_ERROR("task_runner", "task.plan()未知异常");
    return false;
  }
  
  return true;
}

bool TaskRunner::executePlannedTask(mtc::Task& task, const std::string& task_name) {
  // ========== 关键修复：添加异常捕获，防止崩溃 ==========
  try {
    const auto& solutions = task.solutions();
    if (solutions.empty()) {
      LOG_NAMED_ERROR("task_runner", "没有可执行的解");
      return false;
    }
    
    // 输出解的cost信息
    if (solutions.size() > 1) {
      LOG_NAMED_INFO("task_runner", "[Solution选择] 共 {} 个解，显示前3个的cost:", solutions.size());
      size_t count = 0;
      for (const auto& sol : solutions) {
        if (count >= 3) break;
        LOG_NAMED_INFO("task_runner", "  解{}: cost={:.3f}", count + 1, sol->cost());
        count++;
      }
    }
    
    const auto& solution = solutions.front();
    
    // 检查急停
    if (emergency_stop_) {
      LOG_NAMED_WARN("task_runner", "急停触发，停止Task执行");
      return false;
    }
    
    // ========== 关键修复：使用成员变量 action client，确保生命周期 ==========
    // 问题：如果 action client 是局部变量，函数返回后会被析构，导致回调无法触发
    // 解决：使用成员变量 execute_client_，确保在整个执行过程中保持存活
    
    // 检查 action client 是否已初始化
    if (!execute_client_initialized_.load()) {
      LOG_NAMED_WARN("task_runner", "Action client 未初始化，尝试初始化...");
      if (!initializeActionClient()) {
        LOG_NAMED_ERROR("task_runner", "Action client 初始化失败，无法执行任务");
        return false;
      }
    }
    
    if (!execute_client_) {
      LOG_NAMED_ERROR("task_runner", "execute_client_ 为空，无法执行任务");
      return false;
    }
    
    // ========== 关键诊断：打印 use_count 确认 client 未被析构 ==========
    // 注意：use_count=1 是正常的（成员变量本身就是一个引用），不代表会析构
    auto use_count = execute_client_.use_count();
    LOG_NAMED_INFO("task_runner", "使用成员变量 action client，use_count={}", use_count);
    
    // ========== 关键修复：转换解决方案为ROS消息 ==========
    // 问题：即使禁用了 introspection，调用 task.introspection() 可能仍然会创建内部对象或触发断言
    // 解决：直接传入 nullptr，因为我们已经在 task_factory 中禁用了 introspection
    // 这样可以避免 introspection 对象的初始化，防止创建新的 node/publisher
    moveit_task_constructor_msgs::msg::Solution sol_msg;
    
    // 关键修复：即使 introspection 已禁用，调用 task.introspection() 也可能触发内部初始化
    // 因此直接传入 nullptr，避免任何 introspection 相关的调用
    // 这样可以防止创建新的 node/publisher，避免 "Publisher already registered" 警告
    // 添加异常捕获，防止 solution->toMsg() 调用时崩溃
    try {
      solution->toMsg(sol_msg, nullptr);
      LOG_NAMED_DEBUG("task_runner", "使用 nullptr 作为 introspection 参数（introspection 已禁用）");
    } catch (const std::exception& e) {
      LOG_NAMED_ERROR("task_runner", "solution->toMsg() 异常: {}", e.what());
      return false;
    } catch (...) {
      LOG_NAMED_ERROR("task_runner", "solution->toMsg() 未知异常");
      return false;
    }

    // 调参用：打印 plan 原始结果（toMsg 之后、修复前）
    logSolutionForTuning("[Plan 原始]", sol_msg);

    // ========== 修复轨迹时间戳 ==========
  // 问题：MTC生成的轨迹中，第一个点和第二个点的time_from_start可能都是0
  // 这会导致ros2_control的joint_trajectory_controller拒绝执行
  // 解决方案：检查并修复轨迹时间戳
  fixTrajectoryTimestamps(sol_msg);

  // ========== 为每个 stage 指定 controller，消除 "stage does not have any controllers specified" 警告 ==========
  setControllerNamesForSolution(sol_msg);

  // ========== 下发前回调：供主节点将修复后的轨迹发布到网页等，保证 JSON 与执行轨迹一致 ==========
  if (on_solution_before_send_) {
    try {
      on_solution_before_send_(sol_msg);
    } catch (const std::exception& e) {
      LOG_NAMED_WARN("task_runner", "on_solution_before_send 回调异常: {}", e.what());
    }
  }

  // 调参用：打印轨迹修复/优化后的结果（发给 controller 前）
  logSolutionForTuning("[轨迹修复后]", sol_msg);

  // ========== 动态计算超时时间 ==========
  // 根据轨迹实际duration和关节运动量估算执行时间
  // 不使用固定超时，避免"看起来执行成功但超时失败"的问题
  double trajectory_duration = calculateTrajectoryDuration(sol_msg);
  double estimated_max_time = estimateMaxExecutionTime(sol_msg);
  
  // 使用两者中的较大值，加上安全余量（50%）
  double execution_time_estimate = std::max(trajectory_duration, estimated_max_time);
  double result_timeout_seconds = execution_time_estimate * 1.5;  // 50%安全余量
  
  // 设置最小和最大超时限制（最小 ≥ Joint1 范围执行时间 30s，见 trajectory.range_execution_time）
  const double MIN_TIMEOUT = 35.0;   // 35s ≥ Joint1 全行程 30s，避免关节未到位就判超时
  const double MAX_TIMEOUT = 300.0;  // 最大300秒（5分钟，防止异常情况）
  result_timeout_seconds = std::max(MIN_TIMEOUT, std::min(MAX_TIMEOUT, result_timeout_seconds));
  
  LOG_NAMED_INFO("task_runner", "[超时计算] 轨迹duration={:.2f}s, 估算最大时间={:.2f}s, 设置超时={:.2f}s",
                 trajectory_duration, estimated_max_time, result_timeout_seconds);
  
  // 发送执行请求（使用安全的 promise/future 机制）
  moveit_task_constructor_msgs::action::ExecuteTaskSolution::Goal goal;
  goal.solution = sol_msg;
  
  // ========== 关键修复：使用 shared_ptr promise 和 shared_future，避免重复 set_value 和悬空引用 ==========
  auto result_promise = std::make_shared<std::promise<bool>>();
  std::shared_future<bool> result_sf = result_promise->get_future().share();
  auto promise_set = std::make_shared<std::atomic<bool>>(false);
  
  // 安全的 set_value 函数：只设置一次，并且捕获所有异常
  auto safe_set = [result_promise, promise_set](bool v) {
    bool expected = false;
    if (!promise_set->compare_exchange_strong(expected, true)) {
      // 已经设置过，直接返回（避免重复设置导致 future_error）
      return;
    }
    try {
      result_promise->set_value(v);
    } catch (const std::exception& e) {
      // 捕获所有异常，绝不能让异常逃出回调/线程
      LOG_NAMED_ERROR("task_runner", "safe_set 异常: {}", e.what());
    } catch (...) {
      LOG_NAMED_ERROR("task_runner", "safe_set 未知异常");
    }
  };
  
  auto send_goal_options = rclcpp_action::Client<moveit_task_constructor_msgs::action::ExecuteTaskSolution>::SendGoalOptions();
  
  // ========== 关键修复：完全异步执行，避免阻塞 executor ==========
  // 问题：在 FSM callback 中阻塞等待会占用 executor worker 线程，导致回调无法及时调度
  // 解决：executePlannedTask() 只发送 goal 立即返回，回调更新共享状态，FSM 定期检查状态
  // 创建执行状态（供 FSM 检查）
  current_exec_state_ = std::make_shared<AsyncExecState>();
  current_exec_state_->task_name = task_name;
  current_exec_state_->goal_sent = true;
  current_exec_state_->start_time = std::chrono::steady_clock::now();
  current_exec_state_->timeout = std::chrono::seconds(static_cast<int64_t>(result_timeout_seconds));
  
  // 创建等待状态（用于回调）
  current_wait_state_ = std::make_shared<ExecWaitState>();
  auto weak_self = weak_from_this();
  
  send_goal_options.goal_response_callback = [this, safe_set, weak_self](
      const rclcpp_action::ClientGoalHandle<moveit_task_constructor_msgs::action::ExecuteTaskSolution>::SharedPtr& goal_handle) {
    // 检查 TaskRunner 是否仍然有效
    if (auto self = weak_self.lock()) {
      auto use_count = self->execute_client_ ? self->execute_client_.use_count() : 0;
      LOG_NAMED_INFO("task_runner", "✓✓✓ goal_response_callback 被触发！goal_handle={}, execute_client_ use_count={}", 
                     goal_handle ? "非空" : "空", use_count);
    }
    
    if (!goal_handle) {
      LOG_NAMED_ERROR("task_runner", "执行请求被拒绝");
      safe_set(false);
      // 更新执行状态
      if (auto self = weak_self.lock()) {
        if (self->current_exec_state_) {
          self->current_exec_state_->execution_completed.store(true, std::memory_order_release);
          self->current_exec_state_->execution_succeeded.store(false, std::memory_order_release);
        }
      }
      return;
    }
    
    // 更新等待状态
    if (auto self = weak_self.lock()) {
      if (self->current_wait_state_) {
        std::lock_guard<std::mutex> lock(self->current_wait_state_->m);
        self->current_wait_state_->goal_handle = goal_handle;
        self->current_wait_state_->goal_accepted.store(true, std::memory_order_release);
      }
      
      // 更新执行状态
      if (self->current_exec_state_) {
        self->current_exec_state_->goal_accepted.store(true, std::memory_order_release);
      }
      
      auto use_count = self->execute_client_ ? self->execute_client_.use_count() : 0;
      LOG_NAMED_INFO("task_runner", "执行请求已接受（通过goal_response_callback），execute_client_ use_count={}", 
                     use_count);
      
      // 通知等待线程（虽然现在不再等待，但保留以防将来需要）
      if (self->current_wait_state_) {
        self->current_wait_state_->cv_goal.notify_all();
      }
    }
  };
  
  send_goal_options.feedback_callback = [weak_self](
      const rclcpp_action::ClientGoalHandle<moveit_task_constructor_msgs::action::ExecuteTaskSolution>::SharedPtr&,
      const std::shared_ptr<const moveit_task_constructor_msgs::action::ExecuteTaskSolution::Feedback> feedback) {
    // 反馈回调：可以用来检测执行进度
    // 目前只记录，不改变执行状态
    // 注意：使用 weak_self 保持一致性，虽然这里不访问成员变量
    (void)weak_self;  // 避免未使用变量警告
    (void)feedback;   // 避免未使用变量警告
  };
  
  send_goal_options.result_callback = [this, safe_set, weak_self](const auto& result) {
    // 检查 TaskRunner 是否仍然有效
    if (auto self = weak_self.lock()) {
      auto use_count = self->execute_client_ ? self->execute_client_.use_count() : 0;
      LOG_NAMED_INFO("task_runner", "✓✓✓ result_callback 被触发！code={}, execute_client_ use_count={}", 
                     static_cast<int>(result.code), use_count);
    }
    
    bool success = false;
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      success = result.result && 
                result.result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
      
      if (auto self = weak_self.lock()) {
        LOG_NAMED_INFO("task_runner", "任务执行{}（通过result_callback）, error_code={}", 
                       success ? "成功" : "失败", 
                       result.result ? result.result->error_code.val : -1);
      }
    } else {
      if (auto self = weak_self.lock()) {
        LOG_NAMED_ERROR("task_runner", "任务执行失败或被取消（通过result_callback）, code={}", 
                        static_cast<int>(result.code));
      }
    }
    
    // 更新等待状态
    if (auto self = weak_self.lock()) {
      if (self->current_wait_state_) {
        std::lock_guard<std::mutex> lock(self->current_wait_state_->m);
        self->current_wait_state_->execution_succeeded.store(success, std::memory_order_release);
        self->current_wait_state_->execution_completed.store(true, std::memory_order_release);
        self->current_wait_state_->cv_result.notify_all();
      }
      
      // 更新执行状态（供 FSM 检查）
      if (self->current_exec_state_) {
        self->current_exec_state_->execution_succeeded.store(success, std::memory_order_release);
        self->current_exec_state_->execution_completed.store(true, std::memory_order_release);
      }
    }
    
    safe_set(success);
  };
  
  // 发送goal
  LOG_NAMED_INFO("task_runner", "发送 goal 到 execute_task_solution action server...");
  LOG_NAMED_INFO("task_runner", "  发送前 execute_client_ use_count={}", execute_client_.use_count());
  LOG_NAMED_INFO("task_runner", "  发送前 execute_client_ 指针: {}", 
                 static_cast<void*>(execute_client_.get()));
  LOG_NAMED_INFO("task_runner", "  发送前 callback_group_fast 指针: {}", 
                 static_cast<void*>(ctx_.callback_group_fast.get()));
  
  // ========== 关键诊断：验证 action client 状态 ==========
  if (!execute_client_) {
    LOG_NAMED_ERROR("task_runner", "  ❌ execute_client_ 为空！");
    return false;
  }
  
  auto goal_handle_future = execute_client_->async_send_goal(goal, send_goal_options);
  LOG_NAMED_INFO("task_runner", "goal 已发送，异步执行中（task_name={}，超时={:.1f}秒）...", 
                 task_name, result_timeout_seconds);
  
  // ========== 关键修复：完全异步执行，立即返回 ==========
  // 问题：在 FSM callback 中阻塞等待会占用 executor worker 线程，导致回调无法及时调度
  // 解决：只发送 goal，立即返回 true（表示已启动），回调更新共享状态，FSM 定期检查状态
  // 不再等待，让 executor 能及时调度回调
  return true;  // 表示已成功启动异步执行
  
  } catch (const std::exception& e) {
    // ========== 关键修复：捕获所有异常，防止崩溃 ==========
    LOG_NAMED_ERROR("task_runner", "executePlannedTask 异常: {} (类型: {})", 
                   e.what(), typeid(e).name());
    return false;
  } catch (...) {
    // 捕获所有其他异常（包括 std::terminate 等）
    LOG_NAMED_ERROR("task_runner", "executePlannedTask 未知异常");
    return false;
  }
}

bool TaskRunner::executeTask(mtc::Task& task, const std::string& task_name) {
  LOG_NAMED_INFO("task_runner", "[executeTask] 进入，task_name={}", task_name.c_str());
  
  // 检查急停
  if (emergency_stop_) {
    LOG_NAMED_WARN("task_runner", "急停触发，停止Task执行");
    return false;
  }
  
  // 清理MoveGroup状态
  LOG_NAMED_INFO("task_runner", "[executeTask] 即将 cleanupMoveGroupState");
  cleanupMoveGroupState();
  LOG_NAMED_INFO("task_runner", "[executeTask] 即将 planTask");
  
  // 规划
  if (!planTask(task)) {
    LOG_NAMED_ERROR("task_runner", "Task规划失败: {}", task_name.c_str());
    return false;
  }
  
  // 执行（异步启动）
  if (!executePlannedTask(task, task_name)) {
    LOG_NAMED_ERROR("task_runner", "Task启动失败: {}", task_name.c_str());
    return false;
  }
  
  LOG_NAMED_INFO("task_runner", "Task已启动（异步执行）: {}", task_name.c_str());
  return true;  // 表示已启动，不等待完成
}

int TaskRunner::checkExecutionStatus() const {
  if (!current_exec_state_) {
    return 0;  // 未开始
  }
  
  if (current_exec_state_->execution_completed.load(std::memory_order_acquire)) {
    bool success = current_exec_state_->execution_succeeded.load(std::memory_order_acquire);
    return success ? 2 : 3;  // 2=成功, 3=失败
  }
  
  // 检查超时
  auto elapsed = std::chrono::steady_clock::now() - current_exec_state_->start_time;
  if (elapsed > current_exec_state_->timeout) {
    return 4;  // 超时
  }
  
  return 1;  // 执行中
}

void TaskRunner::clearExecutionState() {
  current_exec_state_.reset();
  current_wait_state_.reset();
}

std::string TaskRunner::getCurrentTaskName() const {
  if (!current_exec_state_) {
    return "";
  }
  return current_exec_state_->task_name;
}

bool TaskRunner::waitForArmStable(int timeout_ms, double threshold_rad, int stable_window) {
  if (!ctx_.joint_state_diag || !ctx_.node || !ctx_.move_group || timeout_ms <= 0 || stable_window <= 0) {
    return true;  // 无法判断时直接放行，不阻塞
  }
  const moveit::core::RobotModelConstPtr& robot_model = ctx_.move_group->getRobotModel();
  if (!robot_model) {
    return true;
  }
  const auto* arm_jmg = robot_model->getJointModelGroup(ctx_.arm_group);
  if (!arm_jmg) {
    return true;
  }
  const std::vector<std::string> arm_names = arm_jmg->getActiveJointModelNames();
  if (arm_names.empty()) {
    return true;
  }

  constexpr int sample_interval_ms = 50;
  const double max_age_sec = 2.0;
  std::vector<double> prev;
  int stable_count = 0;
  auto start = std::chrono::steady_clock::now();

  while (true) {
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count();
    if (elapsed_ms >= timeout_ms) {
      LOG_NAMED_WARN("task_runner", "waitForArmStable 超时（{} ms），未达到稳定阈值 {:.4f} rad", timeout_ms, threshold_rad);
      return false;
    }

    rclcpp::Time now = ctx_.node->get_clock()->now();
    std::vector<double> current;
    if (!fillJointPositionsFromDiag(ctx_.joint_state_diag, arm_names, current, now, max_age_sec) ||
        current.size() != arm_names.size()) {
      for (int k = 0; k < 5; ++k) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      continue;
    }

    if (prev.size() == current.size()) {
      double max_diff = 0.0;
      for (size_t i = 0; i < current.size(); ++i) {
        max_diff = std::max(max_diff, std::abs(current[i] - prev[i]));
      }
      if (max_diff < threshold_rad) {
        stable_count++;
        if (stable_count >= stable_window) {
          LOG_NAMED_INFO("task_runner", "waitForArmStable 关节已稳定（{} 次采样 max_diff < {:.4f} rad，耗时 {} ms）",
                         stable_window, threshold_rad, static_cast<int>(elapsed_ms));
          return true;
        }
      } else {
        stable_count = 0;
      }
    }
    prev = current;
    for (int k = 0; k < 5; ++k) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
}

void TaskRunner::startArmStableCheck(int timeout_ms, double threshold_rad, int stable_window,
                                    double velocity_eps_rad_s, int min_dwell_ms) {
  if (!ctx_.joint_state_diag || !ctx_.node || !ctx_.move_group || timeout_ms <= 0 || stable_window <= 0) {
    arm_stable_check_active_ = false;
    return;
  }
  const moveit::core::RobotModelConstPtr& robot_model = ctx_.move_group->getRobotModel();
  if (!robot_model) {
    arm_stable_check_active_ = false;
    return;
  }
  const auto* arm_jmg = robot_model->getJointModelGroup(ctx_.arm_group);
  if (!arm_jmg) {
    arm_stable_check_active_ = false;
    return;
  }
  arm_stable_check_active_ = true;
  arm_stable_start_ = ctx_.node->get_clock()->now();
  arm_stable_prev_joints_.clear();
  arm_stable_count_ = 0;
  arm_stable_timeout_ms_ = timeout_ms;
  arm_stable_threshold_rad_ = threshold_rad;
  arm_stable_window_ = stable_window;
  arm_stable_velocity_eps_rad_s_ = velocity_eps_rad_s;
  arm_stable_min_dwell_ms_ = min_dwell_ms;
}

std::pair<bool, bool> TaskRunner::tickArmStableCheck(const rclcpp::Time& now) {
  if (!arm_stable_check_active_ || !ctx_.joint_state_diag || !ctx_.move_group) {
    return {false, false};
  }
  const moveit::core::RobotModelConstPtr& robot_model = ctx_.move_group->getRobotModel();
  if (!robot_model) {
    arm_stable_check_active_ = false;
    return {false, false};
  }
  const auto* arm_jmg = robot_model->getJointModelGroup(ctx_.arm_group);
  if (!arm_jmg) {
    arm_stable_check_active_ = false;
    return {false, false};
  }
  const std::vector<std::string> arm_names = arm_jmg->getActiveJointModelNames();
  if (arm_names.empty()) {
    arm_stable_check_active_ = false;
    return {false, false};
  }

  const double max_age_sec = 2.0;
  std::vector<double> current;
  if (!fillJointPositionsFromDiag(ctx_.joint_state_diag, arm_names, current, now, max_age_sec) ||
      current.size() != arm_names.size()) {
    return {false, false};
  }

  // 速度判稳：若配置了 velocity_eps，需 max(|vel|) < velocity_eps
  bool velocity_ok = true;
  if (arm_stable_velocity_eps_rad_s_ > 0.0) {
    std::vector<double> vel;
    if (!fillJointVelocitiesFromDiag(ctx_.joint_state_diag, arm_names, vel, now, max_age_sec) ||
        vel.size() != arm_names.size()) {
      velocity_ok = false;
    } else {
      double max_vel = 0.0;
      for (double v : vel) {
        max_vel = std::max(max_vel, std::abs(v));
      }
      velocity_ok = (max_vel < arm_stable_velocity_eps_rad_s_);
    }
  }

  double elapsed_s = 0.0;
  try {
    elapsed_s = (now - arm_stable_start_).seconds();
  } catch (...) {
    elapsed_s = 0.0;
  }
  if (elapsed_s < 0.0) {
    elapsed_s = 0.0;
  }
  if (elapsed_s * 1000.0 >= static_cast<double>(arm_stable_timeout_ms_)) {
    arm_stable_check_active_ = false;
    LOG_NAMED_WARN("task_runner", "tickArmStableCheck 超时（{} ms），未达到稳定阈值 {:.4f} rad",
                   arm_stable_timeout_ms_, arm_stable_threshold_rad_);
    return {true, false};
  }

  if (arm_stable_prev_joints_.size() == current.size()) {
    double max_diff = 0.0;
    for (size_t i = 0; i < current.size(); ++i) {
      max_diff = std::max(max_diff, std::abs(current[i] - arm_stable_prev_joints_[i]));
    }
    bool position_ok = (max_diff < arm_stable_threshold_rad_);
    if (position_ok && velocity_ok) {
      arm_stable_count_++;
      if (arm_stable_count_ >= arm_stable_window_) {
        double min_dwell_s = (arm_stable_min_dwell_ms_ > 0)
            ? (static_cast<double>(arm_stable_min_dwell_ms_) / 1000.0) : 0.0;
        if (elapsed_s >= min_dwell_s) {
          arm_stable_check_active_ = false;
          LOG_NAMED_INFO("task_runner", "tickArmStableCheck 关节已稳定（{} 次采样 pos+vel 满足，耗时 {:.0f} ms）",
                         arm_stable_window_, elapsed_s * 1000.0);
          return {true, true};
        }
      }
    } else {
      arm_stable_count_ = 0;
    }
  }
  arm_stable_prev_joints_ = current;
  return {false, false};
}

}  // namespace m5_grasp
