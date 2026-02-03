#include "m5_grasp/mtc/task_context.hpp"
#include "m5_grasp/logging/logger.hpp"

#include <algorithm>
#include <limits>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/robot_state/robot_state.h>

namespace m5_grasp {

void logStateFreshnessIfNeeded(const TaskContext& ctx) {
  if (!ctx.joint_state_diag || !ctx.node) return;
  auto now = ctx.node->get_clock()->now();
  auto& d = *ctx.joint_state_diag;
  std::lock_guard<std::mutex> lock(d.m);
  if (d.last_log_time.nanoseconds() != 0 &&
      (now - d.last_log_time).seconds() < 1.0)
    return;
  double dt_wall_sec = -1.0;
  double dt_stamp_sec = -1.0;
  if (d.last_js_wall.nanoseconds() != 0)
    dt_wall_sec = (now - d.last_js_wall).seconds();
  if (d.last_js_stamp.nanoseconds() != 0) {
    try {
      dt_stamp_sec = (now - d.last_js_stamp).seconds();
    } catch (...) {
      // 不同时钟类型不能相减，仅用于日志时忽略
    }
  }
  LOG_NAMED_INFO("state_diag",
      "now−last_js_wall={:.2f}s now−last_js_stamp={:.2f}s js_count={}",
      dt_wall_sec, dt_stamp_sec, d.js_count);
  d.last_log_time = now;
}

bool fillJointPositionsFromDiag(
    const std::shared_ptr<JointStateDiag>& diag,
    const std::vector<std::string>& joint_names,
    std::vector<double>& out,
    const rclcpp::Time& now,
    double max_age_sec) {
  out.clear();
  if (!diag || joint_names.empty()) return false;
  std::lock_guard<std::mutex> lock(diag->m);

  // 1) 必须至少收到过一帧 joint_states
  if (diag->js_count == 0) {
    return false;
  }

  // 2) 新鲜度检查：仅用与 now 同源的 last_js_wall，避免 "can't subtract times with different time sources"
  // （last_js_stamp 来自 msg->header，可能与 node 的 clock 类型不同，如 ROS vs Steady）
  double age = std::numeric_limits<double>::infinity();
  if (diag->last_js_wall.nanoseconds() != 0) {
    age = (now - diag->last_js_wall).seconds();
  }
  if (!(age < max_age_sec)) {
    // 数据太旧，不作为当前状态使用
    return false;
  }

  const auto& names = diag->last_names;
  const auto& pos = diag->last_position;
  if (names.size() != pos.size()) return false;
  for (const auto& jname : joint_names) {
    auto it = std::find(names.begin(), names.end(), jname);
    if (it == names.end()) return false;
    out.push_back(pos[static_cast<size_t>(it - names.begin())]);
  }
  return out.size() == joint_names.size();
}

bool fillJointVelocitiesFromDiag(
    const std::shared_ptr<JointStateDiag>& diag,
    const std::vector<std::string>& joint_names,
    std::vector<double>& out,
    const rclcpp::Time& now,
    double max_age_sec) {
  out.clear();
  if (!diag || joint_names.empty()) return false;
  std::lock_guard<std::mutex> lock(diag->m);

  if (diag->js_count == 0) return false;

  double age = std::numeric_limits<double>::infinity();
  if (diag->last_js_wall.nanoseconds() != 0) {
    age = (now - diag->last_js_wall).seconds();
  }
  if (!(age < max_age_sec)) return false;

  const auto& names = diag->last_names;
  const auto& vel = diag->last_velocity;
  if (names.size() != vel.size()) return false;
  for (const auto& jname : joint_names) {
    auto it = std::find(names.begin(), names.end(), jname);
    if (it == names.end()) return false;
    out.push_back(vel[static_cast<size_t>(it - names.begin())]);
  }
  return out.size() == joint_names.size();
}

namespace {

/**
 * @brief 计算向下的固定姿态四元数（绕X轴旋转180度）
 */
geometry_msgs::msg::Quaternion computeDownwardOrientation() {
  tf2::Quaternion q;
  q.setRPY(M_PI, 0.0, 0.0);  // Roll=180°, Pitch=0°, Yaw=0°
  geometry_msgs::msg::Quaternion quat_msg;
  quat_msg.x = q.x();
  quat_msg.y = q.y();
  quat_msg.z = q.z();
  quat_msg.w = q.w();
  return quat_msg;
}

/**
 * @brief 角度归一化到 [-pi, pi]
 */
double normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2 * M_PI;
  while (angle < -M_PI) angle += 2 * M_PI;
  return angle;
}

/**
 * @brief 检查Joint2/3是否在可达范围内
 * 
 * M5机械臂的Joint2/3只能向下弯（限位 [-2.97, 0] rad，即 [-170°, 0°]）
 * 如果IK解需要"向上折"（正值），则不可达
 * 
 * @param state 机器人状态
 * @param jmg 关节模型组
 * @param joint2_value 输出Joint2值
 * @param joint3_value 输出Joint3值
 * @return true 如果Joint2/3在可达范围内
 */
bool checkJoint23Reachable(const moveit::core::RobotState& state,
                           const moveit::core::JointModelGroup* jmg,
                           double& joint2_value, double& joint3_value) {
  if (!jmg) return false;
  
  const auto& joint_names = jmg->getActiveJointModelNames();
  std::vector<double> joint_values;
  state.copyJointGroupPositions(jmg, joint_values);
  
  // 查找Joint2和Joint3的索引
  int joint2_idx = -1, joint3_idx = -1;
  for (size_t i = 0; i < joint_names.size(); ++i) {
    if (joint_names[i] == "Joint2") joint2_idx = static_cast<int>(i);
    else if (joint_names[i] == "Joint3") joint3_idx = static_cast<int>(i);
  }
  
  if (joint2_idx < 0 || joint3_idx < 0) {
    LOG_NAMED_WARN("task_target", "无法找到Joint2或Joint3");
    return false;
  }
  
  joint2_value = joint_values[joint2_idx];
  joint3_value = joint_values[joint3_idx];
  
  // 检查是否在可达范围内 [-2.97, 0] rad；下限略放宽以容纳 IK 数值误差（如 -170.2° ≈ -2.9704 rad）
  const double JOINT_LOWER_LIMIT = -2.99;  // -171.2°，比 -2.97 略松
  const double JOINT_UPPER_LIMIT = 0.05;   // 略微放宽到 +3°，允许小误差
  
  bool joint2_ok = (joint2_value >= JOINT_LOWER_LIMIT && joint2_value <= JOINT_UPPER_LIMIT);
  bool joint3_ok = (joint3_value >= JOINT_LOWER_LIMIT && joint3_value <= JOINT_UPPER_LIMIT);
  
  return joint2_ok && joint3_ok;
}

/**
 * @brief 尝试多个yaw候选进行IK求解
 * 
 * 缆绳两端物理等价，yaw和yaw+π都可以抓取
 * 尝试原始yaw和翻转yaw，选择Joint2/3可达的解
 * 
 * @param pose 目标位姿
 * @param yaw_candidates yaw候选列表
 * @param robot_state 用于IK求解的机器人状态
 * @param jmg 关节模型组
 * @param eef_link 末端执行器链接
 * @param out_joint_values 输出关节值
 * @param out_selected_yaw 输出选定的yaw
 * @return true 如果找到可达解
 */
bool tryYawCandidatesForIK(const geometry_msgs::msg::Pose& pose,
                           const std::vector<double>& yaw_candidates,
                           moveit::core::RobotState& robot_state,
                           const moveit::core::JointModelGroup* jmg,
                           const std::string& eef_link,
                           std::vector<double>& out_joint_values,
                           double& out_selected_yaw) {
  if (!jmg || yaw_candidates.empty()) return false;
  
  LOG_NAMED_INFO("task_target", "[Yaw候选IK] 开始尝试 {} 个yaw候选", yaw_candidates.size());
  
  for (size_t i = 0; i < yaw_candidates.size(); ++i) {
    double candidate_yaw = yaw_candidates[i];
    LOG_NAMED_INFO("task_target", "[Yaw候选IK] 尝试候选#{}: yaw={:.1f}°",
                   i + 1, candidate_yaw * 180.0 / M_PI);
    
    // 构造目标姿态（保持位置，使用候选yaw）
    geometry_msgs::msg::Pose pose_for_ik = pose;
    // 注意：对于4DOF机械臂，我们使用固定向下姿态，yaw由Joint4控制
    // 所以这里不需要修改orientation
    
    // 尝试IK求解（多次尝试不同seed）
    bool ik_solved = false;
    for (int attempt = 0; attempt < 3; ++attempt) {
      if (attempt > 0) {
        robot_state.setToRandomPositions(jmg);
      }
      
      // 增加IK超时时间到1.0秒，确保有足够时间找到平稳的解
      if (robot_state.setFromIK(jmg, pose_for_ik, eef_link, 1.0)) {
        ik_solved = true;
        break;
      }
    }
    
    if (!ik_solved) {
      LOG_NAMED_DEBUG("task_target", "[Yaw候选IK] 候选#{} IK求解失败", i + 1);
      continue;
    }
    
    // IK成功，检查Joint2/3可达性
    double joint2_val, joint3_val;
    if (checkJoint23Reachable(robot_state, jmg, joint2_val, joint3_val)) {
      LOG_NAMED_INFO("task_target", "[Yaw候选IK] ✓ 候选#{} 可达！Joint2={:.1f}°, Joint3={:.1f}°",
                     i + 1, joint2_val * 180.0 / M_PI, joint3_val * 180.0 / M_PI);
      
      robot_state.copyJointGroupPositions(jmg, out_joint_values);
      out_selected_yaw = candidate_yaw;
      return true;
    } else {
      LOG_NAMED_WARN("task_target", "[Yaw候选IK] 候选#{} IK可解但Joint2/3不可达: Joint2={:.1f}°, Joint3={:.1f}°",
                     i + 1, joint2_val * 180.0 / M_PI, joint3_val * 180.0 / M_PI);
    }
  }
  
  LOG_NAMED_WARN("task_target", "[Yaw候选IK] 所有候选都失败");
  return false;
}

}  // namespace

bool TaskTarget::computeFromMessage(const TaskContext& ctx, const m5_msgs::msg::CablePoseWithYaw& msg) {
  valid = false;
  cable_pose_msg = msg;
  
  // 1. 计算Joint4目标值：yaw 等同于 joint4（发 90 即 90），仅加可选微调 offset/flip
  joint4_target = msg.yaw + ctx.yaw_offset;
  if (ctx.yaw_flip) {
    joint4_target += M_PI;
  }
  joint4_target = normalizeAngle(joint4_target);
  
  LOG_NAMED_INFO("task_target", "Joint4目标计算: yaw={:.1f}° + offset={:.1f}° {} = {:.1f}°",
               msg.yaw * 180.0 / M_PI,
               ctx.yaw_offset * 180.0 / M_PI,
               ctx.yaw_flip ? "+ 180°" : "",
               joint4_target * 180.0 / M_PI);
  
  // 2. 保存原始yaw
  cable_yaw = msg.yaw;
  
  // 3. 构造缆绳位姿（夹爪中心目标）
  cable_pose.header = msg.header;
  cable_pose.pose.position = msg.position;
  cable_pose.pose.orientation = computeDownwardOrientation();
  
  if (cable_pose.header.stamp.sec == 0 && cable_pose.header.stamp.nanosec == 0) {
    cable_pose.header.stamp = ctx.node->now();
  }
  if (cable_pose.header.frame_id.empty()) {
    cable_pose.header.frame_id = ctx.planning_frame;
  }
  
  // 4. 计算预抓取位姿（在目标位置上方）
  pregrasp_pose = cable_pose;
  pregrasp_pose.pose.position.z += ctx.approach_offset_z;
  
  // 应用TCP偏移补偿
  pregrasp_pose.pose.position.x -= ctx.tcp_offset_x;
  pregrasp_pose.pose.position.y -= ctx.tcp_offset_y;
  pregrasp_pose.pose.position.z -= ctx.tcp_offset_z;
  
  // 5. 计算抓取位姿（缆绳位置）
  grasp_pose = cable_pose;
  grasp_pose.pose.position.x -= ctx.tcp_offset_x;
  grasp_pose.pose.position.y -= ctx.tcp_offset_y;
  grasp_pose.pose.position.z -= ctx.tcp_offset_z;
  
  LOG_NAMED_INFO("task_target", "目标计算完成:");
  LOG_NAMED_INFO("task_target", "  缆绳位置: ({:.3f}, {:.3f}, {:.3f})",
               cable_pose.pose.position.x, cable_pose.pose.position.y, cable_pose.pose.position.z);
  LOG_NAMED_INFO("task_target", "  预抓取位置: ({:.3f}, {:.3f}, {:.3f})",
               pregrasp_pose.pose.position.x, pregrasp_pose.pose.position.y, pregrasp_pose.pose.position.z);
  LOG_NAMED_INFO("task_target", "  Joint4目标: {:.1f}°", joint4_target * 180.0 / M_PI);
  
  // 6. 自动计算Joint1方位角（参考上一版本的控制策略）
  // Joint1控制机械臂朝向，必须根据目标位置的方位角计算
  double joint1_auto = std::atan2(cable_pose.pose.position.y, cable_pose.pose.position.x);
  LOG_NAMED_INFO("task_target", "Joint1自动计算: 目标({:.3f}, {:.3f}) -> Joint1={:.1f}°",
               cable_pose.pose.position.x, cable_pose.pose.position.y,
               joint1_auto * 180.0 / M_PI);
  
  // 7. 使用IK计算预抓取关节值（4DOF机械臂需要关节目标）
  if (ctx.move_group && ctx.moveit_mutex) {
    try {
      std::lock_guard<std::mutex> lock(*ctx.moveit_mutex);
      
      // 获取当前机器人状态
      const auto& robot_model = ctx.move_group->getRobotModel();
      if (robot_model) {
        auto robot_state = std::make_shared<moveit::core::RobotState>(robot_model);
        robot_state->setToDefaultValues();
        
        const auto* arm_jmg = robot_model->getJointModelGroup(ctx.arm_group);
        if (arm_jmg) {
          // 先设置当前关节值作为种子（IK 与全零检测）
          const auto& arm_names = arm_jmg->getActiveJointModelNames();
          std::vector<double> current_joints;
          bool seed_ok = false;
          rclcpp::Time now = ctx.node ? ctx.node->get_clock()->now() : rclcpp::Time(0, 0);
          if (ctx.joint_state_diag) {
            seed_ok = fillJointPositionsFromDiag(ctx.joint_state_diag, arm_names, current_joints, now, 0.5);
            if (seed_ok) {
              LOG_NAMED_INFO("task_target", "[预抓取IK] 使用 joint_state_diag 作为关节种子");
            }
          }
          if (!seed_ok) {
            // 注意：此处 getCurrentJointValues 可能在 MoveIt 未完全 ready 时抛异常
            current_joints = ctx.move_group->getCurrentJointValues();
            if (current_joints.size() != arm_names.size() && ctx.joint_state_diag) {
              if (fillJointPositionsFromDiag(ctx.joint_state_diag, arm_names, current_joints, now, 0.5)) {
                LOG_NAMED_INFO(
                    "task_target",
                    "[预抓取IK] getCurrentJointValues 不可用，回退 joint_state_diag 作为关节种子");
              }
            }
          }
          
          // 检测是否处于全零状态（奇异点附近）
          bool is_near_zero_state = false;
          const double NEAR_ZERO_THRESHOLD = 0.1;  // rad
          if (current_joints.size() >= 4) {
            is_near_zero_state = true;
            for (size_t i = 0; i < 4; ++i) {
              if (std::abs(current_joints[i]) > NEAR_ZERO_THRESHOLD) {
                is_near_zero_state = false;
                break;
              }
            }
            if (is_near_zero_state) {
              LOG_NAMED_WARN("task_target", "检测到全零状态（奇异点附近）；若IK解出将使用关节目标以避免OMPL位姿采样失败");
            }
          }
          
          if (current_joints.size() == arm_names.size()) {
            robot_state->setJointGroupPositions(arm_jmg, current_joints);
          }
          
          // 准备yaw候选：原始yaw和yaw+π（缆绳两端物理等价）
          std::vector<double> yaw_candidates;
          yaw_candidates.push_back(cable_yaw);
          double yaw_flipped = normalizeAngle(cable_yaw + M_PI);
          yaw_candidates.push_back(yaw_flipped);
          
          LOG_NAMED_INFO("task_target", "[预抓取IK] 尝试yaw候选: 原始yaw={:.1f}°, 翻转yaw={:.1f}°",
                         cable_yaw * 180.0 / M_PI, yaw_flipped * 180.0 / M_PI);
          
          // 尝试yaw候选进行IK求解（检查Joint2/3可达性）
          std::vector<double> ik_joint_values;
          double selected_yaw = cable_yaw;
          bool ik_found = tryYawCandidatesForIK(
              pregrasp_pose.pose, yaw_candidates, *robot_state, arm_jmg, 
              ctx.eef_link, ik_joint_values, selected_yaw);
          
          if (ik_found) {
            // IK 成功则始终使用关节目标，避免 OMPL 对位姿目标采不到有效 goal（Unable to sample any valid states for goal tree）
            const auto& joint_names = arm_jmg->getActiveJointModelNames();
            pregrasp_joint_map.clear();
            for (size_t i = 0; i < joint_names.size() && i < ik_joint_values.size(); ++i) {
              pregrasp_joint_map[joint_names[i]] = ik_joint_values[i];
            }
            
            // 关键：用自动计算的Joint1覆盖IK解（确保机械臂朝向正确）
            if (pregrasp_joint_map.find("Joint1") != pregrasp_joint_map.end()) {
              double ik_joint1 = pregrasp_joint_map["Joint1"];
              LOG_NAMED_INFO("task_target", "Joint1: IK解={:.1f}° -> 使用自动计算值={:.1f}°",
                           ik_joint1 * 180.0 / M_PI, joint1_auto * 180.0 / M_PI);
              pregrasp_joint_map["Joint1"] = joint1_auto;
            }
            
            // 关键：用目标Joint4覆盖IK解（控制夹爪yaw旋转）
            if (pregrasp_joint_map.find("Joint4") != pregrasp_joint_map.end()) {
              double ik_joint4 = pregrasp_joint_map["Joint4"];
              LOG_NAMED_INFO("task_target", "Joint4: IK解={:.1f}° -> 使用目标值={:.1f}°",
                           ik_joint4 * 180.0 / M_PI, joint4_target * 180.0 / M_PI);
              pregrasp_joint_map["Joint4"] = joint4_target;
            }
            
            // 检查Joint2/3是否在合理范围
            double joint2_val = pregrasp_joint_map.count("Joint2") ? pregrasp_joint_map["Joint2"] : 0.0;
            double joint3_val = pregrasp_joint_map.count("Joint3") ? pregrasp_joint_map["Joint3"] : 0.0;
            LOG_NAMED_INFO("task_target", "最终关节目标: Joint2={:.1f}°, Joint3={:.1f}°",
                          joint2_val * 180.0 / M_PI, joint3_val * 180.0 / M_PI);
            
            use_joint_goal = true;
            if (is_near_zero_state) {
              LOG_NAMED_INFO("task_target", "全零状态，但因IK已解出仍使用关节目标模式（避免OMPL位姿采样失败）");
            } else {
              LOG_NAMED_INFO("task_target", "IK计算成功（使用yaw={:.1f}°），使用关节目标模式",
                            selected_yaw * 180.0 / M_PI);
            }
            for (const auto& [name, value] : pregrasp_joint_map) {
              LOG_NAMED_INFO("task_target", "  {}: {:.1f}°", name, value * 180.0 / M_PI);
            }
          } else {
            // IK 失败才退回到位姿目标
            if (is_near_zero_state) {
              LOG_NAMED_INFO("task_target", "全零状态且无有效IK，使用位姿目标模式");
            } else {
              LOG_NAMED_WARN("task_target", "所有yaw候选IK都失败，将使用位姿目标模式（可能因为目标超出工作空间）");
            }
            use_joint_goal = false;
          }
          
          // 8. 对 grasp_pose 做 IK，供下探使用关节目标（避免 OMPL 对 pose 采不到有效 goal）
          grasp_joint_map.clear();
          if (!pregrasp_joint_map.empty()) {
            std::vector<double> seed;
            const auto& jnames = arm_jmg->getActiveJointModelNames();
            for (const auto& name : jnames) {
              auto it = pregrasp_joint_map.find(name);
              if (it == pregrasp_joint_map.end()) break;
              seed.push_back(it->second);
            }
            if (seed.size() == jnames.size()) {
              robot_state->setJointGroupPositions(arm_jmg, seed);
            }
          }
          std::vector<double> grasp_ik_values;
          double grasp_sel_yaw = cable_yaw;
          bool grasp_ik_ok = tryYawCandidatesForIK(
              grasp_pose.pose, yaw_candidates, *robot_state, arm_jmg,
              ctx.eef_link, grasp_ik_values, grasp_sel_yaw);
          if (grasp_ik_ok) {
            const auto& jnames = arm_jmg->getActiveJointModelNames();
            for (size_t i = 0; i < jnames.size() && i < grasp_ik_values.size(); ++i) {
              grasp_joint_map[jnames[i]] = grasp_ik_values[i];
            }
            if (grasp_joint_map.count("Joint1")) grasp_joint_map["Joint1"] = joint1_auto;
            if (grasp_joint_map.count("Joint4")) grasp_joint_map["Joint4"] = joint4_target;
            LOG_NAMED_INFO("task_target", "[下探IK] grasp_pose 关节解成功，将用关节目标下探");
          }
        }
      }
      ctx.move_group->clearPoseTargets();
    } catch (const std::exception& e) {
      // 关键：防止未捕获异常导致整个节点 std::terminate / SIGABRT
      LOG_NAMED_ERROR("task_target",
                      "computeFromMessage 预抓取IK阶段异常: {}，将退回位姿目标模式（use_joint_goal=false）",
                      e.what());
      use_joint_goal = false;
      pregrasp_joint_map.clear();
      grasp_joint_map.clear();
      try {
        ctx.move_group->clearPoseTargets();
      } catch (...) {
        // 忽略清理中的异常
      }
    } catch (...) {
      LOG_NAMED_ERROR("task_target",
                      "computeFromMessage 预抓取IK阶段发生未知异常，将退回位姿目标模式（use_joint_goal=false）");
      use_joint_goal = false;
      pregrasp_joint_map.clear();
      grasp_joint_map.clear();
      try {
        ctx.move_group->clearPoseTargets();
      } catch (...) {
        // 忽略清理中的异常
      }
    }
  }
  
  valid = true;
  return true;
}

moveit::core::RobotStatePtr TaskContext::getCurrentStateSafe() const {
  logStateFreshnessIfNeeded(*this);
  if (!move_group || !moveit_mutex) {
    LOG_NAMED_WARN("task_context", "getCurrentStateSafe: 上下文无效");
    return nullptr;
  }
  
  std::lock_guard<std::mutex> lock(*moveit_mutex);
  
  // 修复时间戳/等待问题：优先用 /joint_states 缓存（joint_state_diag）作为“当前关节值来源”。
  // 原因：MoveGroupInterface::getCurrentJointValues()/getCurrentState() 内部会等待 CurrentStateMonitor 的“recent”
  // joint_states，若短时间内没收到（DDS发现/回调阻塞/偶发掉帧）会报
  // "Didn't receive robot state... latest received state has time 0"。
  // 而本项目已有同进程的 best_effort/volatile /joint_states 轻量订阅（joint_state_diag），更直接可靠。
  try {
    const auto& robot_model = move_group->getRobotModel();
    if (!robot_model) {
      LOG_NAMED_WARN("task_context", "getCurrentStateSafe: 无法获取机器人模型");
      return nullptr;
    }
    
    auto state = std::make_shared<moveit::core::RobotState>(robot_model);
    state->setToDefaultValues();
    
    // arm_group：优先用 joint_state_diag；失败再退回 MoveIt 的 getCurrentJointValues()
    const auto* arm_jmg = robot_model->getJointModelGroup(arm_group);
    rclcpp::Time now = node ? node->get_clock()->now() : rclcpp::Time(0, 0);
    if (arm_jmg) {
      const auto& arm_names = arm_jmg->getActiveJointModelNames();
      std::vector<double> arm_joint_values;
      bool ok_from_diag = false;
      if (joint_state_diag) {
        ok_from_diag = fillJointPositionsFromDiag(joint_state_diag, arm_names, arm_joint_values, now, 0.5);
        if (ok_from_diag) {
          LOG_NAMED_INFO("task_context", "getCurrentStateSafe: arm 使用 joint_state_diag");
        }
      }
      if (!ok_from_diag) {
        arm_joint_values = move_group->getCurrentJointValues();
      }
      if (arm_joint_values.size() == arm_names.size()) {
        state->setJointGroupPositions(arm_jmg, arm_joint_values);
      } else {
        LOG_NAMED_WARN(
            "task_context",
            "getCurrentStateSafe: arm 关节数量不匹配 (got={}, expected={})",
            arm_joint_values.size(), arm_names.size());
      }
    }
    
    // gripper_group：优先用 joint_state_diag；失败再退回 MoveIt 的 getCurrentJointValues()
    const auto* gripper_jmg = robot_model->getJointModelGroup(gripper_group);
    if (gripper_jmg && gripper_group_interface) {
      const auto& gripper_names = gripper_jmg->getActiveJointModelNames();
      std::vector<double> gripper_joint_values;
      bool ok_from_diag = false;
      if (joint_state_diag) {
        ok_from_diag =
            fillJointPositionsFromDiag(joint_state_diag, gripper_names, gripper_joint_values, now, 0.5);
        if (ok_from_diag) {
          LOG_NAMED_INFO("task_context", "getCurrentStateSafe: gripper 使用 joint_state_diag");
        }
      }
      if (!ok_from_diag) {
        gripper_joint_values = gripper_group_interface->getCurrentJointValues();
      }
      if (gripper_joint_values.size() == gripper_names.size()) {
        state->setJointGroupPositions(gripper_jmg, gripper_joint_values);
      }
    }
    
    // 更新状态（计算FK等）
    state->update();
    // 修复Invalid Start State：确保关节值在合法范围内，避免浮点误差导致状态被判为invalid
    state->enforceBounds();
    return state;
  } catch (const std::exception& e) {
    LOG_NAMED_WARN("task_context", "getCurrentJointValues()失败: {}，尝试使用getCurrentState()", e.what());
  }
  
  // Fallback：如果getCurrentJointValues()失败，尝试使用getCurrentState()（带超时）
  try {
    auto state = move_group->getCurrentState(3.0);
    if (state) {
      state->enforceBounds();
      return state;
    }
  } catch (const std::exception& e) {
    LOG_NAMED_WARN("task_context", "getCurrentState()也失败: {}", e.what());
  }
  
  // 后备：从 joint_state_diag（/joint_states 缓存）构造状态，与「实际机械臂数值」来源一致
  if (joint_state_diag) {
    const auto& robot_model = move_group->getRobotModel();
    if (robot_model) {
      auto state = std::make_shared<moveit::core::RobotState>(robot_model);
      state->setToDefaultValues();
      bool arm_filled = false;
      const auto* arm_jmg = robot_model->getJointModelGroup(arm_group);
      rclcpp::Time now = node ? node->get_clock()->now() : rclcpp::Time(0, 0);
      if (arm_jmg) {
        std::vector<double> arm_vals;
        if (fillJointPositionsFromDiag(joint_state_diag, arm_jmg->getActiveJointModelNames(), arm_vals, now, 0.5)) {
          state->setJointGroupPositions(arm_jmg, arm_vals);
          arm_filled = true;
        }
      }
      const auto* gripper_jmg = robot_model->getJointModelGroup(gripper_group);
      if (gripper_jmg && gripper_group_interface) {
        std::vector<double> grip_vals;
        if (fillJointPositionsFromDiag(joint_state_diag, gripper_jmg->getActiveJointModelNames(), grip_vals, now, 0.5)) {
          state->setJointGroupPositions(gripper_jmg, grip_vals);
        }
      }
      if (arm_filled) {
        state->update();
        state->enforceBounds();
        LOG_NAMED_INFO("task_context", "getCurrentStateSafe: 使用 joint_state_diag 后备");
        return state;
      }
    }
  }
  
  // 最后的后备：创建默认状态
  try {
    const auto& robot_model = move_group->getRobotModel();
    if (robot_model) {
      auto default_state = std::make_shared<moveit::core::RobotState>(robot_model);
      default_state->setToDefaultValues();
      default_state->enforceBounds();
      return default_state;
    }
  } catch (...) {
    // 忽略
  }
  
  return nullptr;
}

}  // namespace m5_grasp
