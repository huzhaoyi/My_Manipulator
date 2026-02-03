#pragma once

#include <string>
#include <memory>
#include <map>
#include <mutex>
#include <vector>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <m5_msgs/msg/cable_pose_with_yaw.hpp>

namespace m5_grasp {

/**
 * @brief /joint_states 轻量诊断与缓存。
 * 数据链：机械臂 UDP JSON → M5HardwareInterface::parse_feedback_json → joint_state_broadcaster → /joint_states。
 * MTC 首 stage 已按官方教程用 CurrentState（执行端从 planning scene 取）；本结构用于诊断及
 * 其他需要「最新 /joint_states」的路径（如 IK 种子、getCurrentStateSafe 后备等）。
 */
struct JointStateDiag {
  std::mutex m;
  rclcpp::Time last_js_wall{0, 0};
  rclcpp::Time last_js_stamp{0, 0};
  uint64_t js_count{0};
  rclcpp::Time last_log_time{0, 0};
  /** 最新 /joint_states 的 name/position，供需要「实际机械臂数值」的路径使用 */
  std::vector<std::string> last_names;
  std::vector<double> last_position;
  /** 最新 /joint_states 的 velocity（与 last_names 同序），供判稳时要求速度接近 0 */
  std::vector<double> last_velocity;
};

/**
 * @brief MTC任务上下文 - 封装所有MTC任务需要的共享配置和资源
 * 
 * 设计原则：MTC模块只依赖TaskContext，不依赖Node成员
 */
struct TaskContext {
  // ========== 基本配置 ==========
  std::string arm_group{"arm_group"};
  std::string gripper_group{"gripper_group"};
  std::string eef{"gripper"};
  std::string eef_link{"LinkGG"};
  std::string planning_frame{"Link0"};
  
  // ========== 速度/加速度 ==========
  double velocity_scaling{0.5};
  double acceleration_scaling{0.5};
  double cartesian_velocity_scaling{0.3};
  double cartesian_acceleration_scaling{0.3};
  double cartesian_step_size{0.01};
  
  // ========== 规划参数 ==========
  double planning_time{10.0};
  int planning_attempts{5};
  double goal_position_tolerance{0.001};
  double goal_orientation_tolerance{0.01};
  
  // ========== 抓取参数 ==========
  double approach_offset_z{0.15};     // 预抓取高度偏移
  double descend_distance{0.15};      // 下探距离
  double lift_distance{0.10};         // 抬升距离
  double lift_joint_delta_J2{0.10};   // IK 无解时 Joint2 偏移 (rad)，4-DOF 开臂抬升
  double lift_joint_delta_J3{0.10};   // IK 无解时 Joint3 偏移 (rad)
  double grasp_yaw_add{0.0};          // yaw额外偏移
  double yaw_offset{0.0};             // yaw偏移
  bool yaw_flip{false};               // 是否翻转yaw
  double tcp_offset_x{0.0};
  double tcp_offset_y{0.0};
  double tcp_offset_z{0.0};
  
  // ========== 共享资源（由Node注入）==========
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_interface;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene;
  std::mutex* moveit_mutex{nullptr};
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor{nullptr};  // Executor引用，用于spin_until_future_complete
  rclcpp::CallbackGroup::SharedPtr callback_group_fast{nullptr};  // 快速callback group，用于action client回调

  /** 轻量 joint_states 诊断：last_js_wall/stamp、js_count、1Hz 节流，用于定位「没收到」vs「stamp 老」 */
  mutable std::shared_ptr<JointStateDiag> joint_state_diag{nullptr};
  
  /** 轨迹点稀疏化：相邻点最小关节步长 (rad)，约 10°；厂家建议步长不要太短，点要散一点减少抖动。<=0 表示不稀疏化 */
  double min_joint_step_rad{0.175};

  // ========== 碰撞对象配置 ==========
  std::string cable_name{"cable"};                // 缆绳碰撞对象名称
  double cable_diameter{0.05};                    // 缆绳直径
  double cable_length{0.02};                      // 缆绳长度（圆柱高度）
  std::vector<std::string> allow_touch_links{"LinkGG", "LinkGL", "LinkGR"};  // 允许碰撞的链接
  
  // ========== 辅助方法 ==========
  
  /**
   * @brief 检查上下文是否有效
   */
  bool isValid() const {
    return node && move_group && planning_scene && moveit_mutex;
  }
  
  /**
   * @brief 安全获取当前机器人状态
   * 
   * 绕过MoveIt的时间戳检查问题：
   * - 优先使用getCurrentJointValues()获取关节值
   * - 手动构造RobotState并调用enforceBounds()
   * - 避免"Didn't receive robot state with recent timestamp"错误
   * 
   * @return 机器人状态指针，失败返回nullptr
   */
  moveit::core::RobotStatePtr getCurrentStateSafe() const;
};

/**
 * @brief 在 getCurrentState 前按 1Hz 节流打印：now - last_js_wall、now - last_js_stamp、js_count。
 * 用于区分「没收到」（last_js_wall 老）vs「收到但 stamp 老」（last_js_stamp 老）。
 * 若 ctx.joint_state_diag 为空则无操作。
 */
void logStateFreshnessIfNeeded(const TaskContext& ctx);

/**
 * @brief 从 JointStateDiag 的 last_names/last_position 按 joint_names 顺序填满 out。
 * 供 getCurrentStateSafe、cleanupMoveGroupState、computeFromMessage 等需要「实际机械臂数值」后备时使用。
 * 仅当缓存“新鲜”时才返回 true：now−last_js_stamp 或 now−last_js_wall 必须小于 max_age_sec。
 * @return 是否全部关节均从 diag 中找到并填入 out（且 out.size() == joint_names.size()）
 */
bool fillJointPositionsFromDiag(
  const std::shared_ptr<JointStateDiag>& diag,
  const std::vector<std::string>& joint_names,
  std::vector<double>& out,
  const rclcpp::Time& now,
  double max_age_sec);

/**
 * @brief 从 JointStateDiag 的 last_names/last_velocity 按 joint_names 顺序填满 out。
 * 仅当 last_velocity 与 last_names 同长且缓存新鲜时返回 true。
 * @return 是否全部关节均从 diag 中找到并填入 out（且 out.size() == joint_names.size()）
 */
bool fillJointVelocitiesFromDiag(
  const std::shared_ptr<JointStateDiag>& diag,
  const std::vector<std::string>& joint_names,
  std::vector<double>& out,
  const rclcpp::Time& now,
  double max_age_sec);

/**
 * @brief 任务目标 - 封装单次抓取任务的目标信息
 */
struct TaskTarget {
  // 原始目标
  m5_msgs::msg::CablePoseWithYaw cable_pose_msg;
  
  // 计算后的目标
  geometry_msgs::msg::PoseStamped cable_pose;       // 缆绳位置（规划坐标系）
  double cable_yaw{0.0};                            // 缆绳yaw角
  double joint4_target{0.0};                        // Joint4目标值
  geometry_msgs::msg::PoseStamped pregrasp_pose;    // 预抓取位姿（LinkGG）
  geometry_msgs::msg::PoseStamped grasp_pose;       // 抓取位姿（LinkGG）
  std::map<std::string, double> pregrasp_joint_map; // 预抓取关节值
  std::map<std::string, double> grasp_joint_map;    // 抓取位姿对应关节值（下探用关节目标时使用）
  bool use_joint_goal{false};                       // 是否使用关节目标
  bool valid{false};                                // 目标是否有效
  
  /**
   * @brief 从CablePoseWithYaw计算任务目标
   * @param ctx 任务上下文
   * @param msg 原始消息
   * @return 是否计算成功
   */
  bool computeFromMessage(const TaskContext& ctx, const m5_msgs::msg::CablePoseWithYaw& msg);
};

}  // namespace m5_grasp
