#pragma once

#include <map>
#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>

#include "m5_grasp/mtc/task_context.hpp"

namespace mtc = moveit::task_constructor;

namespace m5_grasp {

/**
 * @brief MTC任务工厂 - 负责构建各种MTC任务
 * 
 * 职责：
 * - 构建预抓取任务（Pregrasp）
 * - 构建对齐任务（Align）
 * - 构建下探任务（Descend）
 * - 构建抬升任务（Lift）
 * 
 * 设计原则：
 * - 只负责构建任务，不负责执行
 * - 每个函数返回一个独立的mtc::Task
 * - 函数内不应超过50行
 */
class TaskFactory {
public:
  /**
   * @brief 构造函数
   * @param ctx 任务上下文
   */
  explicit TaskFactory(const TaskContext& ctx);

  /**
   * @brief 构建预抓取任务（从当前位置移动到预抓取位置）
   * @param target 任务目标
   * @return MTC任务
   */
  mtc::Task makePregrasp(const TaskTarget& target);

  /**
   * @brief 构建对齐任务（设置Joint4到目标值）
   * @param target 任务目标
   * @return MTC任务
   */
  mtc::Task makeAlign(const TaskTarget& target);

  /**
   * @brief 构建下探任务（移到 grasp_pose，用 OMPL，不用笛卡尔，与上一版本一致）
   * @param target 任务目标（含 grasp_pose）
   * @return MTC任务
   */
  mtc::Task makeDescend(const TaskTarget& target);

  /**
   * @brief 构建抬升任务（4-DOF 仅用关节/IK，不用笛卡尔；IK 无解时用关节空间 J2/J3 偏移抬升）
   * @param distance 抬升距离（默认使用 ctx 中的 lift_distance）
   * @param joint4_fix_rad 若给定，抬升时固定 Joint4（yaw）为此值，与 align/descend 一致
   * @param start_joints 若非空，以该关节为起始（如 grasp_joint_map）计算抬升目标并设 FixedState，避免 joint_state_diag 滞后导致无运动
   * @return MTC任务
   */
  mtc::Task makeLift(double distance = -1.0,
                     std::optional<double> joint4_fix_rad = std::nullopt,
                     const std::map<std::string, double>* start_joints = nullptr);

private:
  /**
   * @brief 添加当前状态 stage（CurrentState，从 planning scene 取）
   * @param task 要添加 stage 的任务
   */
  void addCurrentStateStage(mtc::Task& task);

  /**
   * @brief 为笛卡尔类任务（descend/lift）添加起始状态：若 joint_state_diag 可用则用 FixedState 注入，避免 MTC 在 planning scene 上阻塞；否则退化为 CurrentState。
   * @param task 要添加 stage 的任务
   */
  void addStartStateStageForCartesian(mtc::Task& task);

  /**
   * @brief 创建OMPL规划器（缓存，避免重复创建）
   */
  std::shared_ptr<mtc::solvers::PipelinePlanner> createOMPLPlanner();

  /**
   * @brief 创建笛卡尔路径规划器（缓存，避免重复创建）
   */
  std::shared_ptr<mtc::solvers::CartesianPath> createCartesianPlanner();

  // 缓存的规划器（避免重复创建，可能导致重复初始化）
  std::shared_ptr<mtc::solvers::PipelinePlanner> cached_ompl_planner_;
  std::shared_ptr<mtc::solvers::CartesianPath> cached_cartesian_planner_;
  /** 关节插补规划器，用于预抓取关节目标，不依赖 node/PlanningPipeline，避免 "Publisher already registered" / SIGABRT */
  std::shared_ptr<mtc::solvers::JointInterpolationPlanner> cached_joint_planner_;
  /** 专用于 MTC PipelinePlanner::create 的 node，命名带唯一后缀，避免与主节点同名导致 "Publisher already registered" */
  rclcpp::Node::SharedPtr mtc_planning_node_;

  /**
   * @brief 创建MoveTo位姿stage
   * @param name stage名称
   * @param pose 目标位姿
   * @param planner 规划器
   */
  std::unique_ptr<mtc::stages::MoveTo> makeMoveToPose(
    const std::string& name,
    const geometry_msgs::msg::PoseStamped& pose,
    const std::shared_ptr<mtc::solvers::PlannerInterface>& planner);

  /**
   * @brief 创建MoveTo关节目标stage
   * @param name stage名称
   * @param joint_map 关节目标
   * @param planner 规划器
   */
  std::unique_ptr<mtc::stages::MoveTo> makeMoveToJoints(
    const std::string& name,
    const std::map<std::string, double>& joint_map,
    const std::shared_ptr<mtc::solvers::PlannerInterface>& planner);

  /**
   * @brief 创建MoveRelative stage（沿工具系Z轴）
   * @param name stage名称
   * @param distance Z轴距离（正数向上，负数向下）
   * @param planner 规划器
   */
  std::unique_ptr<mtc::stages::MoveRelative> makeMoveRelativeZ(
    const std::string& name,
    double distance,
    const std::shared_ptr<mtc::solvers::CartesianPath>& planner);

  /**
   * @brief 创建添加碰撞对象的stage
   * @param target 任务目标（包含缆绳位姿）
   * @return ModifyPlanningScene stage
   */
  std::unique_ptr<mtc::stages::ModifyPlanningScene> makeAddCollisionObject(
    const TaskTarget& target);

  /**
   * @brief 创建设置ACM允许碰撞的stage
   * 
   * 允许夹爪链接与缆绳碰撞对象之间的碰撞
   * 这对于下探和闭合阶段非常重要
   * 
   * @return ModifyPlanningScene stage
   */
  std::unique_ptr<mtc::stages::ModifyPlanningScene> makeAllowCollisions();

  /**
   * @brief 创建附着物体的stage
   * 
   * 将缆绳碰撞对象附着到夹爪上
   * 
   * @return ModifyPlanningScene stage
   */
  std::unique_ptr<mtc::stages::ModifyPlanningScene> makeAttachObject();

  /**
   * @brief 创建分离物体的stage
   * 
   * 将缆绳碰撞对象从夹爪分离
   * 
   * @return ModifyPlanningScene stage
   */
  std::unique_ptr<mtc::stages::ModifyPlanningScene> makeDetachObject();

  TaskContext ctx_;
};

}  // namespace m5_grasp
