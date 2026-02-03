#ifndef M5_GRASP_MTC_TASK_CREATOR_HPP_
#define M5_GRASP_MTC_TASK_CREATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <mutex>
#include <vector>
#include <string>
#include <functional>

#include "m5_grasp/utils/grasp_types.hpp"

namespace mtc = moveit::task_constructor;

namespace m5_grasp
{

/**
 * @brief MTC任务创建器类，封装所有MoveIt Task Constructor相关的功能
 */
class MTCTaskCreator
{
public:
  /**
   * @brief 构造函数
   */
  MTCTaskCreator(
      rclcpp::Node::SharedPtr node,
      std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface,
      std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_interface,
      std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface,
      std::shared_ptr<tf2_ros::Buffer> tf_buffer,
      std::mutex& moveit_mutex);

  /**
   * @brief 设置基本参数
   */
  void setBasicParameters(
      const std::string& planning_frame,
      const std::string& eef_link,
      const std::string& cable_name,
      double ik_timeout,
      bool add_collision_object);

  /**
   * @brief 设置抓取参数
   */
  void setGraspParameters(
      double approach_offset_z,
      double descend_distance,
      double lift_height,
      double cable_length,
      double cable_diameter);

  /**
   * @brief 设置夹爪参数
   */
  void setGripperParameters(
      double gripper_open_width,
      double gripper_close_width,
      double gripper_min_joint,
      double gripper_max_joint,
      double gripper_min_width,
      double gripper_max_width);

  /**
   * @brief 设置TCP偏移参数
   */
  void setTCPParameters(
      double tcp_offset_x,
      double tcp_offset_y,
      double tcp_offset_z);

  /**
   * @brief 设置关节约束容差参数
   */
  void setJointTolerances(
      double joint1_tolerance,
      double joint4_tolerance);

  /**
   * @brief 设置允许接触的link列表
   */
  void setAllowTouchLinks(const std::vector<std::string>& links);

  /**
   * @brief 设置坐标转换回调函数
   */
  void setTransformCallback(std::function<bool(geometry_msgs::msg::PoseStamped&)> callback);

  /**
   * @brief 设置宽度到关节角度转换回调函数
   */
  void setWidthToJointCallback(std::function<double(double)> callback);

  /**
   * @brief 设置orientation计算回调函数
   */
  void setOrientationCallbacks(
      std::function<geometry_msgs::msg::Quaternion()> compute_downward_fixed,
      std::function<geometry_msgs::msg::Pose(const geometry_msgs::msg::PoseStamped&, double)> make_cable_cylinder_pose);

  /**
   * @brief 设置IK检查回调函数
   */
  void setIKCheckCallback(
      std::function<YawIKResult(const geometry_msgs::msg::Pose&, const std::vector<double>&, const std::string&, double)> callback);

  /**
   * @brief 创建抓取任务
   * @param cable_pose 缆绳位姿（夹爪中心目标）
   * @param cable_yaw 缆绳yaw角度
   * @param joint4_target Joint4目标值
   * @return MTC Task对象
   */
  mtc::Task createGraspTask(
      const geometry_msgs::msg::PoseStamped& cable_pose,
      double cable_yaw,
      double joint4_target);

  /**
   * @brief 创建放置任务
   * @param place_pose 放置位姿
   * @param cable_yaw 缆绳yaw角度
   * @return MTC Task对象
   */
  mtc::Task createPlaceTask(
      const geometry_msgs::msg::PoseStamped& place_pose,
      double cable_yaw);

  /**
   * @brief 创建完整的抓取-放置任务
   */
  mtc::Task createGraspAndPlaceTask(
      const geometry_msgs::msg::PoseStamped& cable_pose,
      const geometry_msgs::msg::PoseStamped& place_pose,
      double cable_yaw,
      double joint4_target);

private:
  // 节点和接口
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_interface_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::mutex& moveit_mutex_;

  // 基本参数
  std::string planning_frame_;
  std::string eef_link_;
  std::string cable_name_;
  double ik_timeout_;
  bool add_collision_object_;

  // 抓取参数
  double approach_offset_z_;
  double descend_distance_;
  double lift_height_;
  double cable_length_;
  double cable_diameter_;

  // 夹爪参数
  double gripper_open_width_;
  double gripper_close_width_;
  double gripper_min_joint_;
  double gripper_max_joint_;
  double gripper_min_width_;
  double gripper_max_width_;

  // TCP偏移参数
  double tcp_offset_x_;
  double tcp_offset_y_;
  double tcp_offset_z_;

  // 关节约束容差
  double joint1_tolerance_;
  double joint4_tolerance_;

  // 允许接触的link列表
  std::vector<std::string> allow_touch_links_;

  // 回调函数
  std::function<bool(geometry_msgs::msg::PoseStamped&)> transform_callback_;
  std::function<double(double)> width_to_joint_callback_;
  std::function<geometry_msgs::msg::Quaternion()> compute_downward_fixed_callback_;
  std::function<geometry_msgs::msg::Pose(const geometry_msgs::msg::PoseStamped&, double)> make_cable_cylinder_pose_callback_;
  std::function<YawIKResult(const geometry_msgs::msg::Pose&, const std::vector<double>&, const std::string&, double)> ik_check_callback_;

  // 内部辅助函数
  mtc::solvers::PipelinePlannerPtr createPipelinePlanner(mtc::Task& task);
  double calculateJoint1Auto(const geometry_msgs::msg::PoseStamped& target_pose);
  std::map<std::string, double> preparePregraspJointMap(
      const geometry_msgs::msg::PoseStamped& pregrasp_pose,
      double cable_yaw,
      double joint1_auto,
      double joint4_target,
      double& selected_yaw,
      bool& use_joint_goal);
  void addFixedStartState(mtc::Task& task);
  void addOpenGripperStage(mtc::Task& task, mtc::solvers::PipelinePlannerPtr planner);
  void addCloseGripperStage(mtc::Task& task, mtc::solvers::PipelinePlannerPtr planner);
  void addMoveToPregraspStage(
      mtc::Task& task,
      mtc::solvers::PipelinePlannerPtr planner,
      const geometry_msgs::msg::PoseStamped& pregrasp_pose,
      const std::map<std::string, double>& joint_map,
      bool use_joint_goal,
      double joint1_auto,
      double joint4_target);
  void addCableCollisionObject(
      mtc::Task& task,
      const geometry_msgs::msg::PoseStamped& cable_pose,
      double cable_yaw);
  void addAllowCollisionsStage(mtc::Task& task);
  void addDescendStage(
      mtc::Task& task,
      mtc::solvers::PipelinePlannerPtr planner,
      const geometry_msgs::msg::PoseStamped& grasp_pose);
  void addAttachObjectStage(mtc::Task& task);
  void addLiftStage(
      mtc::Task& task,
      mtc::solvers::PipelinePlannerPtr planner);
};

}  // namespace m5_grasp

#endif  // M5_GRASP_MTC_TASK_CREATOR_HPP_
