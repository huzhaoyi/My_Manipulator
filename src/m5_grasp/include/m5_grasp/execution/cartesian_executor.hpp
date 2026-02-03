#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <mutex>
#include <future>
#include <functional>

#include "m5_grasp/utils/trajectory_planner.hpp"

namespace m5_grasp {

/**
 * @brief 笛卡尔路径执行器
 * 
 * 负责笛卡尔路径的计算、执行和分段执行策略
 */
class CartesianExecutor {
public:
  using MoveGroupInterfacePtr = std::shared_ptr<moveit::planning_interface::MoveGroupInterface>;
  using TransformCallback = std::function<bool(geometry_msgs::msg::PoseStamped&)>;
  using FindValidYawCallback = std::function<double(double, double, double, double, double, double)>;
  using ComputeOrientationCallback = std::function<geometry_msgs::msg::Quaternion(double)>;
  using PlanExecuteCallback = std::function<bool(const geometry_msgs::msg::PoseStamped&)>;
  using RecoverCallback = std::function<bool(const geometry_msgs::msg::PoseStamped&)>;
  using WaitForStateCallback = std::function<bool(double, int, double)>;

  /**
   * @brief 构造函数
   */
  CartesianExecutor(rclcpp::Node* node,
                    MoveGroupInterfacePtr move_group,
                    std::mutex& mutex);

  /**
   * @brief 设置基本参数
   */
  void setParameters(double eef_step, double jump_threshold,
                     double cartesian_timeout, double cartesian_timeout_descend,
                     double cartesian_timeout_lift,
                     double cartesian_descend_threshold, double cartesian_lift_threshold,
                     bool cartesian_retry_on_timeout);

  /**
   * @brief 设置线性插补参数
   */
  void setLinearParameters(bool use_linear, double linear_velocity,
                           double linear_min_duration, double linear_step_scale);

  /**
   * @brief 设置B样条参数
   */
  void setBSplineParameters(bool use_bspline, int bspline_degree,
                            double bspline_duration, double bspline_dt);

  /**
   * @brief 设置分段执行参数
   */
  void setSegmentParameters(bool adaptive_segments, int min_segments, int max_segments,
                            double state_sync_wait, double segment_execution_wait,
                            bool enable_recovery);

  /**
   * @brief 设置yaw验证参数
   */
  void setYawValidationParameters(bool enabled, double tolerance, double search_step);

  /**
   * @brief 设置回调函数
   */
  void setCallbacks(TransformCallback transform_cb,
                    FindValidYawCallback find_yaw_cb,
                    ComputeOrientationCallback compute_orientation_cb,
                    PlanExecuteCallback plan_execute_cb,
                    RecoverCallback recover_cb,
                    WaitForStateCallback wait_state_cb);

  /**
   * @brief 设置TrajectoryPlanner引用
   */
  void setTrajectoryPlanner(TrajectoryPlanner* planner);

  /**
   * @brief 设置planning frame和eef link
   */
  void setFrames(const std::string& planning_frame, const std::string& eef_link);

  /**
   * @brief 设置IK超时
   */
  void setIKTimeout(double timeout);

  /**
   * @brief 带超时的笛卡尔路径计算
   */
  double computeCartesianPathWithTimeout(
      const std::vector<geometry_msgs::msg::Pose>& waypoints,
      double step, double jump_threshold,
      moveit_msgs::msg::RobotTrajectory& trajectory,
      double timeout_sec, bool allow_retry = false);

  /**
   * @brief 执行笛卡尔路径
   * @param waypoints 路径点
   * @param use_descend true表示下压，false表示抬起
   */
  bool executeCartesian(const std::vector<geometry_msgs::msg::Pose>& waypoints,
                        bool use_descend = true);

  /**
   * @brief 计算自适应分段数
   */
  int calculateAdaptiveSegments(const geometry_msgs::msg::PoseStamped& start_pose,
                                const geometry_msgs::msg::PoseStamped& end_pose);

  /**
   * @brief 分段下压执行
   */
  bool executeSegmentedDescend(const geometry_msgs::msg::PoseStamped& start_pose,
                               const geometry_msgs::msg::PoseStamped& end_pose,
                               int num_segments = -1);

  /**
   * @brief 分段提升执行
   */
  bool executeSegmentedLift(const geometry_msgs::msg::PoseStamped& start_pose,
                            const geometry_msgs::msg::PoseStamped& end_pose,
                            int num_segments = 3);

private:
  rclcpp::Node* node_;
  MoveGroupInterfacePtr move_group_interface_;
  std::mutex& moveit_mutex_;
  TrajectoryPlanner* trajectory_planner_{nullptr};

  // 基本参数
  double eef_step_{0.01};
  double jump_threshold_{0.0};
  double cartesian_timeout_{3.0};
  double cartesian_timeout_descend_{5.0};
  double cartesian_timeout_lift_{3.0};
  double cartesian_descend_threshold_{0.95};
  double cartesian_lift_threshold_{0.90};
  bool cartesian_retry_on_timeout_{true};

  // 线性插补参数
  bool use_linear_interpolation_{false};
  double linear_velocity_{0.05};
  double linear_min_duration_{0.5};
  double linear_step_scale_{2.0};

  // B样条参数
  bool use_bspline_{false};
  int bspline_degree_{3};
  double bspline_duration_{2.0};
  double bspline_dt_{0.02};

  // 分段执行参数
  bool adaptive_segments_{true};
  int min_segments_{2};
  int max_segments_{5};
  double state_sync_wait_{0.1};
  double segment_execution_wait_{0.2};
  bool enable_recovery_{true};

  // yaw验证参数
  bool yaw_validation_enabled_{false};
  double yaw_tolerance_{0.087};  // 5度
  double yaw_search_step_{0.0175};  // 1度

  // 帧信息
  std::string planning_frame_{"world"};
  std::string eef_link_{"LinkGG"};
  double ik_timeout_{0.5};

  // 回调函数
  TransformCallback transform_callback_;
  FindValidYawCallback find_valid_yaw_callback_;
  ComputeOrientationCallback compute_orientation_callback_;
  PlanExecuteCallback plan_execute_callback_;
  RecoverCallback recover_callback_;
  WaitForStateCallback wait_state_callback_;
};

}  // namespace m5_grasp
