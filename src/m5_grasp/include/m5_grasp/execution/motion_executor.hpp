#ifndef M5_GRASP_MOTION_EXECUTOR_HPP_
#define M5_GRASP_MOTION_EXECUTOR_HPP_

#include <functional>
#include <future>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_ros/buffer.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <vector>

#include "m5_grasp/utils/grasp_types.hpp"

// 前向声明
namespace m5_grasp
{
class TrajectoryPlanner;
}

namespace m5_grasp
{

/**
 * @brief 运动执行器类，封装所有运动规划和执行相关的功能
 */
class MotionExecutor
{
  public:
    /**
     * @brief 构造函数
     * @param node ROS2节点指针
     * @param move_group_interface MoveGroupInterface共享指针
     * @param tf_buffer TF2 Buffer共享指针
     * @param trajectory_planner TrajectoryPlanner引用
     * @param moveit_mutex MoveIt互斥锁引用
     */
    MotionExecutor(
        rclcpp::Node* node,
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface,
        std::shared_ptr<tf2_ros::Buffer> tf_buffer, TrajectoryPlanner& trajectory_planner,
        std::mutex& moveit_mutex);

    /**
     * @brief 设置运动参数
     */
    void setParameters(const std::string& planning_frame, const std::string& eef_link,
                       double ik_timeout, bool is_4dof,
                       // Cartesian path parameters
                       double eef_step, double jump_threshold, double cartesian_timeout_descend,
                       double cartesian_timeout_lift, double cartesian_descend_threshold,
                       double cartesian_lift_threshold, bool cartesian_retry_on_timeout,
                       // Interpolation parameters
                       bool use_linear_interpolation, double linear_velocity,
                       double linear_min_duration, double linear_step_scale, bool use_bspline,
                       int bspline_degree, double bspline_duration, double bspline_dt,
                       bool use_polynomial_interpolation, const std::string& polynomial_type,
                       double polynomial_duration, double polynomial_dt,
                       // Segment parameters
                       bool adaptive_segments, int min_segments, int max_segments,
                       double segment_execution_wait, double state_sync_wait,
                       // Recovery parameters
                       bool enable_recovery, double recovery_timeout,
                       // Planning parameters
                       double default_planning_time, double fallback_planning_time,
                       // Yaw validation
                       bool yaw_validation_enabled, double yaw_tolerance, double yaw_search_step);

    /**
     * @brief 设置坐标转换回调函数
     */
    void setTransformCallback(std::function<bool(geometry_msgs::msg::PoseStamped&)> callback);

    /**
     * @brief 设置yaw搜索回调函数
     */
    void setFindValidYawCallback(std::function<double(double, double, double, double,
                                                      moveit::core::RobotStatePtr, double, double)>
                                     callback);

    /**
     * @brief 设置orientation计算回调函数
     */
    void setComputeOrientationCallbacks(
        std::function<geometry_msgs::msg::Quaternion()> compute_downward,
        std::function<geometry_msgs::msg::Quaternion(double)> compute_downward_with_yaw,
        std::function<std::vector<geometry_msgs::msg::Quaternion>()> compute_candidates);

    /**
     * @brief 设置状态发布回调函数
     */
    void
    setPublishStateCallback(std::function<void(const std::string&, const std::string&)> callback);

    // ============== Cartesian Path Execution ==============

    /**
     * @brief 带超时的笛卡尔路径计算
     * @return 成功率（-1.0表示超时）
     */
    double computeCartesianPathWithTimeout(const std::vector<geometry_msgs::msg::Pose>& waypoints,
                                           double step, double jump_threshold,
                                           moveit_msgs::msg::RobotTrajectory& trajectory,
                                           double timeout_sec, bool allow_retry = false);

    /**
     * @brief 执行笛卡尔路径（支持B样条和线性插补）
     * @param waypoints 路径点列表
     * @param use_descend true表示下压，false表示抬起
     */
    bool executeCartesian(const std::vector<geometry_msgs::msg::Pose>& waypoints,
                          bool use_descend = true);

    // ============== Segmented Movement ==============

    /**
     * @brief 计算自适应分段数
     */
    int calculate_adaptive_segments(const geometry_msgs::msg::PoseStamped& start_pose,
                                  const geometry_msgs::msg::PoseStamped& end_pose);

    /**
     * @brief 分段下压执行（4DOF专用）
     */
    bool execute_segmented_descend(const geometry_msgs::msg::PoseStamped& start_pose,
                                 const geometry_msgs::msg::PoseStamped& end_pose,
                                 int num_segments = -1);

    /**
     * @brief 分段提升执行
     */
    bool executeSegmentedLift(const geometry_msgs::msg::PoseStamped& start_pose,
                              const geometry_msgs::msg::PoseStamped& end_pose,
                              int num_segments = 3);

    // ============== IK and Joint Target ==============

    /**
     * @brief IK→joint target规划执行（内部实现，需在锁内调用）
     */
    bool planExecuteIKJointTargetImpl(const geometry_msgs::msg::PoseStamped& target_pose,
                                      int max_ik_attempts = 3);

    /**
     * @brief IK→joint target规划执行（公共接口，带锁）
     */
    bool planExecuteIKJointTarget(const geometry_msgs::msg::PoseStamped& target_pose,
                                  int max_ik_attempts = 3);

    // ============== Polynomial and B-Spline Trajectories ==============

    /**
     * @brief 使用多项式插值执行关节轨迹
     */
    bool executePolynomialJointTrajectory(const std::vector<double>& start_joints,
                                          const std::vector<double>& end_joints);

    /**
     * @brief 使用B样条执行关节轨迹
     */
    bool executeBSplineJointTrajectory(const std::vector<std::vector<double>>& control_points);

    // ============== Pose Target Planning ==============

    /**
     * @brief 点对点运动规划执行
     */
    bool planExecutePoseTarget(const geometry_msgs::msg::PoseStamped& target_pose);

    // ============== Recovery ==============

    /**
     * @brief 回滚到预抓取位置
     */
    bool recoverToPregrasp(const geometry_msgs::msg::PoseStamped& pregrasp_pose);

    /**
     * @brief 带回滚的执行
     */
    bool executeWithRecovery(moveit::planning_interface::MoveGroupInterface::Plan& plan,
                             const geometry_msgs::msg::PoseStamped& recovery_pose);

    // ============== State Utilities ==============

    /**
     * @brief 等待状态稳定
     */
    bool waitForStateStable(double timeout_sec, int check_count = 5, double threshold = 0.002);

    /**
     * @brief 显式设置起始状态
     */
    void setStartStateExplicit();

    /**
     * @brief 安全获取当前状态
     */
    moveit::core::RobotStatePtr get_current_state_safe(double timeout = 1.0);

    /**
     * @brief 安全获取当前关节值
     */
    std::vector<double> get_current_joint_values_safe();

    /**
     * @brief 安全获取当前位姿
     */
    geometry_msgs::msg::PoseStamped get_current_pose_safe();

  private:
    rclcpp::Node* node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    TrajectoryPlanner& trajectory_planner_;
    std::mutex& moveit_mutex_;

    // Parameters
    std::string planning_frame_;
    std::string eef_link_;
    double ik_timeout_;
    bool is_4dof_;

    // Cartesian path parameters
    double eef_step_;
    double jump_threshold_;
    double cartesian_timeout_descend_;
    double cartesian_timeout_lift_;
    double cartesian_descend_threshold_;
    double cartesian_lift_threshold_;
    bool cartesian_retry_on_timeout_;

    // Interpolation parameters
    bool use_linear_interpolation_;
    double linear_velocity_;
    double linear_min_duration_;
    double linear_step_scale_;
    bool use_bspline_;
    int bspline_degree_;
    double bspline_duration_;
    double bspline_dt_;
    bool use_polynomial_interpolation_;
    std::string polynomial_type_;
    double polynomial_duration_;
    double polynomial_dt_;

    // Segment parameters
    bool adaptive_segments_;
    int min_segments_;
    int max_segments_;
    double segment_execution_wait_;
    double state_sync_wait_;

    // Recovery parameters
    bool enable_recovery_;
    double recovery_timeout_;

    // Planning parameters
    double default_planning_time_;
    double fallback_planning_time_;

    // Yaw validation
    bool yaw_validation_enabled_;
    double yaw_tolerance_;
    double yaw_search_step_;

    // Callback functions
    std::function<bool(geometry_msgs::msg::PoseStamped&)> transform_to_planning_callback_;
    std::function<double(double, double, double, double, moveit::core::RobotStatePtr, double,
                         double)>
        find_valid_yaw_callback_;
    std::function<geometry_msgs::msg::Quaternion()> compute_downward_callback_;
    std::function<geometry_msgs::msg::Quaternion(double)> compute_downward_with_yaw_callback_;
    std::function<std::vector<geometry_msgs::msg::Quaternion>()>
        compute_orientation_candidates_callback_;
    std::function<void(const std::string&, const std::string&)> publish_state_callback_;

    // Internal helpers
    bool checkIKOnlyImpl(const geometry_msgs::msg::PoseStamped& pose_in_planning,
                         const moveit::core::RobotStatePtr& state);
};

} // namespace m5_grasp

#endif // M5_GRASP_MOTION_EXECUTOR_HPP_
