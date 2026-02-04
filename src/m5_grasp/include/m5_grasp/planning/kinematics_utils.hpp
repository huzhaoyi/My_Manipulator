#ifndef M5_GRASP_KINEMATICS_UTILS_HPP
#define M5_GRASP_KINEMATICS_UTILS_HPP

#include "m5_grasp/utils/grasp_types.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace m5_grasp
{

/**
 * @brief 运动学工具类
 *
 * 提供IK检查、工作空间检查、yaw搜索等功能
 */
class KinematicsUtils
{
  public:
    /**
     * @brief 构造函数
     */
    KinematicsUtils(
        rclcpp::Node* node,
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface,
        std::mutex& moveit_mutex);

    /**
     * @brief 设置参数
     */
    void setParameters(const std::string& eef_link, double ik_timeout, bool is_4dof,
                       const WorkspaceParameters& workspace_params, double yaw_tolerance,
                       double yaw_search_step, bool joint4_yaw_search_enabled,
                       double joint4_yaw_search_range, double joint4_yaw_search_step);

    /**
     * @brief 检查位置是否在工作空间内
     * @param x X坐标
     * @param y Y坐标
     * @param z Z坐标
     * @return 是否可达
     */
    bool isReachable(double x, double y, double z);

    /**
     * @brief 调整位置到工作空间内
     */
    void adjustToWorkspace(double& x, double& y, double& z);

    /**
     * @brief 检查IK是否可解（仅检查，不执行）
     * @param pose_in_planning planning frame中的位姿
     * @return 是否可解
     */
    bool checkIKOnly(const geometry_msgs::msg::PoseStamped& pose_in_planning);

    /**
     * @brief 检查IK是否可解且无碰撞
     * @param pose_in_planning planning frame中的位姿
     * @return 是否可解且无碰撞
     */
    bool checkIKWithCollision(const geometry_msgs::msg::PoseStamped& pose_in_planning);

    /**
     * @brief 检查指定yaw的IK是否有效
     * @param x X坐标
     * @param y Y坐标
     * @param z Z坐标
     * @param yaw yaw角度
     * @param valid_yaw 输出的有效yaw
     * @return 是否有效
     */
    bool isYawIKValid(double x, double y, double z, double yaw, double& valid_yaw);

    /**
     * @brief 寻找有效的yaw角度
     * @param x X坐标
     * @param y Y坐标
     * @param z Z坐标
     * @param original_yaw 原始yaw
     * @param tolerance 容差
     * @param step 搜索步长
     * @return 有效的yaw，失败返回NaN
     */
    double findValidYaw(double x, double y, double z, double original_yaw, double tolerance,
                        double step);

    /**
     * @brief 寻找最佳的yaw候选
     */
    double findBestYawCandidate(double x, double y, double z, double reference_yaw,
                                double search_range, double search_step);

    /**
     * @brief 检查Joint2/3是否在可达范围内
     * @param state 机器人状态
     * @param jmg_name 关节组名称
     * @param joint2_3_values 输出的Joint2/3值
     * @return 是否可达
     */
    bool check_joint23_reachable(const moveit::core::RobotStatePtr& state,
                               const std::string& jmg_name, std::vector<double>& joint2_3_values);

    /**
     * @brief 尝试多个yaw候选进行IK求解
     */
    YawIKResult try_yaw_candidates_for_ik(const geometry_msgs::msg::Pose& target_pose,
                                      const std::vector<double>& yaw_candidates,
                                      const std::string& eef_link, double ik_timeout = 0.5);

    /**
     * @brief 寻找最优的yaw使Joint4达到目标
     */
    bool findOptimalYawForJoint4(double target_yaw, double x, double y, double z, double& best_yaw,
                                 double& best_joint4, moveit::core::RobotStatePtr& best_state);

    /**
     * @brief 计算向下的姿态（夹爪朝下）
     */
    geometry_msgs::msg::Quaternion compute_downward_orientation();

    /**
     * @brief 计算带yaw的向下姿态
     */
    geometry_msgs::msg::Quaternion compute_downward_orientation_with_yaw(double yaw);

    /**
     * @brief 获取当前机器人状态（线程安全）
     */
    moveit::core::RobotStatePtr get_current_state_safe(double timeout = 0.0);

    /**
     * @brief 获取当前关节值（线程安全）
     */
    std::vector<double> get_current_joint_values_safe();

    /**
     * @brief 获取当前位姿（线程安全）
     */
    geometry_msgs::msg::PoseStamped get_current_pose_safe();

  private:
    rclcpp::Node* node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    std::mutex& moveit_mutex_;

    // 参数
    std::string eef_link_;
    double ik_timeout_{0.1};
    bool is_4dof_{false};
    WorkspaceParameters workspace_params_;
    double yaw_tolerance_{0.0873};
    double yaw_search_step_{0.0175};
    bool joint4_yaw_search_enabled_{true};
    double joint4_yaw_search_range_{M_PI};
    double joint4_yaw_search_step_{0.0873};
};

} // namespace m5_grasp

#endif // M5_GRASP_KINEMATICS_UTILS_HPP
