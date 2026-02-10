#ifndef M5_GRASP__TRAJECTORY_PLANNER_HPP_
#define M5_GRASP__TRAJECTORY_PLANNER_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <map>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <string>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <vector>

namespace sealien_payload_grasp
{

/**
 * @brief 时间参数化算法类型
 */
enum class TimeParameterizationType
{
    TOTG,               // Time-Optimal Trajectory Generation（时间最优，默认）
    ITERATIVE_SPLINE,   // Iterative Spline Parameterization（迭代样条，更平滑）
    ITERATIVE_PARABOLIC // Iterative Parabolic Time Parameterization（迭代抛物线）
};

/**
 * @brief 轨迹规划工具类
 *
 * 提供关节空间多项式插值、笛卡尔空间直线插补、B样条曲线轨迹规划和时间参数化功能
 */
class TrajectoryPlanner
{
  public:
    TrajectoryPlanner() = default;
    ~TrajectoryPlanner() = default;

    /**
     * @brief 生成关节空间三次多项式插值轨迹
     *
     * @param start_joints 起始关节角度（弧度）
     * @param end_joints 目标关节角度（弧度）
     * @param start_vel 起始关节速度（弧度/秒），如果为空则默认为0
     * @param end_vel 目标关节速度（弧度/秒），如果为空则默认为0
     * @param duration 轨迹持续时间（秒）
     * @param dt 时间步长（秒），默认0.01
     * @param joint_names 关节名称列表
     * @return trajectory_msgs::msg::JointTrajectory 生成的轨迹
     */
    trajectory_msgs::msg::JointTrajectory generate_cubic_polynomial_trajectory(
        const std::vector<double>& start_joints, const std::vector<double>& end_joints,
        const std::vector<double>& start_vel, const std::vector<double>& end_vel, double duration,
        const std::vector<std::string>& joint_names, double dt = 0.01);

    /**
     * @brief 生成关节空间五次多项式插值轨迹
     *
     * @param start_joints 起始关节角度（弧度）
     * @param end_joints 目标关节角度（弧度）
     * @param start_vel 起始关节速度（弧度/秒）
     * @param end_vel 目标关节速度（弧度/秒）
     * @param start_acc 起始关节加速度（弧度/秒²）
     * @param end_acc 目标关节加速度（弧度/秒²）
     * @param duration 轨迹持续时间（秒）
     * @param dt 时间步长（秒），默认0.01
     * @param joint_names 关节名称列表
     * @return trajectory_msgs::msg::JointTrajectory 生成的轨迹
     */
    trajectory_msgs::msg::JointTrajectory generate_quintic_polynomial_trajectory(
        const std::vector<double>& start_joints, const std::vector<double>& end_joints,
        const std::vector<double>& start_vel, const std::vector<double>& end_vel,
        const std::vector<double>& start_acc, const std::vector<double>& end_acc, double duration,
        const std::vector<std::string>& joint_names, double dt = 0.01);

    /**
     * @brief 生成笛卡尔空间直线插补轨迹
     *
     * 位置使用线性插补，姿态使用球面线性插补（SLERP）
     *
     * @param start_pose 起始位姿
     * @param end_pose 目标位姿
     * @param duration 轨迹持续时间（秒）
     * @param dt 时间步长（秒），默认0.01
     * @return std::vector<geometry_msgs::msg::Pose> 生成的轨迹点序列
     */
    std::vector<geometry_msgs::msg::Pose>
    generate_linear_cartesian_trajectory(const geometry_msgs::msg::Pose& start_pose,
                                      const geometry_msgs::msg::Pose& end_pose, double duration,
                                      double dt = 0.01);

    /**
     * @brief 生成关节空间B样条曲线轨迹
     *
     * @param control_points 控制点序列，每个控制点是一个关节角度向量
     * @param degree B样条阶数（3或4）
     * @param duration 轨迹持续时间（秒）
     * @param dt 时间步长（秒），默认0.01
     * @param joint_names 关节名称列表
     * @return trajectory_msgs::msg::JointTrajectory 生成的轨迹
     */
    trajectory_msgs::msg::JointTrajectory
    generate_bspline_joint_trajectory(const std::vector<std::vector<double>>& control_points,
                                   int degree, double duration,
                                   const std::vector<std::string>& joint_names, double dt = 0.01);

    /**
     * @brief 生成笛卡尔空间B样条曲线轨迹
     *
     * @param control_points 控制点位姿序列
     * @param degree B样条阶数（3或4）
     * @param duration 轨迹持续时间（秒）
     * @param dt 时间步长（秒），默认0.01
     * @return std::vector<geometry_msgs::msg::Pose> 生成的轨迹点序列
     */
    std::vector<geometry_msgs::msg::Pose>
    generate_bspline_cartesian_trajectory(const std::vector<geometry_msgs::msg::Pose>& control_points,
                                       int degree, double duration, double dt = 0.01);

    /**
     * @brief 对轨迹进行时间参数化（重新计算时间戳，考虑速度/加速度限制）
     *
     * 使用MoveIt的时间参数化算法对轨迹进行处理，生成满足关节速度和加速度限制的平滑轨迹。
     *
     * @param trajectory 输入/输出的JointTrajectory消息
     * @param robot_model MoveIt机器人模型
     * @param group_name 规划组名称（如"arm_group"）
     * @param max_velocity_scaling 最大速度缩放因子（0-1）
     * @param max_acceleration_scaling 最大加速度缩放因子（0-1）
     * @param algorithm 时间参数化算法类型
     * @return bool 成功返回true，失败返回false
     */
    bool apply_time_parameterization(
        trajectory_msgs::msg::JointTrajectory& trajectory,
        const moveit::core::RobotModelConstPtr& robot_model, const std::string& group_name,
        double max_velocity_scaling = 1.0, double max_acceleration_scaling = 1.0,
        TimeParameterizationType algorithm = TimeParameterizationType::ITERATIVE_SPLINE);

    /**
     * @brief 对RobotTrajectory进行时间参数化
     *
     * @param robot_trajectory MoveIt RobotTrajectory对象
     * @param max_velocity_scaling 最大速度缩放因子（0-1）
     * @param max_acceleration_scaling 最大加速度缩放因子（0-1）
     * @param algorithm 时间参数化算法类型
     * @return bool 成功返回true，失败返回false
     */
    bool apply_time_parameterization(
        robot_trajectory::RobotTrajectory& robot_trajectory, double max_velocity_scaling = 1.0,
        double max_acceleration_scaling = 1.0,
        TimeParameterizationType algorithm = TimeParameterizationType::ITERATIVE_SPLINE);

    /**
     * @brief 根据控制频率对轨迹进行重采样
     *
     * 将轨迹重采样到指定的控制频率（如20Hz），确保轨迹点间隔均匀
     *
     * @param trajectory 输入/输出的JointTrajectory消息
     * @param control_frequency 控制频率（Hz），默认20Hz
     * @return bool 成功返回true，失败返回false
     */
    bool resample_trajectory(trajectory_msgs::msg::JointTrajectory& trajectory,
                            double control_frequency = 20.0);

  private:
    /**
     * @brief 计算B样条基函数值（Cox-de Boor递归算法）
     *
     * @param i 基函数索引
     * @param k 阶数（degree）
     * @param t 参数值（通常在[0, 1]范围内）
     * @param knots 节点向量
     * @return double 基函数值
     */
    double bspline_basis(int i, int k, double t, const std::vector<double>& knots);

    /**
     * @brief 生成均匀节点向量
     *
     * @param num_control_points 控制点数量
     * @param degree B样条阶数
     * @return std::vector<double> 节点向量
     */
    std::vector<double> generate_knot_vector(int num_control_points, int degree);

    /**
     * @brief 四元数球面线性插补（SLERP）
     *
     * @param q1 起始四元数
     * @param q2 目标四元数
     * @param t 插值参数（0到1）
     * @return geometry_msgs::msg::Quaternion 插值后的四元数
     */
    geometry_msgs::msg::Quaternion slerp(const geometry_msgs::msg::Quaternion& q1,
                                         const geometry_msgs::msg::Quaternion& q2, double t);

    /**
     * @brief 归一化四元数
     *
     * @param q 输入四元数
     * @return geometry_msgs::msg::Quaternion 归一化后的四元数
     */
    geometry_msgs::msg::Quaternion normalize_quaternion(const geometry_msgs::msg::Quaternion& q);
};

} // namespace sealien_payload_grasp

#endif // M5_GRASP__TRAJECTORY_PLANNER_HPP_
