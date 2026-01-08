#ifndef M5_GRASP__TRAJECTORY_PLANNER_HPP_
#define M5_GRASP__TRAJECTORY_PLANNER_HPP_

#include <vector>
#include <geometry_msgs/msg/pose.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace m5_grasp
{

/**
 * @brief 轨迹规划工具类
 * 
 * 提供关节空间多项式插值、笛卡尔空间直线插补和B样条曲线轨迹规划功能
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
  trajectory_msgs::msg::JointTrajectory generateCubicPolynomialTrajectory(
    const std::vector<double>& start_joints,
    const std::vector<double>& end_joints,
    const std::vector<double>& start_vel,
    const std::vector<double>& end_vel,
    double duration,
    const std::vector<std::string>& joint_names,
    double dt = 0.01
  );

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
  trajectory_msgs::msg::JointTrajectory generateQuinticPolynomialTrajectory(
    const std::vector<double>& start_joints,
    const std::vector<double>& end_joints,
    const std::vector<double>& start_vel,
    const std::vector<double>& end_vel,
    const std::vector<double>& start_acc,
    const std::vector<double>& end_acc,
    double duration,
    const std::vector<std::string>& joint_names,
    double dt = 0.01
  );

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
  std::vector<geometry_msgs::msg::Pose> generateLinearCartesianTrajectory(
    const geometry_msgs::msg::Pose& start_pose,
    const geometry_msgs::msg::Pose& end_pose,
    double duration,
    double dt = 0.01
  );

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
  trajectory_msgs::msg::JointTrajectory generateBSplineJointTrajectory(
    const std::vector<std::vector<double>>& control_points,
    int degree,
    double duration,
    const std::vector<std::string>& joint_names,
    double dt = 0.01
  );

  /**
   * @brief 生成笛卡尔空间B样条曲线轨迹
   * 
   * @param control_points 控制点位姿序列
   * @param degree B样条阶数（3或4）
   * @param duration 轨迹持续时间（秒）
   * @param dt 时间步长（秒），默认0.01
   * @return std::vector<geometry_msgs::msg::Pose> 生成的轨迹点序列
   */
  std::vector<geometry_msgs::msg::Pose> generateBSplineCartesianTrajectory(
    const std::vector<geometry_msgs::msg::Pose>& control_points,
    int degree,
    double duration,
    double dt = 0.01
  );

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
  double bsplineBasis(int i, int k, double t, const std::vector<double>& knots);

  /**
   * @brief 生成均匀节点向量
   * 
   * @param num_control_points 控制点数量
   * @param degree B样条阶数
   * @return std::vector<double> 节点向量
   */
  std::vector<double> generateKnotVector(int num_control_points, int degree);

  /**
   * @brief 四元数球面线性插补（SLERP）
   * 
   * @param q1 起始四元数
   * @param q2 目标四元数
   * @param t 插值参数（0到1）
   * @return geometry_msgs::msg::Quaternion 插值后的四元数
   */
  geometry_msgs::msg::Quaternion slerp(
    const geometry_msgs::msg::Quaternion& q1,
    const geometry_msgs::msg::Quaternion& q2,
    double t
  );

  /**
   * @brief 归一化四元数
   * 
   * @param q 输入四元数
   * @return geometry_msgs::msg::Quaternion 归一化后的四元数
   */
  geometry_msgs::msg::Quaternion normalizeQuaternion(
    const geometry_msgs::msg::Quaternion& q
  );
};

}  // namespace m5_grasp

#endif  // M5_GRASP__TRAJECTORY_PLANNER_HPP_
