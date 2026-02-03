#include "m5_grasp/utils/trajectory_planner.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

// MoveIt时间参数化头文件
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/robot_state/robot_state.h>

namespace m5_grasp
{

trajectory_msgs::msg::JointTrajectory TrajectoryPlanner::generateCubicPolynomialTrajectory(
  const std::vector<double>& start_joints,
  const std::vector<double>& end_joints,
  const std::vector<double>& start_vel,
  const std::vector<double>& end_vel,
  double duration,
  const std::vector<std::string>& joint_names,
  double dt)
{
  // 参数验证
  if (start_joints.size() != end_joints.size())
  {
    throw std::invalid_argument("起始关节和目标关节数量必须相同");
  }
  if (duration <= 0.0)
  {
    throw std::invalid_argument("持续时间必须大于0");
  }
  if (dt <= 0.0 || dt >= duration)
  {
    throw std::invalid_argument("时间步长必须大于0且小于持续时间");
  }
  if (joint_names.size() != start_joints.size())
  {
    throw std::invalid_argument("关节名称数量必须与关节数量相同");
  }

  size_t num_joints = start_joints.size();
  std::vector<double> start_vel_actual = start_vel.empty() ? 
    std::vector<double>(num_joints, 0.0) : start_vel;
  std::vector<double> end_vel_actual = end_vel.empty() ? 
    std::vector<double>(num_joints, 0.0) : end_vel;

  if (start_vel_actual.size() != num_joints || end_vel_actual.size() != num_joints)
  {
    throw std::invalid_argument("速度向量大小必须与关节数量相同");
  }

  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = joint_names;

  // 对每个关节计算三次多项式系数
  // q(t) = a0 + a1*t + a2*t² + a3*t³
  // 约束条件：
  // q(0) = start, q(duration) = end
  // q'(0) = start_vel, q'(duration) = end_vel
  
  for (size_t joint_idx = 0; joint_idx < num_joints; ++joint_idx)
  {
    double q0 = start_joints[joint_idx];
    double qf = end_joints[joint_idx];
    double v0 = start_vel_actual[joint_idx];
    double vf = end_vel_actual[joint_idx];
    double T = duration;

    // 求解系数
    // a0 = q0
    // a1 = v0
    // a2 = (3*(qf - q0) - (2*v0 + vf)*T) / T²
    // a3 = ((vf + v0)*T - 2*(qf - q0)) / T³
    
    double a0 = q0;
    double a1 = v0;
    double a2 = (3.0 * (qf - q0) - (2.0 * v0 + vf) * T) / (T * T);
    double a3 = ((vf + v0) * T - 2.0 * (qf - q0)) / (T * T * T);

    // 生成轨迹点
    int num_points = static_cast<int>(std::ceil(duration / dt)) + 1;
    for (int i = 0; i < num_points; ++i)
    {
      double t = std::min(static_cast<double>(i) * dt, duration);
      double t2 = t * t;
      double t3 = t2 * t;

      // 计算位置
      double position = a0 + a1 * t + a2 * t2 + a3 * t3;
      
      // 计算速度
      double velocity = a1 + 2.0 * a2 * t + 3.0 * a3 * t2;
      
      // 计算加速度
      double acceleration = 2.0 * a2 + 6.0 * a3 * t;

      // 创建或获取轨迹点
      if (i >= static_cast<int>(trajectory.points.size()))
      {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions.resize(num_joints);
        point.velocities.resize(num_joints);
        point.accelerations.resize(num_joints);
        point.time_from_start.sec = static_cast<int>(t);
        point.time_from_start.nanosec = static_cast<uint32_t>((t - static_cast<int>(t)) * 1e9);
        trajectory.points.push_back(point);
      }

      trajectory.points[i].positions[joint_idx] = position;
      trajectory.points[i].velocities[joint_idx] = velocity;
      trajectory.points[i].accelerations[joint_idx] = acceleration;
    }
  }

  // 确保最后一个点的时间是精确的duration
  if (!trajectory.points.empty())
  {
    auto& last_point = trajectory.points.back();
    last_point.time_from_start.sec = static_cast<int>(duration);
    last_point.time_from_start.nanosec = static_cast<uint32_t>((duration - static_cast<int>(duration)) * 1e9);
  }

  return trajectory;
}

trajectory_msgs::msg::JointTrajectory TrajectoryPlanner::generateQuinticPolynomialTrajectory(
  const std::vector<double>& start_joints,
  const std::vector<double>& end_joints,
  const std::vector<double>& start_vel,
  const std::vector<double>& end_vel,
  const std::vector<double>& start_acc,
  const std::vector<double>& end_acc,
  double duration,
  const std::vector<std::string>& joint_names,
  double dt)
{
  // 参数验证
  if (start_joints.size() != end_joints.size())
  {
    throw std::invalid_argument("起始关节和目标关节数量必须相同");
  }
  if (duration <= 0.0)
  {
    throw std::invalid_argument("持续时间必须大于0");
  }
  if (dt <= 0.0 || dt >= duration)
  {
    throw std::invalid_argument("时间步长必须大于0且小于持续时间");
  }
  if (joint_names.size() != start_joints.size())
  {
    throw std::invalid_argument("关节名称数量必须与关节数量相同");
  }

  size_t num_joints = start_joints.size();
  if (start_vel.size() != num_joints || end_vel.size() != num_joints ||
      start_acc.size() != num_joints || end_acc.size() != num_joints)
  {
    throw std::invalid_argument("速度/加速度向量大小必须与关节数量相同");
  }

  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = joint_names;

  // 对每个关节计算五次多项式系数
  // q(t) = a0 + a1*t + a2*t² + a3*t³ + a4*t⁴ + a5*t⁵
  // 约束条件：
  // q(0) = start, q(duration) = end
  // q'(0) = start_vel, q'(duration) = end_vel
  // q''(0) = start_acc, q''(duration) = end_acc

  for (size_t joint_idx = 0; joint_idx < num_joints; ++joint_idx)
  {
    double q0 = start_joints[joint_idx];
    double qf = end_joints[joint_idx];
    double v0 = start_vel[joint_idx];
    double vf = end_vel[joint_idx];
    double a0 = start_acc[joint_idx];
    double af = end_acc[joint_idx];
    double T = duration;
    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;

    // 求解系数（使用矩阵求逆或直接公式）
    // a0 = q0
    // a1 = v0
    // a2 = a0 / 2
    // a3 = (20*(qf - q0) - (8*vf + 12*v0)*T - (3*a0 - af)*T²) / (2*T³)
    // a4 = (30*(q0 - qf) + (14*vf + 16*v0)*T + (3*a0 - 2*af)*T²) / T⁴
    // a5 = (12*(qf - q0) - 6*(vf + v0)*T - (af - a0)*T²) / T⁵

    double a0_coeff = q0;
    double a1_coeff = v0;
    double a2_coeff = a0 / 2.0;
    double a3_coeff = (20.0 * (qf - q0) - (8.0 * vf + 12.0 * v0) * T - (3.0 * a0 - af) * T2) / (2.0 * T3);
    double a4_coeff = (30.0 * (q0 - qf) + (14.0 * vf + 16.0 * v0) * T + (3.0 * a0 - 2.0 * af) * T2) / T4;
    double a5_coeff = (12.0 * (qf - q0) - 6.0 * (vf + v0) * T - (af - a0) * T2) / T5;

    // 生成轨迹点
    int num_points = static_cast<int>(std::ceil(duration / dt)) + 1;
    for (int i = 0; i < num_points; ++i)
    {
      double t = std::min(static_cast<double>(i) * dt, duration);
      double t2 = t * t;
      double t3 = t2 * t;
      double t4 = t3 * t;
      double t5 = t4 * t;

      // 计算位置
      double position = a0_coeff + a1_coeff * t + a2_coeff * t2 + 
                       a3_coeff * t3 + a4_coeff * t4 + a5_coeff * t5;
      
      // 计算速度
      double velocity = a1_coeff + 2.0 * a2_coeff * t + 
                       3.0 * a3_coeff * t2 + 4.0 * a4_coeff * t3 + 5.0 * a5_coeff * t4;
      
      // 计算加速度
      double acceleration = 2.0 * a2_coeff + 6.0 * a3_coeff * t + 
                          12.0 * a4_coeff * t2 + 20.0 * a5_coeff * t3;

      // 创建或获取轨迹点
      if (i >= static_cast<int>(trajectory.points.size()))
      {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions.resize(num_joints);
        point.velocities.resize(num_joints);
        point.accelerations.resize(num_joints);
        point.time_from_start.sec = static_cast<int>(t);
        point.time_from_start.nanosec = static_cast<uint32_t>((t - static_cast<int>(t)) * 1e9);
        trajectory.points.push_back(point);
      }

      trajectory.points[i].positions[joint_idx] = position;
      trajectory.points[i].velocities[joint_idx] = velocity;
      trajectory.points[i].accelerations[joint_idx] = acceleration;
    }
  }

  // 确保最后一个点的时间是精确的duration
  if (!trajectory.points.empty())
  {
    auto& last_point = trajectory.points.back();
    last_point.time_from_start.sec = static_cast<int>(duration);
    last_point.time_from_start.nanosec = static_cast<uint32_t>((duration - static_cast<int>(duration)) * 1e9);
  }

  return trajectory;
}

std::vector<geometry_msgs::msg::Pose> TrajectoryPlanner::generateLinearCartesianTrajectory(
  const geometry_msgs::msg::Pose& start_pose,
  const geometry_msgs::msg::Pose& end_pose,
  double duration,
  double dt)
{
  // 参数验证
  if (duration <= 0.0)
  {
    throw std::invalid_argument("持续时间必须大于0");
  }
  if (dt <= 0.0 || dt >= duration)
  {
    throw std::invalid_argument("时间步长必须大于0且小于持续时间");
  }

  std::vector<geometry_msgs::msg::Pose> trajectory;
  
  int num_points = static_cast<int>(std::ceil(duration / dt)) + 1;
  trajectory.reserve(num_points);

  // 位置线性插补
  double x0 = start_pose.position.x;
  double y0 = start_pose.position.y;
  double z0 = start_pose.position.z;
  double xf = end_pose.position.x;
  double yf = end_pose.position.y;
  double zf = end_pose.position.z;

  // 姿态SLERP插补
  geometry_msgs::msg::Quaternion q0 = normalizeQuaternion(start_pose.orientation);
  geometry_msgs::msg::Quaternion qf = normalizeQuaternion(end_pose.orientation);

  for (int i = 0; i < num_points; ++i)
  {
    double t = std::min(static_cast<double>(i) * dt, duration);
    double s = t / duration;  // 归一化参数 [0, 1]

    geometry_msgs::msg::Pose pose;
    
    // 位置线性插补
    pose.position.x = x0 + (xf - x0) * s;
    pose.position.y = y0 + (yf - y0) * s;
    pose.position.z = z0 + (zf - z0) * s;
    
    // 姿态SLERP插补
    pose.orientation = slerp(q0, qf, s);

    trajectory.push_back(pose);
  }

  return trajectory;
}

trajectory_msgs::msg::JointTrajectory TrajectoryPlanner::generateBSplineJointTrajectory(
  const std::vector<std::vector<double>>& control_points,
  int degree,
  double duration,
  const std::vector<std::string>& joint_names,
  double dt)
{
  // 参数验证
  if (control_points.empty())
  {
    throw std::invalid_argument("控制点不能为空");
  }
  if (degree < 1 || degree > 4)
  {
    throw std::invalid_argument("B样条阶数必须在1到4之间");
  }
  if (static_cast<int>(control_points.size()) < degree + 1)
  {
    throw std::invalid_argument("控制点数量必须至少为阶数+1");
  }
  if (duration <= 0.0)
  {
    throw std::invalid_argument("持续时间必须大于0");
  }
  if (dt <= 0.0 || dt >= duration)
  {
    throw std::invalid_argument("时间步长必须大于0且小于持续时间");
  }

  size_t num_joints = control_points[0].size();
  for (const auto& point : control_points)
  {
    if (point.size() != num_joints)
    {
      throw std::invalid_argument("所有控制点的关节数量必须相同");
    }
  }
  if (joint_names.size() != num_joints)
  {
    throw std::invalid_argument("关节名称数量必须与关节数量相同");
  }

  // 生成节点向量
  std::vector<double> knots = generateKnotVector(control_points.size(), degree);

  trajectory_msgs::msg::JointTrajectory trajectory;
  trajectory.joint_names = joint_names;

  int num_points = static_cast<int>(std::ceil(duration / dt)) + 1;
  
  // 初始化轨迹点
  for (int i = 0; i < num_points; ++i)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.resize(num_joints);
    point.velocities.resize(num_joints);
    point.accelerations.resize(num_joints);
    double t = std::min(static_cast<double>(i) * dt, duration);
    double u = t / duration;  // 归一化到[0, 1]
    
    // 将u映射到节点向量的有效范围 [knots[degree], knots[control_points.size()]]
    double u_min = knots[degree];
    double u_max = knots[control_points.size()];
    double u_mapped = u_min + u * (u_max - u_min);
    
    point.time_from_start.sec = static_cast<int>(t);
    point.time_from_start.nanosec = static_cast<uint32_t>((t - static_cast<int>(t)) * 1e9);
    trajectory.points.push_back(point);

    // 对每个关节计算B样条插值
    for (size_t joint_idx = 0; joint_idx < num_joints; ++joint_idx)
    {
      double position = 0.0;
      double velocity = 0.0;
      double acceleration = 0.0;

      // 计算B样条插值
      for (size_t i = 0; i < control_points.size(); ++i)
      {
        double basis = bsplineBasis(i, degree, u_mapped, knots);
        position += control_points[i][joint_idx] * basis;
      }

      // 计算速度（数值微分，使用小步长）
      double h = 1e-6;
      double u_plus = std::min(u_mapped + h, u_max);
      double u_minus = std::max(u_mapped - h, u_min);
      
      double pos_plus = 0.0;
      double pos_minus = 0.0;
      for (size_t i = 0; i < control_points.size(); ++i)
      {
        pos_plus += control_points[i][joint_idx] * bsplineBasis(i, degree, u_plus, knots);
        pos_minus += control_points[i][joint_idx] * bsplineBasis(i, degree, u_minus, knots);
      }
      velocity = (pos_plus - pos_minus) / (2.0 * h) * (u_max - u_min) / duration;

      // 计算加速度
      double pos_center = position;
      double pos_plus2 = 0.0;
      double pos_minus2 = 0.0;
      for (size_t i = 0; i < control_points.size(); ++i)
      {
        pos_plus2 += control_points[i][joint_idx] * bsplineBasis(i, degree, u_plus + h, knots);
        pos_minus2 += control_points[i][joint_idx] * bsplineBasis(i, degree, u_minus - h, knots);
      }
      acceleration = (pos_plus2 - 2.0 * pos_center + pos_minus2) / (h * h) * 
                     ((u_max - u_min) / duration) * ((u_max - u_min) / duration);

      trajectory.points[i].positions[joint_idx] = position;
      trajectory.points[i].velocities[joint_idx] = velocity;
      trajectory.points[i].accelerations[joint_idx] = acceleration;
    }
  }

  // 确保最后一个点的时间是精确的duration
  if (!trajectory.points.empty())
  {
    auto& last_point = trajectory.points.back();
    last_point.time_from_start.sec = static_cast<int>(duration);
    last_point.time_from_start.nanosec = static_cast<uint32_t>((duration - static_cast<int>(duration)) * 1e9);
  }

  return trajectory;
}

std::vector<geometry_msgs::msg::Pose> TrajectoryPlanner::generateBSplineCartesianTrajectory(
  const std::vector<geometry_msgs::msg::Pose>& control_points,
  int degree,
  double duration,
  double dt)
{
  // 参数验证
  if (control_points.empty())
  {
    throw std::invalid_argument("控制点不能为空");
  }
  if (degree < 1 || degree > 4)
  {
    throw std::invalid_argument("B样条阶数必须在1到4之间");
  }
  if (static_cast<int>(control_points.size()) < degree + 1)
  {
    throw std::invalid_argument("控制点数量必须至少为阶数+1");
  }
  if (duration <= 0.0)
  {
    throw std::invalid_argument("持续时间必须大于0");
  }
  if (dt <= 0.0 || dt >= duration)
  {
    throw std::invalid_argument("时间步长必须大于0且小于持续时间");
  }

  // 生成节点向量
  std::vector<double> knots = generateKnotVector(control_points.size(), degree);

  std::vector<geometry_msgs::msg::Pose> trajectory;
  int num_points = static_cast<int>(std::ceil(duration / dt)) + 1;
  trajectory.reserve(num_points);

  double u_min = knots[degree];
  double u_max = knots[control_points.size()];

  for (int i = 0; i < num_points; ++i)
  {
    double t = std::min(static_cast<double>(i) * dt, duration);
    double u = t / duration;  // 归一化到[0, 1]
    double u_mapped = u_min + u * (u_max - u_min);

    geometry_msgs::msg::Pose pose;
    
    // 位置B样条插值
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    
    for (size_t j = 0; j < control_points.size(); ++j)
    {
      double basis = bsplineBasis(j, degree, u_mapped, knots);
      pose.position.x += control_points[j].position.x * basis;
      pose.position.y += control_points[j].position.y * basis;
      pose.position.z += control_points[j].position.z * basis;
    }

    // 姿态使用SLERP（在相邻控制点之间）
    // 简化实现：找到最接近的控制点，使用SLERP
    // 更精确的方法是对四元数进行B样条插值，但这里使用简化版本
    double u_normalized = (u_mapped - u_min) / (u_max - u_min);
    int segment_idx = static_cast<int>(u_normalized * (control_points.size() - 1));
    segment_idx = std::min(segment_idx, static_cast<int>(control_points.size() - 2));
    segment_idx = std::max(segment_idx, 0);
    
    double segment_t = (u_normalized * (control_points.size() - 1)) - segment_idx;
    segment_t = std::max(0.0, std::min(1.0, segment_t));
    
    geometry_msgs::msg::Quaternion q0 = normalizeQuaternion(control_points[segment_idx].orientation);
    geometry_msgs::msg::Quaternion q1 = normalizeQuaternion(control_points[segment_idx + 1].orientation);
    pose.orientation = slerp(q0, q1, segment_t);

    trajectory.push_back(pose);
  }

  return trajectory;
}

double TrajectoryPlanner::bsplineBasis(int i, int k, double t, const std::vector<double>& knots)
{
  // Cox-de Boor递归算法
  if (k == 0)
  {
    // 零阶基函数（分段常数）
    if (t >= knots[i] && t < knots[i + 1])
    {
      return 1.0;
    }
    return 0.0;
  }

  // 处理边界情况
  if (i + k >= static_cast<int>(knots.size()))
  {
    return 0.0;
  }

  double denom1 = knots[i + k] - knots[i];
  double denom2 = knots[i + k + 1] - knots[i + 1];

  double term1 = 0.0;
  if (denom1 > 1e-10)  // 避免除零
  {
    term1 = ((t - knots[i]) / denom1) * bsplineBasis(i, k - 1, t, knots);
  }

  double term2 = 0.0;
  if (denom2 > 1e-10)  // 避免除零
  {
    term2 = ((knots[i + k + 1] - t) / denom2) * bsplineBasis(i + 1, k - 1, t, knots);
  }

  return term1 + term2;
}

std::vector<double> TrajectoryPlanner::generateKnotVector(int num_control_points, int degree)
{
  // 生成均匀节点向量（clamped uniform knot vector）
  // 对于n+1个控制点和k阶B样条，需要n+k+2个节点
  int num_knots = num_control_points + degree + 1;
  std::vector<double> knots(num_knots);

  // 前degree+1个节点为0（clamped）
  for (int i = 0; i <= degree; ++i)
  {
    knots[i] = 0.0;
  }

  // 中间节点均匀分布
  for (int i = degree + 1; i < num_control_points; ++i)
  {
    knots[i] = static_cast<double>(i - degree) / (num_control_points - degree);
  }

  // 后degree+1个节点为1（clamped）
  for (int i = num_control_points; i < num_knots; ++i)
  {
    knots[i] = 1.0;
  }

  return knots;
}

geometry_msgs::msg::Quaternion TrajectoryPlanner::slerp(
  const geometry_msgs::msg::Quaternion& q1,
  const geometry_msgs::msg::Quaternion& q2,
  double t)
{
  // 确保t在[0, 1]范围内
  t = std::max(0.0, std::min(1.0, t));

  // 计算点积（四元数内积）
  double dot = q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;

  // 如果点积为负，取反q2以确保走最短路径
  geometry_msgs::msg::Quaternion q2_adj = q2;
  if (dot < 0.0)
  {
    q2_adj.w = -q2.w;
    q2_adj.x = -q2.x;
    q2_adj.y = -q2.y;
    q2_adj.z = -q2.z;
    dot = -dot;
  }

  // 如果点积接近1，四元数几乎相同，使用线性插值避免数值问题
  const double threshold = 0.9995;
  if (dot > threshold)
  {
    geometry_msgs::msg::Quaternion result;
    result.w = q1.w + t * (q2_adj.w - q1.w);
    result.x = q1.x + t * (q2_adj.x - q1.x);
    result.y = q1.y + t * (q2_adj.y - q1.y);
    result.z = q1.z + t * (q2_adj.z - q1.z);
    return normalizeQuaternion(result);
  }

  // 标准SLERP公式
  double theta = std::acos(dot);
  double sin_theta = std::sin(theta);
  double w1 = std::sin((1.0 - t) * theta) / sin_theta;
  double w2 = std::sin(t * theta) / sin_theta;

  geometry_msgs::msg::Quaternion result;
  result.w = w1 * q1.w + w2 * q2_adj.w;
  result.x = w1 * q1.x + w2 * q2_adj.x;
  result.y = w1 * q1.y + w2 * q2_adj.y;
  result.z = w1 * q1.z + w2 * q2_adj.z;

  return normalizeQuaternion(result);
}

geometry_msgs::msg::Quaternion TrajectoryPlanner::normalizeQuaternion(
  const geometry_msgs::msg::Quaternion& q)
{
  double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  
  if (norm < 1e-10)
  {
    // 如果四元数接近零，返回单位四元数
    geometry_msgs::msg::Quaternion result;
    result.w = 1.0;
    result.x = 0.0;
    result.y = 0.0;
    result.z = 0.0;
    return result;
  }

  geometry_msgs::msg::Quaternion result;
  result.w = q.w / norm;
  result.x = q.x / norm;
  result.y = q.y / norm;
  result.z = q.z / norm;
  return result;
}

bool TrajectoryPlanner::applyTimeParameterization(
  trajectory_msgs::msg::JointTrajectory& trajectory,
  const moveit::core::RobotModelConstPtr& robot_model,
  const std::string& group_name,
  double max_velocity_scaling,
  double max_acceleration_scaling,
  TimeParameterizationType algorithm)
{
  if (!robot_model)
  {
    return false;
  }

  if (trajectory.points.empty())
  {
    return false;
  }

  // 获取关节模型组
  const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(group_name);
  if (!jmg)
  {
    return false;
  }

  // 创建RobotTrajectory对象
  robot_trajectory::RobotTrajectory robot_traj(robot_model, group_name);

  // 从JointTrajectory消息填充RobotTrajectory
  moveit::core::RobotState state(robot_model);
  state.setToDefaultValues();

  for (const auto& point : trajectory.points)
  {
    // 设置关节位置
    for (size_t i = 0; i < trajectory.joint_names.size() && i < point.positions.size(); ++i)
    {
      state.setJointPositions(trajectory.joint_names[i], &point.positions[i]);
    }
    
    // 计算时间（从time_from_start）
    double time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
    robot_traj.addSuffixWayPoint(state, time);
  }

  // 应用时间参数化
  bool success = applyTimeParameterization(robot_traj, max_velocity_scaling, 
                                           max_acceleration_scaling, algorithm);
  
  if (!success)
  {
    return false;
  }

  // 从RobotTrajectory转换回JointTrajectory消息
  trajectory.points.clear();
  for (size_t i = 0; i < robot_traj.getWayPointCount(); ++i)
  {
    const moveit::core::RobotState& wp = robot_traj.getWayPoint(i);
    double time = robot_traj.getWayPointDurationFromStart(i);

    trajectory_msgs::msg::JointTrajectoryPoint point;
    
    // 获取位置、速度、加速度
    for (const auto& joint_name : trajectory.joint_names)
    {
      const double* pos = wp.getJointPositions(joint_name);
      const double* vel = wp.getJointVelocities(joint_name);
      const double* acc = wp.getJointAccelerations(joint_name);
      
      if (pos) point.positions.push_back(*pos);
      if (vel) point.velocities.push_back(*vel);
      if (acc) point.accelerations.push_back(*acc);
    }

    // 设置时间戳
    point.time_from_start.sec = static_cast<int>(time);
    point.time_from_start.nanosec = static_cast<uint32_t>((time - static_cast<int>(time)) * 1e9);

    trajectory.points.push_back(point);
  }

  return true;
}

bool TrajectoryPlanner::applyTimeParameterization(
  robot_trajectory::RobotTrajectory& robot_trajectory,
  double max_velocity_scaling,
  double max_acceleration_scaling,
  TimeParameterizationType algorithm)
{
  if (robot_trajectory.getWayPointCount() == 0)
  {
    return false;
  }

  // 限制缩放因子在有效范围内
  max_velocity_scaling = std::max(0.01, std::min(1.0, max_velocity_scaling));
  max_acceleration_scaling = std::max(0.01, std::min(1.0, max_acceleration_scaling));

  bool success = false;

  switch (algorithm)
  {
    case TimeParameterizationType::TOTG:
    {
      // 时间最优轨迹生成（Time-Optimal Trajectory Generation）
      // 生成时间最优但可能会轻微偏离原始路径的轨迹
      trajectory_processing::TimeOptimalTrajectoryGeneration totg;
      success = totg.computeTimeStamps(robot_trajectory, max_velocity_scaling, max_acceleration_scaling);
      break;
    }
    
    case TimeParameterizationType::ITERATIVE_SPLINE:
    {
      // 迭代样条参数化（Iterative Spline Parameterization）
      // 使用三次样条拟合，保证位置/速度/加速度连续，更平滑
      // 参数：add_points=true 在起点和终点添加额外点以更好地满足边界条件
      trajectory_processing::IterativeSplineParameterization isp(true);
      success = isp.computeTimeStamps(robot_trajectory, max_velocity_scaling, max_acceleration_scaling);
      break;
    }
    
    case TimeParameterizationType::ITERATIVE_PARABOLIC:
    {
      // 迭代抛物线时间参数化（Iterative Parabolic Time Parameterization）
      // 使用抛物线混合（梯形速度曲线），适合简单运动
      trajectory_processing::IterativeParabolicTimeParameterization iptp;
      success = iptp.computeTimeStamps(robot_trajectory, max_velocity_scaling, max_acceleration_scaling);
      break;
    }
    
    default:
      // 默认使用迭代样条
      trajectory_processing::IterativeSplineParameterization isp_default(true);
      success = isp_default.computeTimeStamps(robot_trajectory, max_velocity_scaling, max_acceleration_scaling);
      break;
  }

  return success;
}

bool TrajectoryPlanner::resampleTrajectory(
  trajectory_msgs::msg::JointTrajectory& trajectory,
  double control_frequency)
{
  if (trajectory.points.empty() || control_frequency <= 0.0)
  {
    return false;
  }

  // 计算采样周期
  double dt = 1.0 / control_frequency;

  // 获取轨迹总时长
  const auto& last_point = trajectory.points.back();
  double total_duration = last_point.time_from_start.sec + 
                          last_point.time_from_start.nanosec * 1e-9;

  if (total_duration <= 0.0)
  {
    return false;
  }

  // 创建新的轨迹点列表
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> new_points;
  size_t num_joints = trajectory.joint_names.size();

  // 按固定时间间隔重采样
  for (double t = 0.0; t <= total_duration + 1e-6; t += dt)
  {
    // 找到t所在的区间 [points[i], points[i+1]]
    size_t idx = 0;
    for (size_t i = 0; i < trajectory.points.size() - 1; ++i)
    {
      double t_i = trajectory.points[i].time_from_start.sec + 
                   trajectory.points[i].time_from_start.nanosec * 1e-9;
      double t_next = trajectory.points[i + 1].time_from_start.sec + 
                      trajectory.points[i + 1].time_from_start.nanosec * 1e-9;
      if (t >= t_i && t <= t_next)
      {
        idx = i;
        break;
      }
      if (i == trajectory.points.size() - 2)
      {
        idx = i;  // 最后一个区间
      }
    }

    // 获取区间端点
    const auto& p0 = trajectory.points[idx];
    const auto& p1 = trajectory.points[std::min(idx + 1, trajectory.points.size() - 1)];
    
    double t0 = p0.time_from_start.sec + p0.time_from_start.nanosec * 1e-9;
    double t1 = p1.time_from_start.sec + p1.time_from_start.nanosec * 1e-9;

    // 计算插值参数
    double alpha = 0.0;
    if (t1 > t0)
    {
      alpha = (t - t0) / (t1 - t0);
      alpha = std::max(0.0, std::min(1.0, alpha));
    }

    // 创建新的轨迹点
    trajectory_msgs::msg::JointTrajectoryPoint new_point;
    new_point.positions.resize(num_joints);
    new_point.velocities.resize(num_joints);
    new_point.accelerations.resize(num_joints);

    // 对每个关节进行插值
    for (size_t j = 0; j < num_joints; ++j)
    {
      // 位置线性插值
      double pos0 = (j < p0.positions.size()) ? p0.positions[j] : 0.0;
      double pos1 = (j < p1.positions.size()) ? p1.positions[j] : pos0;
      new_point.positions[j] = pos0 + alpha * (pos1 - pos0);

      // 速度线性插值
      double vel0 = (j < p0.velocities.size()) ? p0.velocities[j] : 0.0;
      double vel1 = (j < p1.velocities.size()) ? p1.velocities[j] : vel0;
      new_point.velocities[j] = vel0 + alpha * (vel1 - vel0);

      // 加速度线性插值
      double acc0 = (j < p0.accelerations.size()) ? p0.accelerations[j] : 0.0;
      double acc1 = (j < p1.accelerations.size()) ? p1.accelerations[j] : acc0;
      new_point.accelerations[j] = acc0 + alpha * (acc1 - acc0);
    }

    // 设置时间戳
    new_point.time_from_start.sec = static_cast<int>(t);
    new_point.time_from_start.nanosec = static_cast<uint32_t>((t - static_cast<int>(t)) * 1e9);

    new_points.push_back(new_point);
  }

  // 确保最后一个点是原始轨迹的终点
  if (!new_points.empty())
  {
    auto& last_new = new_points.back();
    const auto& last_orig = trajectory.points.back();
    
    // 如果最后一个采样点和终点相差较大，添加终点
    double t_last_new = last_new.time_from_start.sec + last_new.time_from_start.nanosec * 1e-9;
    if (total_duration - t_last_new > dt * 0.5)
    {
      new_points.push_back(last_orig);
    }
    else
    {
      // 直接使用原始终点的位置（确保精确到达）
      for (size_t j = 0; j < num_joints && j < last_orig.positions.size(); ++j)
      {
        last_new.positions[j] = last_orig.positions[j];
      }
      // 终点速度设为0
      for (size_t j = 0; j < num_joints; ++j)
      {
        if (j < last_new.velocities.size())
        {
          last_new.velocities[j] = 0.0;
        }
        if (j < last_new.accelerations.size())
        {
          last_new.accelerations[j] = 0.0;
        }
      }
      last_new.time_from_start = last_orig.time_from_start;
    }
  }

  // 替换原轨迹
  trajectory.points = std::move(new_points);

  return true;
}

}  // namespace m5_grasp
