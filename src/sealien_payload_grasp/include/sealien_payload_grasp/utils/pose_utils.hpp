#ifndef M5_GRASP_POSE_UTILS_HPP
#define M5_GRASP_POSE_UTILS_HPP

#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace sealien_payload_grasp
{

/**
 * @brief 计算向下的姿态（夹爪朝下）
 * @return 向下的四元数姿态
 */
inline geometry_msgs::msg::Quaternion compute_downward_orientation()
{
    // 夹爪向下：Z轴朝下（即世界-Z方向）
    // 这对应于绕X轴旋转180度
    tf2::Quaternion q;
    q.setRPY(M_PI, 0.0, 0.0); // roll=180°, pitch=0°, yaw=0°
    geometry_msgs::msg::Quaternion orientation;
    orientation.x = q.x();
    orientation.y = q.y();
    orientation.z = q.z();
    orientation.w = q.w();
    return orientation;
}

/**
 * @brief 计算带yaw的向下姿态
 * @param yaw 绕Z轴的旋转角度（弧度）
 * @return 向下且带yaw旋转的四元数姿态
 */
inline geometry_msgs::msg::Quaternion compute_downward_orientation_with_yaw(double yaw)
{
    // 先绕X轴旋转180度（使Z轴朝下），然后绕Z轴旋转yaw
    tf2::Quaternion q;
    q.setRPY(M_PI, 0.0, yaw); // roll=180°, pitch=0°, yaw=指定值
    geometry_msgs::msg::Quaternion orientation;
    orientation.x = q.x();
    orientation.y = q.y();
    orientation.z = q.z();
    orientation.w = q.w();
    return orientation;
}

/**
 * @brief 检查是否为默认/无效的四元数姿态
 * @param orientation 要检查的姿态
 * @return 如果是默认姿态返回true
 */
inline bool is_default_orientation(const geometry_msgs::msg::Quaternion& orientation)
{
    // 检查是否是单位四元数或全零（可能表示未设置）
    double norm = std::sqrt(orientation.x * orientation.x + orientation.y * orientation.y +
                            orientation.z * orientation.z + orientation.w * orientation.w);

    // 如果范数接近0，说明是未初始化的姿态
    if (norm < 1e-6)
    {
        return true;
    }

    // 检查是否是单位四元数(0,0,0,1)
    if (std::abs(orientation.x) < 1e-6 && std::abs(orientation.y) < 1e-6 &&
        std::abs(orientation.z) < 1e-6 && std::abs(orientation.w - 1.0) < 1e-6)
    {
        return true;
    }

    return false;
}

/**
 * @brief 从四元数提取yaw角
 * @param orientation 四元数姿态
 * @return yaw角（弧度）
 */
inline double extract_yaw_from_quaternion(const geometry_msgs::msg::Quaternion& orientation)
{
    tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
}

/**
 * @brief 补偿TCP偏移，计算EEF(LinkGG)的目标位姿
 * @param center_pose 夹爪中心的目标位姿
 * @param eef_to_center EEF到夹爪中心的偏移（在EEF坐标系中）
 * @return LinkGG的目标位姿
 */
inline geometry_msgs::msg::PoseStamped
compensate_tcp_offset_for_eef(const geometry_msgs::msg::PoseStamped& center_pose,
                              const tf2::Vector3& eef_to_center)
{
    geometry_msgs::msg::PoseStamped eef_pose = center_pose;

    // 获取目标姿态的旋转矩阵
    tf2::Quaternion q;
    tf2::fromMsg(center_pose.pose.orientation, q);
    tf2::Matrix3x3 R(q);

    // 将EEF坐标系中的偏移转换到世界坐标系
    tf2::Vector3 offset_world = R * eef_to_center;

    // LinkGG位置 = 夹爪中心位置 - 偏移
    // 因为 center = LinkGG + offset，所以 LinkGG = center - offset
    eef_pose.pose.position.x = center_pose.pose.position.x - offset_world.x();
    eef_pose.pose.position.y = center_pose.pose.position.y - offset_world.y();
    eef_pose.pose.position.z = center_pose.pose.position.z - offset_world.z();

    return eef_pose;
}

/**
 * @brief 规范化角度差到[-PI, PI]
 * @param diff 角度差
 * @return 规范化后的角度差
 */
inline double normalize_angle_diff(double diff)
{
    while (diff > M_PI)
        diff -= 2 * M_PI;
    while (diff < -M_PI)
        diff += 2 * M_PI;
    return diff;
}

/**
 * @brief 规范化角度到[-PI, PI]
 * @param angle 角度
 * @return 规范化后的角度
 */
inline double normalize_angle(double angle)
{
    while (angle > M_PI)
        angle -= 2 * M_PI;
    while (angle < -M_PI)
        angle += 2 * M_PI;
    return angle;
}

} // namespace sealien_payload_grasp

#endif // M5_GRASP_POSE_UTILS_HPP
