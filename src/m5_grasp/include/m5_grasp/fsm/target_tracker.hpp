#pragma once

#include <deque>
#include <mutex>
#include <optional>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <m5_msgs/msg/cable_pose_with_yaw.hpp>

namespace m5_grasp {

/**
 * @brief 目标位置跟踪器 - 负责目标滤波和稳定性判定
 * 
 * 职责：
 * - 维护目标滑窗
 * - 判断目标是否稳定
 * - 判断目标是否过期
 * - 判断目标是否跳变
 */
class TargetTracker {
public:
  /**
   * @brief 配置参数
   */
  struct Config {
    size_t window_size;           // 滑窗大小
    double stale_timeout;         // 过期超时 (s)
    double stable_pos_threshold;  // 位置稳定阈值 (m)
    double stable_yaw_threshold;  // yaw稳定阈值 (rad, ~5°)
    double jump_pos_threshold;    // 位置跳变阈值 (m)
    double jump_yaw_threshold;    // yaw跳变阈值 (rad, ~10°)
    
    Config()
      : window_size(5)
      , stale_timeout(0.5)
      , stable_pos_threshold(0.01)
      , stable_yaw_threshold(0.0873)
      , jump_pos_threshold(0.03)
      , jump_yaw_threshold(0.1745)
    {}
  };

  /**
   * @brief 默认构造函数
   */
  TargetTracker();
  
  /**
   * @brief 带配置的构造函数
   * @param config 配置参数
   */
  explicit TargetTracker(const Config& config);

  /**
   * @brief 更新配置
   */
  void setConfig(const Config& config);

  /**
   * @brief 推入新的目标
   * @param msg 目标消息
   * @param stamp 时间戳
   */
  void push(const m5_msgs::msg::CablePoseWithYaw& msg, const rclcpp::Time& stamp);

  /**
   * @brief 检查目标是否稳定
   * @param now 当前时间
   * @return true 如果目标稳定
   */
  bool hasStableTarget(const rclcpp::Time& now) const;

  /**
   * @brief 获取稳定的目标
   * @return 目标消息，如果没有稳定目标则返回空
   */
  std::optional<m5_msgs::msg::CablePoseWithYaw> getStableTarget() const;

  /**
   * @brief 获取最新的目标（不要求稳定）
   * @return 最新目标，如果没有则返回空
   */
  std::optional<m5_msgs::msg::CablePoseWithYaw> getLatestTarget() const;

  /**
   * @brief 获取最新目标的时间戳
   */
  rclcpp::Time getLatestStamp() const;

  /**
   * @brief 检查目标是否过期
   * @param now 当前时间
   * @return true 如果目标过期
   */
  bool isStale(const rclcpp::Time& now) const;

  /**
   * @brief 检查目标是否相对于参考位置发生跳变
   * @param ref 参考目标
   * @return true 如果发生跳变
   */
  bool isJumping(const m5_msgs::msg::CablePoseWithYaw& ref) const;

  /**
   * @brief 检查目标是否有效（既不过期也没跳变）
   * @param now 当前时间
   * @param ref 参考目标（用于跳变检测）
   * @return true 如果目标有效
   */
  bool isValid(const rclcpp::Time& now, const m5_msgs::msg::CablePoseWithYaw& ref) const;

  /**
   * @brief 清空滑窗
   */
  void clear();

  /**
   * @brief 获取滑窗大小
   */
  size_t windowSize() const;

private:
  /**
   * @brief 角度归一化到 [-pi, pi]
   */
  static double normalizeAngle(double angle);

  mutable std::mutex mutex_;
  Config config_;
  std::deque<m5_msgs::msg::CablePoseWithYaw> window_;
  m5_msgs::msg::CablePoseWithYaw latest_target_;
  rclcpp::Time latest_stamp_;
  bool has_target_{false};
};

}  // namespace m5_grasp
