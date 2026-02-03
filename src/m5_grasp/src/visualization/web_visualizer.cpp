#include "m5_grasp/visualization/web_visualizer.hpp"
#include "m5_grasp/logging/logger.hpp"
#include <chrono>

namespace m5_grasp {

WebVisualizer::WebVisualizer(rclcpp::Node* node, double heartbeat_rate_hz)
    : node_(node), heartbeat_rate_hz_(heartbeat_rate_hz) {
  
  // 创建发布器
  planned_path_pub_ = node_->create_publisher<std_msgs::msg::String>(
      "/visualization/planned_path", 10);
  
  ik_status_pub_ = node_->create_publisher<std_msgs::msg::String>(
      "/visualization/ik_status", 10);
  
  // 心跳发布器 - 仅一个主控心跳，供网页判断 m5_grasp 是否在线
  heartbeat_m5_grasp_pub_ = node_->create_publisher<std_msgs::msg::String>(
      "/heartbeat/m5_grasp", 10);
  
  LOG_NAMED_INFO("web_viz", "WebVisualizer已初始化");
  LOG_NAMED_INFO("web_viz", "  发布话题: /visualization/planned_path");
  LOG_NAMED_INFO("web_viz", "  发布话题: /visualization/ik_status");
  LOG_NAMED_INFO("web_viz", "  发布话题: /heartbeat/m5_grasp ({:.1f} Hz)", heartbeat_rate_hz_);
}

void WebVisualizer::startHeartbeat() {
  if (heartbeat_timer_) {
    LOG_NAMED_WARN("web_viz", "心跳定时器已在运行");
    return;
  }
  
  auto period = std::chrono::milliseconds(
      static_cast<int>(1000.0 / heartbeat_rate_hz_));
  
  heartbeat_timer_ = node_->create_wall_timer(
      period,
      std::bind(&WebVisualizer::heartbeatCallback, this));
  
  LOG_NAMED_INFO("web_viz", "心跳定时器已启动 ({:.1f} Hz)", heartbeat_rate_hz_);
}

void WebVisualizer::stopHeartbeat() {
  if (heartbeat_timer_) {
    heartbeat_timer_->cancel();
    heartbeat_timer_.reset();
    LOG_NAMED_INFO("web_viz", "心跳定时器已停止");
  }
}

void WebVisualizer::heartbeatCallback() {
  publishHeartbeat();
}

void WebVisualizer::publishHeartbeat() {
  std_msgs::msg::String msg;
  msg.data = "alive";
  if (heartbeat_m5_grasp_pub_) {
    heartbeat_m5_grasp_pub_->publish(msg);
  }
}

void WebVisualizer::publishPlannedPath(const std::vector<geometry_msgs::msg::Pose>& waypoints) {
  if (!planned_path_pub_ || waypoints.empty()) return;
  
  std::stringstream ss;
  ss << "[";
  for (size_t i = 0; i < waypoints.size(); ++i) {
    if (i > 0) ss << ",";
    ss << "{\"x\":" << waypoints[i].position.x 
       << ",\"y\":" << waypoints[i].position.y 
       << ",\"z\":" << waypoints[i].position.z << "}";
  }
  ss << "]";
  
  std_msgs::msg::String msg;
  msg.data = ss.str();
  planned_path_pub_->publish(msg);
  
  LOG_NAMED_DEBUG("web_viz", "发布规划路径: {} 个点", waypoints.size());
}

void WebVisualizer::publishPlannedPathFromTrajectory(
    const trajectory_msgs::msg::JointTrajectory& trajectory,
    const moveit::core::RobotStatePtr& robot_state,
    const std::string& eef_link,
    size_t max_points) {
  
  if (!planned_path_pub_ || !robot_state || trajectory.points.empty()) return;
  
  std::vector<geometry_msgs::msg::Pose> waypoints;
  const auto& joint_model_group = robot_state->getJointModelGroup("arm_group");
  if (!joint_model_group) {
    LOG_NAMED_WARN("web_viz", "无法获取arm_group，跳过轨迹发布");
    return;
  }
  
  // 采样轨迹点
  size_t step = std::max(size_t(1), trajectory.points.size() / max_points);
  
  for (size_t i = 0; i < trajectory.points.size(); i += step) {
    const auto& point = trajectory.points[i];
    
    // 设置关节位置
    for (size_t j = 0; j < trajectory.joint_names.size() && j < point.positions.size(); ++j) {
      robot_state->setJointPositions(trajectory.joint_names[j], &point.positions[j]);
    }
    robot_state->update();
    
    // 获取末端位姿
    const Eigen::Isometry3d& eef_pose = robot_state->getGlobalLinkTransform(eef_link);
    geometry_msgs::msg::Pose pose;
    pose.position.x = eef_pose.translation().x();
    pose.position.y = eef_pose.translation().y();
    pose.position.z = eef_pose.translation().z();
    waypoints.push_back(pose);
  }
  
  // 确保包含最后一个点
  if (trajectory.points.size() > 1 && (trajectory.points.size() - 1) % step != 0) {
    const auto& last_point = trajectory.points.back();
    for (size_t j = 0; j < trajectory.joint_names.size() && j < last_point.positions.size(); ++j) {
      robot_state->setJointPositions(trajectory.joint_names[j], &last_point.positions[j]);
    }
    robot_state->update();
    
    const Eigen::Isometry3d& eef_pose = robot_state->getGlobalLinkTransform(eef_link);
    geometry_msgs::msg::Pose pose;
    pose.position.x = eef_pose.translation().x();
    pose.position.y = eef_pose.translation().y();
    pose.position.z = eef_pose.translation().z();
    waypoints.push_back(pose);
  }
  
  publishPlannedPath(waypoints);
}

void WebVisualizer::publishIKStatus(bool reachable, double confidence, const std::string& message) {
  if (!ik_status_pub_) return;
  
  // 检查是否与上次状态相同（避免重复发布）
  if (message == last_ik_message_ && reachable == last_ik_reachable_) {
    return;  // 状态未变化，不发布
  }
  
  // 更新缓存
  last_ik_message_ = message;
  last_ik_reachable_ = reachable;
  
  std::stringstream ss;
  ss << "{\"reachable\":" << (reachable ? "true" : "false")
     << ",\"confidence\":" << confidence;
  if (!message.empty()) {
    ss << ",\"message\":\"" << escapeJsonString(message) << "\"";
  }
  ss << "}";
  
  std_msgs::msg::String msg;
  msg.data = ss.str();
  ik_status_pub_->publish(msg);
  
  LOG_NAMED_DEBUG("web_viz", "发布IK状态: reachable={}, confidence={:.2f}, msg={}", 
                  reachable, confidence, message);
}

void WebVisualizer::clearPlannedPath() {
  if (!planned_path_pub_) return;
  
  std_msgs::msg::String msg;
  msg.data = "[]";
  planned_path_pub_->publish(msg);
  
  LOG_NAMED_DEBUG("web_viz", "清除规划路径显示");
}

size_t WebVisualizer::getHeartbeatSubscriberCount() const {
  return heartbeat_m5_grasp_pub_ ? heartbeat_m5_grasp_pub_->get_subscription_count() : 0;
}

size_t WebVisualizer::getPlannedPathSubscriberCount() const {
  return planned_path_pub_ ? planned_path_pub_->get_subscription_count() : 0;
}

size_t WebVisualizer::getIKStatusSubscriberCount() const {
  return ik_status_pub_ ? ik_status_pub_->get_subscription_count() : 0;
}

std::string WebVisualizer::escapeJsonString(const std::string& input) {
  std::stringstream ss;
  for (char c : input) {
    switch (c) {
      case '"': ss << "\\\""; break;
      case '\\': ss << "\\\\"; break;
      case '\b': ss << "\\b"; break;
      case '\f': ss << "\\f"; break;
      case '\n': ss << "\\n"; break;
      case '\r': ss << "\\r"; break;
      case '\t': ss << "\\t"; break;
      default: ss << c; break;
    }
  }
  return ss.str();
}

}  // namespace m5_grasp
