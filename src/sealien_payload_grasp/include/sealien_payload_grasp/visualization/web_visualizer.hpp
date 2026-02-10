#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <vector>

namespace sealien_payload_grasp
{

/**
 * @brief 网页3D可视化器 - 发布数据到网页前端
 *
 * 功能：
 * 1. 发布规划路径到 /visualization/planned_path
 * 2. 发布IK状态到 /visualization/ik_status
 * 3. 发布心跳到 /heartbeat/sealien_payload_grasp（主控存活，供网页显示）
 */
class WebVisualizer
{
  public:
    /**
     * @brief 构造函数
     * @param node ROS2节点指针
     * @param heartbeat_rate_hz 心跳频率（默认1Hz）
     */
    explicit WebVisualizer(rclcpp::Node* node, double heartbeat_rate_hz = 1.0);

    /**
     * @brief 析构函数
     */
    ~WebVisualizer() = default;

    /**
     * @brief 发布规划路径（从Pose列表）
     * @param waypoints 路径点列表
     */
    void publishPlannedPath(const std::vector<geometry_msgs::msg::Pose>& waypoints);

    /**
     * @brief 发布规划路径（从轨迹消息，需要FK计算）
     * @param trajectory 关节轨迹
     * @param robot_state 机器人状态（用于FK）
     * @param eef_link 末端执行器link名称
     * @param max_points 最大采样点数（默认50）
     */
    void publish_planned_path_from_trajectory(const trajectory_msgs::msg::JointTrajectory& trajectory,
                                          const moveit::core::RobotStatePtr& robot_state,
                                          const std::string& eef_link, size_t max_points = 50);

    /**
     * @brief 发布IK状态
     * @param reachable 是否可达
     * @param confidence 置信度 (0.0 - 1.0)
     * @param message 附加消息
     */
    void publish_ik_status(bool reachable, double confidence, const std::string& message = "");

    /**
     * @brief 清除规划路径显示
     */
    void clearPlannedPath();

    /**
     * @brief 启动心跳（如果使用定时器方式）
     */
    void start_heartbeat();

    /**
     * @brief 停止心跳
     */
    void stopHeartbeat();

    /**
     * @brief 手动发布一次心跳
     */
    void publishHeartbeat();

    /**
     * @brief 获取心跳发布器订阅者数量
     */
    size_t getHeartbeatSubscriberCount() const;

    /**
     * @brief 获取规划路径发布器订阅者数量
     */
    size_t getPlannedPathSubscriberCount() const;

    /**
     * @brief 获取IK状态发布器订阅者数量
     */
    size_t getIKStatusSubscriberCount() const;

  private:
    rclcpp::Node* node_;

    // 发布器
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr planned_path_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ik_status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr heartbeat_sealien_payload_grasp_pub_;

    // 心跳定时器
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    double heartbeat_rate_hz_;

    // IK状态缓存（避免重复发布相同状态）
    std::string last_ik_message_;
    bool last_ik_reachable_{false};

    // 心跳回调
    void heartbeat_callback();

    // JSON转义字符串
    static std::string escapeJsonString(const std::string& input);
};

} // namespace sealien_payload_grasp
