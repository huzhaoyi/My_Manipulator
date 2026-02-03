#ifndef M5_GRASP_SCENE_MANAGER_HPP
#define M5_GRASP_SCENE_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/buffer.h>
#include <mutex>
#include <string>
#include <vector>
#include <memory>

namespace m5_grasp
{

/**
 * @brief 场景管理器类
 * 
 * 负责管理MoveIt规划场景中的碰撞对象，包括缆绳和地面
 */
class SceneManager
{
public:
  /**
   * @brief 构造函数
   */
  SceneManager(
    rclcpp::Node* node,
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface,
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::mutex& moveit_mutex);

  /**
   * @brief 设置场景参数
   */
  void setParameters(
    const std::string& planning_frame,
    const std::string& eef_link,
    double cable_diameter,
    double cable_length,
    double cable_center_offset_z,
    double ground_height,
    double ground_offset_below_base,
    const std::vector<std::string>& allow_touch_links);

  /**
   * @brief 添加缆绳碰撞体到场景
   * @param cable_pose 缆绳位姿
   * @param cable_yaw 缆绳yaw角（用于确定圆柱体方向）
   * @param object_id 对象ID
   * @return 是否成功
   */
  bool addCableObject(
    const geometry_msgs::msg::PoseStamped& cable_pose,
    double cable_yaw,
    const std::string& object_id = "cable_1");

  /**
   * @brief 添加地面碰撞对象
   * @return 是否成功
   */
  bool addGroundPlane();

  /**
   * @brief 附着物体到末端执行器
   * @param object_id 对象ID
   * @param eef_link 末端执行器link
   * @param cable_pose 缆绳位姿
   * @param cable_yaw 缆绳yaw角
   * @return 是否成功
   */
  bool attachObject(
    const std::string& object_id,
    const std::string& eef_link,
    const geometry_msgs::msg::PoseStamped& cable_pose,
    double cable_yaw);

  /**
   * @brief 移除碰撞对象
   * @param object_id 对象ID
   * @return 是否成功
   */
  bool removeObject(const std::string& object_id);

  /**
   * @brief 分离并移除附着对象
   * @param object_id 对象ID
   * @param eef_link 末端执行器link
   * @return 是否成功
   */
  bool detachAndRemove(const std::string& object_id, const std::string& eef_link);

  /**
   * @brief 检查对象是否已附着
   * @param object_id 对象ID
   * @param eef_link 末端执行器link
   * @return 是否已附着
   */
  bool isObjectAttached(const std::string& object_id, const std::string& eef_link);

  /**
   * @brief 创建缆绳圆柱体的位姿
   * @param cable_pose 缆绳中心位姿（在planning frame中）
   * @param cable_yaw 缆绳yaw角
   * @return 圆柱体位姿
   */
  geometry_msgs::msg::Pose makeCableCylinderPose(
    const geometry_msgs::msg::PoseStamped& cable_pose,
    double cable_yaw);

  /**
   * @brief 转换位姿到planning frame
   * @param pose 输入/输出位姿
   * @return 是否成功
   */
  bool transformPoseToPlanning(geometry_msgs::msg::PoseStamped& pose);

  /**
   * @brief 获取planning frame
   */
  const std::string& getPlanningFrame() const { return planning_frame_; }

private:
  rclcpp::Node* node_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::mutex& moveit_mutex_;

  // 参数
  std::string planning_frame_;
  std::string eef_link_;
  double cable_diameter_{0.008};
  double cable_length_{0.20};
  double cable_center_offset_z_{0.0};
  double ground_height_{0.0};
  double ground_offset_below_base_{-0.05};
  std::vector<std::string> allow_touch_links_;

  // TF fallback计数
  std::atomic<size_t> tf_fallback_count_{0};
};

}  // namespace m5_grasp

#endif  // M5_GRASP_SCENE_MANAGER_HPP
