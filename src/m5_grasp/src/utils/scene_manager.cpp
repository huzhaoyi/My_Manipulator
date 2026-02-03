#include "m5_grasp/logging/logger.hpp"
#include "m5_grasp/utils/scene_manager.hpp"
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/time.h>
#include <thread>
#include <chrono>
#include <cmath>

namespace m5_grasp
{

SceneManager::SceneManager(
    rclcpp::Node* node,
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface,
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::mutex& moveit_mutex)
  : node_(node)
  , planning_scene_interface_(planning_scene_interface)
  , move_group_interface_(move_group_interface)
  , tf_buffer_(tf_buffer)
  , moveit_mutex_(moveit_mutex)
{
}

void SceneManager::setParameters(
    const std::string& planning_frame,
    const std::string& eef_link,
    double cable_diameter,
    double cable_length,
    double cable_center_offset_z,
    double ground_height,
    double ground_offset_below_base,
    const std::vector<std::string>& allow_touch_links)
{
  planning_frame_ = planning_frame;
  eef_link_ = eef_link;
  cable_diameter_ = cable_diameter;
  cable_length_ = cable_length;
  cable_center_offset_z_ = cable_center_offset_z;
  ground_height_ = ground_height;
  ground_offset_below_base_ = ground_offset_below_base;
  allow_touch_links_ = allow_touch_links;
}

bool SceneManager::transformPoseToPlanning(geometry_msgs::msg::PoseStamped& pose)
{
  if (pose.header.frame_id == planning_frame_)
  {
    return true;
  }

  // 优先使用 pose.header.stamp
  if (pose.header.stamp.sec != 0 || pose.header.stamp.nanosec != 0)
  {
    rclcpp::Time transform_time(pose.header.stamp);
    rclcpp::Duration timeout(0, 100000000);  // 100ms
    
    if (tf_buffer_->canTransform(planning_frame_, pose.header.frame_id, transform_time, timeout))
    {
      try
      {
        geometry_msgs::msg::TransformStamped transform = 
            tf_buffer_->lookupTransform(planning_frame_, pose.header.frame_id, transform_time);
        tf2::doTransform(pose, pose, transform);
        pose.header.frame_id = planning_frame_;
        return true;
      }
      catch (const tf2::TransformException& ex)
      {
        LOG_NAMED_DEBUG("scene", 
                     "canTransform通过但lookup失败，fallback到最新TF: {}", ex.what());
      }
    }
  }

  // Fallback：使用 tf2::TimePointZero
  tf_fallback_count_++;
  if (tf_fallback_count_ > 10)
  {
    LOG_THROTTLE_WARN(5000,
                         "TF转换fallback次数过多（{}次）", tf_fallback_count_.load());
  }
  
  try
  {
    std::chrono::nanoseconds timeout_ns(100000000);
    tf2::Duration timeout(timeout_ns);
    if (tf_buffer_->canTransform(planning_frame_, pose.header.frame_id, tf2::TimePointZero, timeout))
    {
      geometry_msgs::msg::TransformStamped transform = 
          tf_buffer_->lookupTransform(planning_frame_, pose.header.frame_id, tf2::TimePointZero);
      tf2::doTransform(pose, pose, transform);
      pose.header.frame_id = planning_frame_;
      return true;
    }
    else
    {
      LOG_NAMED_ERROR("scene", "转换位姿到planning frame失败：TF不可用");
      return false;
    }
  }
  catch (const tf2::TransformException& ex)
  {
    LOG_NAMED_ERROR("scene", "转换位姿失败: {}", ex.what());
    return false;
  }
}

geometry_msgs::msg::Pose SceneManager::makeCableCylinderPose(
    const geometry_msgs::msg::PoseStamped& cable_pose_planning,
    double cable_yaw)
{
  geometry_msgs::msg::Pose pose;
  
  // 计算圆柱中心位置
  pose.position.x = cable_pose_planning.pose.position.x;
  pose.position.y = cable_pose_planning.pose.position.y;
  pose.position.z = cable_pose_planning.pose.position.z + cable_center_offset_z_;
  
  // 构造圆柱体的 orientation
  // 绕Y轴旋转90度使Z轴水平，然后绕Z轴旋转yaw
  tf2::Quaternion q;
  q.setRPY(0.0, M_PI/2, cable_yaw);
  q.normalize();
  
  pose.orientation = tf2::toMsg(q);
  
  // 日志
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  
  LOG_NAMED_INFO("scene", 
              "[圆柱体姿态] yaw: {:.3f} rad ({:.1f} deg), orientation: ({:.3f}, {:.3f}, {:.3f}, {:.3f})",
              cable_yaw, cable_yaw * 180.0 / M_PI,
              pose.orientation.x, pose.orientation.y, 
              pose.orientation.z, pose.orientation.w);
  
  return pose;
}

bool SceneManager::addCableObject(
    const geometry_msgs::msg::PoseStamped& cable_pose,
    double cable_yaw,
    const std::string& object_id)
{
  std::lock_guard<std::mutex> lock(moveit_mutex_);

  // 检查对象是否已存在
  std::vector<std::string> known_objects = planning_scene_interface_->getKnownObjectNames();
  bool object_exists = std::find(known_objects.begin(), known_objects.end(), object_id) != known_objects.end();

  // 创建碰撞对象
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = planning_frame_;
  collision_object.id = object_id;
  collision_object.operation = object_exists ? 
      moveit_msgs::msg::CollisionObject::MOVE : moveit_msgs::msg::CollisionObject::ADD;

  // 创建圆柱体
  shape_msgs::msg::SolidPrimitive cylinder;
  cylinder.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  cylinder.dimensions.resize(2);
  cylinder.dimensions[0] = cable_length_;
  cylinder.dimensions[1] = cable_diameter_ / 2.0;
  collision_object.primitives.push_back(cylinder);

  // 转换位姿到planning frame
  geometry_msgs::msg::PoseStamped pose_in_planning = cable_pose;
  if (!transformPoseToPlanning(pose_in_planning))
  {
    LOG_NAMED_ERROR("scene", "转换缆绳位姿失败");
    return false;
  }

  // 创建圆柱体pose
  geometry_msgs::msg::Pose cylinder_pose = makeCableCylinderPose(pose_in_planning, cable_yaw);
  collision_object.primitive_poses.push_back(cylinder_pose);

  // 添加到场景
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  planning_scene_interface_->addCollisionObjects(collision_objects);

  LOG_NAMED_INFO("scene", "添加缆绳碰撞体: {}, 位置=({:.3f}, {:.3f}, {:.3f}), yaw={:.1f}°",
              object_id.c_str(), cylinder_pose.position.x, cylinder_pose.position.y, 
              cylinder_pose.position.z, cable_yaw * 180.0 / M_PI);

  // 等待场景更新（分片 10ms，避免长时间阻塞 executor）
  const int max_attempts = 30;
  const int wait_ms = 10;
  for (int i = 0; i < max_attempts; ++i)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
    known_objects = planning_scene_interface_->getKnownObjectNames();
    if (std::find(known_objects.begin(), known_objects.end(), object_id) != known_objects.end())
    {
      LOG_NAMED_INFO("scene", "缆绳碰撞体已添加（等待了 {} ms）", (i + 1) * wait_ms);
      return true;
    }
  }

  LOG_NAMED_WARN("scene", "缆绳碰撞体添加超时，但继续执行");
  return true;
}

bool SceneManager::addGroundPlane()
{
  std::lock_guard<std::mutex> lock(moveit_mutex_);

  const std::string ground_id = "ground_plane";
  std::vector<std::string> known_objects = planning_scene_interface_->getKnownObjectNames();
  bool ground_exists = std::find(known_objects.begin(), known_objects.end(), ground_id) != known_objects.end();

  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = planning_frame_;
  collision_object.id = ground_id;
  collision_object.operation = ground_exists ? 
      moveit_msgs::msg::CollisionObject::MOVE : moveit_msgs::msg::CollisionObject::ADD;

  // 创建地面box
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions.resize(3);
  box.dimensions[0] = 2.0;
  box.dimensions[1] = 2.0;
  box.dimensions[2] = 0.01;
  collision_object.primitives.push_back(box);

  // 获取base_link位置计算地面高度
  double base_z = 0.0;
  try {
    geometry_msgs::msg::TransformStamped base_transform = 
      tf_buffer_->lookupTransform(planning_frame_, "base_link", tf2::TimePointZero);
    base_z = base_transform.transform.translation.z;
  } catch (const tf2::TransformException& ex) {
    LOG_NAMED_WARN("scene", "无法获取base_link位置: {}", ex.what());
  }
  
  double calculated_ground_height = base_z + ground_offset_below_base_;
  double final_ground_height = (ground_height_ != 0.0) ? ground_height_ : calculated_ground_height;

  geometry_msgs::msg::Pose ground_pose;
  ground_pose.position.x = 0.0;
  ground_pose.position.y = 0.0;
  ground_pose.position.z = final_ground_height - 0.005;
  ground_pose.orientation.w = 1.0;
  collision_object.primitive_poses.push_back(ground_pose);

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  planning_scene_interface_->addCollisionObjects(collision_objects);

  LOG_NAMED_INFO("scene", "添加地面碰撞体: z={:.3f} m", final_ground_height);

  // 等待场景更新（分片 10ms，避免长时间阻塞 executor）
  const int max_attempts = 30;
  const int wait_ms = 10;
  for (int i = 0; i < max_attempts; ++i)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
    known_objects = planning_scene_interface_->getKnownObjectNames();
    if (std::find(known_objects.begin(), known_objects.end(), ground_id) != known_objects.end())
    {
      return true;
    }
  }

  return false;
}

bool SceneManager::isObjectAttached(const std::string& object_id, const std::string& eef_link)
{
  try {
    if (!move_group_interface_) {
      return false;
    }
    
    auto state = move_group_interface_->getCurrentState(0.0);
    if (!state) {
      return false;
    }
    
    std::vector<const moveit::core::AttachedBody*> attached_bodies;
    state->getAttachedBodies(attached_bodies);
    
    for (const auto* attached_body : attached_bodies) {
      if (attached_body && attached_body->getName() == object_id) {
        if (attached_body->getAttachedLink() && 
            attached_body->getAttachedLink()->getName() == eef_link) {
          return true;
        }
      }
    }
    
    return false;
  } catch (const std::exception& e) {
    LOG_NAMED_WARN("scene", "检查附着对象异常: {}", e.what());
    return false;
  }
}

bool SceneManager::attachObject(
    const std::string& object_id,
    const std::string& eef_link,
    const geometry_msgs::msg::PoseStamped& cable_pose,
    double cable_yaw)
{
  std::lock_guard<std::mutex> lock(moveit_mutex_);

  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.object.id = object_id;
  attached_object.object.operation = moveit_msgs::msg::CollisionObject::ADD;
  attached_object.object.header.frame_id = eef_link;
  
  // 创建圆柱体几何
  shape_msgs::msg::SolidPrimitive cylinder;
  cylinder.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  cylinder.dimensions.resize(2);
  cylinder.dimensions[0] = cable_length_;
  cylinder.dimensions[1] = cable_diameter_ / 2.0;
  attached_object.object.primitives.push_back(cylinder);
  
  // 转换位姿
  geometry_msgs::msg::PoseStamped pose_in_planning = cable_pose;
  if (pose_in_planning.header.frame_id != planning_frame_)
  {
    if (!transformPoseToPlanning(pose_in_planning))
    {
      LOG_NAMED_ERROR("scene", "附着物体时转换位姿失败");
      return false;
    }
  }
  
  geometry_msgs::msg::Pose cylinder_pose_planning = makeCableCylinderPose(pose_in_planning, cable_yaw);
  
  // 转换到eef_link frame
  geometry_msgs::msg::PoseStamped cylinder_pose_stamped;
  cylinder_pose_stamped.header.frame_id = planning_frame_;
  cylinder_pose_stamped.pose = cylinder_pose_planning;
  
  try {
    rclcpp::Time now = node_->now();
    geometry_msgs::msg::TransformStamped transform = 
        tf_buffer_->lookupTransform(eef_link, planning_frame_, now, tf2::durationFromSec(0.1));
    tf2::doTransform(cylinder_pose_stamped, cylinder_pose_stamped, transform);
    attached_object.object.primitive_poses.push_back(cylinder_pose_stamped.pose);
  } catch (const tf2::TransformException& ex) {
    LOG_NAMED_WARN("scene", "无法转换到eef_link frame，fallback: {}", ex.what());
    attached_object.object.header.frame_id = planning_frame_;
    attached_object.object.primitive_poses.push_back(cylinder_pose_planning);
  }
  
  attached_object.link_name = eef_link;
  attached_object.touch_links = allow_touch_links_;

  // 先移除旧附着
  if (isObjectAttached(object_id, eef_link))
  {
    moveit_msgs::msg::AttachedCollisionObject rm;
    rm.object.id = object_id;
    rm.link_name = eef_link;
    rm.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    planning_scene_interface_->applyAttachedCollisionObject(rm);
    for (int k = 0; k < 2; ++k) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  // 移除场景中的world对象
  std::vector<std::string> known_objects = planning_scene_interface_->getKnownObjectNames();
  if (std::find(known_objects.begin(), known_objects.end(), object_id) != known_objects.end())
  {
    std::vector<std::string> object_ids = {object_id};
    planning_scene_interface_->removeCollisionObjects(object_ids);
  }
  
  for (int k = 0; k < 5; ++k) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  
  // 附着
  planning_scene_interface_->applyAttachedCollisionObject(attached_object);

  LOG_NAMED_INFO("scene", "附着物体 {} 到 {}", object_id.c_str(), eef_link.c_str());
  return true;
}

bool SceneManager::removeObject(const std::string& object_id)
{
  std::lock_guard<std::mutex> lock(moveit_mutex_);

  moveit_msgs::msg::CollisionObject remove_object;
  remove_object.id = object_id;
  remove_object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(remove_object);
  planning_scene_interface_->applyCollisionObjects(collision_objects);

  LOG_NAMED_INFO("scene", "移除物体: {}", object_id.c_str());
  return true;
}

bool SceneManager::detachAndRemove(const std::string& object_id, const std::string& eef_link)
{
  std::lock_guard<std::mutex> lock(moveit_mutex_);

  std::vector<std::string> known_objects = planning_scene_interface_->getKnownObjectNames();
  bool world_object_exists = std::find(known_objects.begin(), known_objects.end(), object_id) != known_objects.end();
  bool attached_object_exists = isObjectAttached(object_id, eef_link);

  // Detach
  if (attached_object_exists)
  {
    moveit_msgs::msg::AttachedCollisionObject aco;
    aco.object.id = object_id;
    aco.link_name = eef_link;
    aco.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    planning_scene_interface_->applyAttachedCollisionObject(aco);
    LOG_NAMED_INFO("scene", "已分离附着对象: {}", object_id.c_str());
    for (int k = 0; k < 5; ++k) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  // Remove from world
  if (world_object_exists)
  {
    std::vector<std::string> object_ids = {object_id};
    planning_scene_interface_->removeCollisionObjects(object_ids);
    LOG_NAMED_INFO("scene", "已移除场景对象: {}", object_id.c_str());
  }

  return true;
}

}  // namespace m5_grasp
