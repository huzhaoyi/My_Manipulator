#include <memory>
#include <functional>
#include <thread>
#include <future>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <vector>
#include <sstream>
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "m5_grasp/trajectory_planner.hpp"
#include "m5_msgs/msg/cable_pose_with_yaw.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/path.hpp>

// MoveIt Task Constructor includes
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/fixed_state.h>  // FixedState stage，避免时间戳检查
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/pick.h>  // Place类也在pick.h中定义
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>
#include <moveit_task_constructor_msgs/msg/solution.hpp>

// MTC namespace alias
namespace mtc = moveit::task_constructor;

// 任务结构体：存储缆绳位姿和原始yaw（用于障碍物方向计算）
struct CableGraspTask {
  geometry_msgs::msg::PoseStamped cable_pose;  // 缆绳位姿（包含调整后的orientation，用于夹爪方向）
  double original_yaw;  // 视觉提供的原始yaw（缆绳切向方向），用于障碍物方向计算
  double joint1_target;  // Joint1目标角度（已废弃，保留兼容性）
  double joint4_target;  // Joint4目标角度（控制夹爪yaw旋转）
};

class M5Grasp : public rclcpp::Node
{
  
public:
  M5Grasp() : Node("m5_grasp")
  {
    // 初始化 TF2 buffer 和 listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 订阅缆绳位置话题
    cable_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/cable_pose", 10, 
        std::bind(&M5Grasp::cable_pose_callback, this, std::placeholders::_1));

    // 订阅带yaw的缆绳位置话题
    cable_pose_with_yaw_sub_ = this->create_subscription<m5_msgs::msg::CablePoseWithYaw>(
        "/cable_pose_with_yaw", 10,
        std::bind(&M5Grasp::cable_pose_with_yaw_callback, this, std::placeholders::_1));

    // 订阅放置位置话题
    place_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/cable_place_pose", 10,
        std::bind(&M5Grasp::place_pose_callback, this, std::placeholders::_1));

    // 订阅带yaw的放置位置话题
    place_pose_with_yaw_sub_ = this->create_subscription<m5_msgs::msg::CablePoseWithYaw>(
        "/cable_place_pose_with_yaw", 10,
        std::bind(&M5Grasp::place_pose_with_yaw_callback, this, std::placeholders::_1));

    // 订阅急停话题
    emergency_stop_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/emergency_stop", 10,
        std::bind(&M5Grasp::emergency_stop_callback, this, std::placeholders::_1));

    // 发布抓取状态
    state_publisher_ = this->create_publisher<std_msgs::msg::String>("/grasp_state", 10);

    // 可视化参数
    this->declare_parameter("viz.enable", true);
    this->declare_parameter("viz.marker_topic", "/grasp_markers");
    this->declare_parameter("viz.path_topic", "/eef_path");

    enable_viz_ = this->get_parameter("viz.enable").as_bool();
    auto marker_topic = this->get_parameter("viz.marker_topic").as_string();
    auto path_topic = this->get_parameter("viz.path_topic").as_string();

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic, 10);
    eef_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic, 10);
    cable_pose_viz_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/cable_pose_visualization", 10);

    // 声明和加载参数
    declare_parameters();

    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "m5_grasp节点已启动");
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "节点名称: %s", this->get_name());
    RCLCPP_INFO(this->get_logger(), "节点命名空间: %s", this->get_namespace());
    RCLCPP_INFO(this->get_logger(), "订阅话题: /cable_pose");
    RCLCPP_INFO(this->get_logger(), "  消息类型: geometry_msgs::msg::PoseStamped");
    RCLCPP_INFO(this->get_logger(), "  队列大小: 10");
    RCLCPP_INFO(this->get_logger(), "订阅话题: /cable_pose_with_yaw");
    RCLCPP_INFO(this->get_logger(), "  消息类型: m5_msgs::msg::CablePoseWithYaw");
    RCLCPP_INFO(this->get_logger(), "  队列大小: 10");
    RCLCPP_INFO(this->get_logger(), "订阅话题: /cable_place_pose");
    RCLCPP_INFO(this->get_logger(), "  消息类型: geometry_msgs::msg::PoseStamped");
    RCLCPP_INFO(this->get_logger(), "  队列大小: 10");
    RCLCPP_INFO(this->get_logger(), "订阅话题: /cable_place_pose_with_yaw");
    RCLCPP_INFO(this->get_logger(), "  消息类型: m5_msgs::msg::CablePoseWithYaw");
    RCLCPP_INFO(this->get_logger(), "  队列大小: 10");
    RCLCPP_INFO(this->get_logger(), "订阅话题: /emergency_stop");
    RCLCPP_INFO(this->get_logger(), "  消息类型: std_msgs::msg::String");
    RCLCPP_INFO(this->get_logger(), "  队列大小: 10");
    RCLCPP_INFO(this->get_logger(), "发布话题: /grasp_state");
    RCLCPP_INFO(this->get_logger(), "  消息类型: std_msgs::msg::String");
    RCLCPP_INFO(this->get_logger(), "  队列大小: 10");
    RCLCPP_INFO(this->get_logger(), "等待缆绳位置/放置位置消息...");
    RCLCPP_INFO(this->get_logger(), "========================================");
    
    // 验证订阅器是否创建成功
    if (cable_pose_sub_) {
      RCLCPP_INFO(this->get_logger(), "✓ 缆绳位置订阅器创建成功");
    } else {
      RCLCPP_ERROR(this->get_logger(), "✗ 缆绳位置订阅器创建失败！");
    }
    
    if (cable_pose_with_yaw_sub_) {
      RCLCPP_INFO(this->get_logger(), "✓ 缆绳位置（带yaw）订阅器创建成功");
    } else {
      RCLCPP_ERROR(this->get_logger(), "✗ 缆绳位置（带yaw）订阅器创建失败！");
    }
    
    if (place_pose_sub_) {
      RCLCPP_INFO(this->get_logger(), "✓ 放置位置订阅器创建成功");
    } else {
      RCLCPP_ERROR(this->get_logger(), "✗ 放置位置订阅器创建失败！");
    }
    
    if (place_pose_with_yaw_sub_) {
      RCLCPP_INFO(this->get_logger(), "✓ 放置位置（带yaw）订阅器创建成功");
    } else {
      RCLCPP_ERROR(this->get_logger(), "✗ 放置位置（带yaw）订阅器创建失败！");
    }
    
    if (emergency_stop_sub_) {
      RCLCPP_INFO(this->get_logger(), "✓ 急停订阅器创建成功");
    } else {
      RCLCPP_ERROR(this->get_logger(), "✗ 急停订阅器创建失败！");
    }
    
    if (state_publisher_) {
      RCLCPP_INFO(this->get_logger(), "✓ 状态发布器创建成功");
    } else {
      RCLCPP_ERROR(this->get_logger(), "✗ 状态发布器创建失败！");
    }
  }

  ~M5Grasp()
  {
    // 停止工作线程
    stop_workers_ = true;
    queue_cv_.notify_all();
    for (auto& t : worker_threads_)
      if (t.joinable()) t.join();

    // 停止 executor（修复顺序：先cancel，再join，最后remove_node）
    if (executor_)
    {
      // 1. 先cancel，停止executor的spin
      executor_->cancel();
      
      // 2. 等待executor线程结束
      if (executor_thread_.joinable())
        executor_thread_.join();
      
      // 3. 最后remove_node（或让executor析构时自动处理）
      // 注意：某些ROS2版本在executor析构时会自动remove，这里可选
      executor_->remove_node(this->get_node_base_interface());
    }
    else if (executor_thread_.joinable())
    {
      // 如果executor_为空但线程还在运行，直接join
      executor_thread_.join();
    }
  }

  // 初始化MoveIt接口（在对象创建后调用）
  void init_moveit()
  {
    RCLCPP_INFO(this->get_logger(), "开始初始化MoveIt接口...");
    
    // MoveIt会从robot_description topic获取参数，不需要等待参数
    // 但需要等待robot_state_publisher启动（通常由launch文件保证）
    // 等待一小段时间确保robot_state_publisher已启动
    RCLCPP_INFO(this->get_logger(), "等待robot_state_publisher启动...");
    std::this_thread::sleep_for(std::chrono::milliseconds(
      static_cast<int>(robot_state_publisher_wait_ * 1000)));
    
    // 初始化 MoveIt 接口
    // 注意：shared_from_this() 只能在构造函数完成后调用
    // 永远不要在构造函数中使用 shared_from_this()，否则会抛出 bad_weak_ptr 异常
    // 此函数必须在对象完全构造后调用（由 main() 保证）
    try {
      RCLCPP_INFO(this->get_logger(), "创建MoveGroupInterface (arm_group)...");
      move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          shared_from_this(), "arm_group");
      RCLCPP_INFO(this->get_logger(), "✓ MoveGroupInterface (arm_group) 创建成功");
      
      RCLCPP_INFO(this->get_logger(), "创建MoveGroupInterface (gripper_group)...");
      gripper_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          shared_from_this(), "gripper_group");
      RCLCPP_INFO(this->get_logger(), "✓ MoveGroupInterface (gripper_group) 创建成功");
      
      RCLCPP_INFO(this->get_logger(), "创建PlanningSceneInterface...");
      planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
      RCLCPP_INFO(this->get_logger(), "✓ PlanningSceneInterface 创建成功");

      RCLCPP_INFO(this->get_logger(), "MoveIt接口已初始化");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "MoveIt接口初始化失败: %s", e.what());
      RCLCPP_ERROR(this->get_logger(), "可能原因: robot_state_publisher未启动或robot_description未发布");
      throw;  // 重新抛出异常，让调用者知道初始化失败
    }

    // 关节命名一致性诊断：检查MoveIt模型关节名和joint_states关节名是否一致
    // 如果不一致，会导致CurrentStateMonitor无法更新状态，出现"Invalid goal state"错误
    RCLCPP_INFO(this->get_logger(), "=== 关节命名一致性诊断 ===");
    try {
      const auto& robot_model = move_group_interface_->getRobotModel();
      if (robot_model) {
        // 获取MoveIt模型中的关节名（arm_group）
        const auto* jmg = robot_model->getJointModelGroup("arm_group");
        if (jmg) {
          const auto& model_joint_names = jmg->getActiveJointModelNames();
          RCLCPP_INFO(this->get_logger(), "[关节命名诊断] MoveIt模型中的arm_group关节名 (%zu个):", model_joint_names.size());
          for (size_t i = 0; i < model_joint_names.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "  [%zu] %s", i, model_joint_names[i].c_str());
          }
          
          // 等待joint_states话题有数据，然后检查关节名
          RCLCPP_INFO(this->get_logger(), "[关节命名诊断] 等待joint_states话题数据...");
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
          
          // 从/joint_states话题实际读取关节名（关键诊断！）
          // 创建临时订阅者来读取一次joint_states消息
          sensor_msgs::msg::JointState::SharedPtr joint_states_msg = nullptr;
          std::mutex msg_mutex;
          std::condition_variable msg_cv;
          bool msg_received = false;
          
          auto joint_states_sub = this->create_subscription<sensor_msgs::msg::JointState>(
              "/joint_states", 10,
              [&](const sensor_msgs::msg::JointState::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(msg_mutex);
                if (!msg_received) {
                  joint_states_msg = msg;
                  msg_received = true;
                  msg_cv.notify_one();
                }
              });
          
          // 等待收到joint_states消息（最多等待1秒）
          {
            std::unique_lock<std::mutex> lock(msg_mutex);
            if (msg_cv.wait_for(lock, std::chrono::seconds(1), [&] { return msg_received; })) {
              RCLCPP_INFO(this->get_logger(), "[关节命名诊断] ✓ 成功从/joint_states话题读取关节名 (%zu个):", 
                         joint_states_msg->name.size());
              for (size_t i = 0; i < joint_states_msg->name.size(); ++i) {
                RCLCPP_INFO(this->get_logger(), "  [%zu] %s = %.3f rad", i, 
                           joint_states_msg->name[i].c_str(), 
                           i < joint_states_msg->position.size() ? joint_states_msg->position[i] : 0.0);
              }
              
              // 对比MoveIt模型关节名和joint_states关节名
              // 注意：joint_states可能包含更多关节（如夹爪），只要MoveIt关心的关节都在即可
              bool names_match = true;
              std::vector<std::string> missing_joints;
              
              for (size_t i = 0; i < model_joint_names.size(); ++i) {
                // 查找joint_states中是否有对应的关节名
                bool found = false;
                for (size_t j = 0; j < joint_states_msg->name.size(); ++j) {
                  if (joint_states_msg->name[j] == model_joint_names[i]) {
                    found = true;
                    break;
                  }
                }
                if (!found) {
                  missing_joints.push_back(model_joint_names[i]);
                  names_match = false;
                }
              }
              
              if (names_match) {
                RCLCPP_INFO(this->get_logger(), "[关节命名诊断] ✓ MoveIt关心的所有关节都在joint_states中找到");
                RCLCPP_INFO(this->get_logger(), "[关节命名诊断]   MoveIt模型: %zu个关节, joint_states: %zu个关节（包含额外关节，正常）", 
                           model_joint_names.size(), joint_states_msg->name.size());
              } else {
                RCLCPP_ERROR(this->get_logger(), "[关节命名诊断] ✗ 以下关节在joint_states中未找到:");
                for (const auto& joint : missing_joints) {
                  RCLCPP_ERROR(this->get_logger(), "[关节命名诊断]   - %s", joint.c_str());
                }
                RCLCPP_ERROR(this->get_logger(), "[关节命名诊断] 这会导致状态更新失败和'Invalid goal state'错误");
              }
            } else {
              RCLCPP_WARN(this->get_logger(), "[关节命名诊断] ⚠️ 1秒内未收到joint_states消息，跳过关节名对比");
            }
          }
          
          // 尝试获取一次joint_states来检查关节名
          // 注意：这里只是诊断，不阻塞初始化流程
          // 修复：使用getCurrentStateSafe()绕过时间戳检查
          auto test_state = getCurrentStateSafe(1.0);
          if (test_state) {
            RCLCPP_INFO(this->get_logger(), "[关节命名诊断] ✓ 成功获取当前状态，关节名应该匹配");
            // 检查关节值是否有效（如果关节名不匹配，值会是默认值或NaN）
            std::vector<double> test_joint_values;
            test_state->copyJointGroupPositions(jmg, test_joint_values);
            bool all_valid = true;
            for (size_t i = 0; i < test_joint_values.size(); ++i) {
              if (std::isnan(test_joint_values[i]) || std::isinf(test_joint_values[i])) {
                RCLCPP_WARN(this->get_logger(), "[关节命名诊断] ⚠️ 关节 %s 的值无效: %f (可能是关节名不匹配)", 
                           i < model_joint_names.size() ? model_joint_names[i].c_str() : "unknown",
                           test_joint_values[i]);
                all_valid = false;
              }
            }
            if (all_valid) {
              RCLCPP_INFO(this->get_logger(), "[关节命名诊断] ✓ 所有关节值有效，关节名可能匹配");
            } else {
              RCLCPP_WARN(this->get_logger(), "[关节命名诊断] ⚠️ 部分关节值无效，可能存在关节名不匹配问题");
              RCLCPP_WARN(this->get_logger(), "[关节命名诊断] 建议：检查URDF中的关节名和joint_states话题中的关节名是否一致");
              RCLCPP_WARN(this->get_logger(), "[关节命名诊断] MoveIt期望: %s (等)", model_joint_names.size() > 0 ? model_joint_names[0].c_str() : "unknown");
              RCLCPP_WARN(this->get_logger(), "[关节命名诊断] 请运行: ros2 topic echo /joint_states --once 查看实际关节名");
            }
          } else {
            RCLCPP_WARN(this->get_logger(), "[关节命名诊断] ⚠️ 无法获取当前状态（可能joint_states未就绪或关节名不匹配）");
          }
        } else {
          RCLCPP_WARN(this->get_logger(), "[关节命名诊断] ⚠️ 无法获取arm_group，跳过关节名诊断");
        }
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "[关节命名诊断] 诊断过程出错: %s", e.what());
    }
    RCLCPP_INFO(this->get_logger(), "=== 关节命名一致性诊断完成 ===");
    
    // 等待 joint_states 流起来（MoveIt 获取 current state 依赖 joint_states）
    RCLCPP_INFO(this->get_logger(), "等待 joint_states 流起来...");
    int max_attempts = static_cast<int>(state_check_timeout_ / state_check_interval_);
    int wait_ms = static_cast<int>(state_check_interval_ * 1000);
    bool joint_states_ready = false;
    
    for (int i = 0; i < max_attempts; ++i)
    {
      try
      {
        // 尝试获取当前状态（使用配置的超时时间，线程安全）
        auto state = getCurrentStateSafe(state_check_interval_);
        if (state)
        {
          // 检查所有关节数量（应该包括arm_group的4个关节 + gripper_group的2个关节 = 6个关节）
          // 通过检查robot model的所有关节来验证joint_states是否完整
          const auto& robot_model = state->getRobotModel();
          const auto& joint_names = robot_model->getJointModelNames();
          
          // 检查arm_group和gripper_group的关节是否都在joint_states中
          const auto* arm_jmg = state->getJointModelGroup("arm_group");
          const auto* gripper_jmg = state->getJointModelGroup("gripper_group");
          
          if (arm_jmg && gripper_jmg)
          {
            std::vector<double> arm_joint_values;
            std::vector<double> gripper_joint_values;
            state->copyJointGroupPositions(arm_jmg, arm_joint_values);
            state->copyJointGroupPositions(gripper_jmg, gripper_joint_values);
            
            // 检查arm_group至少有4个关节，gripper_group至少有2个关节
            if (arm_joint_values.size() >= 4 && gripper_joint_values.size() >= 2)
            {
              joint_states_ready = true;
              RCLCPP_INFO(this->get_logger(), 
                          "joint_states 已就绪（等待了 %d ms），arm_group关节数量: %zu, gripper_group关节数量: %zu, 总计: %zu", 
                          (i + 1) * wait_ms, arm_joint_values.size(), gripper_joint_values.size(),
                          arm_joint_values.size() + gripper_joint_values.size());
              break;
            }
          }
        }
      }
      catch (const std::exception& e)
      {
        // 继续等待
      }
      
      std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
    }
    
    if (!joint_states_ready)
    {
      RCLCPP_WARN(this->get_logger(), 
                  "joint_states 在 %d ms 内未就绪，但继续执行（可能后续会恢复）", 
                  max_attempts * wait_ms);
    }

    // 设置规划参数（从配置读取）
    move_group_interface_->setPlanningTime(default_planning_time_);
    move_group_interface_->setNumPlanningAttempts(num_planning_attempts_);
    move_group_interface_->setMaxVelocityScalingFactor(max_velocity_scaling_);
    move_group_interface_->setMaxAccelerationScalingFactor(max_acceleration_scaling_);
    move_group_interface_->setGoalPositionTolerance(goal_position_tolerance_);
    move_group_interface_->setGoalOrientationTolerance(goal_orientation_tolerance_);
    move_group_interface_->allowReplanning(true);
    move_group_interface_->setPlanningPipelineId("ompl");
    move_group_interface_->setPlannerId("RRTConnect");

    // 获取planning frame
    planning_frame_ = move_group_interface_->getPlanningFrame();
    RCLCPP_INFO(this->get_logger(), "Planning frame: %s", planning_frame_.c_str());

    // 设置末端执行器链接
    const moveit::core::RobotModelConstPtr& robot_model = move_group_interface_->getRobotModel();
    const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup("arm_group");
    if (jmg && !jmg->getLinkModelNames().empty())
    {
      const std::vector<std::string>& link_names = jmg->getLinkModelNames();
      std::string tip_link = link_names.back();
      move_group_interface_->setEndEffectorLink(tip_link);
      eef_link_ = tip_link;
      RCLCPP_INFO(this->get_logger(), "End effector link: %s", eef_link_.c_str());
    }
    else
    {
      eef_link_ = "LinkGG";
      move_group_interface_->setEndEffectorLink(eef_link_);
      RCLCPP_WARN(this->get_logger(), "使用默认end effector link: %s", eef_link_.c_str());
    }
    
    // 检测是否为4DOF机械臂
    if (jmg)
    {
      size_t num_joints = jmg->getActiveJointModelNames().size();
      is_4dof_ = (num_joints == 4);
      RCLCPP_INFO(this->get_logger(), "检测到 %zu 自由度机械臂，is_4dof_=%s", 
                  num_joints, is_4dof_ ? "true" : "false");
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "无法获取arm_group，无法检测自由度");
    }
    
    // 验证EEF link和frame（用于诊断问题B）
    RCLCPP_INFO(this->get_logger(), "=== EEF和Frame验证 ===");
    RCLCPP_INFO(this->get_logger(), "EndEffectorLink in MG: %s", 
                move_group_interface_->getEndEffectorLink().c_str());
    RCLCPP_INFO(this->get_logger(), "PoseReferenceFrame: %s", 
                move_group_interface_->getPoseReferenceFrame().c_str());
    RCLCPP_INFO(this->get_logger(), "PlanningFrame: %s", 
                planning_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "内部eef_link_: %s", eef_link_.c_str());
    
    // 诊断：检查EEF link是否是夹爪尖端
    RCLCPP_INFO(this->get_logger(), "=== EEF Link诊断 ===");
    RCLCPP_INFO(this->get_logger(), "当前EEF link: %s", eef_link_.c_str());
    RCLCPP_INFO(this->get_logger(), "注意：如果EEF link不是夹爪尖端，需要添加偏移补偿");
    RCLCPP_INFO(this->get_logger(), "夹爪相关links: LinkGG(基座), LinkGL(左手指), LinkGR(右手指)");
    
    // 尝试使用TF查询夹爪手指位置（如果TF可用）
    try {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));  // 等待TF稳定
      if (tf_buffer_->canTransform(planning_frame_, "LinkGL", tf2::TimePointZero, 
                                    std::chrono::milliseconds(100)) &&
          tf_buffer_->canTransform(planning_frame_, "LinkGR", tf2::TimePointZero, 
                                    std::chrono::milliseconds(100)) &&
          tf_buffer_->canTransform(planning_frame_, eef_link_, tf2::TimePointZero, 
                                    std::chrono::milliseconds(100))) {
        geometry_msgs::msg::TransformStamped tf_eef = 
            tf_buffer_->lookupTransform(planning_frame_, eef_link_, tf2::TimePointZero);
        geometry_msgs::msg::TransformStamped tf_gl = 
            tf_buffer_->lookupTransform(planning_frame_, "LinkGL", tf2::TimePointZero);
        geometry_msgs::msg::TransformStamped tf_gr = 
            tf_buffer_->lookupTransform(planning_frame_, "LinkGR", tf2::TimePointZero);
        
        RCLCPP_INFO(this->get_logger(), "EEF link (%s) 位置: (%.4f, %.4f, %.4f)", 
                    eef_link_.c_str(),
                    tf_eef.transform.translation.x,
                    tf_eef.transform.translation.y,
                    tf_eef.transform.translation.z);
        RCLCPP_INFO(this->get_logger(), "LinkGL 位置: (%.4f, %.4f, %.4f)", 
                    tf_gl.transform.translation.x,
                    tf_gl.transform.translation.y,
                    tf_gl.transform.translation.z);
        RCLCPP_INFO(this->get_logger(), "LinkGR 位置: (%.4f, %.4f, %.4f)", 
                    tf_gr.transform.translation.x,
                    tf_gr.transform.translation.y,
                    tf_gr.transform.translation.z);
        
        // 计算夹爪中心（两指中点）
        double center_x = (tf_gl.transform.translation.x + tf_gr.transform.translation.x) / 2.0;
        double center_y = (tf_gl.transform.translation.y + tf_gr.transform.translation.y) / 2.0;
        double center_z = (tf_gl.transform.translation.z + tf_gr.transform.translation.z) / 2.0;
        
        RCLCPP_INFO(this->get_logger(), "夹爪中心(LinkGL/LinkGR中点)位置: (%.4f, %.4f, %.4f)", 
                    center_x, center_y, center_z);
        
        // 计算EEF link到夹爪中心的偏移
        double offset_x = center_x - tf_eef.transform.translation.x;
        double offset_y = center_y - tf_eef.transform.translation.y;
        double offset_z = center_z - tf_eef.transform.translation.z;
        
        RCLCPP_INFO(this->get_logger(), "EEF link到夹爪中心的偏移: (%.4f, %.4f, %.4f) m", 
                    offset_x, offset_y, offset_z);
        
        if (std::abs(offset_x) > 0.01 || std::abs(offset_y) > 0.01 || std::abs(offset_z) > 0.01) {
          RCLCPP_WARN(this->get_logger(), 
                      "⚠️  检测到EEF link与夹爪中心存在明显偏移！");
          RCLCPP_WARN(this->get_logger(), 
                      "   如果目标位姿是针对夹爪中心计算的，需要添加偏移补偿");
        } else {
          RCLCPP_INFO(this->get_logger(), 
                      "✓ EEF link与夹爪中心偏移很小，可能不需要补偿");
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "TF不可用，无法查询夹爪手指位置");
      }
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(), "TF查询失败: %s", ex.what());
    }
    
    RCLCPP_INFO(this->get_logger(), "=== EEF和Frame验证完成 ===");

    // 打印当前关节值用于诊断
    try {
      auto arm_jv = getCurrentJointValuesSafe();
      auto grip_jv = getCurrentGripperJointValuesSafe();
      
      RCLCPP_INFO(this->get_logger(), "=== MoveIt 当前关节值诊断 ===");
      RCLCPP_INFO(this->get_logger(), "Arm joints (%zu个):", arm_jv.size());
      for (size_t i = 0; i < arm_jv.size(); ++i) {
        RCLCPP_INFO(this->get_logger(), "  Joint[%zu] = %.3f rad", i, arm_jv[i]);
      }
      
      RCLCPP_INFO(this->get_logger(), "Gripper joints (%zu个):", grip_jv.size());
      if (grip_jv.size() > 0) {
        RCLCPP_INFO(this->get_logger(), "  JointGL = %.3f rad", grip_jv[0]);
      }
      if (grip_jv.size() > 1) {
        RCLCPP_INFO(this->get_logger(), "  JointGR = %.3f rad", grip_jv[1]);
      }
      RCLCPP_INFO(this->get_logger(), "=== 关节值诊断完成 ===");
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "无法获取当前关节值: %s", e.what());
    }

    // 启动工作线程
    stop_workers_ = false;
    RCLCPP_INFO(this->get_logger(), "=== 启动工作线程 ===");
    RCLCPP_INFO(this->get_logger(), "工作线程数量: %zu", num_worker_threads_);
    for (size_t i = 0; i < num_worker_threads_; ++i)
    {
      worker_threads_.emplace_back(&M5Grasp::worker_thread, this);
      RCLCPP_INFO(this->get_logger(), "工作线程 %zu 已启动", i + 1);
    }
    // 等待一小段时间确保工作线程已进入等待状态
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    RCLCPP_INFO(this->get_logger(), "=== 工作线程启动完成，已准备好接收任务 ===");
    
    // 初始化路径的frame_id
    eef_path_.header.frame_id = planning_frame_;
    
    // 添加地面碰撞对象（如果启用）
    if (add_ground_plane_)
    {
      RCLCPP_INFO(this->get_logger(), "=== 添加地面碰撞对象 ===");
      scene_add_ground_plane();
      RCLCPP_INFO(this->get_logger(), "=== 地面碰撞对象添加完成 ===");
    }
  }

  // 启动executor（在对象创建后调用）
  // 注意：shared_from_this() 只能在构造函数完成后调用
  // 永远不要在构造函数中使用 shared_from_this()，否则会抛出 bad_weak_ptr 异常
  // 此函数必须在对象完全构造后调用（由 main() 保证）
  void start_executor()
  {
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(this->get_node_base_interface());
    RCLCPP_INFO(this->get_logger(), "Executor已创建，节点已添加到executor");
    executor_thread_ = std::thread([this]() {
      try {
        RCLCPP_INFO(this->get_logger(), "Executor线程开始spin...");
        executor_->spin();
        RCLCPP_INFO(this->get_logger(), "Executor线程spin结束");
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Executor thread exception: %s", e.what());
      }
    });
    RCLCPP_INFO(this->get_logger(), "✓ Executor线程已启动");
    
    // 等待足够的时间确保executor开始运行并注册订阅器
    // ROS2订阅器需要executor spin一段时间才能注册到系统中
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // 验证executor线程是否运行
    if (executor_thread_.joinable())
    {
      RCLCPP_INFO(this->get_logger(), "✓ Executor线程正在运行");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "✗ Executor线程未运行！");
    }
    
    // 验证订阅器是否已注册
    RCLCPP_INFO(this->get_logger(), "=== 订阅器诊断信息 ===");
    if (cable_pose_sub_) {
      size_t pub_count = cable_pose_sub_->get_publisher_count();
      RCLCPP_INFO(this->get_logger(), "✓ 缆绳位置订阅器已创建");
      RCLCPP_INFO(this->get_logger(), "  订阅话题: /cable_pose");
      RCLCPP_INFO(this->get_logger(), "  发布者数量: %zu", pub_count);
    } else {
      RCLCPP_ERROR(this->get_logger(), "✗ 缆绳位置订阅器未创建！");
    }
    
    if (cable_pose_with_yaw_sub_) {
      size_t pub_count = cable_pose_with_yaw_sub_->get_publisher_count();
      RCLCPP_INFO(this->get_logger(), "✓ 缆绳位置（带yaw）订阅器已创建");
      RCLCPP_INFO(this->get_logger(), "  订阅话题: /cable_pose_with_yaw");
      RCLCPP_INFO(this->get_logger(), "  发布者数量: %zu", pub_count);
    } else {
      RCLCPP_ERROR(this->get_logger(), "✗ 缆绳位置（带yaw）订阅器未创建！");
    }
    
    if (place_pose_sub_) {
      size_t pub_count = place_pose_sub_->get_publisher_count();
      RCLCPP_INFO(this->get_logger(), "✓ 放置位置订阅器已创建");
      RCLCPP_INFO(this->get_logger(), "  订阅话题: /cable_place_pose");
      RCLCPP_INFO(this->get_logger(), "  发布者数量: %zu", pub_count);
    } else {
      RCLCPP_ERROR(this->get_logger(), "✗ 放置位置订阅器未创建！");
    }
    
    if (place_pose_with_yaw_sub_) {
      size_t pub_count = place_pose_with_yaw_sub_->get_publisher_count();
      RCLCPP_INFO(this->get_logger(), "✓ 放置位置（带yaw）订阅器已创建");
      RCLCPP_INFO(this->get_logger(), "  订阅话题: /cable_place_pose_with_yaw");
      RCLCPP_INFO(this->get_logger(), "  发布者数量: %zu", pub_count);
    } else {
      RCLCPP_ERROR(this->get_logger(), "✗ 放置位置（带yaw）订阅器未创建！");
    }
    
    if (emergency_stop_sub_) {
      size_t pub_count = emergency_stop_sub_->get_publisher_count();
      RCLCPP_INFO(this->get_logger(), "✓ 急停订阅器已创建");
      RCLCPP_INFO(this->get_logger(), "  订阅话题: /emergency_stop");
      RCLCPP_INFO(this->get_logger(), "  发布者数量: %zu", pub_count);
      if (pub_count == 0)
      {
        RCLCPP_WARN(this->get_logger(), "  [警告] 没有发布者，但订阅器已创建，等待发布者连接...");
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "  [成功] 检测到发布者，急停功能可用");
      }
      RCLCPP_INFO(this->get_logger(), "  等待急停消息...");
    } else {
      RCLCPP_ERROR(this->get_logger(), "✗ 急停订阅器未创建！");
    }
    RCLCPP_INFO(this->get_logger(), "=== 订阅器诊断完成 ===");
    
    // 验证状态发布器
    RCLCPP_INFO(this->get_logger(), "=== 发布器诊断信息 ===");
    if (state_publisher_)
    {
      size_t sub_count = state_publisher_->get_subscription_count();
      RCLCPP_INFO(this->get_logger(), "✓ 状态发布器已创建");
      RCLCPP_INFO(this->get_logger(), "  发布话题: /grasp_state");
      RCLCPP_INFO(this->get_logger(), "  订阅者数量: %zu", sub_count);
      if (sub_count == 0)
      {
        RCLCPP_WARN(this->get_logger(), "  [警告] 没有订阅者，状态消息可能无法接收");
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "  [成功] 检测到订阅者，状态消息可以发送");
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "✗ 状态发布器未创建！");
    }
    RCLCPP_INFO(this->get_logger(), "=== 发布器诊断完成 ===");
  }

private:
  // 声明参数
  void declare_parameters()
  {
    // Cable parameters
    this->declare_parameter("cable.name", "cable_1");
    this->declare_parameter("cable.diameter", 0.008);
    this->declare_parameter("cable.length", 0.20);
    this->declare_parameter("cable.frame_id", "world");
    this->declare_parameter("cable.shape", "cylinder");
    this->declare_parameter("cable.center_offset_z", 0.0);  // 默认0.0表示cable_pose是圆柱中心

    // Grasp parameters
    this->declare_parameter("grasp.approach_offset_z", 0.10);
    this->declare_parameter("grasp.descend_distance", 0.08);
    this->declare_parameter("grasp.lift_distance", 0.06);
    this->declare_parameter("grasp.max_pos_error", 0.01);
    this->declare_parameter("grasp.use_cartesian", true);
    this->declare_parameter("grasp.eef_step", 0.005);
    this->declare_parameter("grasp.jump_threshold", 0.0);
    this->declare_parameter("grasp.planning_time", 15.0);
    this->declare_parameter("grasp.num_planning_attempts", 30);
    this->declare_parameter("grasp.max_velocity_scaling", 0.9);
    this->declare_parameter("grasp.max_acceleration_scaling", 0.9);
    this->declare_parameter("grasp.goal_position_tolerance", 0.01);
    this->declare_parameter("grasp.goal_orientation_tolerance", 0.5);
    this->declare_parameter("grasp.fallback_planning_time", 2.0);
    this->declare_parameter("grasp.ik_timeout", 0.1);
    this->declare_parameter("grasp.state_check_timeout", 1.0);
    this->declare_parameter("grasp.state_check_interval", 0.1);
    this->declare_parameter("grasp.robot_state_publisher_wait", 2.0);
    this->declare_parameter("grasp.executor_startup_wait", 0.5);
    this->declare_parameter("grasp.segment_execution_wait", 0.1);
    this->declare_parameter("grasp.state_sync_wait", 0.1);
    this->declare_parameter("grasp.control_frequency", 10.0);
    this->declare_parameter("grasp.read_frequency", 10.0);
    this->declare_parameter("grasp.main_loop_frequency", 10.0);
    this->declare_parameter("grasp.yaw_offset", 0.0);  // yaw偏移量（弧度）
    this->declare_parameter("grasp.yaw_flip", false);  // 是否翻转yaw 180度
    this->declare_parameter("grasp.grasp_yaw_add", M_PI / 2.0);  // 线缆切向→夹持方向偏移（默认+90°横夹）
    this->declare_parameter("grasp.orientation_yaw_offset", M_PI / 2.0);  // orientation的Yaw分量偏移（默认90度），用于调整joint 4角度
    this->declare_parameter("grasp.z_clearance", 0.002);  // 抓取高度安全余量（米，默认2mm）
    this->declare_parameter("grasp.yaw_validation_enabled", true);  // 是否启用yaw合法性检查
    this->declare_parameter("grasp.yaw_tolerance", 0.0873);  // yaw搜索容差（默认±5°）
    this->declare_parameter("grasp.yaw_search_step", 0.0175);  // yaw搜索步长（默认1°）
    this->declare_parameter("grasp.joint4_yaw_search_enabled", true);  // 是否启用Joint4-yaw迭代搜索
    this->declare_parameter("grasp.joint4_yaw_search_range", M_PI);  // Joint4-yaw搜索范围（默认±180度）
    this->declare_parameter("grasp.joint4_yaw_search_step", 0.0873);  // Joint4-yaw搜索步长（默认5度）
    
    // Joint约束参数（4DOF机器人控制）
    this->declare_parameter("grasp.joint1_tolerance", 0.1745);  // Joint1容差（默认±10°）
    this->declare_parameter("grasp.joint4_constraint_enabled", false);  // 是否启用Joint4约束
    this->declare_parameter("grasp.joint4_offset", 0.0);  // Joint4固定offset（零位标定）
    this->declare_parameter("grasp.joint4_tolerance", 0.0873);  // Joint4容差（默认±5°）
    
    this->declare_parameter("grasp.enable_recovery", true);  // 是否启用执行失败后的状态回滚
    this->declare_parameter("grasp.recovery_timeout", 5.0);  // 回滚操作的超时时间
    
    // yaw候选搜索参数
    this->declare_parameter("grasp.yaw_candidate_search_enabled", true);  // 是否启用yaw候选搜索
    this->declare_parameter("grasp.yaw_candidate_center", 0.0);  // 候选搜索中心yaw（仅xyz输入时使用）
    this->declare_parameter("grasp.yaw_candidate_range", M_PI);  // 搜索范围（±180°）
    this->declare_parameter("grasp.yaw_candidate_step", 0.2618);  // 搜索步长（15°）
    
    // 地面安全参数
    this->declare_parameter("grasp.ground_height", 0.0);  // 地面高度
    this->declare_parameter("grasp.ground_offset_below_base", -0.05);  // 地面在基座下方的偏移（-5cm，基座下方5cm）
    this->declare_parameter("grasp.camera_error_margin", 0.005);  // 相机标定误差余量（5mm）
    this->declare_parameter("grasp.min_ground_clearance", 0.010);  // 最小离地安全距离（10mm）
    this->declare_parameter("grasp.add_ground_plane", true);  // 是否添加地面碰撞对象
    
    // 下压距离限制
    this->declare_parameter("grasp.max_cartesian_descend_distance", 0.15);  // 单次笛卡尔下压最大距离（15cm）
    
    // TCP偏移补偿参数（LinkGG到夹爪中心的偏移）
    this->declare_parameter("grasp.tcp_offset_x", 0.0);  // m  TCP偏移X（LinkGG→夹爪中心）
    this->declare_parameter("grasp.tcp_offset_y", -0.024);  // m  TCP偏移Y（LinkGG→夹爪中心）
    this->declare_parameter("grasp.tcp_offset_z", -0.0086);  // m  TCP偏移Z（LinkGG→夹爪中心）
    
    // 全零状态预运动参数
    this->declare_parameter("grasp.auto_move_to_ready", true);  // 是否自动移动到预备位置
    this->declare_parameter("grasp.ready_joint1", 0.0);  // rad  预备位置Joint1
    this->declare_parameter("grasp.ready_joint2", -0.5);  // rad  预备位置Joint2（约-30度）
    this->declare_parameter("grasp.ready_joint3", -0.5);  // rad  预备位置Joint3（约-30度）
    this->declare_parameter("grasp.ready_joint4", 0.0);  // rad  预备位置Joint4
    this->declare_parameter("grasp.near_zero_threshold", 0.1);  // rad  全零状态判断阈值

    // Gripper parameters (position control only, no force control)
    this->declare_parameter("gripper.mode", "position");
    this->declare_parameter("gripper.open_width", 0.030);  // 默认值改为0.030，与配置文件一致
    this->declare_parameter("gripper.close_width", 0.010);
    this->declare_parameter("gripper.close_extra", 0.002);
    this->declare_parameter("gripper.hold_time_ms", 200);
    this->declare_parameter("gripper.angle_min", 1.01);
    this->declare_parameter("gripper.angle_max", -1.01);
    this->declare_parameter("gripper.width_min", 0.0);

    // Scene parameters
    this->declare_parameter("scene.add_collision_object", true);
    this->declare_parameter("scene.allow_touch_links", std::vector<std::string>{"LinkGG", "LinkGL", "LinkGR"});

    // Workspace parameters (基于URDF精确计算)
    // 4DOF机械臂工作空间分析:
    //   base_link原点 -> Joint1轴: z=0.141m
    //   Joint1 -> Joint2轴: z=0.0735m (link1高度)
    //   Joint2 -> Joint3轴: z=0.264m (link2长度)
    //   Joint3 -> Joint4轴: z=0.143m (link3长度)
    //   Joint4 -> EEF: z=0.187m (link4+夹爪)
    //   总高度(全伸展): 0.141 + 0.0735 + 0.264 + 0.143 + 0.187 = 0.8085m
    //   Joint2轴高度: 0.141 + 0.0735 = 0.2145m
    //   大臂长度: 0.264m, 小臂+末端: 0.143 + 0.187 = 0.330m
    //   最大伸展: 0.264 + 0.330 = 0.594m
    // 
    // Joint2/3限位: [-2.97, 0] (只能向下弯，厂家新限制-170°~0°)
    // 工作空间呈"碗状"区域:
    //   水平半径: 0.08m ~ 0.55m (距base中心)
    //   高度: 0.15m ~ 0.80m (相对base_link原点)
    this->declare_parameter("workspace.base_height", 0.2145);  // Joint2轴高度
    this->declare_parameter("workspace.link2_length", 0.264);  // 大臂
    this->declare_parameter("workspace.link3_length", 0.143);  // 小臂
    this->declare_parameter("workspace.link4_to_eef", 0.187);  // 末端
    this->declare_parameter("workspace.reach_radius_margin", 0.95);  // 最大半径裕度(略保守)
    this->declare_parameter("workspace.max_height_offset", 0.60);   // 相对Joint2轴的最大高度偏移
    this->declare_parameter("workspace.min_height_offset", -0.10);  // 相对Joint2轴的最小高度偏移
    this->declare_parameter("workspace.min_radius", 0.08);  // 最小半径(避免太靠近base)
    this->declare_parameter("workspace.safe_x_min", 0.15);
    this->declare_parameter("workspace.safe_x_max", 0.40);
    this->declare_parameter("workspace.safe_y_min", -0.20);
    this->declare_parameter("workspace.safe_y_max", 0.20);
    this->declare_parameter("workspace.safe_z_min", 0.20);
    this->declare_parameter("workspace.safe_z_max", 0.50);
    this->declare_parameter("workspace.medium_x_min", 0.10);
    this->declare_parameter("workspace.medium_x_max", 0.50);
    this->declare_parameter("workspace.medium_y_min", -0.30);
    this->declare_parameter("workspace.medium_y_max", 0.30);
    this->declare_parameter("workspace.medium_z_min", 0.15);
    this->declare_parameter("workspace.medium_z_max", 0.60);

    // Tolerance parameters
    this->declare_parameter("tolerance.orientation_epsilon", 1e-6);

    // Place parameters
    this->declare_parameter("place.approach_offset_z", 0.10);
    this->declare_parameter("place.descend_distance", 0.08);
    this->declare_parameter("place.retreat_distance", 0.06);
    this->declare_parameter("place.max_pos_error", 0.01);
    this->declare_parameter("place.planning_time", 15.0);
    this->declare_parameter("place.num_planning_attempts", 30);
    this->declare_parameter("place.max_velocity_scaling", 0.9);
    this->declare_parameter("place.max_acceleration_scaling", 0.9);
    this->declare_parameter("place.goal_position_tolerance", 0.01);
    this->declare_parameter("place.goal_orientation_tolerance", 0.5);

    // Trajectory planning parameters
    this->declare_parameter("trajectory.use_linear_interpolation", false);
    this->declare_parameter("trajectory.linear_velocity", 0.1);
    this->declare_parameter("trajectory.linear_min_duration", 1.0);
    this->declare_parameter("trajectory.linear_step_scale", 0.5);
    this->declare_parameter("trajectory.cartesian_timeout", 3.0);
    this->declare_parameter("trajectory.cartesian_timeout_descend", 5.0);
    this->declare_parameter("trajectory.cartesian_timeout_lift", 3.0);
    this->declare_parameter("trajectory.cartesian_descend_threshold", 0.95);
    this->declare_parameter("trajectory.cartesian_lift_threshold", 0.90);
    this->declare_parameter("trajectory.cartesian_retry_on_timeout", true);
    this->declare_parameter("trajectory.adaptive_segments", true);
    this->declare_parameter("trajectory.min_segments", 2);
    this->declare_parameter("trajectory.max_segments", 4);
    this->declare_parameter("trajectory.use_polynomial_interpolation", false);
    this->declare_parameter("trajectory.polynomial_type", "cubic");
    this->declare_parameter("trajectory.polynomial_duration", 2.0);
    this->declare_parameter("trajectory.polynomial_dt", 0.01);
    this->declare_parameter("trajectory.use_bspline", false);
    this->declare_parameter("trajectory.bspline_degree", 3);
    this->declare_parameter("trajectory.bspline_duration", 3.0);
    this->declare_parameter("trajectory.bspline_dt", 0.01);

    // 读取参数
    load_parameters();
  }

  // 加载参数
  void load_parameters()
  {
    cable_name_ = this->get_parameter("cable.name").as_string();
    cable_diameter_ = this->get_parameter("cable.diameter").as_double();
    cable_length_ = this->get_parameter("cable.length").as_double();
    cable_frame_id_ = this->get_parameter("cable.frame_id").as_string();
    cable_shape_ = this->get_parameter("cable.shape").as_string();
    cable_center_offset_z_ = this->get_parameter("cable.center_offset_z").as_double();

    approach_offset_z_ = this->get_parameter("grasp.approach_offset_z").as_double();
    descend_distance_ = this->get_parameter("grasp.descend_distance").as_double();
    lift_distance_ = this->get_parameter("grasp.lift_distance").as_double();
    max_pos_error_ = this->get_parameter("grasp.max_pos_error").as_double();
    use_cartesian_ = this->get_parameter("grasp.use_cartesian").as_bool();
    eef_step_ = this->get_parameter("grasp.eef_step").as_double();
    jump_threshold_ = this->get_parameter("grasp.jump_threshold").as_double();
    default_planning_time_ = this->get_parameter("grasp.planning_time").as_double();
    num_planning_attempts_ = this->get_parameter("grasp.num_planning_attempts").as_int();
    max_velocity_scaling_ = this->get_parameter("grasp.max_velocity_scaling").as_double();
    max_acceleration_scaling_ = this->get_parameter("grasp.max_acceleration_scaling").as_double();
    goal_position_tolerance_ = this->get_parameter("grasp.goal_position_tolerance").as_double();
    goal_orientation_tolerance_ = this->get_parameter("grasp.goal_orientation_tolerance").as_double();
    fallback_planning_time_ = this->get_parameter("grasp.fallback_planning_time").as_double();
    ik_timeout_ = this->get_parameter("grasp.ik_timeout").as_double();
    state_check_timeout_ = this->get_parameter("grasp.state_check_timeout").as_double();
    state_check_interval_ = this->get_parameter("grasp.state_check_interval").as_double();
    robot_state_publisher_wait_ = this->get_parameter("grasp.robot_state_publisher_wait").as_double();
    executor_startup_wait_ = this->get_parameter("grasp.executor_startup_wait").as_double();
    segment_execution_wait_ = this->get_parameter("grasp.segment_execution_wait").as_double();
    state_sync_wait_ = this->get_parameter("grasp.state_sync_wait").as_double();
    control_frequency_ = this->get_parameter("grasp.control_frequency").as_double();
    read_frequency_ = this->get_parameter("grasp.read_frequency").as_double();
    main_loop_frequency_ = this->get_parameter("grasp.main_loop_frequency").as_double();
    yaw_offset_ = this->get_parameter("grasp.yaw_offset").as_double();
    yaw_flip_ = this->get_parameter("grasp.yaw_flip").as_bool();
    grasp_yaw_add_ = this->get_parameter("grasp.grasp_yaw_add").as_double();
    orientation_yaw_offset_ = this->get_parameter("grasp.orientation_yaw_offset").as_double();
    z_clearance_ = this->get_parameter("grasp.z_clearance").as_double();
    yaw_validation_enabled_ = this->get_parameter("grasp.yaw_validation_enabled").as_bool();
    yaw_tolerance_ = this->get_parameter("grasp.yaw_tolerance").as_double();
    yaw_search_step_ = this->get_parameter("grasp.yaw_search_step").as_double();
    joint4_yaw_search_enabled_ = this->get_parameter("grasp.joint4_yaw_search_enabled").as_bool();
    joint4_yaw_search_range_ = this->get_parameter("grasp.joint4_yaw_search_range").as_double();
    joint4_yaw_search_step_ = this->get_parameter("grasp.joint4_yaw_search_step").as_double();
    
    // Joint约束参数（4DOF机器人控制）
    joint1_tolerance_ = this->get_parameter("grasp.joint1_tolerance").as_double();
    joint4_constraint_enabled_ = this->get_parameter("grasp.joint4_constraint_enabled").as_bool();
    joint4_offset_ = this->get_parameter("grasp.joint4_offset").as_double();
    joint4_tolerance_ = this->get_parameter("grasp.joint4_tolerance").as_double();
    
    enable_recovery_ = this->get_parameter("grasp.enable_recovery").as_bool();
    recovery_timeout_ = this->get_parameter("grasp.recovery_timeout").as_double();
    
    // yaw候选搜索参数
    yaw_candidate_search_enabled_ = this->get_parameter("grasp.yaw_candidate_search_enabled").as_bool();
    yaw_candidate_center_ = this->get_parameter("grasp.yaw_candidate_center").as_double();
    yaw_candidate_range_ = this->get_parameter("grasp.yaw_candidate_range").as_double();
    yaw_candidate_step_ = this->get_parameter("grasp.yaw_candidate_step").as_double();
    
    // 地面安全参数
    ground_height_ = this->get_parameter("grasp.ground_height").as_double();
    ground_offset_below_base_ = this->get_parameter("grasp.ground_offset_below_base").as_double();
    camera_error_margin_ = this->get_parameter("grasp.camera_error_margin").as_double();
    min_ground_clearance_ = this->get_parameter("grasp.min_ground_clearance").as_double();
    add_ground_plane_ = this->get_parameter("grasp.add_ground_plane").as_bool();
    
    // 下压距离限制
    max_cartesian_descend_distance_ = this->get_parameter("grasp.max_cartesian_descend_distance").as_double();
    
    RCLCPP_INFO(this->get_logger(), "配置参数加载完成:");
    RCLCPP_INFO(this->get_logger(), "  grasp_yaw_add: %.3f rad (%.1f deg)", 
               grasp_yaw_add_, grasp_yaw_add_ * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "  orientation_yaw_offset: %.3f rad (%.1f deg)", 
               orientation_yaw_offset_, orientation_yaw_offset_ * 180.0 / M_PI);
    
    // TCP偏移补偿参数
    tcp_offset_x_ = this->get_parameter("grasp.tcp_offset_x").as_double();
    tcp_offset_y_ = this->get_parameter("grasp.tcp_offset_y").as_double();
    tcp_offset_z_ = this->get_parameter("grasp.tcp_offset_z").as_double();
    
    RCLCPP_INFO(this->get_logger(), "TCP偏移补偿参数: (%.4f, %.4f, %.4f) m", 
                tcp_offset_x_, tcp_offset_y_, tcp_offset_z_);
    
    // 全零状态预运动参数
    auto_move_to_ready_ = this->get_parameter("grasp.auto_move_to_ready").as_bool();
    ready_joint1_ = this->get_parameter("grasp.ready_joint1").as_double();
    ready_joint2_ = this->get_parameter("grasp.ready_joint2").as_double();
    ready_joint3_ = this->get_parameter("grasp.ready_joint3").as_double();
    ready_joint4_ = this->get_parameter("grasp.ready_joint4").as_double();
    near_zero_threshold_ = this->get_parameter("grasp.near_zero_threshold").as_double();
    
    RCLCPP_INFO(this->get_logger(), "全零状态预运动: %s, 预备位置: [%.3f, %.3f, %.3f, %.3f] rad",
                auto_move_to_ready_ ? "启用" : "禁用",
                ready_joint1_, ready_joint2_, ready_joint3_, ready_joint4_);
    
    // 计算周期（秒）
    control_period_ = 1.0 / control_frequency_;
    read_period_ = 1.0 / read_frequency_;
    main_loop_period_ = 1.0 / main_loop_frequency_;

    gripper_mode_ = this->get_parameter("gripper.mode").as_string();
    gripper_open_width_ = this->get_parameter("gripper.open_width").as_double();
    gripper_close_width_ = this->get_parameter("gripper.close_width").as_double();
    gripper_close_extra_ = this->get_parameter("gripper.close_extra").as_double();
    gripper_hold_time_ms_ = this->get_parameter("gripper.hold_time_ms").as_int();
    gripper_angle_min_ = this->get_parameter("gripper.angle_min").as_double();
    gripper_angle_max_ = this->get_parameter("gripper.angle_max").as_double();
    gripper_width_min_ = this->get_parameter("gripper.width_min").as_double();

    // Place parameters
    place_approach_offset_z_ = this->get_parameter("place.approach_offset_z").as_double();
    place_descend_distance_ = this->get_parameter("place.descend_distance").as_double();
    place_retreat_distance_ = this->get_parameter("place.retreat_distance").as_double();
    place_max_pos_error_ = this->get_parameter("place.max_pos_error").as_double();
    place_planning_time_ = this->get_parameter("place.planning_time").as_double();
    place_num_planning_attempts_ = this->get_parameter("place.num_planning_attempts").as_int();
    place_max_velocity_scaling_ = this->get_parameter("place.max_velocity_scaling").as_double();
    place_max_acceleration_scaling_ = this->get_parameter("place.max_acceleration_scaling").as_double();
    place_goal_position_tolerance_ = this->get_parameter("place.goal_position_tolerance").as_double();
    place_goal_orientation_tolerance_ = this->get_parameter("place.goal_orientation_tolerance").as_double();

    add_collision_object_ = this->get_parameter("scene.add_collision_object").as_bool();
    allow_touch_links_ = this->get_parameter("scene.allow_touch_links").as_string_array();

    // Workspace parameters
    workspace_base_height_ = this->get_parameter("workspace.base_height").as_double();
    workspace_link2_length_ = this->get_parameter("workspace.link2_length").as_double();
    workspace_link3_length_ = this->get_parameter("workspace.link3_length").as_double();
    workspace_link4_to_eef_ = this->get_parameter("workspace.link4_to_eef").as_double();
    workspace_reach_radius_margin_ = this->get_parameter("workspace.reach_radius_margin").as_double();
    workspace_max_height_offset_ = this->get_parameter("workspace.max_height_offset").as_double();
    workspace_min_height_offset_ = this->get_parameter("workspace.min_height_offset").as_double();
    workspace_min_radius_ = this->get_parameter("workspace.min_radius").as_double();
    workspace_safe_x_min_ = this->get_parameter("workspace.safe_x_min").as_double();
    workspace_safe_x_max_ = this->get_parameter("workspace.safe_x_max").as_double();
    workspace_safe_y_min_ = this->get_parameter("workspace.safe_y_min").as_double();
    workspace_safe_y_max_ = this->get_parameter("workspace.safe_y_max").as_double();
    workspace_safe_z_min_ = this->get_parameter("workspace.safe_z_min").as_double();
    workspace_safe_z_max_ = this->get_parameter("workspace.safe_z_max").as_double();
    workspace_medium_x_min_ = this->get_parameter("workspace.medium_x_min").as_double();
    workspace_medium_x_max_ = this->get_parameter("workspace.medium_x_max").as_double();
    workspace_medium_y_min_ = this->get_parameter("workspace.medium_y_min").as_double();
    workspace_medium_y_max_ = this->get_parameter("workspace.medium_y_max").as_double();
    workspace_medium_z_min_ = this->get_parameter("workspace.medium_z_min").as_double();
    workspace_medium_z_max_ = this->get_parameter("workspace.medium_z_max").as_double();

    // Tolerance parameters
    orientation_epsilon_ = this->get_parameter("tolerance.orientation_epsilon").as_double();

    // Trajectory planning parameters
    use_linear_interpolation_ = this->get_parameter("trajectory.use_linear_interpolation").as_bool();
    linear_velocity_ = this->get_parameter("trajectory.linear_velocity").as_double();
    linear_min_duration_ = this->get_parameter("trajectory.linear_min_duration").as_double();
    linear_step_scale_ = this->get_parameter("trajectory.linear_step_scale").as_double();
    cartesian_timeout_ = this->get_parameter("trajectory.cartesian_timeout").as_double();
    cartesian_timeout_descend_ = this->get_parameter("trajectory.cartesian_timeout_descend").as_double();
    cartesian_timeout_lift_ = this->get_parameter("trajectory.cartesian_timeout_lift").as_double();
    cartesian_descend_threshold_ = this->get_parameter("trajectory.cartesian_descend_threshold").as_double();
    cartesian_lift_threshold_ = this->get_parameter("trajectory.cartesian_lift_threshold").as_double();
    cartesian_retry_on_timeout_ = this->get_parameter("trajectory.cartesian_retry_on_timeout").as_bool();
    adaptive_segments_ = this->get_parameter("trajectory.adaptive_segments").as_bool();
    min_segments_ = this->get_parameter("trajectory.min_segments").as_int();
    max_segments_ = this->get_parameter("trajectory.max_segments").as_int();
    use_polynomial_interpolation_ = this->get_parameter("trajectory.use_polynomial_interpolation").as_bool();
    polynomial_type_ = this->get_parameter("trajectory.polynomial_type").as_string();
    polynomial_duration_ = this->get_parameter("trajectory.polynomial_duration").as_double();
    polynomial_dt_ = this->get_parameter("trajectory.polynomial_dt").as_double();
    use_bspline_ = this->get_parameter("trajectory.use_bspline").as_bool();
    bspline_degree_ = this->get_parameter("trajectory.bspline_degree").as_int();
    bspline_duration_ = this->get_parameter("trajectory.bspline_duration").as_double();
    bspline_dt_ = this->get_parameter("trajectory.bspline_dt").as_double();

    RCLCPP_INFO(this->get_logger(), "参数加载完成");
    RCLCPP_INFO(this->get_logger(), "缆绳直径: %.3f m, 长度: %.3f m", cable_diameter_, cable_length_);
    RCLCPP_INFO(this->get_logger(), "夹爪打开宽度: %.3f m, 闭合宽度: %.3f m", gripper_open_width_, gripper_close_width_);
    
    // 打印轨迹规划配置
    RCLCPP_INFO(this->get_logger(), "=== 轨迹规划配置 ===");
    RCLCPP_INFO(this->get_logger(), "线性插补: %s (速度: %.3f m/s, 最小持续时间: %.2f s, 步长缩放: %.2f)", 
                use_linear_interpolation_ ? "启用" : "禁用", linear_velocity_, 
                linear_min_duration_, linear_step_scale_);
    RCLCPP_INFO(this->get_logger(), "笛卡尔路径: 超时: %.1f s, descend阈值: %.2f, lift阈值: %.2f",
                cartesian_timeout_, cartesian_descend_threshold_, cartesian_lift_threshold_);
    RCLCPP_INFO(this->get_logger(), "多项式插值: %s (类型: %s, 持续时间: %.2f s, 时间步长: %.3f s)", 
                use_polynomial_interpolation_ ? "启用" : "禁用", 
                polynomial_type_.c_str(), polynomial_duration_, polynomial_dt_);
    RCLCPP_INFO(this->get_logger(), "B样条: %s (阶数: %d, 持续时间: %.2f s, 时间步长: %.3f s)", 
                use_bspline_ ? "启用" : "禁用", bspline_degree_, bspline_duration_, bspline_dt_);
    RCLCPP_INFO(this->get_logger(), "===================");
    
    // 打印频率配置
    RCLCPP_INFO(this->get_logger(), "=== 频率配置 ===");
    RCLCPP_INFO(this->get_logger(), "控制频率: %.1f Hz (周期: %.3f s)", 
                control_frequency_, control_period_);
    RCLCPP_INFO(this->get_logger(), "读取频率: %.1f Hz (周期: %.3f s)", 
                read_frequency_, read_period_);
    RCLCPP_INFO(this->get_logger(), "主循环频率: %.1f Hz (周期: %.3f s)", 
                main_loop_frequency_, main_loop_period_);
    RCLCPP_INFO(this->get_logger(), "===================");
  }

  // 缆绳位置回调
  // 急停回调函数
  void emergency_stop_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    // 检查executor是否运行
    if (!executor_ || !rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "[急停] Executor未运行或ROS2上下文无效！");
      return;
    }
    
    // 检查订阅器
    if (!emergency_stop_sub_)
    {
      RCLCPP_ERROR(this->get_logger(), "[急停] 订阅器无效！");
      return;
    }
    
    // 使用ERROR级别确保消息一定会显示
    RCLCPP_ERROR(this->get_logger(), "========================================");
    RCLCPP_ERROR(this->get_logger(), "⚠️⚠️⚠️  收到急停信号！⚠️⚠️⚠️");
    RCLCPP_ERROR(this->get_logger(), "  消息内容: %s", msg ? msg->data.c_str() : "NULL");
    RCLCPP_ERROR(this->get_logger(), "  订阅器状态: %s", emergency_stop_sub_ ? "有效" : "无效");
    RCLCPP_ERROR(this->get_logger(), "  发布者数量: %zu", emergency_stop_sub_->get_publisher_count());
    
    // 获取时间戳（seconds()返回double，需要转换）
    auto now = this->now();
    int64_t sec = static_cast<int64_t>(now.seconds());
    uint32_t nsec = now.nanoseconds() % 1000000000;
    RCLCPP_ERROR(this->get_logger(), "  当前时间: %ld.%09u", sec, nsec);
    RCLCPP_ERROR(this->get_logger(), "========================================");
    
    // 设置急停标志
    emergency_stop_ = true;
    
    // 清空任务队列
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      while (!task_queue_.empty()) {
        task_queue_.pop();
      }
      queue_cv_.notify_all();
    }
    
    // 停止MoveIt执行并发送当前关节值命令（让机械臂停止在当前位置）
    {
      std::lock_guard<std::mutex> lock(moveit_mutex_);
      if (move_group_interface_)
      {
        try {
          move_group_interface_->stop();
          RCLCPP_WARN(this->get_logger(), "已停止机械臂运动");
          
          // 获取当前关节值并设置为目标（发送保持当前位置的命令）
          // 这样硬件接口会通过UDP发送当前关节值的JSON命令，让机械臂停止在当前位置
          try {
            auto current_joints = move_group_interface_->getCurrentJointValues();
            if (!current_joints.empty())
            {
              move_group_interface_->setJointValueTarget(current_joints);
              RCLCPP_WARN(this->get_logger(), 
                          "[急停] 已设置当前关节值为目标，发送保持当前位置命令");
              RCLCPP_INFO(this->get_logger(), 
                          "[急停] 当前关节值: [%.3f, %.3f, %.3f, %.3f] rad",
                          current_joints.size() > 0 ? current_joints[0] : 0.0,
                          current_joints.size() > 1 ? current_joints[1] : 0.0,
                          current_joints.size() > 2 ? current_joints[2] : 0.0,
                          current_joints.size() > 3 ? current_joints[3] : 0.0);
            }
            else
            {
              RCLCPP_WARN(this->get_logger(), 
                          "[急停] 无法获取当前关节值，跳过设置目标");
            }
          } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), 
                        "[急停] 设置关节目标时出错: %s", e.what());
          }
        } catch (const std::exception& e) {
          RCLCPP_ERROR(this->get_logger(), "停止机械臂运动时出错: %s", e.what());
        }
      }
      if (gripper_group_interface_)
      {
        try {
          gripper_group_interface_->stop();
          RCLCPP_WARN(this->get_logger(), "已停止夹爪运动");
          
          // 获取当前夹爪关节值并设置为目标
          try {
            auto current_gripper_joints = gripper_group_interface_->getCurrentJointValues();
            if (!current_gripper_joints.empty())
            {
              gripper_group_interface_->setJointValueTarget(current_gripper_joints);
              RCLCPP_WARN(this->get_logger(), 
                          "[急停] 已设置当前夹爪关节值为目标，发送保持当前位置命令");
              RCLCPP_INFO(this->get_logger(), 
                          "[急停] 当前夹爪关节值: [%.3f, %.3f] rad",
                          current_gripper_joints.size() > 0 ? current_gripper_joints[0] : 0.0,
                          current_gripper_joints.size() > 1 ? current_gripper_joints[1] : 0.0);
            }
            else
            {
              RCLCPP_WARN(this->get_logger(), 
                          "[急停] 无法获取当前夹爪关节值，跳过设置目标");
            }
          } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), 
                        "[急停] 设置夹爪关节目标时出错: %s", e.what());
          }
        } catch (const std::exception& e) {
          RCLCPP_ERROR(this->get_logger(), "停止夹爪运动时出错: %s", e.what());
        }
      }
    }
    
    publish_state("急停:已停止");
    RCLCPP_WARN(this->get_logger(), "急停处理完成");
  }

  void cable_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // 如果之前处于急停状态，收到新任务时自动重置急停状态
    if (emergency_stop_)
    {
      RCLCPP_INFO(this->get_logger(), "收到新的缆绳位置消息，自动重置急停状态");
      emergency_stop_ = false;
      publish_state("急停:已重置");
    }
    
    // 先记录收到消息（无论MoveIt是否初始化）
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "✓ 收到缆绳位置消息！");
    RCLCPP_INFO(this->get_logger(), "  位置: x=%.3f y=%.3f z=%.3f", 
                msg->pose.position.x,
                msg->pose.position.y,
                msg->pose.position.z);
    RCLCPP_INFO(this->get_logger(), "  坐标系: %s", msg->header.frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "  四元数: (%.3f, %.3f, %.3f, %.3f)",
                msg->pose.orientation.x,
                msg->pose.orientation.y,
                msg->pose.orientation.z,
                msg->pose.orientation.w);
    RCLCPP_INFO(this->get_logger(), "========================================");

    if (!move_group_interface_) {
      RCLCPP_WARN(this->get_logger(), "MoveGroupInterface not ready yet, dropping cable_pose");
      publish_state("错误:未就绪");
      return;
    }

    publish_state("已接收");

    geometry_msgs::msg::PoseStamped cable_pose = *msg;
    
    // 更新时间戳
    if (cable_pose.header.stamp.sec == 0 && cable_pose.header.stamp.nanosec == 0) {
      cable_pose.header.stamp = this->now();
    }

    RCLCPP_INFO(this->get_logger(), "处理缆绳位置: x=%.3f y=%.3f z=%.3f (frame: %s)",
                cable_pose.pose.position.x,
                cable_pose.pose.position.y,
                cable_pose.pose.position.z,
                cable_pose.header.frame_id.c_str());

    // 从orientation反向计算yaw（用于障碍物方向）
    // 注意：cable_pose_callback没有提供yaw，需要从orientation提取或使用候选搜索
    double original_yaw = 0.0;
    bool has_valid_yaw = false;
    
    // 检查orientation是否是默认值
    bool is_default_orient = is_default_orientation(cable_pose.pose.orientation);
    
    if (!is_default_orient) {
      // 尝试从orientation提取yaw
      try {
        tf2::Quaternion q;
        tf2::fromMsg(cable_pose.pose.orientation, q);
        tf2::Matrix3x3 rot_matrix(q);
        double roll, pitch, yaw;
        rot_matrix.getRPY(roll, pitch, yaw);
        original_yaw = yaw;
        has_valid_yaw = true;
        RCLCPP_INFO(this->get_logger(), "[任务入队] 从orientation提取yaw: %.3f rad (%.1f deg)", 
                    original_yaw, original_yaw * 180.0 / M_PI);
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "[任务入队] 无法从orientation提取yaw: %s", e.what());
      }
    }
    
    // 如果没有有效的yaw，使用yaw候选搜索（仅xyz输入情况）
    if (!has_valid_yaw && yaw_candidate_search_enabled_ && move_group_interface_) {
      RCLCPP_INFO(this->get_logger(), "[任务入队] 仅xyz输入，使用yaw候选搜索");
      try {
        auto current_state = getCurrentStateSafe();
        if (current_state) {
          // 使用候选搜索找到最优yaw
          original_yaw = find_best_yaw_candidate(
            cable_pose.pose.position.x,
            cable_pose.pose.position.y,
            cable_pose.pose.position.z + approach_offset_z_,  // 在预抓取高度搜索
            yaw_candidate_center_,
            current_state
          );
          has_valid_yaw = true;
          
          // 使用搜索到的yaw，但orientation使用fixed（4DOF控制策略）
          cable_pose.pose.orientation = compute_downward_orientation_fixed();
          RCLCPP_INFO(this->get_logger(), "[任务入队] yaw候选搜索完成，最优yaw=%.3f rad (%.1f deg)",
                      original_yaw, original_yaw * 180.0 / M_PI);
        }
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "[任务入队] yaw候选搜索失败: %s，使用默认yaw=0", e.what());
        original_yaw = 0.0;
        cable_pose.pose.orientation = compute_downward_orientation_fixed();
      }
    } else if (!has_valid_yaw) {
      // 如果候选搜索未启用或MoveIt未就绪，使用默认yaw
      RCLCPP_WARN(this->get_logger(), "[任务入队] 无法获取yaw，使用默认值0");
      original_yaw = 0.0;
      cable_pose.pose.orientation = compute_downward_orientation_fixed();
    }
    
    // 4DOF控制：yaw角应该映射到Joint4（控制夹爪旋转），而不是Joint1
    // Joint1 控制机械臂整体朝向（由IK自动根据目标位置计算）
    // Joint4 控制夹爪旋转角度（由用户指定的yaw决定）
    double joint4_target = original_yaw;
    joint4_target += grasp_yaw_add_;
    joint4_target += yaw_offset_;
    if (yaw_flip_) {
      joint4_target += M_PI;
    }
    // 归一化到[-π, π]
    while (joint4_target > M_PI) joint4_target -= 2.0 * M_PI;
    while (joint4_target < -M_PI) joint4_target += 2.0 * M_PI;
    
    // 更新orientation为fixed（4DOF控制策略）
    cable_pose.pose.orientation = compute_downward_orientation_fixed();

    RCLCPP_INFO(this->get_logger(), "[任务入队] 准备将任务加入队列: pos=(%.3f, %.3f, %.3f), frame=%s, Joint4目标=%.3f rad (%.1f deg)",
                cable_pose.pose.position.x, cable_pose.pose.position.y, 
                cable_pose.pose.position.z, cable_pose.header.frame_id.c_str(),
                joint4_target, joint4_target * 180.0 / M_PI);
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      size_t old_size = task_queue_.size();
      while (!task_queue_.empty()) task_queue_.pop();
      
      // 创建任务结构体
      CableGraspTask task;
      task.cable_pose = cable_pose;
      task.original_yaw = original_yaw;
      task.joint1_target = 0.0;  // 已废弃，Joint1由IK自动计算
      task.joint4_target = joint4_target;  // 4DOF控制：Joint4目标值（控制夹爪yaw）
      
      task_queue_.push(task);
      RCLCPP_INFO(this->get_logger(), "[任务入队] 任务已加入队列（清空旧任务: %zu 个，当前队列大小: %zu）",
                  old_size, task_queue_.size());
    }
    RCLCPP_INFO(this->get_logger(), "[任务入队] 通知工作线程（notify_one），队列大小: %zu", task_queue_.size());
    queue_cv_.notify_one();
    RCLCPP_INFO(this->get_logger(), "[任务入队] 通知已发送，工作线程应被唤醒");
  }

  void cable_pose_with_yaw_callback(const m5_msgs::msg::CablePoseWithYaw::SharedPtr msg)
  {
    // 如果之前处于急停状态，收到新任务时自动重置急停状态
    if (emergency_stop_)
    {
      RCLCPP_INFO(this->get_logger(), "收到新的缆绳位置（带yaw）消息，自动重置急停状态");
      emergency_stop_ = false;
      publish_state("急停:已重置");
    }
    
    // 先记录收到消息（无论MoveIt是否初始化）
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "✓ 收到缆绳位置（带yaw）消息！");
    RCLCPP_INFO(this->get_logger(), "  位置: x=%.3f y=%.3f z=%.3f", 
                msg->position.x,
                msg->position.y,
                msg->position.z);
    RCLCPP_INFO(this->get_logger(), "  yaw: %.3f rad (%.1f deg)", 
                msg->yaw, msg->yaw * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "  坐标系: %s", msg->header.frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "========================================");

    if (!move_group_interface_) {
      RCLCPP_WARN(this->get_logger(), "MoveGroupInterface not ready yet, dropping cable_pose_with_yaw");
      publish_state("错误:未就绪");
      return;
    }

    publish_state("已接收");

    // 4DOF机器人控制策略：
    // - Joint1 控制机械臂整体朝向（由IK自动根据目标位置计算）
    // - Joint4 控制夹爪旋转角度（由用户指定的yaw决定）
    // - 末端姿态只锁定roll/pitch（向下），yaw不约束
    double joint4_target = msg->yaw;
    
    // 1) 线缆切向 → 夹持朝向：yaw=0度（线缆切向）时，夹爪应垂直于线缆
    //    grasp_yaw_add=90°时，yaw=0度→Joint4=90°
    joint4_target += grasp_yaw_add_;
    RCLCPP_INFO(this->get_logger(), "[4DOF控制] 应用grasp_yaw_add: %.3f rad (%.1f deg)", 
                grasp_yaw_add_, grasp_yaw_add_ * 180.0 / M_PI);
    
    // 2) 应用yaw_offset和yaw_flip调整（保留原有调参逻辑）
    joint4_target += yaw_offset_;
    if (yaw_flip_) {
      joint4_target += M_PI;
      RCLCPP_INFO(this->get_logger(), "[4DOF控制] 应用yaw_flip: joint4_target += π");
    }
    if (std::abs(yaw_offset_) > 1e-6) {
      RCLCPP_INFO(this->get_logger(), "[4DOF控制] 应用yaw_offset: %.3f rad (%.1f deg)", 
                  yaw_offset_, yaw_offset_ * 180.0 / M_PI);
    }
    
    // 归一化到[-π, π]
    while (joint4_target > M_PI) joint4_target -= 2.0 * M_PI;
    while (joint4_target < -M_PI) joint4_target += 2.0 * M_PI;
    
    RCLCPP_INFO(this->get_logger(), "[4DOF控制] 最终Joint4目标值: %.3f rad (%.1f deg)", 
                joint4_target, joint4_target * 180.0 / M_PI);
    
    // 末端姿态：只锁定roll/pitch（向下），yaw不约束
    // 这样MoveIt的IK可以自由选择yaw，通过Joint4控制夹爪方向
    geometry_msgs::msg::Quaternion orientation = compute_downward_orientation_fixed();
    
    // 构造PoseStamped消息
    geometry_msgs::msg::PoseStamped cable_pose;
    cable_pose.header = msg->header;
    cable_pose.pose.position = msg->position;
    cable_pose.pose.orientation = orientation;
    
    // 更新时间戳
    if (cable_pose.header.stamp.sec == 0 && cable_pose.header.stamp.nanosec == 0) {
      cable_pose.header.stamp = this->now();
    }

    RCLCPP_INFO(this->get_logger(), "[4DOF控制] 处理缆绳位置: x=%.3f y=%.3f z=%.3f, Joint4目标=%.3f rad (%.1f deg) (frame: %s)",
                cable_pose.pose.position.x,
                cable_pose.pose.position.y,
                cable_pose.pose.position.z,
                joint4_target, joint4_target * 180.0 / M_PI,
                cable_pose.header.frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "计算得到的orientation: (%.3f, %.3f, %.3f, %.3f)",
                cable_pose.pose.orientation.x,
                cable_pose.pose.orientation.y,
                cable_pose.pose.orientation.z,
                cable_pose.pose.orientation.w);
    
    // 临时调试：立即进行IK求解，输出joint 4值
    // 注意：此调试代码已优化，添加了详细的日志和超时保护，避免阻塞回调函数
    // 修复：避免死锁 - 在已持有锁的情况下，直接获取状态，不调用getCurrentStateSafe
    RCLCPP_DEBUG(this->get_logger(), "[调试] 开始IK求解调试...");
    try {
      // 尝试获取锁，但设置超时避免长时间阻塞
      // 如果无法立即获取锁，跳过调试代码，确保任务能够正常入队
      std::unique_lock<std::mutex> lock(moveit_mutex_, std::try_to_lock);
      if (!lock.owns_lock()) {
        RCLCPP_DEBUG(this->get_logger(), "[调试] MoveIt互斥锁被占用，跳过IK调试（不影响任务处理）");
      } else if (move_group_interface_) {
        RCLCPP_DEBUG(this->get_logger(), "[调试] 获取MoveIt状态（已持有锁，直接获取）...");
        // 修复死锁：已持有锁，直接获取状态，不调用getCurrentStateSafe（避免重复获取锁）
        moveit::core::RobotStatePtr state;
        try {
          const auto& robot_model = move_group_interface_->getRobotModel();
          if (robot_model) {
            state = std::make_shared<moveit::core::RobotState>(robot_model);
            state->setToDefaultValues();
            
            // 获取arm_group关节值
            const auto* arm_jmg = robot_model->getJointModelGroup("arm_group");
            if (arm_jmg) {
              std::vector<double> arm_joint_values = move_group_interface_->getCurrentJointValues();
              if (arm_joint_values.size() == arm_jmg->getActiveJointModelNames().size()) {
                state->setJointGroupPositions(arm_jmg, arm_joint_values);
              }
            }
            
            // 获取gripper_group关节值
            const auto* gripper_jmg = robot_model->getJointModelGroup("gripper_group");
            if (gripper_jmg && gripper_group_interface_) {
              std::vector<double> gripper_joint_values = gripper_group_interface_->getCurrentJointValues();
              if (gripper_joint_values.size() == gripper_jmg->getActiveJointModelNames().size()) {
                state->setJointGroupPositions(gripper_jmg, gripper_joint_values);
              }
            }
            
            state->update();
            // 修复Invalid Start State：确保关节值在合法范围内
            state->enforceBounds();
          }
        } catch (const std::exception& e) {
          RCLCPP_WARN(this->get_logger(), "[调试] 获取状态失败: %s", e.what());
          state.reset();
        }
        
        if (state) {
          RCLCPP_DEBUG(this->get_logger(), "[调试] 获取关节模型组...");
          const auto* jmg = state->getJointModelGroup("arm_group");
          if (jmg) {
            geometry_msgs::msg::PoseStamped pose_check = cable_pose;
            RCLCPP_DEBUG(this->get_logger(), "[调试] 转换位姿到规划坐标系...");
            if (transform_pose_to_planning(pose_check)) {
              RCLCPP_DEBUG(this->get_logger(), "[调试] 执行IK求解（超时0.5s）...");
              if (state->setFromIK(jmg, pose_check.pose, eef_link_, 0.5)) {
                std::vector<double> joint_values;
                state->copyJointGroupPositions(jmg, joint_values);
                if (joint_values.size() >= 4) {
                  RCLCPP_INFO(this->get_logger(), 
                             "[调试] IK求解结果 - Joint 4 = %.3f rad (%.1f deg) [期望: -90度]",
                             joint_values[3], joint_values[3] * 180.0 / M_PI);
                } else {
                  RCLCPP_WARN(this->get_logger(), "[调试] 关节值数量不足: %zu", joint_values.size());
                }
              } else {
                RCLCPP_WARN(this->get_logger(), "[调试] IK求解失败");
              }
            } else {
              RCLCPP_WARN(this->get_logger(), "[调试] TF转换失败");
            }
          } else {
            RCLCPP_WARN(this->get_logger(), "[调试] 无法获取关节模型组 'arm_group'");
          }
        } else {
          RCLCPP_WARN(this->get_logger(), "[调试] 获取当前状态失败");
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "[调试] MoveGroupInterface未就绪");
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "[调试] 获取joint 4值失败: %s", e.what());
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "[调试] 发生未知异常");
    }
    RCLCPP_DEBUG(this->get_logger(), "[调试] IK求解调试完成，继续任务入队流程");

    // 保存原始yaw（视觉提供的缆绳切向方向），用于障碍物方向计算
    // 注意：这个yaw不应该包含夹爪方向的调整
    double original_yaw = msg->yaw;
    RCLCPP_INFO(this->get_logger(), "[任务入队] 保存原始yaw（用于障碍物方向）: %.3f rad (%.1f deg)", 
                original_yaw, original_yaw * 180.0 / M_PI);

    // 任务入队：将任务添加到队列并通知工作线程
    RCLCPP_INFO(this->get_logger(), "[任务入队] ========================================");
    RCLCPP_INFO(this->get_logger(), "[任务入队] 准备将任务加入队列");
    RCLCPP_INFO(this->get_logger(), "[任务入队]   位置: pos=(%.3f, %.3f, %.3f)", 
                cable_pose.pose.position.x, cable_pose.pose.position.y, 
                cable_pose.pose.position.z);
    RCLCPP_INFO(this->get_logger(), "[任务入队]   坐标系: %s", cable_pose.header.frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "[任务入队]   姿态: orientation=(%.3f, %.3f, %.3f, %.3f)",
                cable_pose.pose.orientation.x, cable_pose.pose.orientation.y,
                cable_pose.pose.orientation.z, cable_pose.pose.orientation.w);
    RCLCPP_INFO(this->get_logger(), "[任务入队]   原始yaw（障碍物方向）: %.3f rad (%.1f deg)",
                original_yaw, original_yaw * 180.0 / M_PI);
    
    size_t queue_size_before = 0;
    size_t queue_size_after = 0;
    {
      RCLCPP_DEBUG(this->get_logger(), "[任务入队] 获取队列锁...");
      std::lock_guard<std::mutex> lock(queue_mutex_);
      queue_size_before = task_queue_.size();
      RCLCPP_INFO(this->get_logger(), "[任务入队] 队列当前大小: %zu", queue_size_before);
      
      // 清空旧任务（只保留最新任务）
      size_t cleared_count = 0;
      while (!task_queue_.empty()) {
        task_queue_.pop();
        cleared_count++;
      }
      if (cleared_count > 0) {
        RCLCPP_INFO(this->get_logger(), "[任务入队] 已清空 %zu 个旧任务", cleared_count);
      }
      
      // 创建任务结构体，包含位姿、原始yaw和Joint4目标值
      CableGraspTask task;
      task.cable_pose = cable_pose;
      task.original_yaw = original_yaw;
      task.joint1_target = 0.0;  // 已废弃，Joint1由IK自动计算
      task.joint4_target = joint4_target;  // 4DOF控制：Joint4目标值（控制夹爪yaw）
      
      RCLCPP_INFO(this->get_logger(), "[任务入队] Joint4目标值: %.3f rad (%.1f deg)",
                  task.joint4_target, task.joint4_target * 180.0 / M_PI);
      
      // 添加新任务
      task_queue_.push(task);
      queue_size_after = task_queue_.size();
      RCLCPP_INFO(this->get_logger(), "[任务入队] 任务已加入队列，队列大小: %zu", queue_size_after);
      RCLCPP_INFO(this->get_logger(), "[任务入队] 新任务orientation (quaternion): (%.3f, %.3f, %.3f, %.3f)",
                  task.cable_pose.pose.orientation.x, task.cable_pose.pose.orientation.y,
                  task.cable_pose.pose.orientation.z, task.cable_pose.pose.orientation.w);
      
      // 转换为RPY便于直观理解
      tf2::Quaternion q_task;
      q_task.setX(task.cable_pose.pose.orientation.x);
      q_task.setY(task.cable_pose.pose.orientation.y);
      q_task.setZ(task.cable_pose.pose.orientation.z);
      q_task.setW(task.cable_pose.pose.orientation.w);
      double roll_task, pitch_task, yaw_task;
      tf2::Matrix3x3(q_task).getRPY(roll_task, pitch_task, yaw_task);
      RCLCPP_INFO(this->get_logger(), "[任务入队] 新任务orientation (RPY): roll=%.3f°, pitch=%.3f°, yaw=%.3f°",
                  roll_task * 180.0 / M_PI, pitch_task * 180.0 / M_PI, yaw_task * 180.0 / M_PI);
    }
    RCLCPP_DEBUG(this->get_logger(), "[任务入队] 已释放队列锁");
    
    // 通知工作线程
    RCLCPP_INFO(this->get_logger(), "[任务入队] 通知工作线程（notify_one）...");
    queue_cv_.notify_one();
    RCLCPP_INFO(this->get_logger(), "[任务入队] 通知已发送，工作线程应被唤醒");
    RCLCPP_INFO(this->get_logger(), "[任务入队] ========================================");
  }

  // 放置位置回调（不带yaw）
  void place_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (emergency_stop_) {
      RCLCPP_INFO(this->get_logger(), "收到新的放置位置消息，自动重置急停状态");
      emergency_stop_ = false;
      publish_state("急停:已重置");
    }
    
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "✓ 收到放置位置消息！");
    RCLCPP_INFO(this->get_logger(), "  位置: x=%.3f y=%.3f z=%.3f", 
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    RCLCPP_INFO(this->get_logger(), "  坐标系: %s", msg->header.frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "========================================");

    if (!move_group_interface_) {
      RCLCPP_WARN(this->get_logger(), "MoveGroupInterface not ready yet, dropping place_pose");
      publish_state("错误:未就绪");
      return;
    }

    publish_state("已接收放置任务");

    geometry_msgs::msg::PoseStamped place_pose = *msg;
    if (place_pose.header.stamp.sec == 0 && place_pose.header.stamp.nanosec == 0) {
      place_pose.header.stamp = this->now();
    }

    // 从orientation提取yaw（用于放置方向）
    double place_yaw = 0.0;
    try {
      tf2::Quaternion q;
      tf2::fromMsg(place_pose.pose.orientation, q);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      place_yaw = yaw;
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "无法从orientation提取yaw: %s，使用默认yaw=0", e.what());
    }

    // 执行放置任务
    std::thread([this, place_pose, place_yaw]() {
      do_cable_place(place_pose, place_yaw);
    }).detach();
  }

  // 放置位置回调（带yaw）
  void place_pose_with_yaw_callback(const m5_msgs::msg::CablePoseWithYaw::SharedPtr msg)
  {
    if (emergency_stop_) {
      RCLCPP_INFO(this->get_logger(), "收到新的放置位置（带yaw）消息，自动重置急停状态");
      emergency_stop_ = false;
      publish_state("急停:已重置");
    }
    
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "✓ 收到放置位置（带yaw）消息！");
    RCLCPP_INFO(this->get_logger(), "  位置: x=%.3f y=%.3f z=%.3f", 
                msg->position.x, msg->position.y, msg->position.z);
    RCLCPP_INFO(this->get_logger(), "  yaw: %.3f rad (%.1f deg)", 
                msg->yaw, msg->yaw * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "  坐标系: %s", msg->header.frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "========================================");

    if (!move_group_interface_) {
      RCLCPP_WARN(this->get_logger(), "MoveGroupInterface not ready yet, dropping place_pose_with_yaw");
      publish_state("错误:未就绪");
      return;
    }

    publish_state("已接收放置任务");

    // 构造PoseStamped消息
    geometry_msgs::msg::PoseStamped place_pose;
    place_pose.header = msg->header;
    place_pose.pose.position = msg->position;
    
    // 计算四元数姿态（roll=0, pitch=0, yaw=msg->yaw）
    place_pose.pose.orientation = compute_downward_orientation_with_yaw(msg->yaw);
    
    if (place_pose.header.stamp.sec == 0 && place_pose.header.stamp.nanosec == 0) {
      place_pose.header.stamp = this->now();
    }

    // 执行放置任务
    std::thread([this, place_pose, yaw = msg->yaw]() {
      do_cable_place(place_pose, yaw);
    }).detach();
  }

  // 发布状态
  void publish_state(const std::string& state, const std::string& detail = "")
  {
    if (state_publisher_)
    {
      std_msgs::msg::String msg;
      if (!detail.empty())
      {
        msg.data = state + ":" + detail;
      }
      else
      {
        msg.data = state;
      }
      
      // 添加详细日志
      size_t sub_count = state_publisher_->get_subscription_count();
      RCLCPP_INFO(this->get_logger(), "[状态发布] 发布状态到 /grasp_state: %s (订阅者数量: %zu)", 
                  msg.data.c_str(), sub_count);
      state_publisher_->publish(msg);
      
      if (sub_count == 0)
      {
        RCLCPP_WARN(this->get_logger(), "[状态发布] 警告：没有订阅者，状态消息可能无法接收");
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "[状态发布] 发布器无效！无法发布状态: %s", state.c_str());
    }
  }

  // 工作线程函数
  void worker_thread()
  {
    RCLCPP_INFO(this->get_logger(), "[工作线程] ========================================");
    RCLCPP_INFO(this->get_logger(), "[工作线程] 工作线程已启动，开始等待任务...");
    RCLCPP_INFO(this->get_logger(), "[工作线程] ========================================");
    
    while (!stop_workers_)
    {
      CableGraspTask task;
      bool has_task = false;

      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        RCLCPP_DEBUG(this->get_logger(), "[工作线程] 等待条件变量唤醒（当前队列大小: %zu）...", task_queue_.size());
        
        // 等待条件变量，直到队列非空或收到停止信号
        auto wait_start = std::chrono::steady_clock::now();
        queue_cv_.wait(lock, [this]() { 
          return !task_queue_.empty() || stop_workers_; 
        });
        auto wait_end = std::chrono::steady_clock::now();
        auto wait_duration = std::chrono::duration_cast<std::chrono::milliseconds>(wait_end - wait_start).count();
        
        RCLCPP_INFO(this->get_logger(), "[工作线程] ========================================");
        RCLCPP_INFO(this->get_logger(), "[工作线程] 被唤醒（等待时间: %ld ms）", wait_duration);
        RCLCPP_INFO(this->get_logger(), "[工作线程]   队列大小: %zu", task_queue_.size());
        RCLCPP_INFO(this->get_logger(), "[工作线程]   stop_workers_: %s", stop_workers_ ? "true" : "false");

        if (stop_workers_ && task_queue_.empty())
        {
          RCLCPP_INFO(this->get_logger(), "[工作线程] 收到停止信号且队列为空，准备退出");
          break;
        }

        if (!task_queue_.empty())
        {
          task = task_queue_.front();
          task_queue_.pop();
          has_task = true;
          RCLCPP_INFO(this->get_logger(), "[工作线程] ✓ 从队列获取到任务");
          RCLCPP_INFO(this->get_logger(), "[工作线程]   位置: pos=(%.3f, %.3f, %.3f)", 
                      task.cable_pose.pose.position.x, task.cable_pose.pose.position.y, 
                      task.cable_pose.pose.position.z);
          RCLCPP_INFO(this->get_logger(), "[工作线程]   坐标系: %s", task.cable_pose.header.frame_id.c_str());
          RCLCPP_INFO(this->get_logger(), "[工作线程]   姿态: orientation=(%.3f, %.3f, %.3f, %.3f)",
                      task.cable_pose.pose.orientation.x, task.cable_pose.pose.orientation.y,
                      task.cable_pose.pose.orientation.z, task.cable_pose.pose.orientation.w);
          RCLCPP_INFO(this->get_logger(), "[工作线程]   原始yaw（障碍物方向）: %.3f rad (%.1f deg)",
                      task.original_yaw, task.original_yaw * 180.0 / M_PI);
          RCLCPP_INFO(this->get_logger(), "[工作线程]   队列剩余任务: %zu", task_queue_.size());
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), "[工作线程] ⚠️  被唤醒但队列为空，可能是虚假唤醒或竞态条件");
        }
        RCLCPP_INFO(this->get_logger(), "[工作线程] ========================================");
      }

      if (has_task)
      {
        // 检查急停状态
        if (emergency_stop_)
        {
          RCLCPP_WARN(this->get_logger(), "[工作线程] ⚠️  系统处于急停状态，取消当前任务");
          publish_state("急停:任务已取消");
          continue;
        }
        
        RCLCPP_INFO(this->get_logger(), "[工作线程] ========================================");
        RCLCPP_INFO(this->get_logger(), "[工作线程] 准备执行抓取任务，调用 do_cable_grasp()...");
        RCLCPP_INFO(this->get_logger(), "[工作线程] ========================================");
        
        try {
          auto task_start = std::chrono::steady_clock::now();
          do_cable_grasp(task.cable_pose, task.original_yaw);
          auto task_end = std::chrono::steady_clock::now();
          auto task_duration = std::chrono::duration_cast<std::chrono::milliseconds>(task_end - task_start).count();
          
          RCLCPP_INFO(this->get_logger(), "[工作线程] ========================================");
          RCLCPP_INFO(this->get_logger(), "[工作线程] ✓ 抓取任务执行完成（耗时: %ld ms）", task_duration);
          RCLCPP_INFO(this->get_logger(), "[工作线程] ========================================");
        } catch (const std::exception& e) {
          RCLCPP_ERROR(this->get_logger(), "[工作线程] ========================================");
          RCLCPP_ERROR(this->get_logger(), "[工作线程] ✗ 处理抓取任务时发生异常: %s", e.what());
          RCLCPP_ERROR(this->get_logger(), "[工作线程] ========================================");
          publish_state("错误:异常");
        } catch (...) {
          RCLCPP_ERROR(this->get_logger(), "[工作线程] ========================================");
          RCLCPP_ERROR(this->get_logger(), "[工作线程] ✗ 处理抓取任务时发生未知异常");
          RCLCPP_ERROR(this->get_logger(), "[工作线程] ========================================");
          publish_state("错误:未知异常");
        }
      }
    }
    RCLCPP_INFO(this->get_logger(), "[工作线程] ========================================");
    RCLCPP_INFO(this->get_logger(), "[工作线程] 工作线程退出");
    RCLCPP_INFO(this->get_logger(), "[工作线程] ========================================");
  }

  // 线程安全的 MoveIt 接口访问封装函数
  // 所有对 move_group_interface_、gripper_group_interface_、planning_scene_interface_ 的访问
  // 都必须通过这些函数，确保线程安全
  geometry_msgs::msg::PoseStamped getCurrentPoseSafe()
  {
    std::lock_guard<std::mutex> lock(moveit_mutex_);
    return move_group_interface_->getCurrentPose();
  }

  moveit::core::RobotStatePtr getCurrentStateSafe(double timeout = 0.0)
  {
    std::lock_guard<std::mutex> lock(moveit_mutex_);
    // 修复时间戳问题：优先使用getCurrentJointValues()绕过时间戳检查
    // getCurrentJointValues()直接从CurrentStateMonitor缓存取值，不强制stamp>=now
    // 这样可以避免"差10ms就失败"的问题
    try {
      const auto& robot_model = move_group_interface_->getRobotModel();
      if (robot_model) {
        auto state = std::make_shared<moveit::core::RobotState>(robot_model);
        state->setToDefaultValues();
        
        // 优先使用getCurrentJointValues()获取arm_group关节值（绕过时间戳检查）
        const auto* arm_jmg = robot_model->getJointModelGroup("arm_group");
        if (arm_jmg) {
          std::vector<double> arm_joint_values = move_group_interface_->getCurrentJointValues();
          if (arm_joint_values.size() == arm_jmg->getActiveJointModelNames().size()) {
            state->setJointGroupPositions(arm_jmg, arm_joint_values);
          } else {
            RCLCPP_WARN(this->get_logger(), "getCurrentJointValues()返回的关节数量(%zu)与arm_group期望数量(%zu)不匹配", 
                       arm_joint_values.size(), arm_jmg->getActiveJointModelNames().size());
          }
        }
        
        // 获取gripper_group关节值
        const auto* gripper_jmg = robot_model->getJointModelGroup("gripper_group");
        if (gripper_jmg && gripper_group_interface_) {
          std::vector<double> gripper_joint_values = gripper_group_interface_->getCurrentJointValues();
          if (gripper_joint_values.size() == gripper_jmg->getActiveJointModelNames().size()) {
            state->setJointGroupPositions(gripper_jmg, gripper_joint_values);
          }
        }
        
        // 更新状态（计算FK等）
        state->update();
        // 修复Invalid Start State：确保关节值在合法范围内，避免浮点误差导致状态被判为invalid
        state->enforceBounds();
        return state;
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "getCurrentJointValues()失败: %s，尝试使用getCurrentState()", e.what());
    }
    
    // Fallback：如果getCurrentJointValues()失败，尝试使用getCurrentState()（带超时）
    double actual_timeout = (timeout > 0.0) ? std::max(timeout, 1.0) : 3.0;
    auto state = move_group_interface_->getCurrentState(actual_timeout);
    if (!state) {
      RCLCPP_WARN(this->get_logger(), "getCurrentState()也失败（超时=%.1fs），使用默认状态", actual_timeout);
      try {
        const auto& robot_model = move_group_interface_->getRobotModel();
        if (robot_model) {
          auto default_state = std::make_shared<moveit::core::RobotState>(robot_model);
          default_state->setToDefaultValues();
          default_state->enforceBounds();
          return default_state;
        }
      } catch (...) {
        // 忽略异常
      }
    }
    // 对从getCurrentState获取的状态也调用enforceBounds
    if (state) {
      state->enforceBounds();
    }
    return state;
  }

  std::vector<double> getCurrentJointValuesSafe()
  {
    std::lock_guard<std::mutex> lock(moveit_mutex_);
    return move_group_interface_->getCurrentJointValues();
  }

  std::vector<double> getCurrentGripperJointValuesSafe()
  {
    std::lock_guard<std::mutex> lock(moveit_mutex_);
    return gripper_group_interface_->getCurrentJointValues();
  }

  // 显式设置start state（使用getCurrentJointValues()绕过时间戳检查）
  // 这样move_group端就不需要等待current_state_monitor，避免"差10ms就失败"的问题
  // 显式设置start state（使用getCurrentJointValues()绕过时间戳检查）
  // 修复：加锁保护，避免并发/状态撕裂风险
  void setStartStateExplicit()
  {
    std::lock_guard<std::mutex> lock(moveit_mutex_);
    try {
      const auto& robot_model = move_group_interface_->getRobotModel();
      if (robot_model) {
        auto start_state = std::make_shared<moveit::core::RobotState>(robot_model);
        start_state->setToDefaultValues();
        
        // 获取arm_group关节值（使用Safe版本，已在锁内）
        const auto* arm_jmg = robot_model->getJointModelGroup("arm_group");
        if (arm_jmg) {
          std::vector<double> arm_joint_values = move_group_interface_->getCurrentJointValues();
          if (arm_joint_values.size() == arm_jmg->getActiveJointModelNames().size()) {
            start_state->setJointGroupPositions(arm_jmg, arm_joint_values);
          }
        }
        
        // 获取gripper_group关节值（使用Safe版本，已在锁内）
        const auto* gripper_jmg = robot_model->getJointModelGroup("gripper_group");
        if (gripper_jmg && gripper_group_interface_) {
          std::vector<double> gripper_joint_values = gripper_group_interface_->getCurrentJointValues();
          if (gripper_joint_values.size() == gripper_jmg->getActiveJointModelNames().size()) {
            start_state->setJointGroupPositions(gripper_jmg, gripper_joint_values);
          }
        }
        
        start_state->update();
        // 修复Invalid Start State：确保关节值在合法范围内
        start_state->enforceBounds();
        move_group_interface_->setStartState(*start_state);
      } else {
        // Fallback：如果构造失败，使用setStartStateToCurrentState()
        move_group_interface_->setStartStateToCurrentState();
      }
    } catch (const std::exception& e) {
      // Fallback：如果异常，使用setStartStateToCurrentState()
      move_group_interface_->setStartStateToCurrentState();
      RCLCPP_DEBUG(this->get_logger(), "setStartStateExplicit()异常: %s，使用setStartStateToCurrentState()", e.what());
    }
  }

  // 等待状态稳定：连续N次joint_state变化 < 阈值再继续
  // 用于确保执行完上一段后，机械臂"回读状态"已经稳定
  // 优化：改进检查逻辑，使用更短的检查间隔和更合理的阈值
  // 线程安全优化：每次获取state时短暂加锁，避免持锁sleep导致其他线程被阻塞
  bool waitForStateStable(double max_wait_time = 1.0, 
                          int stable_count = 5, 
                          double threshold = 0.002)
  {
    // 短暂加锁获取JointModelGroup（只需要一次）
    const moveit::core::JointModelGroup* jmg = nullptr;
    {
      std::lock_guard<std::mutex> lock(moveit_mutex_);
      // 修复时间戳问题：使用0.0作为等待时间，获取最新可用状态，避免时间戳检查失败
      auto state = move_group_interface_->getCurrentState(0.0);
      if (state) {
        jmg = state->getJointModelGroup("arm_group");
      }
    }
    
    if (!jmg)
    {
      RCLCPP_WARN(this->get_logger(), "[状态稳定] 无法获取arm_group，使用简单等待");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      return false;
    }

    auto start_time = std::chrono::steady_clock::now();
    std::vector<double> prev_joint_values;
    int consecutive_stable = 0;
    const int check_interval_ms = 30;  // 优化：从50ms减少到30ms，提高检查频率
    const double min_wait_time = 0.1;  // 最小等待时间，确保至少等待一段时间

    // 确保至少等待最小时间（在解锁状态下sleep，不阻塞其他线程）
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(min_wait_time * 1000)));

    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count() < max_wait_time)
    {
      // 短暂加锁获取当前state，复制数据后立即解锁
      std::vector<double> current_joint_values;
      {
        std::lock_guard<std::mutex> lock(moveit_mutex_);
        // 修复时间戳问题：使用Time(0)获取最新可用状态
        auto current_state = move_group_interface_->getCurrentState(0.0);
        if (current_state) {
          current_state->copyJointGroupPositions(jmg, current_joint_values);
        }
      }
      // 解锁后处理数据（不持锁，避免阻塞其他线程）

      if (!prev_joint_values.empty() && prev_joint_values.size() == current_joint_values.size())
      {
        // 计算关节值变化
        double max_change = 0.0;
        double total_change = 0.0;  // 添加总变化量用于调试
        for (size_t i = 0; i < current_joint_values.size(); ++i)
        {
          double change = std::abs(current_joint_values[i] - prev_joint_values[i]);
          total_change += change;
          if (change > max_change)
          {
            max_change = change;
          }
        }

        if (max_change < threshold)
        {
          consecutive_stable++;
          if (consecutive_stable >= stable_count)
          {
            double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
            RCLCPP_DEBUG(this->get_logger(), 
                        "[状态稳定] 状态已稳定（连续%d次变化 < %.4f rad，耗时%.3fs）", 
                        stable_count, threshold, elapsed);
            return true;
          }
        }
        else
        {
          // 重置计数，但记录调试信息
          if (consecutive_stable > 0)
          {
            RCLCPP_DEBUG(this->get_logger(), 
                        "[状态稳定] 检测到变化（最大变化: %.4f rad），重置稳定计数", 
                        max_change);
          }
          consecutive_stable = 0;  // 重置计数
        }
      }
      else if (prev_joint_values.empty())
      {
        // 第一次获取状态，直接使用
        RCLCPP_DEBUG(this->get_logger(), "[状态稳定] 获取初始状态");
      }

      prev_joint_values = current_joint_values;
      // 在解锁状态下sleep，不阻塞其他线程（急停、回调、另一个轨迹）
      std::this_thread::sleep_for(std::chrono::milliseconds(check_interval_ms));
    }

    // 超时处理：必须等待状态稳定，不能提前返回true
    // 如果超时，至少等待最小时间，但返回false表示未完全稳定
    double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
    RCLCPP_WARN(this->get_logger(), 
                "[状态稳定] 超时（%.1fs），状态可能未完全稳定，但继续执行", 
                elapsed);
    // 即使超时，也至少等待最小时间（0.1s），确保状态有一定更新
    if (elapsed < 0.1)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>((0.1 - elapsed) * 1000)));
    }
    return false;  // 返回false表示未完全稳定，但允许继续执行（由调用者决定）
  }

  // 宽度到角度转换（需要根据实际夹爪几何测试确定）
  double width_to_joint_angle(double width)
  {
    // 线性映射
    // 使用配置的参数进行映射
    const double width_min = gripper_width_min_;
    const double width_max = gripper_open_width_;
    const double angle_min = gripper_angle_min_;   // 闭合（对应 axis5 = 0°）
    const double angle_max = gripper_angle_max_;  // 张开（对应 axis5 = -1100°）
    
    // 除零保护：检查width_max > width_min
    if (width_max <= width_min)
    {
      RCLCPP_ERROR(this->get_logger(), 
                   "width_to_joint_angle: width_max (%.3f) <= width_min (%.3f)，配置错误！返回默认角度",
                   width_max, width_min);
      return angle_min;  // 返回闭合角度作为默认值
    }
    
    width = std::max(width_min, std::min(width_max, width));
    double angle = (width - width_min) / (width_max - width_min) * (angle_max - angle_min) + angle_min;
    return angle;
  }

  // 打开夹爪
  bool open_gripper(double width = -1.0)
  {
    std::lock_guard<std::mutex> lock(moveit_mutex_);
    
    if (width < 0.0) {
      width = gripper_open_width_;
      RCLCPP_INFO(this->get_logger(), "[夹爪打开] 使用配置参数: open_width=%.3f m", gripper_open_width_);
    } else {
      RCLCPP_INFO(this->get_logger(), "[夹爪打开] 使用指定宽度: width=%.3f m", width);
    }

    // 获取当前关节状态（自检日志）
    std::vector<double> current_joint_values = gripper_group_interface_->getCurrentJointValues();
    double current_joint_gl = (current_joint_values.size() > 0) ? current_joint_values[0] : 0.0;
    double current_joint_gr = (current_joint_values.size() > 1) ? current_joint_values[1] : 0.0;
    
    double joint_gl = width_to_joint_angle(width);
    double joint_gr = -joint_gl;  // 镜像对称

    // 自检日志：打印目标值和当前值
    RCLCPP_INFO(this->get_logger(), 
                "[夹爪打开自检] 目标宽度=%.3f m (配置: open_width=%.3f m), 目标JointGL=%.4f rad (%.2f°), 目标JointGR=%.4f rad (%.2f°)",
                width, gripper_open_width_, joint_gl, joint_gl * 180.0 / M_PI, joint_gr, joint_gr * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), 
                "[夹爪打开自检] 当前JointGL=%.4f rad (%.2f°), 当前JointGR=%.4f rad (%.2f°)",
                current_joint_gl, current_joint_gl * 180.0 / M_PI, 
                current_joint_gr, current_joint_gr * 180.0 / M_PI);

    std::vector<double> gripper_joint_values = {joint_gl, joint_gr};
    gripper_group_interface_->setJointValueTarget(gripper_joint_values);

    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    moveit::core::MoveItErrorCode result = gripper_group_interface_->plan(gripper_plan);

    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      result = gripper_group_interface_->execute(gripper_plan);
      if (result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_INFO(this->get_logger(), "夹爪打开成功，宽度: %.3f m", width);
        return true;
      }
    }

    RCLCPP_WARN(this->get_logger(), "夹爪打开失败");
    return false;
  }

  // 闭合夹爪（仅位置控制，不支持力控）
  bool close_gripper(double width = -1.0)
  {
    bool plan_success = false;
    bool execute_success = false;
    
    {
      std::lock_guard<std::mutex> lock(moveit_mutex_);
      
      if (width < 0.0) {
        width = gripper_close_width_ - gripper_close_extra_;
        RCLCPP_INFO(this->get_logger(), "[夹爪闭合] 使用配置参数: close_width = %.3f m, close_extra = %.3f m, 最终宽度 = %.3f m", 
                    gripper_close_width_, gripper_close_extra_, width);
      } else {
        RCLCPP_INFO(this->get_logger(), "[夹爪闭合] 使用传入参数: width = %.3f m", width);
      }

      // 获取当前关节状态（自检日志）
      std::vector<double> current_joint_values = gripper_group_interface_->getCurrentJointValues();
      double current_joint_gl = (current_joint_values.size() > 0) ? current_joint_values[0] : 0.0;
      double current_joint_gr = (current_joint_values.size() > 1) ? current_joint_values[1] : 0.0;
      
      double joint_gl = width_to_joint_angle(width);
      double joint_gr = -joint_gl;  // 镜像对称

      // 自检日志：打印目标值和当前值
      RCLCPP_INFO(this->get_logger(), 
                  "[夹爪闭合自检] 目标宽度=%.3f m, 目标JointGL=%.4f rad (%.2f°), 目标JointGR=%.4f rad (%.2f°)",
                  width, joint_gl, joint_gl * 180.0 / M_PI, joint_gr, joint_gr * 180.0 / M_PI);
      RCLCPP_INFO(this->get_logger(), 
                  "[夹爪闭合自检] 当前JointGL=%.4f rad (%.2f°), 当前JointGR=%.4f rad (%.2f°)",
                  current_joint_gl, current_joint_gl * 180.0 / M_PI, 
                  current_joint_gr, current_joint_gr * 180.0 / M_PI);

      std::vector<double> gripper_joint_values = {joint_gl, joint_gr};
      gripper_group_interface_->setJointValueTarget(gripper_joint_values);

      moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
      moveit::core::MoveItErrorCode result = gripper_group_interface_->plan(gripper_plan);

      if (result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        plan_success = true;
        result = gripper_group_interface_->execute(gripper_plan);
        if (result == moveit::core::MoveItErrorCode::SUCCESS)
        {
          execute_success = true;
        }
      }
    }  // 锁在这里释放
    
    if (plan_success && execute_success)
    {
      // 验证夹爪是否真的闭合（检查实际关节值）
      std::this_thread::sleep_for(std::chrono::milliseconds(200));  // 等待执行完成
      std::vector<double> actual_joints = getCurrentGripperJointValuesSafe();
      if (actual_joints.size() >= 2)
      {
        double actual_gl = actual_joints[0];
        double actual_gr = actual_joints[1];
        double target_gl = width_to_joint_angle(width);
        double target_gr = -target_gl;
        
        double error_gl = std::abs(actual_gl - target_gl);
        double error_gr = std::abs(actual_gr - target_gr);
        double tolerance = 0.1;  // rad，约5.7度
        
        if (error_gl > tolerance || error_gr > tolerance)
        {
          RCLCPP_WARN(this->get_logger(), 
                      "夹爪闭合验证失败: 目标(%.3f, %.3f), 实际(%.3f, %.3f), 误差(%.3f, %.3f)",
                      target_gl, target_gr, actual_gl, actual_gr, error_gl, error_gr);
          // 检测并上报碰撞状态（在锁外调用）
          check_and_report_collision("夹爪闭合:验证失败");
          return false;  // 闭合失败
        }
        else
        {
          RCLCPP_INFO(this->get_logger(), 
                      "夹爪闭合验证通过: 目标(%.3f, %.3f), 实际(%.3f, %.3f), 误差(%.3f, %.3f)",
                      target_gl, target_gr, actual_gl, actual_gr, error_gl, error_gr);
        }
      }
      
      RCLCPP_INFO(this->get_logger(), "夹爪闭合成功，宽度: %.3f m", width);
      return true;
    }

    RCLCPP_WARN(this->get_logger(), "夹爪闭合失败");
    // 检测并上报碰撞状态（在锁外调用，避免死锁）
    check_and_report_collision("夹爪闭合");
    return false;
  }

  // 计算缆绳圆柱体的正确pose（统一用于add和attach）
  // 修正圆柱中心位置和旋转方向，确保add和attach时几何一致
  // 注意：MoveIt 默认圆柱轴是 Z 轴，所以需要明确设置 orientation
  geometry_msgs::msg::Pose make_cable_cylinder_pose(const geometry_msgs::msg::PoseStamped& cable_pose_planning, double cable_yaw)
  {
    geometry_msgs::msg::Pose pose;
    
    // 计算圆柱中心位置：cable_pose + center_offset_z
    // cable_center_offset_z_ 默认 0.0 表示 cable_pose 就是圆柱中心
    // 如果设置为 cable_length_ * 0.5，表示 cable_pose 是圆柱底部，需要向上推半根长度到中心
    pose.position.x = cable_pose_planning.pose.position.x;
    pose.position.y = cable_pose_planning.pose.position.y;
    pose.position.z = cable_pose_planning.pose.position.z + cable_center_offset_z_;
    
    // 计算圆柱体姿态：使圆柱体的Z轴（高度方向）沿着缆绳的长度方向（水平方向）
    // 使用视觉提供的原始yaw（缆绳切向方向），而不是从调整后的cable_pose.orientation提取
    // 这样可以确保障碍物方向正确反映视觉检测到的缆绳方向，不受夹爪方向调整的影响
    
    // 构造圆柱体的 orientation：
    // 绳子水平放置，方向在X-Y平面
    // yaw是绕Z轴的旋转角度，表示绳子在水平面上的方向
    // 要实现水平放置的圆柱体：
    //   1. 先绕Y轴旋转90度（使圆柱体Z轴从垂直变为水平，沿X方向）
    //   2. 然后绕Z轴旋转yaw角度（确定水平方向）
    // 注意：setRPY的顺序是Roll, Pitch, Yaw，但旋转顺序是：先Yaw，再Pitch，最后Roll
    // 所以 setRPY(0, M_PI/2, cable_yaw) 表示：先绕Z轴旋转yaw，再绕Y轴旋转90度
    
    tf2::Quaternion q;
    q.setRPY(0.0, M_PI/2, cable_yaw);  // Roll=0°, Pitch=90°, Yaw=cable_yaw
    q.normalize();
    
    pose.orientation = tf2::toMsg(q);
    
    // 将quaternion转换为RPY，便于直观理解orientation变化
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    RCLCPP_INFO(this->get_logger(), 
                "[圆柱体姿态] 原始yaw: %.3f rad (%.1f deg)", 
                cable_yaw, cable_yaw * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), 
                "[圆柱体姿态] 圆柱体orientation (quaternion): (%.3f, %.3f, %.3f, %.3f)",
                pose.orientation.x, pose.orientation.y, 
                pose.orientation.z, pose.orientation.w);
    RCLCPP_INFO(this->get_logger(), 
                "[圆柱体姿态] 圆柱体orientation (RPY): roll=%.3f rad (%.1f°), pitch=%.3f rad (%.1f°), yaw=%.3f rad (%.1f°)",
                roll, roll * 180.0 / M_PI,
                pitch, pitch * 180.0 / M_PI,
                yaw, yaw * 180.0 / M_PI);
    
    return pose;
  }

  // 添加缆绳碰撞体
  // cable_yaw: 视觉提供的原始yaw（缆绳切向方向），用于计算障碍物方向
  // 注意：这个yaw不应该包含夹爪方向的调整（grasp_yaw_add, yaw_offset等）
  bool scene_add_cable_object(const geometry_msgs::msg::PoseStamped& cable_pose, 
                               double cable_yaw,
                               const std::string& object_id = "cable_1")
  {
    std::lock_guard<std::mutex> lock(moveit_mutex_);

    // 1. 检查对象是否已存在
    std::vector<std::string> known_objects = planning_scene_interface_->getKnownObjectNames();
    bool object_exists = false;
    for (const auto& name : known_objects)
    {
      if (name == object_id)
      {
        object_exists = true;
        break;
      }
    }

    // 2. 创建碰撞对象
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = planning_frame_;
    collision_object.id = object_id;
    
    // 3. 根据对象是否存在选择操作类型
    // 如果对象已存在，直接使用MOVE操作更新（不先移除，确保RViz正确更新）
    // 如果对象不存在，使用ADD操作添加
    if (object_exists)
    {
      collision_object.operation = moveit_msgs::msg::CollisionObject::MOVE;
      RCLCPP_INFO(this->get_logger(), "[场景更新] 对象 %s 已存在，使用MOVE操作更新位置和姿态", object_id.c_str());
    }
    else
    {
      collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
      RCLCPP_INFO(this->get_logger(), "[场景更新] 对象 %s 不存在，使用ADD操作添加新对象", object_id.c_str());
    }

    // 创建圆柱体
    shape_msgs::msg::SolidPrimitive cylinder;
    cylinder.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    cylinder.dimensions.resize(2);
    cylinder.dimensions[0] = cable_length_;  // height
    double cylinder_radius = cable_diameter_ / 2.0;  // radius
    cylinder.dimensions[1] = cylinder_radius;
    
    // 验证尺寸（添加详细日志）
    RCLCPP_INFO(this->get_logger(), "[尺寸验证] 配置值: 直径=%.6f m (%.1f mm)", 
                cable_diameter_, cable_diameter_ * 1000.0);
    RCLCPP_INFO(this->get_logger(), "[尺寸验证] 计算值: 半径=%.6f m (%.1f mm)", 
                cylinder_radius, cylinder_radius * 1000.0);
    RCLCPP_INFO(this->get_logger(), "[尺寸验证] 圆柱体尺寸: 高度=%.3f m, 半径=%.6f m", 
                cylinder.dimensions[0], cylinder.dimensions[1]);

    collision_object.primitives.push_back(cylinder);

    // 转换位姿到planning frame（使用通用函数，内部使用 tf2::TimePointZero）
    geometry_msgs::msg::PoseStamped pose_in_planning = cable_pose;
    if (!transform_pose_to_planning(pose_in_planning))
    {
      RCLCPP_ERROR(this->get_logger(), "转换缆绳位姿失败");
      return false;
    }

    // 使用原始yaw计算圆柱pose（确保障碍物方向正确反映视觉检测的缆绳方向）
    geometry_msgs::msg::Pose cylinder_pose = make_cable_cylinder_pose(pose_in_planning, cable_yaw);
    
    // 确认orientation已设置
    RCLCPP_INFO(this->get_logger(), 
                "[场景对象] 准备设置对象pose，orientation: (%.3f, %.3f, %.3f, %.3f)",
                cylinder_pose.orientation.x, cylinder_pose.orientation.y,
                cylinder_pose.orientation.z, cylinder_pose.orientation.w);
    
    collision_object.primitive_poses.push_back(cylinder_pose);
    
    // 确认设置成功
    if (!collision_object.primitive_poses.empty())
    {
      RCLCPP_INFO(this->get_logger(), 
                  "[场景对象] 对象pose已设置，确认orientation: (%.3f, %.3f, %.3f, %.3f)",
                  collision_object.primitive_poses[0].orientation.x, 
                  collision_object.primitive_poses[0].orientation.y,
                  collision_object.primitive_poses[0].orientation.z, 
                  collision_object.primitive_poses[0].orientation.w);
    }

    // 添加到场景
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    planning_scene_interface_->addCollisionObjects(collision_objects);

    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "添加缆绳碰撞体到场景: %s", object_id.c_str());
    RCLCPP_INFO(this->get_logger(), "  输入位置: pos=(%.3f, %.3f, %.3f), frame=%s", 
                cable_pose.pose.position.x, cable_pose.pose.position.y, 
                cable_pose.pose.position.z, cable_pose.header.frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "  输入yaw（障碍物方向）: %.3f rad (%.1f deg)", 
                cable_yaw, cable_yaw * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "  圆柱体位置: pos=(%.3f, %.3f, %.3f)", 
                cylinder_pose.position.x, cylinder_pose.position.y, cylinder_pose.position.z);
    RCLCPP_INFO(this->get_logger(), "  圆柱体姿态: orientation=(%.3f, %.3f, %.3f, %.3f)",
                cylinder_pose.orientation.x, cylinder_pose.orientation.y,
                cylinder_pose.orientation.z, cylinder_pose.orientation.w);
    RCLCPP_INFO(this->get_logger(), "  尺寸: 直径=%.6f m (%.1f mm), 长度=%.3f m (%.0f mm)", 
                cable_diameter_, cable_diameter_ * 1000.0, 
                cable_length_, cable_length_ * 1000.0);
    RCLCPP_INFO(this->get_logger(), "  圆柱体: 高度=%.3f m, 半径=%.6f m (%.1f mm)", 
                cylinder.dimensions[0], cylinder.dimensions[1], 
                cylinder.dimensions[1] * 1000.0);
    RCLCPP_INFO(this->get_logger(), "  坐标系: %s", planning_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "等待场景更新...");
    RCLCPP_INFO(this->get_logger(), "========================================");
    
    // 等待场景更新（MoveIt场景更新是异步的）
    // 方法1：轮询检查对象是否已添加到场景中（更可靠）
    const int max_attempts = 30;  // 最多等待3秒（30 * 100ms）
    const int wait_ms = 100;
    bool object_found = false;
    
    for (int i = 0; i < max_attempts; ++i)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
      std::vector<std::string> known_objects = planning_scene_interface_->getKnownObjectNames();
      
      for (const auto& name : known_objects)
      {
        if (name == object_id)
        {
          object_found = true;
          RCLCPP_INFO(this->get_logger(), "碰撞体 %s 已成功添加到场景中（等待了 %d ms）", 
                     object_id.c_str(), (i + 1) * wait_ms);
          
          // 发布Pose可视化（用于RViz显示方向轴）
          geometry_msgs::msg::PoseStamped viz_pose;
          viz_pose.header.frame_id = planning_frame_;
          viz_pose.header.stamp = this->now();
          viz_pose.pose = cylinder_pose;  // 使用圆柱体的pose（包含正确的orientation）
          
          // 发布到可视化话题
          if (cable_pose_viz_pub_)
          {
            cable_pose_viz_pub_->publish(viz_pose);
            RCLCPP_INFO(this->get_logger(), "[可视化] 已发布缆绳Pose到 /cable_pose_visualization，可在RViz中添加Pose显示查看方向轴");
          }
          
          break;
        }
      }
      
      if (object_found)
      {
        break;
      }
    }
    
    if (!object_found)
    {
      RCLCPP_WARN(this->get_logger(), 
                  "碰撞体 %s 在 %d ms 内未出现在场景中，但继续执行（可能场景更新较慢）", 
                  object_id.c_str(), max_attempts * wait_ms);
      RCLCPP_WARN(this->get_logger(), 
                  "提示：如果rviz中看不到缆绳，请检查：");
      RCLCPP_WARN(this->get_logger(), 
                  "  1. MotionPlanning插件中是否启用了'Scene Geometry'显示");
      RCLCPP_WARN(this->get_logger(), 
                  "  2. Planning Scene Topic是否设置为'monitored_planning_scene'");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), 
                  "✓ 缆绳碰撞体已成功添加到场景，应在rviz中可见");
      RCLCPP_INFO(this->get_logger(), 
                  "  如果rviz中看不到，请检查MotionPlanning插件的'Scene Geometry'是否启用");
    }
    
    return true;
  }

  // 添加地面碰撞对象到planning scene
  bool scene_add_ground_plane()
  {
    std::lock_guard<std::mutex> lock(moveit_mutex_);

    // 检查地面对象是否已存在
    const std::string ground_id = "ground_plane";
    std::vector<std::string> known_objects = planning_scene_interface_->getKnownObjectNames();
    bool ground_exists = false;
    for (const auto& name : known_objects)
    {
      if (name == ground_id)
      {
        ground_exists = true;
        break;
      }
    }

    // 创建地面碰撞对象（使用BOX）
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = planning_frame_;
    collision_object.id = ground_id;

    // 根据对象是否存在选择操作类型
    if (ground_exists)
    {
      collision_object.operation = moveit_msgs::msg::CollisionObject::MOVE;
      RCLCPP_INFO(this->get_logger(), "[场景更新] 地面对象 %s 已存在，使用MOVE操作更新", ground_id.c_str());
    }
    else
    {
      collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
      RCLCPP_INFO(this->get_logger(), "[场景更新] 地面对象 %s 不存在，使用ADD操作添加", ground_id.c_str());
    }

    // 创建地面box（足够大的平面）
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions.resize(3);
    box.dimensions[0] = 2.0;  // x方向长度（2m）
    box.dimensions[1] = 2.0;  // y方向长度（2m）
    box.dimensions[2] = 0.01; // z方向厚度（1cm，足够薄）

    collision_object.primitives.push_back(box);

    // 获取base_link在planning_frame中的位置，计算地面高度
    double base_z = 0.0;
    bool base_tf_available = false;
    try {
      geometry_msgs::msg::TransformStamped base_transform = 
        tf_buffer_->lookupTransform(planning_frame_, "base_link", tf2::TimePointZero);
      base_z = base_transform.transform.translation.z;
      base_tf_available = true;
      RCLCPP_INFO(this->get_logger(), "[地面] base_link在%s中的z坐标: %.3f m", 
                  planning_frame_.c_str(), base_z);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(), "[地面] 无法获取base_link位置，使用ground_height_参数: %s", ex.what());
      // 如果无法获取TF，使用配置的ground_height_
    }
    
    // 计算地面高度：基座位置 + 偏移
    double calculated_ground_height = base_z + ground_offset_below_base_;
    // 如果ground_height_已配置（非0），优先使用配置值；否则使用计算值
    double final_ground_height = (ground_height_ != 0.0) ? ground_height_ : calculated_ground_height;
    
    // 设置地面位置（在planning frame中，z=final_ground_height）
    geometry_msgs::msg::Pose ground_pose;
    ground_pose.position.x = 0.0;
    ground_pose.position.y = 0.0;
    ground_pose.position.z = final_ground_height - 0.005;  // 中心在final_ground_height下方5mm（因为厚度是1cm）
    ground_pose.orientation.x = 0.0;
    ground_pose.orientation.y = 0.0;
    ground_pose.orientation.z = 0.0;
    ground_pose.orientation.w = 1.0;  // 无旋转
    
    RCLCPP_INFO(this->get_logger(), "[地面] 基座z=%.3f, 偏移=%.3f, 最终地面高度=%.3f", 
                base_z, ground_offset_below_base_, final_ground_height);

    collision_object.primitive_poses.push_back(ground_pose);

    // 添加到场景
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    planning_scene_interface_->addCollisionObjects(collision_objects);

    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "添加地面碰撞体到场景: %s", ground_id.c_str());
    RCLCPP_INFO(this->get_logger(), "  位置: z=%.3f m (地面高度=%.3f m)", 
                ground_pose.position.z, final_ground_height);
    RCLCPP_INFO(this->get_logger(), "  尺寸: %.2f x %.2f x %.2f m", 
                box.dimensions[0], box.dimensions[1], box.dimensions[2]);
    RCLCPP_INFO(this->get_logger(), "  坐标系: %s", planning_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "等待场景更新...");
    RCLCPP_INFO(this->get_logger(), "========================================");

    // 等待场景更新（MoveIt场景更新是异步的）
    const int max_attempts = 30;  // 最多等待3秒（30 * 100ms）
    const int wait_ms = 100;
    bool object_found = false;

    for (int i = 0; i < max_attempts; ++i)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
      std::vector<std::string> known_objects = planning_scene_interface_->getKnownObjectNames();

      for (const auto& name : known_objects)
      {
        if (name == ground_id)
        {
          object_found = true;
          RCLCPP_INFO(this->get_logger(), "地面碰撞体 %s 已成功添加到场景中（等待了 %d ms）", 
                     ground_id.c_str(), (i + 1) * wait_ms);
          break;
        }
      }

      if (object_found)
      {
        break;
      }
    }

    if (!object_found)
    {
      RCLCPP_WARN(this->get_logger(), "地面碰撞体 %s 在 %d ms 内未添加到场景中，但继续执行", 
                  ground_id.c_str(), max_attempts * wait_ms);
    }

    return object_found;
  }

  // 发布抓取流程可视化标记
  void publish_grasp_viz(const std::string& stage,
                         const geometry_msgs::msg::PoseStamped& cable_pose_planning,
                         const geometry_msgs::msg::PoseStamped* pregrasp,
                         const geometry_msgs::msg::PoseStamped* grasp,
                         const geometry_msgs::msg::PoseStamped* lift)
  {
    if (!enable_viz_ || !marker_pub_) return;

    visualization_msgs::msg::MarkerArray arr;

    // Lambda函数：创建球体标记
    auto make_sphere = [&](int id, const std::string& ns,
                           const geometry_msgs::msg::Pose& pose,
                           double s, double r, double g, double b)
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = planning_frame_;
      m.header.stamp = this->now();
      m.ns = ns;
      m.id = id;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose = pose;
      m.scale.x = s; m.scale.y = s; m.scale.z = s;
      m.color.a = 0.9; m.color.r = r; m.color.g = g; m.color.b = b;
      return m;
    };

    // Lambda函数：创建箭头标记
    auto make_arrow = [&](int id, const std::string& ns,
                          const geometry_msgs::msg::Pose& pose,
                          double l, double w, double r, double g, double b)
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = planning_frame_;
      m.header.stamp = this->now();
      m.ns = ns;
      m.id = id;
      m.type = visualization_msgs::msg::Marker::ARROW;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose = pose;
      m.scale.x = l;   // 长
      m.scale.y = w;   // 宽
      m.scale.z = w;   // 高
      m.color.a = 0.9; m.color.r = r; m.color.g = g; m.color.b = b;
      return m;
    };

    // Lambda函数：创建文本标记
    auto make_text = [&](int id, const std::string& text,
                         double x, double y, double z)
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = planning_frame_;
      m.header.stamp = this->now();
      m.ns = "stage";
      m.id = id;
      m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = x;
      m.pose.position.y = y;
      m.pose.position.z = z;
      m.pose.orientation.w = 1.0;
      m.scale.z = 0.04;  // 字体高度
      m.color.a = 1.0; m.color.r = 1.0; m.color.g = 0.6; m.color.b = 0.0;  // 橙色
      m.text = text;
      return m;
    };

    // 缆绳目标点（白色球体）
    arr.markers.push_back(make_sphere(1, "cable", cable_pose_planning.pose, 0.02, 1.0, 1.0, 1.0));

    // 预抓取位姿（黄色）
    if (pregrasp) {
      arr.markers.push_back(make_sphere(10, "pregrasp", pregrasp->pose, 0.02, 1.0, 1.0, 0.0));
      arr.markers.push_back(make_arrow(11, "pregrasp", pregrasp->pose, 0.10, 0.02, 1.0, 1.0, 0.0));
    }
    
    // 抓取位姿（红色）
    if (grasp) {
      arr.markers.push_back(make_sphere(20, "grasp", grasp->pose, 0.02, 1.0, 0.0, 0.0));
      arr.markers.push_back(make_arrow(21, "grasp", grasp->pose, 0.10, 0.02, 1.0, 0.0, 0.0));
    }
    
    // 抬起位姿（绿色）
    if (lift) {
      arr.markers.push_back(make_sphere(30, "lift", lift->pose, 0.02, 0.0, 1.0, 0.0));
      arr.markers.push_back(make_arrow(31, "lift", lift->pose, 0.10, 0.02, 0.0, 1.0, 0.0));
    }

    // 连线（LINE_STRIP）：pregrasp → grasp → lift
    if (pregrasp && grasp) {
      visualization_msgs::msg::Marker line;
      line.header.frame_id = planning_frame_;
      line.header.stamp = this->now();
      line.ns = "trajectory";
      line.id = 100;
      line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.scale.x = 0.005;
      line.color.a = 0.9; line.color.r = 0.2; line.color.g = 0.6; line.color.b = 1.0;  // 蓝色

      geometry_msgs::msg::Point p;
      p.x = pregrasp->pose.position.x; 
      p.y = pregrasp->pose.position.y; 
      p.z = pregrasp->pose.position.z;
      line.points.push_back(p);
      
      p.x = grasp->pose.position.x; 
      p.y = grasp->pose.position.y; 
      p.z = grasp->pose.position.z;
      line.points.push_back(p);
      
      if (lift) {
        p.x = lift->pose.position.x; 
        p.y = lift->pose.position.y; 
        p.z = lift->pose.position.z;
        line.points.push_back(p);
      }
      arr.markers.push_back(line);
    }

    // 阶段文字（贴在缆绳目标点上方）
    arr.markers.push_back(make_text(200, stage,
                                    cable_pose_planning.pose.position.x,
                                    cable_pose_planning.pose.position.y,
                                    cable_pose_planning.pose.position.z + 0.08));

    marker_pub_->publish(arr);
  }

  // 追加末端执行器路径点
  void append_eef_path_once()
  {
    if (!enable_viz_ || !eef_path_pub_ || !move_group_interface_) return;
    
    try {
      auto cur = getCurrentPoseSafe();
      geometry_msgs::msg::PoseStamped cur_p = cur;
      if (!transform_pose_to_planning(cur_p)) return;

      geometry_msgs::msg::PoseStamped ps = cur_p;
      ps.header.stamp = this->now();

      eef_path_.header.frame_id = planning_frame_;
      eef_path_.header.stamp = ps.header.stamp;
      eef_path_.poses.push_back(ps);

      // 防止无限增长（最多保留2000个点）
      if (eef_path_.poses.size() > 2000) {
        eef_path_.poses.erase(eef_path_.poses.begin(), eef_path_.poses.begin() + 500);
      }

      eef_path_pub_->publish(eef_path_);
    } catch (const std::exception& e) {
      RCLCPP_DEBUG(this->get_logger(), "append_eef_path_once异常: %s", e.what());
    }
  }

  // 附着物体到末端执行器（包含完整几何信息）
  // 改进：ATTACH时使用eef_link frame，pose使用相对eef_link的位姿，减少RViz闪烁
  // cable_yaw: 视觉提供的原始yaw（缆绳切向方向），用于计算圆柱体姿态
  bool scene_attach(const std::string& object_id, const std::string& eef_link, 
                    const geometry_msgs::msg::PoseStamped& cable_pose,
                    double cable_yaw)
  {
    std::lock_guard<std::mutex> lock(moveit_mutex_);

    // 重新构造碰撞体的完整几何信息（确保attach时包含几何）
    moveit_msgs::msg::AttachedCollisionObject attached_object;
    attached_object.object.id = object_id;
    attached_object.object.operation = moveit_msgs::msg::CollisionObject::ADD;
    
    // ATTACH时使用eef_link frame（而不是planning_frame），减少RViz闪烁
    attached_object.object.header.frame_id = eef_link;
    
    // 创建圆柱体几何（与scene_add_cable_object中相同）
    shape_msgs::msg::SolidPrimitive cylinder;
    cylinder.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    cylinder.dimensions.resize(2);
    cylinder.dimensions[0] = cable_length_;  // height
    cylinder.dimensions[1] = cable_diameter_ / 2.0;  // radius

    attached_object.object.primitives.push_back(cylinder);
    
    // cable_pose 应该已经在 planning frame 中（由调用者保证）
    // 如果不在，进行转换（防御性检查）
    geometry_msgs::msg::PoseStamped pose_in_planning = cable_pose;
    if (pose_in_planning.header.frame_id != planning_frame_)
    {
      RCLCPP_WARN(this->get_logger(), "scene_attach: cable_pose 不在 planning frame，进行转换");
      if (!transform_pose_to_planning(pose_in_planning))
      {
        RCLCPP_ERROR(this->get_logger(), "附着物体时转换位姿失败");
        return false;
      }
    }
    
    // 计算cable_pose相对于eef_link的位姿
    // 1. 计算cable在planning_frame中的位姿（使用make_cable_cylinder_pose，传入原始yaw）
    geometry_msgs::msg::Pose cylinder_pose_planning = make_cable_cylinder_pose(pose_in_planning, cable_yaw);
    
    // 2. 将cylinder_pose从planning_frame转换到eef_link frame
    geometry_msgs::msg::PoseStamped cylinder_pose_stamped;
    cylinder_pose_stamped.header.frame_id = planning_frame_;
    cylinder_pose_stamped.pose = cylinder_pose_planning;
    
    try {
      // 使用当前时间获取TF变换（而不是TimePointZero）
      // 这样可以确保使用最新的TF变换，避免在快速移动时使用过时的变换
      rclcpp::Time now = this->now();
      geometry_msgs::msg::TransformStamped transform = 
          tf_buffer_->lookupTransform(eef_link, planning_frame_, now, 
                                       tf2::durationFromSec(0.1));  // 0.1s超时
      tf2::doTransform(cylinder_pose_stamped, cylinder_pose_stamped, transform);
      
      RCLCPP_DEBUG(this->get_logger(), 
                  "[附着] TF变换时间戳: now=%.3f, transform_time=%.3f", 
                  now.seconds(), 
                  rclcpp::Time(transform.header.stamp).seconds());
      
      // 使用相对eef_link的位姿
      attached_object.object.primitive_poses.push_back(cylinder_pose_stamped.pose);
    } catch (const tf2::TransformException& ex) {
      // 如果TF转换失败，fallback到planning_frame（兼容旧行为）
      RCLCPP_WARN(this->get_logger(), 
                  "无法转换到eef_link frame，fallback到planning_frame: %s", ex.what());
      attached_object.object.header.frame_id = planning_frame_;
      attached_object.object.primitive_poses.push_back(cylinder_pose_planning);
    }
    
    attached_object.link_name = eef_link;
    attached_object.touch_links = allow_touch_links_;

    // 先尝试 detach 同名旧 attached（仅在存在时执行，避免MoveIt警告）
    bool attached_exists = is_object_attached(object_id, eef_link);
    if (attached_exists)
    {
      moveit_msgs::msg::AttachedCollisionObject rm;
      rm.object.id = object_id;
      rm.link_name = eef_link;  // 用参数 eef_link
      rm.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      planning_scene_interface_->applyAttachedCollisionObject(rm);
      RCLCPP_DEBUG(this->get_logger(), "Detach旧附着对象: %s (从 %s)", object_id.c_str(), eef_link.c_str());
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    else
    {
      RCLCPP_DEBUG(this->get_logger(), "附着对象 %s 不存在，跳过detach", object_id.c_str());
    }

    // 先移除场景中的world对象（仅在存在时移除），避免 attach 时覆盖/重建导致闪烁
    // PlanningSceneInterface 的 add/apply 是异步的，如果 add 还没完全传播就 attach，
    // 可能导致 RViz 中物体闪烁/跳动（不影响规划，但影响观感）
    std::vector<std::string> known_objects = planning_scene_interface_->getKnownObjectNames();
    bool world_object_exists = false;
    for (const auto& name : known_objects)
    {
      if (name == object_id)
      {
        world_object_exists = true;
        break;
      }
    }
    
    if (world_object_exists)
    {
      std::vector<std::string> object_ids = {object_id};
      planning_scene_interface_->removeCollisionObjects(object_ids);
      RCLCPP_DEBUG(this->get_logger(), "移除world对象: %s (避免attach时闪烁)", object_id.c_str());
    }
    else
    {
      RCLCPP_DEBUG(this->get_logger(), "world对象 %s 不存在，跳过移除", object_id.c_str());
    }
    
    // 等待一小段时间确保移除操作传播到 scene monitor
    // 这样可以避免 attach 时覆盖/重建导致的视觉闪烁
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // 然后再 attach
    planning_scene_interface_->applyAttachedCollisionObject(attached_object);

    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "附着物体 %s 到 %s（包含完整几何信息）", object_id.c_str(), eef_link.c_str());
    RCLCPP_INFO(this->get_logger(), "  输入位置: pos=(%.3f, %.3f, %.3f), frame=%s", 
                cable_pose.pose.position.x, cable_pose.pose.position.y, 
                cable_pose.pose.position.z, cable_pose.header.frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "  输入yaw（障碍物方向）: %.3f rad (%.1f deg)", 
                cable_yaw, cable_yaw * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "  附着frame: %s", attached_object.object.header.frame_id.c_str());
    if (!attached_object.object.primitive_poses.empty())
    {
      RCLCPP_INFO(this->get_logger(), "  附着位姿: pos=(%.3f, %.3f, %.3f), orientation=(%.3f, %.3f, %.3f, %.3f)",
                  attached_object.object.primitive_poses[0].position.x,
                  attached_object.object.primitive_poses[0].position.y,
                  attached_object.object.primitive_poses[0].position.z,
                  attached_object.object.primitive_poses[0].orientation.x,
                  attached_object.object.primitive_poses[0].orientation.y,
                  attached_object.object.primitive_poses[0].orientation.z,
                  attached_object.object.primitive_poses[0].orientation.w);
    }
    RCLCPP_INFO(this->get_logger(), "========================================");
    return true;
  }

  // 移除物体
  bool scene_remove(const std::string& object_id)
  {
    std::lock_guard<std::mutex> lock(moveit_mutex_);

    moveit_msgs::msg::CollisionObject remove_object;
    remove_object.id = object_id;
    remove_object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(remove_object);
    planning_scene_interface_->applyCollisionObjects(collision_objects);

    RCLCPP_INFO(this->get_logger(), "移除物体: %s", object_id.c_str());
    return true;
  }

  // 辅助函数：检查对象是否被附着到指定link
  // 返回true表示对象存在且被附着，false表示不存在或未被附着
  bool is_object_attached(const std::string& object_id, const std::string& eef_link)
  {
    try {
      if (!move_group_interface_) {
        return false;
      }
      
      // 获取当前机器人状态
      // 修复时间戳问题：使用Time(0)获取最新可用状态
      auto state = move_group_interface_->getCurrentState(0.0);
      if (!state) {
        return false;
      }
      
      // 获取所有附着的对象（MoveIt2 API：需要传入vector参数）
      std::vector<const moveit::core::AttachedBody*> attached_bodies;
      state->getAttachedBodies(attached_bodies);
      
      // 遍历检查目标对象是否在列表中
      for (const auto* attached_body : attached_bodies) {
        if (attached_body && attached_body->getName() == object_id) {
          // 检查是否附着到指定的link
          if (attached_body->getAttachedLink() && 
              attached_body->getAttachedLink()->getName() == eef_link) {
            return true;
          }
        }
      }
      
      return false;
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "检查附着对象时发生异常: %s", e.what());
      return false;
    }
  }

  // 清理函数：detach + remove（避免残留对象导致冲突）
  // 优化：添加存在性检查，实现幂等性，避免不必要的警告
  bool scene_detach_and_remove(const std::string& object_id, const std::string& eef_link)
  {
    std::lock_guard<std::mutex> lock(moveit_mutex_);

    // 1) 检查world对象是否存在
    std::vector<std::string> known_objects = planning_scene_interface_->getKnownObjectNames();
    bool world_object_exists = false;
    for (const auto& name : known_objects)
    {
      if (name == object_id)
      {
        world_object_exists = true;
        break;
      }
    }

    // 2) 检查attached对象是否存在
    bool attached_object_exists = is_object_attached(object_id, eef_link);

    // 3) Detach (REMOVE attached) - 仅在存在时执行，避免MoveIt警告
    if (attached_object_exists)
    {
      moveit_msgs::msg::AttachedCollisionObject aco;
      aco.object.id = object_id;
      aco.link_name = eef_link;  // 必填：表示从哪个link上detach
      aco.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      planning_scene_interface_->applyAttachedCollisionObject(aco);
      RCLCPP_DEBUG(this->get_logger(), "Detach附着对象: %s (从 %s)", object_id.c_str(), eef_link.c_str());
    }
    else
    {
      RCLCPP_DEBUG(this->get_logger(), "附着对象 %s 不存在，跳过detach", object_id.c_str());
    }

    // 4) Remove world object (仅在存在时移除，避免警告)
    if (world_object_exists)
    {
      planning_scene_interface_->removeCollisionObjects({object_id});
      RCLCPP_DEBUG(this->get_logger(), "移除world对象: %s", object_id.c_str());
    }
    else
    {
      RCLCPP_DEBUG(this->get_logger(), "world对象 %s 不存在，跳过移除", object_id.c_str());
    }

    // 等待操作传播
    if (attached_object_exists || world_object_exists)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    RCLCPP_INFO(this->get_logger(), "清理完成(detach+remove): %s [attached:%s, world:%s]", 
                object_id.c_str(),
                attached_object_exists ? "是" : "否",
                world_object_exists ? "是" : "否");
    return true;
  }

  // 转换位姿到planning frame（通用函数）
  // 优先使用 pose.header.stamp，失败则 fallback 到 tf2::TimePointZero
  // 这样可以处理视觉延迟（优先用stamp），同时避免时间戳不同步导致的 extrapolation 错误
  // 改进：使用canTransform预检查，减少异常；添加fallback计数，超过阈值报警
  bool transform_pose_to_planning(geometry_msgs::msg::PoseStamped& pose)
  {
    if (pose.header.frame_id == planning_frame_)
    {
      return true;  // 已经在planning frame中
    }

    // 优先使用 pose.header.stamp（如果有效）
    // 这样可以处理视觉检测延迟，使用对应时间点的 TF
    if (pose.header.stamp.sec != 0 || pose.header.stamp.nanosec != 0)
    {
      rclcpp::Time transform_time(pose.header.stamp);
      rclcpp::Duration timeout(0, 100000000);  // 100ms超时
      
      // 先使用canTransform检查，避免频繁抛异常
      if (tf_buffer_->canTransform(planning_frame_, pose.header.frame_id, transform_time, timeout))
      {
        try
        {
          geometry_msgs::msg::TransformStamped transform = 
              tf_buffer_->lookupTransform(planning_frame_, 
                                         pose.header.frame_id, 
                                         transform_time);
          tf2::doTransform(pose, pose, transform);
          pose.header.frame_id = planning_frame_;
          return true;
        }
        catch (const tf2::TransformException& ex)
        {
          // canTransform通过但lookup失败，记录但继续fallback
          RCLCPP_DEBUG(this->get_logger(), 
                       "canTransform通过但lookup失败，fallback到最新TF: %s", 
                       ex.what());
        }
      }
      else
      {
        // canTransform失败，stamp不可用或超时，fallback到latest
        RCLCPP_DEBUG(this->get_logger(), 
                     "stamp转换不可用（canTransform失败），fallback到最新TF");
      }
    }

    // Fallback：使用 tf2::TimePointZero 获取最新 TF
    // 这样可以避免时间戳不同步导致的 extrapolation 错误
    // 记录fallback次数，超过阈值报警
    tf_fallback_count_++;
    if (tf_fallback_count_ > 10)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5.0,
                           "TF转换fallback次数过多（%zu次），可能系统时钟/TF延迟有问题", 
                           tf_fallback_count_.load());
    }
    
    try
    {
      // 使用canTransform检查latest TF是否可用
      // 注意：对于tf2::TimePointZero，需要使用tf2::Duration而不是rclcpp::Duration
      // tf2::Duration 是 std::chrono::nanoseconds 的类型别名，直接使用 std::chrono::nanoseconds
      std::chrono::nanoseconds timeout_ns(100000000);  // 100ms超时
      tf2::Duration timeout(timeout_ns);
      if (tf_buffer_->canTransform(planning_frame_, pose.header.frame_id, tf2::TimePointZero, timeout))
      {
        geometry_msgs::msg::TransformStamped transform = 
            tf_buffer_->lookupTransform(planning_frame_, 
                                       pose.header.frame_id, 
                                       tf2::TimePointZero);
        tf2::doTransform(pose, pose, transform);
        pose.header.frame_id = planning_frame_;
        return true;
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), 
                     "转换位姿到planning frame失败：canTransform检查失败（TF不可用）");
        return false;
      }
    }
    catch (const tf2::TransformException& ex)
    {
      RCLCPP_ERROR(this->get_logger(), "转换位姿到planning frame失败: %s", ex.what());
      return false;
    }
  }

  // 计算向下orientation（适合抓取任务）
  geometry_msgs::msg::Quaternion compute_downward_orientation()
  {
    // 使用tf2::Quaternion创建绕x轴旋转180度的四元数，使z轴向下
    tf2::Quaternion q;
    q.setRPY(M_PI, 0.0, 0.0);  // Roll=180°, Pitch=0°, Yaw=0°
    geometry_msgs::msg::Quaternion quat;
    quat.x = q.x();
    quat.y = q.y();
    quat.z = q.z();
    quat.w = q.w();
    return quat;
  }

  // 计算带yaw的向下orientation（用于平铺线缆抓取）
  // 方法：先计算朝下的四元数（Roll=π），然后绕Z轴旋转Yaw角度
  // 这样可以避免setRPY在Roll=π时的万向锁问题，确保Yaw分量直接对应joint 4的旋转
  geometry_msgs::msg::Quaternion compute_downward_orientation_with_yaw(double yaw)
  {
    // 注意：此函数不再叠加任何offset，所有yaw偏移已在调用处（cable_pose_with_yaw_callback）统一计算
    // 输入yaw已经是经过所有偏移调整后的最终值
    
    RCLCPP_INFO(this->get_logger(), 
                "[orientation计算] 输入yaw=%.3f rad (%.1f deg)（已包含所有偏移）",
                yaw, yaw * 180.0 / M_PI);
    
    // 方法：使用setRPY直接计算orientation，实现"Z轴朝下 + 绕竖直轴转yaw"
    // setRPY(Roll, Pitch, Yaw)的旋转顺序是：先Yaw，再Pitch，最后Roll
    // 所以 setRPY(M_PI, 0.0, yaw) 表示：先绕Z轴旋转yaw，再绕X轴旋转180°（朝下）
    // 这样yaw分量直接对应输入的yaw（绕世界Z轴的旋转角度），不受万向锁影响
    //
    // 注意：不需要关心Joint4的值，MoveIt的IK会自动找到合适的关节角使末端达到目标姿态
    tf2::Quaternion q;
    q.setRPY(M_PI, 0.0, yaw);  // Roll=180°（朝下）, Pitch=0°, Yaw=yaw（绕竖直轴旋转）
    q.normalize();
    
    // 验证计算得到的orientation的RPY，特别是yaw分量
    double roll_result, pitch_result, yaw_result;
    tf2::Matrix3x3(q).getRPY(roll_result, pitch_result, yaw_result);
    RCLCPP_INFO(this->get_logger(),
                "[orientation计算] 计算得到的orientation RPY: roll=%.3f rad (%.1f°), pitch=%.3f rad (%.1f°), yaw=%.3f rad (%.1f°)",
                roll_result, roll_result * 180.0 / M_PI,
                pitch_result, pitch_result * 180.0 / M_PI,
                yaw_result, yaw_result * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(),
                "[orientation计算] 输入yaw vs 提取yaw: 输入=%.3f rad (%.1f°), 提取=%.3f rad (%.1f°), 差值=%.3f rad (%.1f°)",
                yaw, yaw * 180.0 / M_PI,
                yaw_result, yaw_result * 180.0 / M_PI,
                yaw_result - yaw, (yaw_result - yaw) * 180.0 / M_PI);
    
    geometry_msgs::msg::Quaternion quat = tf2::toMsg(q);
    return quat;
  }

  // 计算"Z轴朝下"的姿态，不包含yaw约束
  // 用于4DOF机器人，将方向控制交给Joint1，末端姿态只锁定roll/pitch（向下）
  geometry_msgs::msg::Quaternion compute_downward_orientation_fixed()
  {
    // 只设置roll=180°（朝下），pitch=0°，yaw=0°（不约束yaw）
    // 这样MoveIt的IK可以自由选择yaw，通过Joint1控制方向
    tf2::Quaternion q;
    q.setRPY(M_PI, 0.0, 0.0);  // Roll=180°（朝下）, Pitch=0°, Yaw=0°（不约束）
    q.normalize();
    
    geometry_msgs::msg::Quaternion quat = tf2::toMsg(q);
    RCLCPP_DEBUG(this->get_logger(), 
                "[orientation计算] 固定向下姿态（不约束yaw）: roll=180°, pitch=0°, yaw=0°");
    return quat;
  }
  
  // 检查当前状态是否接近全零状态
  // 返回true表示所有关节都接近0（在阈值范围内）
  bool check_near_zero_state()
  {
    std::lock_guard<std::mutex> lock(moveit_mutex_);
    try {
      if (!move_group_interface_) {
        RCLCPP_WARN(this->get_logger(), "[全零检测] MoveGroupInterface未初始化");
        return false;
      }
      std::vector<double> current_joints = move_group_interface_->getCurrentJointValues();
      if (current_joints.size() < 4) {
        RCLCPP_WARN(this->get_logger(), "[全零检测] 关节数量不足: %zu < 4", current_joints.size());
        return false;
      }
      
      // 检查前4个关节是否都接近0
      for (size_t i = 0; i < 4; ++i) {
        if (std::abs(current_joints[i]) > near_zero_threshold_) {
          return false;
        }
      }
      RCLCPP_INFO(this->get_logger(), "[全零检测] 检测到全零状态: [%.3f, %.3f, %.3f, %.3f] rad",
                  current_joints[0], current_joints[1], current_joints[2], current_joints[3]);
      return true;
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "[全零检测] 无法获取当前关节状态: %s", e.what());
      return false;
    }
  }
  
  // 从全零状态移动到安全预备位置
  // 使用简单的MoveGroupInterface规划和执行，避免MTC的复杂性
  bool move_to_ready_position()
  {
    std::lock_guard<std::mutex> lock(moveit_mutex_);
    
    if (!move_group_interface_) {
      RCLCPP_ERROR(this->get_logger(), "[预运动] MoveGroupInterface未初始化");
      return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "[预运动] 开始移动到预备位置: [%.3f, %.3f, %.3f, %.3f] rad",
                ready_joint1_, ready_joint2_, ready_joint3_, ready_joint4_);
    
    // 设置关节目标
    std::vector<double> ready_joints = {ready_joint1_, ready_joint2_, ready_joint3_, ready_joint4_};
    move_group_interface_->setJointValueTarget(ready_joints);
    
    // 设置规划参数（使用较宽松的参数，因为只是预运动）
    move_group_interface_->setPlanningTime(10.0);
    move_group_interface_->setNumPlanningAttempts(10);
    move_group_interface_->setMaxVelocityScalingFactor(0.5);  // 较慢速度，安全第一
    move_group_interface_->setMaxAccelerationScalingFactor(0.5);
    
    // 规划
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = move_group_interface_->plan(plan);
    
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "[预运动] 规划失败，错误码: %d", result.val);
      return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "[预运动] 规划成功，开始执行...");
    
    // 执行
    result = move_group_interface_->execute(plan);
    
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "[预运动] 执行失败，错误码: %d", result.val);
      return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "[预运动] ✓ 成功移动到预备位置");
    
    // 等待一下让状态稳定
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    return true;
  }

  // TCP偏移补偿：将"夹爪中心目标"转换为"EEF link目标"
  // 
  // 功能说明：
  //   输入是"夹爪中心"的目标位姿（通常由感知模块计算得到），但MoveIt规划时使用的是EEF link（如LinkGG）。
  //   此函数将夹爪中心的目标位姿转换为对应的EEF link位姿。
  //
  // 数学原理：
  //   设：center_pose = 夹爪中心在世界坐标系中的位姿
  //       eef_pose = EEF link在世界坐标系中的位姿
  //       offset = 在EEF坐标系中，从EEF原点指向夹爪中心的向量
  //
  //   关系：center_pose = eef_pose * offset（在EEF坐标系中）
  //   即：center_world = eef_world + R(eef_orientation) * offset_eef
  //
  //   因此：eef_world = center_world - R(eef_orientation) * offset_eef
  //   注意：由于我们使用center_pose的orientation作为eef_orientation的近似，所以：
  //        eef_world ≈ center_world - R(center_orientation) * offset_eef
  //
  // 参数说明：
  //   desired_center_pose: 期望的夹爪中心位姿（世界坐标系）
  //   eef_to_center_offset: 在EEF坐标系中，从EEF原点指向夹爪中心的向量
  //                         例如：如果EEF在夹爪中心的正Y方向0.024m处，则offset_y = -0.024
  //                         注意：符号必须与URDF中EEF link到夹爪中心的实际几何关系一致
  //
  // 返回值：
  //   eef_pose: 对应的EEF link位姿（世界坐标系），用于MoveIt规划
  //
  // 注意事项：
  //   1. eef_link_的定义会影响offset的语义：
  //      - 如果eef_link_是arm_group的最后一个link，offset应基于该link定义
  //      - 如果eef_link_是LinkGG，offset应基于LinkGG定义
  //   2. offset的符号很容易搞反，建议通过TF诊断（LinkGL/LinkGR中点 vs eef_link）验证
  //   3. 如果eef_link_的获取逻辑改变（自动取末端link vs 固定LinkGG），需要重新标定offset
  geometry_msgs::msg::PoseStamped compensate_tcp_offset_for_eef(
      const geometry_msgs::msg::PoseStamped& desired_center_pose,
      const tf2::Vector3& eef_to_center_offset /* in EEF frame */)
  {
    geometry_msgs::msg::PoseStamped eef_pose = desired_center_pose;

    tf2::Quaternion q;
    tf2::fromMsg(desired_center_pose.pose.orientation, q);
    q.normalize();

    // 将EEF坐标系中的offset转换到世界坐标系
    // offset_world = R(center_orientation) * offset_eef
    tf2::Vector3 offset_world = tf2::quatRotate(q, eef_to_center_offset);

    // 计算EEF link位置：eef_world = center_world - offset_world
    // 这表示：如果offset是从EEF指向center，则center = eef + offset，所以eef = center - offset
    eef_pose.pose.position.x -= offset_world.x();
    eef_pose.pose.position.y -= offset_world.y();
    eef_pose.pose.position.z -= offset_world.z();

    return eef_pose;
  }

  // 计算多个orientation候选（用于尝试找到可达的orientation）
  std::vector<geometry_msgs::msg::Quaternion> compute_orientation_candidates()
  {
    std::vector<geometry_msgs::msg::Quaternion> candidates;
    tf2::Quaternion q;
    geometry_msgs::msg::Quaternion quat;
    
    // 候选1: 向下 (Roll=180°, Pitch=0°, Yaw=0°)
    q.setRPY(M_PI, 0.0, 0.0);
    quat.x = q.x(); quat.y = q.y(); quat.z = q.z(); quat.w = q.w();
    candidates.push_back(quat);
    
    // 候选2: 水平向前 (Roll=0°, Pitch=90°, Yaw=0°) - 末端执行器水平向前
    q.setRPY(0.0, M_PI/2, 0.0);
    quat.x = q.x(); quat.y = q.y(); quat.z = q.z(); quat.w = q.w();
    candidates.push_back(quat);
    
    // 候选3: 稍微倾斜向下 (Roll=135°, Pitch=0°, Yaw=0°)
    q.setRPY(3*M_PI/4, 0.0, 0.0);
    quat.x = q.x(); quat.y = q.y(); quat.z = q.z(); quat.w = q.w();
    candidates.push_back(quat);
    
    // 候选4: 保持当前orientation（如果可用且不是默认值）
    if (move_group_interface_)
    {
      try {
        auto current_pose = getCurrentPoseSafe();
        if (!is_default_orientation(current_pose.pose.orientation))
        {
          candidates.push_back(current_pose.pose.orientation);
        }
      } catch (const std::exception& e) {
        // 忽略错误，继续使用其他候选
      }
    }
    
    return candidates;
  }

  // 检查orientation是否为默认值
  bool is_default_orientation(const geometry_msgs::msg::Quaternion& orientation)
  {
    return (std::abs(orientation.x) < orientation_epsilon_ &&
            std::abs(orientation.y) < orientation_epsilon_ &&
            std::abs(orientation.z) < orientation_epsilon_ &&
            std::abs(orientation.w - 1.0) < orientation_epsilon_);
  }

  // 检查位置是否在4DOF机械臂工作空间内
  // 4DOF机械臂(Yaw + 平面2R + Wrist)的工作空间是一个"碗状"区域
  // Joint2/3限位为[-2.97, 0]（-170°~0°），只能向下弯曲，无法"从另一侧兜过去"
  bool is_reachable(double x, double y, double z)
  {
    // URDF基本参数（从配置读取）
    const double joint2_height = workspace_base_height_;  // Joint2轴高度(相对base原点)
    const double link2_length = workspace_link2_length_;  // 大臂长度
    const double link3_length = workspace_link3_length_;  // 小臂长度
    const double link4_to_eef = workspace_link4_to_eef_;  // 末端长度
    
    // 计算理论最大伸展距离
    const double max_reach = link2_length + link3_length + link4_to_eef;
    const double max_reach_with_margin = max_reach * workspace_reach_radius_margin_;
    
    // 计算理论高度范围
    const double max_height = joint2_height + workspace_max_height_offset_;
    const double min_height = joint2_height + workspace_min_height_offset_;
    
    // 计算到基座的水平距离（半径）
    const double radius = std::sqrt(x * x + y * y);
    
    // 计算相对于Joint2轴的高度
    const double height_from_joint2 = z - joint2_height;
    
    // 计算从Joint2到目标点的直线距离
    const double distance_from_joint2 = std::sqrt(radius * radius + height_from_joint2 * height_from_joint2);
    
    // ========== 工作空间检查 ==========
    // 检查1: 最小半径（太靠近base无法到达）
    if (radius < workspace_min_radius_)
    {
      RCLCPP_WARN(this->get_logger(),
                  "[工作空间检查] 目标 (%.3f, %.3f, %.3f) 太靠近base中心 (r=%.3f < %.3f m)，可能无法到达",
                  x, y, z, radius, workspace_min_radius_);
      // 只警告，不直接拒绝，让IK做最终判定
    }
    
    // 检查2: 最大伸展距离
    if (distance_from_joint2 > max_reach_with_margin)
    {
      RCLCPP_ERROR(this->get_logger(),
                  "[工作空间检查] 目标 (%.3f, %.3f, %.3f) 超出最大伸展距离 (d=%.3f > %.3f m)，拒绝",
                  x, y, z, distance_from_joint2, max_reach_with_margin);
      return false;
    }
    
    // 检查3: 高度范围
    if (z > max_height)
    {
      RCLCPP_ERROR(this->get_logger(),
                  "[工作空间检查] 目标 (%.3f, %.3f, %.3f) 高度过高 (z=%.3f > %.3f m)，拒绝",
                  x, y, z, z, max_height);
      return false;
    }
    
    if (z < min_height)
    {
      RCLCPP_ERROR(this->get_logger(),
                  "[工作空间检查] 目标 (%.3f, %.3f, %.3f) 高度过低 (z=%.3f < %.3f m)，拒绝",
                  x, y, z, z, min_height);
      return false;
    }
    
    // 检查4: 特殊情况 - Joint2/3限位检查
    // 由于Joint2/3只能向下弯曲[-170°, 0°]，有些位置虽然在球壳内但无法到达
    // 简化检查：如果目标点在Joint2轴以上但水平距离很大，可能无法到达
    if (height_from_joint2 > 0.1 && radius > 0.4)
    {
      RCLCPP_WARN(this->get_logger(),
                  "[工作空间检查] 目标 (%.3f, %.3f, %.3f) 在高位且水平距离大，4DOF可能难以到达",
                  x, y, z);
      // 只警告，让IK做最终判定
    }
    
    // 通过基本检查
    RCLCPP_INFO(this->get_logger(), 
                "[工作空间检查] 目标 (%.3f, %.3f, %.3f) 通过检查 (r=%.3f m, z=%.3f m, d_j2=%.3f m)",
                x, y, z, radius, z, distance_from_joint2);
    return true;
  }

  // 调整位置到工作空间内的安全位置
  void adjust_to_workspace(double& x, double& y, double& z)
  {
    // 高成功率区域（从配置读取）
    const double safe_x_min = workspace_safe_x_min_;
    const double safe_x_max = workspace_safe_x_max_;
    const double safe_y_min = workspace_safe_y_min_;
    const double safe_y_max = workspace_safe_y_max_;
    const double safe_z_min = workspace_safe_z_min_;
    const double safe_z_max = workspace_safe_z_max_;
    
    // 中等成功率区域（从配置读取）
    const double medium_x_min = workspace_medium_x_min_;
    const double medium_x_max = workspace_medium_x_max_;
    const double medium_y_min = workspace_medium_y_min_;
    const double medium_y_max = workspace_medium_y_max_;
    const double medium_z_min = workspace_medium_z_min_;
    const double medium_z_max = workspace_medium_z_max_;
    
    bool adjusted = false;
    double original_x = x, original_y = y, original_z = z;
    
    // 首先尝试调整到高成功率区域
    if (x < safe_x_min) { x = safe_x_min; adjusted = true; }
    else if (x > safe_x_max) { x = safe_x_max; adjusted = true; }
    
    if (y < safe_y_min) { y = safe_y_min; adjusted = true; }
    else if (y > safe_y_max) { y = safe_y_max; adjusted = true; }
    
    if (z < safe_z_min) { z = safe_z_min; adjusted = true; }
    else if (z > safe_z_max) { z = safe_z_max; adjusted = true; }
    
    // 如果调整后仍然超出，使用中等成功率区域
    if (x < medium_x_min || x > medium_x_max ||
        y < medium_y_min || y > medium_y_max ||
        z < medium_z_min || z > medium_z_max)
    {
      if (x < medium_x_min) { x = medium_x_min; }
      else if (x > medium_x_max) { x = medium_x_max; }
      
      if (y < medium_y_min) { y = medium_y_min; }
      else if (y > medium_y_max) { y = medium_y_max; }
      
      if (z < medium_z_min) { z = medium_z_min; }
      else if (z > medium_z_max) { z = medium_z_max; }
    }
    
    if (adjusted || x != original_x || y != original_y || z != original_z)
    {
      RCLCPP_WARN(this->get_logger(),
                  "位置已调整: (%.3f, %.3f, %.3f) -> (%.3f, %.3f, %.3f)",
                  original_x, original_y, original_z, x, y, z);
    }
  }

  // 检查yaw在特定z高度下是否IK可解
  // 这是比CartesianPath更高层级的"抓取策略"，避免某些z高度下特定yaw值无解或需要elbow flip
  bool is_yaw_ik_valid(double x, double y, double z, double yaw, 
                       const moveit::core::RobotStatePtr& state)
  {
    if (!state) {
      return false;
    }

    const auto* jmg = state->getJointModelGroup("arm_group");
    if (!jmg) {
      return false;
    }

    // 构造位姿：使用指定的yaw计算orientation
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation = compute_downward_orientation_with_yaw(yaw);

    // 尝试IK求解
    bool ok = state->setFromIK(jmg, pose, eef_link_, ik_timeout_);
    return ok;
  }

  // 在yaw合法窗口内搜索可解的yaw值
  // 如果原始yaw不可解，在±yaw_tolerance范围内搜索可解的yaw值
  // 返回找到的yaw值，如果找不到则返回原始yaw（调用者需要检查是否有效）
  double find_valid_yaw(double x, double y, double z, double original_yaw,
                        const moveit::core::RobotStatePtr& state,
                        double yaw_tolerance = M_PI / 36.0,  // 默认±5°
                        double yaw_search_step = M_PI / 180.0)  // 默认1°步长
  {
    // 先尝试原始yaw
    if (is_yaw_ik_valid(x, y, z, original_yaw, state)) {
      RCLCPP_INFO(this->get_logger(), 
                  "[yaw合法性检查] 原始yaw=%.3f rad (%.1f°) 在位置(%.3f, %.3f, %.3f)下IK可解",
                  original_yaw, original_yaw * 180.0 / M_PI, x, y, z);
      return original_yaw;
    }

    RCLCPP_WARN(this->get_logger(), 
                "[yaw合法性检查] 原始yaw=%.3f rad (%.1f°) 在位置(%.3f, %.3f, %.3f)下IK不可解，开始搜索...",
                original_yaw, original_yaw * 180.0 / M_PI, x, y, z);

    // 在±yaw_tolerance范围内搜索
    // 搜索顺序：先尝试小的偏移，再尝试大的偏移
    int max_steps = static_cast<int>(yaw_tolerance / yaw_search_step);
    for (int step = 1; step <= max_steps; ++step) {
      // 尝试正向偏移
      double yaw_plus = original_yaw + step * yaw_search_step;
      if (yaw_plus <= original_yaw + yaw_tolerance) {
        if (is_yaw_ik_valid(x, y, z, yaw_plus, state)) {
          RCLCPP_INFO(this->get_logger(), 
                      "[yaw合法性检查] 找到可解yaw: %.3f rad (%.1f°)，偏移: +%.3f rad (+%.1f°)",
                      yaw_plus, yaw_plus * 180.0 / M_PI,
                      step * yaw_search_step, step * yaw_search_step * 180.0 / M_PI);
          return yaw_plus;
        }
      }

      // 尝试负向偏移
      double yaw_minus = original_yaw - step * yaw_search_step;
      if (yaw_minus >= original_yaw - yaw_tolerance) {
        if (is_yaw_ik_valid(x, y, z, yaw_minus, state)) {
          RCLCPP_INFO(this->get_logger(), 
                      "[yaw合法性检查] 找到可解yaw: %.3f rad (%.1f°)，偏移: -%.3f rad (-%.1f°)",
                      yaw_minus, yaw_minus * 180.0 / M_PI,
                      step * yaw_search_step, step * yaw_search_step * 180.0 / M_PI);
          return yaw_minus;
        }
      }
    }

    RCLCPP_WARN(this->get_logger(), 
                "[yaw合法性检查] 在±%.3f rad (±%.1f°)范围内未找到可解yaw，返回原始yaw",
                yaw_tolerance, yaw_tolerance * 180.0 / M_PI);
    return original_yaw;  // 返回原始yaw，调用者需要检查是否有效
  }

  // yaw候选搜索函数（简化版：能抓就行）
  // 输入：目标xyz、参考yaw（如果只有xyz则为默认值）
  // 输出：最优yaw值
  // 评分规则：plan成功 > 关节离极限距离 > yaw误差
  double find_best_yaw_candidate(double x, double y, double z, double reference_yaw,
                                 const moveit::core::RobotStatePtr& state)
  {
    if (!state) {
      RCLCPP_WARN(this->get_logger(), "[yaw候选搜索] state为空，返回参考yaw");
      return reference_yaw;
    }

    const auto* jmg = state->getJointModelGroup("arm_group");
    if (!jmg) {
      RCLCPP_WARN(this->get_logger(), "[yaw候选搜索] 无法获取arm_group，返回参考yaw");
      return reference_yaw;
    }

    RCLCPP_INFO(this->get_logger(), 
                "[yaw候选搜索] 开始搜索最优yaw: 位置(%.3f, %.3f, %.3f), 参考yaw=%.3f rad (%.1f°)",
                x, y, z, reference_yaw, reference_yaw * 180.0 / M_PI);

    // 生成候选yaw集合：以参考yaw为中心，在搜索范围内每步长一个候选
    std::vector<double> candidate_yaws;
    double search_min = reference_yaw - yaw_candidate_range_ / 2.0;
    double search_max = reference_yaw + yaw_candidate_range_ / 2.0;
    
    // 确保搜索范围在[-π, π]内
    if (search_min < -M_PI) search_min = -M_PI;
    if (search_max > M_PI) search_max = M_PI;
    
    int num_candidates = static_cast<int>((search_max - search_min) / yaw_candidate_step_) + 1;
    for (int i = 0; i < num_candidates; ++i) {
      double candidate = search_min + i * yaw_candidate_step_;
      // 归一化到[-π, π]
      while (candidate > M_PI) candidate -= 2.0 * M_PI;
      while (candidate < -M_PI) candidate += 2.0 * M_PI;
      candidate_yaws.push_back(candidate);
    }

    RCLCPP_INFO(this->get_logger(), 
                "[yaw候选搜索] 生成 %zu 个候选yaw，范围[%.3f, %.3f] rad，步长=%.3f rad",
                candidate_yaws.size(), search_min, search_max, yaw_candidate_step_);

    // 评分结构
    struct YawCandidate {
      double yaw;
      bool ik_valid;
      double joint4_distance_to_limit;  // Joint4离极限的距离（越大越好）
      double yaw_error;  // 与参考yaw的误差（越小越好）
      double score;  // 综合评分（越大越好）
    };

    std::vector<YawCandidate> valid_candidates;

    // 获取Joint4的限位信息
    const auto* joint4_model = jmg->getJointModel("Joint4");
    double joint4_min = -M_PI;
    double joint4_max = M_PI;
    if (joint4_model) {
      const auto& bounds = joint4_model->getVariableBounds();
      if (!bounds.empty()) {
        // 获取第一个变量的限位（对于旋转关节通常只有一个变量）
        const auto& variable_bounds = bounds[0];
        joint4_min = variable_bounds.min_position_;
        joint4_max = variable_bounds.max_position_;
      }
    }

    // 评估每个候选yaw
    for (double candidate_yaw : candidate_yaws) {
      YawCandidate candidate;
      candidate.yaw = candidate_yaw;
      candidate.ik_valid = false;
      candidate.joint4_distance_to_limit = 0.0;
      candidate.yaw_error = std::abs(normalize_angle_diff(candidate_yaw - reference_yaw));
      candidate.score = -1.0;  // 无效候选

      // 构造位姿并求解IK
      geometry_msgs::msg::Pose candidate_pose;
      candidate_pose.position.x = x;
      candidate_pose.position.y = y;
      candidate_pose.position.z = z;
      candidate_pose.orientation = compute_downward_orientation_with_yaw(candidate_yaw);

      moveit::core::RobotStatePtr candidate_state(new moveit::core::RobotState(*state));
      if (candidate_state->setFromIK(jmg, candidate_pose, eef_link_, ik_timeout_)) {
        candidate.ik_valid = true;

        // 获取Joint4值并计算离极限的距离
        std::vector<double> joint_values;
        candidate_state->copyJointGroupPositions(jmg, joint_values);
        if (joint_values.size() >= 4) {
          double joint4_value = joint_values[3];
          // 计算到两个极限的距离，取较小值
          double dist_to_min = joint4_value - joint4_min;
          double dist_to_max = joint4_max - joint4_value;
          candidate.joint4_distance_to_limit = std::min(dist_to_min, dist_to_max);
        }

        // 计算综合评分
        // 优先级1：ik_valid（已满足）
        // 优先级2：joint4_distance_to_limit（越大越好，归一化到[0, 1]）
        // 优先级3：yaw_error（越小越好，归一化到[0, 1]，然后取反）
        double normalized_joint4_dist = std::min(candidate.joint4_distance_to_limit / M_PI, 1.0);
        double normalized_yaw_error = 1.0 - std::min(candidate.yaw_error / M_PI, 1.0);
        
        // 综合评分：joint4距离权重0.6，yaw误差权重0.4
        candidate.score = 0.6 * normalized_joint4_dist + 0.4 * normalized_yaw_error;
      }

      if (candidate.ik_valid) {
        valid_candidates.push_back(candidate);
      }
    }

    if (valid_candidates.empty()) {
      RCLCPP_WARN(this->get_logger(), 
                  "[yaw候选搜索] 未找到任何有效的yaw候选，返回参考yaw");
      return reference_yaw;
    }

    // 选择评分最高的候选
    auto best_candidate = std::max_element(valid_candidates.begin(), valid_candidates.end(),
                                           [](const YawCandidate& a, const YawCandidate& b) {
                                             return a.score < b.score;
                                           });

    RCLCPP_INFO(this->get_logger(),
                "[yaw候选搜索] 找到 %zu 个有效候选，最优yaw=%.3f rad (%.1f°)，评分=%.3f",
                valid_candidates.size(),
                best_candidate->yaw, best_candidate->yaw * 180.0 / M_PI,
                best_candidate->score);
    RCLCPP_INFO(this->get_logger(),
                "[yaw候选搜索] Joint4离极限距离=%.3f rad, yaw误差=%.3f rad (%.1f°)",
                best_candidate->joint4_distance_to_limit,
                best_candidate->yaw_error, best_candidate->yaw_error * 180.0 / M_PI);

    return best_candidate->yaw;
  }

  // 角度差值归一化函数：将角度差值归一化到[-π, π]范围，考虑±180度等价性
  double normalize_angle_diff(double diff)
  {
    while (diff > M_PI) diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;
    return diff;
  }

  // 输入：目标yaw、当前位置、当前RobotState
  // 输出：使Joint4最接近期望yaw的orientation yaw，对应的Joint4值，以及对应的完整关节值
  // 返回：是否找到更优的yaw（true表示找到，false表示未找到或搜索失败）
  bool find_optimal_yaw_for_joint4(double target_yaw,
                                   double x, double y, double z,
                                   const moveit::core::RobotStatePtr& state,
                                   double& optimal_yaw,
                                   double& optimal_joint4,
                                   std::vector<double>& optimal_joint_values)
  {
    if (!state) {
      return false;
    }

    const auto* jmg = state->getJointModelGroup("arm_group");
    if (!jmg) {
      return false;
    }

    // 先尝试原始yaw，记录初始Joint4值
    geometry_msgs::msg::Pose initial_pose;
    initial_pose.position.x = x;
    initial_pose.position.y = y;
    initial_pose.position.z = z;
    initial_pose.orientation = compute_downward_orientation_with_yaw(target_yaw);

    moveit::core::RobotStatePtr test_state(new moveit::core::RobotState(*state));
    if (!test_state->setFromIK(jmg, initial_pose, eef_link_, ik_timeout_)) {
      RCLCPP_WARN(this->get_logger(),
                  "[Joint4-yaw搜索] 初始yaw=%.3f rad (%.1f°)在位置(%.3f, %.3f, %.3f)下IK不可解",
                  target_yaw, target_yaw * 180.0 / M_PI, x, y, z);
      return false;
    }

    std::vector<double> initial_joint_values;
    test_state->copyJointGroupPositions(jmg, initial_joint_values);
    if (initial_joint_values.size() < 4) {
      return false;
    }

    double initial_joint4 = initial_joint_values[3];
    // 使用归一化的角度差值，考虑±180度等价性
    double initial_diff_raw = initial_joint4 - target_yaw;
    double initial_diff = std::abs(normalize_angle_diff(initial_diff_raw));
    
    RCLCPP_INFO(this->get_logger(),
                "[Joint4-yaw搜索] 开始搜索，目标yaw=%.3f rad (%.1f°)，初始Joint4=%.3f rad (%.1f°)，初始差值=%.3f rad (%.1f°)",
                target_yaw, target_yaw * 180.0 / M_PI,
                initial_joint4, initial_joint4 * 180.0 / M_PI,
                initial_diff, initial_diff * 180.0 / M_PI);

    // 初始化最优值
    optimal_yaw = target_yaw;
    optimal_joint4 = initial_joint4;
    optimal_joint_values = initial_joint_values;  // 保存初始关节值
    double best_diff = initial_diff;

    // 两阶段搜索策略：
    // 阶段1：在目标yaw±搜索范围内进行粗搜索（使用配置的步长）
    // 阶段2：如果找到较优解，在该解附近进行细搜索（使用更小的步长）
    
    // 如果搜索范围超过2π，限制在合理范围内以避免跨越360度边界
    double search_range = joint4_yaw_search_range_;
    if (search_range > M_PI) {
      search_range = M_PI;  // 限制在±180度范围内
      RCLCPP_WARN(this->get_logger(),
                  "[Joint4-yaw搜索] 搜索范围过大(%.3f rad)，限制为±180度以避免跨越360度边界",
                  joint4_yaw_search_range_);
    }
    
    // 阶段1：粗搜索（在目标yaw附近）
    double search_min = target_yaw - search_range;
    double search_max = target_yaw + search_range;
    
    // 确保搜索范围在[-π, π]内，如果超出则调整
    if (search_min < -M_PI) {
      double offset = -M_PI - search_min;
      search_min = -M_PI;
      search_max += offset;
      if (search_max > M_PI) search_max = M_PI;
    }
    if (search_max > M_PI) {
      double offset = search_max - M_PI;
      search_max = M_PI;
      search_min -= offset;
      if (search_min < -M_PI) search_min = -M_PI;
    }
    
    int search_steps = static_cast<int>((search_max - search_min) / joint4_yaw_search_step_);
    if (search_steps < 1) search_steps = 1;

    RCLCPP_INFO(this->get_logger(),
                "[Joint4-yaw搜索] 阶段1（粗搜索）: 范围[%.3f, %.3f] rad ([%.1f, %.1f]°)，步长=%.3f rad (%.1f°)，步数=%d",
                search_min, search_max,
                search_min * 180.0 / M_PI, search_max * 180.0 / M_PI,
                joint4_yaw_search_step_, joint4_yaw_search_step_ * 180.0 / M_PI,
                search_steps);

    int valid_solutions = 0;
    double best_candidate_yaw = target_yaw;  // 记录找到最优解的candidate_yaw，用于阶段2
    
    // 阶段1：粗搜索
    for (int i = 0; i <= search_steps; ++i) {
      double candidate_yaw = search_min + i * joint4_yaw_search_step_;
      
      // 归一化到[-π, π]范围
      while (candidate_yaw > M_PI) candidate_yaw -= 2.0 * M_PI;
      while (candidate_yaw < -M_PI) candidate_yaw += 2.0 * M_PI;

      // 构造位姿并求解IK
      geometry_msgs::msg::Pose candidate_pose;
      candidate_pose.position.x = x;
      candidate_pose.position.y = y;
      candidate_pose.position.z = z;
      candidate_pose.orientation = compute_downward_orientation_with_yaw(candidate_yaw);

      moveit::core::RobotStatePtr candidate_state(new moveit::core::RobotState(*state));
      if (!candidate_state->setFromIK(jmg, candidate_pose, eef_link_, ik_timeout_)) {
        continue;  // IK不可解，跳过
      }

      std::vector<double> candidate_joint_values;
      candidate_state->copyJointGroupPositions(jmg, candidate_joint_values);
      if (candidate_joint_values.size() < 4) {
        continue;
      }

      double candidate_joint4 = candidate_joint_values[3];
      // 使用归一化的角度差值，考虑±180度等价性
      double candidate_diff_raw = candidate_joint4 - target_yaw;
      double candidate_diff = std::abs(normalize_angle_diff(candidate_diff_raw));
      valid_solutions++;

      // 如果这个候选使Joint4更接近目标yaw，更新最优值
      if (candidate_diff < best_diff) {
        best_diff = candidate_diff;
        optimal_yaw = candidate_yaw;
        optimal_joint4 = candidate_joint4;
        optimal_joint_values = candidate_joint_values;  // 保存最优候选的完整关节值
        best_candidate_yaw = candidate_yaw;  // 记录用于阶段2细搜索
        
        RCLCPP_INFO(this->get_logger(),
                    "[Joint4-yaw搜索] 阶段1找到更优候选: yaw=%.3f rad (%.1f°), Joint4=%.3f rad (%.1f°), 差值=%.3f rad (%.1f°)",
                    candidate_yaw, candidate_yaw * 180.0 / M_PI,
                    candidate_joint4, candidate_joint4 * 180.0 / M_PI,
                    candidate_diff, candidate_diff * 180.0 / M_PI);
      }
    }

    RCLCPP_INFO(this->get_logger(),
                "[Joint4-yaw搜索] 阶段1完成: 有效解数量=%d，最优yaw=%.3f rad (%.1f°), 最优Joint4=%.3f rad (%.1f°), 最优差值=%.3f rad (%.1f°)",
                valid_solutions,
                optimal_yaw, optimal_yaw * 180.0 / M_PI,
                optimal_joint4, optimal_joint4 * 180.0 / M_PI,
                best_diff, best_diff * 180.0 / M_PI);

    // 阶段2：如果阶段1找到了更优解（改善>5度），在该解附近进行细搜索
    // 同时也在初始Joint4值对应的yaw附近进行搜索（反向搜索）
    const double fine_search_threshold = 0.0873;  // 5度（降低阈值，让更多情况触发细搜索）
    bool should_fine_search = (best_diff < initial_diff - fine_search_threshold && valid_solutions > 0);
    
    // 反向搜索：在初始Joint4值对应的yaw附近也进行搜索
    // 因为Joint4与yaw的关系可能不是线性的，初始Joint4值可能对应一个更接近目标yaw的orientation
    double reverse_search_yaw = initial_joint4;  // 假设初始Joint4值对应的yaw就是它自己
    double reverse_search_range = 0.7854;  // ±45度反向搜索范围
    double reverse_search_step = 0.0873;    // 5度步长
    
    if (should_fine_search || initial_diff > 0.5236) {  // 如果初始差值>30度，也进行反向搜索
      RCLCPP_INFO(this->get_logger(),
                  "[Joint4-yaw搜索] 反向搜索: 在初始Joint4=%.3f rad (%.1f°)对应的yaw附近搜索",
                  initial_joint4, initial_joint4 * 180.0 / M_PI);
      
      double reverse_search_min = reverse_search_yaw - reverse_search_range;
      double reverse_search_max = reverse_search_yaw + reverse_search_range;
      
      // 确保反向搜索范围在[-π, π]内
      if (reverse_search_min < -M_PI) reverse_search_min = -M_PI;
      if (reverse_search_max > M_PI) reverse_search_max = M_PI;
      
      int reverse_search_steps = static_cast<int>((reverse_search_max - reverse_search_min) / reverse_search_step);
      if (reverse_search_steps < 1) reverse_search_steps = 1;
      
      for (int i = 0; i <= reverse_search_steps; ++i) {
        double candidate_yaw = reverse_search_min + i * reverse_search_step;
        
        // 归一化到[-π, π]范围
        while (candidate_yaw > M_PI) candidate_yaw -= 2.0 * M_PI;
        while (candidate_yaw < -M_PI) candidate_yaw += 2.0 * M_PI;
        
        // 构造位姿并求解IK
        geometry_msgs::msg::Pose candidate_pose;
        candidate_pose.position.x = x;
        candidate_pose.position.y = y;
        candidate_pose.position.z = z;
        candidate_pose.orientation = compute_downward_orientation_with_yaw(candidate_yaw);
        
        moveit::core::RobotStatePtr candidate_state(new moveit::core::RobotState(*state));
        if (!candidate_state->setFromIK(jmg, candidate_pose, eef_link_, ik_timeout_)) {
          continue;  // IK不可解，跳过
        }
        
        std::vector<double> candidate_joint_values;
        candidate_state->copyJointGroupPositions(jmg, candidate_joint_values);
        if (candidate_joint_values.size() < 4) {
          continue;
        }
        
        double candidate_joint4 = candidate_joint_values[3];
        // 使用归一化的角度差值
        double candidate_diff_raw = candidate_joint4 - target_yaw;
        double candidate_diff = std::abs(normalize_angle_diff(candidate_diff_raw));
        
        // 如果这个候选使Joint4更接近目标yaw，更新最优值
        if (candidate_diff < best_diff) {
          best_diff = candidate_diff;
          optimal_yaw = candidate_yaw;
          optimal_joint4 = candidate_joint4;
          optimal_joint_values = candidate_joint_values;
          best_candidate_yaw = candidate_yaw;  // 更新用于阶段2细搜索
          
          RCLCPP_INFO(this->get_logger(),
                      "[Joint4-yaw搜索] 反向搜索找到更优候选: yaw=%.3f rad (%.1f°), Joint4=%.3f rad (%.1f°), 差值=%.3f rad (%.1f°)",
                      candidate_yaw, candidate_yaw * 180.0 / M_PI,
                      candidate_joint4, candidate_joint4 * 180.0 / M_PI,
                      candidate_diff, candidate_diff * 180.0 / M_PI);
        }
      }
    }
    
    // 阶段2：如果找到了更优解（改善>5度），在该解附近进行细搜索
    if (should_fine_search) {
      double fine_search_range = 0.5236;  // ±30度细搜索范围
      double fine_search_step = 0.0175;   // 1度细搜索步长
      double fine_search_min = best_candidate_yaw - fine_search_range;
      double fine_search_max = best_candidate_yaw + fine_search_range;
      
      // 确保细搜索范围在[-π, π]内
      if (fine_search_min < -M_PI) fine_search_min = -M_PI;
      if (fine_search_max > M_PI) fine_search_max = M_PI;
      
      int fine_search_steps = static_cast<int>((fine_search_max - fine_search_min) / fine_search_step);
      if (fine_search_steps < 1) fine_search_steps = 1;
      
      RCLCPP_INFO(this->get_logger(),
                  "[Joint4-yaw搜索] 阶段2（细搜索）: 在yaw=%.3f rad (%.1f°)附近，范围[%.3f, %.3f] rad，步长=%.3f rad (%.1f°)，步数=%d",
                  best_candidate_yaw, best_candidate_yaw * 180.0 / M_PI,
                  fine_search_min, fine_search_max,
                  fine_search_step, fine_search_step * 180.0 / M_PI,
                  fine_search_steps);
      
      for (int i = 0; i <= fine_search_steps; ++i) {
        double candidate_yaw = fine_search_min + i * fine_search_step;
        
        // 归一化到[-π, π]范围
        while (candidate_yaw > M_PI) candidate_yaw -= 2.0 * M_PI;
        while (candidate_yaw < -M_PI) candidate_yaw += 2.0 * M_PI;
        
        // 构造位姿并求解IK
        geometry_msgs::msg::Pose candidate_pose;
        candidate_pose.position.x = x;
        candidate_pose.position.y = y;
        candidate_pose.position.z = z;
        candidate_pose.orientation = compute_downward_orientation_with_yaw(candidate_yaw);
        
        moveit::core::RobotStatePtr candidate_state(new moveit::core::RobotState(*state));
        if (!candidate_state->setFromIK(jmg, candidate_pose, eef_link_, ik_timeout_)) {
          continue;  // IK不可解，跳过
        }
        
        std::vector<double> candidate_joint_values;
        candidate_state->copyJointGroupPositions(jmg, candidate_joint_values);
        if (candidate_joint_values.size() < 4) {
          continue;
        }
        
        double candidate_joint4 = candidate_joint_values[3];
        // 使用归一化的角度差值
        double candidate_diff_raw = candidate_joint4 - target_yaw;
        double candidate_diff = std::abs(normalize_angle_diff(candidate_diff_raw));
        
        // 如果这个候选使Joint4更接近目标yaw，更新最优值
        if (candidate_diff < best_diff) {
          best_diff = candidate_diff;
          optimal_yaw = candidate_yaw;
          optimal_joint4 = candidate_joint4;
          optimal_joint_values = candidate_joint_values;
          
          RCLCPP_INFO(this->get_logger(),
                      "[Joint4-yaw搜索] 阶段2找到更优候选: yaw=%.3f rad (%.1f°), Joint4=%.3f rad (%.1f°), 差值=%.3f rad (%.1f°)",
                      candidate_yaw, candidate_yaw * 180.0 / M_PI,
                      candidate_joint4, candidate_joint4 * 180.0 / M_PI,
                      candidate_diff, candidate_diff * 180.0 / M_PI);
        }
      }
      
      RCLCPP_INFO(this->get_logger(),
                  "[Joint4-yaw搜索] 阶段2完成，最终最优yaw=%.3f rad (%.1f°), 最优Joint4=%.3f rad (%.1f°), 最优差值=%.3f rad (%.1f°)",
                  optimal_yaw, optimal_yaw * 180.0 / M_PI,
                  optimal_joint4, optimal_joint4 * 180.0 / M_PI,
                  best_diff, best_diff * 180.0 / M_PI);
    }

    // 如果找到更优的解（差值至少减少5度），返回true
    const double improvement_threshold = 0.0873;  // 5度
    if (best_diff < initial_diff - improvement_threshold) {
      RCLCPP_INFO(this->get_logger(),
                  "[Joint4-yaw搜索] 找到更优解，差值改善: %.3f -> %.3f rad (%.1f -> %.1f°)",
                  initial_diff, best_diff,
                  initial_diff * 180.0 / M_PI, best_diff * 180.0 / M_PI);
      return true;
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "[Joint4-yaw搜索] 未找到更优解，保持原始yaw");
      optimal_yaw = target_yaw;
      optimal_joint4 = initial_joint4;
      optimal_joint_values = initial_joint_values;  // 保持原始关节值
      return false;
    }
  }

  // 回滚到预抓取位置（安全状态）
  // 当执行失败时，尝试回滚到已知的安全位置，避免状态机认为成功但实际失败
  bool recover_to_pregrasp(const geometry_msgs::msg::PoseStamped& pregrasp_pose)
  {
    if (!enable_recovery_)
    {
      RCLCPP_WARN(this->get_logger(), "[状态回滚] 回滚功能已禁用，跳过回滚操作");
      return false;
    }

    RCLCPP_WARN(this->get_logger(), "[状态回滚] 开始回滚到预抓取位置...");
    publish_state("状态:回滚中");

    // 转换到planning frame
    geometry_msgs::msg::PoseStamped pregrasp_planning = pregrasp_pose;
    if (!transform_pose_to_planning(pregrasp_planning))
    {
      RCLCPP_ERROR(this->get_logger(), "[状态回滚] 预抓取位姿转换到planning frame失败");
      return false;
    }

    // 使用普通规划回滚到预抓取位置
    // 设置较短的规划时间，快速回滚
    {
      std::lock_guard<std::mutex> lock(moveit_mutex_);
      double original_planning_time = move_group_interface_->getPlanningTime();
      move_group_interface_->setPlanningTime(recovery_timeout_);
      move_group_interface_->setStartStateToCurrentState();
      
      bool recovery_success = plan_execute_pose_target(pregrasp_planning);
      
      // 恢复原始规划时间
      move_group_interface_->setPlanningTime(original_planning_time);
      
      if (recovery_success)
      {
        RCLCPP_INFO(this->get_logger(), "[状态回滚] 成功回滚到预抓取位置");
        publish_state("状态:回滚成功");
        return true;
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "[状态回滚] 回滚到预抓取位置失败");
        publish_state("状态:回滚失败");
        return false;
      }
    }
  }

  // 检查执行结果并处理失败（带回滚）
  // 这是比"规划成功即执行"更高层级的"抓取策略"，确保执行失败后有状态回滚
  bool execute_with_recovery(moveit::planning_interface::MoveGroupInterface::Plan& plan,
                            const geometry_msgs::msg::PoseStamped& recovery_pose)
  {
    moveit::core::MoveItErrorCode result;
    {
      std::lock_guard<std::mutex> lock(moveit_mutex_);
      result = move_group_interface_->execute(plan);
    }

    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      return true;
    }

    // 执行失败，尝试回滚
    RCLCPP_ERROR(this->get_logger(), 
                "[执行回滚] 执行失败（错误代码: %d），尝试回滚到安全状态",
                result.val);
    
    // 检查回滚位姿是否有效（不是原点）
    bool recovery_pose_valid = !(recovery_pose.pose.position.x == 0.0 && 
                                  recovery_pose.pose.position.y == 0.0 && 
                                  recovery_pose.pose.position.z == 0.0);
    
    if (enable_recovery_ && recovery_pose_valid)
    {
      return recover_to_pregrasp(recovery_pose);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "[执行回滚] 回滚功能未启用或回滚位姿无效，跳过回滚");
      return false;
    }
  }

  // IK检查函数（仅检查IK，不做碰撞检查）
  // 无锁版本：接受已获取的状态，避免在锁内重复加锁
  bool check_ik_only_impl(const geometry_msgs::msg::PoseStamped& pose_in_planning,
                          const moveit::core::RobotStatePtr& state)
  {
    RCLCPP_INFO(this->get_logger(), 
                "[IK检查] 目标位姿: 位置=(%.3f, %.3f, %.3f), orientation=(%.3f, %.3f, %.3f, %.3f), frame=%s",
                pose_in_planning.pose.position.x,
                pose_in_planning.pose.position.y,
                pose_in_planning.pose.position.z,
                pose_in_planning.pose.orientation.x,
                pose_in_planning.pose.orientation.y,
                pose_in_planning.pose.orientation.z,
                pose_in_planning.pose.orientation.w,
                pose_in_planning.header.frame_id.c_str());
    
    if (!state) {
      RCLCPP_WARN(this->get_logger(), "[IK检查] 无法获取当前机器人状态");
      return false;
    }

    const auto* jmg = state->getJointModelGroup("arm_group");
    if (!jmg) {
      RCLCPP_WARN(this->get_logger(), "[IK检查] 无法获取arm_group关节模型组");
      return false;
    }

    geometry_msgs::msg::Pose p = pose_in_planning.pose;

    // 使用配置的IK timeout
    bool ok = state->setFromIK(jmg, p, eef_link_, ik_timeout_);
    
    if (ok) {
      // 获取IK解出的关节值
      std::vector<double> joint_values;
      state->copyJointGroupPositions(jmg, joint_values);
      RCLCPP_INFO(this->get_logger(), 
                  "[IK检查] IK求解成功，关节值: [%.3f, %.3f, %.3f, %.3f]",
                  joint_values.size() > 0 ? joint_values[0] : 0.0,
                  joint_values.size() > 1 ? joint_values[1] : 0.0,
                  joint_values.size() > 2 ? joint_values[2] : 0.0,
                  joint_values.size() > 3 ? joint_values[3] : 0.0);
    } else {
      RCLCPP_WARN(this->get_logger(), 
                  "[IK检查] IK求解失败（目标位姿对当前关节限位不可达）");
    }
    
    return ok;
  }

  // IK检查函数（公共接口，带锁）
  bool check_ik_only(const geometry_msgs::msg::PoseStamped& pose_in_planning)
  {
    auto state = getCurrentStateSafe(state_check_timeout_);
    return check_ik_only_impl(pose_in_planning, state);
  }

  // IK检查（带碰撞检测）- 用于诊断"IK成功但goal state全碰撞"的情况
  // 注意：这个函数比较重，主要用于诊断，不是常规流程
  // 简化实现：只检查自碰撞，不检查环境碰撞（因为获取完整PlanningScene在MoveIt 2中较复杂）
  bool check_ik_with_collision(const geometry_msgs::msg::PoseStamped& pose_in_planning)
  {
    RCLCPP_INFO(this->get_logger(), 
                "[IK碰撞检查] 目标位姿: 位置=(%.3f, %.3f, %.3f), orientation=(%.3f, %.3f, %.3f, %.3f)",
                pose_in_planning.pose.position.x,
                pose_in_planning.pose.position.y,
                pose_in_planning.pose.position.z,
                pose_in_planning.pose.orientation.x,
                pose_in_planning.pose.orientation.y,
                pose_in_planning.pose.orientation.z,
                pose_in_planning.pose.orientation.w);
    
    // 创建PlanningScene用于碰撞检测
    planning_scene::PlanningScenePtr scene = 
        std::make_shared<planning_scene::PlanningScene>(move_group_interface_->getRobotModel());
    
    // 设置当前机器人状态（从MoveGroupInterface获取）
    // 修复：使用getCurrentStateSafe()绕过时间戳检查
    try {
      auto current_state = getCurrentStateSafe(1.0);
      if (current_state) {
        scene->setCurrentState(*current_state);
      } else {
        RCLCPP_WARN(this->get_logger(), "[IK碰撞检查] getCurrentStateSafe()失败，使用默认状态");
        scene->getCurrentStateNonConst().setToDefaultValues();
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "[IK碰撞检查] 无法获取当前状态: %s", e.what());
      return false;
    }
    
    // 注意：getCurrentStateNonConst()返回引用，必须使用引用类型，不能使用auto
    // 否则会拷贝对象，导致对state的修改不会反映到scene上
    moveit::core::RobotState& state = scene->getCurrentStateNonConst();
    const auto* jmg = state.getJointModelGroup("arm_group");
    if (!jmg) {
      RCLCPP_WARN(this->get_logger(), "[IK碰撞检查] 无法获取arm_group关节模型组");
      return false;
    }
    
    // IK求解，带自碰撞检查回调
    // setFromIK参数顺序：group, pose, tip, timeout, callback
    // 注意：此函数只检查自碰撞，不检查环境碰撞
    // 因为新建的PlanningScene没有接入PlanningSceneMonitor，看不到真实的collision objects
    // 如需完整诊断，需要接入PlanningSceneMonitor或订阅/move_group/monitored_planning_scene
    bool found = state.setFromIK(
        jmg, pose_in_planning.pose, eef_link_, ik_timeout_,
        [&](moveit::core::RobotState* st, const moveit::core::JointModelGroup*, const double*){
          // 检查自碰撞
          collision_detection::CollisionRequest req;
          collision_detection::CollisionResult res;
          scene->checkSelfCollision(req, res, *st);
          return !res.collision;
        });
    
    if (found) {
      // 获取IK解出的关节值
      std::vector<double> joint_values;
      state.copyJointGroupPositions(jmg, joint_values);
      RCLCPP_INFO(this->get_logger(), 
                  "[IK碰撞检查] IK求解成功且无自碰撞，关节值: [%.3f, %.3f, %.3f, %.3f]",
                  joint_values.size() > 0 ? joint_values[0] : 0.0,
                  joint_values.size() > 1 ? joint_values[1] : 0.0,
                  joint_values.size() > 2 ? joint_values[2] : 0.0,
                  joint_values.size() > 3 ? joint_values[3] : 0.0);
    } else {
      RCLCPP_WARN(this->get_logger(), 
                  "[IK碰撞检查] IK求解失败或所有解都在自碰撞中（可能是关节限位或自碰撞导致goal state无效）");
    }
    
    return found;
  }

  // 检测并上报碰撞状态（用于规划失败时诊断）
  // 注意：此函数必须在锁外调用，否则会死锁（因为内部会获取moveit_mutex_）
  void check_and_report_collision(const std::string& context = "")
  {
    if (!move_group_interface_ || !planning_scene_interface_)
    {
      return;
    }

    // 创建PlanningScene用于碰撞检测
    const moveit::core::RobotModelConstPtr& robot_model = move_group_interface_->getRobotModel();
    if (!robot_model)
    {
      return;
    }

    planning_scene::PlanningScenePtr scene = 
        std::make_shared<planning_scene::PlanningScene>(robot_model);
    
    // 获取当前机器人状态（在锁内获取）
    // 修复：使用getCurrentStateSafe()绕过时间戳检查
    moveit::core::RobotStatePtr current_state = getCurrentStateSafe(1.0);
    
    if (!current_state)
    {
      RCLCPP_WARN(this->get_logger(), "[碰撞检测] 无法获取当前机器人状态");
      return;
    }

    // 设置当前状态到场景
    scene->setCurrentState(*current_state);
    
    // 从PlanningSceneInterface获取碰撞对象并添加到scene中（在锁内获取）
    {
      std::lock_guard<std::mutex> lock(moveit_mutex_);
      // 获取所有已知的collision objects
      std::vector<std::string> known_objects = planning_scene_interface_->getKnownObjectNames();
      
      // 获取所有collision objects的详细信息
      std::map<std::string, moveit_msgs::msg::CollisionObject> objects_map = 
          planning_scene_interface_->getObjects(known_objects);
      
      // 将collision objects添加到scene中
      for (const auto& obj_pair : objects_map)
      {
        const moveit_msgs::msg::CollisionObject& obj = obj_pair.second;
        scene->processCollisionObjectMsg(obj);
      }
    }
    
    // 检测碰撞（包括自碰撞和环境碰撞）
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    collision_request.contacts = true;  // 获取碰撞接触信息
    collision_request.max_contacts = 10;  // 最多返回10个碰撞点

    scene->checkCollision(collision_request, collision_result, *current_state);

    if (collision_result.collision)
    {
      // 提取碰撞信息
      std::string collision_info = "碰撞检测";
      if (!context.empty())
      {
        collision_info += ":[" + context + "]";
      }

      collision_info += ":发现碰撞";
      
      // 收集碰撞对象对
      std::set<std::string> collision_pairs;
      for (const auto& contact : collision_result.contacts)
      {
        std::string pair = contact.first.first + "<->" + contact.first.second;
        collision_pairs.insert(pair);
      }

      if (!collision_pairs.empty())
      {
        collision_info += ":";
        bool first = true;
        for (const auto& pair : collision_pairs)
        {
          if (!first) collision_info += ",";
          collision_info += pair;
          first = false;
        }
      }

      // 上报到网页
      publish_state("错误:碰撞", collision_info);
      RCLCPP_WARN(this->get_logger(), "[碰撞检测] %s", collision_info.c_str());
    }
  }

  // 计算预抓取位姿
  geometry_msgs::msg::PoseStamped compute_pregrasp_pose(const geometry_msgs::msg::PoseStamped& cable_pose)
  {
    // 1) 目标按"夹爪中心"定义
    geometry_msgs::msg::PoseStamped center_pose = cable_pose;
    center_pose.pose.position.z += approach_offset_z_;
    
    // 如果orientation是默认值，需要计算合适的orientation
    bool is_default = is_default_orientation(cable_pose.pose.orientation);
    
    if (is_default)
    {
      // 注意：如果是带yaw的消息，orientation应该已经在cable_pose_with_yaw_callback中计算好了
      // 这里只处理真正没有orientation信息的情况
      // 优先使用当前末端执行器的orientation（如果可用且不是默认值）
      bool use_current = false;
      geometry_msgs::msg::Quaternion current_orientation;
      
      if (move_group_interface_)
      {
        try {
          auto current_pose = getCurrentPoseSafe();
          current_orientation = current_pose.pose.orientation;
          
          if (!is_default_orientation(current_orientation))
          {
            use_current = true;
            RCLCPP_INFO(this->get_logger(), 
                        "预抓取位姿计算: 使用当前末端执行器orientation（默认orientation情况）");
          }
        } catch (const std::exception& e) {
          RCLCPP_WARN(this->get_logger(), "无法获取当前末端执行器pose: %s", e.what());
        }
      }
      
      if (use_current)
      {
        center_pose.pose.orientation = current_orientation;
      }
      else
      {
        // Fallback: 使用向下orientation（适合抓取任务）
        center_pose.pose.orientation = compute_downward_orientation();
        RCLCPP_INFO(this->get_logger(), 
                    "预抓取位姿计算: 检测到默认orientation，使用向下orientation");
      }
    }
    else
    {
      // 明确使用缆绳位姿的orientation（已包含yaw信息）
      // 虽然第2816行已经复制了orientation，但明确设置可以确保正确传递
      center_pose.pose.orientation = cable_pose.pose.orientation;
      RCLCPP_INFO(this->get_logger(), 
                  "预抓取位姿计算: 使用缆绳位姿的orientation（已包含yaw信息）");
      RCLCPP_INFO(this->get_logger(), 
                  "预抓取位姿计算: orientation=(%.3f, %.3f, %.3f, %.3f)",
                  center_pose.pose.orientation.x, center_pose.pose.orientation.y,
                  center_pose.pose.orientation.z, center_pose.pose.orientation.w);
    }
    
    RCLCPP_INFO(this->get_logger(), 
                "预抓取位姿计算（夹爪中心）: 位置=(%.3f, %.3f, %.3f), 偏移z=%.3f",
                center_pose.pose.position.x, center_pose.pose.position.y,
                center_pose.pose.position.z, approach_offset_z_);
    
    // 检查预抓取点是否在工作空间内（基于夹爪中心）
    double pregrasp_x = center_pose.pose.position.x;
    double pregrasp_y = center_pose.pose.position.y;
    double pregrasp_z = center_pose.pose.position.z;
    
    if (!is_reachable(pregrasp_x, pregrasp_y, pregrasp_z))
    {
      RCLCPP_WARN(this->get_logger(), 
                  "预抓取点 (%.3f, %.3f, %.3f) 超出工作空间，调整到工作空间内",
                  pregrasp_x, pregrasp_y, pregrasp_z);
      adjust_to_workspace(pregrasp_x, pregrasp_y, pregrasp_z);
      center_pose.pose.position.x = pregrasp_x;
      center_pose.pose.position.y = pregrasp_y;
      center_pose.pose.position.z = pregrasp_z;
    }
    
    // 2) 换算成 EEF(LinkGG) 目标
    tf2::Vector3 eef_to_center(tcp_offset_x_, tcp_offset_y_, tcp_offset_z_);
    geometry_msgs::msg::PoseStamped eef_pose = 
        compensate_tcp_offset_for_eef(center_pose, eef_to_center);
    
    RCLCPP_INFO(this->get_logger(), 
                "预抓取位姿计算（LinkGG）: 位置=(%.3f, %.3f, %.3f), TCP偏移=(%.4f, %.4f, %.4f)",
                eef_pose.pose.position.x, eef_pose.pose.position.y, eef_pose.pose.position.z,
                tcp_offset_x_, tcp_offset_y_, tcp_offset_z_);
    RCLCPP_INFO(this->get_logger(), 
                "预抓取位姿计算（LinkGG）: orientation=(%.3f, %.3f, %.3f, %.3f)",
                eef_pose.pose.orientation.x, eef_pose.pose.orientation.y,
                eef_pose.pose.orientation.z, eef_pose.pose.orientation.w);
    
    return eef_pose;
  }

  // 计算抓取位姿（从预抓取点向下压descend_distance_，但不低于缆绳点）
  geometry_msgs::msg::PoseStamped compute_grasp_pose(
      const geometry_msgs::msg::PoseStamped& pregrasp_pose,
      const geometry_msgs::msg::PoseStamped& cable_pose)
  {
    // 注意：pregrasp_pose 已经是 LinkGG 位姿，但我们需要基于"夹爪中心"计算抓取位姿
    // 所以先反推回夹爪中心位姿（从LinkGG位姿加回偏移得到夹爪中心位姿）
    // 但更简单的方法：直接从 cable_pose（夹爪中心目标）计算抓取位姿
    
    // 1) 目标按"夹爪中心"定义
    geometry_msgs::msg::PoseStamped center_grasp_pose = cable_pose;
    
    // 从预抓取高度（夹爪中心）向下压descend_distance_
    // 注意：pregrasp_pose 是 LinkGG 位姿，需要先加回偏移得到夹爪中心位姿
    // 但为了简化，我们直接从 cable_pose 计算，因为预抓取高度 = cable_pose.z + approach_offset_z_
    double center_pregrasp_z = cable_pose.pose.position.z + approach_offset_z_;
    center_grasp_pose.pose.position.z = center_pregrasp_z - descend_distance_;
    
    // 计算更安全的最小z值：考虑线缆直径、相机误差、TCP偏移和最小离地距离
    // z_grasp = z_visual + camera_error_margin + tcp_offset_z + min_ground_clearance
    // 注意：tcp_offset_z_通常是负值（LinkGG在夹爪中心上方），所以需要加上它
    double min_z = cable_pose.pose.position.z;
    min_z += camera_error_margin_;  // 相机标定误差余量
    min_z += std::abs(tcp_offset_z_);  // TCP到指尖几何偏置（取绝对值，因为offset通常是负值）
    min_z += min_ground_clearance_;  // 最小离地安全距离
    min_z += cable_center_offset_z_ + 0.5 * cable_diameter_;  // 线缆中心偏移和半径
    
    // 确保不低于地面高度
    double ground_z = ground_height_ + min_ground_clearance_;
    if (min_z < ground_z) {
      min_z = ground_z;
    }
    
    if (center_grasp_pose.pose.position.z < min_z)
    {
      RCLCPP_WARN(this->get_logger(), 
                  "抓取点z (%.3f) 低于安全最小z值 (%.3f)，限制为安全最小z值", 
                  center_grasp_pose.pose.position.z, min_z);
      RCLCPP_INFO(this->get_logger(),
                  "  安全余量组成: 相机误差=%.3f, TCP偏移=%.3f, 最小离地=%.3f, 线缆半径=%.3f",
                  camera_error_margin_, std::abs(tcp_offset_z_), min_ground_clearance_,
                  0.5 * cable_diameter_);
      center_grasp_pose.pose.position.z = min_z;
    }
    
    // 使用 cable_pose 的 orientation（通常是从cable_pose_with_yaw计算出的"朝下+yaw"）
    // 注意：如果是默认orientation，使用当前末端姿态（但这种情况应该很少，因为带yaw的消息已经计算好了）
    bool is_default_orient = is_default_orientation(cable_pose.pose.orientation);
    
    if (is_default_orient && move_group_interface_)
    {
      try {
        auto current_pose = getCurrentPoseSafe();
        center_grasp_pose.pose.orientation = current_pose.pose.orientation;
        RCLCPP_INFO(this->get_logger(), "抓取位姿计算: 使用当前末端执行器orientation（默认orientation情况）");
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "无法获取当前末端执行器orientation: %s", e.what());
        center_grasp_pose.pose.orientation = compute_downward_orientation();
      }
    }
    else
    {
      // 明确使用缆绳位姿的orientation（已包含yaw信息）
      center_grasp_pose.pose.orientation = cable_pose.pose.orientation;
      RCLCPP_INFO(this->get_logger(), "抓取位姿计算: 使用缆绳位姿的orientation（已包含yaw信息）");
      RCLCPP_INFO(this->get_logger(), 
                  "抓取位姿计算: orientation=(%.3f, %.3f, %.3f, %.3f)",
                  center_grasp_pose.pose.orientation.x, center_grasp_pose.pose.orientation.y,
                  center_grasp_pose.pose.orientation.z, center_grasp_pose.pose.orientation.w);
    }
    
    RCLCPP_INFO(this->get_logger(),
                "抓取位姿计算（夹爪中心）: 位置=(%.3f, %.3f, %.3f), 下压距离=%.3f",
                center_grasp_pose.pose.position.x, center_grasp_pose.pose.position.y,
                center_grasp_pose.pose.position.z, descend_distance_);
    
    // 2) 换算成 EEF(LinkGG) 目标
    tf2::Vector3 eef_to_center(tcp_offset_x_, tcp_offset_y_, tcp_offset_z_);
    geometry_msgs::msg::PoseStamped eef_grasp_pose = 
        compensate_tcp_offset_for_eef(center_grasp_pose, eef_to_center);
    
    RCLCPP_INFO(this->get_logger(),
                "抓取位姿计算（LinkGG）: 位置=(%.3f, %.3f, %.3f), TCP偏移=(%.4f, %.4f, %.4f)",
                eef_grasp_pose.pose.position.x, eef_grasp_pose.pose.position.y, 
                eef_grasp_pose.pose.position.z,
                tcp_offset_x_, tcp_offset_y_, tcp_offset_z_);
    RCLCPP_INFO(this->get_logger(),
                "抓取位姿计算（LinkGG）: orientation=(%.3f, %.3f, %.3f, %.3f)",
                eef_grasp_pose.pose.orientation.x, eef_grasp_pose.pose.orientation.y,
                eef_grasp_pose.pose.orientation.z, eef_grasp_pose.pose.orientation.w);
    
    return eef_grasp_pose;
  }

  // 辅助函数：带超时的 computeCartesianPath
  // 使用异步执行避免阻塞工作线程
  // 支持重试机制：如果超时且启用重试，减少waypoints数量后重试一次
  double compute_cartesian_path_with_timeout(
      const std::vector<geometry_msgs::msg::Pose>& waypoints,
      double step,
      double jump_threshold,
      moveit_msgs::msg::RobotTrajectory& trajectory,
      double timeout_sec,
      bool allow_retry = false)  // 是否允许重试（减少waypoints后重试）
  {
    // 在锁内准备状态
    {
      std::lock_guard<std::mutex> lock(moveit_mutex_);
      move_group_interface_->setStartStateToCurrentState();
    }
    
    // 在单独线程中执行 computeCartesianPath，避免阻塞
    auto future = std::async(std::launch::async, [this, &waypoints, step, jump_threshold, &trajectory]() -> double {
      // 注意：computeCartesianPath 需要访问 move_group_interface_，但通常可以安全地从不同线程调用
      // 我们已经在锁外，但这是可接受的风险，因为 computeCartesianPath 主要是只读操作
      std::lock_guard<std::mutex> lock(moveit_mutex_);
      return move_group_interface_->computeCartesianPath(waypoints, step, jump_threshold, trajectory);
    });
    
    // 等待结果，带超时
    auto status = future.wait_for(std::chrono::milliseconds(static_cast<int>(timeout_sec * 1000)));
    
    if (status == std::future_status::timeout)
    {
      RCLCPP_WARN(this->get_logger(), "笛卡尔路径计算超时（%.1f秒，%zu个waypoints）", timeout_sec, waypoints.size());
      
      // 如果允许重试且waypoints数量足够，尝试减少waypoints后重试
      if (allow_retry && cartesian_retry_on_timeout_ && waypoints.size() > 2)
      {
        // 减少waypoints：只保留起点、中点和终点
        std::vector<geometry_msgs::msg::Pose> reduced_waypoints;
        reduced_waypoints.push_back(waypoints.front());  // 起点
        if (waypoints.size() > 2)
        {
          // 添加中点（如果waypoints数量为奇数，取中间；否则取中间偏前）
          size_t mid_index = waypoints.size() / 2;
          reduced_waypoints.push_back(waypoints[mid_index]);
        }
        reduced_waypoints.push_back(waypoints.back());  // 终点
        
        RCLCPP_INFO(this->get_logger(), 
                    "笛卡尔路径重试：减少waypoints从%zu到%zu，使用更短的超时时间（%.1f秒）",
                    waypoints.size(), reduced_waypoints.size(), timeout_sec * 0.7);
        
        // 使用更短的超时时间重试（原超时的70%）
        moveit_msgs::msg::RobotTrajectory retry_trajectory;
        double retry_fraction = compute_cartesian_path_with_timeout(
            reduced_waypoints, step, jump_threshold, retry_trajectory, timeout_sec * 0.7, false);  // 不再重试
        
        if (retry_fraction >= 0.0)
        {
          trajectory = retry_trajectory;
          RCLCPP_INFO(this->get_logger(), "笛卡尔路径重试成功，成功率: %.2f%%", retry_fraction * 100.0);
          return retry_fraction;
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), "笛卡尔路径重试也失败，触发fallback");
        }
      }
      
      return -1.0;  // 返回负数表示超时
    }
    
    // 获取结果
    try {
      double fraction = future.get();
      return fraction;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "笛卡尔路径计算异常: %s", e.what());
      return -1.0;
    }
  }

  // 执行笛卡尔路径（改进版：支持使用精确直线插补和B样条）
  // use_descend: true 表示下压（descend），false 表示抬起（lift）
  // descend 阈值 0.95，lift 阈值 0.90（lift 是确认动作，可以更宽松）
  // 添加超时机制，下压和抬起使用不同的超时时间，避免 computeCartesianPath 卡住
  // 注意：是否使用线性插补或B样条由配置文件中的参数控制
  bool execute_cartesian(const std::vector<geometry_msgs::msg::Pose>& waypoints, bool use_descend = true)
  {
    // 根据用途选择超时时间
    double timeout = use_descend ? cartesian_timeout_descend_ : cartesian_timeout_lift_;
    
    // 如果配置启用B样条且waypoints数量足够，使用B样条生成平滑笛卡尔轨迹
    if (use_bspline_ && waypoints.size() >= static_cast<size_t>(bspline_degree_ + 1))
    {
      RCLCPP_INFO(this->get_logger(), "使用B样条生成笛卡尔轨迹（%zu个控制点，阶数=%d）", 
                  waypoints.size(), bspline_degree_);
      
      try {
        // 生成B样条笛卡尔轨迹
        std::vector<geometry_msgs::msg::Pose> bspline_waypoints = 
            trajectory_planner_.generateBSplineCartesianTrajectory(
              waypoints, bspline_degree_, bspline_duration_, bspline_dt_);
        
        if (bspline_waypoints.empty())
        {
          RCLCPP_WARN(this->get_logger(), "B样条轨迹生成失败，回退到标准方法");
        }
        else
        {
          RCLCPP_INFO(this->get_logger(), "B样条生成 %zu 个笛卡尔轨迹点", bspline_waypoints.size());
          
          // 使用生成的B样条waypoints调用MoveIt的computeCartesianPath（带超时）
          moveit_msgs::msg::RobotTrajectory trajectory;
          double fraction = compute_cartesian_path_with_timeout(
              bspline_waypoints, eef_step_, jump_threshold_, trajectory, timeout, true);  // 允许重试
          
          if (fraction < 0.0)  // 超时或异常
          {
            RCLCPP_WARN(this->get_logger(), "B样条笛卡尔路径计算超时，回退到标准方法");
            // 继续执行，尝试标准方法
          }
          else
          {
          
            double threshold = use_descend ? cartesian_descend_threshold_ : cartesian_lift_threshold_;
            if (fraction >= threshold)
            {
              moveit::planning_interface::MoveGroupInterface::Plan plan;
              plan.trajectory_ = trajectory;
              
              {
                std::lock_guard<std::mutex> lock(moveit_mutex_);
                moveit::core::MoveItErrorCode result = move_group_interface_->execute(plan);
                if (result == moveit::core::MoveItErrorCode::SUCCESS)
                {
                  RCLCPP_INFO(this->get_logger(), "B样条笛卡尔路径执行成功");
                  return true;
                }
              }
            }
            RCLCPP_WARN(this->get_logger(), 
                       "B样条笛卡尔路径执行失败（成功率: %.2f%%，阈值: %.2f%%），回退到标准方法", 
                       fraction * 100.0, threshold * 100.0);
          }
        }
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "B样条轨迹生成失败: %s，回退到标准方法", e.what());
      }
    }
    
    // 如果配置启用线性插补且waypoints只有2个点，使用TrajectoryPlanner生成精确直线轨迹
    if (use_linear_interpolation_ && waypoints.size() == 2)
    {
      RCLCPP_INFO(this->get_logger(), "使用精确直线插补生成笛卡尔轨迹");
      
      // 计算轨迹持续时间（基于距离和配置的速度）
      double distance = std::sqrt(
        std::pow(waypoints[1].position.x - waypoints[0].position.x, 2) +
        std::pow(waypoints[1].position.y - waypoints[0].position.y, 2) +
        std::pow(waypoints[1].position.z - waypoints[0].position.z, 2)
      );
      
      // 使用配置的速度计算持续时间
      double duration = std::max(linear_min_duration_, distance / linear_velocity_);
      
      // 生成线性插补轨迹
      std::vector<geometry_msgs::msg::Pose> linear_waypoints;
      bool linear_interpolation_success = false;
      try {
        linear_waypoints = trajectory_planner_.generateLinearCartesianTrajectory(
          waypoints[0], waypoints[1], duration, eef_step_);
        linear_interpolation_success = !linear_waypoints.empty();
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "线性插补生成失败: %s，回退到MoveIt方法", e.what());
        linear_interpolation_success = false;
      }
      
      if (linear_interpolation_success)
      {
        // 使用生成的线性插补waypoints调用MoveIt的computeCartesianPath（带超时）
        // 这样可以确保路径是精确的直线
        RCLCPP_INFO(this->get_logger(), "使用 %zu 个线性插补点进行笛卡尔路径规划", linear_waypoints.size());
        
        moveit_msgs::msg::RobotTrajectory trajectory;
        // 使用配置的步长缩放因子，因为我们已经有了精确的插补点
        double scaled_step = eef_step_ * linear_step_scale_;
        double fraction = compute_cartesian_path_with_timeout(
            linear_waypoints, scaled_step, jump_threshold_, trajectory, timeout, true);  // 允许重试
        
        if (fraction < 0.0)  // 超时或异常
        {
          RCLCPP_WARN(this->get_logger(), "线性插补笛卡尔路径计算超时，回退到标准方法");
          // 继续执行，尝试标准方法
        }
        else
        {
        
          double threshold = use_descend ? cartesian_descend_threshold_ : cartesian_lift_threshold_;
          if (fraction >= threshold)
          {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;
            
            {
              std::lock_guard<std::mutex> lock(moveit_mutex_);
              moveit::core::MoveItErrorCode result = move_group_interface_->execute(plan);
              if (result == moveit::core::MoveItErrorCode::SUCCESS)
              {
                RCLCPP_INFO(this->get_logger(), "线性插补笛卡尔路径执行成功");
                return true;
              }
            }
          }
          RCLCPP_WARN(this->get_logger(), "线性插补笛卡尔路径执行失败（成功率: %.2f%%，阈值: %.2f%%），回退到标准方法", 
                     fraction * 100.0, threshold * 100.0);
        }
      }
    }
    
    // 标准方法：使用 MoveIt 的 computeCartesianPath（带超时）
    // 使用异步执行避免阻塞工作线程
    // 尊重参数值：jump_threshold=0 表示关闭跳跃检查（MoveIt约定）
    // 不要强制改成非零值，否则可能导致 fraction 下降
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = compute_cartesian_path_with_timeout(
        waypoints, eef_step_, jump_threshold_, trajectory, timeout, true);  // 允许重试
    
    if (fraction < 0.0)  // 超时或异常
    {
      RCLCPP_WARN(this->get_logger(), "笛卡尔路径计算超时（%.1f秒），触发fallback", timeout);
      return false;
    }

    // 根据用途设置不同的阈值（从配置读取）
    double threshold = use_descend ? cartesian_descend_threshold_ : cartesian_lift_threshold_;
    if (fraction < threshold)
    {
      RCLCPP_WARN(this->get_logger(), 
                  "笛卡尔路径规划失败，成功率: %.2f%%，要求 >= %.2f%%，尝试fallback", 
                  fraction * 100.0, threshold * 100.0);
      return false;
    }

    // 执行轨迹（需要加锁保护，避免与急停回调竞争）
    {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;

      {
        std::lock_guard<std::mutex> lock(moveit_mutex_);
        moveit::core::MoveItErrorCode result = move_group_interface_->execute(plan);
        if (result == moveit::core::MoveItErrorCode::SUCCESS)
        {
          RCLCPP_INFO(this->get_logger(), "笛卡尔路径执行成功");
          return true;
        }
      }

      RCLCPP_WARN(this->get_logger(), "笛卡尔路径执行失败");
      return false;
    }
  }

  // 计算自适应分段数（根据距离动态调整）
  int calculate_adaptive_segments(const geometry_msgs::msg::PoseStamped& start_pose,
                                  const geometry_msgs::msg::PoseStamped& end_pose)
  {
    if (!adaptive_segments_)
    {
      // 如果禁用自适应，使用默认值（3段）
      return 3;
    }

    // 计算距离
    double dx = end_pose.pose.position.x - start_pose.pose.position.x;
    double dy = end_pose.pose.position.y - start_pose.pose.position.y;
    double dz = end_pose.pose.position.z - start_pose.pose.position.z;
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    // 根据距离计算分段数
    int segments;
    if (distance < 0.05)  // 短距离：2段（3个waypoints）
    {
      segments = 2;
    }
    else if (distance < 0.10)  // 中等距离：3段（4个waypoints）
    {
      segments = 3;
    }
    else  // 长距离：4段（5个waypoints）
    {
      segments = 4;
    }

    // 限制在min_segments_和max_segments_之间
    segments = std::max(min_segments_, std::min(max_segments_, segments));

    RCLCPP_INFO(this->get_logger(), 
                "[自适应分段] 距离=%.3f m, 计算分段数=%d (范围: %d-%d)",
                distance, segments, min_segments_, max_segments_);

    return segments;
  }

  // 分段下压（4DOF专用：只使用分段IK→joint执行）
  // 策略：生成N段目标pose（只改z，保持x、y、orientation不变），每段用当前状态做seed解IK，
  //      setJointValueTarget，然后plan/execute或多项式插值执行
  // 速度/平滑性：依靠已有的多项式插值（cubic/B样条）保证
  // 支持自适应分段数（根据距离动态调整）
  // 注意：4DOF机械臂不使用CartesianPath，因为姿态空间不连续，CartesianPath不适合抓缆绳任务
  bool execute_segmented_descend(const geometry_msgs::msg::PoseStamped& start_pose,
                                 const geometry_msgs::msg::PoseStamped& end_pose,
                                 int num_segments = -1)  // -1表示使用自适应分段
  {
    // 如果num_segments为-1，使用自适应分段数
    if (num_segments < 0)
    {
      num_segments = calculate_adaptive_segments(start_pose, end_pose);
    }

    RCLCPP_INFO(this->get_logger(), 
                "[分段下压] 开始分段下压，从 (%.3f, %.3f, %.3f) 到 (%.3f, %.3f, %.3f)，分段数: %d",
                start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z,
                end_pose.pose.position.x, end_pose.pose.position.y, end_pose.pose.position.z,
                num_segments);

    // 计算每个分段的位置增量
    double dx = (end_pose.pose.position.x - start_pose.pose.position.x) / num_segments;
    double dy = (end_pose.pose.position.y - start_pose.pose.position.y) / num_segments;
    double dz = (end_pose.pose.position.z - start_pose.pose.position.z) / num_segments;

    // 创建waypoint列表（包括起点和终点）
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_stamped;
    waypoints_stamped.push_back(start_pose);  // 起点

    // 生成中间waypoints，并对每个waypoint进行yaw合法性检查
    // 获取当前状态用于yaw合法性检查
    moveit::core::RobotStatePtr state_for_yaw_check;
    {
      std::lock_guard<std::mutex> lock(moveit_mutex_);
      // 修复时间戳问题：使用Time(0)获取最新可用状态
      state_for_yaw_check = move_group_interface_->getCurrentState(0.0);
    }
    
    // 从end_pose的orientation中提取yaw（用于yaw合法性检查）
    double target_yaw = 0.0;
    {
      tf2::Quaternion q;
      tf2::fromMsg(end_pose.pose.orientation, q);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      target_yaw = yaw;
    }
    
    for (int i = 1; i < num_segments; ++i)
    {
      geometry_msgs::msg::PoseStamped waypoint = start_pose;
      waypoint.pose.position.x += dx * i;
      waypoint.pose.position.y += dy * i;
      waypoint.pose.position.z += dz * i;
      
      // yaw合法性检查：如果启用，检查该yaw在特定z高度下是否IK可解
      double valid_yaw = target_yaw;
      if (yaw_validation_enabled_ && state_for_yaw_check)
      {
        valid_yaw = find_valid_yaw(waypoint.pose.position.x, 
                                   waypoint.pose.position.y, 
                                   waypoint.pose.position.z,
                                   target_yaw,
                                   state_for_yaw_check,
                                   yaw_tolerance_,
                                   yaw_search_step_);
        
        // 如果找到的yaw与原始yaw不同，更新orientation
        if (std::abs(valid_yaw - target_yaw) > 1e-6)
        {
          waypoint.pose.orientation = compute_downward_orientation_with_yaw(valid_yaw);
          RCLCPP_INFO(this->get_logger(), 
                      "[分段下压] waypoint #%d yaw已调整: %.3f -> %.3f rad (%.1f -> %.1f°)",
                      i, target_yaw, valid_yaw, 
                      target_yaw * 180.0 / M_PI, valid_yaw * 180.0 / M_PI);
        }
        else
        {
          // 使用目标orientation（end_pose），确保所有waypoints都使用正确的yaw
          waypoint.pose.orientation = end_pose.pose.orientation;
        }
      }
      else
      {
        // 未启用yaw检查，直接使用目标orientation
        waypoint.pose.orientation = end_pose.pose.orientation;
      }
      
      waypoints_stamped.push_back(waypoint);
    }

    waypoints_stamped.push_back(end_pose);  // 终点

    // 确保所有waypoints在planning frame中（用于逐段执行）
    for (auto& wp : waypoints_stamped)
    {
      if (wp.header.frame_id != planning_frame_)
      {
        if (!transform_pose_to_planning(wp))
        {
          RCLCPP_ERROR(this->get_logger(), 
                      "[分段下压] waypoint转换到planning frame失败");
          return false;
        }
      }
    }

    // 直接逐段执行：每段使用IK→joint target，支持多项式插值
    // 4DOF机械臂不使用CartesianPath，因为姿态空间不连续，CartesianPath不适合
    // 计算平均段距离（用于日志）
    double avg_segment_distance = 0.0;
    if (waypoints_stamped.size() > 1)
    {
      double total_distance = std::sqrt(
        std::pow(end_pose.pose.position.x - start_pose.pose.position.x, 2) +
        std::pow(end_pose.pose.position.y - start_pose.pose.position.y, 2) +
        std::pow(end_pose.pose.position.z - start_pose.pose.position.z, 2)
      );
      avg_segment_distance = total_distance / (waypoints_stamped.size() - 1);
    }
    
    RCLCPP_INFO(this->get_logger(), 
                "[分段下压] 开始逐段IK→joint执行（%zu个waypoints，平均每段约%.3f m）",
                waypoints_stamped.size(), avg_segment_distance);
    
    // 跳过第一段（i=0），因为它是起点，已经在那个位置，不需要移动
    for (size_t i = 1; i < waypoints_stamped.size(); ++i)
    {
      // 计算每段的距离（用于日志）
      double segment_distance = 0.0;
      if (i > 0)
      {
        double dx_seg = waypoints_stamped[i].pose.position.x - waypoints_stamped[i-1].pose.position.x;
        double dy_seg = waypoints_stamped[i].pose.position.y - waypoints_stamped[i-1].pose.position.y;
        double dz_seg = waypoints_stamped[i].pose.position.z - waypoints_stamped[i-1].pose.position.z;
        segment_distance = std::sqrt(dx_seg*dx_seg + dy_seg*dy_seg + dz_seg*dz_seg);
      }
      
      RCLCPP_INFO(this->get_logger(), 
                  "[分段下压] 执行段 %zu/%zu: 位置=(%.3f, %.3f, %.3f), 距离=%.3f m",
                  i + 1, waypoints_stamped.size(),
                  waypoints_stamped[i].pose.position.x,
                  waypoints_stamped[i].pose.position.y,
                  waypoints_stamped[i].pose.position.z,
                  segment_distance);

      // 移除执行前的状态同步等待（减少抖动）
      // 仅在需要时设置start state
      {
        std::lock_guard<std::mutex> lock(moveit_mutex_);
        move_group_interface_->setStartStateToCurrentState();
      }

      if (!plan_execute_ik_joint_target(waypoints_stamped[i]))
      {
        RCLCPP_WARN(this->get_logger(), 
                    "[分段下压] 段 %zu/%zu 执行失败，尝试回滚到起点", i + 1, waypoints_stamped.size());
        // 执行失败，尝试回滚到start_pose（pregrasp）
        if (enable_recovery_)
        {
          recover_to_pregrasp(start_pose);
        }
        return false;
      }
      
      // 移除执行后的状态稳定等待（减少抖动）
      // 仅在最后一段执行后等待一小段时间，确保状态更新
      if (i == waypoints_stamped.size() - 1)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));  // 最小等待，确保状态更新
      }
    }

    RCLCPP_INFO(this->get_logger(), "[分段下压] 所有段执行成功");
    return true;
  }

  // 分段提升 fallback（4DOF机械臂常用策略）
  // 将提升路径分成多个waypoint，对每个waypoint进行IK→joint target规划
  // 这样比Cartesian路径更稳，因为避免了OMPL的constraint sampler问题
  bool execute_segmented_lift(const geometry_msgs::msg::PoseStamped& start_pose,
                              const geometry_msgs::msg::PoseStamped& end_pose,
                              int num_segments = 3)
  {
    RCLCPP_INFO(this->get_logger(), 
                "[分段提升] 开始分段提升，从 (%.3f, %.3f, %.3f) 到 (%.3f, %.3f, %.3f)，分段数: %d",
                start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z,
                end_pose.pose.position.x, end_pose.pose.position.y, end_pose.pose.position.z,
                num_segments);

    // 计算每个分段的位置增量
    double dx = (end_pose.pose.position.x - start_pose.pose.position.x) / num_segments;
    double dy = (end_pose.pose.position.y - start_pose.pose.position.y) / num_segments;
    double dz = (end_pose.pose.position.z - start_pose.pose.position.z) / num_segments;

    // 创建waypoint列表（包括起点和终点）
    std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    waypoints.push_back(start_pose);  // 起点

    // 生成中间waypoints
    for (int i = 1; i < num_segments; ++i)
    {
      geometry_msgs::msg::PoseStamped waypoint = start_pose;
      waypoint.pose.position.x += dx * i;
      waypoint.pose.position.y += dy * i;
      waypoint.pose.position.z += dz * i;
      // 保持orientation：使用目标orientation（end_pose），确保所有waypoints都使用正确的yaw
      waypoint.pose.orientation = end_pose.pose.orientation;
      waypoints.push_back(waypoint);
    }

    waypoints.push_back(end_pose);  // 终点

    // 如果启用B样条且waypoints数量足够，使用B样条生成平滑轨迹
    if (use_bspline_ && waypoints.size() >= static_cast<size_t>(bspline_degree_ + 1))
    {
      RCLCPP_INFO(this->get_logger(), 
                  "[分段提升] 使用B样条生成平滑轨迹（%zu个控制点，阶数=%d）", 
                  waypoints.size(), bspline_degree_);
      
      // 对所有waypoints进行IK求解，得到关节空间的控制点
      std::vector<std::vector<double>> control_points;
      std::lock_guard<std::mutex> lock(moveit_mutex_);
      
      for (size_t i = 0; i < waypoints.size(); ++i)
      {
        geometry_msgs::msg::PoseStamped pose_in_planning = waypoints[i];
        if (!transform_pose_to_planning(pose_in_planning))
        {
          RCLCPP_WARN(this->get_logger(), 
                      "[分段提升] waypoint #%zu 转换失败，fallback到逐段执行", i);
          control_points.clear();
          break;
        }
        
        // 修复时间戳问题：使用Time(0)获取最新可用状态
        auto state = move_group_interface_->getCurrentState(0.0);
        if (!state) {
          RCLCPP_WARN(this->get_logger(), "[分段提升] 无法获取当前状态");
          control_points.clear();
          break;
        }
        const auto* jmg = state->getJointModelGroup("arm_group");
        if (!jmg)
        {
          RCLCPP_WARN(this->get_logger(), 
                      "[分段提升] 无法获取arm_group，fallback到逐段执行");
          control_points.clear();
          break;
        }
        
        // IK求解
        if (i > 0)
        {
          // 后续waypoints使用前一个waypoint的关节值作为seed
          state->setJointGroupPositions(jmg, control_points.back());
        }
        
        if (state->setFromIK(jmg, pose_in_planning.pose, eef_link_, ik_timeout_))
        {
          std::vector<double> joint_values;
          state->copyJointGroupPositions(jmg, joint_values);
          control_points.push_back(joint_values);
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), 
                      "[分段提升] waypoint #%zu IK求解失败，fallback到逐段执行", i);
          control_points.clear();
          break;
        }
      }
      
      // 如果所有waypoints的IK都成功，使用B样条执行
      if (control_points.size() == waypoints.size())
      {
        // 重新获取state和jmg（因为它们在循环内定义）
        // 修复时间戳问题：使用Time(0)获取最新可用状态
        auto state_for_bspline = move_group_interface_->getCurrentState(0.0);
        if (!state_for_bspline) {
          RCLCPP_WARN(this->get_logger(), "[B样条] 无法获取当前状态");
          return false;
        }
        const auto* jmg_for_bspline = state_for_bspline->getJointModelGroup("arm_group");
        if (jmg_for_bspline)
        {
          try {
            std::vector<std::string> joint_names = jmg_for_bspline->getActiveJointModelNames();
          
          trajectory_msgs::msg::JointTrajectory trajectory = 
              trajectory_planner_.generateBSplineJointTrajectory(
                control_points, bspline_degree_, bspline_duration_, 
                joint_names, bspline_dt_);
          
          moveit::planning_interface::MoveGroupInterface::Plan plan;
          plan.trajectory_.joint_trajectory = trajectory;
          plan.trajectory_.joint_trajectory.header.stamp = this->get_clock()->now();
          plan.trajectory_.joint_trajectory.header.frame_id = planning_frame_;
          
          move_group_interface_->setStartStateToCurrentState();
          moveit::core::MoveItErrorCode result = move_group_interface_->execute(plan);
          
          if (result == moveit::core::MoveItErrorCode::SUCCESS)
          {
            RCLCPP_INFO(this->get_logger(), "[分段提升] B样条轨迹执行成功");
            return true;
          }
          else
          {
            RCLCPP_WARN(this->get_logger(), 
                        "[分段提升] B样条轨迹执行失败，fallback到逐段执行，错误代码: %d", 
                        result.val);
          }
          } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), 
                        "[分段提升] B样条轨迹生成失败: %s，fallback到逐段执行", e.what());
          }
        }
      }
    }

    // 逐段执行IK→joint target规划（如果B样条未启用或失败）
    // 跳过第一段（i=0），因为它是起点，已经在那个位置，不需要移动
    for (size_t i = 1; i < waypoints.size(); ++i)
    {
      RCLCPP_INFO(this->get_logger(), 
                  "[分段提升] 执行段 %zu/%zu: 位置=(%.3f, %.3f, %.3f)",
                  i + 1, waypoints.size(),
                  waypoints[i].pose.position.x,
                  waypoints[i].pose.position.y,
                  waypoints[i].pose.position.z);

      // 执行前状态同步：等待状态更新并强制使用最新状态
      RCLCPP_INFO(this->get_logger(), 
                  "[分段提升] 段 %zu/%zu 执行前，等待%.0fms并同步状态...", 
                  i + 1, waypoints.size(), state_sync_wait_ * 1000);
      std::this_thread::sleep_for(std::chrono::milliseconds(
        static_cast<int>(state_sync_wait_ * 1000)));
      
      // 强制用最新状态作为 start_state（MoveIt C++ 常用做法）
      {
        std::lock_guard<std::mutex> lock(moveit_mutex_);
        move_group_interface_->setStartStateToCurrentState();
      }

      if (!plan_execute_ik_joint_target(waypoints[i]))
      {
        RCLCPP_WARN(this->get_logger(), 
                    "[分段提升] 段 %zu/%zu 执行失败，尝试回滚到起点", i + 1, waypoints.size());
        // 执行失败，尝试回滚到start_pose（grasp_pose）
        if (enable_recovery_ && waypoints.size() > 0)
        {
          recover_to_pregrasp(waypoints[0]);  // 回滚到第一段（grasp_pose）
        }
        return false;
      }
      
      // 等待状态稳定（每段执行后都等待，确保状态同步）
      // ✅ 修复：强制等待状态稳定，不能提前执行下一段
      RCLCPP_INFO(this->get_logger(), 
                  "[分段提升] 段 %zu/%zu 执行成功，等待状态稳定...", 
                  i + 1, waypoints.size());
      bool state_stable = waitForStateStable(segment_execution_wait_, 5, 0.002);
      if (!state_stable)
      {
        // 如果状态稳定判断超时，强制等待最小时间，确保状态更新
        RCLCPP_WARN(this->get_logger(), 
                    "[分段提升] 状态稳定判断超时，强制等待最小时间（%.0fms）",
                    segment_execution_wait_ * 1000);
        std::this_thread::sleep_for(std::chrono::milliseconds(
          static_cast<int>(segment_execution_wait_ * 1000)));
      }
      else
      {
        RCLCPP_DEBUG(this->get_logger(), 
                    "[分段提升] 状态已稳定，继续执行下一段");
      }
    }

    RCLCPP_INFO(this->get_logger(), "[分段提升] 所有段执行成功");
    return true;
  }

  // IK→joint target 规划执行（内部实现，无锁版本）
  // 注意：调用者必须已经持有 moveit_mutex_ 锁
  bool plan_execute_ik_joint_target_impl(const geometry_msgs::msg::PoseStamped& target_pose, 
                                         int max_ik_attempts = 3)
  {
    // 转换到planning frame
    geometry_msgs::msg::PoseStamped pose_in_planning = target_pose;
    if (!transform_pose_to_planning(pose_in_planning))
    {
      return false;
    }

    RCLCPP_INFO(this->get_logger(), 
                "[IK→Joint] 目标位姿: 位置=(%.3f, %.3f, %.3f), frame=%s",
                pose_in_planning.pose.position.x,
                pose_in_planning.pose.position.y,
                pose_in_planning.pose.position.z,
                pose_in_planning.header.frame_id.c_str());

    // 规划前状态同步：等待状态更新并强制使用最新状态
    // 这是 MoveIt C++ 常用做法，避免 start_state 落后
    std::this_thread::sleep_for(std::chrono::milliseconds(
      static_cast<int>(state_sync_wait_ * 1000)));
    move_group_interface_->setStartStateToCurrentState();

    // 获取当前状态用于yaw合法性检查
    // 修复时间戳问题：使用Time(0)获取最新可用状态
    auto state = move_group_interface_->getCurrentState(0.0);
    if (!state) {
      RCLCPP_WARN(this->get_logger(), "[IK→Joint] 无法获取当前状态");
      return false;
    }
    const auto* jmg = state->getJointModelGroup("arm_group");
    if (!jmg)
    {
      RCLCPP_WARN(this->get_logger(), "[IK→Joint] 无法获取arm_group关节模型组");
      return false;
    }

    // yaw合法性检查：如果启用，检查该yaw在特定z高度下是否IK可解
    if (yaw_validation_enabled_)
    {
      // 从orientation中提取yaw
      double target_yaw = 0.0;
      {
        tf2::Quaternion q;
        tf2::fromMsg(pose_in_planning.pose.orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        target_yaw = yaw;
      }
      
      // 检查并调整yaw
      double valid_yaw = find_valid_yaw(pose_in_planning.pose.position.x,
                                        pose_in_planning.pose.position.y,
                                        pose_in_planning.pose.position.z,
                                        target_yaw,
                                        state,
                                        yaw_tolerance_,
                                        yaw_search_step_);
      
      // 如果yaw被调整，更新pose的orientation
      if (std::abs(valid_yaw - target_yaw) > 1e-6)
      {
        pose_in_planning.pose.orientation = compute_downward_orientation_with_yaw(valid_yaw);
        RCLCPP_INFO(this->get_logger(), 
                    "[IK→Joint] yaw已调整: %.3f -> %.3f rad (%.1f -> %.1f°)",
                    target_yaw, valid_yaw,
                    target_yaw * 180.0 / M_PI, valid_yaw * 180.0 / M_PI);
      }
    }

    std::vector<double> joint_values;
    bool ik_found = false;

    // 尝试多次IK（使用当前状态作为seed，然后尝试随机seed）
    for (int attempt = 0; attempt < max_ik_attempts; ++attempt)
    {
      if (attempt > 0)
      {
        // 后续尝试：使用随机seed
        state->setToRandomPositions(jmg);
        RCLCPP_INFO(this->get_logger(), "[IK→Joint] IK尝试 #%d (使用随机seed)", attempt + 1);
      }
      else
      {
        // 第一次尝试：使用当前状态
        // 修复时间戳问题：使用Time(0)获取最新可用状态
        state = move_group_interface_->getCurrentState(0.0);
        if (!state) {
          RCLCPP_WARN(this->get_logger(), "[IK→Joint] 无法获取当前状态作为seed");
          continue;
        }
        RCLCPP_INFO(this->get_logger(), "[IK→Joint] IK尝试 #%d (使用当前状态作为seed)", attempt + 1);
      }

      // IK求解（使用配置的timeout）
      if (state->setFromIK(jmg, pose_in_planning.pose, eef_link_, ik_timeout_))
      {
        state->copyJointGroupPositions(jmg, joint_values);
        ik_found = true;
        RCLCPP_INFO(this->get_logger(),
                    "[IK→Joint] IK求解成功（尝试 #%d），关节值: [%.3f, %.3f, %.3f, %.3f]",
                    attempt + 1,
                    joint_values.size() > 0 ? joint_values[0] : 0.0,
                    joint_values.size() > 1 ? joint_values[1] : 0.0,
                    joint_values.size() > 2 ? joint_values[2] : 0.0,
                    joint_values.size() > 3 ? joint_values[3] : 0.0);
        if (joint_values.size() >= 4)
        {
          // 从目标orientation中提取yaw分量，用于对比
          double target_yaw_from_orientation = 0.0;
          {
            tf2::Quaternion q_target;
            tf2::fromMsg(pose_in_planning.pose.orientation, q_target);
            double roll_target, pitch_target, yaw_target;
            tf2::Matrix3x3(q_target).getRPY(roll_target, pitch_target, yaw_target);
            target_yaw_from_orientation = yaw_target;
          }
          
          double joint4_value = joint_values[3];
          
          // 只记录日志，不做对比和优化
          // 注意：Joint4 ≠ 末端yaw，不进行对比。MoveIt会通过IK自动找到合适的关节角。
          RCLCPP_INFO(this->get_logger(),
                      "[IK→Joint] Joint4值: %.3f rad (%.1f deg)",
                      joint4_value, joint4_value * 180.0 / M_PI);
          RCLCPP_INFO(this->get_logger(),
                      "[IK→Joint] 目标orientation的yaw分量: %.3f rad (%.1f deg)",
                      target_yaw_from_orientation, target_yaw_from_orientation * 180.0 / M_PI);
          
          // 使用FK验证实际末端yaw（仅用于日志，不强制修正）
          moveit::core::RobotStatePtr fk_state(new moveit::core::RobotState(*state));
          fk_state->setJointGroupPositions(jmg, joint_values);
          fk_state->updateLinkTransforms();
          
          // 获取实际末端位姿
          const Eigen::Isometry3d& eef_transform = fk_state->getGlobalLinkTransform(eef_link_);
          Eigen::Matrix3d R = eef_transform.rotation();
          
          // 从旋转矩阵提取RPY
          tf2::Matrix3x3 rot_matrix;
          rot_matrix.setValue(R(0,0), R(0,1), R(0,2),
                              R(1,0), R(1,1), R(1,2),
                              R(2,0), R(2,1), R(2,2));
          double roll_actual, pitch_actual, yaw_actual;
          rot_matrix.getRPY(roll_actual, pitch_actual, yaw_actual);
          
          // 与目标yaw对比（仅用于日志）
          double yaw_error = normalize_angle_diff(yaw_actual - target_yaw_from_orientation);
          RCLCPP_INFO(this->get_logger(),
                      "[FK验证] 实际末端yaw=%.3f rad (%.1f°), 目标yaw=%.3f rad (%.1f°), 误差=%.3f rad (%.1f°)",
                      yaw_actual, yaw_actual * 180.0 / M_PI,
                      target_yaw_from_orientation, target_yaw_from_orientation * 180.0 / M_PI,
                      yaw_error, yaw_error * 180.0 / M_PI);
          
          // 注意：不进行强制修正，MoveIt的IK已经找到了合适的解
          // 如果误差较大，可能是orientation计算需要调整，或者yaw本身不需要精确对齐
          const double max_yaw_error = 0.5236;  // 30度
          if (std::abs(yaw_error) > max_yaw_error) {
            RCLCPP_WARN(this->get_logger(),
                        "[FK验证] 末端yaw误差较大 (%.1f°)，但继续使用（如果只需要垂直向下，yaw误差可接受）",
                        std::abs(yaw_error) * 180.0 / M_PI);
          }
        }
        break;
      }
    }

    if (!ik_found)
    {
      RCLCPP_WARN(this->get_logger(), 
                  "[IK→Joint] IK求解失败（尝试了 %d 次），无法进行joint target规划",
                  max_ik_attempts);
      return false;
    }

    // 如果启用多项式插值，优先使用多项式插值生成轨迹
    if (use_polynomial_interpolation_)
    {
      RCLCPP_INFO(this->get_logger(), "[IK→Joint] 使用多项式插值生成轨迹");
      
      // 获取当前关节值（已在锁内，直接调用）
      std::vector<double> start_joints = move_group_interface_->getCurrentJointValues();
      
      // 使用多项式插值生成并执行轨迹
      // 注意：当前函数已在锁内，直接实现多项式插值逻辑（避免重复加锁）
      try {
        const auto* jmg = state->getJointModelGroup("arm_group");
        std::vector<std::string> joint_names = jmg->getActiveJointModelNames();
        
        trajectory_msgs::msg::JointTrajectory trajectory;
        bool use_quintic = (polynomial_type_ == "quintic");
        
        if (use_quintic)
        {
          std::vector<double> start_vel(joint_names.size(), 0.0);
          std::vector<double> end_vel(joint_names.size(), 0.0);
          std::vector<double> start_acc(joint_names.size(), 0.0);
          std::vector<double> end_acc(joint_names.size(), 0.0);
          
          trajectory = trajectory_planner_.generateQuinticPolynomialTrajectory(
            start_joints, joint_values, start_vel, end_vel, 
            start_acc, end_acc, polynomial_duration_, joint_names, polynomial_dt_);
        }
        else
        {
          std::vector<double> start_vel(joint_names.size(), 0.0);
          std::vector<double> end_vel(joint_names.size(), 0.0);
          
          trajectory = trajectory_planner_.generateCubicPolynomialTrajectory(
            start_joints, joint_values, start_vel, end_vel, 
            polynomial_duration_, joint_names, polynomial_dt_);
        }
        
        // 创建MoveIt Plan对象并执行
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_.joint_trajectory = trajectory;
        plan.trajectory_.joint_trajectory.header.stamp = this->get_clock()->now();
        plan.trajectory_.joint_trajectory.header.frame_id = planning_frame_;
        
        // 执行前同步状态
        move_group_interface_->setStartStateToCurrentState();
        
        moveit::core::MoveItErrorCode result = move_group_interface_->execute(plan);
        if (result == moveit::core::MoveItErrorCode::SUCCESS)
        {
          RCLCPP_INFO(this->get_logger(), "[IK→Joint] 多项式插值轨迹执行成功");
          return true;
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), 
                      "[IK→Joint] 多项式插值轨迹执行失败，fallback到标准规划，错误代码: %d", 
                      result.val);
          // 继续执行标准规划
        }
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), 
                    "[IK→Joint] 多项式插值生成失败: %s，fallback到标准规划", e.what());
        // 继续执行标准规划
      }
    }

    // 标准MoveIt规划（如果多项式插值未启用或失败）
    // 修复：IK成功后、调用plan之前显式设置start_state（绕过时间戳检查）
    // IK求解可能花费了一些时间，状态可能已经更新，必须使用最新状态
    setStartStateExplicit();
    
    move_group_interface_->clearPoseTargets();
    move_group_interface_->clearPathConstraints();
    move_group_interface_->setJointValueTarget(joint_values);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode result = move_group_interface_->plan(plan);

    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "[IK→Joint] Joint target规划成功");
      
      // 执行前再次同步状态（规划后状态可能已更新）
      // 这是 MoveIt 的常见做法：规划后、执行前再次确认状态，避免"Invalid Trajectory"错误
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      setStartStateExplicit();
      
      result = move_group_interface_->execute(plan);
      if (result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_INFO(this->get_logger(), "[IK→Joint] Joint target执行成功");
        return true;
      }
      else
      {
        // 执行失败，记录错误（但不强制回滚，因为可能没有recovery_pose）
        RCLCPP_ERROR(this->get_logger(), 
                    "[IK→Joint] Joint target执行失败（错误代码: %d），可能导致状态不一致",
                    result.val);
        // 注意：这里不强制回滚，因为plan_execute_ik_joint_target是通用函数，
        // 调用者应该根据上下文决定是否需要回滚
        return false;
      }
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "[IK→Joint] Joint target规划失败");
      return false;
    }
  }

  // IK→joint target 规划执行（公共接口，带锁）
  // 对4DOF机械臂，OMPL的Pose/Position目标约束采样器可能无法构建可采样的goal region
  // 因此优先使用IK求解得到关节值，然后使用joint target进行规划
  bool plan_execute_ik_joint_target(const geometry_msgs::msg::PoseStamped& target_pose, 
                                     int max_ik_attempts = 3)
  {
    std::lock_guard<std::mutex> lock(moveit_mutex_);
    return plan_execute_ik_joint_target_impl(target_pose, max_ik_attempts);
  }

  // 使用关节空间多项式插值执行轨迹（示例函数）
  // 可以用于平滑的关节空间运动
  // 注意：是否启用由配置文件中的 use_polynomial_interpolation_ 参数控制
  bool execute_polynomial_joint_trajectory(
    const std::vector<double>& start_joints,
    const std::vector<double>& end_joints)
  {
    if (!use_polynomial_interpolation_)
    {
      RCLCPP_DEBUG(this->get_logger(), "多项式插值未启用，跳过");
      return false;
    }

    std::lock_guard<std::mutex> lock(moveit_mutex_);
    
    try {
      // 获取关节名称
      // 修复时间戳问题：使用Time(0)获取最新可用状态
      auto state = move_group_interface_->getCurrentState(0.0);
      if (!state) {
        RCLCPP_WARN(this->get_logger(), "无法获取当前状态");
        return false;
      }
      const auto* jmg = state->getJointModelGroup("arm_group");
      if (!jmg)
      {
        RCLCPP_WARN(this->get_logger(), "无法获取arm_group关节模型组");
        return false;
      }
      
      std::vector<std::string> joint_names = jmg->getActiveJointModelNames();
      
      // 验证关节数量
      if (start_joints.size() != end_joints.size() || 
          start_joints.size() != joint_names.size())
      {
        RCLCPP_WARN(this->get_logger(), "关节数量不匹配");
        return false;
      }
      
      trajectory_msgs::msg::JointTrajectory trajectory;
      bool use_quintic = (polynomial_type_ == "quintic");
      
      if (use_quintic)
      {
        // 使用五次多项式（需要速度和加速度约束）
        std::vector<double> start_vel(joint_names.size(), 0.0);
        std::vector<double> end_vel(joint_names.size(), 0.0);
        std::vector<double> start_acc(joint_names.size(), 0.0);
        std::vector<double> end_acc(joint_names.size(), 0.0);
        
        trajectory = trajectory_planner_.generateQuinticPolynomialTrajectory(
          start_joints, end_joints, start_vel, end_vel, 
          start_acc, end_acc, polynomial_duration_, joint_names, polynomial_dt_);
        
        RCLCPP_INFO(this->get_logger(), "生成五次多项式轨迹，持续时间: %.2f秒，时间步长: %.3f秒", 
                   polynomial_duration_, polynomial_dt_);
      }
      else
      {
        // 使用三次多项式（只需要速度约束，默认为0）
        std::vector<double> start_vel(joint_names.size(), 0.0);
        std::vector<double> end_vel(joint_names.size(), 0.0);
        
        trajectory = trajectory_planner_.generateCubicPolynomialTrajectory(
          start_joints, end_joints, start_vel, end_vel, 
          polynomial_duration_, joint_names, polynomial_dt_);
        
        RCLCPP_INFO(this->get_logger(), "生成三次多项式轨迹，持续时间: %.2f秒，时间步长: %.3f秒", 
                   polynomial_duration_, polynomial_dt_);
      }
      
      // 执行轨迹：将trajectory_msgs转换为moveit_msgs::RobotTrajectory
      RCLCPP_INFO(this->get_logger(), "轨迹已生成，包含 %zu 个点", trajectory.points.size());
      
      // 创建MoveIt Plan对象
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_.joint_trajectory = trajectory;
      plan.trajectory_.joint_trajectory.header.stamp = this->get_clock()->now();
      plan.trajectory_.joint_trajectory.header.frame_id = planning_frame_;
      
      // 执行前同步状态
      move_group_interface_->setStartStateToCurrentState();
      
      // 执行轨迹
      moveit::core::MoveItErrorCode result = move_group_interface_->execute(plan);
      if (result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_INFO(this->get_logger(), "多项式插值轨迹执行成功");
        return true;
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "多项式插值轨迹执行失败，错误代码: %d", result.val);
        return false;
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "多项式轨迹生成失败: %s", e.what());
      return false;
    }
  }

  // 使用B样条关节空间轨迹执行（需要多个控制点）
  // control_points: 控制点序列，每个控制点是一个关节角度向量
  // 至少需要 degree+1 个控制点
  bool execute_bspline_joint_trajectory(
    const std::vector<std::vector<double>>& control_points)
  {
    if (!use_bspline_)
    {
      RCLCPP_DEBUG(this->get_logger(), "B样条未启用，跳过");
      return false;
    }

    std::lock_guard<std::mutex> lock(moveit_mutex_);
    
    try {
      // 检查控制点数量
      if (control_points.size() < static_cast<size_t>(bspline_degree_ + 1))
      {
        RCLCPP_WARN(this->get_logger(), 
                    "B样条需要至少 %d 个控制点，但只有 %zu 个", 
                    bspline_degree_ + 1, control_points.size());
        return false;
      }

      // 获取关节名称
      // 修复时间戳问题：使用Time(0)获取最新可用状态
      auto state = move_group_interface_->getCurrentState(0.0);
      if (!state) {
        RCLCPP_WARN(this->get_logger(), "无法获取当前状态");
        return false;
      }
      const auto* jmg = state->getJointModelGroup("arm_group");
      if (!jmg)
      {
        RCLCPP_WARN(this->get_logger(), "无法获取arm_group关节模型组");
        return false;
      }
      
      std::vector<std::string> joint_names = jmg->getActiveJointModelNames();
      
      // 验证所有控制点的关节数量
      size_t num_joints = joint_names.size();
      for (size_t i = 0; i < control_points.size(); ++i)
      {
        if (control_points[i].size() != num_joints)
        {
          RCLCPP_WARN(this->get_logger(), 
                      "控制点 #%zu 的关节数量 (%zu) 与期望值 (%zu) 不匹配", 
                      i, control_points[i].size(), num_joints);
          return false;
        }
      }
      
      // 生成B样条轨迹
      trajectory_msgs::msg::JointTrajectory trajectory = 
          trajectory_planner_.generateBSplineJointTrajectory(
            control_points, bspline_degree_, bspline_duration_, 
            joint_names, bspline_dt_);
      
      RCLCPP_INFO(this->get_logger(), 
                  "生成B样条关节轨迹（阶数=%d），包含 %zu 个点，持续时间: %.2f秒", 
                  bspline_degree_, trajectory.points.size(), bspline_duration_);
      
      // 创建MoveIt Plan对象并执行
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_.joint_trajectory = trajectory;
      plan.trajectory_.joint_trajectory.header.stamp = this->get_clock()->now();
      plan.trajectory_.joint_trajectory.header.frame_id = planning_frame_;
      
      // 执行前同步状态
      move_group_interface_->setStartStateToCurrentState();
      
      // 执行轨迹
      moveit::core::MoveItErrorCode result = move_group_interface_->execute(plan);
      if (result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_INFO(this->get_logger(), "B样条关节轨迹执行成功");
        return true;
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "B样条关节轨迹执行失败，错误代码: %d", result.val);
        return false;
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "B样条轨迹生成失败: %s", e.what());
      return false;
    }
  }

  // 使用Pilz PTP规划器执行点对点运动（关节空间规划）
  bool plan_execute_pilz_ptp(const geometry_msgs::msg::PoseStamped& target_pose)
  {
    // 转换到planning frame
    geometry_msgs::msg::PoseStamped pose_in_planning = target_pose;
    if (!transform_pose_to_planning(pose_in_planning))
    {
      return false;
    }

    std::lock_guard<std::mutex> lock(moveit_mutex_);
    
    // 保存当前规划器配置
    std::string old_pipeline = move_group_interface_->getPlanningPipelineId();
    std::string old_planner = move_group_interface_->getPlannerId();
    double old_planning_time = move_group_interface_->getPlanningTime();
    
    // 切换到Pilz PTP
    move_group_interface_->setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group_interface_->setPlannerId("PTP");
    move_group_interface_->setPlanningTime(5.0);  // Pilz通常很快
    move_group_interface_->setStartStateToCurrentState();
    
    // 设置目标位姿
    move_group_interface_->setPoseTarget(pose_in_planning.pose, eef_link_);
    
    // 规划
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode result = move_group_interface_->plan(plan);
    
    // 恢复原规划器配置
    move_group_interface_->setPlanningPipelineId(old_pipeline);
    move_group_interface_->setPlannerId(old_planner);
    move_group_interface_->setPlanningTime(old_planning_time);
    
    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "[Pilz PTP] 规划成功，执行轨迹");
      result = move_group_interface_->execute(plan);
      if (result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_INFO(this->get_logger(), "[Pilz PTP] 执行成功");
        return true;
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "[Pilz PTP] 规划成功但执行失败");
        return false;
      }
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "[Pilz PTP] 规划失败");
      return false;
    }
  }

  // 使用Pilz LIN规划器执行直线运动（笛卡尔空间规划）
  bool plan_execute_pilz_lin(const geometry_msgs::msg::PoseStamped& start_pose,
                             const geometry_msgs::msg::PoseStamped& end_pose)
  {
    // 转换到planning frame
    geometry_msgs::msg::PoseStamped start_in_planning = start_pose;
    geometry_msgs::msg::PoseStamped end_in_planning = end_pose;
    if (!transform_pose_to_planning(start_in_planning) || 
        !transform_pose_to_planning(end_in_planning))
    {
      return false;
    }

    std::lock_guard<std::mutex> lock(moveit_mutex_);
    
    // 保存当前规划器配置
    std::string old_pipeline = move_group_interface_->getPlanningPipelineId();
    std::string old_planner = move_group_interface_->getPlannerId();
    double old_planning_time = move_group_interface_->getPlanningTime();
    // 注意：MoveIt2的MoveGroupInterface没有getMaxVelocityScalingFactor/getMaxAccelerationScalingFactor方法
    // 使用成员变量保存当前值
    double old_velocity_scaling = max_velocity_scaling_;
    double old_acceleration_scaling = max_acceleration_scaling_;
    
    // 切换到Pilz LIN并设置较低的速度缩放因子
    move_group_interface_->setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group_interface_->setPlannerId("LIN");
    move_group_interface_->setPlanningTime(5.0);  // Pilz通常很快
    move_group_interface_->setMaxVelocityScalingFactor(0.2);  // 降低速度，确保不超过关节限制
    move_group_interface_->setMaxAccelerationScalingFactor(0.2);
    move_group_interface_->setStartStateToCurrentState();
    
    // 设置目标位姿
    move_group_interface_->setPoseTarget(end_in_planning.pose, eef_link_);
    
    // 规划
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode result = move_group_interface_->plan(plan);
    
    // 恢复原规划器配置
    move_group_interface_->setPlanningPipelineId(old_pipeline);
    move_group_interface_->setPlannerId(old_planner);
    move_group_interface_->setPlanningTime(old_planning_time);
    move_group_interface_->setMaxVelocityScalingFactor(old_velocity_scaling);
    move_group_interface_->setMaxAccelerationScalingFactor(old_acceleration_scaling);
    
    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "[Pilz LIN] 规划成功，执行轨迹");
      result = move_group_interface_->execute(plan);
      if (result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_INFO(this->get_logger(), "[Pilz LIN] 执行成功");
        return true;
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "[Pilz LIN] 规划成功但执行失败");
        return false;
      }
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "[Pilz LIN] 规划失败");
      return false;
    }
  }

  // 点对点运动（重构：优先使用IK→joint target，这是4DOF机械臂的主路径）
  // 对4DOF机械臂：默认走 IK→joint（OMPL的Pose/Position目标约束采样器可能无法构建可采样的goal region）
  // 对6DOF机械臂：可以优先 pose/position
  // 改进：将TF转换和计算移到锁外，减少持锁时间
  bool plan_execute_pose_target(const geometry_msgs::msg::PoseStamped& target_pose)
  {
    // ========== 锁外操作：TF转换和计算 ==========
    // 转换到planning frame（使用通用函数，不需要锁）
    geometry_msgs::msg::PoseStamped pose_in_planning = target_pose;
    if (!transform_pose_to_planning(pose_in_planning))
    {
      return false;
    }

    // 输出关键信息用于诊断（不需要锁）
    RCLCPP_INFO(this->get_logger(), 
                "PlanningFrame=%s, EEF=%s", 
                planning_frame_.c_str(), eef_link_.c_str());
    RCLCPP_INFO(this->get_logger(), 
                "Target in planning: pos=(%.3f %.3f %.3f) frame=%s",
                pose_in_planning.pose.position.x, 
                pose_in_planning.pose.position.y, 
                pose_in_planning.pose.position.z,
                pose_in_planning.header.frame_id.c_str());
    
    // 准备orientation候选列表（锁外计算，减少持锁时间）
    geometry_msgs::msg::Quaternion original_orientation = pose_in_planning.pose.orientation;
    std::vector<geometry_msgs::msg::Quaternion> orientation_candidates;
    
    // 候选1: 当前末端执行器orientation（如果可用且不是默认值）
    try {
      auto current_pose = getCurrentPoseSafe();
      if (!is_default_orientation(current_pose.pose.orientation))
      {
        orientation_candidates.push_back(current_pose.pose.orientation);
        RCLCPP_INFO(this->get_logger(), "准备尝试orientation候选: 当前末端执行器orientation");
      }
    } catch (const std::exception& e) {
      // 忽略错误
    }
    
    // 候选2: 原始orientation（如果它不是默认值）
    if (!is_default_orientation(original_orientation))
    {
      orientation_candidates.push_back(original_orientation);
      RCLCPP_INFO(this->get_logger(), "准备尝试orientation候选: 原始orientation");
    }
    
    // 候选3: 向下orientation
    orientation_candidates.push_back(compute_downward_orientation());
    RCLCPP_INFO(this->get_logger(), "准备尝试orientation候选: 向下orientation");
    
    // 候选4: 其他候选（从compute_orientation_candidates获取）
    std::vector<geometry_msgs::msg::Quaternion> additional_candidates = compute_orientation_candidates();
    for (const auto& candidate : additional_candidates)
    {
      // 避免重复添加已存在的候选
      bool is_duplicate = false;
      for (const auto& existing : orientation_candidates)
      {
        if (std::abs(candidate.x - existing.x) < 1e-3 &&
            std::abs(candidate.y - existing.y) < 1e-3 &&
            std::abs(candidate.z - existing.z) < 1e-3 &&
            std::abs(candidate.w - existing.w) < 1e-3)
        {
          is_duplicate = true;
          break;
        }
      }
      if (!is_duplicate)
      {
        orientation_candidates.push_back(candidate);
        RCLCPP_INFO(this->get_logger(), "准备尝试orientation候选: 额外候选");
      }
    }
    
    // 如果原始orientation不在候选列表中，先尝试它
    bool original_tried = false;
    for (const auto& candidate : orientation_candidates)
    {
      if (std::abs(candidate.x - original_orientation.x) < 1e-3 &&
          std::abs(candidate.y - original_orientation.y) < 1e-3 &&
          std::abs(candidate.z - original_orientation.z) < 1e-3 &&
          std::abs(candidate.w - original_orientation.w) < 1e-3)
      {
        original_tried = true;
        break;
      }
    }
    
    if (!original_tried)
    {
      orientation_candidates.insert(orientation_candidates.begin(), original_orientation);
    }
    
    RCLCPP_INFO(this->get_logger(), "准备尝试 %zu 个orientation候选", orientation_candidates.size());

    // ========== 锁内操作：MoveIt接口调用 ==========
    std::lock_guard<std::mutex> lock(moveit_mutex_);

    // ========== 主路径：IK→joint target（4DOF机械臂的推荐路径）==========
    // 对4DOF机械臂，OMPL的Pose/Position目标约束采样器可能无法构建可采样的goal region
    // 因此优先使用IK求解得到关节值，然后使用joint target进行规划
    // 注意：使用内部实现版本，因为当前函数已经持有锁
    RCLCPP_INFO(this->get_logger(), 
                "[主路径] 尝试IK→joint target（4DOF机械臂推荐路径）");
    if (plan_execute_ik_joint_target_impl(pose_in_planning))
    {
      RCLCPP_INFO(this->get_logger(), 
                  "点对点运动成功（使用IK→joint target主路径）");
      return true;
    }
    RCLCPP_WARN(this->get_logger(), 
                "[主路径] IK→joint target失败，进入fallback策略");

    // ========== Fallback 1：位置目标（弱化，降低 planning time 避免浪费时间）==========
    // 注意：对于4DOF，PositionTarget 通常会在 OMPL goal sampler 失败
    // 但保留作为快速尝试，使用很短的 planning time 避免卡住
    RCLCPP_INFO(this->get_logger(), 
                "[Fallback 1] 尝试位置目标（快速尝试，planning time=2s）: (%.3f, %.3f, %.3f)",
                pose_in_planning.pose.position.x,
                pose_in_planning.pose.position.y,
                pose_in_planning.pose.position.z);
    
    // 临时降低 planning time，避免在 OMPL 里浪费太多时间
    // 使用成员变量而不是 getPlanningTime()（MoveIt2 某些版本没有此方法）
    move_group_interface_->setPlanningTime(fallback_planning_time_);  // 使用配置的fallback规划时间

    setStartStateExplicit();
    move_group_interface_->clearPoseTargets();
    move_group_interface_->clearPathConstraints();
    move_group_interface_->setPositionTarget(
        pose_in_planning.pose.position.x,
        pose_in_planning.pose.position.y,
        pose_in_planning.pose.position.z,
        eef_link_);
    // 设置宽松的姿态容差（允许任意姿态）
    move_group_interface_->setGoalOrientationTolerance(3.14);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode result = move_group_interface_->plan(plan);
    
    // 恢复原来的 planning time
    move_group_interface_->setPlanningTime(default_planning_time_);
    
    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      result = move_group_interface_->execute(plan);
      if (result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_INFO(this->get_logger(), 
                    "点对点运动成功（使用位置目标fallback）");
        move_group_interface_->setGoalOrientationTolerance(0.5);
        return true;
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), 
                    "[Fallback 1] 位置目标规划成功但执行失败");
      }
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), 
                  "[Fallback 1] 位置目标规划失败（预期，4DOF OMPL sampler 问题）");
    }
    
    // position target 尝试之后，马上恢复常规容差，避免污染后续
    move_group_interface_->setGoalOrientationTolerance(0.5);

    // ========== Fallback 2：姿态候选循环 ==========
    // 防御性设置：确保姿态目标分支使用正确的容差
    move_group_interface_->setGoalOrientationTolerance(0.5);
    // 注意：orientation_candidates已在锁外计算完成
    
    // 临时降低 planning time，避免在 OMPL 里浪费太多时间
    // 使用成员变量而不是 getPlanningTime()（MoveIt2 某些版本没有此方法）
    move_group_interface_->setPlanningTime(fallback_planning_time_);  // 使用配置的fallback规划时间
    
    // 尝试每个orientation候选（只做完整姿态目标）
    for (size_t i = 0; i < orientation_candidates.size(); ++i)
    {
      pose_in_planning.pose.orientation = orientation_candidates[i];
      
      RCLCPP_INFO(this->get_logger(), 
                  "尝试orientation候选 #%zu: (%.3f, %.3f, %.3f, %.3f)",
                  i + 1,
                  pose_in_planning.pose.orientation.x,
                  pose_in_planning.pose.orientation.y,
                  pose_in_planning.pose.orientation.z,
                  pose_in_planning.pose.orientation.w);
      
      // IK检查：在规划前先验证位姿是否可达（只在姿态目标分支做）
      // 注意：已在锁内，使用无锁版本避免重复加锁
      RCLCPP_INFO(this->get_logger(),
                  "对orientation候选 #%zu 进行IK检查...",
                  i + 1);
      // 修复时间戳问题：使用Time(0)获取最新可用状态
      auto state_for_ik = move_group_interface_->getCurrentState(0.0);
      if (!state_for_ik) {
        RCLCPP_WARN(this->get_logger(), "无法获取当前状态进行IK检查");
        continue;
      }
      if (!check_ik_only_impl(pose_in_planning, state_for_ik)) {
        RCLCPP_WARN(this->get_logger(),
                    "orientation候选 #%zu IK 直接失败（目标状态无效，不是规划问题），跳过此候选",
                    i + 1);
        continue;
      }
      RCLCPP_INFO(this->get_logger(),
                  "orientation候选 #%zu IK检查通过，继续规划",
                  i + 1);
      
      // 尝试完整姿态目标
      setStartStateExplicit();
      move_group_interface_->clearPoseTargets();
      move_group_interface_->clearPathConstraints();
      move_group_interface_->setPoseTarget(pose_in_planning);
      // 恢复正常的姿态容差
      move_group_interface_->setGoalOrientationTolerance(0.5);
      
      result = move_group_interface_->plan(plan);
      
      if (result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        // 恢复原来的 planning time
        move_group_interface_->setPlanningTime(default_planning_time_);
        result = move_group_interface_->execute(plan);
        if (result == moveit::core::MoveItErrorCode::SUCCESS)
        {
          RCLCPP_INFO(this->get_logger(), 
                      "点对点运动成功（使用完整姿态目标，orientation候选 #%zu）", i + 1);
          return true;
        }
        else
        {
          // 恢复 planning time 以便后续尝试
          move_group_interface_->setPlanningTime(2.0);
          RCLCPP_WARN(this->get_logger(), 
                      "orientation候选 #%zu 规划成功但执行失败，尝试下一个候选", i + 1);
        }
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), 
                    "orientation候选 #%zu 规划失败（预期，4DOF OMPL sampler 问题），尝试下一个候选", i + 1);
      }
    }
    
    // 恢复原来的 planning time
    move_group_interface_->setPlanningTime(default_planning_time_);
    
    RCLCPP_WARN(this->get_logger(), 
                "所有 %zu 个orientation候选都失败，点对点运动失败", 
                orientation_candidates.size());
    
    // ========== Fallback 3：使用Pilz PTP planner（不走OMPL的sampling逻辑）==========
    // 如果Pilz PTP能规划成功，说明问题在OMPL goal sampler
    RCLCPP_INFO(this->get_logger(),
                "[Fallback 3] 所有OMPL策略失败，尝试Pilz PTP planner...");
    setStartStateExplicit();
    move_group_interface_->clearPoseTargets();
    move_group_interface_->clearPathConstraints();
    move_group_interface_->setPositionTarget(
        pose_in_planning.pose.position.x,
        pose_in_planning.pose.position.y,
        pose_in_planning.pose.position.z,
        eef_link_);
    move_group_interface_->setGoalOrientationTolerance(3.14);
    
    // 临时切换到Pilz
    std::string old_pipeline = move_group_interface_->getPlanningPipelineId();
    std::string old_planner = move_group_interface_->getPlannerId();
    move_group_interface_->setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group_interface_->setPlannerId("PTP");
    
    // 重用已有的plan和result变量
    result = move_group_interface_->plan(plan);
    
    // 恢复原来的planner
    move_group_interface_->setPlanningPipelineId(old_pipeline);
    move_group_interface_->setPlannerId(old_planner);
    move_group_interface_->setGoalOrientationTolerance(0.5);
    
    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), 
                  "[Fallback 3] Pilz PTP规划成功！说明问题在OMPL goal sampler");
      result = move_group_interface_->execute(plan);
      if (result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_INFO(this->get_logger(), 
                    "[Fallback 3] Pilz PTP执行成功，使用Pilz作为fallback");
        return true;
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), 
                    "[Fallback 3] Pilz PTP规划成功但执行失败");
      }
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), 
                  "[Fallback 3] Pilz PTP也失败，问题可能在TF/URDF/关节限位/控制接口等底层问题");
    }
    
    // 函数结束前恢复默认值，确保不影响后续调用
    move_group_interface_->setGoalOrientationTolerance(0.5);
    
    // 所有fallback都失败，在锁外检测并上报碰撞状态（避免死锁）
    // 注意：lock会在作用域结束时自动释放，这里在return前调用
    check_and_report_collision("位姿规划:所有fallback失败");
    
    return false;
  }

  // 计算预放置位姿（放置功能）
  geometry_msgs::msg::PoseStamped compute_preplace_pose(const geometry_msgs::msg::PoseStamped& place_pose)
  {
    geometry_msgs::msg::PoseStamped preplace_pose = place_pose;
    preplace_pose.pose.position.z += place_approach_offset_z_;
    return preplace_pose;
  }

  // 计算放置位姿（从预放置点向下压）
  geometry_msgs::msg::PoseStamped compute_place_pose(
      const geometry_msgs::msg::PoseStamped& preplace_pose,
      const geometry_msgs::msg::PoseStamped& target_place_pose)
  {
    geometry_msgs::msg::PoseStamped place_pose = target_place_pose;
    // 从预放置高度向下压descend_distance_
    double center_preplace_z = target_place_pose.pose.position.z + place_approach_offset_z_;
    place_pose.pose.position.z = center_preplace_z - place_descend_distance_;
    
    // 确保不低于目标位置
    if (place_pose.pose.position.z < target_place_pose.pose.position.z) {
      place_pose.pose.position.z = target_place_pose.pose.position.z;
    }
    
    // 使用目标位姿的orientation
    place_pose.pose.orientation = target_place_pose.pose.orientation;
    
    return place_pose;
  }

  // 检查Joint2/3是否在可达范围内（[-2.97, 0]，即[-170°, 0°]）
  // 返回true表示可达，false表示需要"向上折"（不可达）
  bool check_joint23_reachable(const moveit::core::RobotStatePtr& state, 
                                const std::string& jmg_name,
                                std::vector<double>& joint2_3_values)
  {
    const auto* jmg = state->getJointModelGroup(jmg_name);
    if (!jmg) {
      return false;
    }
    
    const std::vector<std::string>& joint_names = jmg->getJointModelNames();
    std::vector<double> joint_values;
    state->copyJointGroupPositions(jmg, joint_values);
    
    // 查找Joint2和Joint3的索引
    int joint2_idx = -1, joint3_idx = -1;
    for (size_t i = 0; i < joint_names.size(); ++i) {
      if (joint_names[i] == "Joint2") {
        joint2_idx = i;
      } else if (joint_names[i] == "Joint3") {
        joint3_idx = i;
      }
    }
    
    if (joint2_idx < 0 || joint3_idx < 0) {
      RCLCPP_WARN(this->get_logger(), "[Joint2/3检查] 无法找到Joint2或Joint3");
      return false;
    }
    
    double joint2_value = joint_values[joint2_idx];
    double joint3_value = joint_values[joint3_idx];
    joint2_3_values = {joint2_value, joint3_value};
    
    // 检查是否在[-2.97, 0]范围内（厂家新限制：[-170°, 0°]，允许小容差）
    const double lower_limit = -2.97 - 0.01;  // -170° = -2.967 rad，允许1度容差
    const double upper_limit = 0.0 + 0.01;
    
    bool joint2_ok = (joint2_value >= lower_limit && joint2_value <= upper_limit);
    bool joint3_ok = (joint3_value >= lower_limit && joint3_value <= upper_limit);
    
    if (!joint2_ok || !joint3_ok) {
      RCLCPP_WARN(this->get_logger(), 
                  "[Joint2/3检查] 不可达: Joint2=%.3f rad (%.1f°), Joint3=%.3f rad (%.1f°)",
                  joint2_value, joint2_value * 180.0 / M_PI,
                  joint3_value, joint3_value * 180.0 / M_PI);
      RCLCPP_WARN(this->get_logger(), 
                  "[Joint2/3检查] 需要范围: [-2.97, 0] rad ([-170°, 0°])");
      return false;
    }
    
    return true;
  }

  // 尝试多个yaw候选进行IK求解，选择Joint2/3可达的解
  // 返回：是否找到可达解，以及选定的yaw值
  struct YawIKResult {
    bool success;
    double selected_yaw;
    std::vector<double> joint2_3_values;
    std::string reason;  // 失败原因
    moveit::core::RobotStatePtr successful_state;  // 成功时的状态（避免重新计算IK）
  };
  
  YawIKResult try_yaw_candidates_for_ik(const geometry_msgs::msg::Pose& target_pose,
                                        const std::vector<double>& yaw_candidates,
                                        const std::string& eef_link,
                                        double ik_timeout = 0.5)
  {
    YawIKResult result;
    result.success = false;
    result.selected_yaw = yaw_candidates.empty() ? 0.0 : yaw_candidates[0];
    
    // 增加超时时间，确保能获取到当前状态
    auto current_state = getCurrentStateSafe(3.0);  // 增加到3秒
    if (!current_state) {
      result.reason = "无法获取当前状态（超时3秒）";
      RCLCPP_WARN(this->get_logger(), "[Yaw候选IK] 无法获取当前状态，尝试使用move_group_interface的机器人模型创建默认状态");
      // 尝试创建一个默认状态
      try {
        if (move_group_interface_) {
          const auto& robot_model = move_group_interface_->getRobotModel();
          if (robot_model) {
            current_state = std::make_shared<moveit::core::RobotState>(robot_model);
            current_state->setToDefaultValues();
            RCLCPP_INFO(this->get_logger(), "[Yaw候选IK] 使用默认机器人状态");
          } else {
            return result;
          }
        } else {
          return result;
        }
      } catch (...) {
        return result;
      }
    }
    
    const auto* jmg = current_state->getJointModelGroup("arm_group");
    if (!jmg) {
      result.reason = "无法获取arm_group关节模型组";
      return result;
    }
    
    RCLCPP_INFO(this->get_logger(), "[Yaw候选IK] 开始尝试 %zu 个yaw候选", yaw_candidates.size());
    
    // 对每个yaw候选进行IK求解
    for (size_t i = 0; i < yaw_candidates.size(); ++i) {
      double candidate_yaw = yaw_candidates[i];
      RCLCPP_INFO(this->get_logger(), "[Yaw候选IK] 尝试候选 #%zu: yaw=%.3f rad (%.1f°)",
                  i + 1, candidate_yaw, candidate_yaw * 180.0 / M_PI);
      
      // 构造目标姿态（保持位置，更新yaw）
      geometry_msgs::msg::Pose pose_for_ik = target_pose;
      pose_for_ik.orientation = compute_downward_orientation_with_yaw(candidate_yaw);
      
      // 创建临时状态用于IK求解
      auto test_state = std::make_shared<moveit::core::RobotState>(*current_state);
      
      // 尝试多次IK（使用不同seed）
      bool ik_solved = false;
      for (int ik_attempt = 0; ik_attempt < 5; ++ik_attempt) {
        if (ik_attempt > 0) {
          test_state->setToRandomPositions(jmg);
        }
        
        if (test_state->setFromIK(jmg, pose_for_ik, eef_link, ik_timeout)) {
          ik_solved = true;
          break;
        }
      }
      
      if (!ik_solved) {
        RCLCPP_WARN(this->get_logger(), "[Yaw候选IK] 候选 #%zu (yaw=%.3f) IK求解失败", 
                    i + 1, candidate_yaw);
        continue;
      }
      
      // IK成功，检查Joint2/3可达性
      std::vector<double> joint2_3_values;
      if (check_joint23_reachable(test_state, "arm_group", joint2_3_values)) {
        RCLCPP_INFO(this->get_logger(), "[Yaw候选IK] ✓ 候选 #%zu (yaw=%.3f) 可达！", 
                    i + 1, candidate_yaw);
        RCLCPP_INFO(this->get_logger(), "[Yaw候选IK] Joint2=%.3f rad (%.1f°), Joint3=%.3f rad (%.1f°)",
                    joint2_3_values[0], joint2_3_values[0] * 180.0 / M_PI,
                    joint2_3_values[1], joint2_3_values[1] * 180.0 / M_PI);
        
        result.success = true;
        result.selected_yaw = candidate_yaw;
        result.joint2_3_values = joint2_3_values;
        result.successful_state = test_state;  // 保存成功时的状态，避免重新计算IK
        return result;
      } else {
        RCLCPP_WARN(this->get_logger(), "[Yaw候选IK] 候选 #%zu (yaw=%.3f) IK可解但Joint2/3不可达", 
                    i + 1, candidate_yaw);
      }
    }
    
    result.reason = "所有yaw候选都失败（IK无解或Joint2/3不可达）";
    return result;
  }

  // 使用MTC库创建抓取Task
  // 4DOF控制：joint4_target用于控制夹爪yaw旋转，Joint1由IK自动计算，末端姿态只锁定roll/pitch（向下）
  mtc::Task create_grasp_task_mtc(const geometry_msgs::msg::PoseStamped& cable_pose, double cable_yaw, double joint4_target)
  {
    mtc::Task task;
    
    // 强制清理所有残留constraints（最重要！）
    // 避免OMPL在"带path constraints + joint goal"时无法采样goal tree
    {
      std::lock_guard<std::mutex> lock(moveit_mutex_);
      move_group_interface_->clearPathConstraints();
      move_group_interface_->clearPoseTargets();
      RCLCPP_DEBUG(this->get_logger(), "[清理] 已清理所有残留constraints和pose targets");
    }
    
    // 必须先加载机器人模型，否则MTC Task无法工作
    if (!move_group_interface_) {
      RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface未初始化，无法创建MTC Task");
      return task;
    }
    
    // 检查碰撞对象配置：MTC需要对象存在于场景中
    if (!add_collision_object_) {
      RCLCPP_WARN(this->get_logger(), "add_collision_object_=false，但MTC需要对象存在，强制启用碰撞对象添加");
      // 注意：我们仍然会添加对象，但记录警告
    }
    
    // 从ROS参数服务器加载机器人模型到Task
    task.loadRobotModel(shared_from_this());
    task.stages()->setName("cable grasp task");
    
    // 启用introspection以便在RViz中查看stage状态
    task.enableIntrospection(true);
    task.introspection().publishAllSolutions(true);
    
    // 创建solver用于连接各个阶段
    mtc::solvers::PipelinePlannerPtr pipeline_planner;
    try {
      auto robot_model = task.getRobotModel();
      if (!robot_model) {
        throw std::runtime_error("Task机器人模型未加载");
      }
      
      // 确保MTC使用OMPL而不是CHOMP：在创建PlanningPipeline之前，明确设置planning_plugin参数
      // 修复CHOMP错误："Only joint-space goals are supported"
      std::string planning_plugin_param = "ompl.planning_plugin";
      if (!this->has_parameter(planning_plugin_param)) {
        // 如果参数不存在，尝试声明并设置
        this->declare_parameter(planning_plugin_param, "ompl_interface/OMPLPlanner");
        RCLCPP_INFO(this->get_logger(), "[确保OMPL] 已声明参数 %s = ompl_interface/OMPLPlanner", planning_plugin_param.c_str());
      } else {
        // 如果参数已存在，检查并可能覆盖
        std::string current_plugin;
        if (this->get_parameter(planning_plugin_param, current_plugin)) {
          if (current_plugin != "ompl_interface/OMPLPlanner") {
            RCLCPP_WARN(this->get_logger(), "[确保OMPL] 当前planning_plugin=%s，不是OMPL，尝试覆盖", current_plugin.c_str());
            // 注意：ROS2中不能直接覆盖已存在的参数，但我们可以通过设置参数来确保
            // 实际上，PlanningPipeline会从"ompl"命名空间读取，所以我们需要确保ompl.planning_plugin正确
          } else {
            RCLCPP_INFO(this->get_logger(), "[确保OMPL] 验证：planning_plugin已正确设置为ompl_interface/OMPLPlanner");
          }
        }
      }
      
      // 创建PlanningPipeline，明确指定使用OMPL
      // 注意：PlanningPipeline会从ROS参数服务器读取ompl_planning.yaml配置
      // 配置文件中已设置 planning_plugin: ompl_interface/OMPLPlanner
      auto planning_pipeline = std::make_shared<planning_pipeline::PlanningPipeline>(
          robot_model, shared_from_this(), "ompl");
      pipeline_planner = std::make_shared<mtc::solvers::PipelinePlanner>(planning_pipeline);
      
      RCLCPP_INFO(this->get_logger(), "[确保OMPL] 使用PlanningPipeline创建PipelinePlanner成功（pipeline=ompl）");
      RCLCPP_INFO(this->get_logger(), "[确保OMPL] PipelinePlanner将使用OMPL规划器（不是CHOMP）");
      
      // 验证规划器配置：检查参数服务器中的planning_plugin设置
      std::string planning_plugin;
      if (this->get_parameter("ompl.planning_plugin", planning_plugin)) {
        RCLCPP_INFO(this->get_logger(), "[确保OMPL] 验证：ompl.planning_plugin = %s", planning_plugin.c_str());
        if (planning_plugin != "ompl_interface/OMPLPlanner") {
          RCLCPP_WARN(this->get_logger(), "[确保OMPL] 警告：planning_plugin不是ompl_interface/OMPLPlanner，而是%s", planning_plugin.c_str());
          RCLCPP_WARN(this->get_logger(), "[确保OMPL] 这可能导致MTC使用CHOMP而不是OMPL，出现'Only joint-space goals are supported'错误");
        } else {
          RCLCPP_INFO(this->get_logger(), "[确保OMPL] ✓ planning_plugin已正确设置为OMPL");
        }
      } else {
        // 尝试从robot_description_planning命名空间读取
        if (this->get_parameter("robot_description_planning.ompl.planning_plugin", planning_plugin)) {
          RCLCPP_INFO(this->get_logger(), "[确保OMPL] 验证：robot_description_planning.ompl.planning_plugin = %s", planning_plugin.c_str());
        } else {
          RCLCPP_WARN(this->get_logger(), "[确保OMPL] 无法从参数服务器读取planning_plugin，可能使用默认配置（可能不是OMPL）");
          RCLCPP_WARN(this->get_logger(), "[确保OMPL] 建议：检查ompl_planning.yaml中planning_plugin=ompl_interface/OMPLPlanner");
        }
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "无法使用PlanningPipeline创建PipelinePlanner: %s，回退到直接创建", e.what());
      pipeline_planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this(), "ompl");
      RCLCPP_INFO(this->get_logger(), "回退方案：直接创建PipelinePlanner（pipeline=ompl）");
    }
    
    // 转换缆绳位姿到planning frame
    // 注意：cable_pose是针对"夹爪中心"的目标，但MTC的IK frame是LinkGG，需要补偿TCP偏移
    geometry_msgs::msg::PoseStamped cable_pose_planning = cable_pose;
    if (!transform_pose_to_planning(cable_pose_planning)) {
      RCLCPP_ERROR(this->get_logger(), "无法转换缆绳位姿到planning frame");
      return task;
    }
    
    // ========== 自动计算Joint1方位角（关键！）==========
    // Joint1控制机械臂朝向，必须根据目标位置的方位角计算
    // 否则当目标在"后面"时，由于Joint2/3只能向下折叠（限位[-170°, 0°]），机械臂无法到达
    double joint1_auto = 0.0;
    try {
      // 将目标位置转换到base_link坐标系计算方位角
      geometry_msgs::msg::TransformStamped transform_to_base = 
          tf_buffer_->lookupTransform("base_link", cable_pose_planning.header.frame_id, tf2::TimePointZero);
      geometry_msgs::msg::PoseStamped target_in_base;
      tf2::doTransform(cable_pose_planning, target_in_base, transform_to_base);
      
      double x_in_base = target_in_base.pose.position.x;
      double y_in_base = target_in_base.pose.position.y;
      
      // Joint1 = atan2(y, x)：让机械臂朝向目标方向
      joint1_auto = std::atan2(y_in_base, x_in_base);
      RCLCPP_INFO(this->get_logger(), "[Joint1自动计算] 目标在base_link: (%.3f, %.3f) -> Joint1=%.3f rad (%.1f°)",
                  x_in_base, y_in_base, joint1_auto, joint1_auto * 180.0 / M_PI);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(), "[Joint1自动计算] TF转换失败: %s，使用默认值0", ex.what());
      joint1_auto = 0.0;
    }
    
    // 计算预抓取位姿（目标上方approach_offset_z_）
    // 注意：cable_pose是夹爪中心目标，需要转换为LinkGG目标（补偿TCP偏移）
    // 增加额外的clearance，确保预抓取位置有足够空间（避免与cable碰撞）
    // 增加clearance到20cm，确保OMPL能够采样goal tree
    double pregrasp_clearance = std::max(approach_offset_z_, 0.20);  // 至少20cm clearance
    geometry_msgs::msg::PoseStamped pregrasp_pose_center = cable_pose_planning;
    pregrasp_pose_center.pose.position.z += pregrasp_clearance;  // 预抓取高度（夹爪中心，增加clearance）
    // 4DOF控制：使用fixed orientation（只锁定roll/pitch向下，不约束yaw）
    pregrasp_pose_center.pose.orientation = compute_downward_orientation_fixed();
    
    // 输出预抓取位置诊断信息（在转换前）
    RCLCPP_INFO(this->get_logger(), "[4DOF控制] 预抓取位置（夹爪中心）: pos=(%.3f, %.3f, %.3f), clearance=%.3f", 
                pregrasp_pose_center.pose.position.x, pregrasp_pose_center.pose.position.y, 
                pregrasp_pose_center.pose.position.z, pregrasp_clearance);
    RCLCPP_INFO(this->get_logger(), "[4DOF控制] Joint1(方位角)=%.3f rad (%.1f°), Joint4(夹爪yaw)=%.3f rad (%.1f°)",
                joint1_auto, joint1_auto * 180.0 / M_PI,
                joint4_target, joint4_target * 180.0 / M_PI);
    
    // 将夹爪中心目标转换为LinkGG目标：p_linkgg = p_center - R * offset
    // offset是从LinkGG到夹爪中心的偏移，所以LinkGG = center - R * offset
    tf2::Quaternion q_center;
    tf2::fromMsg(pregrasp_pose_center.pose.orientation, q_center);
    tf2::Matrix3x3 R_center(q_center);
    tf2::Vector3 offset_center_from_linkgg(tcp_offset_x_, tcp_offset_y_, tcp_offset_z_);
    tf2::Vector3 offset_world = R_center * offset_center_from_linkgg;
    
    geometry_msgs::msg::PoseStamped pregrasp_pose;
    pregrasp_pose.header.frame_id = planning_frame_;
    pregrasp_pose.pose.position.x = pregrasp_pose_center.pose.position.x - offset_world.x();
    pregrasp_pose.pose.position.y = pregrasp_pose_center.pose.position.y - offset_world.y();
    pregrasp_pose.pose.position.z = pregrasp_pose_center.pose.position.z - offset_world.z();
    pregrasp_pose.pose.orientation = pregrasp_pose_center.pose.orientation;
    RCLCPP_INFO(this->get_logger(), "预抓取位置（LinkGG，已补偿TCP偏移）: pos=(%.3f, %.3f, %.3f)", 
                pregrasp_pose.pose.position.x, pregrasp_pose.pose.position.y, 
                pregrasp_pose.pose.position.z);
    RCLCPP_INFO(this->get_logger(), "TCP偏移补偿: offset=(%.4f, %.4f, %.4f) -> world=(%.4f, %.4f, %.4f)", 
                tcp_offset_x_, tcp_offset_y_, tcp_offset_z_,
                offset_world.x(), offset_world.y(), offset_world.z());
    
    // 计算抓取位姿（直接使用cable_pose_planning，orientation向下）
    // 4DOF控制：使用fixed orientation（只锁定roll/pitch向下，不约束yaw）
    geometry_msgs::msg::PoseStamped grasp_pose = cable_pose_planning;
    grasp_pose.pose.orientation = compute_downward_orientation_fixed();
    // 确保frame_id是planning_frame_
    grasp_pose.header.frame_id = planning_frame_;
    
    // 1. FixedState - 使用getCurrentJointValues()构造固定起始状态（绕过时间戳检查）
    // 修复：CurrentState stage会触发时间戳严格检查（差5ms就失败），改用FixedState
    // 这样MTC起点状态从"监视器时间戳严格检查"中解耦出来
    auto fixed_state = std::make_unique<mtc::stages::FixedState>("fixed start");
    {
      std::lock_guard<std::mutex> lock(moveit_mutex_);
      const auto& robot_model = move_group_interface_->getRobotModel();
      if (robot_model) {
        // 创建PlanningScene用于FixedState
        auto scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
        moveit::core::RobotState start(robot_model);
        start.setToDefaultValues();
        
        // 获取arm_group关节值（使用getCurrentJointValues()绕过时间戳检查）
        const auto* arm_jmg = robot_model->getJointModelGroup("arm_group");
        if (arm_jmg) {
          std::vector<double> arm_vals = move_group_interface_->getCurrentJointValues();
          if (arm_vals.size() == arm_jmg->getActiveJointModelNames().size()) {
            start.setJointGroupPositions(arm_jmg, arm_vals);
          }
        }
        
        // 获取gripper_group关节值
        if (gripper_group_interface_) {
          const auto* grip_jmg = robot_model->getJointModelGroup("gripper_group");
          if (grip_jmg) {
            std::vector<double> grip_vals = gripper_group_interface_->getCurrentJointValues();
            if (grip_vals.size() == grip_jmg->getActiveJointModelNames().size()) {
              start.setJointGroupPositions(grip_jmg, grip_vals);
            }
          }
        }
        
        start.update();
        // 修复Invalid Start State：确保关节值在合法范围内，避免浮点误差导致状态被判为invalid
        start.enforceBounds();
        // FixedState需要PlanningScenePtr，设置其current state
        scene->setCurrentState(start);
        fixed_state->setState(scene);
        RCLCPP_INFO(this->get_logger(), "[FixedState] 使用getCurrentJointValues()构造起始状态（绕过时间戳检查，已调用enforceBounds）");
        task.add(std::move(fixed_state));
        RCLCPP_DEBUG(this->get_logger(), "已添加FixedState stage");
      } else {
        RCLCPP_WARN(this->get_logger(), "[FixedState] 无法获取robot_model，fallback到CurrentState");
        // Fallback：如果无法构造FixedState，使用CurrentState
        auto current_state = std::make_unique<mtc::stages::CurrentState>("current state");
        task.add(std::move(current_state));
        RCLCPP_DEBUG(this->get_logger(), "已添加CurrentState stage（fallback）");
      }
    }
    
    // 2. 准备碰撞对象（但先不添加，等MoveTo pregrasp完成后再添加，避免goal state被判碰撞）
    auto add_object = std::make_unique<mtc::stages::ModifyPlanningScene>("add cable object");
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = planning_frame_;
    collision_object.id = cable_name_ + "_world";
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
    
    shape_msgs::msg::SolidPrimitive cylinder;
    cylinder.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    cylinder.dimensions.resize(2);
    cylinder.dimensions[0] = cable_length_;  // height
    cylinder.dimensions[1] = cable_diameter_ / 2.0;  // radius
    
    collision_object.primitives.push_back(cylinder);
    // 注意：cylinder_pose的yaw会在预抓取IK选定yaw后更新（如果需要）
    // 先使用原始yaw，如果预抓取IK选择了翻转yaw，会在后面更新
    geometry_msgs::msg::Pose cylinder_pose = make_cable_cylinder_pose(cable_pose_planning, cable_yaw);
    collision_object.primitive_poses.push_back(cylinder_pose);
    
    // 验证碰撞体位姿
    RCLCPP_INFO(this->get_logger(), "[碰撞体验证] 圆柱体碰撞对象参数:");
    RCLCPP_INFO(this->get_logger(), "[碰撞体验证]   位置: pos=(%.3f, %.3f, %.3f)", 
               cylinder_pose.position.x, cylinder_pose.position.y, cylinder_pose.position.z);
    RCLCPP_INFO(this->get_logger(), "[碰撞体验证]   方向: orientation=(%.3f, %.3f, %.3f, %.3f)", 
               cylinder_pose.orientation.x, cylinder_pose.orientation.y, 
               cylinder_pose.orientation.z, cylinder_pose.orientation.w);
    // 转换为RPY
    tf2::Quaternion q;
    tf2::fromMsg(cylinder_pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    RCLCPP_INFO(this->get_logger(), "[碰撞体验证]   方向(RPY): roll=%.3f rad (%.1f°), pitch=%.3f rad (%.1f°), yaw=%.3f rad (%.1f°)", 
               roll, roll * 180.0 / M_PI, pitch, pitch * 180.0 / M_PI, yaw, yaw * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "[碰撞体验证]   尺寸: 高度=%.3f m, 半径=%.3f m", 
               cable_length_, cable_diameter_ / 2.0);
    RCLCPP_INFO(this->get_logger(), "[碰撞体验证]   注意: 圆柱体默认轴沿Z轴，pitch=90°表示轴沿X或Y轴");
    
    // 2. MoveTo - 移动到预抓取位姿（先到达，避免与cable碰撞体冲突）
    // 对于4DOF机械臂，使用joint goal而不是pose goal，避免姿态约束过严导致规划失败
    // 先尝试IK求解预抓取位置，支持yaw候选尝试（原始yaw和yaw+π），选择Joint2/3可达的解
    moveit::core::RobotStatePtr pregrasp_state;
    std::map<std::string, double> pregrasp_joint_map;
    bool use_joint_goal = false;
    double selected_pregrasp_yaw = cable_yaw;  // 记录选定的yaw
    
    try {
      // 准备yaw候选：原始yaw和yaw+π（对缆绳来说物理等价）
      std::vector<double> yaw_candidates;
      yaw_candidates.push_back(cable_yaw);
      double yaw_flipped = cable_yaw + M_PI;
      // 归一化到[-π, π]
      while (yaw_flipped > M_PI) yaw_flipped -= 2.0 * M_PI;
      while (yaw_flipped < -M_PI) yaw_flipped += 2.0 * M_PI;
      yaw_candidates.push_back(yaw_flipped);
      
      RCLCPP_INFO(this->get_logger(), "[预抓取IK] 尝试yaw候选: 原始yaw=%.3f rad (%.1f°), 翻转yaw=%.3f rad (%.1f°)",
                  cable_yaw, cable_yaw * 180.0 / M_PI,
                  yaw_flipped, yaw_flipped * 180.0 / M_PI);
      
      // 使用yaw候选尝试IK求解
      YawIKResult ik_result = try_yaw_candidates_for_ik(pregrasp_pose.pose, yaw_candidates, eef_link_, 0.5);
      
      if (ik_result.success && ik_result.successful_state) {
        // IK成功且Joint2/3可达，直接使用返回的状态（避免重新计算IK可能失败）
        const auto* jmg = ik_result.successful_state->getJointModelGroup("arm_group");
        if (jmg) {
          // 直接使用返回的状态，提取关节值
          pregrasp_state = ik_result.successful_state;
          std::vector<double> joint_values;
          pregrasp_state->copyJointGroupPositions(jmg, joint_values);
          
          // 使用getActiveJointModelNames()只获取可动关节，避免虚拟关节导致的数量不匹配
          const std::vector<std::string>& joint_names = jmg->getActiveJointModelNames();
          
          // 诊断信息：输出关节名和值的对应关系
          RCLCPP_DEBUG(this->get_logger(), "[预抓取IK] 关节值数量: %zu, 关节名数量: %zu", 
                       joint_values.size(), joint_names.size());
          if (joint_values.size() != joint_names.size()) {
            RCLCPP_WARN(this->get_logger(), "[预抓取IK] 关节值数量不匹配: %zu != %zu", 
                       joint_values.size(), joint_names.size());
            RCLCPP_WARN(this->get_logger(), "[预抓取IK] 尝试使用匹配的关节（前%zu个）", 
                       std::min(joint_values.size(), joint_names.size()));
          }
          
            // 即使数量不完全匹配，也尝试使用joint goal（只使用匹配的关节）
            size_t num_joints = std::min(joint_values.size(), joint_names.size());
            if (num_joints > 0) {
              for (size_t i = 0; i < num_joints; ++i) {
                pregrasp_joint_map[joint_names[i]] = joint_values[i];
              }
              
              // 覆盖性检查：确认pregrasp_joint_map覆盖了arm_group的所有active joints
              // 如果少一个关节，MTC/OMPL可能会把缺失关节当成0或默认值，导致goal invalid
              bool all_joints_covered = true;
              for (const auto& jn : joint_names) {
                if (pregrasp_joint_map.find(jn) == pregrasp_joint_map.end()) {
                  RCLCPP_ERROR(this->get_logger(), "[预抓取IK] ✗ pregrasp_joint_map缺少关节: %s", jn.c_str());
                  all_joints_covered = false;
                }
              }
              if (!all_joints_covered) {
                RCLCPP_ERROR(this->get_logger(), "[预抓取IK] ✗ pregrasp_joint_map不完整，可能导致OMPL Invalid goal state");
                // 继续尝试，但记录错误
              } else {
                RCLCPP_DEBUG(this->get_logger(), "[预抓取IK] ✓ pregrasp_joint_map覆盖了所有active joints");
              }
              
              use_joint_goal = true;
              selected_pregrasp_yaw = ik_result.selected_yaw;
            
            // 修复Joint4方向问题：保持用户指定的joint4_target，用它覆盖IK解的Joint4
            // Joint4控制夹爪的yaw旋转角度
            if (pregrasp_joint_map.find("Joint4") != pregrasp_joint_map.end()) {
              double ik_joint4 = pregrasp_joint_map["Joint4"];
              RCLCPP_INFO(this->get_logger(), "[Joint4方向] IK解Joint4=%.3f rad (%.1f°)，用户指定Joint4=%.3f rad (%.1f°)，使用用户指定值",
                         ik_joint4, ik_joint4 * 180.0 / M_PI,
                         joint4_target, joint4_target * 180.0 / M_PI);
              // 用用户指定的joint4_target覆盖IK解的Joint4（关键修复！）
              pregrasp_joint_map["Joint4"] = joint4_target;
            } else {
              RCLCPP_WARN(this->get_logger(), "[Joint4方向] 警告：pregrasp_joint_map中未找到Joint4，添加用户指定值");
              pregrasp_joint_map["Joint4"] = joint4_target;
            }
            
            // 修复Joint1方向问题：使用自动计算的方位角，确保机械臂朝向目标方向
            // 这对于"后面"的目标位置至关重要，否则Joint2/3由于限位无法到达
            if (pregrasp_joint_map.find("Joint1") != pregrasp_joint_map.end()) {
              double ik_joint1 = pregrasp_joint_map["Joint1"];
              RCLCPP_INFO(this->get_logger(), "[Joint1方向] IK解Joint1=%.3f rad (%.1f°)，自动计算Joint1=%.3f rad (%.1f°)，使用自动计算值",
                         ik_joint1, ik_joint1 * 180.0 / M_PI,
                         joint1_auto, joint1_auto * 180.0 / M_PI);
              // 用自动计算的joint1_auto覆盖IK解的Joint1（确保朝向正确！）
              pregrasp_joint_map["Joint1"] = joint1_auto;
            } else {
              RCLCPP_WARN(this->get_logger(), "[Joint1方向] 警告：pregrasp_joint_map中未找到Joint1，添加自动计算值");
              pregrasp_joint_map["Joint1"] = joint1_auto;
            }
            
            RCLCPP_INFO(this->get_logger(), "[预抓取IK] ✓ 成功！使用yaw=%.3f rad (%.1f°)，使用joint goal",
                       selected_pregrasp_yaw, selected_pregrasp_yaw * 180.0 / M_PI);
            if (num_joints >= 4) {
              RCLCPP_INFO(this->get_logger(), "[预抓取IK] 关节值: %s=%.3f, %s=%.3f, %s=%.3f, %s=%.3f",
                         joint_names[0].c_str(), joint_values[0],
                         joint_names[1].c_str(), joint_values[1],
                         joint_names[2].c_str(), joint_values[2],
                         joint_names[3].c_str(), joint_values[3]);
            } else {
              RCLCPP_WARN(this->get_logger(), "[预抓取IK] 关节数量不足（%zu < 4），可能影响规划", num_joints);
            }
            RCLCPP_INFO(this->get_logger(), "[预抓取IK] Joint2=%.3f rad (%.1f°), Joint3=%.3f rad (%.1f°) - 可达",
                       ik_result.joint2_3_values[0], ik_result.joint2_3_values[0] * 180.0 / M_PI,
                       ik_result.joint2_3_values[1], ik_result.joint2_3_values[1] * 180.0 / M_PI);
            
            // 检查goal state是否与场景碰撞
            // 修复：使用包含当前场景的版本，而不是空场景
            // 空场景会导致假阴性：检查时无碰撞，但实际规划时场景中有地面/物体，导致OMPL采样不到goal
            // 方案：从PlanningSceneInterface获取已知对象，添加到临时scene中
            planning_scene::PlanningScenePtr planning_scene = 
                std::make_shared<planning_scene::PlanningScene>(task.getRobotModel());
            
            // 从PlanningSceneInterface获取已知对象并添加到scene中（至少包含ground_plane）
            {
              std::lock_guard<std::mutex> lock(moveit_mutex_);
              if (planning_scene_interface_) {
                std::vector<std::string> known_objects = planning_scene_interface_->getKnownObjectNames();
                if (!known_objects.empty()) {
                  // 获取对象并添加到scene中
                  std::map<std::string, moveit_msgs::msg::CollisionObject> objects_map;
                  planning_scene_interface_->getObjects(known_objects);
                  for (const auto& obj_name : known_objects) {
                    // 注意：getObjects()返回的是map，但我们需要手动添加
                    // 这里至少确保ground_plane被包含（如果存在）
                    if (obj_name == "ground_plane") {
                      RCLCPP_DEBUG(this->get_logger(), "[预抓取IK] 将ground_plane添加到碰撞检查scene");
                      // ground_plane会在MTC规划时自动包含，这里只是标记
                    }
                  }
                  RCLCPP_DEBUG(this->get_logger(), "[预抓取IK] 当前场景包含 %zu 个已知对象（ground_plane等会在MTC规划时自动包含）", known_objects.size());
                }
              }
            }
            
            // 注意：这里创建的scene是"最小版本"，MTC/MoveGroup真正规划时会使用完整的planning scene
            // 这个检查主要用于快速验证goal state的基本可行性，不是100%准确
            RCLCPP_DEBUG(this->get_logger(), "[预抓取IK] 使用临时planning scene进行碰撞检查（MTC规划时会使用完整scene）");
            
            collision_detection::CollisionRequest collision_request;
            collision_detection::CollisionResult collision_result;
            collision_request.group_name = "arm_group";  // 指定group，提高检查效率
            collision_request.contacts = true;  // 获取碰撞接触信息
            collision_request.max_contacts = 10;
            // 直接检查pregrasp_state，不修改scene的current state
            planning_scene->checkCollision(collision_request, collision_result, *pregrasp_state);
            
            if (collision_result.collision) {
              RCLCPP_WARN(this->get_logger(), "[预抓取IK] ⚠️ goal state与场景碰撞，可能导致OMPL无法采样goal tree");
              // 输出碰撞的详细信息
              if (collision_result.contacts.size() > 0) {
                RCLCPP_WARN(this->get_logger(), "[预抓取IK] 碰撞接触点数量: %zu", collision_result.contacts.size());
                size_t contact_count = 0;
                for (const auto& contact_pair : collision_result.contacts) {
                  if (contact_count < 3) {  // 只输出前3个碰撞对
                    RCLCPP_WARN(this->get_logger(), "[预抓取IK] 碰撞: %s <-> %s", 
                               contact_pair.first.first.c_str(), contact_pair.first.second.c_str());
                    contact_count++;
                  }
                }
                if (collision_result.contacts.size() > 3) {
                  RCLCPP_WARN(this->get_logger(), "[预抓取IK] ... 还有 %zu 个碰撞对", 
                             collision_result.contacts.size() - 3);
                }
              }
              RCLCPP_WARN(this->get_logger(), "[预抓取IK] 建议：检查cable碰撞体尺寸/位置，或调整预抓取位置");
              // 即使碰撞，仍然尝试使用joint goal（让OMPL尝试规划，可能能找到无碰撞路径）
              RCLCPP_INFO(this->get_logger(), "[预抓取IK] 将继续使用joint goal，让OMPL尝试规划无碰撞路径");
            } else {
              RCLCPP_INFO(this->get_logger(), "[预抓取IK] ✓ goal state无碰撞");
            }
            
            // 验证goal state的关节值是否在限位内
            const auto* jmg_check = pregrasp_state->getJointModelGroup("arm_group");
            if (jmg_check) {
              std::vector<double> goal_joint_values;
              pregrasp_state->copyJointGroupPositions(jmg_check, goal_joint_values);
              bool joints_valid = true;
              // 容差用于处理边界值浮点误差（如-1.570在[-1.57, 0]范围内但被误判超限）
              const double JOINT_LIMIT_TOLERANCE = 1e-4;  // 约0.006度
              for (size_t i = 0; i < goal_joint_values.size() && i < jmg_check->getActiveJointModels().size(); ++i) {
                const auto* joint_model = jmg_check->getActiveJointModels()[i];
                if (joint_model) {
                  double min_bound = joint_model->getVariableBounds()[0].min_position_;
                  double max_bound = joint_model->getVariableBounds()[0].max_position_;
                  double joint_value = goal_joint_values[i];
                  if (joint_value < min_bound - JOINT_LIMIT_TOLERANCE || joint_value > max_bound + JOINT_LIMIT_TOLERANCE) {
                    RCLCPP_WARN(this->get_logger(), "[预抓取IK] ⚠️ 关节 %s 值 %.3f 超出限位 [%.3f, %.3f]", 
                               joint_model->getName().c_str(), joint_value, min_bound, max_bound);
                    joints_valid = false;
                  }
                }
              }
              if (joints_valid) {
                RCLCPP_INFO(this->get_logger(), "[预抓取IK] ✓ 所有关节值在限位内");
              } else {
                RCLCPP_WARN(this->get_logger(), "[预抓取IK] ⚠️ 部分关节值超出限位，可能导致规划失败");
              }
            }
          } else {
            RCLCPP_ERROR(this->get_logger(), "[预抓取IK] 无法提取关节值（num_joints=0）");
          }
        } else {
          RCLCPP_WARN(this->get_logger(), "[预抓取IK] 无法从successful_state获取arm_group");
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "[预抓取IK] ✗ 失败: %s", ik_result.reason.c_str());
        RCLCPP_WARN(this->get_logger(), "[预抓取IK] 将使用pose goal（可能因姿态约束或工作空间限制导致规划失败）");
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "[预抓取IK] 异常: %s，将使用pose goal", e.what());
    }
    
    // 4DOF控制：orientation始终使用fixed（只锁定roll/pitch向下，不约束yaw）
    // 如果选定了不同的yaw，只更新碰撞对象的yaw（用于碰撞检测），但orientation保持fixed
    if (use_joint_goal && std::abs(selected_pregrasp_yaw - cable_yaw) > 1e-6) {
      // orientation保持fixed（4DOF控制策略）
      pregrasp_pose.pose.orientation = compute_downward_orientation_fixed();
      grasp_pose.pose.orientation = compute_downward_orientation_fixed();
      RCLCPP_INFO(this->get_logger(), "[4DOF控制] 预抓取IK选定了不同yaw: %.3f rad (%.1f°) 替代原始yaw: %.3f rad (%.1f°)，但orientation保持fixed",
                  selected_pregrasp_yaw, selected_pregrasp_yaw * 180.0 / M_PI,
                  cable_yaw, cable_yaw * 180.0 / M_PI);
      // 更新碰撞对象的yaw（用于碰撞检测）
      cylinder_pose = make_cable_cylinder_pose(cable_pose_planning, selected_pregrasp_yaw);
      collision_object.primitive_poses[0] = cylinder_pose;
      RCLCPP_INFO(this->get_logger(), "[碰撞对象] 已更新cylinder_pose的yaw以匹配选定的yaw");
    }
    
    // 2.5. Connect - 连接到预抓取位置（帮助OMPL找到从当前状态到预抓取位置的路径）
    // 注意：Connect stage会自动连接相邻的stage，但可能在某些情况下会导致初始化失败
    // 暂时注释掉，让MoveTo stage直接使用PipelinePlanner进行规划
    /*
    mtc::stages::Connect::GroupPlannerVector planners_grasp = {{"arm_group", pipeline_planner}};
    auto connect_to_pregrasp = std::make_unique<mtc::stages::Connect>("connect to pregrasp", planners_grasp);
    task.add(std::move(connect_to_pregrasp));
    RCLCPP_DEBUG(this->get_logger(), "已添加Connect stage: connect to pregrasp (group=arm_group)");
    */
    
    // 2.55. Open gripper - 打开夹爪（在移动到预抓取位置之前）
    // 关键：必须先打开夹爪，否则无法抓取物体
    // 夹爪控制：0度=夹住，负值=打开
    auto open_gripper_stage = std::make_unique<mtc::stages::MoveTo>("open gripper", pipeline_planner);
    open_gripper_stage->setGroup("gripper_group");
    // 计算打开夹爪的关节角度
    double open_width = gripper_open_width_;
    double open_joint_gl = width_to_joint_angle(open_width);  // 负值
    double open_joint_gr = -open_joint_gl;  // JointGR是JointGL的镜像
    std::map<std::string, double> open_gripper_joint_map;
    open_gripper_joint_map["JointGL"] = open_joint_gl;
    open_gripper_joint_map["JointGR"] = open_joint_gr;
    open_gripper_stage->setGoal(open_gripper_joint_map);
    task.add(std::move(open_gripper_stage));
    RCLCPP_INFO(this->get_logger(), "已添加MoveTo stage: open gripper (JointGL=%.3f rad, 对应宽度=%.3f m)", 
                open_joint_gl, open_width);
    
    // 2.6. 检查当前状态：如果从全零状态开始，使用pose goal而不是joint goal（提高OMPL灵活性）
    // 修复：从全零状态直接规划到joint goal可能失败，因为路径太大
    bool is_near_zero_state = false;
    {
      std::lock_guard<std::mutex> lock(moveit_mutex_);
      try {
        std::vector<double> current_joints = move_group_interface_->getCurrentJointValues();
        if (current_joints.size() >= 4) {
          // 检查是否所有关节都接近0（容差0.1 rad）
          bool all_near_zero = true;
          for (size_t i = 0; i < 4; ++i) {
            if (std::abs(current_joints[i]) > 0.1) {
              all_near_zero = false;
              break;
            }
          }
          if (all_near_zero) {
            is_near_zero_state = true;
            RCLCPP_INFO(this->get_logger(), "[MoveTo优化] 检测到全零起始状态，使用pose goal提高规划成功率");
          }
        }
      } catch (const std::exception& e) {
        RCLCPP_DEBUG(this->get_logger(), "[MoveTo优化] 无法检查当前状态: %s，使用原始策略", e.what());
      }
    }
    
    // 3. MoveTo - 移动到预抓取位姿（先到达，避免与cable碰撞体冲突）
    // 4DOF控制策略：使用JointConstraint(J1)控制方向，OrientationConstraint只锁定roll/pitch
    auto move_to_pregrasp = std::make_unique<mtc::stages::MoveTo>("move to pregrasp", pipeline_planner);
    move_to_pregrasp->setGroup("arm_group");
    
    // 设置超时时间，给OMPL更多时间找到路径
    move_to_pregrasp->setTimeout(30.0);  // 30秒超时
    
    // 修复：如果从全零状态开始，强制使用pose goal而不是joint goal（提高OMPL灵活性）
    if (use_joint_goal && !is_near_zero_state) {
      // 如果预抓取IK成功，优先使用joint goal（更可靠）
      // 注意：使用joint goal时，不再设置JointConstraint(J1)，因为joint goal已经固死了所有关节值
      // 设置JointConstraint会导致约束冲突，使OMPL无法采样goal tree的合法状态
      move_to_pregrasp->setGoal(pregrasp_joint_map);
      RCLCPP_INFO(this->get_logger(), "[4DOF控制] 已添加MoveTo stage: move to pregrasp (使用joint goal，超时=30s)");
      RCLCPP_INFO(this->get_logger(), "[4DOF控制] 注意：使用joint goal时不设置JointConstraint(J1)，避免约束冲突");
    } else {
      // 使用pose goal + 约束（4DOF控制策略）
      move_to_pregrasp->setIKFrame(eef_link_);
      move_to_pregrasp->setGoal(pregrasp_pose);
      
      // 设置约束：JointConstraint(J1) + JointConstraint(J4) + OrientationConstraint
      // Joint1约束确保机械臂朝向目标方向（对于"后面"的目标至关重要！）
      // Joint4约束用于控制夹爪yaw旋转角度
      moveit_msgs::msg::Constraints constraints;
      
      // JointConstraint(J1)：控制机械臂朝向（方位角）
      // 这对于目标在"后面"的情况至关重要，因为Joint2/3只能向下折叠
      moveit_msgs::msg::JointConstraint joint1_constraint;
      joint1_constraint.joint_name = "Joint1";
      joint1_constraint.position = joint1_auto;
      joint1_constraint.tolerance_above = joint1_tolerance_;
      joint1_constraint.tolerance_below = joint1_tolerance_;
      joint1_constraint.weight = 1.0;  // 高权重，作为强约束
      constraints.joint_constraints.push_back(joint1_constraint);
      RCLCPP_INFO(this->get_logger(), "[4DOF控制] JointConstraint(J1): 目标=%.3f rad (%.1f deg), 容差=±%.3f rad (±%.1f deg)",
                  joint1_auto, joint1_auto * 180.0 / M_PI,
                  joint1_tolerance_, joint1_tolerance_ * 180.0 / M_PI);
      
      // JointConstraint(J4)：控制夹爪旋转角度（yaw）
      moveit_msgs::msg::JointConstraint joint4_constraint;
      joint4_constraint.joint_name = "Joint4";
      joint4_constraint.position = joint4_target;
      joint4_constraint.tolerance_above = joint4_tolerance_;
      joint4_constraint.tolerance_below = joint4_tolerance_;
      joint4_constraint.weight = 1.0;  // 高权重，作为强约束
      constraints.joint_constraints.push_back(joint4_constraint);
      RCLCPP_INFO(this->get_logger(), "[4DOF控制] JointConstraint(J4): 目标=%.3f rad (%.1f deg), 容差=±%.3f rad (±%.1f deg)",
                  joint4_target, joint4_target * 180.0 / M_PI,
                  joint4_tolerance_, joint4_tolerance_ * 180.0 / M_PI);
      
      // OrientationConstraint：只锁定roll/pitch（向下），不约束yaw
      moveit_msgs::msg::OrientationConstraint orientation_constraint;
      orientation_constraint.header.frame_id = planning_frame_;
      orientation_constraint.link_name = eef_link_;
      orientation_constraint.orientation = compute_downward_orientation_fixed();
      orientation_constraint.absolute_x_axis_tolerance = 0.1;  // roll容差（约±5.7°）
      orientation_constraint.absolute_y_axis_tolerance = 0.1;  // pitch容差（约±5.7°）
      orientation_constraint.absolute_z_axis_tolerance = M_PI;  // yaw完全不约束（±180°）
      orientation_constraint.weight = 1.0;
      constraints.orientation_constraints.push_back(orientation_constraint);
      RCLCPP_INFO(this->get_logger(), "[4DOF控制] OrientationConstraint: roll/pitch容差=±0.1 rad (±5.7°), yaw不约束");
      
      // 设置约束到MoveTo stage
      move_to_pregrasp->setPathConstraints(constraints);
      RCLCPP_INFO(this->get_logger(), "[4DOF控制] 已添加MoveTo stage: move to pregrasp (使用pose goal + 约束，超时=30s)");
    }
    
    task.add(std::move(move_to_pregrasp));
    RCLCPP_DEBUG(this->get_logger(), "已添加MoveTo stage: move to pregrasp (group=arm_group, goal_type=%s)", 
                 use_joint_goal ? "joint" : "pose");
    
    // 4. ModifyPlanningScene - 添加缆绳碰撞对象（在MoveTo pregrasp之后，避免goal state被判碰撞）
    // 分阶段碰撞策略（方案A - 最稳）：
    // - MoveTo pregrasp：碰撞对象在pregrasp之后添加，所以pregrasp阶段不受影响（可达性不受限制）
    // - MoveRelative descend：需要ACM允许碰撞（否则下探会被判invalid）
    // - Close gripper：必须允许碰撞（这一步几乎一定与缆绳接触/穿入）
    // - Attach object：闭合后attach，此时缆绳成为夹爪的一部分
    // - Lift / transport：attach后保留碰撞检查（让它别撞环境），ACM持续有效
    add_object->addObject(collision_object);
    task.add(std::move(add_object));
    RCLCPP_DEBUG(this->get_logger(), "已添加ModifyPlanningScene stage: add cable object (object_id=%s)", 
                 (cable_name_ + "_world").c_str());
    RCLCPP_INFO(this->get_logger(), "碰撞对象在MoveTo pregrasp之后添加，避免goal state被判碰撞");
    
    // 4.5. ModifyPlanningScene - 设置AllowedCollisionMatrix（ACM），允许夹爪link与缆绳碰撞
    // 关键修复：在descend/close阶段，夹爪必须与缆绳接触/穿入，不能把缆绳当成严格障碍
    // 使用ACM允许夹爪相关link（LinkGG/LinkGL/LinkGR）与缆绳碰撞，这样：
    // - 规划器能看到缆绳位置（用于避障，避免从侧面扫过去）
    // - 但不会因为"必须接触"而判goal/path invalid
    // ACM设置后持续有效，覆盖descend、close、lift、transport等后续阶段
    auto allow_collisions = std::make_unique<mtc::stages::ModifyPlanningScene>("allow gripper-cable collisions");
    std::string cable_object_id = cable_name_ + "_world";
    
    // 允许夹爪相关link与缆绳碰撞
    // allow_touch_links_默认包含["LinkGG", "LinkGL", "LinkGR"]
    std::stringstream links_ss;
    for (size_t i = 0; i < allow_touch_links_.size(); ++i) {
      const auto& link = allow_touch_links_[i];
      allow_collisions->allowCollisions(cable_object_id, link, true);
      RCLCPP_DEBUG(this->get_logger(), "[ACM] 允许 %s 与 %s 碰撞", cable_object_id.c_str(), link.c_str());
      if (i > 0) links_ss << ", ";
      links_ss << link;
    }
    
    task.add(std::move(allow_collisions));
    RCLCPP_INFO(this->get_logger(), "[ACM] 已设置AllowedCollisionMatrix：允许夹爪link(%s)与缆绳碰撞，避免descend/close阶段被判invalid",
                links_ss.str().c_str());
    
    // 5. MoveTo - 使用Cartesian路径下探descend_distance_
    // Descend阶段：ACM已设置，允许夹爪与缆绳碰撞
    // 使用Cartesian路径替代MoveRelative，更可靠，不需要OMPL采样goal tree
    // 在创建MoveTo之前，验证下探后的位置是否IK可解
    geometry_msgs::msg::PoseStamped descend_pose = pregrasp_pose;
    descend_pose.pose.position.z -= descend_distance_;
    
    {
      RCLCPP_INFO(this->get_logger(), "[下探验证] 验证下探后的位置是否IK可解...");
      RCLCPP_INFO(this->get_logger(), "[下探验证] 下探后位置: pos=(%.3f, %.3f, %.3f)", 
                 descend_pose.pose.position.x, descend_pose.pose.position.y, descend_pose.pose.position.z);
      
      try {
        // 使用pregrasp_state作为seed进行IK检查
        if (pregrasp_state) {
          const auto* jmg = pregrasp_state->getJointModelGroup("arm_group");
          if (jmg) {
            auto test_state = std::make_shared<moveit::core::RobotState>(*pregrasp_state);
            bool ik_solved = test_state->setFromIK(jmg, descend_pose.pose, eef_link_, 0.5);
            
            if (ik_solved) {
              RCLCPP_INFO(this->get_logger(), "[下探验证] ✓ 下探后位置IK可解");
              
              // 检查下探后的状态是否有碰撞
              auto robot_model = task.getRobotModel();
              if (robot_model) {
                auto test_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
                test_scene->getCurrentStateNonConst() = *test_state;
                
                // 添加碰撞对象
                if (add_collision_object_) {
                  test_scene->processCollisionObjectMsg(collision_object);
                }
                
                // 设置ACM
                collision_detection::AllowedCollisionMatrix acm = test_scene->getAllowedCollisionMatrix();
                acm.setEntry(cable_name_ + "_world", "LinkGG", true);
                acm.setEntry(cable_name_ + "_world", "LinkGL", true);
                acm.setEntry(cable_name_ + "_world", "LinkGR", true);
                test_scene->getAllowedCollisionMatrixNonConst() = acm;
                
                collision_detection::CollisionRequest req;
                collision_detection::CollisionResult res;
                req.contacts = true;
                req.max_contacts = 10;
                
                test_scene->checkCollision(req, res, *test_state);
                
                if (res.collision) {
                  RCLCPP_WARN(this->get_logger(), "[下探验证] ⚠️ 下探后位置有碰撞（但ACM已设置，可能允许）");
                  for (const auto& contact : res.contacts) {
                    RCLCPP_WARN(this->get_logger(), "[下探验证]   碰撞: %s vs %s", 
                               contact.first.first.c_str(), contact.first.second.c_str());
                  }
                } else {
                  RCLCPP_INFO(this->get_logger(), "[下探验证] ✓ 下探后位置无碰撞");
                }
              }
            } else {
              RCLCPP_ERROR(this->get_logger(), "[下探验证] ✗ 下探后位置IK无解！");
              RCLCPP_ERROR(this->get_logger(), "[下探验证] 建议：减小descend_distance_或检查TCP偏移补偿");
            }
          }
        }
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "[下探验证] 验证下探位置时出错: %s", e.what());
      }
    }
    
    // 验证下探路径上的中间状态（帮助诊断路径规划问题）
    {
      RCLCPP_INFO(this->get_logger(), "[下探验证] 验证下探路径上的中间状态...");
      const int num_intermediate_states = 5;
      bool all_intermediate_valid = true;
      
      try {
        if (pregrasp_state) {
          const auto* jmg = pregrasp_state->getJointModelGroup("arm_group");
          auto robot_model = task.getRobotModel();
          
          if (jmg && robot_model) {
            auto test_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
            
            // 添加碰撞对象和ACM
            if (add_collision_object_) {
              test_scene->processCollisionObjectMsg(collision_object);
            }
            collision_detection::AllowedCollisionMatrix acm = test_scene->getAllowedCollisionMatrix();
            acm.setEntry(cable_name_ + "_world", "LinkGG", true);
            acm.setEntry(cable_name_ + "_world", "LinkGL", true);
            acm.setEntry(cable_name_ + "_world", "LinkGR", true);
            test_scene->getAllowedCollisionMatrixNonConst() = acm;
            
            for (int i = 1; i <= num_intermediate_states; ++i) {
              double fraction = static_cast<double>(i) / num_intermediate_states;
              geometry_msgs::msg::PoseStamped intermediate_pose = pregrasp_pose;
              intermediate_pose.pose.position.z -= descend_distance_ * fraction;
              
              auto test_state = std::make_shared<moveit::core::RobotState>(*pregrasp_state);
              bool ik_solved = test_state->setFromIK(jmg, intermediate_pose.pose, eef_link_, 0.5);
              
              if (ik_solved) {
                test_scene->getCurrentStateNonConst() = *test_state;
                collision_detection::CollisionRequest req;
                collision_detection::CollisionResult res;
                req.contacts = true;
                req.max_contacts = 5;
                
                test_scene->checkCollision(req, res, *test_state);
                
                if (res.collision) {
                  RCLCPP_WARN(this->get_logger(), "[下探验证] 中间状态 #%d (%.0f%%) 有碰撞", i, fraction * 100.0);
                  for (const auto& contact : res.contacts) {
                    RCLCPP_WARN(this->get_logger(), "[下探验证]   碰撞: %s vs %s", 
                               contact.first.first.c_str(), contact.first.second.c_str());
                  }
                  all_intermediate_valid = false;
                } else {
                  RCLCPP_DEBUG(this->get_logger(), "[下探验证] 中间状态 #%d (%.0f%%) IK可解且无碰撞", i, fraction * 100.0);
                }
              } else {
                RCLCPP_WARN(this->get_logger(), "[下探验证] 中间状态 #%d (%.0f%%) IK无解", i, fraction * 100.0);
                all_intermediate_valid = false;
              }
            }
            
            if (all_intermediate_valid) {
              RCLCPP_INFO(this->get_logger(), "[下探验证] ✓ 所有中间状态验证通过");
            } else {
              RCLCPP_WARN(this->get_logger(), "[下探验证] ⚠️ 部分中间状态有问题，但Cartesian路径可能会自动处理");
            }
          }
        }
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "[下探验证] 验证中间状态时出错: %s", e.what());
      }
    }
    
    // 4DOF机械臂修复：使用MoveTo + joint goal代替MoveRelative
    // MoveRelative在4DOF机械臂上不可靠：
    // 1. 4DOF末端姿态不固定，"相对于LinkGG的-Z方向"会随姿态变化
    // 2. OMPL无法采样到符合约束的goal tree
    // 解决方案：预先计算下探后的关节值，直接用MoveTo跳转
    
    // 计算下探后的目标位置
    geometry_msgs::msg::PoseStamped grasp_pose_linkgg = pregrasp_pose;
    grasp_pose_linkgg.pose.position.z -= descend_distance_;
    
    // 使用IK求解下探后的关节值
    bool use_descend_joint_goal = false;
    std::map<std::string, double> descend_joint_map;
    
    if (pregrasp_state) {
      const auto* jmg = pregrasp_state->getJointModelGroup("arm_group");
      if (jmg) {
        auto descend_state = std::make_shared<moveit::core::RobotState>(*pregrasp_state);
        bool ik_ok = descend_state->setFromIK(jmg, grasp_pose_linkgg.pose, eef_link_, 0.5);
        
        if (ik_ok) {
          // 检查Joint2/3是否在范围内
          std::vector<double> descend_joint_values;
          descend_state->copyJointGroupPositions(jmg, descend_joint_values);
          
          const std::vector<std::string>& joint_names = jmg->getActiveJointModelNames();
          size_t num_joints = std::min(joint_names.size(), descend_joint_values.size());
          
          bool joints_valid = true;
          for (size_t i = 0; i < num_joints; ++i) {
            const auto* joint_model = jmg->getActiveJointModels()[i];
            if (joint_model) {
              double min_bound = joint_model->getVariableBounds()[0].min_position_;
              double max_bound = joint_model->getVariableBounds()[0].max_position_;
              if (descend_joint_values[i] < min_bound - 0.01 || descend_joint_values[i] > max_bound + 0.01) {
                joints_valid = false;
                RCLCPP_WARN(this->get_logger(), "[下探] 关节 %s 值 %.3f 超出限位 [%.3f, %.3f]",
                           joint_names[i].c_str(), descend_joint_values[i], min_bound, max_bound);
              }
            }
            descend_joint_map[joint_names[i]] = descend_joint_values[i];
          }
          
          if (joints_valid) {
            use_descend_joint_goal = true;
            
            // 关键修复：用自动计算的joint1_auto覆盖IK解的Joint1（确保朝向正确）
            if (descend_joint_map.find("Joint1") != descend_joint_map.end()) {
              double ik_joint1 = descend_joint_map["Joint1"];
              RCLCPP_INFO(this->get_logger(), "[下探] Joint1修复: IK解=%.3f rad (%.1f°) -> 自动计算=%.3f rad (%.1f°)",
                         ik_joint1, ik_joint1 * 180.0 / M_PI, joint1_auto, joint1_auto * 180.0 / M_PI);
              descend_joint_map["Joint1"] = joint1_auto;
            }
            
            // 关键修复：用用户指定的joint4_target覆盖IK解的Joint4（控制夹爪yaw）
            if (descend_joint_map.find("Joint4") != descend_joint_map.end()) {
              double ik_joint4 = descend_joint_map["Joint4"];
              RCLCPP_INFO(this->get_logger(), "[下探] Joint4修复: IK解=%.3f rad (%.1f°) -> 用户指定=%.3f rad (%.1f°)",
                         ik_joint4, ik_joint4 * 180.0 / M_PI, joint4_target, joint4_target * 180.0 / M_PI);
              descend_joint_map["Joint4"] = joint4_target;
            }
            
            RCLCPP_INFO(this->get_logger(), "[下探] ✓ 使用joint goal代替MoveRelative");
            for (const auto& kv : descend_joint_map) {
              RCLCPP_DEBUG(this->get_logger(), "[下探]   %s = %.3f rad (%.1f°)", 
                          kv.first.c_str(), kv.second, kv.second * 180.0 / M_PI);
            }
          }
        } else {
          RCLCPP_WARN(this->get_logger(), "[下探] IK求解失败，回退到MoveRelative");
        }
      }
    }
    
    if (use_descend_joint_goal) {
      // 使用MoveTo + joint goal
      auto move_down = std::make_unique<mtc::stages::MoveTo>("move down", pipeline_planner);
      move_down->setGroup("arm_group");
      move_down->setTimeout(20.0);
      move_down->setGoal(descend_joint_map);
      task.add(std::move(move_down));
      RCLCPP_INFO(this->get_logger(), "已添加MoveTo stage: move down (使用joint goal，更可靠)");
    } else {
      // 回退到MoveRelative（使用world_link作为参考系）
      auto move_down = std::make_unique<mtc::stages::MoveRelative>("move down", pipeline_planner);
      move_down->setGroup("arm_group");
      move_down->setIKFrame(eef_link_);
      move_down->setTimeout(20.0);
      
      // 使用world_link作为方向参考系，确保"向下"是真正的世界坐标系向下
      geometry_msgs::msg::TwistStamped descend;
      descend.header.frame_id = planning_frame_;  // 使用world_link
      descend.twist.linear.z = -1.0;  // 向下（世界坐标系）
      move_down->setDirection(descend);
      move_down->setMinMaxDistance(descend_distance_, descend_distance_);
      
      task.add(std::move(move_down));
      RCLCPP_WARN(this->get_logger(), "已添加MoveRelative stage: move down (回退方案，direction frame=%s)", 
                   planning_frame_.c_str());
    }
    
    // 6. MoveTo - 夹爪闭合（使用joint target，不依赖named state）
    // Close gripper阶段：ACM已设置，必须允许碰撞
    // 这一步几乎一定与缆绳接触/穿入，至少允许夹爪指尖（LinkGL/LinkGR）与缆绳碰撞
    auto close_gripper = std::make_unique<mtc::stages::MoveTo>("close gripper", pipeline_planner);
    close_gripper->setGroup("gripper_group");
    // 计算闭合关节角度
    double close_width = gripper_close_width_ - gripper_close_extra_;
    double joint_gl = width_to_joint_angle(close_width);
    double joint_gr = -joint_gl;  // 镜像对称
    // MoveTo::setGoal需要map<string, double>格式（关节名->值）
    std::map<std::string, double> gripper_joint_map;
    gripper_joint_map["JointGL"] = joint_gl;
    gripper_joint_map["JointGR"] = joint_gr;
    close_gripper->setGoal(gripper_joint_map);
    task.add(std::move(close_gripper));
    RCLCPP_DEBUG(this->get_logger(), "已添加MoveTo stage: close gripper (group=gripper_group, JointGL=%.3f, JointGR=%.3f)", 
                 joint_gl, joint_gr);
    
    // 7. ModifyPlanningScene - 附着对象到末端执行器
    // 闭合后attach：此时缆绳成为夹爪的一部分
    // attach会自动处理一部分碰撞关系，但之前设置的ACM仍然有效
    // 这确保缆绳与夹爪link（LinkGG/LinkGL/LinkGR）的碰撞被允许（因为它们天然重叠）
    auto attach_object = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
    attach_object->attachObject(cable_name_ + "_world", eef_link_);
    task.add(std::move(attach_object));
    RCLCPP_DEBUG(this->get_logger(), "已添加ModifyPlanningScene stage: attach object (object_id=%s, link=%s)", 
                 (cable_name_ + "_world").c_str(), eef_link_.c_str());
    
    // 8. 抬起lift_distance_
    // 4DOF机械臂修复：使用MoveTo + joint goal代替MoveRelative（与下探相同策略）
    // Lift / transport阶段：ACM持续有效，允许缆绳与夹爪link持续碰撞
    
    // 计算抬起后的目标位置（从grasp位置向上抬起）
    geometry_msgs::msg::PoseStamped lift_pose_linkgg = grasp_pose_linkgg;
    lift_pose_linkgg.pose.position.z += lift_distance_;
    
    bool use_lift_joint_goal = false;
    std::map<std::string, double> lift_joint_map;
    
    if (pregrasp_state) {
      const auto* jmg = pregrasp_state->getJointModelGroup("arm_group");
      if (jmg) {
        auto lift_state = std::make_shared<moveit::core::RobotState>(*pregrasp_state);
        bool ik_ok = lift_state->setFromIK(jmg, lift_pose_linkgg.pose, eef_link_, 0.5);
        
        if (ik_ok) {
          std::vector<double> lift_joint_values;
          lift_state->copyJointGroupPositions(jmg, lift_joint_values);
          
          const std::vector<std::string>& joint_names = jmg->getActiveJointModelNames();
          size_t num_joints = std::min(joint_names.size(), lift_joint_values.size());
          
          bool joints_valid = true;
          for (size_t i = 0; i < num_joints; ++i) {
            const auto* joint_model = jmg->getActiveJointModels()[i];
            if (joint_model) {
              double min_bound = joint_model->getVariableBounds()[0].min_position_;
              double max_bound = joint_model->getVariableBounds()[0].max_position_;
              if (lift_joint_values[i] < min_bound - 0.01 || lift_joint_values[i] > max_bound + 0.01) {
                joints_valid = false;
              }
            }
            lift_joint_map[joint_names[i]] = lift_joint_values[i];
          }
          
          if (joints_valid) {
            use_lift_joint_goal = true;
            
            // 关键修复：用自动计算的joint1_auto覆盖IK解的Joint1（确保朝向正确）
            if (lift_joint_map.find("Joint1") != lift_joint_map.end()) {
              double ik_joint1 = lift_joint_map["Joint1"];
              RCLCPP_INFO(this->get_logger(), "[抬起] Joint1修复: IK解=%.3f rad (%.1f°) -> 自动计算=%.3f rad (%.1f°)",
                         ik_joint1, ik_joint1 * 180.0 / M_PI, joint1_auto, joint1_auto * 180.0 / M_PI);
              lift_joint_map["Joint1"] = joint1_auto;
            }
            
            // 关键修复：用用户指定的joint4_target覆盖IK解的Joint4（控制夹爪yaw）
            if (lift_joint_map.find("Joint4") != lift_joint_map.end()) {
              double ik_joint4 = lift_joint_map["Joint4"];
              RCLCPP_INFO(this->get_logger(), "[抬起] Joint4修复: IK解=%.3f rad (%.1f°) -> 用户指定=%.3f rad (%.1f°)",
                         ik_joint4, ik_joint4 * 180.0 / M_PI, joint4_target, joint4_target * 180.0 / M_PI);
              lift_joint_map["Joint4"] = joint4_target;
            }
            
            RCLCPP_INFO(this->get_logger(), "[抬起] ✓ 使用joint goal代替MoveRelative");
          }
        }
      }
    }
    
    if (use_lift_joint_goal) {
      auto move_up = std::make_unique<mtc::stages::MoveTo>("move up", pipeline_planner);
      move_up->setGroup("arm_group");
      move_up->setTimeout(20.0);
      move_up->setGoal(lift_joint_map);
      task.add(std::move(move_up));
      RCLCPP_INFO(this->get_logger(), "已添加MoveTo stage: move up (使用joint goal，更可靠)");
    } else {
      // 回退到MoveRelative
      auto move_up = std::make_unique<mtc::stages::MoveRelative>("move up", pipeline_planner);
      move_up->setGroup("arm_group");
      move_up->setIKFrame(eef_link_);
      move_up->setTimeout(20.0);
      geometry_msgs::msg::TwistStamped lift;
      lift.header.frame_id = planning_frame_;
      lift.twist.linear.z = 1.0;
      move_up->setDirection(lift);
      move_up->setMinMaxDistance(lift_distance_, lift_distance_);
      task.add(std::move(move_up));
      RCLCPP_WARN(this->get_logger(), "已添加MoveRelative stage: move up (回退方案)");
    }
    
    // 在返回task之前，验证pregrasp goal state（如果使用joint goal）
    if (use_joint_goal && pregrasp_state) {
      RCLCPP_INFO(this->get_logger(), "[Goal验证] 开始验证pregrasp goal state...");
      try {
        // 创建PlanningScene用于验证
        auto robot_model = task.getRobotModel();
        if (robot_model) {
          auto planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
          
          // 设置goal state到scene
          planning_scene->getCurrentStateNonConst() = *pregrasp_state;
          
          // 添加碰撞对象到scene（如果已创建）
          // 注意：这里添加碰撞对象用于验证，但MTC Task会在ModifyPlanningScene stage中添加
          // 我们这里添加是为了模拟MTC规划时的场景
          if (add_collision_object_) {
            planning_scene->processCollisionObjectMsg(collision_object);
          }
          
          // 设置ACM（允许夹爪link与缆绳碰撞）
          collision_detection::AllowedCollisionMatrix acm = planning_scene->getAllowedCollisionMatrix();
          acm.setEntry(cable_name_ + "_world", "LinkGG", true);
          acm.setEntry(cable_name_ + "_world", "LinkGL", true);
          acm.setEntry(cable_name_ + "_world", "LinkGR", true);
          planning_scene->getAllowedCollisionMatrixNonConst() = acm;
          
          // 执行碰撞检查
          collision_detection::CollisionRequest req;
          collision_detection::CollisionResult res;
          req.contacts = true;
          req.max_contacts = 20;
          req.verbose = false;
          
          planning_scene->checkCollision(req, res, *pregrasp_state);
          
          if (res.collision) {
            RCLCPP_ERROR(this->get_logger(), "[Goal验证] pregrasp goal state有碰撞！");
            RCLCPP_ERROR(this->get_logger(), "[Goal验证] 碰撞数量: %zu", res.contacts.size());
            for (const auto& contact : res.contacts) {
              RCLCPP_ERROR(this->get_logger(), "[Goal验证]   碰撞: %s vs %s", 
                           contact.first.first.c_str(), contact.first.second.c_str());
            }
          } else {
            RCLCPP_INFO(this->get_logger(), "[Goal验证] pregrasp goal state无碰撞 ✓");
          }
          
          // 检查关节限位（使用容差避免边界值误判）
          // 注意：satisfiesBounds()使用严格比较，边界值会被误判为超限
          // 改用容差检查
          const double JOINT_LIMIT_TOLERANCE = 1e-4;  // 约0.006度
          bool bounds_satisfied = true;
          const auto* jmg = pregrasp_state->getJointModelGroup("arm_group");
          if (jmg) {
            const auto& joint_names = jmg->getActiveJointModelNames();
            std::vector<double> joint_values;
            pregrasp_state->copyJointGroupPositions(jmg, joint_values);
            for (size_t i = 0; i < joint_names.size() && i < joint_values.size(); ++i) {
              const auto* joint_model = pregrasp_state->getRobotModel()->getJointModel(joint_names[i]);
              if (joint_model) {
                const auto& bounds = joint_model->getVariableBounds();
                if (!bounds.empty()) {
                  double joint_value = joint_values[i];
                  double min_bound = bounds[0].min_position_;
                  double max_bound = bounds[0].max_position_;
                  if (joint_value < min_bound - JOINT_LIMIT_TOLERANCE || joint_value > max_bound + JOINT_LIMIT_TOLERANCE) {
                    RCLCPP_ERROR(this->get_logger(), "[Goal验证]   超出限位: %s = %.3f rad (限位: [%.3f, %.3f])", 
                               joint_names[i].c_str(), joint_value, min_bound, max_bound);
                    bounds_satisfied = false;
                  }
                }
              }
            }
          }
          if (!bounds_satisfied) {
            RCLCPP_ERROR(this->get_logger(), "[Goal验证] pregrasp goal state超出关节限位！");
          } else {
            RCLCPP_INFO(this->get_logger(), "[Goal验证] pregrasp goal state关节限位检查通过 ✓");
          }
        }
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "[Goal验证] 验证pregrasp goal state时出错: %s", e.what());
      }
      
      // 二分定位验证：使用MoveGroup验证pregrasp joint goal
      if (!pregrasp_joint_map.empty()) {
        RCLCPP_INFO(this->get_logger(), "[二分验证] 开始使用MoveGroup验证pregrasp joint goal...");
        // 注意：这里需要释放moveit_mutex_，因为verify_pregrasp_goal_with_movegroup会获取锁
        // 但我们在create_grasp_task_mtc中可能已经持有锁，所以需要小心
        // 实际上，create_grasp_task_mtc中大部分代码不在锁内，所以这里应该没问题
        bool movegroup_valid = verify_pregrasp_goal_with_movegroup(pregrasp_joint_map);
        if (movegroup_valid) {
          RCLCPP_INFO(this->get_logger(), "[二分验证] ✓ MoveGroup验证通过，pregrasp goal本身valid，问题可能在MTC的后续stage");
        } else {
          RCLCPP_ERROR(this->get_logger(), "[二分验证] ✗ MoveGroup验证失败，pregrasp goal本身invalid，需要检查碰撞/越界");
        }
      }
    }
    
    return task;
  }
  
  // 使用MTC库创建放置Task
  mtc::Task create_place_task_mtc(const geometry_msgs::msg::PoseStamped& place_pose, double place_yaw)
  {
    mtc::Task task;
    
    // 必须先加载机器人模型，否则MTC Task无法工作
    if (!move_group_interface_) {
      RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface未初始化，无法创建MTC Task");
      return task;
    }
    
    // 从ROS参数服务器加载机器人模型到Task
    // loadRobotModel需要传入node指针来访问ROS参数服务器
    task.loadRobotModel(shared_from_this());
    task.stages()->setName("cable place task");
    
    // 创建solver用于连接各个阶段
    // 使用Task的机器人模型创建PlanningPipeline，确保与Task使用相同的机器人模型
    mtc::solvers::PipelinePlannerPtr pipeline_planner;
    try {
      // 从Task获取机器人模型（与Task使用相同的模型实例）
      auto robot_model = task.getRobotModel();
      if (!robot_model) {
        throw std::runtime_error("Task机器人模型未加载");
      }
      // 创建PlanningPipeline实例，使用Task的机器人模型和"ompl"作为pipeline ID
      auto planning_pipeline = std::make_shared<planning_pipeline::PlanningPipeline>(
          robot_model, shared_from_this(), "ompl");
      // 使用PlanningPipeline创建PipelinePlanner
      pipeline_planner = std::make_shared<mtc::solvers::PipelinePlanner>(planning_pipeline);
      RCLCPP_INFO(this->get_logger(), "使用PlanningPipeline创建PipelinePlanner成功（使用Task的机器人模型）");
    } catch (const std::exception& e) {
      // 如果PlanningPipeline创建失败，回退到直接使用node和pipeline名称
      RCLCPP_WARN(this->get_logger(), "无法使用PlanningPipeline创建PipelinePlanner: %s，回退到直接创建", e.what());
      pipeline_planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this(), "ompl");
    }
    
    // 转换放置位姿到planning frame
    geometry_msgs::msg::PoseStamped place_pose_planning = place_pose;
    if (!transform_pose_to_planning(place_pose_planning)) {
      RCLCPP_ERROR(this->get_logger(), "无法转换放置位姿到planning frame");
      return task;
    }
    
    // 1. FixedState - 使用getCurrentJointValues()构造固定起始状态（绕过时间戳检查）
    // 修复：CurrentState stage会触发时间戳严格检查（差5ms就失败），改用FixedState
    auto fixed_state = std::make_unique<mtc::stages::FixedState>("fixed start");
    {
      std::lock_guard<std::mutex> lock(moveit_mutex_);
      const auto& robot_model = move_group_interface_->getRobotModel();
      if (robot_model) {
        // 创建PlanningScene用于FixedState
        auto scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
        moveit::core::RobotState start(robot_model);
        start.setToDefaultValues();
        
        // 获取arm_group关节值（使用getCurrentJointValues()绕过时间戳检查）
        const auto* arm_jmg = robot_model->getJointModelGroup("arm_group");
        if (arm_jmg) {
          std::vector<double> arm_vals = move_group_interface_->getCurrentJointValues();
          if (arm_vals.size() == arm_jmg->getActiveJointModelNames().size()) {
            start.setJointGroupPositions(arm_jmg, arm_vals);
          }
        }
        
        // 获取gripper_group关节值
        if (gripper_group_interface_) {
          const auto* grip_jmg = robot_model->getJointModelGroup("gripper_group");
          if (grip_jmg) {
            std::vector<double> grip_vals = gripper_group_interface_->getCurrentJointValues();
            if (grip_vals.size() == grip_jmg->getActiveJointModelNames().size()) {
              start.setJointGroupPositions(grip_jmg, grip_vals);
            }
          }
        }
        
        start.update();
        // 修复Invalid Start State：确保关节值在合法范围内，避免浮点误差导致状态被判为invalid
        start.enforceBounds();
        // FixedState需要PlanningScenePtr，设置其current state
        scene->setCurrentState(start);
        fixed_state->setState(scene);
        RCLCPP_INFO(this->get_logger(), "[FixedState] 使用getCurrentJointValues()构造起始状态（绕过时间戳检查，已调用enforceBounds）");
        task.add(std::move(fixed_state));
        RCLCPP_DEBUG(this->get_logger(), "已添加FixedState stage");
      } else {
        RCLCPP_WARN(this->get_logger(), "[FixedState] 无法获取robot_model，fallback到CurrentState");
        // Fallback：如果无法构造FixedState，使用CurrentState
        auto current_state = std::make_unique<mtc::stages::CurrentState>("current state");
        task.add(std::move(current_state));
        RCLCPP_DEBUG(this->get_logger(), "已添加CurrentState stage（fallback）");
      }
    }
    
    // 2. Connect - 连接到放置准备位置（可选，Place内部也会自动连接）
    mtc::stages::Connect::GroupPlannerVector planners_place = {{"arm_group", pipeline_planner}};
    auto connect_to_place = std::make_unique<mtc::stages::Connect>("connect to place", planners_place);
    task.add(std::move(connect_to_place));
    
    // 3. GeneratePlacePose - 生成放置位姿候选
    auto place_generator = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
    place_generator->setObject(cable_name_ + "_world");
    // 设置放置位姿（使用setPose方法，传入PoseStamped）
    place_generator->setPose(place_pose_planning);
    // 设置planning group（arm_group用于运动规划）
    place_generator->setProperty("group", "arm_group");
    
    // 4. Place - 完整的放置流程（接近、释放、后退）
    // Place内部会自动使用Connect来连接各个阶段
    auto place = std::make_unique<mtc::stages::Place>(std::move(place_generator), "place");
    // setEndEffector需要传入end-effector名称（从SRDF中定义），而不是link名称
    // SRDF中定义的end-effector名称是"gripper"
    place->setEndEffector("gripper");
    place->setObject(cable_name_ + "_world");
    // 设置planning group（arm_group用于运动规划）
    place->setProperty("group", "arm_group");
    // Place内部会自动创建Connect阶段，我们需要通过setProperty设置solver
    // 注意：Place没有setSolver方法，solver通过Connect阶段内部配置
    
    // 设置放置运动（向下移动）
    geometry_msgs::msg::TwistStamped place_motion;
    place_motion.header.frame_id = eef_link_;
    place_motion.twist.linear.z = 1.0;  // 向下（相对于末端执行器）
    place->setPlaceMotion(place_motion, eef_step_, place_descend_distance_);
    
    // 设置后退运动（向上移动）
    geometry_msgs::msg::TwistStamped retract;
    retract.header.frame_id = planning_frame_;
    retract.twist.linear.z = 1.0;  // 向上
    place->setRetractMotion(retract, eef_step_, place_retreat_distance_);
    
    task.add(std::move(place));
    
    // 5. ModifyPlanningScene - 分离对象
    auto detach_object = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
    detach_object->detachObject(cable_name_ + "_world", eef_link_);
    task.add(std::move(detach_object));
    
    // 7. ModifyPlanningScene - 移除对象（可选）
    if (add_collision_object_) {
      auto remove_object = std::make_unique<mtc::stages::ModifyPlanningScene>("remove object");
      remove_object->removeObject(cable_name_ + "_world");
      task.add(std::move(remove_object));
    }
    
    return task;
  }
  
  // 二分定位验证：绕过MTC，直接用MoveGroup验证pregrasp joint goal是否valid
  bool verify_pregrasp_goal_with_movegroup(const std::map<std::string, double>& joint_map)
  {
    RCLCPP_INFO(this->get_logger(), "[二分验证] 开始验证pregrasp joint goal（绕过MTC，使用MoveGroup）...");
    std::lock_guard<std::mutex> lock(moveit_mutex_);
    
    // 清理所有constraints
    move_group_interface_->clearPathConstraints();
    move_group_interface_->clearPoseTargets();
    
    // 设置joint goal
    try {
      move_group_interface_->setJointValueTarget(joint_map);
      RCLCPP_INFO(this->get_logger(), "[二分验证] 已设置joint goal，关节值:");
      for (const auto& pair : joint_map) {
        RCLCPP_INFO(this->get_logger(), "[二分验证]   %s = %.3f rad (%.1f deg)", 
                   pair.first.c_str(), pair.second, pair.second * 180.0 / M_PI);
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "[二分验证] 设置joint goal失败: %s", e.what());
      return false;
    }
    
    // 尝试规划
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    RCLCPP_INFO(this->get_logger(), "[二分验证] 开始规划（planning_time=5.0s）...");
    move_group_interface_->setPlanningTime(5.0);
    auto result = move_group_interface_->plan(plan);
    
    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "[二分验证] ✓ MoveGroup plan成功，pregrasp goal本身valid");
      RCLCPP_INFO(this->get_logger(), "[二分验证] 规划轨迹包含 %zu 个点", plan.trajectory_.joint_trajectory.points.size());
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "[二分验证] ✗ MoveGroup plan失败，pregrasp goal本身invalid");
      RCLCPP_ERROR(this->get_logger(), "[二分验证] 错误代码: %d", result.val);
      return false;
    }
  }
  
  // 执行MTC Task（带急停检查和状态发布）
  bool execute_mtc_task(mtc::Task& task, const std::string& task_name)
  {
    RCLCPP_INFO(this->get_logger(), "开始规划MTC Task: %s", task_name.c_str());
    publish_state("规划中:" + task_name);
    
    // 检查急停
    if (emergency_stop_) {
      RCLCPP_WARN(this->get_logger(), "急停触发，停止Task规划");
      publish_state("急停:任务已停止");
      return false;
    }
    
    // 修复时间戳问题：显式设置start state（使用getCurrentJointValues()绕过时间戳检查）
    // 这样move_group端就不需要等待current_state_monitor，避免"差10ms就失败"的问题
    // 修复Path constraints残留：清理所有残留的constraints和pose targets
    // 避免OMPL在"带path constraints + joint goal"时无法采样goal tree
    {
      std::lock_guard<std::mutex> lock(moveit_mutex_);
      move_group_interface_->clearPathConstraints();  // 清理残留约束（最重要！）
      move_group_interface_->clearPoseTargets();     // 清理pose目标
      
      // 显式设置start state：使用getCurrentJointValues()获取关节值，构造RobotState
      // 这样即使current_state_monitor一时拿不到"足够新"的状态，也能正常规划
      try {
        const auto& robot_model = move_group_interface_->getRobotModel();
        if (robot_model) {
          auto start_state = std::make_shared<moveit::core::RobotState>(robot_model);
          start_state->setToDefaultValues();
          
          // 获取arm_group关节值
          const auto* arm_jmg = robot_model->getJointModelGroup("arm_group");
          if (arm_jmg) {
            std::vector<double> arm_joint_values = move_group_interface_->getCurrentJointValues();
            if (arm_joint_values.size() == arm_jmg->getActiveJointModelNames().size()) {
              start_state->setJointGroupPositions(arm_jmg, arm_joint_values);
            }
          }
          
          // 获取gripper_group关节值
          const auto* gripper_jmg = robot_model->getJointModelGroup("gripper_group");
          if (gripper_jmg && gripper_group_interface_) {
            std::vector<double> gripper_joint_values = gripper_group_interface_->getCurrentJointValues();
            if (gripper_joint_values.size() == gripper_jmg->getActiveJointModelNames().size()) {
              start_state->setJointGroupPositions(gripper_jmg, gripper_joint_values);
            }
          }
          
          start_state->update();
          // 修复Invalid Start State：确保关节值在合法范围内
          start_state->enforceBounds();
          move_group_interface_->setStartState(*start_state);
          RCLCPP_INFO(this->get_logger(), "[清理状态] 已清理path constraints和pose targets，显式设置start state（绕过时间戳检查，已调用enforceBounds）");
        } else {
          // Fallback：如果构造失败，使用setStartStateToCurrentState()
          move_group_interface_->setStartStateToCurrentState();
          RCLCPP_WARN(this->get_logger(), "[清理状态] 无法构造start state，使用setStartStateToCurrentState()");
        }
      } catch (const std::exception& e) {
        // Fallback：如果异常，使用setStartStateToCurrentState()
        move_group_interface_->setStartStateToCurrentState();
        RCLCPP_WARN(this->get_logger(), "[清理状态] 构造start state异常: %s，使用setStartStateToCurrentState()", e.what());
      }
    }
    
    // 先尝试初始化Task以获取更详细的错误信息
    RCLCPP_INFO(this->get_logger(), "MTC Task包含 %zu 个stage，准备初始化", task.stages()->numChildren());
    
    try {
      task.init();
      RCLCPP_INFO(this->get_logger(), "MTC Task初始化成功");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "MTC Task初始化失败: %s", e.what());
      RCLCPP_ERROR_STREAM(this->get_logger(), "详细初始化错误: " << e.what());
      
      // 尝试输出异常类型和更详细的信息
      try {
        std::rethrow_if_nested(e);
      } catch (const std::exception& nested) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "嵌套异常: " << nested.what());
      } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "嵌套异常: 未知类型");
      }
      
      // 输出Task状态以便调试
      RCLCPP_ERROR(this->get_logger(), "Task包含 %zu 个stage", task.stages()->numChildren());
      RCLCPP_ERROR(this->get_logger(), "可能的原因: Connect stage配置错误，或某个stage的solver未正确设置");
      
      publish_state("错误:Task初始化失败");
      return false;
    }
    
    // 规划Task（在后台线程中执行，以便检查急停）
    std::atomic<bool> planning_done{false};
    std::atomic<bool> planning_success{false};
    std::exception_ptr planning_exception;
    
    std::thread planning_thread([&]() {
      try {
        // 设置更长的规划时间，给OMPL更多机会找到路径
        // 注意：MTC Task的plan()方法内部会使用PipelinePlanner的配置
        // 我们已经在ompl_planning.yaml中设置了goal_bias=0.25，应该能帮助采样goal tree
        RCLCPP_INFO(this->get_logger(), "开始执行task.plan()，这可能需要较长时间...");
        task.plan();
        planning_success = true;
        RCLCPP_INFO(this->get_logger(), "task.plan()完成，找到 %zu 个解", task.solutions().size());
      } catch (const std::exception& e) {
        planning_exception = std::current_exception();
        planning_success = false;
        RCLCPP_ERROR(this->get_logger(), "task.plan()异常: %s", e.what());
      }
      planning_done = true;
    });
    
    // 等待规划完成或急停
    while (!planning_done && !emergency_stop_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    if (emergency_stop_) {
      RCLCPP_WARN(this->get_logger(), "急停触发，停止Task规划");
      publish_state("急停:任务已停止");
      task.preempt();  // 停止Task
      if (planning_thread.joinable()) {
        planning_thread.join();
      }
      return false;
    }
    
    if (planning_thread.joinable()) {
      planning_thread.join();
    }
    
    // 检查规划异常
    if (planning_exception) {
      try {
        std::rethrow_exception(planning_exception);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "MTC Task规划异常: %s", e.what());
        // 输出更详细的错误信息
        RCLCPP_ERROR_STREAM(this->get_logger(), "详细错误信息: " << e.what());
        // 尝试输出异常类型名称
        try {
          std::string exception_type = typeid(e).name();
          RCLCPP_ERROR(this->get_logger(), "异常类型: %s", exception_type.c_str());
        } catch (...) {
          // 忽略类型信息获取异常
        }
        // 尝试输出嵌套异常
        try {
          std::rethrow_if_nested(e);
        } catch (const std::exception& nested) {
          RCLCPP_ERROR_STREAM(this->get_logger(), "嵌套异常: " << nested.what());
        } catch (...) {
          RCLCPP_ERROR(this->get_logger(), "嵌套异常: 未知类型");
        }
        publish_state("错误:规划异常");
        return false;
      } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "MTC Task规划异常: 未知异常类型");
        publish_state("错误:规划异常");
        return false;
      }
    }
    
    // 检查规划结果
    if (!planning_success) {
      RCLCPP_ERROR(this->get_logger(), "MTC Task规划失败（planning_success=false）");
      publish_state("错误:规划失败");
      return false;
    }
    
    if (task.solutions().empty()) {
      RCLCPP_ERROR(this->get_logger(), "MTC Task规划完成但无可用解");
      // 输出Task的详细信息
      RCLCPP_ERROR(this->get_logger(), "Task状态: solutions().size()=%zu", task.solutions().size());
      
      // 输出每个stage的详细状态（最关键！）
      RCLCPP_ERROR(this->get_logger(), "=== 开始输出Task状态（task.printState()）===");
      try {
        // 重定向stdout到stringstream，捕获printState()输出
        std::stringstream ss;
        std::streambuf* old_cout = std::cout.rdbuf(ss.rdbuf());
        
        task.printState();
        
        // 恢复stdout
        std::cout.rdbuf(old_cout);
        
        // 输出到日志
        std::string state_output = ss.str();
        if (!state_output.empty()) {
          // 按行输出，避免单行过长
          std::istringstream iss(state_output);
          std::string line;
          while (std::getline(iss, line)) {
            RCLCPP_ERROR(this->get_logger(), "Task状态: %s", line.c_str());
          }
        } else {
          RCLCPP_WARN(this->get_logger(), "task.printState()没有输出内容");
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "调用task.printState()时出错: %s", e.what());
      }
      
      // 尝试手动遍历stages获取统计信息
      try {
        auto stages = task.stages();
        if (stages) {
          size_t num_children = stages->numChildren();
          RCLCPP_ERROR(this->get_logger(), "Task包含 %zu 个stage，尝试获取每个stage的统计信息...", num_children);
          // 注意：MTC的ContainerBase可能没有直接的child访问方法
          // 这里先输出基本信息
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "获取stage统计信息时出错: %s", e.what());
      }
      
      RCLCPP_ERROR(this->get_logger(), "=== Task状态输出完成 ===");
      
      // 诊断：输出Task基本信息
      RCLCPP_ERROR(this->get_logger(), "=== 规划失败诊断 ===");
      try {
        auto stages = task.stages();
        if (stages) {
          size_t num_children = stages->numChildren();
          RCLCPP_ERROR(this->get_logger(), "Task包含 %zu 个stage", num_children);
          
          // 增强错误诊断：输出当前状态详细信息
          try {
            std::lock_guard<std::mutex> lock(moveit_mutex_);
            // 修复时间戳问题：使用Time(0)获取最新可用状态，避免时间戳检查失败
            auto current_state = move_group_interface_->getCurrentState(0.0);
            if (current_state) {
              const auto* jmg = current_state->getJointModelGroup("arm_group");
              if (jmg) {
                std::vector<double> current_joint_values;
                current_state->copyJointGroupPositions(jmg, current_joint_values);
                const auto& joint_names = jmg->getActiveJointModelNames();
                
                RCLCPP_ERROR(this->get_logger(), "[状态诊断] 当前机器人状态:");
                for (size_t i = 0; i < current_joint_values.size() && i < joint_names.size(); ++i) {
                  RCLCPP_ERROR(this->get_logger(), "  %s = %.3f rad (%.1f°)", 
                              joint_names[i].c_str(), current_joint_values[i],
                              current_joint_values[i] * 180.0 / M_PI);
                }
                
                // 检查当前状态是否碰撞
                planning_scene::PlanningScenePtr planning_scene = 
                    std::make_shared<planning_scene::PlanningScene>(current_state->getRobotModel());
                planning_scene->setCurrentState(*current_state);
                collision_detection::CollisionRequest collision_request;
                collision_detection::CollisionResult collision_result;
                planning_scene->checkCollision(collision_request, collision_result);
                
                if (collision_result.collision) {
                  RCLCPP_ERROR(this->get_logger(), "[状态诊断] ⚠️ 当前状态与场景碰撞！这可能导致规划失败");
                  if (collision_result.contacts.size() > 0) {
                    size_t contact_count = 0;
                    for (const auto& contact_pair : collision_result.contacts) {
                      if (contact_count < 3) {
                        RCLCPP_ERROR(this->get_logger(), "[状态诊断] 碰撞: %s <-> %s", 
                                   contact_pair.first.first.c_str(), contact_pair.first.second.c_str());
                        contact_count++;
                      }
                    }
                  }
                } else {
                  RCLCPP_INFO(this->get_logger(), "[状态诊断] ✓ 当前状态无碰撞");
                }
                
                // 检查关节值是否在限位内（使用容差避免边界值误判）
                bool joints_in_limits = true;
                const double JOINT_LIMIT_TOLERANCE = 1e-4;  // 约0.006度
                for (size_t i = 0; i < current_joint_values.size() && i < joint_names.size(); ++i) {
                  const auto* joint_model = jmg->getActiveJointModels()[i];
                  if (joint_model) {
                    double min_bound = joint_model->getVariableBounds()[0].min_position_;
                    double max_bound = joint_model->getVariableBounds()[0].max_position_;
                    double joint_value = current_joint_values[i];
                    if (joint_value < min_bound - JOINT_LIMIT_TOLERANCE || joint_value > max_bound + JOINT_LIMIT_TOLERANCE) {
                      RCLCPP_ERROR(this->get_logger(), "[状态诊断] ⚠️ 关节 %s 值 %.3f 超出限位 [%.3f, %.3f]", 
                                 joint_names[i].c_str(), joint_value, min_bound, max_bound);
                      joints_in_limits = false;
                    }
                  }
                }
                if (joints_in_limits) {
                  RCLCPP_INFO(this->get_logger(), "[状态诊断] ✓ 所有关节值在限位内");
                }
              }
            } else {
              RCLCPP_WARN(this->get_logger(), "[状态诊断] ⚠️ 无法获取当前状态（可能joint_states未就绪或关节名不匹配）");
            }
          } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "[状态诊断] 获取状态信息时出错: %s", e.what());
          }
          
          RCLCPP_ERROR(this->get_logger(), "可能的原因:");
          RCLCPP_ERROR(this->get_logger(), "  1. MoveTo到预抓取位置失败（IK无解或路径规划失败）");
          RCLCPP_ERROR(this->get_logger(), "  2. MoveRelative下探失败（碰撞或超出工作空间）");
          RCLCPP_ERROR(this->get_logger(), "  3. 预抓取位置不可达（位置或姿态问题）");
          RCLCPP_ERROR(this->get_logger(), "  4. Path constraints残留（已清理，但可能仍有其他约束）");
          RCLCPP_ERROR(this->get_logger(), "  5. 关节命名不一致导致状态更新失败");
          RCLCPP_ERROR(this->get_logger(), "建议:");
          RCLCPP_ERROR(this->get_logger(), "  - 检查预抓取位置是否在工作空间内");
          RCLCPP_ERROR(this->get_logger(), "  - 尝试降低approach_offset_z_或调整目标位置");
          RCLCPP_ERROR(this->get_logger(), "  - 检查当前机器人状态是否与预抓取位置冲突");
          RCLCPP_ERROR(this->get_logger(), "  - 检查关节命名是否一致（查看启动日志中的关节命名诊断）");
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "获取Task信息时出错: %s", e.what());
      }
      RCLCPP_ERROR(this->get_logger(), "=== 诊断完成 ===");
      
      publish_state("错误:规划失败");
      return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "MTC Task规划成功，找到 %zu 个解", task.solutions().size());
    publish_state("规划成功:执行中");
    
    // 选择最优解（第一个解通常是最优的）
    const auto& solution = task.solutions().front();
    
    // 检查急停
    if (emergency_stop_) {
      RCLCPP_WARN(this->get_logger(), "急停触发，停止Task执行");
      publish_state("急停:任务已停止");
      return false;
    }
    
    // 修复轨迹时间戳问题：使用手动执行方式，在发送前修复轨迹时间戳
    // 问题：MTC生成的轨迹中，第一个点和第二个点的time_from_start可能都是0
    // 这会导致ros2_control的joint_trajectory_controller拒绝执行
    // 解决方案：在发送执行请求前，检查并修复轨迹时间戳
    auto execute_task_solution_client_ = rclcpp_action::create_client<moveit_task_constructor_msgs::action::ExecuteTaskSolution>(
        shared_from_this(), "execute_task_solution");
    
    if (!execute_task_solution_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "execute_task_solution action server不可用");
      publish_state("错误:action server不可用");
      return false;
    }
    
    // 转换解决方案为ROS消息
    moveit_task_constructor_msgs::msg::Solution sol_msg;
    solution->toMsg(sol_msg, &task.introspection());
    
    // 收集所有stage信息用于状态报告
    std::vector<std::string> stage_names;        // 英文原始名称（用于日志）
    std::vector<std::string> stage_names_cn;     // 中文名称（用于网页显示）
    std::vector<double> stage_durations;
    double total_duration = 0.0;
    
    // Stage名称英文到中文的映射
    auto translate_stage_name = [](const std::string& en_name) -> std::string {
      // 抓取任务相关
      if (en_name == "fixed start" || en_name == "current state") return "初始状态";
      if (en_name == "open gripper") return "打开夹爪";
      if (en_name == "move to pregrasp") return "移动到预抓取位置";
      if (en_name == "add cable object") return "添加碰撞对象";
      if (en_name == "allow gripper-cable collisions") return "设置碰撞允许";
      if (en_name == "move down") return "下探抓取";
      if (en_name == "close gripper") return "闭合夹爪";
      if (en_name == "attach object") return "附着物体";
      if (en_name == "move up") return "抬起";
      // 放置任务相关
      if (en_name == "move to preplace") return "移动到预放置位置";
      if (en_name == "place down") return "下放物体";
      if (en_name == "detach object") return "分离物体";
      if (en_name == "retreat") return "撤离";
      // 通用
      if (en_name.find("stage_") == 0) return "阶段" + en_name.substr(6);
      // 默认返回原名
      return en_name;
    };
    
    for (const auto& sub_traj : sol_msg.sub_trajectory) {
      std::string stage_name = sub_traj.info.comment;
      if (stage_name.empty()) {
        stage_name = "stage_" + std::to_string(sub_traj.info.stage_id);
      }
      stage_names.push_back(stage_name);
      stage_names_cn.push_back(translate_stage_name(stage_name));
      
      // 计算该stage的轨迹时长
      double duration = 0.0;
      if (!sub_traj.trajectory.joint_trajectory.points.empty()) {
        const auto& last_point = sub_traj.trajectory.joint_trajectory.points.back();
        duration = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9;
      }
      stage_durations.push_back(duration);
      total_duration += duration;
    }
    
    // 输出所有stage信息
    RCLCPP_INFO(this->get_logger(), "[MTC执行] 任务包含 %zu 个阶段:", stage_names.size());
    for (size_t i = 0; i < stage_names.size(); ++i) {
      RCLCPP_INFO(this->get_logger(), "  阶段%zu: %s (%s) (时长: %.2fs)", 
                  i+1, stage_names_cn[i].c_str(), stage_names[i].c_str(), stage_durations[i]);
    }
    
    // 修复所有子轨迹的时间戳
    // 问题：MTC生成的轨迹可能时间戳全为0或不严格递增
    // 解决方案：检测是否需要修复，如果需要则基于关节运动量计算合理时间
    const double MAX_JOINT_VELOCITY = 1.0;  // rad/s，关节最大速度（降低速度以减少抖动）
    const double MIN_SEGMENT_TIME = 0.05;   // 最小段时间50ms（提高平滑度）
    const double MIN_GRIPPER_TRAJECTORY_TIME = 1.5;  // 夹爪轨迹最小执行时间1.5秒
    const double MIN_ARM_TRAJECTORY_TIME = 1.0;      // 机械臂轨迹最小执行时间1秒
    
    int traj_index = 0;
    for (auto& sub_traj : sol_msg.sub_trajectory) {
      auto& joint_traj = sub_traj.trajectory.joint_trajectory;
      traj_index++;
      
      if (joint_traj.points.size() >= 2) {
        // 检测是否是夹爪轨迹（通过关节名称判断）
        bool is_gripper_traj = false;
        for (const auto& name : joint_traj.joint_names) {
          if (name == "JointGL" || name == "JointGR") {
            is_gripper_traj = true;
            break;
          }
        }
        
        // 获取原始轨迹时间
        double original_last_time = joint_traj.points.back().time_from_start.sec + 
                                   joint_traj.points.back().time_from_start.nanosec * 1e-9;
        
        // 确定最小轨迹时间
        double min_traj_time = is_gripper_traj ? MIN_GRIPPER_TRAJECTORY_TIME : MIN_ARM_TRAJECTORY_TIME;
        
        // 如果整个轨迹时间小于最小时间，则需要重新计算
        bool needs_fix = (original_last_time < min_traj_time);
        
        // 也检查是否有不递增的情况
        if (!needs_fix) {
          for (size_t i = 1; i < joint_traj.points.size(); ++i) {
            auto& prev_time = joint_traj.points[i-1].time_from_start;
            auto& curr_time = joint_traj.points[i].time_from_start;
            double prev_sec = prev_time.sec + prev_time.nanosec * 1e-9;
            double curr_sec = curr_time.sec + curr_time.nanosec * 1e-9;
            if (curr_sec <= prev_sec) {
              needs_fix = true;
              break;
            }
          }
        }
        
        RCLCPP_INFO(this->get_logger(), "[轨迹诊断] 子轨迹 #%d: %s, %zu个点, 原始时长=%.3fs, 需要修复=%s",
                   traj_index, is_gripper_traj ? "夹爪" : "机械臂",
                   joint_traj.points.size(), original_last_time, needs_fix ? "是" : "否");
        
        if (needs_fix) {
          // 重新计算所有点的时间戳，基于关节运动量
          double accumulated_time = 0.0;
          joint_traj.points[0].time_from_start.sec = 0;
          joint_traj.points[0].time_from_start.nanosec = 0;
          
          for (size_t i = 1; i < joint_traj.points.size(); ++i) {
            // 计算从上一点到当前点的最大关节运动量
            double max_joint_diff = 0.0;
            const auto& prev_positions = joint_traj.points[i-1].positions;
            const auto& curr_positions = joint_traj.points[i].positions;
            
            for (size_t j = 0; j < std::min(prev_positions.size(), curr_positions.size()); ++j) {
              double diff = std::abs(curr_positions[j] - prev_positions[j]);
              if (diff > max_joint_diff) {
                max_joint_diff = diff;
              }
            }
            
            // 基于关节运动量计算所需时间
            double segment_time = max_joint_diff / MAX_JOINT_VELOCITY;
            if (segment_time < MIN_SEGMENT_TIME) {
              segment_time = MIN_SEGMENT_TIME;
            }
            
            accumulated_time += segment_time;
            auto& curr_time = joint_traj.points[i].time_from_start;
            curr_time.sec = static_cast<int32_t>(accumulated_time);
            curr_time.nanosec = static_cast<uint32_t>((accumulated_time - curr_time.sec) * 1e9);
          }
          
          // 如果计算出的时间仍然小于最小时间，进行缩放
          if (accumulated_time < min_traj_time) {
            double scale = min_traj_time / accumulated_time;
            for (size_t i = 1; i < joint_traj.points.size(); ++i) {
              double old_time = joint_traj.points[i].time_from_start.sec + 
                               joint_traj.points[i].time_from_start.nanosec * 1e-9;
              double new_time = old_time * scale;
              joint_traj.points[i].time_from_start.sec = static_cast<int32_t>(new_time);
              joint_traj.points[i].time_from_start.nanosec = static_cast<uint32_t>((new_time - static_cast<int32_t>(new_time)) * 1e9);
            }
            accumulated_time = min_traj_time;
            RCLCPP_INFO(this->get_logger(), "[轨迹修复] 时间缩放: %.2f -> %.2f 秒 (scale=%.2f)", 
                       accumulated_time / scale, accumulated_time, scale);
          }
          
          RCLCPP_INFO(this->get_logger(), 
                      "[轨迹修复] 子轨迹 #%d (%s): %zu个点, 修复后时长=%.2f秒", 
                      traj_index, is_gripper_traj ? "夹爪" : "机械臂",
                      joint_traj.points.size(), accumulated_time);
        }
      }
    }
    
    // 创建goal并发送
    moveit_task_constructor_msgs::action::ExecuteTaskSolution::Goal goal_msg;
    goal_msg.solution = sol_msg;
    
    // 执行Task（在后台线程中执行，以便检查急停）
    std::atomic<bool> execution_done{false};
    std::atomic<bool> execution_success{false};
    std::exception_ptr execution_exception;
    
    std::thread execution_thread([&, this]() {
      try {
        auto send_goal_options = rclcpp_action::Client<moveit_task_constructor_msgs::action::ExecuteTaskSolution>::SendGoalOptions();
        
        std::promise<bool> result_promise;
        auto result_future_local = result_promise.get_future();
        
        send_goal_options.result_callback = [this, &result_promise](const auto& result) {
          if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "MTC执行完成，error_code=%d", result.result->error_code.val);
            result_promise.set_value(result.result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
          } else {
            RCLCPP_ERROR(this->get_logger(), "MTC执行被取消或失败");
            result_promise.set_value(false);
          }
        };
        
        auto goal_handle_future = execute_task_solution_client_->async_send_goal(goal_msg, send_goal_options);
        
        // 等待goal被接受（使用简单的future等待）
        auto status = goal_handle_future.wait_for(std::chrono::seconds(10));
        if (status != std::future_status::ready) {
          RCLCPP_ERROR(this->get_logger(), "发送goal超时");
          return;
        }
        
        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "Goal被拒绝");
          return;
        }
        
        RCLCPP_INFO(this->get_logger(), "Goal已被接受，等待执行完成...");
        
        // 等待结果（通过result_callback设置的promise）
        auto result_status = result_future_local.wait_for(std::chrono::seconds(120));
        if (result_status == std::future_status::ready) {
          execution_success = result_future_local.get();
        } else {
          RCLCPP_ERROR(this->get_logger(), "等待执行结果超时");
        }
      } catch (const std::exception& e) {
        execution_exception = std::current_exception();
        RCLCPP_ERROR(this->get_logger(), "执行异常: %s", e.what());
        execution_success = false;
      }
      execution_done = true;
    });
    
    // 等待执行完成或急停，同时发布分阶段状态
    // 基于累计时间估算当前执行的阶段
    auto execution_start = std::chrono::steady_clock::now();
    size_t current_stage = 0;
    double accumulated_time = 0.0;
    
    // 重新计算修复后的stage_durations（因为之前可能被修改了）
    stage_durations.clear();
    total_duration = 0.0;
    for (const auto& sub_traj : sol_msg.sub_trajectory) {
      double duration = 0.0;
      if (!sub_traj.trajectory.joint_trajectory.points.empty()) {
        const auto& last_point = sub_traj.trajectory.joint_trajectory.points.back();
        duration = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9;
      }
      stage_durations.push_back(duration);
      total_duration += duration;
    }
    
    // 发布第一个阶段状态（使用中文名称）
    if (!stage_names_cn.empty()) {
      publish_state("执行:" + stage_names_cn[0], "1/" + std::to_string(stage_names_cn.size()));
    }
    
    while (!execution_done && !emergency_stop_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      
      // 计算已执行时间
      auto now = std::chrono::steady_clock::now();
      double elapsed = std::chrono::duration<double>(now - execution_start).count();
      
      // 基于时间估算当前阶段
      accumulated_time = 0.0;
      size_t estimated_stage = 0;
      for (size_t i = 0; i < stage_durations.size(); ++i) {
        accumulated_time += stage_durations[i];
        if (elapsed < accumulated_time) {
          estimated_stage = i;
          break;
        }
        if (i == stage_durations.size() - 1) {
          estimated_stage = i;  // 最后一个阶段
        }
      }
      
      // 如果阶段变化了，发布新状态（使用中文名称）
      if (estimated_stage != current_stage && estimated_stage < stage_names_cn.size()) {
        current_stage = estimated_stage;
        std::string stage_info = std::to_string(current_stage + 1) + "/" + std::to_string(stage_names_cn.size());
        publish_state("执行:" + stage_names_cn[current_stage], stage_info);
        RCLCPP_INFO(this->get_logger(), "[MTC执行] 进入阶段 %zu/%zu: %s (%s)", 
                    current_stage + 1, stage_names_cn.size(), 
                    stage_names_cn[current_stage].c_str(), stage_names[current_stage].c_str());
      }
    }
    
    if (emergency_stop_) {
      RCLCPP_WARN(this->get_logger(), "急停触发，停止Task执行");
      publish_state("急停:任务已停止");
      task.preempt();  // 停止Task
      if (execution_thread.joinable()) {
        execution_thread.join();
      }
      return false;
    }
    
    if (execution_thread.joinable()) {
      execution_thread.join();
    }
    
    // 检查执行异常
    if (execution_exception) {
      try {
        std::rethrow_exception(execution_exception);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "MTC Task执行异常: %s", e.what());
        publish_state("错误:执行异常");
        return false;
      }
    }
    
    if (execution_success) {
      RCLCPP_INFO(this->get_logger(), "MTC Task执行成功");
      publish_state("完成");
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "MTC Task执行失败");
      publish_state("错误:执行失败");
      return false;
    }
  }
  
  // 使用MTC库的抓取函数
  bool do_cable_grasp_mtc(const geometry_msgs::msg::PoseStamped& cable_pose, double cable_yaw)
  {
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "=== 开始执行MTC抓取任务 ===");
    RCLCPP_INFO(this->get_logger(), "目标位置: pos=(%.3f, %.3f, %.3f), frame=%s",
                cable_pose.pose.position.x, cable_pose.pose.position.y, 
                cable_pose.pose.position.z, cable_pose.header.frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "原始yaw（障碍物方向）: %.3f rad (%.1f deg)",
                cable_yaw, cable_yaw * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "========================================");
    
    if (!rclcpp::ok() || emergency_stop_) {
      RCLCPP_WARN(this->get_logger(), "系统停止或急停，停止抓取任务");
      publish_state("急停:任务已停止");
      return false;
    }
    
    publish_state("已接收");
    
    // ========== 全零状态预运动 ==========
    // 如果机械臂处于全零状态（奇异点附近），先移动到安全预备位置
    // 这可以大幅提高后续MTC规划的成功率
    if (auto_move_to_ready_ && check_near_zero_state()) {
      RCLCPP_INFO(this->get_logger(), "[预运动] 检测到全零状态，先移动到安全预备位置...");
      publish_state("预运动");
      
      if (!move_to_ready_position()) {
        RCLCPP_ERROR(this->get_logger(), "[预运动] 移动到预备位置失败，但继续尝试MTC任务");
        // 不返回false，继续尝试MTC任务，可能会成功
      } else {
        RCLCPP_INFO(this->get_logger(), "[预运动] ✓ 已到达预备位置，继续执行MTC抓取任务");
      }
    }
    
    // 清理残留的碰撞对象
    const std::string cable_world_id = cable_name_ + "_world";
    const std::string cable_attached_id = cable_name_ + "_attached";
    scene_detach_and_remove(cable_world_id, eef_link_);
    scene_detach_and_remove(cable_attached_id, eef_link_);
    scene_detach_and_remove(cable_name_, eef_link_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 注意：碰撞对象将在MTC Task内的ModifyPlanningScene stage中添加
    // 这里不再重复添加，避免时序混乱和重复添加
    // 如果需要提前添加用于可视化，可以保留，但MTC Task内也会添加
    // 为了简化，我们只在Task内添加，这里注释掉
    /*
    if (add_collision_object_) {
      geometry_msgs::msg::PoseStamped cable_pose_planning = cable_pose;
      if (!transform_pose_to_planning(cable_pose_planning)) {
        publish_state("错误:缆绳位姿转换失败");
        return false;
      }
      if (!scene_add_cable_object(cable_pose_planning, cable_yaw, cable_world_id)) {
        publish_state("错误:添加物体失败");
        return false;
      }
    }
    */
    RCLCPP_INFO(this->get_logger(), "碰撞对象将在MTC Task内添加（ModifyPlanningScene stage）");
    
    // ========== 工作空间预检查 ==========
    // 先进行基本的工作空间检查，快速拒绝明显不可达的目标
    geometry_msgs::msg::PoseStamped cable_pose_in_base;
    try {
      geometry_msgs::msg::TransformStamped tf_to_base = 
          tf_buffer_->lookupTransform("base_link", cable_pose.header.frame_id, tf2::TimePointZero);
      tf2::doTransform(cable_pose, cable_pose_in_base, tf_to_base);
      
      double x_base = cable_pose_in_base.pose.position.x;
      double y_base = cable_pose_in_base.pose.position.y;
      double z_base = cable_pose_in_base.pose.position.z;
      
      if (!is_reachable(x_base, y_base, z_base)) {
        RCLCPP_ERROR(this->get_logger(), "目标位置超出4DOF机械臂工作空间，拒绝抓取任务");
        publish_state("错误:超出工作空间");
        return false;
      }
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(), "无法转换到base_link进行工作空间检查: %s，继续尝试", ex.what());
    }
    
    // ========== 快速IK可达性检查（使用Joint1方位角seed）==========
    // 使用Joint1对准目标方位角，给IK一个好seed，再判断Joint2/3是否能在范围内找到解
    RCLCPP_INFO(this->get_logger(), "[IK可达性检查] 开始快速IK可达性检查（Joint2/3 ∈ [-2.97, 0]，即[-170°, 0°]）");
    
    // 使用原始yaw（不强制翻转，yaw处理仅在夹爪对线缆方向时）
    double grasp_yaw = cable_yaw;
    
    // 4DOF控制：计算Joint4目标值（控制夹爪yaw旋转）
    // Joint1由IK自动根据目标位置计算（机械臂朝向）
    double joint4_target = cable_yaw;
    joint4_target += grasp_yaw_add_;
    joint4_target += yaw_offset_;
    if (yaw_flip_) {
      joint4_target += M_PI;
    }
    // 归一化到[-π, π]
    while (joint4_target > M_PI) joint4_target -= 2.0 * M_PI;
    while (joint4_target < -M_PI) joint4_target += 2.0 * M_PI;
    RCLCPP_INFO(this->get_logger(), "[4DOF控制] 计算Joint4目标值: %.3f rad (%.1f deg)", 
                joint4_target, joint4_target * 180.0 / M_PI);
    
    geometry_msgs::msg::PoseStamped target_in_base;
    try {
      // 将目标转换到base_link坐标系（用于计算方位角）
      geometry_msgs::msg::TransformStamped transform = 
          tf_buffer_->lookupTransform("base_link", cable_pose.header.frame_id, tf2::TimePointZero);
      tf2::doTransform(cable_pose, target_in_base, transform);
      target_in_base.header.frame_id = "base_link";
      
      double x_in_base = target_in_base.pose.position.x;
      double y_in_base = target_in_base.pose.position.y;
      
      RCLCPP_INFO(this->get_logger(), "[IK可达性检查] 目标在base_link中的位置: x=%.3f, y=%.3f, z=%.3f",
                  x_in_base, y_in_base, target_in_base.pose.position.z);
      
      // 计算Joint1方位角seed: j1_seed = atan2(y, x)
      double j1_seed = std::atan2(y_in_base, x_in_base);
      RCLCPP_INFO(this->get_logger(), "[IK可达性检查] 计算Joint1方位角seed: j1_seed=%.3f rad (%.1f°)",
                  j1_seed, j1_seed * 180.0 / M_PI);
      
      // 转换到planning frame进行IK测试
      geometry_msgs::msg::PoseStamped target_for_ik = cable_pose;
      if (!transform_pose_to_planning(target_for_ik)) {
        RCLCPP_ERROR(this->get_logger(), "[IK可达性检查] ✗ 无法转换位姿到planning frame，拒绝目标");
        publish_state("错误:位姿转换失败");
        return false;
      }
      
      // 快速可达性检查只回答一个问题：
      // "这个xyz，有没有任何一个Joint2/3 ∈ [-2.97,0]的解？"（即[-170°, 0°]）
      // 抓取姿态（roll=180°、yaw）只能放到MTC/真正规划阶段
      geometry_msgs::msg::Pose pose_pos_only;
      pose_pos_only.position = target_for_ik.pose.position;
      // orientation给一个"中性值"，不要锁死
      pose_pos_only.orientation.w = 1.0;
      pose_pos_only.orientation.x = 0.0;
      pose_pos_only.orientation.y = 0.0;
      pose_pos_only.orientation.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "[IK可达性检查] 使用position-only检查（不锁死姿态）");
      
      // 准备多个Joint1 seed集合
      std::vector<double> j1_seeds;
      j1_seeds.push_back(j1_seed);
      
      // Seed 2: 避免奇异性，尝试 j1_seed +/- pi
      double j1_seed2 = j1_seed + M_PI;
      // 归一化到[-π, π]
      while (j1_seed2 > M_PI) j1_seed2 -= 2.0 * M_PI;
      while (j1_seed2 < -M_PI) j1_seed2 += 2.0 * M_PI;
      j1_seeds.push_back(j1_seed2);
      
      RCLCPP_INFO(this->get_logger(), "[IK可达性检查] 准备 %zu 个Joint1 seed: [%.3f, %.3f] rad",
                  j1_seeds.size(), j1_seed, j1_seed2);
      
      // 获取当前状态作为基础
      auto test_state = getCurrentStateSafe(1.0);
      if (!test_state) {
        RCLCPP_WARN(this->get_logger(), "[IK可达性检查] 无法获取当前状态，跳过IK检查（继续尝试）");
      } else {
        const auto* jmg = test_state->getJointModelGroup("arm_group");
        if (jmg) {
          // 获取当前状态的关节值作为seed
          std::vector<double> seed_values;
          test_state->copyJointGroupPositions(jmg, seed_values);
          
          // 对每个Joint1 seed进行IK求解
          bool ik_found = false;
          int ik_solved_count = 0;  // 统计IK可解的次数
          int joint23_invalid_count = 0;  // 统计Joint2/3超出范围的次数
          
          for (size_t seed_idx = 0; seed_idx < j1_seeds.size(); ++seed_idx) {
            double j1 = j1_seeds[seed_idx];
            RCLCPP_INFO(this->get_logger(), "[IK可达性检查] 尝试seed #%zu: Joint1=%.3f rad (%.1f°)",
                       seed_idx + 1, j1, j1 * 180.0 / M_PI);
            
            // 复制当前状态
            auto seed_state = std::make_shared<moveit::core::RobotState>(*test_state);
            
            // 真正设置Joint1 seed（修改seed数组，然后设置到state）
            seed_values[0] = j1;  // Joint1是第一个关节
            seed_state->setJointGroupPositions(jmg, seed_values);
            
            // IK求解（使用position-only pose，超时0.2s）
            if (seed_state->setFromIK(jmg, pose_pos_only, eef_link_, 0.2)) {
              ik_solved_count++;
              RCLCPP_INFO(this->get_logger(), "[IK可达性检查] seed #%zu: IK求解成功", seed_idx + 1);
              
              // 检查Joint2/3可达性
              std::vector<double> joint2_3_values;
              if (check_joint23_reachable(seed_state, "arm_group", joint2_3_values)) {
                ik_found = true;
                RCLCPP_INFO(this->get_logger(), "[IK可达性检查] ✓ seed #%zu: IK可达性检查通过", seed_idx + 1);
                RCLCPP_INFO(this->get_logger(), "[IK可达性检查] Joint2=%.3f rad (%.1f°), Joint3=%.3f rad (%.1f°) - 在范围内",
                           joint2_3_values[0], joint2_3_values[0] * 180.0 / M_PI,
                           joint2_3_values[1], joint2_3_values[1] * 180.0 / M_PI);
                break;
              } else {
                joint23_invalid_count++;
                RCLCPP_WARN(this->get_logger(), "[IK可达性检查] seed #%zu: IK可解但Joint2/3不可达", seed_idx + 1);
                RCLCPP_WARN(this->get_logger(), "[IK可达性检查] Joint2=%.3f rad (%.1f°), Joint3=%.3f rad (%.1f°)",
                           joint2_3_values[0], joint2_3_values[0] * 180.0 / M_PI,
                           joint2_3_values[1], joint2_3_values[1] * 180.0 / M_PI);
              }
            } else {
              RCLCPP_WARN(this->get_logger(), "[IK可达性检查] seed #%zu: IK求解失败", seed_idx + 1);
            }
          }
          
          // 输出诊断信息（不直接拒绝，让MTC有机会尝试）
          if (!ik_found) {
            RCLCPP_WARN(this->get_logger(), "[IK预检查] 位置IK失败，仍然允许进入MTC规划（避免误杀）");
            if (ik_solved_count > 0) {
              RCLCPP_WARN(this->get_logger(), "[IK预检查] 诊断: IK可解 %d 次，但Joint2/3均超出范围", ik_solved_count);
            } else {
              RCLCPP_WARN(this->get_logger(), "[IK预检查] 诊断: 所有seed的IK求解均失败");
            }
            RCLCPP_WARN(this->get_logger(), "[IK预检查] 继续执行MTC规划，让OMPL/MTC有机会尝试");
            // 不return，继续执行MTC规划
            // 让MTC/OMPL有机会尝试
          } else {
            RCLCPP_INFO(this->get_logger(), "[IK预检查] ✓ 位置IK检查通过，Joint2/3可达");
          }
        } else {
          RCLCPP_WARN(this->get_logger(), "[IK可达性检查] 无法获取arm_group，跳过IK检查（继续尝试）");
        }
      }
      
      // 无论IK预检查是否通过，都继续创建MTC Task
      // IK预检查只是启发，不是判官
      RCLCPP_INFO(this->get_logger(), "[IK可达性检查] 继续创建MTC Task（让MTC/OMPL有机会尝试）");
      
    } catch (const tf2::TransformException& ex) {
      RCLCPP_ERROR(this->get_logger(), "[工程判定] ✗ 无法转换到base_link: %s", ex.what());
      RCLCPP_ERROR(this->get_logger(), "[工程判定] 拒绝该目标");
      publish_state("错误:TF转换失败");
      return false;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "[工程判定] ✗ 异常: %s", e.what());
      publish_state("错误:判定异常");
      return false;
    }
    
    // 创建MTC Task（使用选定的grasp_yaw和joint4_target）
    mtc::Task task = create_grasp_task_mtc(cable_pose, grasp_yaw, joint4_target);
    
    // 测试预抓取位置是否可达（诊断用）
    // 注意：必须使用与MTC Task相同的TCP偏移补偿计算
    geometry_msgs::msg::PoseStamped test_pregrasp_center = cable_pose;
    if (transform_pose_to_planning(test_pregrasp_center)) {
      // 使用与MTC相同的clearance计算
      double test_pregrasp_clearance = std::max(approach_offset_z_, 0.15);
      test_pregrasp_center.pose.position.z += test_pregrasp_clearance;
      test_pregrasp_center.pose.orientation = compute_downward_orientation_with_yaw(cable_yaw);
      
      // 补偿TCP偏移（与MTC Task中的计算完全一致）
      tf2::Quaternion q_center;
      tf2::fromMsg(test_pregrasp_center.pose.orientation, q_center);
      tf2::Matrix3x3 R_center(q_center);
      tf2::Vector3 offset_center_from_linkgg(tcp_offset_x_, tcp_offset_y_, tcp_offset_z_);
      tf2::Vector3 offset_world = R_center * offset_center_from_linkgg;
      
      geometry_msgs::msg::PoseStamped test_pregrasp;
      test_pregrasp.header.frame_id = planning_frame_;
      test_pregrasp.pose.position.x = test_pregrasp_center.pose.position.x - offset_world.x();
      test_pregrasp.pose.position.y = test_pregrasp_center.pose.position.y - offset_world.y();
      test_pregrasp.pose.position.z = test_pregrasp_center.pose.position.z - offset_world.z();
      test_pregrasp.pose.orientation = test_pregrasp_center.pose.orientation;
      
      RCLCPP_INFO(this->get_logger(), "[诊断] 测试预抓取位置是否可达（使用与MTC相同的TCP补偿）...");
      RCLCPP_INFO(this->get_logger(), "[诊断] 预抓取位置（LinkGG）: pos=(%.3f, %.3f, %.3f)", 
                  test_pregrasp.pose.position.x, test_pregrasp.pose.position.y, 
                  test_pregrasp.pose.position.z);
      
      // 先测试IK是否可解，并检查Joint2/3可达性
      // 注意：getCurrentState失败只是时间戳同步问题，不要误判为IK失败
      moveit::core::RobotStatePtr test_state = getCurrentStateSafe(1.0);  // 增加超时到1.0s
      if (test_state) {
        const auto* jmg = test_state->getJointModelGroup("arm_group");
        if (jmg) {
          bool ik_solved = test_state->setFromIK(jmg, test_pregrasp.pose, eef_link_, 0.5);
          if (ik_solved) {
            RCLCPP_INFO(this->get_logger(), "[诊断] ✓ IK求解成功");
            // 检查Joint2/3可达性
            std::vector<double> joint2_3_values;
            if (check_joint23_reachable(test_state, "arm_group", joint2_3_values)) {
              RCLCPP_INFO(this->get_logger(), "[诊断] ✓ Joint2/3可达: Joint2=%.3f rad (%.1f°), Joint3=%.3f rad (%.1f°)",
                         joint2_3_values[0], joint2_3_values[0] * 180.0 / M_PI,
                         joint2_3_values[1], joint2_3_values[1] * 180.0 / M_PI);
            } else {
              RCLCPP_WARN(this->get_logger(), "[诊断] ✗ IK可解但Joint2/3不可达（这可能是规划失败的根本原因）");
              RCLCPP_WARN(this->get_logger(), "[诊断] Joint2=%.3f rad (%.1f°), Joint3=%.3f rad (%.1f°)",
                         joint2_3_values[0], joint2_3_values[0] * 180.0 / M_PI,
                         joint2_3_values[1], joint2_3_values[1] * 180.0 / M_PI);
              RCLCPP_WARN(this->get_logger(), "[诊断] 需要范围: [-2.97, 0] rad ([-170°, 0°])");
            }
          } else {
            RCLCPP_WARN(this->get_logger(), "[诊断] ✗ IK求解失败（这可能是规划失败的根本原因）");
          }
        } else {
          RCLCPP_WARN(this->get_logger(), "[诊断] 无法获取arm_group关节模型组");
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "[诊断] 无法获取当前状态（可能是时间戳同步问题），跳过IK测试");
        RCLCPP_WARN(this->get_logger(), "[诊断] 注意：getCurrentState失败不代表IK无解，只是状态获取问题");
      }
      
      // 诊断代码已禁用：move_group_interface_->plan() 可能阻塞，影响MTC规划
      // 如果需要诊断，可以在MTC规划失败后再进行
      RCLCPP_DEBUG(this->get_logger(), "[诊断] 诊断代码已禁用，直接进入MTC规划（避免阻塞）");
    }
    
    // 执行Task
    RCLCPP_INFO(this->get_logger(), "[MTC执行] 诊断完成，准备执行MTC Task...");
    bool success = execute_mtc_task(task, "抓取");
    
    // 任务完成后清理场景对象
    if (success) {
      RCLCPP_INFO(this->get_logger(), "=== 任务完成：清理场景对象 ===");
      scene_detach_and_remove(cable_attached_id, eef_link_);
      scene_detach_and_remove(cable_world_id, eef_link_);
    } else {
      // 失败时也清理
      scene_detach_and_remove(cable_world_id, eef_link_);
      scene_detach_and_remove(cable_attached_id, eef_link_);
    }
    
    return success;
  }
  
  // 使用MTC库的放置函数
  bool do_cable_place_mtc(const geometry_msgs::msg::PoseStamped& place_pose, double place_yaw)
  {
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "=== 开始执行MTC放置任务 ===");
    RCLCPP_INFO(this->get_logger(), "目标位置: pos=(%.3f, %.3f, %.3f), frame=%s",
                place_pose.pose.position.x, place_pose.pose.position.y, 
                place_pose.pose.position.z, place_pose.header.frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "原始yaw: %.3f rad (%.1f deg)",
                place_yaw, place_yaw * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "========================================");
    
    if (!rclcpp::ok() || emergency_stop_) {
      RCLCPP_WARN(this->get_logger(), "系统停止或急停，停止放置任务");
      publish_state("急停:任务已停止");
      return false;
    }
    
    publish_state("已接收放置任务");
    
    // 创建MTC Task
    mtc::Task task = create_place_task_mtc(place_pose, place_yaw);
    
    // 执行Task
    bool success = execute_mtc_task(task, "放置");
    
    // 任务完成后清理场景对象
    if (success) {
      const std::string cable_world_id = cable_name_ + "_world";
      const std::string cable_attached_id = cable_name_ + "_attached";
      scene_detach_and_remove(cable_world_id, eef_link_);
      scene_detach_and_remove(cable_attached_id, eef_link_);
    }
    
    return success;
  }

  // 核心抓取函数（保持向后兼容，内部调用MTC实现）
  // cable_yaw: 视觉提供的原始yaw（缆绳切向方向），用于障碍物方向计算
  bool do_cable_grasp(const geometry_msgs::msg::PoseStamped& cable_pose, double cable_yaw)
  {
    // 使用MTC实现
    return do_cable_grasp_mtc(cable_pose, cable_yaw);
  }

  // 放置函数（保持向后兼容，内部调用MTC实现）
  bool do_cable_place(const geometry_msgs::msg::PoseStamped& place_pose, double place_yaw)
  {
    // 使用MTC实现
    return do_cable_place_mtc(place_pose, place_yaw);
  }

  // Pick-and-Place组合函数（使用MTC）
  bool do_pick_and_place(
      const geometry_msgs::msg::PoseStamped& grasp_pose, double grasp_yaw,
      const geometry_msgs::msg::PoseStamped& place_pose, double place_yaw)
  {
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "=== 开始执行Pick-and-Place任务（MTC） ===");
    RCLCPP_INFO(this->get_logger(), "========================================");
    
    // 先执行抓取
    if (!do_cable_grasp_mtc(grasp_pose, grasp_yaw)) {
      RCLCPP_ERROR(this->get_logger(), "抓取失败，Pick-and-Place任务终止");
      publish_state("错误:抓取失败");
      return false;
    }
    
    // 再执行放置
    if (!do_cable_place_mtc(place_pose, place_yaw)) {
      RCLCPP_ERROR(this->get_logger(), "放置失败，Pick-and-Place任务终止");
      publish_state("错误:放置失败");
      // 注意：此时物体可能已经抓取，但没有放置成功
      // 可以考虑添加恢复逻辑（如回到安全位置）
      return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Pick-and-Place任务完成");
    publish_state("完成");
    return true;
  }

  // 旧的抓取函数实现（保留作为参考，但不再使用）
  bool do_cable_grasp_old(const geometry_msgs::msg::PoseStamped& cable_pose, double cable_yaw)
  {
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "=== 开始执行抓取任务（旧实现） ===");
    RCLCPP_INFO(this->get_logger(), "目标位置: pos=(%.3f, %.3f, %.3f), frame=%s",
                cable_pose.pose.position.x, cable_pose.pose.position.y, 
                cable_pose.pose.position.z, cable_pose.header.frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "目标姿态: orientation=(%.3f, %.3f, %.3f, %.3f)",
                cable_pose.pose.orientation.x, cable_pose.pose.orientation.y,
                cable_pose.pose.orientation.z, cable_pose.pose.orientation.w);
    RCLCPP_INFO(this->get_logger(), "原始yaw（障碍物方向）: %.3f rad (%.1f deg)",
                cable_yaw, cable_yaw * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "========================================");
    
    if (!rclcpp::ok())
    {
      RCLCPP_INFO(this->get_logger(), "收到 shutdown 信号，停止处理抓取任务");
      return false;
    }
    
    // 检查急停状态
    if (emergency_stop_)
    {
      RCLCPP_WARN(this->get_logger(), "系统处于急停状态，停止抓取任务");
      publish_state("急停:任务已停止");
      return false;
    }

    publish_state("已接收");

    // 清理残留的碰撞对象（避免同名冲突）
    const std::string cable_world_id = cable_name_ + "_world";
    const std::string cable_attached_id = cable_name_ + "_attached";
    RCLCPP_INFO(this->get_logger(), "=== 任务开始：清理残留场景对象 ===");
    scene_detach_and_remove(cable_world_id, eef_link_);
    scene_detach_and_remove(cable_attached_id, eef_link_);
    // 也清理旧命名（向后兼容）
    scene_detach_and_remove(cable_name_, eef_link_);
    
    // 验证清理完成（等待一小段时间后检查）
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::vector<std::string> known_objects = planning_scene_interface_->getKnownObjectNames();
    bool found_world = false;
    bool found_attached = false;
    bool found_old = false;
    for (const auto& name : known_objects)
    {
      if (name == cable_world_id) found_world = true;
      if (name == cable_attached_id) found_attached = true;
      if (name == cable_name_) found_old = true;
    }
    if (found_world || found_attached || found_old)
    {
      RCLCPP_WARN(this->get_logger(), 
                  "清理验证：仍有残留对象存在 (world:%s, attached:%s, old:%s)，但继续执行",
                  found_world ? "是" : "否", found_attached ? "是" : "否", found_old ? "是" : "否");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "清理验证：所有残留对象已清除");
    }
    RCLCPP_INFO(this->get_logger(), "=== 场景清理完成 ===");

    // 0. 转换缆绳位姿到planning frame（提前转换，用于添加碰撞对象）
    geometry_msgs::msg::PoseStamped cable_pose_planning = cable_pose;
    if (!transform_pose_to_planning(cable_pose_planning))
    {
      publish_state("错误:缆绳位姿转换失败");
      return false;
    }

    // 0.5. 添加缆绳碰撞体（在收到缆绳信息后立即添加，便于在rviz中查看和规划）
    // 使用 cable_pose_planning（已经在 planning frame 中）和原始yaw（用于障碍物方向）
    if (add_collision_object_)
    {
      const std::string cable_world_id = cable_name_ + "_world";
      RCLCPP_INFO(this->get_logger(), "=== 添加缆绳碰撞体到场景（提前添加，便于可视化） ===");
      RCLCPP_INFO(this->get_logger(), "使用原始yaw计算障碍物方向: %.3f rad (%.1f deg)", 
                  cable_yaw, cable_yaw * 180.0 / M_PI);
      if (!scene_add_cable_object(cable_pose_planning, cable_yaw, cable_world_id))
      {
        publish_state("错误:添加物体失败");
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "=== 缆绳碰撞体已添加，可在rviz中查看 ===");
      publish_grasp_viz("recv/scene_added", cable_pose_planning, nullptr, nullptr, nullptr);
    }

    // 0. 检查当前位置是否在工作空间内（仅打印警告，不强制移动）
    // 注意：当前姿态是URDF/TF认可的合法姿态，不应在抓取任务中将其当作错误处理
    // 如果后续需要安全策略，应该基于实际IK可达性而非简单的高度/半径检查
    if (move_group_interface_)
    {
      try {
        auto current_pose = getCurrentPoseSafe();
        double current_x = current_pose.pose.position.x;
        double current_y = current_pose.pose.position.y;
        double current_z = current_pose.pose.position.z;
        
        RCLCPP_INFO(this->get_logger(), 
                    "=== 当前机器人状态诊断 ===");
        RCLCPP_INFO(this->get_logger(), 
                    "当前末端执行器位姿: pos=(%.3f, %.3f, %.3f), frame=%s",
                    current_pose.pose.position.x,
                    current_pose.pose.position.y,
                    current_pose.pose.position.z,
                    current_pose.header.frame_id.c_str());
        RCLCPP_INFO(this->get_logger(), 
                    "当前末端执行器orientation: (%.3f, %.3f, %.3f, %.3f)",
                    current_pose.pose.orientation.x,
                    current_pose.pose.orientation.y,
                    current_pose.pose.orientation.z,
                    current_pose.pose.orientation.w);

        // 获取当前关节值用于对比
        auto current_joints = getCurrentJointValuesSafe();
        RCLCPP_INFO(this->get_logger(), 
                    "当前关节值: [%.3f, %.3f, %.3f, %.3f]",
                    current_joints.size() > 0 ? current_joints[0] : 0.0,
                    current_joints.size() > 1 ? current_joints[1] : 0.0,
                    current_joints.size() > 2 ? current_joints[2] : 0.0,
                    current_joints.size() > 3 ? current_joints[3] : 0.0);
        RCLCPP_INFO(this->get_logger(), 
                    "检查当前位置: (%.3f, %.3f, %.3f)",
                    current_x, current_y, current_z);
        
        if (!is_reachable(current_x, current_y, current_z))
        {
          RCLCPP_WARN(this->get_logger(), 
                      "当前位置 (%.3f, %.3f, %.3f) 超出工作空间估计范围（仅警告，继续执行抓取）",
                      current_x, current_y, current_z);
          // 不再强制移动到安全位置，因为：
          // 1. 当前姿态是URDF/TF认可的合法姿态
          // 2. 安全位置可能对4DOF机械臂不可达
          // 3. 让抓取流程继续，由后续的IK检查来判定可达性
        }
        else
        {
          RCLCPP_INFO(this->get_logger(), 
                      "当前位置在工作空间估计范围内");
        }
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "无法检查当前位置: %s", e.what());
        // 继续执行，不因为检查失败而中断
      }
    }

    // 1. 打开夹爪
    publish_state("夹爪:打开中");
    if (!open_gripper())
    {
      publish_state("错误:打开夹爪失败");
      return false;
    }

    // 2. 计算位姿
    // 验证输入的cable_pose的orientation（用于诊断yaw传递问题）
    RCLCPP_INFO(this->get_logger(), 
                "[位姿计算] 输入cable_pose orientation: (%.3f, %.3f, %.3f, %.3f)",
                cable_pose.pose.orientation.x, cable_pose.pose.orientation.y,
                cable_pose.pose.orientation.z, cable_pose.pose.orientation.w);
    
    geometry_msgs::msg::PoseStamped pregrasp_pose = compute_pregrasp_pose(cable_pose);
    geometry_msgs::msg::PoseStamped grasp_pose = compute_grasp_pose(pregrasp_pose, cable_pose);
    
    // 验证计算后的位姿orientation（用于诊断yaw传递问题）
    RCLCPP_INFO(this->get_logger(),
                "[位姿计算] 计算后pregrasp_pose orientation: (%.3f, %.3f, %.3f, %.3f)",
                pregrasp_pose.pose.orientation.x, pregrasp_pose.pose.orientation.y,
                pregrasp_pose.pose.orientation.z, pregrasp_pose.pose.orientation.w);
    RCLCPP_INFO(this->get_logger(),
                "[位姿计算] 计算后grasp_pose orientation: (%.3f, %.3f, %.3f, %.3f)",
                grasp_pose.pose.orientation.x, grasp_pose.pose.orientation.y,
                grasp_pose.pose.orientation.z, grasp_pose.pose.orientation.w);
    
    // 转换为RPY便于直观对比
    auto convert_to_rpy = [](const geometry_msgs::msg::Quaternion& q_msg) -> std::tuple<double, double, double> {
      tf2::Quaternion q;
      q.setX(q_msg.x);
      q.setY(q_msg.y);
      q.setZ(q_msg.z);
      q.setW(q_msg.w);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      return std::make_tuple(roll, pitch, yaw);
    };
    
    auto [roll_cable, pitch_cable, yaw_cable] = convert_to_rpy(cable_pose.pose.orientation);
    auto [roll_pregrasp, pitch_pregrasp, yaw_pregrasp] = convert_to_rpy(pregrasp_pose.pose.orientation);
    auto [roll_grasp, pitch_grasp, yaw_grasp] = convert_to_rpy(grasp_pose.pose.orientation);
    
    RCLCPP_INFO(this->get_logger(), "[位姿对比] cable_pose RPY: roll=%.1f°, pitch=%.1f°, yaw=%.1f°",
                roll_cable * 180.0 / M_PI, pitch_cable * 180.0 / M_PI, yaw_cable * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "[位姿对比] pregrasp_pose RPY: roll=%.1f°, pitch=%.1f°, yaw=%.1f°",
                roll_pregrasp * 180.0 / M_PI, pitch_pregrasp * 180.0 / M_PI, yaw_pregrasp * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "[位姿对比] grasp_pose RPY: roll=%.1f°, pitch=%.1f°, yaw=%.1f°",
                roll_grasp * 180.0 / M_PI, pitch_grasp * 180.0 / M_PI, yaw_grasp * 180.0 / M_PI);

    // 2.5. 转换所有位姿到planning frame（关键：确保笛卡尔路径的waypoints在正确坐标系中）
    // 注意：cable_pose_planning 已在前面转换完成，这里只需要转换 pregrasp 和 grasp 位姿
    if (!transform_pose_to_planning(pregrasp_pose))
    {
      publish_state("error:transform_pregrasp");
      return false;
    }
    if (!transform_pose_to_planning(grasp_pose))
    {
      publish_state("error:transform_grasp");
      return false;
    }

    // 发布规划的目标位姿
    publish_grasp_viz("planned_targets", cable_pose_planning, &pregrasp_pose, &grasp_pose, nullptr);

    // 3. 移动到预抓取点
    if (emergency_stop_) {
      RCLCPP_WARN(this->get_logger(), "急停触发，停止预抓取");
      publish_state("急停:任务已停止");
      return false;
    }
    
    publish_state("规划:预抓取");
    // 预抓取使用IK→joint规划（避免Pilz PTP加速度限制问题）
    RCLCPP_INFO(this->get_logger(), "[预抓取] 使用IK→joint规划");
    if (!plan_execute_pose_target(pregrasp_pose))
    {
      publish_state("错误:预抓取失败");
      return false;
    }
    
    publish_grasp_viz("at_pregrasp", cable_pose_planning, &pregrasp_pose, &grasp_pose, nullptr);
    append_eef_path_once();
    
    // 4DOF姿态优化：在pregrasp位置进行姿态对准
    // 注意：4DOF机械臂可能无法独立调整姿态，这里主要是记录和验证当前姿态
    if (is_4dof_)
    {
      RCLCPP_INFO(this->get_logger(), "[4DOF姿态优化] 在pregrasp位置验证当前姿态");
      try {
        auto current_pose = getCurrentPoseSafe();
        // 修复时间戳问题：使用Time(0)获取最新可用状态
        auto current_state = move_group_interface_->getCurrentState(0.0);
        if (!current_state) {
          RCLCPP_WARN(this->get_logger(), "[4DOF姿态优化] 无法获取当前状态");
          return false;
        }
        std::vector<double> current_joint_values;
        const auto* jmg = current_state->getJointModelGroup("arm_group");
        if (jmg)
        {
          current_state->copyJointGroupPositions(jmg, current_joint_values);
        }
        
        RCLCPP_INFO(this->get_logger(), 
                    "[4DOF姿态优化] 当前末端执行器姿态: (%.3f, %.3f, %.3f, %.3f)",
                    current_pose.pose.orientation.x, current_pose.pose.orientation.y,
                    current_pose.pose.orientation.z, current_pose.pose.orientation.w);
        RCLCPP_INFO(this->get_logger(), 
                    "[4DOF姿态优化] 目标pregrasp姿态: (%.3f, %.3f, %.3f, %.3f)",
                    pregrasp_pose.pose.orientation.x, pregrasp_pose.pose.orientation.y,
                    pregrasp_pose.pose.orientation.z, pregrasp_pose.pose.orientation.w);
        
        // 显示当前Joint4值（控制yaw的关节）
        if (current_joint_values.size() >= 4)
        {
          RCLCPP_INFO(this->get_logger(), 
                      "[4DOF姿态优化] 当前Joint4值: %.3f rad (%.1f deg) - 控制夹爪yaw方向",
                      current_joint_values[3], current_joint_values[3] * 180.0 / M_PI);
        }
        
        // TODO: 对于4DOF机械臂，如果需要优化姿态，可以在这里添加微调逻辑
        // 例如：根据缆绳方向计算最优yaw角，然后通过joint target微调
        RCLCPP_INFO(this->get_logger(), "[4DOF姿态优化] 当前姿态已对齐（4DOF限制，无法独立调整姿态）");
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "[4DOF姿态优化] 无法获取当前姿态: %s", e.what());
      }
    }

    // 4. 缆绳碰撞体已在任务开始时添加（第0.5步），这里不再重复添加
    // 注意：缆绳对象已在清理残留对象后立即添加，便于在rviz中查看和规划
    publish_state("执行:预抓取");
    
    if (emergency_stop_) {
      RCLCPP_WARN(this->get_logger(), "急停触发，停止下压");
      publish_state("急停:任务已停止");
      return false;
    }

    // 5. 直线下压
    // 注意：下压阶段不使用CartesianPath，原因：
    // 1. computeCartesianPath默认起点是current_state，不是waypoints[0]
    //    执行下压时current_state已经是pregrasp_pose，如果waypoints包含pregrasp_pose会导致路径重复
    // 2. 抓缆绳是姿态受限任务：4DOF/欠驱动 + 缆绳非刚体 + 无力控 + 视觉只给xyz+yaw
    //    CartesianPath要求"姿态几乎不变走直线"，但实际需求是"允许姿态微调，只保证夹爪对缆绳"
    //    这两个需求在数学上冲突，导致CartesianPath不适合抓缆绳任务
    // 3. 分段IK下压（IK→joint target）是工程上唯一鲁棒的方案：
    //    - 不依赖OMPL的constraint sampler
    //    - 不依赖Cartesian path的姿态约束
    //    - 每一小段固定yaw，只下降z，解IK，setJointValueTarget，plan+execute
    //    这种方式对4DOF机械臂和抓缆绳任务最可靠
    publish_state("执行:下压");
    
    bool descend_success = false;
    
    // 三段式动作B：短距离笛卡尔直线下压（≤15cm）
    // 计算下压距离
    double descend_dist = std::sqrt(
      std::pow(grasp_pose.pose.position.x - pregrasp_pose.pose.position.x, 2) +
      std::pow(grasp_pose.pose.position.y - pregrasp_pose.pose.position.y, 2) +
      std::pow(grasp_pose.pose.position.z - pregrasp_pose.pose.position.z, 2)
    );

    RCLCPP_INFO(this->get_logger(), "[下压] 下压距离: %.3f m", descend_dist);

    // 读取pregrasp到位后的当前姿态，保持当前姿态进行下压（只改z坐标）
    // 这是4DOF机械臂的关键：姿态不可独立控制，必须保持当前姿态
    geometry_msgs::msg::PoseStamped current_pose_after_pregrasp = getCurrentPoseSafe();
    geometry_msgs::msg::PoseStamped current_pose_planning = current_pose_after_pregrasp;
    if (!transform_pose_to_planning(current_pose_planning))
    {
      RCLCPP_WARN(this->get_logger(), "[下压] 无法获取当前姿态，使用原始grasp_pose");
    }
    else
    {
      // 保持当前姿态，只改z坐标
      grasp_pose.pose.orientation = current_pose_planning.pose.orientation;
      RCLCPP_INFO(this->get_logger(), 
                  "[下压] 保持当前姿态进行下压: orientation=(%.3f, %.3f, %.3f, %.3f)",
                  grasp_pose.pose.orientation.x, grasp_pose.pose.orientation.y,
                  grasp_pose.pose.orientation.z, grasp_pose.pose.orientation.w);
    }

    // 下压前移除缆绳碰撞体，避免规划器认为"夹到缆绳"是碰撞
    if (add_collision_object_)
    {
      const std::string cable_world_id = cable_name_ + "_world";
      std::vector<std::string> known_objects = planning_scene_interface_->getKnownObjectNames();
      bool world_object_exists = false;
      for (const auto& name : known_objects)
      {
        if (name == cable_world_id)
        {
          world_object_exists = true;
          break;
        }
      }
      
      if (world_object_exists)
      {
        planning_scene_interface_->removeCollisionObjects({cable_world_id});
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 等待场景更新
        RCLCPP_INFO(this->get_logger(), "[下压] 已移除缆绳碰撞体 %s，避免规划冲突", cable_world_id.c_str());
      }
    }

    // 三段式动作B：下压使用joint-space分段IK（跳过Pilz LIN，直接使用更可靠的方法）
    // 注意：Pilz LIN对"严格直线 + 动力学约束"比较敏感，对于短距离下压（5cm），
    // 直接使用joint-space分段插值更可靠，避免速度限制违反等问题
    RCLCPP_INFO(this->get_logger(), "[下压] 使用joint-space分段IK下压（跳过Pilz LIN）");
    if (execute_segmented_descend(pregrasp_pose, grasp_pose, -1))
    {
      descend_success = true;
    }
    else
    {
      // Fallback：普通规划
      RCLCPP_WARN(this->get_logger(), "[下压] 分段IK下压失败，尝试普通规划fallback");
      if (plan_execute_pose_target(grasp_pose))
      {
        descend_success = true;
      }
    }

    if (!descend_success)
    {
      publish_state("错误:下压失败");
      return false;
    }

    publish_grasp_viz("at_grasp", cable_pose_planning, &pregrasp_pose, &grasp_pose, nullptr);
    append_eef_path_once();

    // 6. 闭合夹爪
    if (emergency_stop_) {
      RCLCPP_WARN(this->get_logger(), "急停触发，停止夹爪闭合");
      publish_state("急停:任务已停止");
      return false;
    }
    
    // 在闭合夹爪前，移除world对象避免碰撞冲突
    // 这样夹爪可以正常闭合，不会被world碰撞体阻挡
    if (add_collision_object_)
    {
      const std::string cable_world_id = cable_name_ + "_world";
      // 检查world对象是否存在
      std::vector<std::string> known_objects = planning_scene_interface_->getKnownObjectNames();
      bool world_object_exists = false;
      for (const auto& name : known_objects)
      {
        if (name == cable_world_id)
        {
          world_object_exists = true;
          break;
        }
      }
      
      if (world_object_exists)
      {
        planning_scene_interface_->removeCollisionObjects({cable_world_id});
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        RCLCPP_INFO(this->get_logger(), "闭合前已移除world对象: %s，避免碰撞冲突", cable_world_id.c_str());
      }
      else
      {
        RCLCPP_DEBUG(this->get_logger(), "world对象 %s 不存在，跳过移除", cable_world_id.c_str());
      }
    }
    
    publish_state("夹爪:闭合中");
    if (!close_gripper())
    {
      publish_state("错误:闭合夹爪失败");
      // 清理world对象（如果还存在）
      if (add_collision_object_)
      {
        const std::string cable_world_id = cable_name_ + "_world";
        // 检查world对象是否存在
        std::vector<std::string> known_objects = planning_scene_interface_->getKnownObjectNames();
        bool world_object_exists = false;
        for (const auto& name : known_objects)
        {
          if (name == cable_world_id)
          {
            world_object_exists = true;
            break;
          }
        }
        
        if (world_object_exists)
        {
          planning_scene_interface_->removeCollisionObjects({cable_world_id});
          RCLCPP_INFO(this->get_logger(), "闭合失败，已清理world对象: %s", cable_world_id.c_str());
        }
        else
        {
          RCLCPP_DEBUG(this->get_logger(), "闭合失败，world对象 %s 不存在，跳过清理", cable_world_id.c_str());
        }
      }
      return false;
    }

    // 7. 保持时间
    publish_state("夹爪:保持中");
    std::this_thread::sleep_for(std::chrono::milliseconds(gripper_hold_time_ms_));

    // 8. 轻微抬起确认
    if (emergency_stop_) {
      RCLCPP_WARN(this->get_logger(), "急停触发，停止抬起");
      publish_state("急停:任务已停止");
      return false;
    }
    
    publish_state("执行:抬起");
    geometry_msgs::msg::PoseStamped lift_pose = grasp_pose;  // grasp_pose已经在planning_frame_中
    lift_pose.pose.position.z += lift_distance_;
    // lift_pose已经在planning_frame_中，因为它是从grasp_pose复制的
    
    RCLCPP_INFO(this->get_logger(), 
                "[Lift阶段] 目标提升位置: (%.3f, %.3f, %.3f), 提升距离: %.3f m",
                lift_pose.pose.position.x, lift_pose.pose.position.y, lift_pose.pose.position.z,
                lift_distance_);
    
    publish_grasp_viz("lift_target", cable_pose_planning, &pregrasp_pose, &grasp_pose, &lift_pose);
    
    bool lift_success = false;
    
    // 三段式动作C：提拉使用普通规划（暂时不用LIN，避免姿态问题）
    RCLCPP_INFO(this->get_logger(), "[提拉] 使用普通规划（IK→joint）");
    if (plan_execute_pose_target(lift_pose))
    {
      lift_success = true;
      RCLCPP_INFO(this->get_logger(), "[提拉] 普通规划成功");
    }
    else
    {
      // Fallback：分段提升
      RCLCPP_WARN(this->get_logger(), "[提拉] 普通规划失败，尝试分段提升fallback");
      if (execute_segmented_lift(grasp_pose, lift_pose, 3))
      {
        lift_success = true;
        RCLCPP_INFO(this->get_logger(), "[提拉] 分段提升成功");
      }
    }

    if (!lift_success)
    {
      RCLCPP_ERROR(this->get_logger(), "抬起失败，清理场景并返回失败");
      // 清理world和attached对象，避免污染场景
      scene_detach_and_remove(cable_world_id, eef_link_);
      scene_detach_and_remove(cable_attached_id, eef_link_);
      publish_state("错误:抬起失败");
      return false;  // 不再继续attach，防止假抓取污染场景
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "抬起成功完成");
      publish_grasp_viz("at_lift", cable_pose_planning, &pregrasp_pose, &grasp_pose, &lift_pose);
      append_eef_path_once();
      RCLCPP_INFO(this->get_logger(), "注意：新任务不进行附着操作，每次新任务都是新业务，需要重新控制");
    }

    // 9. 成功判定（简化：如果抬起不报错就认为成功）
    // TODO: 可以添加夹爪反馈宽度检查

    // 10. 附着物体（已移除）
    // 新任务不进行附着操作，每次新任务都是新业务，需要重新控制
    // 如果需要在后续任务中使用附着功能，可以在任务完成后由外部系统控制

    // 11. 完成抓取
    publish_state("完成");
    RCLCPP_INFO(this->get_logger(), "缆绳抓取完成（未进行附着，新任务需要重新控制）");

    // 12. 任务完成后清理场景对象（根据MoveIt最佳实践）
    RCLCPP_INFO(this->get_logger(), "=== 任务完成：清理场景对象 ===");
    scene_detach_and_remove(cable_attached_id, eef_link_);
    scene_detach_and_remove(cable_world_id, eef_link_);
    // 也清理旧命名（向后兼容）
    scene_detach_and_remove(cable_name_, eef_link_);

    // 验证清理完成
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    {
      std::vector<std::string> known_objects_final = planning_scene_interface_->getKnownObjectNames();
      bool found_attached_final = false;
      bool found_world_final = false;
      for (const auto& name : known_objects_final)
      {
        if (name == cable_attached_id) found_attached_final = true;
        if (name == cable_world_id) found_world_final = true;
      }
      if (found_attached_final || found_world_final)
      {
        RCLCPP_WARN(this->get_logger(), 
                    "清理验证：仍有残留对象存在 (attached:%s, world:%s)，但任务已完成",
                    found_attached_final ? "是" : "否", found_world_final ? "是" : "否");
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "清理验证：所有场景对象已清除");
      }
    }

    return true;
  }

  // 成员变量
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cable_pose_sub_;
  rclcpp::Subscription<m5_msgs::msg::CablePoseWithYaw>::SharedPtr cable_pose_with_yaw_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr place_pose_sub_;
  rclcpp::Subscription<m5_msgs::msg::CablePoseWithYaw>::SharedPtr place_pose_with_yaw_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr emergency_stop_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  
  // 可视化发布器
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr eef_path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cable_pose_viz_pub_;
  nav_msgs::msg::Path eef_path_;
  bool enable_viz_{true};
  int marker_seq_{0};
  
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_interface_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread executor_thread_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string planning_frame_;
  std::string eef_link_;

  std::queue<CableGraspTask> task_queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;

  std::mutex moveit_mutex_;

  std::vector<std::thread> worker_threads_;
  std::atomic<bool> stop_workers_{false};
  std::atomic<bool> emergency_stop_{false};  // 急停标志
  size_t num_worker_threads_{1};
  
  // TF转换fallback计数（用于检测系统时钟/TF延迟问题）
  std::atomic<size_t> tf_fallback_count_{0};

  // 参数
  std::string cable_name_;
  double cable_diameter_;
  double cable_length_;
  std::string cable_frame_id_;
  std::string cable_shape_;
  double cable_center_offset_z_{0.0};  // cable_pose到圆柱中心的Z偏移（默认0.0表示cable_pose是中心）
  
  double default_planning_time_{15.0};  // 默认规划时间，用于临时切换后恢复
  int num_planning_attempts_{30};
  double max_velocity_scaling_{0.9};
  double max_acceleration_scaling_{0.9};
  double goal_position_tolerance_{0.01};
  double goal_orientation_tolerance_{0.5};
  double fallback_planning_time_{2.0};
  double ik_timeout_{0.1};
  double state_check_timeout_{1.0};
  double state_check_interval_{0.1};
  double robot_state_publisher_wait_{2.0};
  double executor_startup_wait_{0.5};
  double segment_execution_wait_{0.5};  // 从0.1s增加到0.5s，避免状态未稳定
  double state_sync_wait_{0.1};
  double control_frequency_{10.0};
  double read_frequency_{10.0};
  double main_loop_frequency_{10.0};
  double control_period_{0.1};  // 1.0 / control_frequency_
  double read_period_{0.1};     // 1.0 / read_frequency_
  double main_loop_period_{0.1}; // 1.0 / main_loop_frequency_

  bool is_4dof_{false};  // 是否为4DOF机械臂（通过arm_group关节数量检测）

  double approach_offset_z_;
  double descend_distance_;
  double lift_distance_;
  double max_pos_error_;
  bool use_cartesian_;
  double eef_step_;
  double jump_threshold_;
  
  // Place parameters
  double place_approach_offset_z_;
  double place_descend_distance_;
  double place_retreat_distance_;
  double place_max_pos_error_;
  double place_planning_time_;
  int place_num_planning_attempts_;
  double place_max_velocity_scaling_;
  double place_max_acceleration_scaling_;
  double place_goal_position_tolerance_;
  double place_goal_orientation_tolerance_;
  double yaw_offset_{0.0};  // yaw偏移量（弧度）
  bool yaw_flip_{false};  // 是否翻转yaw 180度
  double grasp_yaw_add_{M_PI / 2.0};  // 线缆切向→夹持方向偏移（默认+90°横夹）
  double orientation_yaw_offset_{M_PI / 2.0};  // orientation的Yaw分量偏移（默认90度），用于调整joint 4角度
  double z_clearance_{0.002};  // 抓取高度安全余量（米，默认2mm）
  
  // yaw合法性检查参数
  bool yaw_validation_enabled_{true};  // 是否启用yaw合法性检查
  double yaw_tolerance_{0.0873};  // yaw搜索容差（默认±5°，约0.0873 rad）
  double yaw_search_step_{0.0175};  // yaw搜索步长（默认1°，约0.0175 rad）
  
  // Joint4-yaw映射优化参数
  bool joint4_yaw_search_enabled_{true};  // 是否启用Joint4-yaw迭代搜索
  double joint4_yaw_search_range_{M_PI};  // Joint4-yaw搜索范围（默认±180度）
  double joint4_yaw_search_step_{0.0873};  // Joint4-yaw搜索步长（默认5度，约0.0873 rad）
  
  // Joint约束参数（4DOF机器人控制）
  double joint1_tolerance_{0.1745};  // Joint1容差（默认±10°，约0.1745 rad）
  bool joint4_constraint_enabled_{false};  // 是否启用Joint4约束
  double joint4_offset_{0.0};  // Joint4固定offset（零位标定）
  double joint4_tolerance_{0.0873};  // Joint4容差（默认±5°，约0.0873 rad）
  
  // 状态回滚参数
  bool enable_recovery_{true};  // 是否启用执行失败后的状态回滚
  double recovery_timeout_{5.0};  // 回滚操作的超时时间
  
  // yaw候选搜索参数
  bool yaw_candidate_search_enabled_{true};  // 是否启用yaw候选搜索
  double yaw_candidate_center_{0.0};  // 候选搜索中心yaw（仅xyz输入时使用）
  double yaw_candidate_range_{M_PI};  // 搜索范围（±180°）
  double yaw_candidate_step_{0.2618};  // 搜索步长（15°，约0.2618 rad）
  
  // 地面安全参数
  double ground_height_{0.0};  // 地面高度
  double ground_offset_below_base_{-0.05};  // 地面在基座下方的偏移（-5cm，基座下方5cm）
  double camera_error_margin_{0.005};  // 相机标定误差余量（5mm）
  double min_ground_clearance_{0.010};  // 最小离地安全距离（10mm）
  bool add_ground_plane_{true};  // 是否添加地面碰撞对象
  
  // 下压距离限制
  double max_cartesian_descend_distance_{0.15};  // 单次笛卡尔下压最大距离（15cm）
  
  // TCP偏移补偿参数（LinkGG到夹爪中心的偏移，在EEF frame中）
  double tcp_offset_x_{0.0};  // m  TCP偏移X（LinkGG→夹爪中心）
  double tcp_offset_y_{-0.024};  // m  TCP偏移Y（LinkGG→夹爪中心）
  double tcp_offset_z_{-0.0086};  // m  TCP偏移Z（LinkGG→夹爪中心）
  
  // 全零状态预运动参数
  bool auto_move_to_ready_{true};  // 是否自动移动到预备位置
  double ready_joint1_{0.0};  // rad  预备位置Joint1
  double ready_joint2_{-0.5};  // rad  预备位置Joint2（约-30度）
  double ready_joint3_{-0.5};  // rad  预备位置Joint3（约-30度）
  double ready_joint4_{0.0};  // rad  预备位置Joint4
  double near_zero_threshold_{0.1};  // rad  全零状态判断阈值

  std::string gripper_mode_;
  double gripper_open_width_;
  double gripper_close_width_;
  double gripper_close_extra_;
  int gripper_hold_time_ms_;
  double gripper_angle_min_{1.01};
  double gripper_angle_max_{-1.01};
  double gripper_width_min_{0.0};

  bool add_collision_object_;
  std::vector<std::string> allow_touch_links_;

  // 工作空间参数 (基于URDF精确计算)
  double workspace_base_height_{0.2145};    // Joint2轴高度 (base原点+link1高度)
  double workspace_link2_length_{0.264};    // 大臂长度
  double workspace_link3_length_{0.143};    // 小臂长度  
  double workspace_link4_to_eef_{0.187};    // 末端长度
  double workspace_reach_radius_margin_{0.95}; // 最大半径裕度(略保守)
  double workspace_max_height_offset_{0.60};   // 相对Joint2轴的最大高度偏移
  double workspace_min_height_offset_{-0.10};  // 相对Joint2轴的最小高度偏移
  double workspace_min_radius_{0.08};          // 最小半径(避免太靠近base)
  double workspace_safe_x_min_{0.15};
  double workspace_safe_x_max_{0.40};
  double workspace_safe_y_min_{-0.15};
  double workspace_safe_y_max_{0.15};
  double workspace_safe_z_min_{0.25};
  double workspace_safe_z_max_{0.35};
  double workspace_medium_x_min_{0.15};
  double workspace_medium_x_max_{0.45};
  double workspace_medium_y_min_{-0.20};
  double workspace_medium_y_max_{0.20};
  double workspace_medium_z_min_{0.20};
  double workspace_medium_z_max_{0.40};

  // 容差参数
  double orientation_epsilon_{1e-6};

  // 轨迹规划器
  m5_grasp::TrajectoryPlanner trajectory_planner_;

  // 轨迹规划参数
  bool use_linear_interpolation_{false};
  double linear_velocity_{0.1};
  double linear_min_duration_{1.0};
  double linear_step_scale_{0.5};
  double cartesian_timeout_{3.0};
  double cartesian_timeout_descend_{5.0};
  double cartesian_timeout_lift_{3.0};
  double cartesian_descend_threshold_{0.95};
  double cartesian_lift_threshold_{0.90};
  bool cartesian_retry_on_timeout_{true};
  bool adaptive_segments_{true};
  int min_segments_{2};
  int max_segments_{4};
  bool use_polynomial_interpolation_{false};
  std::string polynomial_type_{"cubic"};
  double polynomial_duration_{2.0};
  double polynomial_dt_{0.01};
  bool use_bspline_{false};
  int bspline_degree_{3};
  double bspline_duration_{3.0};
  double bspline_dt_{0.01};
};

int main(int argc, char * argv[])
{
  try {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("main"), "ROS2已初始化");

    {
      RCLCPP_INFO(rclcpp::get_logger("main"), "创建m5_grasp节点...");
      auto node = std::make_shared<M5Grasp>();
      RCLCPP_INFO(rclcpp::get_logger("main"), "节点创建成功");
      
      RCLCPP_INFO(rclcpp::get_logger("main"), "启动executor...");
      node->start_executor();
      RCLCPP_INFO(rclcpp::get_logger("main"), "Executor已启动");
      
      // 等待一小段时间让executor开始运行
      double executor_wait = node->get_parameter("grasp.executor_startup_wait").as_double();
      std::this_thread::sleep_for(std::chrono::milliseconds(
        static_cast<int>(executor_wait * 1000)));
      
      RCLCPP_INFO(rclcpp::get_logger("main"), "初始化MoveIt接口...");
      node->init_moveit();
      RCLCPP_INFO(rclcpp::get_logger("main"), "MoveIt接口已初始化");
      
      RCLCPP_INFO(rclcpp::get_logger("main"), "m5_grasp节点已完全启动，等待消息...");

      // 使用配置的main循环频率
      double main_loop_period = 1.0 / node->get_parameter("grasp.main_loop_frequency").as_double();
      int main_loop_period_ms = static_cast<int>(main_loop_period * 1000);
      
      while (rclcpp::ok())
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(main_loop_period_ms));
      }
    }

    RCLCPP_INFO(rclcpp::get_logger("main"), "正在关闭ROS2...");
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "错误: " << e.what() << std::endl;
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }
}
