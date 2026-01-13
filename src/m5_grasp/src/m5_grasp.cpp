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
#include <cmath>
#include <cstdlib>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
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

// 任务结构体：存储缆绳位姿和原始yaw（用于障碍物方向计算）
struct CableGraspTask {
  geometry_msgs::msg::PoseStamped cable_pose;  // 缆绳位姿（包含调整后的orientation，用于夹爪方向）
  double original_yaw;  // 视觉提供的原始yaw（缆绳切向方向），用于障碍物方向计算
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
    RCLCPP_INFO(this->get_logger(), "订阅话题: /emergency_stop");
    RCLCPP_INFO(this->get_logger(), "  消息类型: std_msgs::msg::String");
    RCLCPP_INFO(this->get_logger(), "  队列大小: 10");
    RCLCPP_INFO(this->get_logger(), "发布话题: /grasp_state");
    RCLCPP_INFO(this->get_logger(), "  消息类型: std_msgs::msg::String");
    RCLCPP_INFO(this->get_logger(), "  队列大小: 10");
    RCLCPP_INFO(this->get_logger(), "等待缆绳位置消息...");
    RCLCPP_INFO(this->get_logger(), "========================================");
    
    // 验证订阅器是否创建成功
    if (cable_pose_sub_) {
      RCLCPP_INFO(this->get_logger(), "✓ 缆绳位置订阅器创建成功");
    } else {
      RCLCPP_ERROR(this->get_logger(), "✗ 缆绳位置订阅器创建失败！");
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
    this->declare_parameter("grasp.enable_recovery", true);  // 是否启用执行失败后的状态回滚
    this->declare_parameter("grasp.recovery_timeout", 5.0);  // 回滚操作的超时时间
    
    // TCP偏移补偿参数（LinkGG到夹爪中心的偏移）
    this->declare_parameter("grasp.tcp_offset_x", 0.0);  // m  TCP偏移X（LinkGG→夹爪中心）
    this->declare_parameter("grasp.tcp_offset_y", -0.024);  // m  TCP偏移Y（LinkGG→夹爪中心）
    this->declare_parameter("grasp.tcp_offset_z", -0.0086);  // m  TCP偏移Z（LinkGG→夹爪中心）

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

    // Workspace parameters
    this->declare_parameter("workspace.base_height", 0.141);
    this->declare_parameter("workspace.link2_length", 0.264);
    this->declare_parameter("workspace.link3_length", 0.143);
    this->declare_parameter("workspace.link4_to_eef", 0.187);
    this->declare_parameter("workspace.reach_radius_margin", 1.2);
    this->declare_parameter("workspace.max_height_offset", 0.9);
    this->declare_parameter("workspace.min_height_offset", -0.3);
    this->declare_parameter("workspace.safe_x_min", 0.20);
    this->declare_parameter("workspace.safe_x_max", 0.35);
    this->declare_parameter("workspace.safe_y_min", -0.15);
    this->declare_parameter("workspace.safe_y_max", 0.15);
    this->declare_parameter("workspace.safe_z_min", 0.25);
    this->declare_parameter("workspace.safe_z_max", 0.35);
    this->declare_parameter("workspace.medium_x_min", 0.15);
    this->declare_parameter("workspace.medium_x_max", 0.45);
    this->declare_parameter("workspace.medium_y_min", -0.20);
    this->declare_parameter("workspace.medium_y_max", 0.20);
    this->declare_parameter("workspace.medium_z_min", 0.20);
    this->declare_parameter("workspace.medium_z_max", 0.40);

    // Tolerance parameters
    this->declare_parameter("tolerance.orientation_epsilon", 1e-6);

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
    enable_recovery_ = this->get_parameter("grasp.enable_recovery").as_bool();
    recovery_timeout_ = this->get_parameter("grasp.recovery_timeout").as_double();
    
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
    // 注意：cable_pose_callback没有提供yaw，需要从orientation提取
    // 如果orientation是向下（roll=π），则yaw可以从orientation的Z分量提取
    double original_yaw = 0.0;
    try {
      tf2::Quaternion q;
      tf2::fromMsg(cable_pose.pose.orientation, q);
      // 提取yaw：从向下orientation中提取Z轴旋转
      // 如果orientation是向下（roll=π），yaw可以通过atan2计算
      tf2::Matrix3x3 rot_matrix(q);
      double roll, pitch, yaw;
      rot_matrix.getRPY(roll, pitch, yaw);
      original_yaw = yaw;
      RCLCPP_INFO(this->get_logger(), "[任务入队] 从orientation提取yaw: %.3f rad (%.1f deg)", 
                  original_yaw, original_yaw * 180.0 / M_PI);
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "[任务入队] 无法从orientation提取yaw，使用默认值0: %s", e.what());
      original_yaw = 0.0;
    }

    RCLCPP_INFO(this->get_logger(), "[任务入队] 准备将任务加入队列: pos=(%.3f, %.3f, %.3f), frame=%s",
                cable_pose.pose.position.x, cable_pose.pose.position.y, 
                cable_pose.pose.position.z, cable_pose.header.frame_id.c_str());
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      size_t old_size = task_queue_.size();
      while (!task_queue_.empty()) task_queue_.pop();
      
      // 创建任务结构体
      CableGraspTask task;
      task.cable_pose = cable_pose;
      task.original_yaw = original_yaw;
      
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

    // 提取xyz坐标和yaw角
    // 统一计算所有yaw偏移，避免在多个地方叠加导致逻辑混乱
    double yaw_cmd = msg->yaw;
    
    // 1) 线缆切向 → 夹持朝向：yaw=0度（线缆切向）时，夹爪应垂直于线缆
    //    grasp_yaw_add=90°时，yaw=0度→orientation的Yaw分量=90°→joint 4=90°
    yaw_cmd += grasp_yaw_add_;
    RCLCPP_INFO(this->get_logger(), "应用grasp_yaw_add: %.3f rad (%.1f deg)", 
                grasp_yaw_add_, grasp_yaw_add_ * 180.0 / M_PI);
    
    // 2) 应用yaw_offset和yaw_flip调整（保留原有调参逻辑）
    yaw_cmd += yaw_offset_;
    if (yaw_flip_) {
      yaw_cmd += M_PI;
      RCLCPP_INFO(this->get_logger(), "应用yaw_flip: yaw += π");
    }
    if (std::abs(yaw_offset_) > 1e-6) {
      RCLCPP_INFO(this->get_logger(), "应用yaw_offset: %.3f rad (%.1f deg)", 
                  yaw_offset_, yaw_offset_ * 180.0 / M_PI);
    }
    
    // 3) 注意：不再应用orientation_yaw_offset到yaw_cmd
    // orientation_yaw_offset应该通过其他方式处理（如果需要零位标定）
    // 这样绳子的yaw可以直接对应到joint 4的旋转角度
    
    RCLCPP_INFO(this->get_logger(), "最终yaw_cmd: %.3f rad (%.1f deg)", 
                yaw_cmd, yaw_cmd * 180.0 / M_PI);
    
    // 计算四元数姿态（roll=0, pitch=0, yaw=最终yaw_cmd）
    // 注意：compute_downward_orientation_with_yaw内部不再叠加任何offset
    geometry_msgs::msg::Quaternion orientation = compute_downward_orientation_with_yaw(yaw_cmd);
    
    // 构造PoseStamped消息
    geometry_msgs::msg::PoseStamped cable_pose;
    cable_pose.header = msg->header;
    cable_pose.pose.position = msg->position;
    cable_pose.pose.orientation = orientation;
    
    // 更新时间戳
    if (cable_pose.header.stamp.sec == 0 && cable_pose.header.stamp.nanosec == 0) {
      cable_pose.header.stamp = this->now();
    }

    RCLCPP_INFO(this->get_logger(), "处理缆绳位置（带yaw）: x=%.3f y=%.3f z=%.3f yaw=%.3f (frame: %s)",
                cable_pose.pose.position.x,
                cable_pose.pose.position.y,
                cable_pose.pose.position.z,
                yaw_cmd,
                cable_pose.header.frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "计算得到的orientation: (%.3f, %.3f, %.3f, %.3f)",
                cable_pose.pose.orientation.x,
                cable_pose.pose.orientation.y,
                cable_pose.pose.orientation.z,
                cable_pose.pose.orientation.w);
    
    // 临时调试：立即进行IK求解，输出joint 4值
    // 注意：此调试代码已优化，添加了详细的日志和超时保护，避免阻塞回调函数
    RCLCPP_DEBUG(this->get_logger(), "[调试] 开始IK求解调试...");
    try {
      // 尝试获取锁，但设置超时避免长时间阻塞
      // 如果无法立即获取锁，跳过调试代码，确保任务能够正常入队
      std::unique_lock<std::mutex> lock(moveit_mutex_, std::try_to_lock);
      if (!lock.owns_lock()) {
        RCLCPP_DEBUG(this->get_logger(), "[调试] MoveIt互斥锁被占用，跳过IK调试（不影响任务处理）");
      } else if (move_group_interface_) {
        RCLCPP_DEBUG(this->get_logger(), "[调试] 获取MoveIt状态（超时0.1s）...");
        auto state = move_group_interface_->getCurrentState(0.1);  // 直接调用，避免递归锁
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
          RCLCPP_WARN(this->get_logger(), "[调试] 获取当前状态失败（可能超时）");
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
      
      // 创建任务结构体，包含位姿和原始yaw
      CableGraspTask task;
      task.cable_pose = cable_pose;
      task.original_yaw = original_yaw;
      
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
    if (timeout > 0.0)
    {
      return move_group_interface_->getCurrentState(timeout);
    }
    return move_group_interface_->getCurrentState();
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
      auto state = move_group_interface_->getCurrentState();
      jmg = state->getJointModelGroup("arm_group");
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
        auto current_state = move_group_interface_->getCurrentState();
        current_state->copyJointGroupPositions(jmg, current_joint_values);
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
      result = gripper_group_interface_->execute(gripper_plan);
      if (result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        // 验证夹爪是否真的闭合（检查实际关节值）
        std::this_thread::sleep_for(std::chrono::milliseconds(200));  // 等待执行完成
        auto actual_joints = gripper_group_interface_->getCurrentJointValues();
        if (actual_joints.size() >= 2)
        {
          double actual_gl = actual_joints[0];
          double actual_gr = actual_joints[1];
          double target_gl = joint_gl;
          double target_gr = joint_gr;
          
          double error_gl = std::abs(actual_gl - target_gl);
          double error_gr = std::abs(actual_gr - target_gr);
          double tolerance = 0.1;  // rad，约5.7度
          
          if (error_gl > tolerance || error_gr > tolerance)
          {
            RCLCPP_WARN(this->get_logger(), 
                        "夹爪闭合验证失败: 目标(%.3f, %.3f), 实际(%.3f, %.3f), 误差(%.3f, %.3f)",
                        target_gl, target_gr, actual_gl, actual_gr, error_gl, error_gr);
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
    }

    RCLCPP_WARN(this->get_logger(), "夹爪闭合失败");
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
      auto state = move_group_interface_->getCurrentState();
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
    
    // 方法：使用setRPY直接计算orientation，避免万向锁问题
    // setRPY(Roll, Pitch, Yaw)的旋转顺序是：先Yaw，再Pitch，最后Roll
    // 所以 setRPY(M_PI, 0.0, yaw) 表示：先绕Z轴旋转yaw，再绕X轴旋转180°（朝下）
    // 这样yaw分量直接对应输入的yaw，不受万向锁影响
    
    // 注意：由于Joint4是绕Link4的局部Z轴旋转，而yaw是绕世界Z轴旋转，
    // 它们之间的关系依赖于Joint1-3的当前状态。对于4DOF机械臂，IK求解器会
    // 自动处理这个关系，但可能不会完全匹配。我们在IK求解后会进行验证和调整。
    tf2::Quaternion q;
    q.setRPY(M_PI, 0.0, yaw);  // Roll=180°（朝下）, Pitch=0°, Yaw=yaw
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

  // 检查位置是否在工作空间内（参考m5_planning.cpp的实现）
  bool is_reachable(double x, double y, double z)
  {
    // URDF基本参数（从配置读取）
    const double base_height = workspace_base_height_;
    const double link2_length = workspace_link2_length_;
    const double link3_length = workspace_link3_length_;
    const double link4_to_eef = workspace_link4_to_eef_;
    
    // 计算理论最大伸展距离（完全伸展时，保守估计）
    const double max_reach_radius = link2_length + link3_length + link4_to_eef;
    const double max_reach_radius_with_margin = max_reach_radius * workspace_reach_radius_margin_;
    
    // 计算理论最大高度（完全伸展时，保守估计）
    const double max_height = base_height + workspace_max_height_offset_;
    const double min_height = base_height + workspace_min_height_offset_;
    
    // 计算到基座的距离（半径）
    const double radius = std::sqrt(x * x + y * y);
    
    // 软过滤：只拒绝明显超出最大半径/高度的点
    // 其他情况都允许进入 IK 测试，由 direct IK 作为最终可达性判定
    if (radius > max_reach_radius_with_margin)
    {
      RCLCPP_WARN(this->get_logger(),
                  "目标位置 (%.3f, %.3f, %.3f) 明显超出最大半径 (%.3f > %.3f m)，拒绝",
                  x, y, z, radius, max_reach_radius_with_margin);
      return false;
    }
    
    if (z > max_height || z < min_height)
    {
      RCLCPP_WARN(this->get_logger(),
                  "目标位置 (%.3f, %.3f, %.3f) 明显超出高度范围 (z=%.3f 不在 [%.3f, %.3f] m)，拒绝",
                  x, y, z, z, min_height, max_height);
      return false;
    }
    
    // 其他情况都允许进入 IK 测试
    RCLCPP_INFO(this->get_logger(), "目标位置 (%.3f, %.3f, %.3f) 通过工作空间检查（半径=%.3f m，高度=%.3f m）",
                x, y, z, radius, z);
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

    // 在目标yaw±搜索范围内搜索
    // 如果搜索范围超过2π，限制在合理范围内以避免跨越360度边界
    double search_range = joint4_yaw_search_range_;
    if (search_range > M_PI) {
      search_range = M_PI;  // 限制在±180度范围内
      RCLCPP_WARN(this->get_logger(),
                  "[Joint4-yaw搜索] 搜索范围过大(%.3f rad)，限制为±180度以避免跨越360度边界",
                  joint4_yaw_search_range_);
    }
    
    double search_min = target_yaw - search_range;
    double search_max = target_yaw + search_range;
    
    // 确保搜索范围在[-π, π]内，如果超出则调整
    if (search_min < -M_PI) {
      double offset = -M_PI - search_min;
      search_min = -M_PI;
      search_max += offset;
    }
    if (search_max > M_PI) {
      double offset = search_max - M_PI;
      search_max = M_PI;
      search_min -= offset;
    }
    
    int search_steps = static_cast<int>((search_max - search_min) / joint4_yaw_search_step_);
    if (search_steps < 1) search_steps = 1;

    RCLCPP_INFO(this->get_logger(),
                "[Joint4-yaw搜索] 搜索范围: [%.3f, %.3f] rad ([%.1f, %.1f]°)，步长=%.3f rad (%.1f°)，步数=%d",
                search_min, search_max,
                search_min * 180.0 / M_PI, search_max * 180.0 / M_PI,
                joint4_yaw_search_step_, joint4_yaw_search_step_ * 180.0 / M_PI,
                search_steps);

    int valid_solutions = 0;
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
        
        RCLCPP_INFO(this->get_logger(),
                    "[Joint4-yaw搜索] 找到更优候选: yaw=%.3f rad (%.1f°), Joint4=%.3f rad (%.1f°), 差值=%.3f rad (%.1f°)",
                    candidate_yaw, candidate_yaw * 180.0 / M_PI,
                    candidate_joint4, candidate_joint4 * 180.0 / M_PI,
                    candidate_diff, candidate_diff * 180.0 / M_PI);
      }
    }

    RCLCPP_INFO(this->get_logger(),
                "[Joint4-yaw搜索] 搜索完成: 有效解数量=%d，最优yaw=%.3f rad (%.1f°), 最优Joint4=%.3f rad (%.1f°), 最优差值=%.3f rad (%.1f°)",
                valid_solutions,
                optimal_yaw, optimal_yaw * 180.0 / M_PI,
                optimal_joint4, optimal_joint4 * 180.0 / M_PI,
                best_diff, best_diff * 180.0 / M_PI);

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
    try {
      auto current_state = move_group_interface_->getCurrentState();
      scene->setCurrentState(*current_state);
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
    
    // 计算更安全的最小z值：考虑线缆直径和安全余量
    // min_z = cable_pose.z + cable_center_offset_z + 0.5 * cable_diameter + z_clearance
    double min_z = cable_pose.pose.position.z + cable_center_offset_z_ + 0.5 * cable_diameter_;
    min_z += z_clearance_;  // 添加安全余量（默认2mm）
    
    if (center_grasp_pose.pose.position.z < min_z)
    {
      RCLCPP_WARN(this->get_logger(), 
                  "抓取点z (%.3f) 低于安全最小z值 (%.3f)，限制为安全最小z值 (线缆中心z=%.3f, 直径=%.3f, 安全余量=%.3f)", 
                  center_grasp_pose.pose.position.z, min_z,
                  cable_pose.pose.position.z + cable_center_offset_z_, cable_diameter_, z_clearance_);
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
      state_for_yaw_check = move_group_interface_->getCurrentState();
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
        
        auto state = move_group_interface_->getCurrentState();
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
        auto state_for_bspline = move_group_interface_->getCurrentState();
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
    auto state = move_group_interface_->getCurrentState();
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
        state = move_group_interface_->getCurrentState();
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
          double yaw_diff = joint4_value - target_yaw_from_orientation;
          
          RCLCPP_INFO(this->get_logger(),
                      "[IK→Joint] Joint4值: %.3f rad (%.1f deg) - 控制夹爪yaw方向",
                      joint4_value, joint4_value * 180.0 / M_PI);
          RCLCPP_INFO(this->get_logger(),
                      "[IK→Joint] 目标orientation的yaw分量: %.3f rad (%.1f deg)",
                      target_yaw_from_orientation, target_yaw_from_orientation * 180.0 / M_PI);
          RCLCPP_INFO(this->get_logger(),
                      "[IK→Joint] Joint4 vs orientation yaw: Joint4=%.3f rad (%.1f°), orientation yaw=%.3f rad (%.1f°), 差值=%.3f rad (%.1f°)",
                      joint4_value, joint4_value * 180.0 / M_PI,
                      target_yaw_from_orientation, target_yaw_from_orientation * 180.0 / M_PI,
                      yaw_diff, yaw_diff * 180.0 / M_PI);
          
          // Joint4验证和调整：如果Joint4与期望yaw差异太大，尝试迭代搜索更优的orientation
          // 注意：由于Joint4是绕Link4局部Z轴旋转，而yaw是绕世界Z轴旋转，它们之间的关系
          // 依赖于Joint1-3的当前状态，所以需要使用迭代搜索来找到最优映射
          const double max_yaw_diff = 0.5236;  // 30度（约0.5236 rad）
          if (std::abs(yaw_diff) > max_yaw_diff && joint4_yaw_search_enabled_)
          {
            RCLCPP_WARN(this->get_logger(),
                        "[IK→Joint] Joint4与期望yaw差异较大 (%.1f°)，开始迭代搜索更优的orientation...",
                        std::abs(yaw_diff) * 180.0 / M_PI);
            
            // 使用迭代搜索找到使Joint4最接近期望yaw的orientation
            double optimal_yaw = target_yaw_from_orientation;
            double optimal_joint4 = joint4_value;
            std::vector<double> optimal_joint_values;
            
            if (find_optimal_yaw_for_joint4(target_yaw_from_orientation,
                                           pose_in_planning.pose.position.x,
                                           pose_in_planning.pose.position.y,
                                           pose_in_planning.pose.position.z,
                                           state,
                                           optimal_yaw,
                                           optimal_joint4,
                                           optimal_joint_values))
            {
              // 找到更优的解，直接使用搜索返回的关节值（避免重新求解IK导致结果不一致）
              RCLCPP_INFO(this->get_logger(),
                          "[IK→Joint] 迭代搜索找到更优解，最优yaw=%.3f rad (%.1f°)，最优Joint4=%.3f rad (%.1f°)",
                          optimal_yaw, optimal_yaw * 180.0 / M_PI,
                          optimal_joint4, optimal_joint4 * 180.0 / M_PI);
              
              if (optimal_joint_values.size() >= 4)
              {
                // 计算使用搜索返回关节值后的差值（使用归一化的角度差值）
                double optimal_yaw_diff_raw = optimal_joint4 - target_yaw_from_orientation;
                double optimal_yaw_diff = normalize_angle_diff(optimal_yaw_diff_raw);
                double yaw_diff_normalized = normalize_angle_diff(yaw_diff);
                
                RCLCPP_INFO(this->get_logger(),
                            "[IK→Joint] 搜索返回的Joint4值: %.3f rad (%.1f deg), 与期望yaw差值: %.3f rad (%.1f°)",
                            optimal_joint4, optimal_joint4 * 180.0 / M_PI,
                            optimal_yaw_diff, optimal_yaw_diff * 180.0 / M_PI);
                
                // 如果搜索返回的Joint4更接近期望yaw，使用搜索返回的结果
                if (std::abs(optimal_yaw_diff) < std::abs(yaw_diff_normalized))
                {
                  RCLCPP_INFO(this->get_logger(),
                              "[IK→Joint] 使用迭代搜索找到的最优orientation和关节值（Joint4更接近期望yaw）");
                  joint_values = optimal_joint_values;  // 直接使用搜索返回的关节值
                  pose_in_planning.pose.orientation = compute_downward_orientation_with_yaw(optimal_yaw);
                }
                else
                {
                  RCLCPP_INFO(this->get_logger(),
                              "[IK→Joint] 保持原始orientation和关节值（搜索返回的值未改善）");
                }
              }
              else
              {
                RCLCPP_WARN(this->get_logger(),
                            "[IK→Joint] 搜索返回的关节值数量不足，保持原始结果");
              }
            }
            else
            {
              RCLCPP_INFO(this->get_logger(),
                          "[IK→Joint] 迭代搜索未找到更优解，保持原始orientation和关节值");
            }
          }
          else if (std::abs(yaw_diff) > max_yaw_diff && !joint4_yaw_search_enabled_)
          {
            RCLCPP_WARN(this->get_logger(),
                        "[IK→Joint] Joint4与期望yaw差异较大 (%.1f°)，但迭代搜索已禁用，保持原始结果",
                        std::abs(yaw_diff) * 180.0 / M_PI);
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
    // 修复：IK成功后、调用plan之前强制刷新start_state，避免使用旧状态
    // IK求解可能花费了一些时间，状态可能已经更新，必须使用最新状态
    move_group_interface_->setStartStateToCurrentState();
    
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
      move_group_interface_->setStartStateToCurrentState();
      
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
      auto state = move_group_interface_->getCurrentState();
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
      auto state = move_group_interface_->getCurrentState();
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
    
    move_group_interface_->setStartStateToCurrentState();
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
      auto state_for_ik = move_group_interface_->getCurrentState();
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
      move_group_interface_->setStartStateToCurrentState();
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
    move_group_interface_->setStartStateToCurrentState();
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
    return false;
  }

  // 核心抓取函数
  // cable_yaw: 视觉提供的原始yaw（缆绳切向方向），用于障碍物方向计算
  bool do_cable_grasp(const geometry_msgs::msg::PoseStamped& cable_pose, double cable_yaw)
  {
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "=== 开始执行抓取任务 ===");
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
        auto current_state = move_group_interface_->getCurrentState();
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
    
    // 统一4DOF和非4DOF分支：都优先使用分段IK下压（唯一鲁棒方案）
    if (is_4dof_)
    {
      // 4DOF：使用分段下压（IK→joint target，唯一鲁棒方案）
      RCLCPP_INFO(this->get_logger(), "[4DOF策略] 使用分段下压（IK→joint target）");
      if (execute_segmented_descend(pregrasp_pose, grasp_pose, -1))  // -1表示自适应分段
      {
        descend_success = true;
      }
      else
      {
        // Fallback：普通规划（直接到目标点）
        RCLCPP_WARN(this->get_logger(), "[4DOF策略] 分段下压失败，尝试普通规划fallback");
        if (plan_execute_pose_target(grasp_pose))
        {
          descend_success = true;
        }
      }
    }
    else
    {
      // 非4DOF：同样优先使用分段下压（IK→joint target，更可靠）
      RCLCPP_INFO(this->get_logger(), "[非4DOF策略] 优先使用分段下压（IK→joint target）");
      if (execute_segmented_descend(pregrasp_pose, grasp_pose, -1))  // -1表示自适应分段
      {
        descend_success = true;
      }
      else
      {
        // Fallback：普通规划（直接到目标点）
        RCLCPP_WARN(this->get_logger(), "[非4DOF策略] 分段下压失败，尝试普通规划fallback");
        if (plan_execute_pose_target(grasp_pose))
        {
          descend_success = true;
        }
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
    
    std::vector<geometry_msgs::msg::Pose> lift_waypoints;
    // 注意：computeCartesianPath默认起点是current_state，不是waypoints[0]
    // 抬起时current_state已经是grasp_pose（下压已完成），所以只放lift_pose
    lift_waypoints.push_back(lift_pose.pose);   // 已经在planning_frame_中
    
    bool lift_success = false;
    
    if (is_4dof_)
    {
      // 4DOF：优先分段提升（IK→joint target，更稳，避免OMPL constraint sampler问题）
      RCLCPP_INFO(this->get_logger(), "[4DOF策略] 优先使用分段提升（IK→joint target）");
      if (execute_segmented_lift(grasp_pose, lift_pose, 3))
      {
        lift_success = true;
        RCLCPP_INFO(this->get_logger(), "[4DOF策略] 分段提升成功");
      }
      else
      {
        // Fallback 1: 尝试笛卡尔路径（如果启用）
        if (use_cartesian_)
        {
          RCLCPP_WARN(this->get_logger(), "[4DOF策略] 分段提升失败，尝试笛卡尔路径fallback");
          lift_success = execute_cartesian(lift_waypoints, false);  // false 表示 lift
          if (lift_success)
          {
            RCLCPP_INFO(this->get_logger(), "[4DOF策略] 笛卡尔路径fallback成功");
          }
        }
        
        // Fallback 2: 普通规划（IK→joint target）
        if (!lift_success)
        {
          RCLCPP_WARN(this->get_logger(), "[4DOF策略] 分段提升和笛卡尔路径都失败，尝试普通规划fallback");
          if (plan_execute_pose_target(lift_pose))
          {
            lift_success = true;
            RCLCPP_INFO(this->get_logger(), "[4DOF策略] 普通规划fallback成功");
          }
        }
      }
    }
    else
    {
      // 非4DOF：保持原逻辑（优先Cartesian，失败后fallback到分段提升）
      if (use_cartesian_)
      {
        RCLCPP_INFO(this->get_logger(), "[非4DOF策略] 尝试笛卡尔路径");
        lift_success = execute_cartesian(lift_waypoints, false);  // false 表示 lift
        if (lift_success)
        {
          RCLCPP_INFO(this->get_logger(), "[非4DOF策略] 笛卡尔路径成功");
        }
        else
        {
          // Fallback 1: 分段提升（使用IK→joint target）
          RCLCPP_WARN(this->get_logger(), "[非4DOF策略] 笛卡尔路径失败，尝试分段提升fallback（IK→joint target）");
          if (execute_segmented_lift(grasp_pose, lift_pose, 3))
          {
            lift_success = true;
            RCLCPP_INFO(this->get_logger(), "[非4DOF策略] 分段提升fallback成功");
          }
          else
          {
            // Fallback 2: 普通规划（直接到目标点）
            RCLCPP_WARN(this->get_logger(), "[非4DOF策略] 分段提升失败，尝试普通规划fallback");
            if (plan_execute_pose_target(lift_pose))
            {
              lift_success = true;
              RCLCPP_INFO(this->get_logger(), "[非4DOF策略] 普通规划fallback成功");
            }
          }
        }
      }
      else
      {
        // 不使用Cartesian，直接使用分段提升或普通规划
        RCLCPP_INFO(this->get_logger(), "[非4DOF策略] 使用分段提升（IK→joint target）");
        if (execute_segmented_lift(grasp_pose, lift_pose, 3))
        {
          lift_success = true;
          RCLCPP_INFO(this->get_logger(), "[非4DOF策略] 分段提升成功");
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), "[非4DOF策略] 分段提升失败，尝试普通规划fallback");
          if (plan_execute_pose_target(lift_pose))
          {
            lift_success = true;
            RCLCPP_INFO(this->get_logger(), "[非4DOF策略] 普通规划fallback成功");
          }
        }
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
  
  // 状态回滚参数
  bool enable_recovery_{true};  // 是否启用执行失败后的状态回滚
  double recovery_timeout_{5.0};  // 回滚操作的超时时间
  
  // TCP偏移补偿参数（LinkGG到夹爪中心的偏移，在EEF frame中）
  double tcp_offset_x_{0.0};  // m  TCP偏移X（LinkGG→夹爪中心）
  double tcp_offset_y_{-0.024};  // m  TCP偏移Y（LinkGG→夹爪中心）
  double tcp_offset_z_{-0.0086};  // m  TCP偏移Z（LinkGG→夹爪中心）

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

  // 工作空间参数
  double workspace_base_height_{0.141};
  double workspace_link2_length_{0.264};
  double workspace_link3_length_{0.143};
  double workspace_link4_to_eef_{0.187};
  double workspace_reach_radius_margin_{1.2};
  double workspace_max_height_offset_{0.9};
  double workspace_min_height_offset_{-0.3};
  double workspace_safe_x_min_{0.20};
  double workspace_safe_x_max_{0.35};
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
