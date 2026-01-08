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

    // 订阅急停话题
    emergency_stop_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/emergency_stop", 10,
        std::bind(&M5Grasp::emergency_stop_callback, this, std::placeholders::_1));

    // 发布抓取状态
    state_publisher_ = this->create_publisher<std_msgs::msg::String>("/grasp_state", 10);

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
      if (node_self_ptr_)
      {
        executor_->remove_node(node_self_ptr_);
      }
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
    for (size_t i = 0; i < num_worker_threads_; ++i)
    {
      worker_threads_.emplace_back(&M5Grasp::worker_thread, this);
      RCLCPP_INFO(this->get_logger(), "工作线程 %zu 已启动", i + 1);
    }
  }

  // 启动executor（在对象创建后调用）
  // 注意：shared_from_this() 只能在构造函数完成后调用
  // 永远不要在构造函数中使用 shared_from_this()，否则会抛出 bad_weak_ptr 异常
  // 此函数必须在对象完全构造后调用（由 main() 保证）
  void start_executor()
  {
    node_self_ptr_ = shared_from_this();
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_self_ptr_);
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

    // Gripper parameters (position control only, no force control)
    this->declare_parameter("gripper.mode", "position");
    this->declare_parameter("gripper.open_width", 0.060);
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
    this->declare_parameter("trajectory.cartesian_descend_threshold", 0.95);
    this->declare_parameter("trajectory.cartesian_lift_threshold", 0.90);
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
    cartesian_descend_threshold_ = this->get_parameter("trajectory.cartesian_descend_threshold").as_double();
    cartesian_lift_threshold_ = this->get_parameter("trajectory.cartesian_lift_threshold").as_double();
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
    
    // 停止MoveIt执行
    {
      std::lock_guard<std::mutex> lock(moveit_mutex_);
      if (move_group_interface_)
      {
        try {
          move_group_interface_->stop();
          RCLCPP_WARN(this->get_logger(), "已停止机械臂运动");
        } catch (const std::exception& e) {
          RCLCPP_ERROR(this->get_logger(), "停止机械臂运动时出错: %s", e.what());
        }
      }
      if (gripper_group_interface_)
      {
        try {
          gripper_group_interface_->stop();
          RCLCPP_WARN(this->get_logger(), "已停止夹爪运动");
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

    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      while (!task_queue_.empty()) task_queue_.pop();
      task_queue_.push(cable_pose);
    }
    queue_cv_.notify_one();
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
    while (!stop_workers_)
    {
      geometry_msgs::msg::PoseStamped task;
      bool has_task = false;

      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait(lock, [this]() { 
          return !task_queue_.empty() || stop_workers_; 
        });

        if (stop_workers_ && task_queue_.empty())
        {
          break;
        }

        if (!task_queue_.empty())
        {
          task = task_queue_.front();
          task_queue_.pop();
          has_task = true;
        }
      }

      if (has_task)
      {
        // 检查急停状态
        if (emergency_stop_)
        {
          RCLCPP_WARN(this->get_logger(), "系统处于急停状态，取消当前任务");
          publish_state("急停:任务已取消");
          continue;
        }
        
        try {
          do_cable_grasp(task);
        } catch (const std::exception& e) {
          RCLCPP_ERROR(this->get_logger(), "处理抓取任务时发生异常: %s", e.what());
          publish_state("错误:异常");
        }
      }
    }
    RCLCPP_INFO(this->get_logger(), "工作线程退出");
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
  bool waitForStateStable(double max_wait_time = 1.0, 
                          int stable_count = 5, 
                          double threshold = 0.002)
  {
    std::lock_guard<std::mutex> lock(moveit_mutex_);
    
    // 通过 getCurrentState() 获取 RobotState，然后获取 JointModelGroup
    auto state = move_group_interface_->getCurrentState();
    const auto* jmg = state->getJointModelGroup("arm_group");
    if (!jmg)
    {
      RCLCPP_WARN(this->get_logger(), "[状态稳定] 无法获取arm_group，使用简单等待");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      return false;
    }

    auto start_time = std::chrono::steady_clock::now();
    std::vector<double> prev_joint_values;
    int consecutive_stable = 0;

    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count() < max_wait_time)
    {
      auto current_state = move_group_interface_->getCurrentState();
      std::vector<double> current_joint_values;
      current_state->copyJointGroupPositions(jmg, current_joint_values);

      if (!prev_joint_values.empty() && prev_joint_values.size() == current_joint_values.size())
      {
        // 计算关节值变化
        double max_change = 0.0;
        for (size_t i = 0; i < current_joint_values.size(); ++i)
        {
          double change = std::abs(current_joint_values[i] - prev_joint_values[i]);
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
            RCLCPP_DEBUG(this->get_logger(), 
                        "[状态稳定] 状态已稳定（连续%d次变化 < %.4f rad）", 
                        stable_count, threshold);
            return true;
          }
        }
        else
        {
          consecutive_stable = 0;  // 重置计数
        }
      }

      prev_joint_values = current_joint_values;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));  // 每50ms检查一次
    }

    RCLCPP_WARN(this->get_logger(), 
                "[状态稳定] 超时（%.1fs），状态可能未完全稳定，但继续执行", 
                max_wait_time);
    return false;  // 超时，但返回false表示未完全稳定
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
    }

    // 获取当前关节状态（自检日志）
    std::vector<double> current_joint_values = gripper_group_interface_->getCurrentJointValues();
    double current_joint_gl = (current_joint_values.size() > 0) ? current_joint_values[0] : 0.0;
    double current_joint_gr = (current_joint_values.size() > 1) ? current_joint_values[1] : 0.0;
    
    double joint_gl = width_to_joint_angle(width);
    double joint_gr = -joint_gl;  // 镜像对称

    // 自检日志：打印目标值和当前值
    RCLCPP_INFO(this->get_logger(), 
                "[夹爪打开自检] 目标宽度=%.3f m, 目标JointGL=%.4f rad (%.2f°), 目标JointGR=%.4f rad (%.2f°)",
                width, joint_gl, joint_gl * 180.0 / M_PI, joint_gr, joint_gr * 180.0 / M_PI);
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
  geometry_msgs::msg::Pose make_cable_cylinder_pose(const geometry_msgs::msg::PoseStamped& cable_pose_planning)
  {
    geometry_msgs::msg::Pose pose;
    
    // 计算圆柱中心位置：cable_pose + center_offset_z
    // cable_center_offset_z_ 默认 0.0 表示 cable_pose 就是圆柱中心
    // 如果设置为 cable_length_ * 0.5，表示 cable_pose 是圆柱底部，需要向上推半根长度到中心
    pose.position.x = cable_pose_planning.pose.position.x;
    pose.position.y = cable_pose_planning.pose.position.y;
    pose.position.z = cable_pose_planning.pose.position.z + cable_center_offset_z_;
    
    // 明确设置圆柱朝向：圆柱轴对齐世界 Z 轴（竖直方向）
    // MoveIt 默认圆柱轴是 Z 轴，所以 orientation 设为单位四元数（无旋转）
    // 这样圆柱就是竖直的，从 (cable_pose.z + center_offset_z - length/2) 到 (cable_pose.z + center_offset_z + length/2)
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    
    // TODO: 如果未来需要支持非竖直缆绳，可以根据 cable_pose.orientation 旋转圆柱轴
    // 需要确认缆绳坐标定义：z轴沿缆绳？还是x轴？
    
    return pose;
  }

  // 添加缆绳碰撞体
  bool scene_add_cable_object(const geometry_msgs::msg::PoseStamped& cable_pose, const std::string& object_id = "cable_1")
  {
    std::lock_guard<std::mutex> lock(moveit_mutex_);

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = planning_frame_;
    collision_object.id = object_id;
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

    // 创建圆柱体
    shape_msgs::msg::SolidPrimitive cylinder;
    cylinder.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    cylinder.dimensions.resize(2);
    cylinder.dimensions[0] = cable_length_;  // height
    cylinder.dimensions[1] = cable_diameter_ / 2.0;  // radius

    collision_object.primitives.push_back(cylinder);

    // 转换位姿到planning frame（使用通用函数，内部使用 tf2::TimePointZero）
    geometry_msgs::msg::PoseStamped pose_in_planning = cable_pose;
    if (!transform_pose_to_planning(pose_in_planning))
    {
      RCLCPP_ERROR(this->get_logger(), "转换缆绳位姿失败");
      return false;
    }

    // 使用统一函数计算圆柱pose（确保与attach时一致）
    geometry_msgs::msg::Pose cylinder_pose = make_cable_cylinder_pose(pose_in_planning);
    collision_object.primitive_poses.push_back(cylinder_pose);

    // 添加到场景
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    planning_scene_interface_->addCollisionObjects(collision_objects);

    RCLCPP_INFO(this->get_logger(), "添加缆绳碰撞体: %s，等待场景更新...", object_id.c_str());
    
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
    }
    
    return true;
  }

  // 附着物体到末端执行器（包含完整几何信息）
  // 改进：ATTACH时使用eef_link frame，pose使用相对eef_link的位姿，减少RViz闪烁
  bool scene_attach(const std::string& object_id, const std::string& eef_link, 
                    const geometry_msgs::msg::PoseStamped& cable_pose)
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
    // 1. 计算cable在planning_frame中的位姿（使用make_cable_cylinder_pose）
    geometry_msgs::msg::Pose cylinder_pose_planning = make_cable_cylinder_pose(pose_in_planning);
    
    // 2. 将cylinder_pose从planning_frame转换到eef_link frame
    geometry_msgs::msg::PoseStamped cylinder_pose_stamped;
    cylinder_pose_stamped.header.frame_id = planning_frame_;
    cylinder_pose_stamped.pose = cylinder_pose_planning;
    
    try {
      // 使用TF转换：从planning_frame转换到eef_link
      geometry_msgs::msg::TransformStamped transform = 
          tf_buffer_->lookupTransform(eef_link, planning_frame_, tf2::TimePointZero);
      tf2::doTransform(cylinder_pose_stamped, cylinder_pose_stamped, transform);
      
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

    // 先尝试 detach 同名旧 attached（即使不存在也没关系）
    moveit_msgs::msg::AttachedCollisionObject rm;
    rm.object.id = object_id;
    rm.link_name = eef_link;  // 用参数 eef_link
    rm.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    planning_scene_interface_->applyAttachedCollisionObject(rm);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    // 先移除场景中的对象，避免 attach 时覆盖/重建导致闪烁
    // PlanningSceneInterface 的 add/apply 是异步的，如果 add 还没完全传播就 attach，
    // 可能导致 RViz 中物体闪烁/跳动（不影响规划，但影响观感）
    std::vector<std::string> object_ids = {object_id};
    planning_scene_interface_->removeCollisionObjects(object_ids);
    
    // 等待一小段时间确保移除操作传播到 scene monitor
    // 这样可以避免 attach 时覆盖/重建导致的视觉闪烁
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // 然后再 attach
    planning_scene_interface_->applyAttachedCollisionObject(attached_object);

    RCLCPP_INFO(this->get_logger(), "附着物体 %s 到 %s（包含完整几何信息）", object_id.c_str(), eef_link.c_str());
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

  // 清理函数：detach + remove（避免残留对象导致冲突）
  bool scene_detach_and_remove(const std::string& object_id, const std::string& eef_link)
  {
    std::lock_guard<std::mutex> lock(moveit_mutex_);

    // 1) Detach (REMOVE attached)
    moveit_msgs::msg::AttachedCollisionObject aco;
    aco.object.id = object_id;
    aco.link_name = eef_link;  // 必填：表示从哪个link上detach
    aco.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    planning_scene_interface_->applyAttachedCollisionObject(aco);

    // 2) Remove world object (如果也存在)
    planning_scene_interface_->removeCollisionObjects({object_id});

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    RCLCPP_INFO(this->get_logger(), "清理完成(detach+remove): %s", object_id.c_str());
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
    geometry_msgs::msg::PoseStamped pregrasp_pose = cable_pose;
    pregrasp_pose.pose.position.z += approach_offset_z_;
    
    // 如果orientation是默认值，需要计算合适的orientation
    bool is_default = is_default_orientation(cable_pose.pose.orientation);
    
    if (is_default)
    {
      // 优先使用当前末端执行器的orientation（如果可用且不是默认值）
      // 因为当前orientation通常对当前位置是可达的
      bool use_current = false;
      geometry_msgs::msg::Quaternion current_orientation;
      
      if (move_group_interface_)
      {
        try {
          auto current_pose = getCurrentPoseSafe();
          current_orientation = current_pose.pose.orientation;
          
          RCLCPP_INFO(this->get_logger(), 
                      "预抓取位姿计算: 当前末端执行器位置: (%.3f, %.3f, %.3f)",
                      current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
          RCLCPP_INFO(this->get_logger(), 
                      "预抓取位姿计算: 当前末端执行器orientation: (%.3f, %.3f, %.3f, %.3f)",
                      current_orientation.x, current_orientation.y,
                      current_orientation.z, current_orientation.w);
          
          if (!is_default_orientation(current_orientation))
          {
            use_current = true;
            RCLCPP_INFO(this->get_logger(), 
                        "预抓取位姿计算: 当前末端执行器orientation不是默认值，将使用它");
          }
          else
          {
            RCLCPP_INFO(this->get_logger(), 
                        "预抓取位姿计算: 当前末端执行器orientation也是默认值，将使用向下orientation");
          }
        } catch (const std::exception& e) {
          RCLCPP_WARN(this->get_logger(), "无法获取当前末端执行器pose: %s", e.what());
        }
      }
      
      if (use_current)
      {
        // 使用当前末端执行器的orientation（通常对当前位置可达）
        pregrasp_pose.pose.orientation = current_orientation;
        RCLCPP_INFO(this->get_logger(), 
                    "预抓取位姿计算: 使用当前末端执行器orientation（通常可达）: (%.3f, %.3f, %.3f, %.3f)",
                    pregrasp_pose.pose.orientation.x, pregrasp_pose.pose.orientation.y,
                    pregrasp_pose.pose.orientation.z, pregrasp_pose.pose.orientation.w);
      }
      else
      {
        // Fallback: 使用向下orientation（适合抓取任务）
        pregrasp_pose.pose.orientation = compute_downward_orientation();
        RCLCPP_INFO(this->get_logger(), 
                    "预抓取位姿计算: 检测到默认orientation，使用向下orientation（适合抓取）: (%.3f, %.3f, %.3f, %.3f)",
                    pregrasp_pose.pose.orientation.x, pregrasp_pose.pose.orientation.y,
                    pregrasp_pose.pose.orientation.z, pregrasp_pose.pose.orientation.w);
      }
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), 
                  "预抓取位姿计算: 使用缆绳位姿的orientation: (%.3f, %.3f, %.3f, %.3f)",
                  pregrasp_pose.pose.orientation.x, pregrasp_pose.pose.orientation.y,
                  pregrasp_pose.pose.orientation.z, pregrasp_pose.pose.orientation.w);
    }
    
    RCLCPP_INFO(this->get_logger(), 
                "预抓取位姿计算: 位置=(%.3f, %.3f, %.3f), 偏移z=%.3f",
                pregrasp_pose.pose.position.x, pregrasp_pose.pose.position.y,
                pregrasp_pose.pose.position.z, approach_offset_z_);
    
    // 检查预抓取点是否在工作空间内
    double pregrasp_x = pregrasp_pose.pose.position.x;
    double pregrasp_y = pregrasp_pose.pose.position.y;
    double pregrasp_z = pregrasp_pose.pose.position.z;
    
    if (!is_reachable(pregrasp_x, pregrasp_y, pregrasp_z))
    {
      RCLCPP_WARN(this->get_logger(), 
                  "预抓取点 (%.3f, %.3f, %.3f) 超出工作空间，调整到工作空间内",
                  pregrasp_x, pregrasp_y, pregrasp_z);
      adjust_to_workspace(pregrasp_x, pregrasp_y, pregrasp_z);
      pregrasp_pose.pose.position.x = pregrasp_x;
      pregrasp_pose.pose.position.y = pregrasp_y;
      pregrasp_pose.pose.position.z = pregrasp_z;
      RCLCPP_INFO(this->get_logger(), 
                  "预抓取位姿已调整: 位置=(%.3f, %.3f, %.3f)",
                  pregrasp_pose.pose.position.x, pregrasp_pose.pose.position.y,
                  pregrasp_pose.pose.position.z);
    }
    
    return pregrasp_pose;
  }

  // 计算抓取位姿（从预抓取点向下压descend_distance_，但不低于缆绳点）
  geometry_msgs::msg::PoseStamped compute_grasp_pose(
      const geometry_msgs::msg::PoseStamped& pregrasp_pose,
      const geometry_msgs::msg::PoseStamped& cable_pose)
  {
    // 从预抓取点向下压descend_distance_
    geometry_msgs::msg::PoseStamped grasp_pose = pregrasp_pose;
    grasp_pose.pose.position.z -= descend_distance_;
    
    // 确保不会压到桌面（不低于缆绳点）
    double min_z = cable_pose.pose.position.z;
    if (grasp_pose.pose.position.z < min_z)
    {
      RCLCPP_WARN(this->get_logger(), 
                  "抓取点z (%.3f) 低于缆绳点z (%.3f)，限制为缆绳点z", 
                  grasp_pose.pose.position.z, min_z);
      grasp_pose.pose.position.z = min_z;
    }
    
    // 如果orientation是默认值，使用当前末端姿态
    bool is_default_orientation = (std::abs(cable_pose.pose.orientation.x) < 1e-6 &&
                                   std::abs(cable_pose.pose.orientation.y) < 1e-6 &&
                                   std::abs(cable_pose.pose.orientation.z) < 1e-6 &&
                                   std::abs(cable_pose.pose.orientation.w - 1.0) < 1e-6);
    
    if (is_default_orientation && move_group_interface_)
    {
      try {
        auto current_pose = getCurrentPoseSafe();
        grasp_pose.pose.orientation = current_pose.pose.orientation;
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "无法获取当前末端执行器orientation: %s", e.what());
      }
    }
    
    RCLCPP_INFO(this->get_logger(), 
                "抓取位姿计算: 预抓取点z=%.3f, 下压距离=%.3f, 抓取点z=%.3f, 缆绳点z=%.3f",
                pregrasp_pose.pose.position.z, descend_distance_, 
                grasp_pose.pose.position.z, min_z);
    
    return grasp_pose;
  }

  // 辅助函数：带超时的 computeCartesianPath
  // 使用异步执行避免阻塞工作线程
  bool compute_cartesian_path_with_timeout(
      const std::vector<geometry_msgs::msg::Pose>& waypoints,
      double step,
      double jump_threshold,
      moveit_msgs::msg::RobotTrajectory& trajectory,
      double timeout_sec)
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
      RCLCPP_WARN(this->get_logger(), "笛卡尔路径计算超时（%.1f秒），触发fallback", timeout_sec);
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
  // 添加超时机制（3秒），避免 computeCartesianPath 卡住
  // 注意：是否使用线性插补或B样条由配置文件中的参数控制
  bool execute_cartesian(const std::vector<geometry_msgs::msg::Pose>& waypoints, bool use_descend = true)
  {
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
              bspline_waypoints, eef_step_, jump_threshold_, trajectory, cartesian_timeout_);
          
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
              
              moveit::core::MoveItErrorCode result = move_group_interface_->execute(plan);
              if (result == moveit::core::MoveItErrorCode::SUCCESS)
              {
                RCLCPP_INFO(this->get_logger(), "B样条笛卡尔路径执行成功");
                return true;
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
            linear_waypoints, scaled_step, jump_threshold_, trajectory, cartesian_timeout_);
        
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
            
            moveit::core::MoveItErrorCode result = move_group_interface_->execute(plan);
            if (result == moveit::core::MoveItErrorCode::SUCCESS)
            {
              RCLCPP_INFO(this->get_logger(), "线性插补笛卡尔路径执行成功");
              return true;
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
        waypoints, eef_step_, jump_threshold_, trajectory, cartesian_timeout_);
    
    if (fraction < 0.0)  // 超时或异常
    {
      RCLCPP_WARN(this->get_logger(), "笛卡尔路径计算超时（%.1f秒），触发fallback", cartesian_timeout_);
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

    // 执行轨迹（已在锁内）
    {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;

      moveit::core::MoveItErrorCode result = move_group_interface_->execute(plan);
      if (result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_INFO(this->get_logger(), "笛卡尔路径执行成功");
        return true;
      }

      RCLCPP_WARN(this->get_logger(), "笛卡尔路径执行失败");
      return false;
    }
  }

  // 分段下压 fallback（4DOF机械臂常用策略）
  // 将下压路径分成多个waypoint，对每个waypoint进行IK→joint target规划
  // 这样比Cartesian路径更稳，因为避免了OMPL的constraint sampler问题
  bool execute_segmented_descend(const geometry_msgs::msg::PoseStamped& start_pose,
                                 const geometry_msgs::msg::PoseStamped& end_pose,
                                 int num_segments = 3)
  {
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
    std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    waypoints.push_back(start_pose);  // 起点

    // 生成中间waypoints
    for (int i = 1; i < num_segments; ++i)
    {
      geometry_msgs::msg::PoseStamped waypoint = start_pose;
      waypoint.pose.position.x += dx * i;
      waypoint.pose.position.y += dy * i;
      waypoint.pose.position.z += dz * i;
      // 保持orientation（使用起点或终点的orientation，这里使用起点）
      waypoint.pose.orientation = start_pose.pose.orientation;
      waypoints.push_back(waypoint);
    }

    waypoints.push_back(end_pose);  // 终点

    // 逐段执行IK→joint target规划
    // 跳过第一段（i=0），因为它是起点，已经在那个位置，不需要移动
    for (size_t i = 1; i < waypoints.size(); ++i)
    {
      RCLCPP_INFO(this->get_logger(), 
                  "[分段下压] 执行段 %zu/%zu: 位置=(%.3f, %.3f, %.3f)",
                  i + 1, waypoints.size(),
                  waypoints[i].pose.position.x,
                  waypoints[i].pose.position.y,
                  waypoints[i].pose.position.z);

      // 执行前状态同步：等待状态更新并强制使用最新状态
      RCLCPP_INFO(this->get_logger(), 
                  "[分段下压] 段 %zu/%zu 执行前，等待%.0fms并同步状态...", 
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
                    "[分段下压] 段 %zu/%zu 执行失败", i + 1, waypoints.size());
        return false;
      }
      
      // 等待状态稳定（每段执行后都等待，确保状态同步）
      // ✅ 修复：使用状态稳定判断函数，而不是简单的sleep
      RCLCPP_INFO(this->get_logger(), 
                  "[分段下压] 段 %zu/%zu 执行成功，等待状态稳定...", 
                  i + 1, waypoints.size());
      if (!waitForStateStable(segment_execution_wait_, 5, 0.002))
      {
        // 如果状态稳定判断超时，至少等待最小时间
        RCLCPP_WARN(this->get_logger(), 
                    "[分段下压] 状态稳定判断超时，使用最小等待时间");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
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
      // 保持orientation（使用起点或终点的orientation，这里使用起点）
      waypoint.pose.orientation = start_pose.pose.orientation;
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
                    "[分段提升] 段 %zu/%zu 执行失败", i + 1, waypoints.size());
        return false;
      }
      
      // 等待状态稳定（每段执行后都等待，确保状态同步）
      // ✅ 修复：使用状态稳定判断函数，而不是简单的sleep
      RCLCPP_INFO(this->get_logger(), 
                  "[分段提升] 段 %zu/%zu 执行成功，等待状态稳定...", 
                  i + 1, waypoints.size());
      if (!waitForStateStable(segment_execution_wait_, 5, 0.002))
      {
        // 如果状态稳定判断超时，至少等待最小时间
        RCLCPP_WARN(this->get_logger(), 
                    "[分段提升] 状态稳定判断超时，使用最小等待时间");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
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

    // 尝试多次IK求解（使用不同的seed）
    auto state = move_group_interface_->getCurrentState();
    const auto* jmg = state->getJointModelGroup("arm_group");
    if (!jmg)
    {
      RCLCPP_WARN(this->get_logger(), "[IK→Joint] 无法获取arm_group关节模型组");
      return false;
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
        RCLCPP_WARN(this->get_logger(), "[IK→Joint] Joint target规划成功但执行失败");
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
  bool do_cable_grasp(const geometry_msgs::msg::PoseStamped& cable_pose)
  {
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
    scene_detach_and_remove(cable_world_id, eef_link_);
    scene_detach_and_remove(cable_attached_id, eef_link_);
    // 也清理旧命名（向后兼容）
    scene_detach_and_remove(cable_name_, eef_link_);

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
    geometry_msgs::msg::PoseStamped pregrasp_pose = compute_pregrasp_pose(cable_pose);
    geometry_msgs::msg::PoseStamped grasp_pose = compute_grasp_pose(pregrasp_pose, cable_pose);

    // 2.5. 转换所有位姿到planning frame（关键：确保笛卡尔路径的waypoints在正确坐标系中）
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
    // 统一转换 cable_pose 到 planning frame，后续所有操作都使用 planning frame
    geometry_msgs::msg::PoseStamped cable_pose_planning = cable_pose;
    if (!transform_pose_to_planning(cable_pose_planning))
    {
      publish_state("错误:缆绳位姿转换失败");
      return false;
    }

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

    // 4. 添加缆绳碰撞体（在pregrasp成功之后，避免被碰撞体阻挡）
    // 使用 cable_pose_planning（已经在 planning frame 中）
    if (add_collision_object_)
    {
      const std::string cable_world_id = cable_name_ + "_world";
      if (!scene_add_cable_object(cable_pose_planning, cable_world_id))
      {
        publish_state("错误:添加物体失败");
        return false;
      }
    }
    publish_state("执行:预抓取");
    
    if (emergency_stop_) {
      RCLCPP_WARN(this->get_logger(), "急停触发，停止下压");
      publish_state("急停:任务已停止");
      return false;
    }

    // 5. 直线下压（现在waypoints已经在planning_frame_中）
    publish_state("执行:下压");
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(pregrasp_pose.pose);
    waypoints.push_back(grasp_pose.pose);
    
    bool descend_success = false;
    
    if (is_4dof_)
    {
      // 4DOF：优先分段下压（IK→joint target，更稳，避免OMPL constraint sampler问题）
      RCLCPP_INFO(this->get_logger(), "[4DOF策略] 优先使用分段下压（IK→joint target）");
      if (execute_segmented_descend(pregrasp_pose, grasp_pose, 3))
      {
        descend_success = true;
      }
      else
      {
        // Fallback 1: 快速尝试 Cartesian（带超时）
        RCLCPP_WARN(this->get_logger(), "[4DOF策略] 分段下压失败，快速尝试Cartesian（带超时）");
        if (execute_cartesian(waypoints, true))  // true 表示 descend
        {
          descend_success = true;
        }
        else
        {
          // Fallback 2: 普通规划（直接到目标点）
          RCLCPP_WARN(this->get_logger(), "[4DOF策略] Cartesian失败，尝试普通规划fallback");
          if (plan_execute_pose_target(grasp_pose))
          {
            descend_success = true;
          }
        }
      }
    }
    else
    {
      // 非4DOF：保持原逻辑（优先Cartesian，失败后fallback到分段下压）
      if (use_cartesian_)
      {
        if (execute_cartesian(waypoints, true))  // true 表示 descend
        {
          descend_success = true;
        }
        else
        {
          // Fallback 1: 分段下压（使用IK→joint target）
          RCLCPP_WARN(this->get_logger(), "笛卡尔路径失败，尝试分段下压fallback（IK→joint target）");
          if (execute_segmented_descend(pregrasp_pose, grasp_pose, 3))
          {
            descend_success = true;
          }
          else
          {
            // Fallback 2: 普通规划（直接到目标点）
            RCLCPP_WARN(this->get_logger(), "分段下压失败，尝试普通规划fallback");
            if (plan_execute_pose_target(grasp_pose))
            {
              descend_success = true;
            }
          }
        }
      }
      else
      {
        // 不使用Cartesian，直接使用分段下压或普通规划
        RCLCPP_INFO(this->get_logger(), "使用分段下压（IK→joint target）");
        if (execute_segmented_descend(pregrasp_pose, grasp_pose, 3))
        {
          descend_success = true;
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), "分段下压失败，尝试普通规划fallback");
          if (plan_execute_pose_target(grasp_pose))
          {
            descend_success = true;
          }
        }
      }
    }

    if (!descend_success)
    {
      publish_state("错误:下压失败");
      return false;
    }

    // 6. 闭合夹爪
    if (emergency_stop_) {
      RCLCPP_WARN(this->get_logger(), "急停触发，停止夹爪闭合");
      publish_state("急停:任务已停止");
      return false;
    }
    
    publish_state("夹爪:闭合中");
    if (!close_gripper())
    {
      publish_state("错误:闭合夹爪失败");
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
    
    std::vector<geometry_msgs::msg::Pose> lift_waypoints;
    lift_waypoints.push_back(grasp_pose.pose);  // 已经在planning_frame_中
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
      RCLCPP_WARN(this->get_logger(), "抬起失败，但继续尝试附着物体");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "抬起成功完成");
    }

    // 9. 成功判定（简化：如果抬起不报错就认为成功）
    // TODO: 可以添加夹爪反馈宽度检查

    // 10. 附着物体（传入cable_pose_planning以包含完整几何信息）
    publish_state("场景:附着");
    // 使用 cable_pose_planning（已经在 planning frame 中）
    // 注意：cable_world_id 和 cable_attached_id 已在函数开始处声明
    // attach 前先 remove world object
    planning_scene_interface_->removeCollisionObjects({cable_world_id});
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    // attach 新的 attached_id（注意不是 world_id）
    if (!scene_attach(cable_attached_id, eef_link_, cable_pose_planning))
    {
      RCLCPP_WARN(this->get_logger(), "附着物体失败");
    }

    // 11. 完成抓取
    publish_state("完成");
    RCLCPP_INFO(this->get_logger(), "缆绳抓取完成");
    return true;
  }

  // 成员变量
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cable_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr emergency_stop_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_interface_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread executor_thread_;
  std::shared_ptr<rclcpp::Node> node_self_ptr_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string planning_frame_;
  std::string eef_link_;

  std::queue<geometry_msgs::msg::PoseStamped> task_queue_;
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
  double cartesian_descend_threshold_{0.95};
  double cartesian_lift_threshold_{0.90};
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
