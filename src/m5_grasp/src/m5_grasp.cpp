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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/time.h>
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
    RCLCPP_INFO(this->get_logger(), "发布话题: /grasp_state");
    RCLCPP_INFO(this->get_logger(), "  消息类型: std_msgs::msg::String");
    RCLCPP_INFO(this->get_logger(), "  队列大小: 10");
    RCLCPP_INFO(this->get_logger(), "等待缆绳位置消息...");
    RCLCPP_INFO(this->get_logger(), "========================================");
    
    // 验证订阅器是否创建成功
    if (cable_pose_sub_) {
      RCLCPP_INFO(this->get_logger(), "✓ 订阅器创建成功");
    } else {
      RCLCPP_ERROR(this->get_logger(), "✗ 订阅器创建失败！");
    }
    
    if (state_publisher_) {
      RCLCPP_INFO(this->get_logger(), "✓ 发布器创建成功");
    } else {
      RCLCPP_ERROR(this->get_logger(), "✗ 发布器创建失败！");
    }
  }

  ~M5Grasp()
  {
    // 停止工作线程
    stop_workers_ = true;
    queue_cv_.notify_all();
    for (auto& t : worker_threads_)
      if (t.joinable()) t.join();

    // 停止 executor
    if (executor_)
    {
      if (node_self_ptr_)
      {
        executor_->remove_node(node_self_ptr_);
      }
      executor_->cancel();
    }

    if (executor_thread_.joinable())
      executor_thread_.join();
  }

  // 初始化MoveIt接口（在对象创建后调用）
  void init_moveit()
  {
    RCLCPP_INFO(this->get_logger(), "开始初始化MoveIt接口...");
    
    // MoveIt会从robot_description topic获取参数，不需要等待参数
    // 但需要等待robot_state_publisher启动（通常由launch文件保证）
    // 等待一小段时间确保robot_state_publisher已启动
    RCLCPP_INFO(this->get_logger(), "等待robot_state_publisher启动...");
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));  // 等待2秒
    
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
    const int max_attempts = 50;  // 最多等待 5 秒（50 * 0.1s）
    const int wait_ms = 100;
    bool joint_states_ready = false;
    
    for (int i = 0; i < max_attempts; ++i)
    {
      try
      {
        // 尝试获取当前状态（0.1s timeout）
        auto state = move_group_interface_->getCurrentState(0.1);
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

    // 设置规划参数
    default_planning_time_ = 15.0;
    move_group_interface_->setPlanningTime(default_planning_time_);
    move_group_interface_->setNumPlanningAttempts(30);
    move_group_interface_->setMaxVelocityScalingFactor(0.9);
    move_group_interface_->setMaxAccelerationScalingFactor(0.9);
    move_group_interface_->setGoalPositionTolerance(0.01);
    move_group_interface_->setGoalOrientationTolerance(0.5);
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
      auto arm_jv = move_group_interface_->getCurrentJointValues();
      auto grip_jv = gripper_group_interface_->getCurrentJointValues();
      
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
    
    // 等待一小段时间确保executor开始运行
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 验证订阅器是否已注册
    if (cable_pose_sub_) {
      RCLCPP_INFO(this->get_logger(), "✓ 订阅器已注册到executor");
    } else {
      RCLCPP_ERROR(this->get_logger(), "✗ 订阅器未注册！");
    }
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

    // Gripper parameters (position control only, no force control)
    this->declare_parameter("gripper.mode", "position");
    this->declare_parameter("gripper.open_width", 0.060);
    this->declare_parameter("gripper.close_width", 0.010);
    this->declare_parameter("gripper.close_extra", 0.002);
    this->declare_parameter("gripper.hold_time_ms", 200);

    // Scene parameters
    this->declare_parameter("scene.add_collision_object", true);
    this->declare_parameter("scene.allow_touch_links", std::vector<std::string>{"LinkGG", "LinkGL", "LinkGR"});

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

    gripper_mode_ = this->get_parameter("gripper.mode").as_string();
    gripper_open_width_ = this->get_parameter("gripper.open_width").as_double();
    gripper_close_width_ = this->get_parameter("gripper.close_width").as_double();
    gripper_close_extra_ = this->get_parameter("gripper.close_extra").as_double();
    gripper_hold_time_ms_ = this->get_parameter("gripper.hold_time_ms").as_int();

    add_collision_object_ = this->get_parameter("scene.add_collision_object").as_bool();
    allow_touch_links_ = this->get_parameter("scene.allow_touch_links").as_string_array();

    RCLCPP_INFO(this->get_logger(), "参数加载完成");
    RCLCPP_INFO(this->get_logger(), "缆绳直径: %.3f m, 长度: %.3f m", cable_diameter_, cable_length_);
    RCLCPP_INFO(this->get_logger(), "夹爪打开宽度: %.3f m, 闭合宽度: %.3f m", gripper_open_width_, gripper_close_width_);
  }

  // 缆绳位置回调
  void cable_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
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
      publish_state("error:not_ready");
      return;
    }

    publish_state("received");

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
      state_publisher_->publish(msg);
      RCLCPP_INFO(this->get_logger(), "发布状态: %s", msg.data.c_str());
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
        try {
          do_cable_grasp(task);
        } catch (const std::exception& e) {
          RCLCPP_ERROR(this->get_logger(), "处理抓取任务时发生异常: %s", e.what());
          publish_state("error:exception");
        }
      }
    }
    RCLCPP_INFO(this->get_logger(), "工作线程退出");
  }

  // 宽度到角度转换（需要根据实际夹爪几何测试确定）
  double width_to_joint_angle(double width)
  {
    // 线性映射
    // width: 0.0 (完全闭合) -> gripper_open_width_ (完全张开)
    // JointGL: 1.01 (闭合) -> -1.01 (张开)
    // 根据axis5映射：axis5 [0°, -1100°] -> JointGL [1.01, -1.01] 弧度
    // 闭合时 axis5=0° → JointGL=1.01 rad，打开时 axis5=-1100° → JointGL=-1.01 rad
    const double width_min = 0.0;
    const double width_max = gripper_open_width_;
    const double angle_min = 1.01;   // 闭合（对应 axis5 = 0°）
    const double angle_max = -1.01;  // 张开（对应 axis5 = -1100°）
    
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
  bool scene_attach(const std::string& object_id, const std::string& eef_link, 
                    const geometry_msgs::msg::PoseStamped& cable_pose)
  {
    std::lock_guard<std::mutex> lock(moveit_mutex_);

    // 重新构造碰撞体的完整几何信息（确保attach时包含几何）
    moveit_msgs::msg::AttachedCollisionObject attached_object;
    attached_object.object.id = object_id;
    attached_object.object.operation = moveit_msgs::msg::CollisionObject::ADD;
    attached_object.object.header.frame_id = planning_frame_;
    
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
    
    // 使用统一函数计算圆柱pose（确保与add时一致，避免attach时物体跳变）
    geometry_msgs::msg::Pose cylinder_pose = make_cable_cylinder_pose(pose_in_planning);
    attached_object.object.primitive_poses.push_back(cylinder_pose);
    
    attached_object.link_name = eef_link;
    attached_object.touch_links = allow_touch_links_;

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

  // 转换位姿到planning frame（通用函数）
  // 优先使用 pose.header.stamp，失败则 fallback 到 tf2::TimePointZero
  // 这样可以处理视觉延迟（优先用stamp），同时避免时间戳不同步导致的 extrapolation 错误
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
      try
      {
        rclcpp::Time transform_time(pose.header.stamp);
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
        // stamp 失败，fallback 到最新 TF
        RCLCPP_DEBUG(this->get_logger(), 
                     "使用 stamp 转换失败（可能时间戳不同步），fallback 到最新 TF: %s", 
                     ex.what());
      }
    }

    // Fallback：使用 tf2::TimePointZero 获取最新 TF
    // 这样可以避免时间戳不同步导致的 extrapolation 错误
    try
    {
      geometry_msgs::msg::TransformStamped transform = 
          tf_buffer_->lookupTransform(planning_frame_, 
                                     pose.header.frame_id, 
                                     tf2::TimePointZero);
      tf2::doTransform(pose, pose, transform);
      pose.header.frame_id = planning_frame_;
      return true;
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
        auto current_pose = move_group_interface_->getCurrentPose();
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
    return (std::abs(orientation.x) < 1e-6 &&
            std::abs(orientation.y) < 1e-6 &&
            std::abs(orientation.z) < 1e-6 &&
            std::abs(orientation.w - 1.0) < 1e-6);
  }

  // 检查位置是否在工作空间内（参考m5_planning.cpp的实现）
  bool is_reachable(double x, double y, double z)
  {
    // URDF基本参数（用于计算明显超出边界的点）
    const double base_height = 0.141;  // Joint1在base_link上的高度
    const double link2_length = 0.264;  // Joint2到Joint3的距离
    const double link3_length = 0.143;  // Joint3到Joint4的距离（Z方向）
    const double link4_to_eef = 0.187;  // Joint4到LinkGG的距离
    
    // 计算理论最大伸展距离（完全伸展时，保守估计）
    const double max_reach_radius = link2_length + link3_length + link4_to_eef;  // 约0.594m
    const double max_reach_radius_with_margin = max_reach_radius * 1.2;  // 留20%余量，避免误杀
    
    // 计算理论最大高度（完全伸展时，保守估计）
    // 提高阈值以覆盖实际可达高度（当前实际可达z=0.809，避免误判）
    const double max_height = base_height + 0.9;  // 约1.041m，覆盖实际可达高度
    const double min_height = base_height - 0.3;  // 保守估计最小高度（base_height - 约0.3m）
    
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
    // 高成功率区域（推荐使用）
    const double safe_x_min = 0.20;
    const double safe_x_max = 0.35;
    const double safe_y_min = -0.15;
    const double safe_y_max = 0.15;
    const double safe_z_min = 0.25;
    const double safe_z_max = 0.35;
    
    // 中等成功率区域（如果高成功率区域不可用）
    const double medium_x_min = 0.15;
    const double medium_x_max = 0.45;
    const double medium_y_min = -0.20;
    const double medium_y_max = 0.20;
    const double medium_z_min = 0.20;
    const double medium_z_max = 0.40;
    
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
  bool check_ik_only(const geometry_msgs::msg::PoseStamped& pose_in_planning)
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
    
    auto state = move_group_interface_->getCurrentState(1.0);
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

    // 0.1s IK timeout
    bool ok = state->setFromIK(jmg, p, eef_link_, 0.1);
    
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
        jmg, pose_in_planning.pose, eef_link_, 0.1,
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
          auto current_pose = move_group_interface_->getCurrentPose();
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
        auto current_pose = move_group_interface_->getCurrentPose();
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

  // 执行笛卡尔路径
  // use_descend: true 表示下压（descend），false 表示抬起（lift）
  // descend 阈值 0.95，lift 阈值 0.90（lift 是确认动作，可以更宽松）
  // 添加超时机制（3秒），避免 computeCartesianPath 卡住
  bool execute_cartesian(const std::vector<geometry_msgs::msg::Pose>& waypoints, bool use_descend = true)
  {
    // 使用 std::async 在单独线程中执行 computeCartesianPath，带超时
    // 注意：computeCartesianPath 需要访问 move_group_interface_，需要在锁内执行
    auto future = std::async(std::launch::async, [this, &waypoints]() -> std::pair<double, moveit_msgs::msg::RobotTrajectory> {
      std::lock_guard<std::mutex> lock(moveit_mutex_);
      move_group_interface_->setStartStateToCurrentState();
      
      // 尊重参数值：jump_threshold=0 表示关闭跳跃检查（MoveIt约定）
      // 不要强制改成非零值，否则可能导致 fraction 下降
      moveit_msgs::msg::RobotTrajectory trajectory;
      double fraction = move_group_interface_->computeCartesianPath(
          waypoints, eef_step_, jump_threshold_, trajectory);
      return {fraction, trajectory};
    });
    
    // 等待结果，带超时（3秒）
    if (future.wait_for(std::chrono::seconds(3)) == std::future_status::timeout)
    {
      RCLCPP_WARN(this->get_logger(), "笛卡尔路径计算超时（3秒），触发fallback");
      return false;  // 超时，触发 fallback
    }
    
    // 获取结果
    auto result_pair = future.get();
    double fraction = result_pair.first;
    moveit_msgs::msg::RobotTrajectory trajectory = result_pair.second;

    // 根据用途设置不同的阈值
    // descend（下压）：0.95（真实硬件上容易 0.95~0.98）
    // lift（抬起）：0.90（lift 是确认动作，可以更宽松）
    double threshold = use_descend ? 0.95 : 0.90;
    if (fraction < threshold)
    {
      RCLCPP_WARN(this->get_logger(), 
                  "笛卡尔路径规划失败，成功率: %.2f%%，要求 >= %.2f%%，尝试fallback", 
                  fraction * 100.0, threshold * 100.0);
      return false;
    }

    // 执行轨迹（需要在锁内执行）
    {
      std::lock_guard<std::mutex> lock(moveit_mutex_);
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

      if (!plan_execute_ik_joint_target(waypoints[i]))
      {
        RCLCPP_WARN(this->get_logger(), 
                    "[分段下压] 段 %zu/%zu 执行失败", i + 1, waypoints.size());
        return false;
      }
      
      // 等待状态稳定（每段执行后都等待，确保状态同步）
      RCLCPP_INFO(this->get_logger(), 
                  "[分段下压] 段 %zu/%zu 执行成功，等待150ms确保状态稳定...", i + 1, waypoints.size());
      std::this_thread::sleep_for(std::chrono::milliseconds(150));
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

    // 逐段执行IK→joint target规划
    // 跳过第一段（i=0），因为它是起点，已经在那个位置，不需要移动
    for (size_t i = 1; i < waypoints.size(); ++i)
    {
      RCLCPP_INFO(this->get_logger(), 
                  "[分段提升] 执行段 %zu/%zu: 位置=(%.3f, %.3f, %.3f)",
                  i + 1, waypoints.size(),
                  waypoints[i].pose.position.x,
                  waypoints[i].pose.position.y,
                  waypoints[i].pose.position.z);

      if (!plan_execute_ik_joint_target(waypoints[i]))
      {
        RCLCPP_WARN(this->get_logger(), 
                    "[分段提升] 段 %zu/%zu 执行失败", i + 1, waypoints.size());
        return false;
      }
      
      // 等待状态稳定（每段执行后都等待，确保状态同步）
      RCLCPP_INFO(this->get_logger(), 
                  "[分段提升] 段 %zu/%zu 执行成功，等待150ms确保状态稳定...", i + 1, waypoints.size());
      std::this_thread::sleep_for(std::chrono::milliseconds(150));
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

      // IK求解（0.1s timeout）
      if (state->setFromIK(jmg, pose_in_planning.pose, eef_link_, 0.1))
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

    // 使用joint target进行规划
    move_group_interface_->setStartStateToCurrentState();
    move_group_interface_->clearPoseTargets();
    move_group_interface_->clearPathConstraints();
    move_group_interface_->setJointValueTarget(joint_values);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode result = move_group_interface_->plan(plan);

    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "[IK→Joint] Joint target规划成功");
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

  // 点对点运动（重构：优先使用IK→joint target，这是4DOF机械臂的主路径）
  // 对4DOF机械臂：默认走 IK→joint（OMPL的Pose/Position目标约束采样器可能无法构建可采样的goal region）
  // 对6DOF机械臂：可以优先 pose/position
  bool plan_execute_pose_target(const geometry_msgs::msg::PoseStamped& target_pose)
  {
    std::lock_guard<std::mutex> lock(moveit_mutex_);

    // 转换到planning frame（使用通用函数）
    geometry_msgs::msg::PoseStamped pose_in_planning = target_pose;
    if (!transform_pose_to_planning(pose_in_planning))
    {
      return false;
    }

    // 输出关键信息用于诊断
    RCLCPP_INFO(this->get_logger(), 
                "PlanningFrame=%s, EEF=%s", 
                planning_frame_.c_str(), eef_link_.c_str());
    RCLCPP_INFO(this->get_logger(), 
                "Target in planning: pos=(%.3f %.3f %.3f) frame=%s",
                pose_in_planning.pose.position.x, 
                pose_in_planning.pose.position.y, 
                pose_in_planning.pose.position.z,
                pose_in_planning.header.frame_id.c_str());

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
    move_group_interface_->setPlanningTime(2.0);  // 只给2秒，快速失败
    
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
    // 保存原始orientation
    geometry_msgs::msg::Quaternion original_orientation = pose_in_planning.pose.orientation;
    
    // 准备orientation候选列表（按优先级排序）
    std::vector<geometry_msgs::msg::Quaternion> orientation_candidates;
    
    // 候选1: 当前末端执行器orientation（如果可用且不是默认值）
    if (move_group_interface_)
    {
      try {
        auto current_pose = move_group_interface_->getCurrentPose();
        if (!is_default_orientation(current_pose.pose.orientation))
        {
          orientation_candidates.push_back(current_pose.pose.orientation);
          RCLCPP_INFO(this->get_logger(), "准备尝试orientation候选: 当前末端执行器orientation");
        }
      } catch (const std::exception& e) {
        // 忽略错误
      }
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
    
    // 临时降低 planning time，避免在 OMPL 里浪费太多时间
    // 使用成员变量而不是 getPlanningTime()（MoveIt2 某些版本没有此方法）
    move_group_interface_->setPlanningTime(2.0);  // 只给2秒，快速失败
    
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
      RCLCPP_INFO(this->get_logger(),
                  "对orientation候选 #%zu 进行IK检查...",
                  i + 1);
      if (!check_ik_only(pose_in_planning)) {
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

    publish_state("received");

    // 0. 检查当前位置是否在工作空间内（仅打印警告，不强制移动）
    // 注意：当前姿态是URDF/TF认可的合法姿态，不应在抓取任务中将其当作错误处理
    // 如果后续需要安全策略，应该基于实际IK可达性而非简单的高度/半径检查
    if (move_group_interface_)
    {
      try {
        auto current_pose = move_group_interface_->getCurrentPose();
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
        auto current_joints = move_group_interface_->getCurrentJointValues();
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
    publish_state("gripper:opening");
    if (!open_gripper())
    {
      publish_state("error:open_gripper");
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
      publish_state("error:transform_cable");
      return false;
    }

    // 3. 移动到预抓取点
    publish_state("planning:pregrasp");
    if (!plan_execute_pose_target(pregrasp_pose))
    {
      publish_state("error:pregrasp");
      return false;
    }

    // 4. 添加缆绳碰撞体（在pregrasp成功之后，避免被碰撞体阻挡）
    // 使用 cable_pose_planning（已经在 planning frame 中）
    if (add_collision_object_)
    {
      if (!scene_add_cable_object(cable_pose_planning, cable_name_))
      {
        publish_state("error:add_object");
        return false;
      }
    }
    publish_state("executing:pregrasp");

    // 5. 直线下压（现在waypoints已经在planning_frame_中）
    publish_state("executing:descend");
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
      publish_state("error:descend");
      return false;
    }

    // 6. 闭合夹爪
    publish_state("gripper:closing");
    if (!close_gripper())
    {
      publish_state("error:close_gripper");
      return false;
    }

    // 7. 保持时间
    publish_state("gripper:holding");
    std::this_thread::sleep_for(std::chrono::milliseconds(gripper_hold_time_ms_));

    // 8. 轻微抬起确认
    publish_state("executing:lift");
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
    publish_state("scene:attach");
    // 使用 cable_pose_planning（已经在 planning frame 中）
    if (!scene_attach(cable_name_, eef_link_, cable_pose_planning))
    {
      RCLCPP_WARN(this->get_logger(), "附着物体失败");
    }

    // 11. 完成
    publish_state("completed");
    RCLCPP_INFO(this->get_logger(), "缆绳抓取完成");
    return true;
  }

  // 成员变量
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cable_pose_sub_;
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
  size_t num_worker_threads_{1};

  // 参数
  std::string cable_name_;
  double cable_diameter_;
  double cable_length_;
  std::string cable_frame_id_;
  std::string cable_shape_;
  double cable_center_offset_z_{0.0};  // cable_pose到圆柱中心的Z偏移（默认0.0表示cable_pose是中心）
  
  double default_planning_time_{15.0};  // 默认规划时间，用于临时切换后恢复

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

  bool add_collision_object_;
  std::vector<std::string> allow_touch_links_;
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
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      
      RCLCPP_INFO(rclcpp::get_logger("main"), "初始化MoveIt接口...");
      node->init_moveit();
      RCLCPP_INFO(rclcpp::get_logger("main"), "MoveIt接口已初始化");
      
      RCLCPP_INFO(rclcpp::get_logger("main"), "m5_grasp节点已完全启动，等待消息...");

      while (rclcpp::ok())
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
