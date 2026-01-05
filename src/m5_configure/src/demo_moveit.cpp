#include <memory>
#include <functional>
#include <thread>
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
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/time.h>
#include <chrono>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_tools.h>

class MoveItDemo : public rclcpp::Node
{
public:
  MoveItDemo() : Node("demo_moveit")
  {
    // 初始化 TF2 buffer 和 listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 订阅目标位姿话题
    target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/target_pose", 10, 
        std::bind(&MoveItDemo::target_pose_callback, this, std::placeholders::_1));

    // 发布状态到网页（用于显示规划中、执行中等状态）
    state_publisher_ = this->create_publisher<std_msgs::msg::String>("/robot_state", 10);

    RCLCPP_INFO(this->get_logger(), "demo_moveit节点已启动，等待目标位姿消息...");
    RCLCPP_INFO(this->get_logger(), "订阅话题: /target_pose");
    RCLCPP_INFO(this->get_logger(), "发布话题: /robot_state");
  }

  ~MoveItDemo()
  {
    // 1) 先停 worker，避免执行中调用 MoveIt action 导致 executor 卡住
    stop_workers_ = true;
    queue_cv_.notify_all();
    for (auto& t : worker_threads_)
      if (t.joinable()) t.join();

    // 2) 停 executor
    if (executor_)
    {
      executor_->cancel();
      // 使用保存的 shared_ptr 而不是 shared_from_this()（析构函数中不安全）
      if (node_self_ptr_)
      {
        executor_->remove_node(node_self_ptr_);
      }
    }

    // 3) join executor thread
    if (executor_thread_.joinable())
      executor_thread_.join();
  }

  


  // 初始化MoveIt接口（在对象创建后调用）
  void init_moveit()
  {
    // 初始化 MoveIt 接口（此时shared_from_this()可以安全使用）
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "arm_group");
    RCLCPP_INFO(this->get_logger(), "MoveIt接口已初始化");

    // 参考成熟实现：设置规划参数（优化以提高成功率）
    move_group_interface_->setPlanningTime(15.0);  // 从10.0增加到15.0秒，给规划器更多时间
    move_group_interface_->setNumPlanningAttempts(30);  // 从20增加到30次，提高成功率
    move_group_interface_->setMaxVelocityScalingFactor(0.9);
    move_group_interface_->setMaxAccelerationScalingFactor(0.9);
    move_group_interface_->setGoalPositionTolerance(0.01);   // 1cm位置容差（从1mm增加到1cm，匹配epsilon=0.2）
    // 对于4DOF机械臂，由于position_only_ik=true，orientation容差需要大幅放宽
    // 参考成熟实现使用0.01，但对于4DOF需要更大容差以提高IK成功率
    move_group_interface_->setGoalOrientationTolerance(0.5); // ~28.6度方向容差（大幅放宽以提高4DOF机械臂成功率）
    move_group_interface_->allowReplanning(true);
    
    move_group_interface_->setPlanningPipelineId("ompl");
    move_group_interface_->setPlannerId("RRTConnect");
    
    RCLCPP_INFO(this->get_logger(), "规划器配置: 规划时间=15.0s, 尝试次数=30");
    RCLCPP_INFO(this->get_logger(), "速度/加速度缩放: 0.9");
    RCLCPP_INFO(this->get_logger(), "目标位置容差: 0.01 m (1cm，匹配epsilon=0.2)");
    RCLCPP_INFO(this->get_logger(), "目标姿态容差: 0.5 rad (~28.6度，4DOF机械臂大幅放宽容差)");
    
    // 获取planning frame（由SRDF中的virtual_joint决定）
    planning_frame_ = move_group_interface_->getPlanningFrame();
    RCLCPP_INFO(this->get_logger(), "Planning frame: %s (由SRDF virtual_joint决定)", planning_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "RViz配置: fixed frame=world_link, robot_description=robot_description, planning_scene_topic=monitored_planning_scene");
    RCLCPP_INFO(this->get_logger(), "支持的坐标系: world (planning frame), world_link (RViz fixed frame), base_link");
    
    // 参考成熟实现：不使用setPoseReferenceFrame，让MoveIt使用默认的planning frame
    // MoveIt会自动处理坐标系转换（world/world_link/base_link -> planning frame）
    
    // 从RobotModel中获取group的tip_link（从SRDF配置中读取）
    const moveit::core::RobotModelConstPtr& robot_model = move_group_interface_->getRobotModel();
    const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup("arm_group");
    if (jmg && !jmg->getLinkModelNames().empty())
    {
      // 获取tip link（group中最后一个link）
      const std::vector<std::string>& link_names = jmg->getLinkModelNames();
      std::string tip_link = link_names.back();
      move_group_interface_->setEndEffectorLink(tip_link);
      RCLCPP_INFO(this->get_logger(), "从SRDF配置读取tip_link: %s", tip_link.c_str());
    }
    else
    {
      // 如果无法从group获取，使用getEndEffectorLink()（可能返回空字符串）
      std::string eef_link = move_group_interface_->getEndEffectorLink();
      if (eef_link.empty())
      {
        RCLCPP_WARN(this->get_logger(), "无法从SRDF获取tip_link，使用默认值LinkGG");
        move_group_interface_->setEndEffectorLink("LinkGG");
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "使用MoveIt默认的end effector link: %s", eef_link.c_str());
      }
    }
    
    RCLCPP_INFO(this->get_logger(), "末端执行器链接: %s", move_group_interface_->getEndEffectorLink().c_str());
    
    // 验证IK求解器配置
    verify_ik_solver_config();
    
    // 配置总结
    RCLCPP_INFO(this->get_logger(), "=== MoveIt配置总结（参考成熟实现）===");
    RCLCPP_INFO(this->get_logger(), "Planning Frame: %s (由SRDF virtual_joint决定)", planning_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "End Effector Link: %s", move_group_interface_->getEndEffectorLink().c_str());
    RCLCPP_INFO(this->get_logger(), "Planning Pipeline: %s", move_group_interface_->getPlanningPipelineId().c_str());
    RCLCPP_INFO(this->get_logger(), "Planner ID: %s", move_group_interface_->getPlannerId().c_str());
    RCLCPP_INFO(this->get_logger(), "Planning Time: %.1f s", move_group_interface_->getPlanningTime());
    RCLCPP_INFO(this->get_logger(), "Planning Attempts: 10 (已设置)");
    RCLCPP_INFO(this->get_logger(), "Max Velocity Scaling: 0.9 (已设置)");
    RCLCPP_INFO(this->get_logger(), "Max Acceleration Scaling: 0.9 (已设置)");
    RCLCPP_INFO(this->get_logger(), "Goal Position Tolerance: 0.001 m (已设置)");
    RCLCPP_INFO(this->get_logger(), "Goal Orientation Tolerance: 0.5 rad (已设置，4DOF机械臂大幅放宽容差)");
    RCLCPP_INFO(this->get_logger(), "Allow Replanning: true (已设置)");
    RCLCPP_INFO(this->get_logger(), "=== 配置总结结束 ===");

    // 启动工作线程池
    stop_workers_ = false;
    for (size_t i = 0; i < num_worker_threads_; ++i)
    {
      worker_threads_.emplace_back(&MoveItDemo::worker_thread, this);
      RCLCPP_INFO(this->get_logger(), "工作线程 %zu 已启动", i + 1);
    }
    RCLCPP_INFO(this->get_logger(), "工作线程池已启动，共 %zu 个线程", num_worker_threads_);

    RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group_interface_->getPlanningFrame().c_str());
    RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group_interface_->getEndEffectorLink().c_str());

  }

  // 启动executor（在对象创建后调用）
  void start_executor()
  {
    // 保存 shared_ptr<Node> 用于析构函数中安全调用 remove_node
    node_self_ptr_ = shared_from_this();
    
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_self_ptr_);
    executor_thread_ = std::thread([this]() {
      try {
        executor_->spin();
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Executor thread exception: %s", e.what());
      }
    });
    
    RCLCPP_INFO(this->get_logger(), "Executor线程已启动");
  }


private:
  void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!move_group_interface_) {
      RCLCPP_WARN(this->get_logger(), "MoveGroupInterface not ready yet, dropping target_pose");
      this->publish_state("error"); // 发布错误状态：MoveIt未准备好
      return;
    }
    
    // 发布状态：收到目标点
    this->publish_state("received");

    // 参考成熟实现：简化处理，验证frame_id
    // MoveIt会自动处理坐标系转换
    // 根据SRDF配置，planning frame是"world"（virtual_joint的parent_frame）
    // RViz fixed frame是"world_link"，两者通过static_transform_publisher连接
    geometry_msgs::msg::PoseStamped target_pose = *msg;
    
    // 支持的frame_id：world（planning frame）、world_link（RViz fixed frame）、base_link
    // 如果frame_id为空或不支持，根据planning frame设置为world
    std::string default_frame = planning_frame_.empty() ? "world" : planning_frame_;
    
    if (target_pose.header.frame_id.empty()) {
      target_pose.header.frame_id = default_frame;
      RCLCPP_INFO(this->get_logger(), "目标位姿frame_id为空，已自动设置为planning frame: %s", 
                  target_pose.header.frame_id.c_str());
    } else if (target_pose.header.frame_id != "world" && 
               target_pose.header.frame_id != "world_link" && 
               target_pose.header.frame_id != "base_link") {
      // 如果是不支持的frame，记录警告并使用planning frame
      // MoveIt的setPoseTarget会自动处理坐标系转换
      RCLCPP_WARN(this->get_logger(), "目标位姿frame_id不是推荐的坐标系 (%s)，MoveIt将自动转换到planning frame: %s", 
                  target_pose.header.frame_id.c_str(), default_frame.c_str());
      // 保持原始frame_id，让MoveIt处理转换（MoveIt支持从任何frame转换到planning frame）
    }
    
    // 更新时间戳（如果为0，使用当前时间）
    if (target_pose.header.stamp.sec == 0 && target_pose.header.stamp.nanosec == 0) {
      target_pose.header.stamp = this->now();
    }

    RCLCPP_INFO(this->get_logger(), "收到目标位姿: x=%.3f y=%.3f z=%.3f (frame: %s)",
                target_pose.pose.position.x,
                target_pose.pose.position.y,
                target_pose.pose.position.z,
                target_pose.header.frame_id.c_str());

    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      while (!task_queue_.empty()) task_queue_.pop();
      task_queue_.push(target_pose);
    }
    queue_cv_.notify_one();
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread executor_thread_;
  
  // 保存 shared_ptr<Node> 用于析构函数中安全调用 remove_node
  std::shared_ptr<rclcpp::Node> node_self_ptr_;

  // TF2 变换支持
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // 缓存的 planning frame（避免 callback 访问 MoveGroupInterface）
  std::string planning_frame_;

  // 任务队列相关
  std::queue<geometry_msgs::msg::PoseStamped> task_queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;

  // MoveIt接口保护（MoveGroupInterface不是线程安全的）
  std::mutex moveit_mutex_;

  // 工作线程池
  std::vector<std::thread> worker_threads_;
  std::atomic<bool> stop_workers_{false};
  size_t num_worker_threads_{1};  // 默认1个线程（排队处理）

  // IK求解器配置验证函数：获取并输出IK求解器实际使用的配置
  void verify_ik_solver_config()
  {
    RCLCPP_INFO(this->get_logger(), "=== IK求解器配置验证 ===");
    
    const moveit::core::RobotModelConstPtr& robot_model = move_group_interface_->getRobotModel();
    const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup("arm_group");
    if (!jmg)
    {
      RCLCPP_ERROR(this->get_logger(), "无法获取JointModelGroup: arm_group");
      return;
    }
    
    // 获取base frame和tip frame
    std::string base_frame = jmg->getSolverInstance()->getBaseFrame();
    std::vector<std::string> tip_frames = jmg->getSolverInstance()->getTipFrames();
    
    RCLCPP_INFO(this->get_logger(), "IK求解器配置:");
    RCLCPP_INFO(this->get_logger(), "  Base Frame: %s", base_frame.c_str());
    RCLCPP_INFO(this->get_logger(), "  Tip Frame(s):");
    for (const auto& tip : tip_frames)
    {
      RCLCPP_INFO(this->get_logger(), "    - %s", tip.c_str());
    }
    
    // 获取group的link信息
    const std::vector<std::string>& link_names = jmg->getLinkModelNames();
    
    // 尝试获取group的实际base link（通过检查第一个joint的parent link）
    std::string group_base_link = base_frame; // 默认使用IK求解器的base_frame
    if (!jmg->getJointModelNames().empty())
    {
      const moveit::core::JointModel* first_joint = jmg->getJointModelNames()[0].empty() ? 
          nullptr : robot_model->getJointModel(jmg->getJointModelNames()[0]);
      if (first_joint)
      {
        group_base_link = first_joint->getParentLinkModel()->getName();
        RCLCPP_INFO(this->get_logger(), "  Group实际Base Link (第一个joint的parent): %s", group_base_link.c_str());
      }
    }
    
    if (!link_names.empty())
    {
      RCLCPP_INFO(this->get_logger(), "  Group Link Chain (从SRDF):");
      RCLCPP_INFO(this->get_logger(), "    Base Link (link_names.front()): %s", link_names.front().c_str());
      RCLCPP_INFO(this->get_logger(), "    Tip Link: %s", link_names.back().c_str());
      RCLCPP_INFO(this->get_logger(), "    完整链: %s -> ... -> %s", 
                  link_names.front().c_str(), link_names.back().c_str());
      
      // 输出所有links以便调试
      RCLCPP_INFO(this->get_logger(), "    所有links (%zu个):", link_names.size());
      for (size_t i = 0; i < link_names.size() && i < 10; ++i)  // 只显示前10个
      {
        RCLCPP_INFO(this->get_logger(), "      [%zu] %s", i, link_names[i].c_str());
      }
      if (link_names.size() > 10)
      {
        RCLCPP_INFO(this->get_logger(), "      ... (还有 %zu 个links)", link_names.size() - 10);
      }
    }
    
    // 验证base_frame和tip_frame是否与SRDF一致
    // 关键：IK求解器的base_frame应该与group的实际base link一致
    // getLinkModelNames()可能不包含base_link（如果它没有可动关节），所以使用第一个joint的parent link
    if (base_frame != group_base_link)
    {
      RCLCPP_ERROR(this->get_logger(), "❌ 致命错误：IK求解器base_frame (%s) 与SRDF group base_link (%s) 不一致！",
                  base_frame.c_str(), link_names.front().c_str());
      RCLCPP_ERROR(this->get_logger(), "   这会导致OMPL规划器无法采样目标状态（RRTConnect: Unable to sample any valid states for goal tree）");
      RCLCPP_ERROR(this->get_logger(), "   原因：OMPL使用SRDF group的base_link (%s)，但IK求解器使用base_frame (%s)",
                  link_names.front().c_str(), base_frame.c_str());
      RCLCPP_ERROR(this->get_logger(), "   解决方案：确保SRDF中arm_group的chain base_link是base_link，而不是Link1");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "  ✅ Base frame与SRDF group base_link一致: %s", base_frame.c_str());
    }
    
    if (!tip_frames.empty() && tip_frames[0] != link_names.back())
    {
      RCLCPP_WARN(this->get_logger(), "警告：IK求解器tip_frame (%s) 与SRDF group tip_link (%s) 不一致！",
                  tip_frames[0].c_str(), link_names.back().c_str());
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "  Tip frame与SRDF group tip_link一致");
    }
    
    // 尝试从参数服务器读取kinematics配置
    std::string kinematics_prefix = "robot_description_kinematics.arm_group";
    RCLCPP_INFO(this->get_logger(), "尝试读取kinematics配置参数 (前缀: %s):", kinematics_prefix.c_str());
    
    // 读取kinematics配置参数
    std::vector<std::string> param_names = {
      kinematics_prefix + ".kinematics_solver",
      kinematics_prefix + ".kinematics_solver_timeout",
      kinematics_prefix + ".position_only_ik",
      kinematics_prefix + ".epsilon",
      kinematics_prefix + ".solve_type"
    };
    
    for (const auto& param_name : param_names)
    {
      if (this->has_parameter(param_name))
      {
        rclcpp::Parameter param = this->get_parameter(param_name);
        RCLCPP_INFO(this->get_logger(), "  %s = %s", param_name.c_str(), 
                    param.value_to_string().c_str());
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "  参数 %s 不存在", param_name.c_str());
      }
    }
    
    RCLCPP_INFO(this->get_logger(), "=== IK求解器配置验证结束 ===");
  }
  
  // 正向运动学验证函数：计算给定关节角度的末端执行器位置
  void verify_forward_kinematics(const moveit::core::RobotStatePtr& state)
  {
    RCLCPP_INFO(this->get_logger(), "=== 正向运动学验证 ===");
    
    const moveit::core::RobotModelConstPtr& robot_model = move_group_interface_->getRobotModel();
    const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup("arm_group");
    if (!jmg)
    {
      RCLCPP_ERROR(this->get_logger(), "无法获取JointModelGroup: arm_group");
      return;
    }
    
    // 获取关节角度
    std::vector<double> joint_values;
    state->copyJointGroupPositions(jmg, joint_values);
    const std::vector<std::string>& joint_names = jmg->getJointModelNames();
    
    RCLCPP_INFO(this->get_logger(), "输入关节角度:");
    for (size_t i = 0; i < joint_names.size() && i < joint_values.size(); ++i)
    {
      RCLCPP_INFO(this->get_logger(), "  %s = %.6f rad (%.2f°)", 
                  joint_names[i].c_str(), joint_values[i], joint_values[i] * 180.0 / M_PI);
    }
    
    // 使用MoveIt计算正向运动学
    std::string eef_link = move_group_interface_->getEndEffectorLink();
    if (eef_link.empty())
    {
      eef_link = "LinkGG";
    }
    
    const Eigen::Isometry3d& eef_transform = state->getGlobalLinkTransform(eef_link);
    Eigen::Vector3d eef_position = eef_transform.translation();
    Eigen::Quaterniond eef_orientation(eef_transform.rotation());
    
    RCLCPP_INFO(this->get_logger(), "MoveIt计算的末端执行器位置 (frame: %s):", planning_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  位置: (%.6f, %.6f, %.6f)", 
                eef_position.x(), eef_position.y(), eef_position.z());
    RCLCPP_INFO(this->get_logger(), "  姿态: (%.6f, %.6f, %.6f, %.6f)",
                eef_orientation.x(), eef_orientation.y(), 
                eef_orientation.z(), eef_orientation.w());
    
    // 使用MoveIt的正向运动学服务验证（更准确）
    // 获取base_link坐标系下的末端执行器位置
    std::string base_link = "base_link";
    try
    {
      const Eigen::Isometry3d& base_transform = state->getGlobalLinkTransform(base_link);
      const Eigen::Isometry3d& eef_in_base = base_transform.inverse() * eef_transform;
      Eigen::Vector3d eef_in_base_pos = eef_in_base.translation();
      
      RCLCPP_INFO(this->get_logger(), "末端执行器在base_link坐标系中的位置:");
      RCLCPP_INFO(this->get_logger(), "  位置: (%.6f, %.6f, %.6f)", 
                  eef_in_base_pos.x(), eef_in_base_pos.y(), eef_in_base_pos.z());
    }
    catch (const std::exception& e)
    {
      RCLCPP_WARN(this->get_logger(), "无法计算base_link坐标系下的位置: %s", e.what());
    }
    
    // 根据URDF参数手动计算理论位置（考虑所有偏移）
    // 注意：这是精确计算，考虑所有URDF偏移
    if (joint_values.size() >= 4)
    {
      double j1 = joint_values[0];  // Joint1: 绕Z轴旋转
      double j2 = joint_values[1];  // Joint2: 绕X轴旋转（-1.57到0）
      double j3 = joint_values[2];  // Joint3: 绕X轴旋转（-1.57到0）
      double j4 = joint_values[3];  // Joint4: 绕Z轴旋转
      
      // URDF参数（从URDF提取，精确值）
      const double base_height = 0.141;  // Joint1在base_link上的高度
      
      // Joint2在Link1上的偏移
      const double j2_offset_x = -0.048;
      const double j2_offset_y = 0.0525;
      const double j2_offset_z = 0.0735;
      
      // Link2长度（Joint2到Joint3）
      const double link2_length = 0.264;
      
      // Joint4在Link3上的偏移
      const double j4_offset_x = 0.048;
      const double j4_offset_y = -0.0525;
      const double j4_offset_z = 0.143;  // Joint3到Joint4的Z距离
      
      // LinkGG在Link4上的偏移
      const double link4_to_eef = 0.187;  // Joint4到LinkGG的距离
      
      // 正向运动学计算（考虑所有偏移和旋转）
      // 从base_link开始
      Eigen::Vector3d pos(0, 0, base_height);  // Joint1位置
      
      // Joint1旋转（绕Z轴）
      Eigen::Matrix3d rot1 = Eigen::AngleAxisd(j1, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      
      // Joint2位置（在Link1坐标系中）
      Eigen::Vector3d j2_pos_local(j2_offset_x, j2_offset_y, j2_offset_z);
      Eigen::Vector3d j2_pos_world = pos + rot1 * j2_pos_local;
      
      // Joint2旋转（绕X轴，在Link1坐标系中）
      Eigen::Matrix3d rot2_local = Eigen::AngleAxisd(j2, Eigen::Vector3d::UnitX()).toRotationMatrix();
      Eigen::Matrix3d rot2_world = rot1 * rot2_local;
      
      // Link2向量（在Link2坐标系中，沿Z方向）
      Eigen::Vector3d link2_vec_local(0, 0, link2_length);
      Eigen::Vector3d link2_vec_world = rot2_world * link2_vec_local;
      Eigen::Vector3d j3_pos = j2_pos_world + link2_vec_world;
      
      // Joint3旋转（绕X轴，在Link2坐标系中）
      Eigen::Matrix3d rot3_local = Eigen::AngleAxisd(j3, Eigen::Vector3d::UnitX()).toRotationMatrix();
      Eigen::Matrix3d rot3_world = rot2_world * rot3_local;
      
      // Joint4位置（在Link3坐标系中）
      Eigen::Vector3d j4_offset_local(j4_offset_x, j4_offset_y, j4_offset_z);
      Eigen::Vector3d j4_pos = j3_pos + rot3_world * j4_offset_local;
      
      // Joint4旋转（绕Z轴，在Link3坐标系中）
      Eigen::Matrix3d rot4_local = Eigen::AngleAxisd(j4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      Eigen::Matrix3d rot4_world = rot3_world * rot4_local;
      
      // LinkGG位置（在Link4坐标系中，沿Z方向）
      Eigen::Vector3d eef_offset_local(0, 0, link4_to_eef);
      Eigen::Vector3d eef_pos_calculated = j4_pos + rot4_world * eef_offset_local;
      
      RCLCPP_INFO(this->get_logger(), "理论计算位置（考虑所有URDF偏移）:");
      RCLCPP_INFO(this->get_logger(), "  位置: (%.6f, %.6f, %.6f)", 
                  eef_pos_calculated.x(), eef_pos_calculated.y(), eef_pos_calculated.z());
      
      // 计算位置差异
      double pos_diff = std::sqrt(
        std::pow(eef_position.x() - eef_pos_calculated.x(), 2) +
        std::pow(eef_position.y() - eef_pos_calculated.y(), 2) +
        std::pow(eef_position.z() - eef_pos_calculated.z(), 2)
      );
      RCLCPP_INFO(this->get_logger(), "  位置差异: %.6f m", pos_diff);
      
      if (pos_diff > 0.01)  // 如果差异大于1cm，输出警告
      {
        RCLCPP_WARN(this->get_logger(), "位置差异较大 (%.6f m)，可能的原因:", pos_diff);
        RCLCPP_WARN(this->get_logger(), "  1. 坐标系不匹配（MoveIt使用world_link，计算使用base_link）");
        RCLCPP_WARN(this->get_logger(), "  2. URDF参数与实际模型不一致");
        RCLCPP_WARN(this->get_logger(), "  3. 正向运动学计算有误");
      }
    }
    
    RCLCPP_INFO(this->get_logger(), "=== 正向运动学验证结束 ===");
  }
  
  // 直接IK测试：使用MoveIt的KinematicsBase直接测试IK求解，绕过MoveGroupInterface
  // 如果成功，通过solution参数返回求解得到的关节角度
  bool test_direct_ik(const geometry_msgs::msg::Pose& target_pose, 
                     const std::vector<double>& seed_state,
                     const std::string& base_frame,
                     const std::string& tip_frame,
                     std::vector<double>& solution)
  {
    RCLCPP_INFO(this->get_logger(), "=== 直接IK测试（绕过MoveGroupInterface） ===");
    RCLCPP_INFO(this->get_logger(), "Base frame: %s", base_frame.c_str());
    RCLCPP_INFO(this->get_logger(), "Tip frame: %s", tip_frame.c_str());
    RCLCPP_INFO(this->get_logger(), "目标位置: (%.6f, %.6f, %.6f)",
                target_pose.position.x, target_pose.position.y, target_pose.position.z);
    
    const moveit::core::RobotModelConstPtr& robot_model = move_group_interface_->getRobotModel();
    const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup("arm_group");
    if (!jmg)
    {
      RCLCPP_ERROR(this->get_logger(), "无法获取JointModelGroup: arm_group");
      return false;
    }
    
    // 获取IK求解器实例
    const kinematics::KinematicsBaseConstPtr& solver = jmg->getSolverInstance();
    if (!solver)
    {
      RCLCPP_ERROR(this->get_logger(), "无法获取IK求解器实例");
      return false;
    }
    
    // 准备IK求解参数
    solution.clear();  // 清空输出参数
    moveit_msgs::msg::MoveItErrorCodes error_code;
    kinematics::KinematicsQueryOptions options;
    
    RCLCPP_INFO(this->get_logger(), "调用IK求解器getPositionIK...");
    auto ik_start_time = std::chrono::steady_clock::now();
    bool ik_success = solver->getPositionIK(target_pose, seed_state, solution, error_code, options);
    auto ik_end_time = std::chrono::steady_clock::now();
    auto ik_duration = std::chrono::duration_cast<std::chrono::milliseconds>(ik_end_time - ik_start_time);
    RCLCPP_INFO(this->get_logger(), "直接IK求解耗时: %ld ms", ik_duration.count());
    
    if (ik_success)
    {
      RCLCPP_INFO(this->get_logger(), "直接IK求解成功！");
      RCLCPP_INFO(this->get_logger(), "错误代码: %d (1=SUCCESS)", error_code.val);
      RCLCPP_INFO(this->get_logger(), "求解得到的关节角度:");
      const std::vector<std::string>& joint_names = jmg->getJointModelNames();
      for (size_t i = 0; i < joint_names.size() && i < solution.size(); ++i)
      {
        RCLCPP_INFO(this->get_logger(), "  %s = %.6f rad (%.2f°)", 
                    joint_names[i].c_str(), solution[i], solution[i] * 180.0 / M_PI);
      }
      RCLCPP_INFO(this->get_logger(), "=== 直接IK测试结束（成功）===");
      return true;
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "直接IK求解失败");
      RCLCPP_WARN(this->get_logger(), "错误代码: %d", error_code.val);
      RCLCPP_WARN(this->get_logger(), "错误代码含义: SUCCESS=1, NO_IK_SOLUTION=-31, TIMED_OUT=-32, INVALID_LINK_NAME=-33");
      RCLCPP_INFO(this->get_logger(), "=== 直接IK测试结束（失败）===");
      solution.clear();  // 失败时清空solution
      return false;
    }
  }
  
  // 测试已知可达位置：验证IK求解器是否正常工作
  bool test_known_reachable_positions(const moveit::core::RobotStatePtr& start_state)
  {
    RCLCPP_INFO(this->get_logger(), "=== 测试已知可达位置（验证IK求解器） ===");
    
    // 根据文档，推荐测试位置
    std::vector<std::tuple<double, double, double, std::string>> test_positions = {
      {0.30, 0.00, 0.30, "中心前方（推荐起始位置）"},
      {0.25, 0.00, 0.35, "中心前方（高）"},
      {0.35, 0.00, 0.25, "中心前方（低）"},
      {0.20, 0.00, 0.30, "前方近"},
    };
    
    bool all_passed = true;
    for (const auto& [x, y, z, desc] : test_positions)
    {
      RCLCPP_INFO(this->get_logger(), "测试位置: %s (%.3f, %.3f, %.3f)", desc.c_str(), x, y, z);
      
      geometry_msgs::msg::PoseStamped test_pose;
      test_pose.header.frame_id = planning_frame_;
      test_pose.header.stamp = this->now();
      test_pose.pose.position.x = x;
      test_pose.pose.position.y = y;
      test_pose.pose.position.z = z;
      test_pose.pose.orientation.w = 1.0;  // 默认姿态
      
      bool ik_success = test_ik_solver(test_pose, start_state);
      if (ik_success)
      {
        RCLCPP_INFO(this->get_logger(), "  ✓ IK求解成功");
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "  ✗ IK求解失败（这可能表明IK求解器配置有问题）");
        all_passed = false;
      }
    }
    
    if (all_passed)
    {
      RCLCPP_INFO(this->get_logger(), "所有已知可达位置测试通过，IK求解器工作正常");
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "部分已知可达位置测试失败，IK求解器可能配置不当");
    }
    
    RCLCPP_INFO(this->get_logger(), "=== 已知可达位置测试结束 ===");
    return all_passed;
  }
  
  // IK求解器测试函数：独立验证目标位姿是否可以通过IK求解
  bool test_ik_solver(const geometry_msgs::msg::PoseStamped& target_pose, 
                      const moveit::core::RobotStatePtr& start_state)
  {
    RCLCPP_INFO(this->get_logger(), "=== IK求解器测试 ===");
    
    const moveit::core::RobotModelConstPtr& robot_model = move_group_interface_->getRobotModel();
    const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup("arm_group");
    if (!jmg)
    {
      RCLCPP_ERROR(this->get_logger(), "无法获取JointModelGroup: arm_group");
      return false;
    }
    
    // 创建目标状态
    moveit::core::RobotState target_state(*start_state);
    
    // 获取末端执行器link
    std::string eef_link = move_group_interface_->getEndEffectorLink();
    if (eef_link.empty())
    {
      eef_link = "LinkGG";
    }
    
    // 获取IK求解器使用的base_frame
    std::string ik_base_frame = jmg->getSolverInstance()->getBaseFrame();
    RCLCPP_INFO(this->get_logger(), "IK求解器base_frame: %s", ik_base_frame.c_str());
    RCLCPP_INFO(this->get_logger(), "目标位姿frame_id: %s", target_pose.header.frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "Planning frame: %s", planning_frame_.c_str());
    
    // 关键修复：IK求解器期望目标位姿在base_frame坐标系中，而不是planning frame
    // 需要将目标位姿从planning frame转换到base_frame
    geometry_msgs::msg::PoseStamped pose_in_planning_frame = target_pose;
    
    // 首先转换到planning frame（如果需要）
    if (pose_in_planning_frame.header.frame_id != planning_frame_)
    {
      try
      {
        geometry_msgs::msg::TransformStamped transform = 
            tf_buffer_->lookupTransform(planning_frame_, 
                                       pose_in_planning_frame.header.frame_id, 
                                       tf2::TimePointZero);
        tf2::doTransform(pose_in_planning_frame, pose_in_planning_frame, transform);
        pose_in_planning_frame.header.frame_id = planning_frame_;
        RCLCPP_INFO(this->get_logger(), "已转换目标位姿到planning frame: %s", planning_frame_.c_str());
      }
      catch (const tf2::TransformException& ex)
      {
        RCLCPP_ERROR(this->get_logger(), "IK测试：转换到planning frame失败: %s", ex.what());
        return false;
      }
    }
    
    // 然后转换到IK求解器的base_frame（如果不同）
    geometry_msgs::msg::PoseStamped pose_for_ik = pose_in_planning_frame;
    if (ik_base_frame != planning_frame_)
    {
      try
      {
        RCLCPP_INFO(this->get_logger(), "转换目标位姿从planning frame (%s) 到IK base_frame (%s)",
                    planning_frame_.c_str(), ik_base_frame.c_str());
        geometry_msgs::msg::TransformStamped transform = 
            tf_buffer_->lookupTransform(ik_base_frame, 
                                       pose_in_planning_frame.header.frame_id, 
                                       tf2::TimePointZero);
        tf2::doTransform(pose_in_planning_frame, pose_for_ik, transform);
        pose_for_ik.header.frame_id = ik_base_frame;
        RCLCPP_INFO(this->get_logger(), "已转换目标位姿到IK base_frame: %s", ik_base_frame.c_str());
        RCLCPP_INFO(this->get_logger(), "转换后位置: (%.6f, %.6f, %.6f)",
                    pose_for_ik.pose.position.x, pose_for_ik.pose.position.y, pose_for_ik.pose.position.z);
      }
      catch (const tf2::TransformException& ex)
      {
        RCLCPP_ERROR(this->get_logger(), "IK测试：转换到IK base_frame失败: %s", ex.what());
        RCLCPP_ERROR(this->get_logger(), "尝试使用planning frame中的位姿（MoveIt可能会自动转换）");
        pose_for_ik = pose_in_planning_frame;
      }
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "IK base_frame与planning frame相同，无需额外转换");
    }
    
    // 输出IK求解前的详细信息
    RCLCPP_INFO(this->get_logger(), "IK求解参数:");
    RCLCPP_INFO(this->get_logger(), "  原始目标位置 (frame: %s): (%.6f, %.6f, %.6f)", 
                target_pose.header.frame_id.c_str(),
                target_pose.pose.position.x,
                target_pose.pose.position.y,
                target_pose.pose.position.z);
    RCLCPP_INFO(this->get_logger(), "  用于IK的目标位置 (frame: %s): (%.6f, %.6f, %.6f)", 
                pose_for_ik.header.frame_id.c_str(),
                pose_for_ik.pose.position.x,
                pose_for_ik.pose.position.y,
                pose_for_ik.pose.position.z);
    RCLCPP_INFO(this->get_logger(), "  目标姿态: (%.6f, %.6f, %.6f, %.6f)",
                pose_for_ik.pose.orientation.x,
                pose_for_ik.pose.orientation.y,
                pose_for_ik.pose.orientation.z,
                pose_for_ik.pose.orientation.w);
    RCLCPP_INFO(this->get_logger(), "  末端执行器link: %s", eef_link.c_str());
    RCLCPP_INFO(this->get_logger(), "  IK base_frame: %s", ik_base_frame.c_str());
    RCLCPP_INFO(this->get_logger(), "  超时时间: 10.0秒");
    
    // 输出起始状态信息
    std::vector<double> start_joints;
    start_state->copyJointGroupPositions(jmg, start_joints);
    RCLCPP_INFO(this->get_logger(), "起始关节角度:");
    const std::vector<std::string>& joint_names = jmg->getJointModelNames();
    for (size_t i = 0; i < joint_names.size() && i < start_joints.size(); ++i)
    {
      RCLCPP_INFO(this->get_logger(), "  %s = %.6f rad (%.2f°)", 
                  joint_names[i].c_str(), start_joints[i], start_joints[i] * 180.0 / M_PI);
    }
    
    // 关键发现：MoveIt的setFromIK会自动处理坐标系转换！
    // 根据MoveIt文档，setFromIK期望的pose应该在planning frame中，MoveIt会自动转换到IK求解器的base_frame
    // 所以我们应该使用planning frame中的位姿，而不是手动转换
    // 但为了诊断，我们先尝试使用转换后的位姿
    
    // 获取当前末端执行器在base_link坐标系中的位置（用于对比）
    std::string eef_link_name = eef_link;
    const Eigen::Isometry3d& current_eef_transform = start_state->getGlobalLinkTransform(eef_link_name);
    const Eigen::Isometry3d& base_transform = start_state->getGlobalLinkTransform(ik_base_frame);
    const Eigen::Isometry3d& eef_in_base = base_transform.inverse() * current_eef_transform;
    Eigen::Vector3d current_eef_pos_in_base = eef_in_base.translation();
    RCLCPP_INFO(this->get_logger(), "当前末端执行器在base_link坐标系中的位置: (%.6f, %.6f, %.6f)",
                current_eef_pos_in_base.x(), current_eef_pos_in_base.y(), current_eef_pos_in_base.z());
    
    // 关键修复：MoveIt的setFromIK期望planning frame中的位姿，会自动转换到IK求解器的base_frame
    // 根据MoveIt源码，setFromIK内部会处理坐标系转换
    RCLCPP_INFO(this->get_logger(), "开始IK求解（使用planning frame中的位姿，MoveIt会自动转换）...");
    RCLCPP_INFO(this->get_logger(), "Planning frame: %s, IK base_frame: %s", planning_frame_.c_str(), ik_base_frame.c_str());
    auto ik_start_time = std::chrono::steady_clock::now();
    bool ik_found = target_state.setFromIK(jmg, pose_in_planning_frame.pose, eef_link, 10.0);
    auto ik_end_time = std::chrono::steady_clock::now();
    auto ik_duration = std::chrono::duration_cast<std::chrono::milliseconds>(ik_end_time - ik_start_time);
    RCLCPP_INFO(this->get_logger(), "IK求解耗时: %ld ms", ik_duration.count());
    
    if (ik_found)
    {
      RCLCPP_INFO(this->get_logger(), "IK求解成功！");
      
      // 输出求解得到的关节角度
      std::vector<double> joint_values;
      target_state.copyJointGroupPositions(jmg, joint_values);
      RCLCPP_INFO(this->get_logger(), "IK求解得到的关节角度:");
      for (size_t i = 0; i < joint_names.size() && i < joint_values.size(); ++i)
      {
        RCLCPP_INFO(this->get_logger(), "  %s = %.6f rad (%.2f°)", 
                    joint_names[i].c_str(), joint_values[i], joint_values[i] * 180.0 / M_PI);
        
        // 检查关节限制
        const moveit::core::JointModel* joint = robot_model->getJointModel(joint_names[i]);
        if (joint && joint->getVariableCount() > 0)
        {
          const moveit::core::VariableBounds& bounds = joint->getVariableBounds()[0];
          if (joint_values[i] < bounds.min_position_ || joint_values[i] > bounds.max_position_)
          {
            RCLCPP_WARN(this->get_logger(), "  警告：关节 %s 的值 %.6f 超出限制范围 [%.6f, %.6f]",
                       joint_names[i].c_str(), joint_values[i], 
                       bounds.min_position_, bounds.max_position_);
          }
        }
      }
      
      // 验证实际末端执行器位姿
      const Eigen::Isometry3d& eef_transform = target_state.getGlobalLinkTransform(eef_link);
      Eigen::Vector3d eef_position = eef_transform.translation();
      Eigen::Quaterniond eef_orientation(eef_transform.rotation());
      
      RCLCPP_INFO(this->get_logger(), "IK求解后的实际末端执行器位姿:");
      RCLCPP_INFO(this->get_logger(), "  位置: (%.6f, %.6f, %.6f)", 
                  eef_position.x(), eef_position.y(), eef_position.z());
      RCLCPP_INFO(this->get_logger(), "  姿态: (%.6f, %.6f, %.6f, %.6f)",
                  eef_orientation.x(), eef_orientation.y(), 
                  eef_orientation.z(), eef_orientation.w());
      
      // 计算位置误差
      double pos_error = std::sqrt(
        std::pow(eef_position.x() - pose_in_planning_frame.pose.position.x, 2) +
        std::pow(eef_position.y() - pose_in_planning_frame.pose.position.y, 2) +
        std::pow(eef_position.z() - pose_in_planning_frame.pose.position.z, 2)
      );
      RCLCPP_INFO(this->get_logger(), "  位置误差: %.6f m", pos_error);
      
      RCLCPP_INFO(this->get_logger(), "=== IK求解器测试结束（成功）===");
      return true;
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "IK求解失败！无法找到满足目标位姿的关节配置");
      RCLCPP_WARN(this->get_logger(), "目标位姿 (frame: %s): position=(%.6f, %.6f, %.6f)",
                  pose_for_ik.header.frame_id.c_str(),
                  pose_for_ik.pose.position.x,
                  pose_for_ik.pose.position.y,
                  pose_for_ik.pose.position.z);
      RCLCPP_WARN(this->get_logger(), "原始目标位姿 (frame: %s): position=(%.6f, %.6f, %.6f)",
                  target_pose.header.frame_id.c_str(),
                  target_pose.pose.position.x,
                  target_pose.pose.position.y,
                  target_pose.pose.position.z);
      RCLCPP_INFO(this->get_logger(), "=== IK求解器测试结束（失败）===");
      return false;
    }
  }
  
  // 碰撞检测测试函数：检查机器人状态是否有碰撞
  bool test_collision(const moveit::core::RobotStatePtr& state, const std::string& state_name)
  {
    RCLCPP_INFO(this->get_logger(), "=== 碰撞检测测试: %s ===", state_name.c_str());
    
    // 创建规划场景
    const moveit::core::RobotModelConstPtr& robot_model = move_group_interface_->getRobotModel();
    planning_scene::PlanningScene planning_scene(robot_model);
    
    // 设置机器人状态
    planning_scene.setCurrentState(*state);
    
    // 检查自碰撞
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    collision_request.contacts = true;
    collision_request.max_contacts = 100;
    
    planning_scene.checkSelfCollision(collision_request, collision_result);
    
    if (collision_result.collision)
    {
      RCLCPP_WARN(this->get_logger(), "检测到自碰撞！");
      RCLCPP_WARN(this->get_logger(), "碰撞接触点数量: %zu", collision_result.contacts.size());
      
      // 输出碰撞的link对
      for (const auto& contact : collision_result.contacts)
      {
        RCLCPP_WARN(this->get_logger(), "  碰撞: %s <-> %s", 
                    contact.first.first.c_str(), contact.first.second.c_str());
        if (contact.second.size() > 0)
        {
          RCLCPP_WARN(this->get_logger(), "    接触点位置: (%.6f, %.6f, %.6f)",
                      contact.second[0].pos.x(), contact.second[0].pos.y(), contact.second[0].pos.z());
        }
      }
      
      RCLCPP_INFO(this->get_logger(), "=== 碰撞检测测试结束（有碰撞）===");
      return true;  // 返回true表示有碰撞
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "无自碰撞");
      
      // 检查与环境的碰撞（如果有障碍物）
      collision_detection::CollisionRequest env_collision_request;
      collision_detection::CollisionResult env_collision_result;
      env_collision_request.contacts = true;
      env_collision_request.max_contacts = 100;
      
      planning_scene.checkCollision(env_collision_request, env_collision_result);
      
      if (env_collision_result.collision)
      {
        RCLCPP_WARN(this->get_logger(), "检测到与环境障碍物的碰撞！");
        RCLCPP_WARN(this->get_logger(), "碰撞接触点数量: %zu", env_collision_result.contacts.size());
        
        for (const auto& contact : env_collision_result.contacts)
        {
          RCLCPP_WARN(this->get_logger(), "  碰撞: %s <-> %s", 
                      contact.first.first.c_str(), contact.first.second.c_str());
        }
        
        RCLCPP_INFO(this->get_logger(), "=== 碰撞检测测试结束（有环境碰撞）===");
        return true;  // 返回true表示有碰撞
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "无环境碰撞");
        RCLCPP_INFO(this->get_logger(), "=== 碰撞检测测试结束（无碰撞）===");
        return false;  // 返回false表示无碰撞
      }
    }
  }
  
  // 可达性快速过滤函数（基于URDF实际参数和关节限制的精确判定）
  // 考虑Joint2和Joint3的限制（只能向下弯曲-1.57到0），进行更精确的工作空间验证
  bool is_reachable(double x, double y, double z)
  {
    // URDF实际参数（从m5_updated_from_csv.urdf提取）
    const double base_height = 0.141;  // Joint1在base_link上的高度
    const double link2_length = 0.264;  // Joint2到Joint3的距离
    const double link3_length = 0.143;  // Joint3到Joint4的距离（Z方向）
    const double link4_to_eef = 0.187;  // Joint4到LinkGG的距离
    
    // Joint偏移参数
    const double j2_offset_x = -0.048;
    const double j2_offset_y = 0.0525;
    const double j2_offset_z = 0.0735;
    const double j4_offset_x = 0.048;
    const double j4_offset_y = -0.0525;
    const double j4_offset_z = 0.143;
    
    // 计算理论最大伸展距离（完全伸展时）
    const double max_reach = link2_length + link3_length + link4_to_eef;  // 约0.594m
    const double min_reach = 0.05;  // 最小距离（考虑关节偏移和基座）
    
    // 计算到基座的距离（半径）
    const double radius = std::sqrt(x * x + y * y);
    
    // 计算相对高度（相对于Joint1，即base_height）
    const double relative_height = z - base_height;
    
    // 考虑Joint2和Joint3的限制（只能向下弯曲-1.57到0）
    // 这意味着机械臂主要工作在前方和下方区域
    // 当Joint2和Joint3都为0时，机械臂完全伸展，末端执行器在最高位置
    // 当Joint2和Joint3都为-1.57时，机械臂完全折叠，末端执行器在最低位置
    
    // 计算理论最小和最大高度
    // 完全伸展时（Joint2=0, Joint3=0）：高度最大
    const double max_height_theoretical = base_height + j2_offset_z + link2_length + j4_offset_z + link4_to_eef;
    // 完全折叠时（Joint2=-1.57, Joint3=-1.57）：高度最小（简化计算）
    const double min_height_theoretical = base_height - (link2_length + link3_length + link4_to_eef) * 0.5;  // 简化估算
    
    // 计算理论最小和最大半径
    // 完全伸展时：半径最大
    const double max_radius_theoretical = link2_length + link3_length + link4_to_eef;
    // 完全折叠时：半径最小
    const double min_radius_theoretical = 0.05;  // 最小半径估算
    
    // 调试信息
    RCLCPP_INFO(this->get_logger(), "工作空间参数:");
    RCLCPP_INFO(this->get_logger(), "  base_height = %.3f m", base_height);
    RCLCPP_INFO(this->get_logger(), "  link2_length = %.3f m", link2_length);
    RCLCPP_INFO(this->get_logger(), "  link3_length = %.3f m", link3_length);
    RCLCPP_INFO(this->get_logger(), "  link4_to_eef = %.3f m", link4_to_eef);
    RCLCPP_INFO(this->get_logger(), "  max_reach = %.3f m", max_reach);
    RCLCPP_INFO(this->get_logger(), "  理论高度范围: [%.3f, %.3f] m", min_height_theoretical, max_height_theoretical);
    RCLCPP_INFO(this->get_logger(), "  理论半径范围: [%.3f, %.3f] m", min_radius_theoretical, max_radius_theoretical);
    RCLCPP_INFO(this->get_logger(), "目标位置分析:");
    RCLCPP_INFO(this->get_logger(), "  半径 = %.6f m", radius);
    RCLCPP_INFO(this->get_logger(), "  绝对高度 = %.6f m", z);
    RCLCPP_INFO(this->get_logger(), "  相对高度 = %.6f m (相对于base_height)", relative_height);
    
    // 第一级：检查基本几何边界（考虑关节限制）
    // 使用理论计算的范围，但留一些余量
    const double min_radius = min_radius_theoretical * 0.8;   // 最小半径（留20%余量）
    const double max_radius = max_radius_theoretical * 1.1;  // 最大半径（留10%余量）
    const double min_height = min_height_theoretical - 0.05;  // 最小高度（留5cm余量）
    const double max_height = max_height_theoretical + 0.05;  // 最大高度（留5cm余量）
    
    if (radius < min_radius || radius > max_radius || z < min_height || z > max_height)
    {
      RCLCPP_WARN(this->get_logger(),
                  "目标位置 (%.3f, %.3f, %.3f) 超出理论工作空间边界 "
                  "(半径范围: [%.3f, %.3f], 高度范围: [%.3f, %.3f])",
                  x, y, z, min_radius, max_radius, min_height, max_height);
      RCLCPP_WARN(this->get_logger(),
                  "注意：Joint2和Joint3只能向下弯曲(-1.57到0)，限制了工作空间范围");
      return false;
    }
    
    // 第二级：高成功率区域（推荐使用）- 基于实际测试和文档
    const double safe_x_min = 0.15, safe_x_max = 0.40;
    const double safe_y_min = -0.20, safe_y_max = 0.20;
    const double safe_z_min = 0.20, safe_z_max = 0.40;
    
    if (x >= safe_x_min && x <= safe_x_max &&
        y >= safe_y_min && y <= safe_y_max &&
        z >= safe_z_min && z <= safe_z_max)
    {
      RCLCPP_INFO(this->get_logger(), "目标位置在安全区域内（高成功率）");
      return true;  // 在安全区域内，直接通过
    }
    
    // 第三级：中等成功率区域（谨慎使用）
    const double medium_x_min = 0.10, medium_x_max = 0.50;
    const double medium_y_min = -0.25, medium_y_max = 0.25;
    const double medium_z_min = 0.15, medium_z_max = 0.45;
    
    if (x >= medium_x_min && x <= medium_x_max &&
        y >= medium_y_min && y <= medium_y_max &&
        z >= medium_z_min && z <= medium_z_max)
    {
      RCLCPP_INFO(this->get_logger(), "目标位置在中等成功率区域内");
      return true;  // 在中等区域内，允许尝试
    }
    
    // 第四级：其它区域 → 如果通过了基本几何边界检查，允许尝试但输出警告
    // 让IK求解器自己判断是否真正可达，而不是在这里做复杂的2D IK验证
    RCLCPP_WARN(this->get_logger(),
                "目标不在推荐区域 (%.3f, %.3f, %.3f)，半径=%.3f, 相对高度=%.3f, "
                "但基本几何验证通过，允许尝试（由IK求解器判断是否真正可达）",
                x, y, z, radius, relative_height);
    return true;
  }

  // 规划处理函数（在工作线程中执行）
  void process_planning_task(const geometry_msgs::msg::PoseStamped& target_pose)
  {
    // 早期退出检查：如果收到 Ctrl+C，立即停止处理
    if (!rclcpp::ok())
    {
      RCLCPP_INFO(this->get_logger(), "收到 shutdown 信号，停止处理规划任务");
      return;
    }

    // 保护MoveGroupInterface的使用（线程安全）
    std::lock_guard<std::mutex> lock(moveit_mutex_);

    double x = target_pose.pose.position.x;
    double y = target_pose.pose.position.y;
    double z = target_pose.pose.position.z;
    
    // 可达性快速过滤（三级判定：超边界拒绝，推荐区域直接通过，其它区域允许但警告）
    RCLCPP_INFO(this->get_logger(), "=== 工作空间验证 ===");
    RCLCPP_INFO(this->get_logger(), "验证目标位置: (%.6f, %.6f, %.6f)", x, y, z);
    
    if (!is_reachable(x, y, z))
    {
      RCLCPP_WARN(this->get_logger(), "目标位置 (%.6f, %.6f, %.6f) 超出工作空间边界，跳过规划",
                  x, y, z);
      RCLCPP_INFO(this->get_logger(), "=== 工作空间验证结束（拒绝）===");
      // 发布状态：拒绝目标点
      this->publish_state("rejected");
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "工作空间验证通过");
    RCLCPP_INFO(this->get_logger(), "=== 工作空间验证结束 ===");
    
    RCLCPP_INFO(this->get_logger(), "开始处理规划任务: x=%.3f, y=%.3f, z=%.3f", x, y, z);
    
    // 发布状态：开始处理（初始化阶段）
    this->publish_state("planning", "初始化");

    // 清理之前可能残留的目标位姿和路径约束，确保从干净状态开始
    move_group_interface_->clearPoseTargets();
    move_group_interface_->clearPathConstraints();
    move_group_interface_->setStartStateToCurrentState(); // 设置起始状态为当前状态

    // 发布状态：获取当前状态
    this->publish_state("planning", "获取当前状态");

    // 获取并打印当前关节状态（用于调试）
    auto state = move_group_interface_->getCurrentState(2.0);
    if (!state)
    {
      RCLCPP_ERROR(this->get_logger(), "getCurrentState() 失败：MoveIt 没拿到有效 joint_states");
      // 发布状态：无效状态
      this->publish_state("invalid");
      return;
    }

    const auto* jmg = state->getJointModelGroup("arm_group");
    if (!jmg)
    {
      RCLCPP_ERROR(this->get_logger(), "找不到 JointModelGroup: arm_group（SRDF group 名可能不一致）");
      // 发布状态：无效状态
      this->publish_state("invalid");
      return;
    }

    std::vector<double> joints;
    state->copyJointGroupPositions(jmg, joints);

    // 详细的关节状态调试信息
    std::ostringstream oss;
    oss << "Current joint positions (rad): ";
    for (double v : joints) oss << v << " ";
    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
    
    // 打印每个关节的名称和值，验证关节顺序
    const std::vector<std::string>& joint_names = jmg->getJointModelNames();
    RCLCPP_INFO(this->get_logger(), "关节顺序和值（MoveIt读取）:");
    for (size_t i = 0; i < joint_names.size() && i < joints.size(); ++i)
    {
      RCLCPP_INFO(this->get_logger(), "  [%zu] %s = %.6f rad (%.2f°)", 
                  i, joint_names[i].c_str(), joints[i], joints[i] * 180.0 / M_PI);
    }
    
    // 验证关节值是否在合理范围内
    bool joints_valid = true;
    for (size_t i = 0; i < joints.size(); ++i)
    {
      if (std::isnan(joints[i]) || std::isinf(joints[i]))
      {
        RCLCPP_ERROR(this->get_logger(), "关节 %s 的值无效: %f", 
                     i < joint_names.size() ? joint_names[i].c_str() : "unknown", joints[i]);
        joints_valid = false;
      }
    }
    
    if (!joints_valid)
    {
      RCLCPP_ERROR(this->get_logger(), "检测到无效的关节值，无法继续规划");
      // 发布状态：无效状态
      this->publish_state("invalid");
      return;
    }
    
    // 验证正向运动学
    verify_forward_kinematics(state);

    // 参考成熟实现：直接使用传入的target_pose，不进行任何修改
    // MoveIt会自动处理从base_link到planning frame的转换
    RCLCPP_INFO(this->get_logger(), "=== 规划参数设置（参考成熟实现）===");
    RCLCPP_INFO(this->get_logger(), "清理之前的目标位姿...");
    move_group_interface_->clearPoseTargets();
    move_group_interface_->setStartStateToCurrentState();
    
    // 对于4DOF机械臂，如果orientation是默认值(0,0,0,1)，使用当前末端执行器的orientation
    geometry_msgs::msg::PoseStamped final_target_pose = target_pose;
    bool is_default_orientation = (std::abs(target_pose.pose.orientation.x) < 1e-6 &&
                                   std::abs(target_pose.pose.orientation.y) < 1e-6 &&
                                   std::abs(target_pose.pose.orientation.z) < 1e-6 &&
                                   std::abs(target_pose.pose.orientation.w - 1.0) < 1e-6);
    
    if (is_default_orientation) {
      try {
        auto current_pose = move_group_interface_->getCurrentPose();
        // 添加详细的调试信息（对比RViz）
        RCLCPP_INFO(this->get_logger(), "=== getCurrentPose() 返回信息（对比RViz）===");
        RCLCPP_INFO(this->get_logger(), "  frame_id: %s", current_pose.header.frame_id.c_str());
        RCLCPP_INFO(this->get_logger(), "  planning_frame: %s", planning_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  frame_id匹配: %s", 
                   (current_pose.header.frame_id == planning_frame_) ? "是" : "否");
        RCLCPP_INFO(this->get_logger(), "  position: (%.6f, %.6f, %.6f)", 
                   current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
        RCLCPP_INFO(this->get_logger(), "  orientation: (%.6f, %.6f, %.6f, %.6f)",
                   current_pose.pose.orientation.x, current_pose.pose.orientation.y,
                   current_pose.pose.orientation.z, current_pose.pose.orientation.w);
        RCLCPP_INFO(this->get_logger(), "=== getCurrentPose() 信息结束 ===");
        
        final_target_pose.pose.orientation = current_pose.pose.orientation;
        RCLCPP_INFO(this->get_logger(), "检测到默认orientation，使用当前末端执行器orientation: (%.3f, %.3f, %.3f, %.3f)",
                    final_target_pose.pose.orientation.x, final_target_pose.pose.orientation.y,
                    final_target_pose.pose.orientation.z, final_target_pose.pose.orientation.w);
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "无法获取当前末端执行器orientation，使用默认值: %s", e.what());
      }
    }
    
    RCLCPP_INFO(this->get_logger(), "目标位姿: position=(%.6f, %.6f, %.6f), frame_id=%s",
                final_target_pose.pose.position.x, final_target_pose.pose.position.y, 
                final_target_pose.pose.position.z, final_target_pose.header.frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "目标位姿orientation: (%.3f, %.3f, %.3f, %.3f)",
                final_target_pose.pose.orientation.x, final_target_pose.pose.orientation.y,
                final_target_pose.pose.orientation.z, final_target_pose.pose.orientation.w);
    
    // 添加详细的调试信息（对比RViz）
    RCLCPP_INFO(this->get_logger(), "=== 传递给setPoseTarget的位姿信息（对比RViz）===");
    RCLCPP_INFO(this->get_logger(), "  frame_id: %s", final_target_pose.header.frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "  planning_frame: %s", planning_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  frame_id匹配: %s", 
               (final_target_pose.header.frame_id == planning_frame_) ? "是" : "否");
    RCLCPP_INFO(this->get_logger(), "  position: (%.6f, %.6f, %.6f)", 
               final_target_pose.pose.position.x, final_target_pose.pose.position.y, 
               final_target_pose.pose.position.z);
    RCLCPP_INFO(this->get_logger(), "  orientation: (%.6f, %.6f, %.6f, %.6f)",
               final_target_pose.pose.orientation.x, final_target_pose.pose.orientation.y,
               final_target_pose.pose.orientation.z, final_target_pose.pose.orientation.w);
    
    // 如果目标位姿的frame_id与planning frame不同，进行转换
    if (final_target_pose.header.frame_id != planning_frame_) {
      RCLCPP_WARN(this->get_logger(), "目标位姿frame_id (%s) 与planning frame (%s) 不匹配，进行转换",
                 final_target_pose.header.frame_id.c_str(), planning_frame_.c_str());
      try {
        // 使用TF2转换到planning frame
        geometry_msgs::msg::TransformStamped transform = 
            tf_buffer_->lookupTransform(planning_frame_, 
                                       final_target_pose.header.frame_id, 
                                       tf2::TimePointZero);
        tf2::doTransform(final_target_pose, final_target_pose, transform);
        final_target_pose.header.frame_id = planning_frame_;
        RCLCPP_INFO(this->get_logger(), "转换完成: frame_id=%s, position=(%.6f, %.6f, %.6f)",
                   final_target_pose.header.frame_id.c_str(),
                   final_target_pose.pose.position.x, final_target_pose.pose.position.y,
                   final_target_pose.pose.position.z);
        RCLCPP_INFO(this->get_logger(), "转换后orientation: (%.6f, %.6f, %.6f, %.6f)",
                   final_target_pose.pose.orientation.x, final_target_pose.pose.orientation.y,
                   final_target_pose.pose.orientation.z, final_target_pose.pose.orientation.w);
      } catch (const tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "坐标系转换失败: %s", ex.what());
        RCLCPP_ERROR(this->get_logger(), "无法从 %s 转换到 %s，规划可能失败", 
                    final_target_pose.header.frame_id.c_str(), planning_frame_.c_str());
        return;
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "目标位姿frame_id与planning frame匹配，无需转换");
    }
    RCLCPP_INFO(this->get_logger(), "=== setPoseTarget前信息结束 ===");
    
    // 参考成熟实现：使用setPoseTarget，MoveIt会自动处理坐标系转换（如果frame_id匹配planning frame）
    move_group_interface_->setPoseTarget(final_target_pose);
    
    // 规划参数已在init_moveit()中设置，这里只输出日志
    RCLCPP_INFO(this->get_logger(), "规划时间: %.1f 秒", move_group_interface_->getPlanningTime());
    RCLCPP_INFO(this->get_logger(), "规划管道: %s", move_group_interface_->getPlanningPipelineId().c_str());
    RCLCPP_INFO(this->get_logger(), "规划器ID: %s", move_group_interface_->getPlannerId().c_str());
    RCLCPP_INFO(this->get_logger(), "末端执行器链接: %s", move_group_interface_->getEndEffectorLink().c_str());
    RCLCPP_INFO(this->get_logger(), "Planning frame: %s", planning_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "=== 规划参数设置完成 ===");

    // 在规划前进行诊断测试
    RCLCPP_INFO(this->get_logger(), "=== 规划前诊断测试 ===");
    
    // 发布状态：诊断测试阶段
    this->publish_state("planning", "诊断测试");
    
    // 0. 验证IK求解器配置并直接IK测试
    // 保存直接IK求解得到的关节角度，用于后续规划
    std::vector<double> direct_ik_solution;
    
    const moveit::core::RobotModelConstPtr& robot_model_for_ik = move_group_interface_->getRobotModel();
    const moveit::core::JointModelGroup* jmg_for_ik_test = robot_model_for_ik->getJointModelGroup("arm_group");
    if (jmg_for_ik_test)
    {
      std::string ik_base_frame = jmg_for_ik_test->getSolverInstance()->getBaseFrame();
      std::vector<std::string> ik_tip_frames = jmg_for_ik_test->getSolverInstance()->getTipFrames();
      RCLCPP_INFO(this->get_logger(), "IK求解器配置: base_frame=%s, tip_frame=%s",
                  ik_base_frame.c_str(), 
                  ik_tip_frames.empty() ? "unknown" : ik_tip_frames[0].c_str());
      
      // 如果IK求解器使用base_link，需要转换目标位姿
      if (ik_base_frame != planning_frame_)
      {
        RCLCPP_WARN(this->get_logger(), "IK求解器base_frame (%s) 与planning frame (%s) 不同，需要转换目标位姿",
                    ik_base_frame.c_str(), planning_frame_.c_str());
      }
      
      // 0.1. 直接IK测试（使用base_link坐标系）
      if (!ik_tip_frames.empty())
      {
        // 发布状态：IK测试阶段
        this->publish_state("planning", "IK测试");
        
        // 转换目标位姿到base_link坐标系
        geometry_msgs::msg::PoseStamped pose_in_base = final_target_pose;
        if (pose_in_base.header.frame_id != ik_base_frame)
        {
          try
          {
            geometry_msgs::msg::TransformStamped transform = 
                tf_buffer_->lookupTransform(ik_base_frame, 
                                           pose_in_base.header.frame_id, 
                                           tf2::TimePointZero);
            tf2::doTransform(pose_in_base, pose_in_base, transform);
            pose_in_base.header.frame_id = ik_base_frame;
            RCLCPP_INFO(this->get_logger(), "已转换目标位姿到base_link坐标系进行直接IK测试");
          }
          catch (const tf2::TransformException& ex)
          {
            RCLCPP_WARN(this->get_logger(), "无法转换到base_link进行直接IK测试: %s", ex.what());
          }
        }
        
        std::vector<double> seed_joints;
        state->copyJointGroupPositions(jmg_for_ik_test, seed_joints);
        bool direct_ik_success = test_direct_ik(pose_in_base.pose, seed_joints, ik_base_frame, ik_tip_frames[0], direct_ik_solution);
        if (direct_ik_success)
        {
          RCLCPP_INFO(this->get_logger(), "直接IK测试成功，说明IK求解器工作正常");
          // direct_ik_solution已经在test_direct_ik中填充，将在后续规划中使用
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), "直接IK测试失败，说明IK求解器可能有问题或目标位置不可达");
          // 发布状态：拒绝目标点（IK不可达）
          this->publish_state("rejected");
          return;
        }
      }
    }
    
    // 0.2. 测试已知可达位置（验证IK求解器是否正常工作）
    // 注释掉以加快规划速度，直接IK测试已经验证了IK求解器工作正常
    // bool ik_solver_works = test_known_reachable_positions(state);
    
    // 1. 测试目标位姿的IK求解（简化版，只测试一次，不进行详细诊断）
    // 注释掉详细测试以加快规划速度，直接IK测试已经验证了IK求解器工作正常
    // 注意：直接IK测试在之前已经完成，如果失败会在那里处理并返回
    // 这里假设IK成功（因为如果失败已经返回了）
    bool ik_success = true;  // 直接IK测试已通过，假设IK成功
    
    // 发布状态：碰撞检测阶段
    this->publish_state("planning", "碰撞检测");
    
    // 2. 测试当前状态的碰撞
    bool start_collision = test_collision(state, "起始状态");
    if (start_collision)
    {
      RCLCPP_WARN(this->get_logger(), "规划前诊断：起始状态存在碰撞，可能影响规划");
    }
    
    // 3. 如果IK成功，测试目标状态的碰撞
    if (ik_success)
    {
      const moveit::core::RobotModelConstPtr& robot_model = move_group_interface_->getRobotModel();
      const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup("arm_group");
      if (jmg)
      {
        moveit::core::RobotState target_state(*state);
        std::string eef_link = move_group_interface_->getEndEffectorLink();
        if (eef_link.empty()) eef_link = "LinkGG";
        
        // 转换目标位姿到planning frame（如果还没转换）
        geometry_msgs::msg::PoseStamped pose_for_ik = final_target_pose;
        if (pose_for_ik.header.frame_id != planning_frame_)
        {
          try
          {
            geometry_msgs::msg::TransformStamped transform = 
                tf_buffer_->lookupTransform(planning_frame_, 
                                           pose_for_ik.header.frame_id, 
                                           tf2::TimePointZero);
            tf2::doTransform(pose_for_ik, pose_for_ik, transform);
            pose_for_ik.header.frame_id = planning_frame_;
          }
          catch (const tf2::TransformException& ex)
          {
            RCLCPP_WARN(this->get_logger(), "无法转换目标位姿进行碰撞检测: %s", ex.what());
          }
        }
        
        if (target_state.setFromIK(jmg, pose_for_ik.pose, eef_link, 10.0))
        {
          moveit::core::RobotStatePtr target_state_ptr = 
              std::make_shared<moveit::core::RobotState>(target_state);
          bool target_collision = test_collision(target_state_ptr, "目标状态");
          if (target_collision)
          {
            RCLCPP_WARN(this->get_logger(), "规划前诊断：目标状态存在碰撞，可能无法规划到该位置");
          }
        }
      }
    }
    
    RCLCPP_INFO(this->get_logger(), "=== 规划前诊断测试结束 ===");
    
    // 发布状态：准备规划
    this->publish_state("planning", "准备规划");
    
    // 目标状态验证和设置：优先使用直接IK求解的结果
    const moveit::core::RobotModelConstPtr& robot_model_for_planning = move_group_interface_->getRobotModel();
    const moveit::core::JointModelGroup* jmg_for_planning = robot_model_for_planning->getJointModelGroup("arm_group");
    bool use_joint_target = false;
    std::vector<double> target_joint_values;
    
    // 优先使用直接IK求解得到的关节角度（如果可用）
    if (jmg_for_planning && !direct_ik_solution.empty())
    {
      // 验证直接IK求解得到的关节角度是否有效
      const std::vector<std::string>& joint_names = jmg_for_planning->getJointModelNames();
      bool joints_valid = true;
      
      if (direct_ik_solution.size() >= joint_names.size())
      {
        for (size_t i = 0; i < joint_names.size() && i < direct_ik_solution.size(); ++i)
        {
          const moveit::core::JointModel* joint_model = jmg_for_planning->getJointModel(joint_names[i]);
          if (joint_model && joint_model->getVariableBounds().size() > 0)
          {
            const moveit::core::VariableBounds& bounds = joint_model->getVariableBounds()[0];
            if (direct_ik_solution[i] < bounds.min_position_ || direct_ik_solution[i] > bounds.max_position_)
            {
              RCLCPP_WARN(this->get_logger(), "直接IK求解得到的关节 %s 角度 %.6f 超出限制范围 [%.6f, %.6f]",
                          joint_names[i].c_str(), direct_ik_solution[i], 
                          bounds.min_position_, bounds.max_position_);
              joints_valid = false;
            }
          }
        }
        
        if (joints_valid)
        {
          target_joint_values = direct_ik_solution;  // 使用直接IK求解的结果
          RCLCPP_INFO(this->get_logger(), "使用直接IK求解得到的关节角度作为目标状态:");
          for (size_t i = 0; i < joint_names.size() && i < target_joint_values.size(); ++i)
          {
            RCLCPP_INFO(this->get_logger(), "  %s = %.6f rad (%.2f°)", 
                        joint_names[i].c_str(), target_joint_values[i], 
                        target_joint_values[i] * 180.0 / M_PI);
          }
        }
      }
    }
    
    // 如果直接IK求解的结果不可用，尝试使用MoveIt的setFromIK
    if (target_joint_values.empty() && jmg_for_planning)
    {
      moveit::core::RobotState target_state_for_validation(*state);
      std::string eef_link = move_group_interface_->getEndEffectorLink();
      if (eef_link.empty()) eef_link = "LinkGG";
      
      // 尝试IK求解获取目标关节角度
      geometry_msgs::msg::PoseStamped pose_for_ik_validation = final_target_pose;
      if (pose_for_ik_validation.header.frame_id != planning_frame_)
      {
        try
        {
          geometry_msgs::msg::TransformStamped transform = 
              tf_buffer_->lookupTransform(planning_frame_, 
                                         pose_for_ik_validation.header.frame_id, 
                                         tf2::TimePointZero);
          tf2::doTransform(pose_for_ik_validation, pose_for_ik_validation, transform);
          pose_for_ik_validation.header.frame_id = planning_frame_;
        }
        catch (const tf2::TransformException& ex)
        {
          RCLCPP_WARN(this->get_logger(), "无法转换目标位姿进行验证: %s", ex.what());
        }
      }
      
      if (target_state_for_validation.setFromIK(jmg_for_planning, pose_for_ik_validation.pose, eef_link, 10.0))
      {
        // IK求解成功，获取关节角度
        target_state_for_validation.copyJointGroupPositions(jmg_for_planning, target_joint_values);
        
        // 验证关节角度是否在限制范围内
        bool joints_valid = true;
        const std::vector<std::string>& joint_names = jmg_for_planning->getJointModelNames();
        for (size_t i = 0; i < joint_names.size() && i < target_joint_values.size(); ++i)
        {
          const moveit::core::JointModel* joint_model = jmg_for_planning->getJointModel(joint_names[i]);
          if (joint_model && joint_model->getVariableBounds().size() > 0)
          {
            const moveit::core::VariableBounds& bounds = joint_model->getVariableBounds()[0];
            if (target_joint_values[i] < bounds.min_position_ || target_joint_values[i] > bounds.max_position_)
            {
              RCLCPP_WARN(this->get_logger(), "目标关节 %s 角度 %.6f 超出限制范围 [%.6f, %.6f]",
                          joint_names[i].c_str(), target_joint_values[i], 
                          bounds.min_position_, bounds.max_position_);
              joints_valid = false;
            }
          }
        }
        
        if (joints_valid)
        {
          RCLCPP_INFO(this->get_logger(), "目标状态验证通过，关节角度有效:");
          for (size_t i = 0; i < joint_names.size() && i < target_joint_values.size(); ++i)
          {
            RCLCPP_INFO(this->get_logger(), "  %s = %.6f rad (%.2f°)", 
                        joint_names[i].c_str(), target_joint_values[i], 
                        target_joint_values[i] * 180.0 / M_PI);
          }
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), "目标状态验证失败，关节角度超出限制");
          target_joint_values.clear();
        }
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "无法通过IK求解获取目标关节角度");
      }
    }
    
    // 采用直接IK+关节角度目标规划策略
    // 流程：视觉 → 目标点(x,y,z) → IK求解 → 关节角度目标 → 执行
    RCLCPP_INFO(this->get_logger(), "=== 使用直接IK求解结果进行关节角度目标规划 ===");
    
    // 检查是否有有效的目标关节角度
    if (target_joint_values.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "无法获取有效的目标关节角度，规划失败");
      RCLCPP_ERROR(this->get_logger(), "可能原因：IK求解失败或关节角度超出限制范围");
      this->publish_state("error");
      return;
    }
    
    // 设置关节角度目标
    move_group_interface_->clearPoseTargets();
    move_group_interface_->setJointValueTarget(target_joint_values);
    RCLCPP_INFO(this->get_logger(), "使用直接IK求解得到的关节角度进行规划");
    
    // 规划器自动切换：如果第一个规划器失败，自动尝试下一个
    std::vector<std::string> planner_ids = {"RRTConnect", "RRTstar", "EST"};
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = false;
    moveit::core::MoveItErrorCode code = moveit::core::MoveItErrorCode::FAILURE;
    
    // 使用关节角度目标进行规划
    for (size_t i = 0; i < planner_ids.size(); ++i)
    {
      move_group_interface_->setPlannerId(planner_ids[i]);
      RCLCPP_INFO(this->get_logger(), "尝试规划器 [%zu/%zu]: %s (关节角度目标)", i + 1, planner_ids.size(), planner_ids[i].c_str());
      
      // 发布状态：规划中（包含规划器名称）
      this->publish_state("planning", planner_ids[i]);
      
      code = move_group_interface_->plan(plan);
      success = (code == moveit::core::MoveItErrorCode::SUCCESS);
      
      if (success)
      {
        RCLCPP_INFO(this->get_logger(), "规划器 %s 成功（关节角度目标）！", planner_ids[i].c_str());
        break;
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "规划器 %s 失败（关节角度目标），错误代码: %d", planner_ids[i].c_str(), code.val);
        
        // 如果规划超时（错误代码-7），发布带规划器信息的状态
        if (code.val == -7)  // TIMED_OUT
        {
          this->publish_state("planning", planner_ids[i]);
        }
      }
    }

    if (success)
    {
      RCLCPP_INFO(this->get_logger(), "=== 关节角度目标规划成功 ===");
      RCLCPP_INFO(this->get_logger(), "轨迹包含 %zu 个点", 
                  plan.trajectory_.joint_trajectory.points.size());
      
      // 记录轨迹的第一个和最后一个点（用于调试）
      if (plan.trajectory_.joint_trajectory.points.size() > 0)
      {
        const auto& first_point = plan.trajectory_.joint_trajectory.points[0];
        const auto& last_point = plan.trajectory_.joint_trajectory.points.back();
        RCLCPP_INFO(this->get_logger(), "轨迹起点关节角度: %.3f %.3f %.3f %.3f (rad)",
                    first_point.positions.size() > 0 ? first_point.positions[0] : 0.0,
                    first_point.positions.size() > 1 ? first_point.positions[1] : 0.0,
                    first_point.positions.size() > 2 ? first_point.positions[2] : 0.0,
                    first_point.positions.size() > 3 ? first_point.positions[3] : 0.0);
        RCLCPP_INFO(this->get_logger(), "轨迹终点关节角度: %.3f %.3f %.3f %.3f (rad)",
                    last_point.positions.size() > 0 ? last_point.positions[0] : 0.0,
                    last_point.positions.size() > 1 ? last_point.positions[1] : 0.0,
                    last_point.positions.size() > 2 ? last_point.positions[2] : 0.0,
                    last_point.positions.size() > 3 ? last_point.positions[3] : 0.0);
      }
      
      // 在执行前检查是否收到 shutdown 信号
      if (!rclcpp::ok())
      {
        RCLCPP_INFO(this->get_logger(), "收到 shutdown 信号，取消执行规划");
        return;
      }
      
      RCLCPP_INFO(this->get_logger(), "=== 开始执行规划 ===");
      
      // 发布状态：执行中
      this->publish_state("executing");
      
      // 执行规划
      moveit::core::MoveItErrorCode result = move_group_interface_->execute(plan);
      
      if (result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_INFO(this->get_logger(), "=== 执行成功 ===");
        // 发布状态：空闲
        this->publish_state("idle");
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "=== 执行失败 ===");
        RCLCPP_WARN(this->get_logger(), "错误代码: %d", result.val);
        // MoveItErrorCode没有getMessage()方法，直接输出错误代码
        RCLCPP_WARN(this->get_logger(), "错误代码含义: SUCCESS=1, FAILURE=-1, PLANNING_FAILED=-2, INVALID_MOTION_PLAN=-3, MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE=-4, CONTROL_FAILED=-5, UNABLE_TO_AQUIRE_SENSOR_DATA=-6, TIMED_OUT=-7, PREEMPTED=-8");
        // 发布状态：错误
        this->publish_state("error");
      }
    }
    else
    {
      // 发布状态：错误（包含最后一个尝试的规划器信息）
      std::string last_planner = planner_ids.empty() ? "" : planner_ids.back();
      this->publish_state("error", last_planner);
      
      // 规划失败时的详细诊断
      RCLCPP_ERROR(this->get_logger(), "=== 关节角度目标规划失败 - 详细诊断 ===");
      RCLCPP_ERROR(this->get_logger(), "错误代码: %d", code.val);
      
      // 详细的错误代码解释
      std::string error_msg;
      switch (code.val)
      {
        case 1:
          error_msg = "SUCCESS (不应该出现在这里)";
          break;
        case -1:
          error_msg = "FAILURE (一般性失败)";
          break;
        case -2:
          error_msg = "PLANNING_FAILED (规划器无法找到路径)";
          break;
        case -3:
          error_msg = "INVALID_MOTION_PLAN (生成的路径无效)";
          break;
        case -4:
          error_msg = "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE (环境变化导致路径失效)";
          break;
        case -5:
          error_msg = "CONTROL_FAILED (控制失败)";
          break;
        case -6:
          error_msg = "UNABLE_TO_AQUIRE_SENSOR_DATA (无法获取传感器数据)";
          break;
        case -7:
          error_msg = "TIMED_OUT (规划超时)";
          break;
        case -8:
          error_msg = "PREEMPTED (规划被中断)";
          break;
        default:
          error_msg = "未知错误代码";
          break;
      }
      RCLCPP_ERROR(this->get_logger(), "错误类型: %s", error_msg.c_str());
      
      // 输出规划器配置信息
      RCLCPP_ERROR(this->get_logger(), "规划器配置:");
      RCLCPP_ERROR(this->get_logger(), "  规划管道: %s", move_group_interface_->getPlanningPipelineId().c_str());
      RCLCPP_ERROR(this->get_logger(), "  规划器ID: %s", move_group_interface_->getPlannerId().c_str());
      RCLCPP_ERROR(this->get_logger(), "  规划时间: %.1f 秒", move_group_interface_->getPlanningTime());
      // 注意：MoveGroupInterface没有getNumPlanningAttempts()方法，只能获取设置的值
      RCLCPP_ERROR(this->get_logger(), "  规划尝试次数: 20 (已设置)");
      
      // 输出目标位姿信息
      RCLCPP_ERROR(this->get_logger(), "目标位姿信息:");
      RCLCPP_ERROR(this->get_logger(), "  frame_id: %s", final_target_pose.header.frame_id.c_str());
      RCLCPP_ERROR(this->get_logger(), "  planning_frame: %s", planning_frame_.c_str());
      RCLCPP_ERROR(this->get_logger(), "  位置: (%.6f, %.6f, %.6f)", 
                  final_target_pose.pose.position.x,
                  final_target_pose.pose.position.y,
                  final_target_pose.pose.position.z);
      RCLCPP_ERROR(this->get_logger(), "  姿态: (%.6f, %.6f, %.6f, %.6f)",
                  final_target_pose.pose.orientation.x,
                  final_target_pose.pose.orientation.y,
                  final_target_pose.pose.orientation.z,
                  final_target_pose.pose.orientation.w);
      
      // 输出当前状态信息
      RCLCPP_ERROR(this->get_logger(), "当前状态信息:");
      std::vector<double> current_joints;
      const auto* jmg = state->getJointModelGroup("arm_group");
      if (jmg)
      {
        state->copyJointGroupPositions(jmg, current_joints);
        const std::vector<std::string>& joint_names = jmg->getJointModelNames();
        for (size_t i = 0; i < joint_names.size() && i < current_joints.size(); ++i)
        {
          RCLCPP_ERROR(this->get_logger(), "  %s = %.6f rad (%.2f°)", 
                      joint_names[i].c_str(), current_joints[i], 
                      current_joints[i] * 180.0 / M_PI);
        }
      }
      
      // 再次进行诊断测试（规划失败后）
      RCLCPP_ERROR(this->get_logger(), "=== 规划失败后诊断测试 ===");
      
      // 1. 再次测试IK求解器
      // bool ik_success_after = test_ik_solver(final_target_pose, state);  // 注释掉以加快速度
      bool ik_success_after = true;  // 假设成功，因为直接IK测试已经验证
      if (false)  // 禁用诊断以加快速度
      {
        RCLCPP_ERROR(this->get_logger(), "诊断结果：IK求解失败，这是规划失败的主要原因");
        RCLCPP_ERROR(this->get_logger(), "建议解决方案:");
        RCLCPP_ERROR(this->get_logger(), "  1. 检查目标位置是否在工作空间内");
        RCLCPP_ERROR(this->get_logger(), "  2. 调整目标位置到更接近当前位置");
        RCLCPP_ERROR(this->get_logger(), "  3. 检查IK求解器配置（kinematics.yaml）");
        RCLCPP_ERROR(this->get_logger(), "  4. 增加IK求解器超时时间或放宽精度要求");
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "诊断结果：IK求解成功，但规划失败");
        RCLCPP_WARN(this->get_logger(), "可能原因:");
        RCLCPP_WARN(this->get_logger(), "  1. 起始状态到目标状态之间无法找到无碰撞路径");
        RCLCPP_WARN(this->get_logger(), "  2. 规划器参数设置不当（时间、尝试次数等）");
        RCLCPP_WARN(this->get_logger(), "  3. 中间路径存在碰撞");
        RCLCPP_WARN(this->get_logger(), "建议解决方案:");
        RCLCPP_WARN(this->get_logger(), "  1. 增加规划时间和尝试次数");
        RCLCPP_WARN(this->get_logger(), "  2. 尝试不同的规划器（如RRTstar、EST）");
        RCLCPP_WARN(this->get_logger(), "  3. 检查并优化碰撞检测配置（SRDF）");
        RCLCPP_WARN(this->get_logger(), "  4. 使用中间路点分段规划");
      }
      
      // 2. 检查碰撞
      bool start_collision_after = test_collision(state, "起始状态（规划失败后）");
      if (start_collision_after)
      {
        RCLCPP_ERROR(this->get_logger(), "诊断结果：起始状态存在碰撞，这可能导致规划失败");
        RCLCPP_ERROR(this->get_logger(), "建议：使用FixStartStateCollision适配器或手动调整起始状态");
      }
      
      RCLCPP_ERROR(this->get_logger(), "=== 规划失败诊断结束 ===");
    }

    // 清理目标位姿和路径约束，避免残留影响后续规划
    move_group_interface_->clearPoseTargets();
    move_group_interface_->clearPathConstraints();
  }

  // 发布状态到网页
  void publish_state(const std::string& state, const std::string& planner_name = "")
  {
    if (state_publisher_)
    {
      std_msgs::msg::String msg;
      if (!planner_name.empty())
      {
        // 如果提供了规划器名称，格式化为 "state:planner_name"
        msg.data = state + ":" + planner_name;
      }
      else
      {
        msg.data = state;
      }
      state_publisher_->publish(msg);
      // 使用INFO级别，确保状态发布可见（用于调试和监控）
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

      // 从队列中取出任务
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        
        // 等待任务或停止信号
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

      // 处理任务
      if (has_task)
      {
        try {
          process_planning_task(task);
        } catch (const std::exception& e) {
          RCLCPP_ERROR(this->get_logger(), "处理规划任务时发生异常: %s", e.what());
          this->publish_state("error");
        }
      }
    }

    RCLCPP_INFO(this->get_logger(), "工作线程退出");
  }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  {
    auto node = std::make_shared<MoveItDemo>();
    node->start_executor();
    node->init_moveit();

    // 等待 Ctrl+C / kill，直到 rclcpp::shutdown() 触发
    while (rclcpp::ok())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  rclcpp::shutdown();
  return 0;
}

