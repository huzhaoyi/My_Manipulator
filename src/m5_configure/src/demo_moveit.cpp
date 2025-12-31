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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/time.h>
#include <chrono>

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

    RCLCPP_INFO(this->get_logger(), "demo_moveit节点已启动，等待目标位姿消息...");
    RCLCPP_INFO(this->get_logger(), "订阅话题: /target_pose");
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

    // 参考成熟实现：设置规划参数
    move_group_interface_->setPlanningTime(5.0);
    move_group_interface_->setNumPlanningAttempts(10);
    move_group_interface_->setMaxVelocityScalingFactor(0.9);
    move_group_interface_->setMaxAccelerationScalingFactor(0.9);
    move_group_interface_->setGoalPositionTolerance(0.001);   // 1mm位置容差
    // 对于4DOF机械臂，由于position_only_ik=true，orientation容差需要大幅放宽
    // 参考成熟实现使用0.01，但对于4DOF需要更大容差以提高IK成功率
    move_group_interface_->setGoalOrientationTolerance(0.5); // ~28.6度方向容差（大幅放宽以提高4DOF机械臂成功率）
    move_group_interface_->allowReplanning(true);
    
    move_group_interface_->setPlanningPipelineId("ompl");
    move_group_interface_->setPlannerId("RRTConnect");
    
    RCLCPP_INFO(this->get_logger(), "规划器配置: 规划时间=5.0s, 尝试次数=10");
    RCLCPP_INFO(this->get_logger(), "速度/加速度缩放: 0.9");
    RCLCPP_INFO(this->get_logger(), "目标位置容差: 0.001 m (1mm)");
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
      return;
    }

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

  // 可达性快速过滤函数（基于URDF实际参数的三级判定）
  // 简化验证逻辑：只做基本几何边界检查，让IK求解器判断是否真正可达
  bool is_reachable(double x, double y, double z)
  {
    // URDF实际参数（从m5_updated_from_csv.urdf提取）
    const double base_height = 0.141;  // Joint1在base_link上的高度
    const double link2_length = 0.264;  // Joint2到Joint3的距离
    const double link3_length = 0.143;  // Joint3到Joint4的距离（Z方向）
    const double link4_to_eef = 0.187;  // Joint4到LinkGG的距离
    
    // 计算理论最大伸展距离（完全伸展时）
    const double max_reach = link2_length + link3_length + link4_to_eef;  // 约0.594m
    const double min_reach = 0.05;  // 最小距离（考虑关节偏移和基座）
    
    // 计算到基座的距离（半径）
    const double radius = std::sqrt(x * x + y * y);
    
    // 计算相对高度（相对于Joint1）
    const double relative_height = z - base_height;
    
    // 调试信息
    RCLCPP_INFO(this->get_logger(), "工作空间参数:");
    RCLCPP_INFO(this->get_logger(), "  base_height = %.3f m", base_height);
    RCLCPP_INFO(this->get_logger(), "  link2_length = %.3f m", link2_length);
    RCLCPP_INFO(this->get_logger(), "  link3_length = %.3f m", link3_length);
    RCLCPP_INFO(this->get_logger(), "  link4_to_eef = %.3f m", link4_to_eef);
    RCLCPP_INFO(this->get_logger(), "  max_reach = %.3f m", max_reach);
    RCLCPP_INFO(this->get_logger(), "目标位置分析:");
    RCLCPP_INFO(this->get_logger(), "  半径 = %.6f m", radius);
    RCLCPP_INFO(this->get_logger(), "  绝对高度 = %.6f m", z);
    RCLCPP_INFO(this->get_logger(), "  相对高度 = %.6f m (相对于base_height)", relative_height);
    
    // 第一级：检查基本几何边界（放宽条件，允许更多尝试）
    const double min_radius = 0.03;   // 最小半径（稍微放宽）
    const double max_radius = max_reach * 1.05;  // 最大半径（留5%余量，允许稍微超出）
    const double min_height = base_height - 0.10;  // 最小高度（允许略低于基座）
    const double max_height = base_height + max_reach * 1.05;  // 最大高度（允许稍微超出）
    
    if (radius < min_radius || radius > max_radius || z < min_height || z > max_height)
    {
      RCLCPP_WARN(this->get_logger(),
                  "目标位置 (%.3f, %.3f, %.3f) 超出理论工作空间边界 "
                  "(半径范围: [%.3f, %.3f], 高度范围: [%.3f, %.3f])",
                  x, y, z, min_radius, max_radius, min_height, max_height);
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
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "工作空间验证通过");
    RCLCPP_INFO(this->get_logger(), "=== 工作空间验证结束 ===");
    
    RCLCPP_INFO(this->get_logger(), "开始处理规划任务: x=%.3f, y=%.3f, z=%.3f", x, y, z);

    // 清理之前可能残留的目标位姿和路径约束，确保从干净状态开始
    move_group_interface_->clearPoseTargets();
    move_group_interface_->clearPathConstraints();
    move_group_interface_->setStartStateToCurrentState(); // 设置起始状态为当前状态


    // 获取并打印当前关节状态（用于调试）
    auto state = move_group_interface_->getCurrentState(2.0);
    if (!state)
    {
      RCLCPP_ERROR(this->get_logger(), "getCurrentState() 失败：MoveIt 没拿到有效 joint_states");
      return;
    }

    const auto* jmg = state->getJointModelGroup("arm_group");
    if (!jmg)
    {
      RCLCPP_ERROR(this->get_logger(), "找不到 JointModelGroup: arm_group（SRDF group 名可能不一致）");
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
      return;
    }

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

    // 参考成熟实现：直接进行规划，MoveIt会自动处理IK和坐标系转换
    RCLCPP_INFO(this->get_logger(), "=== 开始位姿规划（参考成熟实现）===");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto code = move_group_interface_->plan(plan);
    bool success = (code == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
      RCLCPP_INFO(this->get_logger(), "=== 位姿规划成功 ===");
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
      
      // 执行规划
      moveit::core::MoveItErrorCode result = move_group_interface_->execute(plan);
      
      if (result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_INFO(this->get_logger(), "=== 执行成功 ===");
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "=== 执行失败 ===");
        RCLCPP_WARN(this->get_logger(), "错误代码: %d", result.val);
        // MoveItErrorCode没有getMessage()方法，直接输出错误代码
        RCLCPP_WARN(this->get_logger(), "错误代码含义: SUCCESS=1, FAILURE=-1, PLANNING_FAILED=-2, INVALID_MOTION_PLAN=-3, MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE=-4, CONTROL_FAILED=-5, UNABLE_TO_AQUIRE_SENSOR_DATA=-6, TIMED_OUT=-7, PREEMPTED=-8");
      }
    }
    else
    {
      // 参考成熟实现：位姿规划失败就直接失败，不进行复杂的回退
      RCLCPP_WARN(this->get_logger(), "=== 位姿规划失败 ===");
      RCLCPP_WARN(this->get_logger(), "错误代码: %d", code.val);
      RCLCPP_WARN(this->get_logger(), "错误代码含义: SUCCESS=1, FAILURE=-1, PLANNING_FAILED=-2, INVALID_MOTION_PLAN=-3, MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE=-4, CONTROL_FAILED=-5, UNABLE_TO_AQUIRE_SENSOR_DATA=-6, TIMED_OUT=-7, PREEMPTED=-8");
      RCLCPP_WARN(this->get_logger(), "可能的原因:");
      RCLCPP_WARN(this->get_logger(), "  1. 目标位置不可达（超出工作空间）");
      RCLCPP_WARN(this->get_logger(), "  2. 规划器参数设置不当");
      RCLCPP_WARN(this->get_logger(), "  3. 存在碰撞约束");
      RCLCPP_WARN(this->get_logger(), "  4. 目标位置距离当前位置太远");
    }

    // 清理目标位姿和路径约束，避免残留影响后续规划
    move_group_interface_->clearPoseTargets();
    move_group_interface_->clearPathConstraints();
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

