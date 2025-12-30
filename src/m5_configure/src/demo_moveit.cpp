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

    // 配置规划器参数（4DOF机械臂需要更多时间和尝试次数）
    move_group_interface_->setPlanningTime(3.0);
    move_group_interface_->setNumPlanningAttempts(10);
    RCLCPP_INFO(this->get_logger(), "规划器配置: 规划时间=3.0s, 尝试次数=10");

    // 设置较小的速度/加速度（实际执行更稳定）
    move_group_interface_->setMaxVelocityScalingFactor(0.2);
    move_group_interface_->setMaxAccelerationScalingFactor(0.2);
    RCLCPP_INFO(this->get_logger(), "速度/加速度缩放: 0.2");

    move_group_interface_->setPlanningPipelineId("ompl");
    move_group_interface_->setPlannerId("RRTConnect");

    // 诊断：允许碰撞（仅用于测试）
    move_group_interface_->setGoalTolerance(0.01);
    move_group_interface_->allowReplanning(true);

    
    // 设置参考帧和末端执行器链接
    // 使用 MoveIt 的 planning frame 作为唯一真相（更稳）
    planning_frame_ = move_group_interface_->getPlanningFrame();
    move_group_interface_->setPoseReferenceFrame(planning_frame_);
    move_group_interface_->setEndEffectorLink("Link4");
    RCLCPP_INFO(this->get_logger(), "设置参考帧: %s, 末端执行器链接: Link4", planning_frame_.c_str());

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

    const std::string planning_frame = planning_frame_.empty() ? "world_link" : planning_frame_;
    geometry_msgs::msg::PoseStamped target_in_planning;

    const bool use_latest = (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0);
    const auto timeout = tf2::durationFromSec(0.1);

    try
    {
      if (use_latest)
      {
        if (!tf_buffer_->canTransform(planning_frame, msg->header.frame_id, tf2::TimePointZero, timeout)) {
          RCLCPP_WARN(this->get_logger(), "No TF (%s -> %s) available yet (latest)",
                      msg->header.frame_id.c_str(), planning_frame.c_str());
          return;
        }

        auto tf = tf_buffer_->lookupTransform(planning_frame, msg->header.frame_id, tf2::TimePointZero);
        tf2::doTransform(*msg, target_in_planning, tf);

        target_in_planning.header.frame_id = planning_frame;
        target_in_planning.header.stamp = this->now();
      }
      else
      {
        if (!tf_buffer_->canTransform(planning_frame, msg->header.frame_id, msg->header.stamp, timeout)) {
          RCLCPP_WARN(this->get_logger(), "No TF (%s -> %s) available yet",
                      msg->header.frame_id.c_str(), planning_frame.c_str());
          return;
        }

        target_in_planning = tf_buffer_->transform(*msg, planning_frame, timeout);
        target_in_planning.header.frame_id = planning_frame;  // 强制一致
      }
    }
    catch (const tf2::TransformException& ex)
    {
      RCLCPP_WARN(this->get_logger(), "TF transform failed (%s -> %s): %s",
                  msg->header.frame_id.c_str(), planning_frame.c_str(), ex.what());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "收到目标位姿(已转到%s): x=%.3f y=%.3f z=%.3f",
                planning_frame.c_str(),
                target_in_planning.pose.position.x,
                target_in_planning.pose.position.y,
                target_in_planning.pose.position.z);

    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      while (!task_queue_.empty()) task_queue_.pop();
      task_queue_.push(target_in_planning);
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

  // 可达性快速过滤函数（三级判定：超边界拒绝，推荐区域直接通过，其它区域允许但警告）
  bool is_reachable(double x, double y, double z)
  {
    // 计算到基座的距离（半径）
    const double radius = std::sqrt(x * x + y * y);
    
    // 工作空间边界检查（基于文档中的实际可达区域）
    const double min_radius = 0.10;  // 最小半径（接近基座）
    const double max_radius = 0.60;  // 最大半径（完全伸展）
    const double min_height = 0.14;   // 最小高度（基座高度）
    const double max_height = 0.74;   // 最大高度（基座高度 + 完全伸展）
    
    // 高成功率区域（推荐使用）
    const double safe_x_min = 0.20, safe_x_max = 0.35;
    const double safe_y_min = -0.15, safe_y_max = 0.15;
    const double safe_z_min = 0.25, safe_z_max = 0.35;
    
    // 中等成功率区域（谨慎使用）
    const double medium_x_min = 0.15, medium_x_max = 0.45;
    const double medium_y_min = -0.20, medium_y_max = 0.20;
    const double medium_z_min = 0.20, medium_z_max = 0.40;
    
    // 第一级：超边界 → false
    if (radius < min_radius || radius > max_radius || z < min_height || z > max_height)
    {
      return false;
    }
    
    // 第二级：安全区/中等区 → true
    if (x >= safe_x_min && x <= safe_x_max &&
        y >= safe_y_min && y <= safe_y_max &&
        z >= safe_z_min && z <= safe_z_max)
    {
      return true;
    }
    
    if (x >= medium_x_min && x <= medium_x_max &&
        y >= medium_y_min && y <= medium_y_max &&
        z >= medium_z_min && z <= medium_z_max)
    {
      return true;
    }
    
    // 第三级：其它区域 → 允许尝试但输出 warning
    RCLCPP_WARN(this->get_logger(),
                "目标不在推荐区域 (%.3f, %.3f, %.3f)，可能规划失败，但仍尝试",
                x, y, z);
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
    if (!is_reachable(x, y, z))
    {
      RCLCPP_WARN(this->get_logger(), "目标位置 (%.3f, %.3f, %.3f) 超出工作空间边界，跳过规划",
                  x, y, z);
      return;
    }
    
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

    // 对于4DOF机械臂，只设置位置目标，让MoveIt自动找到合适的orientation
    const std::string eef = move_group_interface_->getEndEffectorLink();
    auto current_pose = move_group_interface_->getCurrentPose(eef);
    
    // 规划前验证：检查当前位姿是否有效
    if (current_pose.header.frame_id.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "无法获取当前末端执行器位姿，frame_id为空");
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "当前末端执行器位姿: x=%.3f y=%.3f z=%.3f (frame: %s)",
                current_pose.pose.position.x, current_pose.pose.position.y, 
                current_pose.pose.position.z, current_pose.header.frame_id.c_str());
    
    geometry_msgs::msg::Pose goal = current_pose.pose;
    goal.position.x = x;
    goal.position.y = y;
    goal.position.z = z;
    
    // 规划前验证：检查目标位置是否在可达范围内（已在is_reachable中检查，这里再次确认）
    double distance = std::sqrt((x - current_pose.pose.position.x) * (x - current_pose.pose.position.x) +
                                 (y - current_pose.pose.position.y) * (y - current_pose.pose.position.y) +
                                 (z - current_pose.pose.position.z) * (z - current_pose.pose.position.z));
    RCLCPP_INFO(this->get_logger(), "目标位置距离当前位置: %.3f m", distance);
    
    move_group_interface_->setGoalPositionTolerance(0.005);      // 5mm
    move_group_interface_->setGoalOrientationTolerance(0.2);     // 放宽姿态容忍（弧度）

    move_group_interface_->setPoseTarget(goal, eef);
    
    // 在 plan 之前再调用一次，防止 current state 的轻微漂移导致规划失败
    move_group_interface_->setStartStateToCurrentState();
    
    // 规划前验证：尝试直接IK求解，验证目标位置是否可达
    RCLCPP_INFO(this->get_logger(), "规划前验证：尝试IK求解目标位置...");
    moveit::core::RobotStatePtr test_ik_state(new moveit::core::RobotState(*state));
    const moveit::core::JointModelGroup* test_ik_jmg = test_ik_state->getJointModelGroup("arm_group");
    bool test_ik_found = test_ik_state->setFromIK(test_ik_jmg, goal, eef, 0.2, 1.0);
    
    if (test_ik_found)
    {
      std::vector<double> test_ik_joints;
      test_ik_state->copyJointGroupPositions(test_ik_jmg, test_ik_joints);
      RCLCPP_INFO(this->get_logger(), "规划前验证：IK求解成功，目标位置可达");
      RCLCPP_INFO(this->get_logger(), "  目标关节角度: %.3f %.3f %.3f %.3f (rad)",
                  test_ik_joints[0], test_ik_joints[1], test_ik_joints[2], 
                  test_ik_joints.size() > 3 ? test_ik_joints[3] : 0.0);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "规划前验证：IK求解失败，目标位置可能不可达，但仍尝试规划");
    }

    // 进行规划
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_interface_->plan(plan));

    if (success)
    {
      RCLCPP_INFO(this->get_logger(), "规划成功！轨迹包含 %zu 个点", 
                  plan.trajectory_.joint_trajectory.points.size());
      
      // 在执行前检查是否收到 shutdown 信号
      if (!rclcpp::ok())
      {
        RCLCPP_INFO(this->get_logger(), "收到 shutdown 信号，取消执行规划");
        return;
      }
      
      RCLCPP_INFO(this->get_logger(), "正在执行规划...");
      
      // 执行规划
      moveit::core::MoveItErrorCode result = move_group_interface_->execute(plan);
      
      if (result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_INFO(this->get_logger(), "执行完成！");
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "执行失败！错误代码: %d", result.val);
      }
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "位姿规划失败！尝试关节空间规划...");
      
      // 尝试使用IK求解器获取目标位置的关节角度（多次尝试不同的姿态种子）
      geometry_msgs::msg::Pose ik_pose;
      ik_pose.position.x = x;
      ik_pose.position.y = y;
      ik_pose.position.z = z;
      
      // 获取末端执行器链接名称
      std::string end_effector_link = move_group_interface_->getEndEffectorLink();
      
      // 使用当前状态作为种子状态
      moveit::core::RobotStatePtr ik_state(new moveit::core::RobotState(*state));
      const moveit::core::JointModelGroup* ik_jmg = ik_state->getJointModelGroup("arm_group");
      
      // 尝试多个不同的姿态种子进行IK求解
      std::vector<geometry_msgs::msg::Quaternion> orientation_seeds = {
        move_group_interface_->getCurrentPose(end_effector_link).pose.orientation,  // 当前姿态
      };
      
      // 添加一些额外的姿态种子（4DOF机械臂可能需要不同的姿态）
      geometry_msgs::msg::Quaternion seed1, seed2, seed3;
      seed1.w = 1.0; seed1.x = 0.0; seed1.y = 0.0; seed1.z = 0.0;  // 单位四元数
      seed2.w = 0.707; seed2.x = 0.0; seed2.y = 0.707; seed2.z = 0.0;  // 绕Y轴旋转90度
      seed3.w = 0.707; seed3.x = 0.707; seed3.y = 0.0; seed3.z = 0.0;  // 绕X轴旋转90度
      orientation_seeds.push_back(seed1);
      orientation_seeds.push_back(seed2);
      orientation_seeds.push_back(seed3);
      
      bool ik_found = false;
      int ik_attempts = 0;
      const int max_ik_attempts = orientation_seeds.size();
      
      RCLCPP_INFO(this->get_logger(), "开始IK求解，将尝试 %d 个不同的姿态种子", max_ik_attempts);
      
      for (const auto& orientation_seed : orientation_seeds)
      {
        ik_attempts++;
        ik_pose.orientation = orientation_seed;
        
        // 重置状态到当前状态
        ik_state->setVariablePositions(state->getVariablePositions());
        
        // 尝试IK求解
        bool attempt_result = ik_state->setFromIK(ik_jmg, ik_pose, end_effector_link, 0.2, 1.0);
        
        if (attempt_result)
        {
          ik_found = true;
          RCLCPP_INFO(this->get_logger(), "IK求解成功（尝试 %d/%d，使用姿态种子: w=%.3f x=%.3f y=%.3f z=%.3f）",
                      ik_attempts, max_ik_attempts, 
                      orientation_seed.w, orientation_seed.x, orientation_seed.y, orientation_seed.z);
          break;
        }
        else
        {
          RCLCPP_DEBUG(this->get_logger(), "IK求解尝试 %d/%d 失败（姿态种子: w=%.3f x=%.3f y=%.3f z=%.3f）",
                       ik_attempts, max_ik_attempts,
                       orientation_seed.w, orientation_seed.x, orientation_seed.y, orientation_seed.z);
        }
      }
      
      if (!ik_found)
      {
        RCLCPP_WARN(this->get_logger(), "IK求解失败：尝试了 %d 个不同的姿态种子，均无法找到解", max_ik_attempts);
        RCLCPP_WARN(this->get_logger(), "目标位置: x=%.3f y=%.3f z=%.3f 可能超出工作空间或不可达", x, y, z);
        
        // 打印当前关节限制信息
        const moveit::core::JointModel* joint1 = ik_jmg->getJointModel("Joint1");
        const moveit::core::JointModel* joint2 = ik_jmg->getJointModel("Joint2");
        const moveit::core::JointModel* joint3 = ik_jmg->getJointModel("Joint3");
        const moveit::core::JointModel* joint4 = ik_jmg->getJointModel("Joint4");
        
        std::ostringstream limits_oss;
        limits_oss << "关节限制: ";
        if (joint1 && joint1->getVariableBounds().size() > 0)
        {
          limits_oss << "Joint1[" << joint1->getVariableBounds()[0].min_position_ 
                     << ", " << joint1->getVariableBounds()[0].max_position_ << "] ";
        }
        if (joint2 && joint2->getVariableBounds().size() > 0)
        {
          limits_oss << "Joint2[" << joint2->getVariableBounds()[0].min_position_ 
                     << ", " << joint2->getVariableBounds()[0].max_position_ << "] ";
        }
        if (joint3 && joint3->getVariableBounds().size() > 0)
        {
          limits_oss << "Joint3[" << joint3->getVariableBounds()[0].min_position_ 
                     << ", " << joint3->getVariableBounds()[0].max_position_ << "] ";
        }
        if (joint4 && joint4->getVariableBounds().size() > 0)
        {
          limits_oss << "Joint4[" << joint4->getVariableBounds()[0].min_position_ 
                     << ", " << joint4->getVariableBounds()[0].max_position_ << "]";
        }
        RCLCPP_INFO(this->get_logger(), "%s", limits_oss.str().c_str());
      }
      
      if (ik_found)
      {
        RCLCPP_INFO(this->get_logger(), "IK求解成功！尝试关节空间规划...");
        
        // 获取IK求解得到的关节角度
        std::vector<double> ik_joints;
        ik_state->copyJointGroupPositions(ik_jmg, ik_joints);
        
        // 直接设置关节目标（不再使用 setPoseTarget，减少一次 IK pipeline 的不确定性）
        move_group_interface_->setJointValueTarget(ik_joints);
        move_group_interface_->setStartStateToCurrentState();
        
        // 进行关节空间规划
        moveit::planning_interface::MoveGroupInterface::Plan joint_plan;
        bool joint_success = static_cast<bool>(move_group_interface_->plan(joint_plan));
        
        if (joint_success)
        {
          RCLCPP_INFO(this->get_logger(), "关节空间规划成功！轨迹包含 %zu 个点", 
                     joint_plan.trajectory_.joint_trajectory.points.size());
          
          // 在执行前检查是否收到 shutdown 信号
          if (!rclcpp::ok())
          {
            RCLCPP_INFO(this->get_logger(), "收到 shutdown 信号，取消执行关节空间规划");
            return;
          }
          
          RCLCPP_INFO(this->get_logger(), "正在执行关节空间规划...");
          
          // 执行规划
          moveit::core::MoveItErrorCode result = move_group_interface_->execute(joint_plan);
          
          if (result == moveit::core::MoveItErrorCode::SUCCESS)
          {
            RCLCPP_INFO(this->get_logger(), "关节空间规划执行完成！");
          }
          else
          {
            RCLCPP_WARN(this->get_logger(), "关节空间规划执行失败！错误代码: %d", result.val);
          }
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), "关节空间规划也失败！");
        }
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "IK求解失败，无法进行关节空间规划");
      }
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

