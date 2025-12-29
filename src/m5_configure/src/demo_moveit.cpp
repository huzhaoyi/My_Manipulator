#include <memory>
#include <functional>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

class MoveItDemo : public rclcpp::Node
{
public:
  MoveItDemo() : Node("demo_moveit")
  {
    // 订阅目标位姿话题
    target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/target_pose", 10, 
        std::bind(&MoveItDemo::target_pose_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "demo_moveit节点已启动，等待目标位姿消息...");
    RCLCPP_INFO(this->get_logger(), "订阅话题: /target_pose");
  }

  ~MoveItDemo()
  {
    // 停止工作线程
    stop_workers_ = true;
    queue_cv_.notify_all();  // 唤醒所有等待的工作线程

    // 等待所有工作线程完成
    for (auto& thread : worker_threads_)
    {
      if (thread.joinable())
      {
        thread.join();
      }
    }
    worker_threads_.clear();
    RCLCPP_INFO(this->get_logger(), "所有工作线程已停止");

    // 停止executor
    if (executor_)
    {
      executor_->cancel();
    }
    if (executor_thread_.joinable())
    {
      executor_thread_.join();
    }
  }

  // 初始化MoveIt接口（在对象创建后调用）
  void init_moveit()
  {
    // 初始化 MoveIt 接口（此时shared_from_this()可以安全使用）
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "arm_group");
    RCLCPP_INFO(this->get_logger(), "MoveIt接口已初始化");

    // 启动工作线程池
    stop_workers_ = false;
    for (size_t i = 0; i < num_worker_threads_; ++i)
    {
      worker_threads_.emplace_back(&MoveItDemo::worker_thread, this);
      RCLCPP_INFO(this->get_logger(), "工作线程 %zu 已启动", i + 1);
    }
    RCLCPP_INFO(this->get_logger(), "工作线程池已启动，共 %zu 个线程", num_worker_threads_);
  }

  // 启动executor（在对象创建后调用）
  void start_executor()
  {
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(shared_from_this());
    executor_thread_ = std::thread([this]() { executor_->spin(); });
    RCLCPP_INFO(this->get_logger(), "Executor线程已启动");
  }

  // 提供公共方法让主线程等待executor线程
  void wait_for_executor()
  {
    if (executor_thread_.joinable())
    {
      executor_thread_.join();
    }
  }

private:
  void target_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "收到目标位姿: x=%.3f, y=%.3f, z=%.3f",
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    // 将任务加入队列（非阻塞操作）
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      task_queue_.push(*msg);
    }
    
    // 通知工作线程有新任务
    queue_cv_.notify_one();
    
    RCLCPP_DEBUG(this->get_logger(), "目标位姿已加入任务队列，等待处理...");
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread executor_thread_;

  // 任务队列相关
  std::queue<geometry_msgs::msg::PoseStamped> task_queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;

  // 工作线程池
  std::vector<std::thread> worker_threads_;
  std::atomic<bool> stop_workers_{false};
  size_t num_worker_threads_{1};  // 默认1个线程（排队处理）

  // 规划处理函数（在工作线程中执行）
  void process_planning_task(const geometry_msgs::msg::PoseStamped& target_pose)
  {
    RCLCPP_INFO(this->get_logger(), "开始处理规划任务: x=%.3f, y=%.3f, z=%.3f",
                target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);

    // 获取当前位姿，使用当前姿态的orientation，只改变position（提高IK成功率）
    geometry_msgs::msg::Pose target_pose_processed = target_pose.pose;
    
    try {
      auto current_pose = move_group_interface_->getCurrentPose();
      // 如果接收到的位姿没有设置orientation（全为0或单位四元数w=1.0），使用当前位姿的orientation
      bool use_current_orientation = false;
      if ((target_pose.pose.orientation.w == 0.0 && 
           target_pose.pose.orientation.x == 0.0 && 
           target_pose.pose.orientation.y == 0.0 && 
           target_pose.pose.orientation.z == 0.0) ||
          (target_pose.pose.orientation.w == 1.0 && 
           target_pose.pose.orientation.x == 0.0 && 
           target_pose.pose.orientation.y == 0.0 && 
           target_pose.pose.orientation.z == 0.0))
      {
        use_current_orientation = true;
      }
      
      if (use_current_orientation)
      {
        target_pose_processed.orientation = current_pose.pose.orientation;
        RCLCPP_INFO(this->get_logger(), "使用当前位姿的orientation（提高IK成功率）");
      }
      else
      {
        // 如果接收到的位姿有有效的orientation，使用接收到的orientation
        RCLCPP_INFO(this->get_logger(), "使用接收到的orientation: w=%.3f, x=%.3f, y=%.3f, z=%.3f", 
                    target_pose.pose.orientation.w, target_pose.pose.orientation.x, 
                    target_pose.pose.orientation.y, target_pose.pose.orientation.z);
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "无法获取当前位姿，使用接收到的位姿: %s", e.what());
    }

    // 设置目标位姿
    move_group_interface_->setPoseTarget(target_pose_processed);

    // 进行规划
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = static_cast<bool>(move_group_interface_->plan(plan));

    if (success)
    {
      RCLCPP_INFO(this->get_logger(), "规划成功！轨迹包含 %zu 个点", 
                  plan.trajectory_.joint_trajectory.points.size());
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
      RCLCPP_WARN(this->get_logger(), "规划失败！");
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
        }
      }
    }

    RCLCPP_INFO(this->get_logger(), "工作线程退出");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveItDemo>();
  
  // 在对象创建后初始化MoveIt接口（此时shared_from_this()可以安全使用）
  node->init_moveit();
  
  // 在对象创建后启动executor（此时shared_from_this()可以安全使用）
  node->start_executor();
  
  // executor在后台线程中运行，主线程等待直到收到信号（Ctrl+C）
  // 当收到信号时，rclcpp::shutdown()会被调用，executor的spin()会返回
  // 主线程等待executor线程完成
  node->wait_for_executor();
  
  rclcpp::shutdown();
  return 0;
}
