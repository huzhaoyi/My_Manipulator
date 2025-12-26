#include <memory>
#include <thread>
#include <chrono>
#include <atomic>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char * argv[])
{
  // 初始化 ROS 并创建节点
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "m5_planning_node",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // 创建一个 ROS logger
  auto const logger = rclcpp::get_logger("m5_planning_node");

  // 创建 MoveIt 的 MoveGroup 接口
  // 使用 m5 机械臂的规划组名称 "arm_group"
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm_group");

  // 创建规划场景接口（用于添加碰撞物体）
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // 设置规划参数
  move_group_interface.setPlanningTime(10.0);  // 设置规划时间限制为10秒
  move_group_interface.setNumPlanningAttempts(10);  // 设置规划尝试次数

  // 使用原子标志确保 timer 回调只执行一次
  std::atomic_bool done{false};

  // 创建定时器，在 executor 线程中执行规划任务（只执行一次）
  // 延迟 1 秒执行，等待节点初始化完成
  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(1000),
    [&done, &logger, &move_group_interface, &planning_scene_interface, node]() {
      // 确保只执行一次
      if (done.exchange(true)) {
        return;
    }

      RCLCPP_INFO(logger, "节点初始化完成，开始规划...");

  // 打印当前状态信息
  RCLCPP_INFO(logger, "规划组名称: %s", move_group_interface.getName().c_str());
  RCLCPP_INFO(logger, "末端执行器链接: %s", move_group_interface.getEndEffectorLink().c_str());
  
      // ========== 等待 /joint_states topic 可用 ==========
      // 这是关键：MoveIt 需要 /joint_states 才能获取当前机器人状态
      RCLCPP_INFO(logger, "检查 /joint_states topic 是否可用...");
      
      // 检查 topic 是否有发布者
      auto topic_info = node->get_topic_names_and_types();
      bool joint_states_topic_exists = false;
      for (const auto& [topic_name, types] : topic_info) {
        if (topic_name == "/joint_states") {
          joint_states_topic_exists = true;
          RCLCPP_INFO(logger, "发现 /joint_states topic (类型: %s)", types[0].c_str());
          break;
        }
      }
      
      if (!joint_states_topic_exists) {
        RCLCPP_WARN(logger, "/joint_states topic 尚未发布，等待中...");
        // 等待最多 5 秒，检查 topic 是否出现
        for (int i = 0; i < 50 && rclcpp::ok(); ++i) {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          topic_info = node->get_topic_names_and_types();
          for (const auto& [topic_name, types] : topic_info) {
            if (topic_name == "/joint_states") {
              joint_states_topic_exists = true;
              RCLCPP_INFO(logger, "/joint_states topic 已可用！");
              break;
            }
          }
          if (joint_states_topic_exists) break;
          if (i % 10 == 0 && i > 0) {
            RCLCPP_WARN(logger, "等待 /joint_states topic... (%d/5 秒)", i / 10);
          }
        }
      }
      
      if (!joint_states_topic_exists) {
        RCLCPP_ERROR(logger, "错误：/joint_states topic 不可用！");
        RCLCPP_ERROR(logger, "请确保以下节点正在运行：");
        RCLCPP_ERROR(logger, "1. robot_state_publisher");
        RCLCPP_ERROR(logger, "2. joint_state_broadcaster (ros2_control)");
        RCLCPP_ERROR(logger, "3. 或者 joint_state_publisher (用于仿真)");
        RCLCPP_ERROR(logger, "程序将退出...");
        rclcpp::shutdown();
        return;
      }
      
      // ========== 等待 MoveIt 的 current_state_monitor 接收到有效的 joint_states ==========
      // 这是关键：MoveIt 需要时间来处理 joint_states 并更新内部状态
      RCLCPP_INFO(logger, "等待 MoveIt 接收并处理 joint_states...");
      
      // 先等待更长时间，让 MoveIt 的 current_state_monitor 有时间建立连接并处理消息
      // MoveIt 的 current_state_monitor 需要时间来订阅和处理 joint_states
      std::this_thread::sleep_for(std::chrono::milliseconds(5000));  // 增加到 5 秒
      
      // ========== 获取当前位姿并验证有效性（只检查 timestamp） ==========
      geometry_msgs::msg::Pose current_pose_msg;
      bool has_current_pose = false;
      int retry_count = 0;
      const int max_retries = 30;  // 增加重试次数
      
      while (!has_current_pose && retry_count < max_retries && rclcpp::ok()) {
  try {
    auto current_pose = move_group_interface.getCurrentPose();
          auto stamp = current_pose.header.stamp;
          
          // 只检查 timestamp 是否有效（不为 0）
          if (stamp.sec > 0 || (stamp.sec == 0 && stamp.nanosec > 0)) {
            current_pose_msg = current_pose.pose;
            has_current_pose = true;
            RCLCPP_INFO(logger, "成功获取当前位置（timestamp: %d.%09d）", 
                        stamp.sec, stamp.nanosec);
    RCLCPP_INFO(logger, "当前位置: x=%.3f, y=%.3f, z=%.3f", 
                        current_pose_msg.position.x,
                        current_pose_msg.position.y,
                        current_pose_msg.position.z);
            RCLCPP_INFO(logger, "当前姿态: qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f",
                        current_pose_msg.orientation.x,
                        current_pose_msg.orientation.y,
                        current_pose_msg.orientation.z,
                        current_pose_msg.orientation.w);
          } else {
            RCLCPP_WARN(logger, "获取的位姿 timestamp 无效 (timestamp = 0)，重试中... (%d/%d)", 
                        retry_count + 1, max_retries);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));  // 增加等待时间到 1 秒
            retry_count++;
          }
  } catch (const std::exception& e) {
          RCLCPP_WARN(logger, "获取当前位置时出错: %s，重试中... (%d/%d)", 
                      e.what(), retry_count + 1, max_retries);
          std::this_thread::sleep_for(std::chrono::milliseconds(1000));  // 增加等待时间到 1 秒
          retry_count++;
        }
      }
      
      if (!has_current_pose) {
        RCLCPP_ERROR(logger, "错误：经过 %d 次重试后仍无法获取有效的当前位置！", max_retries);
        RCLCPP_ERROR(logger, "这通常意味着 MoveIt 的 current_state_monitor 没有收到有效的 /joint_states 数据。");
        RCLCPP_ERROR(logger, "请检查：");
        RCLCPP_ERROR(logger, "1. robot_state_publisher 是否正在运行");
        RCLCPP_ERROR(logger, "2. /joint_states topic 是否有数据发布（timestamp 不为0）");
        RCLCPP_ERROR(logger, "3. 运行: ros2 topic echo /joint_states 查看数据");
        RCLCPP_ERROR(logger, "4. 检查 MoveIt 的 current_state_monitor 日志");
        RCLCPP_ERROR(logger, "程序将退出...");
        rclcpp::shutdown();
        return;
  }

  // 方法1：尝试使用位姿规划（直接规划到目标位姿）
  RCLCPP_INFO(logger, "=== 方法1：位姿规划（直接规划） ===");
  
  // 使用当前姿态的orientation，只改变position（提高IK成功率）
  geometry_msgs::msg::Pose target_pose = current_pose_msg;
  
  // 目标位置：障碍物盒子在(0.18, 0.0, 0.20)，目标在盒子前方
  target_pose.position.x = 0.30;
  target_pose.position.y = 0.0;
  target_pose.position.z = 0.30;
  
  RCLCPP_INFO(logger, "当前位置: x=%.3f, y=%.3f, z=%.3f", 
              current_pose_msg.position.x,
              current_pose_msg.position.y,
              current_pose_msg.position.z);
  RCLCPP_INFO(logger, "目标位置: x=%.3f, y=%.3f, z=%.3f", 
              target_pose.position.x,
              target_pose.position.y,
              target_pose.position.z);
  
  move_group_interface.setPoseTarget(target_pose);

  RCLCPP_INFO(logger, "开始位姿规划...");
  auto const [pose_success, pose_plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  bool pose_planning_success = false;
  
  if (pose_success) {
    RCLCPP_INFO(logger, "位姿规划成功！轨迹包含 %zu 个点", pose_plan.trajectory_.joint_trajectory.points.size());
    RCLCPP_INFO(logger, "正在执行规划...");
    moveit::core::MoveItErrorCode result = move_group_interface.execute(pose_plan);
    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(logger, "位姿规划执行完成！");
      // 等待执行完成后的回调处理（确保 action result 回调被处理）
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      // 额外等待，确保机器人完全停止运动
      RCLCPP_INFO(logger, "等待机器人完全停止运动...");
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      pose_planning_success = true;
    } else {
      RCLCPP_WARN(logger, "位姿规划执行失败！错误代码: %d，尝试关节空间规划", result.val);
    }
  } else {
    RCLCPP_WARN(logger, "位姿规划失败，尝试关节空间规划...");
  }
#if 0
  // 如果位姿规划成功，跳过关节空间规划，直接到程序末尾统一退出
  if (pose_planning_success) {
    RCLCPP_INFO(logger, "位姿规划成功，跳过关节空间规划");
  } else {
    // 如果位姿规划失败，继续尝试关节空间规划
    RCLCPP_WARN(logger, "位姿规划失败，尝试关节空间规划...");

  // 方法2：使用关节空间规划（更可靠）
  RCLCPP_INFO(logger, "=== 方法2：使用关节空间规划 ===");
  std::vector<double> target_joint_values = {0.0, -0.5, -0.5, 0.0};  // Joint1, Joint2, Joint3, Joint4
  RCLCPP_INFO(logger, "目标关节角度: J1=%.3f, J2=%.3f, J3=%.3f, J4=%.3f (弧度)", 
              target_joint_values[0], target_joint_values[1], 
              target_joint_values[2], target_joint_values[3]);
  
  move_group_interface.setJointValueTarget(target_joint_values);

  RCLCPP_INFO(logger, "开始关节空间规划...");
  auto const [joint_success, joint_plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // 执行关节空间规划
  if(joint_success) {
    RCLCPP_INFO(logger, "关节空间规划成功！轨迹包含 %zu 个点", joint_plan.trajectory_.joint_trajectory.points.size());
    RCLCPP_INFO(logger, "正在执行规划...");
      // 使用 execute()，配合后台的阻塞式 executor.spin() 来确保回调被处理
      // 后台线程已经在运行 executor.spin()，所以 execute() 的回调会被正确处理
    moveit::core::MoveItErrorCode result = move_group_interface.execute(joint_plan);
    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(logger, "执行完成！");
    } else {
      RCLCPP_ERROR(logger, "执行失败！错误代码: %d", result.val);
    }
      // 等待执行完成后的回调处理（确保 action result 回调被处理）
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      // 额外等待，确保机器人完全停止运动
      RCLCPP_INFO(logger, "等待机器人完全停止运动...");
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  } else {
    RCLCPP_ERROR(logger, "关节空间规划也失败！");
    RCLCPP_INFO(logger, "提示：");
    RCLCPP_INFO(logger, "1. 检查目标关节角度是否在关节限制范围内");
    RCLCPP_INFO(logger, "2. 检查是否有碰撞");
    RCLCPP_INFO(logger, "3. 检查运动学求解器是否正确配置");
  }
  }  // 结束 if (!all_stages_success) 分支
#endif
      // 在所有执行完成后，再次确认等待机器人完全停止
      RCLCPP_INFO(logger, "所有规划执行完成，等待机器人完全停止...");
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      
      // 规划完成，关闭 ROS（这会自动停止 executor.spin()）
      RCLCPP_INFO(logger, "规划任务完成，准备退出...");
  rclcpp::shutdown();
    });

  // 主线程运行 executor.spin()（常驻，处理所有 ROS 消息和回调）
  // 所有操作（包括 plan/execute）都在 executor 线程中执行，避免竞态问题
  RCLCPP_INFO(logger, "主线程开始运行 executor.spin()...");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();  // 阻塞式 spin，直到 rclcpp::shutdown() 被调用

  return 0;
}

