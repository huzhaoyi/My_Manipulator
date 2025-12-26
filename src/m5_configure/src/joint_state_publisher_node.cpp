#include <memory>
#include <vector>
#include <string>
#include <mutex>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

class JointStatePublisherNode : public rclcpp::Node
{
public:
  JointStatePublisherNode()
    : Node("m5_joint_state_publisher")
  {
    // 声明参数：是否启用重新发布（默认启用）
    this->declare_parameter<bool>("republish", true);
    bool republish = this->get_parameter("republish").as_bool();

    if (republish)
    {
      // 创建订阅者，订阅 joint_state_broadcaster 发布的原始消息
      // 注意：为了避免循环，我们需要让 joint_state_broadcaster 发布到另一个 topic
      // 或者，我们可以直接替换 joint_state_broadcaster 的功能
      // 这里我们假设 joint_state_broadcaster 发布到 /joint_states_raw
      // 如果不存在，我们可以通过 remap 来改变 joint_state_broadcaster 的发布 topic
      joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states_raw",  // 需要配置 joint_state_broadcaster 发布到这里
        rclcpp::QoS(10),
        std::bind(&JointStatePublisherNode::jointStateCallback, this, std::placeholders::_1)
      );

      // 创建发布者，发布带有正确时间戳的 joint_states
      joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", 
        rclcpp::QoS(10)
      );

      // 存储最后接收到的消息
      last_msg_ = std::make_shared<sensor_msgs::msg::JointState>();
      last_msg_received_ = false;
      last_publish_time_ = std::chrono::steady_clock::now();

      // 创建定时器，定期发布（即使没有新消息，也确保时间戳更新）
      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),  // 100Hz
        std::bind(&JointStatePublisherNode::timerCallback, this)
      );

      RCLCPP_INFO(this->get_logger(), "JointStatePublisherNode 已启动（重新发布模式）");
      RCLCPP_INFO(this->get_logger(), "将确保每条 /joint_states 消息都使用 node->now() 设置时间戳");
      RCLCPP_WARN(this->get_logger(), "注意：需要配置 joint_state_broadcaster 发布到 /joint_states_raw");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "JointStatePublisherNode 已启动（禁用模式）");
    }
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    
    // 保存消息数据（但不保存时间戳）
    last_msg_->header.frame_id = msg->header.frame_id;
    last_msg_->name = msg->name;
    last_msg_->position = msg->position;
    last_msg_->velocity = msg->velocity;
    last_msg_->effort = msg->effort;
    last_msg_received_ = true;
    last_publish_time_ = std::chrono::steady_clock::now();
    
    // 立即发布，使用当前时间戳
    publishWithCurrentTime();
  }

  void timerCallback()
  {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    
    // 如果有消息，定期发布以确保时间戳是最新的
    // 但不要发布太频繁（避免循环）
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_publish_time_).count();
    
    if (last_msg_received_ && elapsed >= 10)  // 至少间隔 10ms
    {
      publishWithCurrentTime();
      last_publish_time_ = now;
    }
  }

  void publishWithCurrentTime()
  {
    if (!last_msg_received_)
    {
      return;
    }

    // 创建新消息，复制所有数据
    auto new_msg = sensor_msgs::msg::JointState();
    new_msg.header.frame_id = last_msg_->header.frame_id;
    
    // 关键：使用 node->now() 设置时间戳
    new_msg.header.stamp = this->now();
    
    new_msg.name = last_msg_->name;
    new_msg.position = last_msg_->position;
    new_msg.velocity = last_msg_->velocity;
    new_msg.effort = last_msg_->effort;
    
    // 发布带有正确时间戳的消息
    joint_state_pub_->publish(new_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex msg_mutex_;
  sensor_msgs::msg::JointState::SharedPtr last_msg_;
  bool last_msg_received_;
  std::chrono::steady_clock::time_point last_publish_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<JointStatePublisherNode>();
  
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}

