#include <memory>
#include <vector>
#include <string>
#include <mutex>
#include <chrono>
#include <map>
#include <algorithm>
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
      // 定义正确的关节顺序
      correct_joint_order_ = {"Joint1", "Joint2", "Joint3", "Joint4", "JointGL", "JointGR"};
      
      // 声明参数：原始话题名称（默认从 /joint_states_raw 订阅，如果不存在则从 /joint_states 订阅）
      this->declare_parameter<std::string>("source_topic", "/joint_states_raw");
      std::string source_topic = this->get_parameter("source_topic").as_string();
      
      // 声明参数：输出话题名称（默认发布到 /joint_states_fixed，避免与 joint_state_broadcaster 冲突）
      this->declare_parameter<std::string>("output_topic", "/joint_states_fixed");
      std::string output_topic = this->get_parameter("output_topic").as_string();
      
      // 创建订阅者，订阅 joint_state_broadcaster 发布的原始消息
      // 订阅原始话题，然后重新排序后发布
      joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        source_topic,  // 原始话题（可通过参数配置）
        rclcpp::QoS(10),
        std::bind(&JointStatePublisherNode::jointStateCallback, this, std::placeholders::_1)
      );

      // 创建发布者，发布带有正确时间戳和顺序的 joint_states
      // 注意：发布到 /joint_states_fixed 而不是 /joint_states，避免与 joint_state_broadcaster 冲突
      joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        output_topic,  // 输出话题（可通过参数配置，默认 /joint_states_fixed）
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
      RCLCPP_INFO(this->get_logger(), "订阅话题: %s", source_topic.c_str());
      RCLCPP_INFO(this->get_logger(), "发布话题: %s (避免与 joint_state_broadcaster 冲突)", output_topic.c_str());
      RCLCPP_INFO(this->get_logger(), "将确保每条消息都使用 node->now() 设置时间戳");
      RCLCPP_INFO(this->get_logger(), "关节顺序将调整为: Joint1, Joint2, Joint3, Joint4, JointGL, JointGR");
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

    // 创建新消息
    auto new_msg = sensor_msgs::msg::JointState();
    new_msg.header.frame_id = last_msg_->header.frame_id;
    
    // 关键：使用 node->now() 设置时间戳
    new_msg.header.stamp = this->now();
    
    // 创建映射表，将关节名称映射到其索引
    std::map<std::string, size_t> joint_index_map;
    for (size_t i = 0; i < last_msg_->name.size(); ++i)
    {
      joint_index_map[last_msg_->name[i]] = i;
    }
    
    // 按照正确的顺序重新排列关节数据
    for (const auto& joint_name : correct_joint_order_)
    {
      auto it = joint_index_map.find(joint_name);
      if (it != joint_index_map.end())
      {
        size_t idx = it->second;
        new_msg.name.push_back(joint_name);
        if (idx < last_msg_->position.size())
          new_msg.position.push_back(last_msg_->position[idx]);
        else
          new_msg.position.push_back(0.0);
        
        if (idx < last_msg_->velocity.size())
          new_msg.velocity.push_back(last_msg_->velocity[idx]);
        else
          new_msg.velocity.push_back(0.0);
        
        if (idx < last_msg_->effort.size())
          new_msg.effort.push_back(last_msg_->effort[idx]);
        else
          new_msg.effort.push_back(0.0);
      }
    }
    
    // 发布带有正确时间戳和顺序的消息
    joint_state_pub_->publish(new_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex msg_mutex_;
  sensor_msgs::msg::JointState::SharedPtr last_msg_;
  bool last_msg_received_;
  std::chrono::steady_clock::time_point last_publish_time_;
  std::vector<std::string> correct_joint_order_;  // 正确的关节顺序
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

