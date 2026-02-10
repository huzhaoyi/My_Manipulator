#ifndef SEALIEN_PAYLOAD_HARDWARE__SEALIEN_PAYLOAD_HARDWARE_INTERFACE_HPP_
#define SEALIEN_PAYLOAD_HARDWARE__SEALIEN_PAYLOAD_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "sealien_payload_hardware/json.hpp"

namespace sealien_payload_hardware
{
class SealienPayloadHardwareInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // UDP通信
  // UDP通信函数
  bool connect_to_robot();
  void disconnect_from_robot();
  bool send_command(const std::vector<double> & joint_positions);
  bool receive_feedback();
  void communication_thread();
  
  // UDP目标地址
  struct sockaddr_in server_addr_;

  // JSON解析
  bool parse_feedback_json(const std::string & json_str);

  /** 夹爪直接目标：从 /tmp/gripper_direct.txt 读取 (JointGL, JointGR) 弧度，文件 mtime 在有效期内则使用 */
  bool read_gripper_direct_file(double & gl, double & gr);
  
  // 参数
  std::string robot_ip_;
  int robot_port_;
  std::string local_ip_;  // 本地绑定IP（可选）
  int local_port_;        // 本地绑定端口（可选，-1表示不绑定）
  int socket_fd_;
  bool connected_;
  
  // 线程控制
  std::thread comm_thread_;
  std::atomic<bool> stop_thread_;
  std::atomic<bool> command_changed_;  // 命令值是否变化，需要发送
  std::atomic<bool> has_received_feedback_;  // 是否已经收到过反馈（只有在收到反馈后才允许发送命令）
  std::mutex data_mutex_;
  
  // 关节数据
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
  std::vector<double> hw_prev_commands_;  // 上一次的命令值，用于速度计算
  
  // 关节名称映射（ROS关节名 -> S3轴编号）
  std::map<std::string, int> joint_to_axis_map_;
  size_t joint_gl_idx_{0};
  size_t joint_gr_idx_{0};
  
  // 状态反馈
  std::vector<double> feedback_positions_;
  int robot_status_;
  
  // 安全参数
  double max_velocity_;
  double position_tolerance_;
  
  // 连接状态监控
  std::chrono::steady_clock::time_point last_successful_comm_;
};

}  // namespace sealien_payload_hardware

#endif  // SEALIEN_PAYLOAD_HARDWARE__SEALIEN_PAYLOAD_HARDWARE_INTERFACE_HPP_
