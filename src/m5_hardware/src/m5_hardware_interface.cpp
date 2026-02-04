#include "m5_hardware/m5_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <map>
#include <sstream>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <sys/stat.h>
#include <ctime>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

using json = nlohmann::json;

// ANSI颜色代码
#define COLOR_RESET   "\033[0m"
#define COLOR_RED     "\033[31m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_BLUE    "\033[34m"
#define COLOR_MAGENTA "\033[35m"
#define COLOR_CYAN    "\033[36m"
#define COLOR_WHITE   "\033[37m"
#define COLOR_BOLD    "\033[1m"

// 彩色日志宏
#define RCLCPP_INFO_COLOR(logger, color, ...) \
  RCLCPP_INFO_STREAM(logger, color << __VA_ARGS__ << COLOR_RESET)

#define RCLCPP_WARN_COLOR(logger, color, ...) \
  RCLCPP_WARN_STREAM(logger, color << __VA_ARGS__ << COLOR_RESET)

#define RCLCPP_ERROR_COLOR(logger, color, ...) \
  RCLCPP_ERROR_STREAM(logger, color << __VA_ARGS__ << COLOR_RESET)

namespace m5_hardware
{

hardware_interface::CallbackReturn M5HardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 初始化关节数据
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);
  hw_prev_commands_.resize(info_.joints.size(), 0.0);
  feedback_positions_.resize(info_.joints.size(), 0.0);
  
  // 从URDF获取初始位置
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    const auto & joint = info_.joints[i];
    for (const auto & state_if : joint.state_interfaces)
    {
      if (state_if.name == hardware_interface::HW_IF_POSITION)
      {
        // initial_value是字符串，需要解析
        if (!state_if.initial_value.empty())
        {
          try
          {
            double init_val = std::stod(state_if.initial_value);
            hw_positions_[i] = init_val;
            hw_commands_[i] = init_val;
            hw_prev_commands_[i] = init_val;
            feedback_positions_[i] = init_val;
            RCLCPP_INFO_COLOR(rclcpp::get_logger("M5HardwareInterface"), COLOR_CYAN,
              "关节 " << joint.name << " 初始位置: " << init_val << " rad (" 
              << init_val * 180.0 / M_PI << "°)");
          }
          catch (const std::exception & e)
          {
            RCLCPP_WARN_COLOR(rclcpp::get_logger("M5HardwareInterface"), COLOR_YELLOW,
              "无法解析关节 " << joint.name << " 的初始位置: " << state_if.initial_value);
          }
        }
      }
    }
  }

  // 从参数获取IP和端口
  robot_ip_ = info_.hardware_parameters.at("robot_ip");
  robot_port_ = std::stoi(info_.hardware_parameters.at("robot_port"));
  
  // 获取本地绑定IP和端口（可选参数）
  if (info_.hardware_parameters.count("local_ip") > 0)
  {
    local_ip_ = info_.hardware_parameters.at("local_ip");
  }
  else
  {
    local_ip_ = "";  // 空字符串表示不绑定本地IP
  }
  
  if (info_.hardware_parameters.count("local_port") > 0)
  {
    local_port_ = std::stoi(info_.hardware_parameters.at("local_port"));
  }
  else
  {
    local_port_ = -1;  // -1表示不绑定本地端口
  }
  
  // 初始化UDP目标地址
  memset(&server_addr_, 0, sizeof(server_addr_));
  server_addr_.sin_family = AF_INET;
  server_addr_.sin_port = htons(robot_port_);
  if (inet_pton(AF_INET, robot_ip_.c_str(), &server_addr_.sin_addr) <= 0)
  {
    RCLCPP_ERROR_COLOR(rclcpp::get_logger("M5HardwareInterface"), COLOR_RED, 
      "无效的IP地址: " << robot_ip_);
  }
  
  // 初始化关节到轴的映射（Joint1->axis1, Joint2->axis2, ...）
  // 注意：S3机械臂有5个轴，M5有4个主要关节（Joint1-4）和2个夹爪关节（JointGL, JointGR）
  // 只映射主要关节到S3的轴1-4
  std::map<std::string, int> joint_axis_mapping = {
    {"Joint1", 1},
    {"Joint2", 2},
    {"Joint3", 3},
    {"Joint4", 4}
    // JointGL和JointGR不映射到S3的轴（如果S3有第5个轴，可以在这里添加）
  };
  
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    const std::string & joint_name = info_.joints[i].name;
    if (joint_axis_mapping.find(joint_name) != joint_axis_mapping.end())
    {
      joint_to_axis_map_[joint_name] = joint_axis_mapping[joint_name];
      RCLCPP_INFO_COLOR(rclcpp::get_logger("M5HardwareInterface"), COLOR_CYAN,
        "映射关节 " << joint_name << " -> axis" << joint_axis_mapping[joint_name]);
    }
    if (joint_name == "JointGL") joint_gl_idx_ = i;
    else if (joint_name == "JointGR") joint_gr_idx_ = i;
  }

  // 安全参数
  max_velocity_ = std::stod(info_.hardware_parameters.count("max_velocity") > 0 
    ? info_.hardware_parameters.at("max_velocity") : "2.0");
  position_tolerance_ = std::stod(info_.hardware_parameters.count("position_tolerance") > 0 
    ? info_.hardware_parameters.at("position_tolerance") : "0.01");

  connected_ = false;
  socket_fd_ = -1;
  stop_thread_ = false;
  command_changed_ = false;
  has_received_feedback_ = false;  // 初始化：还没有收到反馈
  robot_status_ = 0;
  last_successful_comm_ = std::chrono::steady_clock::now();

  RCLCPP_INFO_COLOR(rclcpp::get_logger("M5HardwareInterface"), COLOR_GREEN,
    "硬件接口初始化完成，目标: " << robot_ip_ << ":" << robot_port_);
  if (!local_ip_.empty() || local_port_ >= 0)
  {
    RCLCPP_INFO_COLOR(rclcpp::get_logger("M5HardwareInterface"), COLOR_CYAN,
      "本地绑定: " << (local_ip_.empty() ? "自动" : local_ip_) 
      << ":" << (local_port_ < 0 ? "自动" : std::to_string(local_port_)));
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn M5HardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO_COLOR(rclcpp::get_logger("M5HardwareInterface"), COLOR_BLUE, "配置硬件接口...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> M5HardwareInterface::export_state_interfaces()
{
  // 按固定顺序导出，使 /joint_states 的 name 为 Joint1, Joint2, Joint3, Joint4, JointGL, JointGR（避免 2,3,1,4 乱序）
  const std::vector<std::string> desired_order = {"Joint1", "Joint2", "Joint3", "Joint4", "JointGL", "JointGR"};
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (const auto & name : desired_order)
  {
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      if (info_.joints[i].name != name) continue;
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
      break;
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> M5HardwareInterface::export_command_interfaces()
{
  const std::vector<std::string> desired_order = {"Joint1", "Joint2", "Joint3", "Joint4", "JointGL", "JointGR"};
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (const auto & name : desired_order)
  {
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      if (info_.joints[i].name != name) continue;
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
      break;
    }
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn M5HardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO_COLOR(rclcpp::get_logger("M5HardwareInterface"), COLOR_BLUE, "激活硬件接口...");
  
  // 连接到机器人
  if (!connect_to_robot())
  {
    RCLCPP_ERROR_COLOR(rclcpp::get_logger("M5HardwareInterface"), COLOR_RED, "无法连接到机器人");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 启动通信线程
  stop_thread_ = false;
  comm_thread_ = std::thread(&M5HardwareInterface::communication_thread, this);

  // 确保命令值有效（如果位置是NaN，使用0）
  for (size_t i = 0; i < hw_commands_.size(); ++i)
  {
    if (std::isnan(hw_positions_[i]) || std::isinf(hw_positions_[i]))
    {
      hw_positions_[i] = 0.0;
    }
    hw_commands_[i] = hw_positions_[i];
    hw_prev_commands_[i] = hw_positions_[i];  // 初始化上一次命令值
  }

  // 重要：确保不会因为初始化而触发发送命令
  // 只有在收到反馈后，才允许发送命令（避免开机时发送不必要的控制命令）
  command_changed_ = false;
  has_received_feedback_ = false;  // 重置反馈标志（激活时重置）

  RCLCPP_INFO_COLOR(rclcpp::get_logger("M5HardwareInterface"), COLOR_GREEN COLOR_BOLD, "硬件接口已激活 ✓");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn M5HardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO_COLOR(rclcpp::get_logger("M5HardwareInterface"), COLOR_YELLOW, "停用硬件接口...");
  
  // 停止通信线程
  stop_thread_ = true;
  if (comm_thread_.joinable())
  {
    comm_thread_.join();
  }

  // 断开连接
  disconnect_from_robot();

  RCLCPP_INFO_COLOR(rclcpp::get_logger("M5HardwareInterface"), COLOR_YELLOW, "硬件接口已停用");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type M5HardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Stamp 说明：joint_states 的 header.stamp 由 joint_state_broadcaster 在 update() 里设为
  // controller_manager 传入的 time（=本周期“当前时间”）。本接口只提供 position/velocity，不写 stamp。
  // 若 stamp 停住，多为：(1) read() 持锁过久拖慢 control cycle；(2) 勿用 UDP 包内/对端时间。
  // UDP→state 处从未用对端时间，仅解析关节位置（见 parse_feedback_json）。
  //
  // 持锁尽量短：只复制“最近一帧”和命令快照，在锁外写 hw_positions_，避免长时间阻塞 UDP 写入。
  std::vector<double> fb_snapshot;
  std::vector<double> cmd_snapshot;
  bool has_fb = false;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    fb_snapshot = feedback_positions_;
    cmd_snapshot = hw_commands_;
    has_fb = has_received_feedback_.load();
  }

  for (size_t i = 0; i < hw_positions_.size(); ++i)
  {
    if (i < fb_snapshot.size() && has_fb)
      hw_positions_[i] = fb_snapshot[i];
    else if (i < cmd_snapshot.size() && !std::isnan(cmd_snapshot[i]))
      hw_positions_[i] = cmd_snapshot[i];
    hw_velocities_[i] = 0.0;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type M5HardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 检测命令值是否变化
  std::lock_guard<std::mutex> lock(data_mutex_);
  
  bool has_change = false;
  const double tolerance = 1e-6;  // 很小的阈值，避免浮点误差
  
  for (size_t i = 0; i < hw_commands_.size(); ++i)
  {
    // 检查命令值是否有效且与上次不同
    if (!std::isnan(hw_commands_[i]) && !std::isinf(hw_commands_[i]))
    {
      if (i >= hw_prev_commands_.size() || 
          std::abs(hw_commands_[i] - hw_prev_commands_[i]) > tolerance)
      {
        has_change = true;
        break;
      }
    }
  }
  
  // 如果有变化，且已经收到过反馈，才标记需要发送
  // 这样可以避免在启动时（还未收到反馈）发送不必要的控制命令
  if (has_change && has_received_feedback_.load())
  {
    command_changed_ = true;
    // 更新上一次命令值
    for (size_t i = 0; i < hw_commands_.size(); ++i)
    {
      if (i < hw_prev_commands_.size())
      {
        hw_prev_commands_[i] = hw_commands_[i];
      }
    }
  }
  else if (has_change && !has_received_feedback_.load())
  {
    // 命令有变化但还没有收到反馈，只更新上一次命令值，不发送
    // 这样可以避免在启动时发送命令
    for (size_t i = 0; i < hw_commands_.size(); ++i)
    {
      if (i < hw_prev_commands_.size())
      {
        hw_prev_commands_[i] = hw_commands_[i];
      }
    }
  }

  return hardware_interface::return_type::OK;
}

bool M5HardwareInterface::connect_to_robot()
{
  // 使用UDP socket
  socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0)
  {
    RCLCPP_ERROR_COLOR(rclcpp::get_logger("M5HardwareInterface"), COLOR_RED, "创建UDP socket失败");
    return false;
  }

  // 如果需要绑定本地地址和端口
  if (!local_ip_.empty() || local_port_ >= 0)
  {
    struct sockaddr_in local_addr;
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    
    // 设置本地端口
    if (local_port_ >= 0)
    {
      local_addr.sin_port = htons(local_port_);
    }
    else
    {
      local_addr.sin_port = 0;  // 系统自动分配
    }
    
    // 设置本地IP
    if (!local_ip_.empty())
    {
      if (inet_pton(AF_INET, local_ip_.c_str(), &local_addr.sin_addr) <= 0)
      {
        RCLCPP_ERROR_COLOR(rclcpp::get_logger("M5HardwareInterface"), COLOR_RED, 
          "无效的本地IP地址: " << local_ip_);
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
      }
    }
    else
    {
      local_addr.sin_addr.s_addr = INADDR_ANY;  // 绑定到所有接口
    }
    
    // 绑定本地地址
    if (bind(socket_fd_, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0)
    {
      RCLCPP_ERROR_COLOR(rclcpp::get_logger("M5HardwareInterface"), COLOR_RED, 
        "绑定本地地址失败: " << strerror(errno) << " (IP: " 
        << (local_ip_.empty() ? "ANY" : local_ip_) 
        << ", Port: " << (local_port_ < 0 ? "AUTO" : std::to_string(local_port_)) << ")");
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }
    
    RCLCPP_INFO_COLOR(rclcpp::get_logger("M5HardwareInterface"), COLOR_CYAN,
      "已绑定本地地址: " << (local_ip_.empty() ? "ANY" : local_ip_) 
      << ":" << (local_port_ < 0 ? "AUTO" : std::to_string(local_port_)));
  }

  // 设置socket为非阻塞模式（可选，用于接收）
  int flags = fcntl(socket_fd_, F_GETFL, 0);
  fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);
  
  // UDP不需要connect，但可以设置socket选项
  int broadcast = 1;
  if (setsockopt(socket_fd_, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0)
  {
    RCLCPP_WARN_COLOR(rclcpp::get_logger("M5HardwareInterface"), COLOR_YELLOW,
      "设置UDP广播选项失败（不影响使用）");
  }

  connected_ = true;
  last_successful_comm_ = std::chrono::steady_clock::now();
  RCLCPP_INFO_COLOR(rclcpp::get_logger("M5HardwareInterface"), COLOR_GREEN COLOR_BOLD,
    "UDP socket已创建，目标: " << robot_ip_ << ":" << robot_port_);
  return true;
}

void M5HardwareInterface::disconnect_from_robot()
{
  if (socket_fd_ >= 0)
  {
    close(socket_fd_);
    socket_fd_ = -1;
    connected_ = false;
    RCLCPP_INFO_COLOR(rclcpp::get_logger("M5HardwareInterface"), COLOR_YELLOW, "已断开与机器人的连接");
  }
}

namespace
{
constexpr const char * GRIPPER_DIRECT_FILE = "/tmp/gripper_direct.txt";
constexpr int GRIPPER_DIRECT_VALID_SEC = 15;
constexpr const char * UDP_FEEDBACK_FILE = "/tmp/udp_feedback.txt";  // 供 m5_grasp 直接读 UDP 反馈（夹爪），不依赖 /joint_states
}

bool M5HardwareInterface::read_gripper_direct_file(double & gl, double & gr)
{
  struct stat st;
  if (stat(GRIPPER_DIRECT_FILE, &st) != 0)
    return false;
  std::ifstream f(GRIPPER_DIRECT_FILE);
  if (!f || !(f >> gl >> gr))
    return false;
  time_t now_sec = time(nullptr);
  if (now_sec - st.st_mtime > GRIPPER_DIRECT_VALID_SEC)
    return false;
  return !std::isnan(gl) && !std::isnan(gr) && !std::isinf(gl) && !std::isinf(gr);
}

bool M5HardwareInterface::send_command(const std::vector<double> & joint_positions)
{
  if (!connected_ || socket_fd_ < 0)
  {
    return false;
  }

  // 构建JSON命令，格式：{"pos": [axis1, axis2, axis3, axis4, axis5, key]}
  json command_obj;
  json pos_array = json::array();
  
  // 按照轴顺序构建数组：axis1, axis2, axis3, axis4, axis5, key
  // 初始化5个轴和1个key值（共6个元素）
  std::vector<double> axis_values(6, 0.0);  // [axis1, axis2, axis3, axis4, axis5, key]
  
  // 根据关节映射填充轴值
  for (const auto & pair : joint_to_axis_map_)
  {
    const std::string & joint_name = pair.first;
    int axis_num = pair.second;  // 1-4
    
    // 找到对应的关节索引
    size_t joint_idx = 0;
    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      if (info_.joints[i].name == joint_name)
      {
        joint_idx = i;
        break;
      }
    }
    
    if (joint_idx < joint_positions.size())
    {
      double pos = joint_positions[joint_idx];
      // 检查是否为有效数值
      if (!std::isnan(pos) && !std::isinf(pos))
      {
        // 转换为度并存储到对应轴位置（axis_num从1开始，数组索引从0开始）
        if (axis_num >= 1 && axis_num <= 4)
        {
          axis_values[axis_num - 1] = pos * 180.0 / M_PI;
        }
      }
      else
      {
        RCLCPP_WARN_COLOR(rclcpp::get_logger("M5HardwareInterface"), COLOR_YELLOW,
          "关节 " << joint_name << " 位置无效 (NaN/Inf)，使用0");
      }
    }
  }
  
  // 处理axis5（夹爪）：将JointGL和JointGR映射到axis5
  // 由于JointGL和JointGR是镜像对称的，使用JointGL的值
  size_t joint_gl_idx = info_.joints.size();  // 初始化为无效索引
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    if (info_.joints[i].name == "JointGL")
    {
      joint_gl_idx = i;
      break;
    }
  }
  
  if (joint_gl_idx < info_.joints.size() && joint_gl_idx < joint_positions.size())
  {
    double joint_gl_pos = joint_positions[joint_gl_idx];
    if (!std::isnan(joint_gl_pos) && !std::isinf(joint_gl_pos))
    {
      // 映射：JointGL [−1.01, 1.01] 弧度 -> axis5 [-1100, 0]（厂家实际范围，之前 -2505~5 为留余）
      // 闭合 → axis5=0，张开 → axis5=-1100
      const double joint_min = -1.01;    // 弧度（打开）
      const double joint_max = 1.01;      // 弧度（闭合）
      const double axis5_min = -1100.0;   // 范围值（打开，厂家建议）
      const double axis5_max = 0.0;       // 范围值（闭合）
      
      // 限制在有效范围内
      joint_gl_pos = std::max(joint_min, std::min(joint_max, joint_gl_pos));
      
      // 线性映射：JointGL 弧度 -> axis5 范围值
      // axis5 = (joint_gl_pos - joint_min) / (joint_max - joint_min) * (axis5_max - axis5_min) + axis5_min
      double axis5_value = (joint_gl_pos - joint_min) / (joint_max - joint_min) * (axis5_max - axis5_min) + axis5_min;
      
      // 验证：joint_gl_pos = -1.01 -> axis5 = -2505，joint_gl_pos = 1.01 -> axis5 = 5
      // 添加调试日志（打印夹爪变化）
      // static double last_axis5_sent = 0.0;
      // if (std::abs(axis5_value - last_axis5_sent) > 10.0)  // 变化超过10度才打印
      // {
      //   RCLCPP_INFO(rclcpp::get_logger("M5HardwareInterface"),
      //     "[夹爪发送] JointGL=%.3f rad (%.1f°) -> axis5=%.1f° (上次: %.1f°)",
      //     joint_gl_pos, joint_gl_pos * 180.0 / M_PI, axis5_value, last_axis5_sent);
      //   last_axis5_sent = axis5_value;
      // }
      
      axis_values[4] = axis5_value;  // axis5是第5个元素（索引4）
    }
  }

  // 将轴值添加到数组（axis5已填充，key保持为0）
  for (size_t i = 0; i < 6; ++i)
  {
    pos_array.push_back(axis_values[i]);
  }
  
  // 手动构建JSON字符串，确保格式完全一致：{"pos":[30,0,0,0,0,0]}
  std::ostringstream json_stream;
  json_stream << "{\"pos\":[";
  for (size_t i = 0; i < pos_array.size(); ++i)
  {
    if (i > 0)
    {
      json_stream << ",";
    }
    // 直接输出数值，不添加小数点后的0（如果可能）
    double val = pos_array[i].get<double>();
    if (val == static_cast<int>(val))
    {
      json_stream << static_cast<int>(val);
    }
    else
    {
      json_stream << val;
    }
  }
  json_stream << "]}";
  std::string json_str = json_stream.str();

  // 打印发给机械臂的 JSON
  // arm = axis1~4(度)，夹爪 = axis5(范围值 -1100~0，厂家建议)
  // {
  //   static std::string last_printed_json;
  //   if (last_printed_json != json_str)
  //   {
  //     last_printed_json = json_str;
  //     RCLCPP_INFO_COLOR(rclcpp::get_logger("M5HardwareInterface"), COLOR_CYAN,
  //       "[发给机械臂] arm(度)=[" << axis_values[0] << ", " << axis_values[1] << ", " << axis_values[2]
  //       << ", " << axis_values[3] << "] 夹爪(axis5)=" << axis_values[4] << " | JSON: " << json_str);
  //   }
  // }
  
  // 使用UDP sendto发送命令
  ssize_t sent = sendto(socket_fd_, json_str.c_str(), json_str.length(), 0,
                        (struct sockaddr *)&server_addr_, sizeof(server_addr_));
  if (sent < 0)
  {
    RCLCPP_ERROR_COLOR(rclcpp::get_logger("M5HardwareInterface"), COLOR_RED, 
      "UDP发送命令失败: " << strerror(errno));
    connected_ = false;
    return false;
  }

  last_successful_comm_ = std::chrono::steady_clock::now();
  
  return true;
}

bool M5HardwareInterface::receive_feedback()
{
  if (!connected_ || socket_fd_ < 0)
  {
    return false;
  }

  static std::string buffer_remainder = "";  // 保存不完整的数据
  char buffer[4096] = {0};
  struct sockaddr_in from_addr;
  socklen_t from_len = sizeof(from_addr);
  
  // 使用UDP recvfrom接收数据
  ssize_t received = recvfrom(socket_fd_, buffer, sizeof(buffer) - 1, MSG_DONTWAIT,
                              (struct sockaddr *)&from_addr, &from_len);
  
  if (received < 0)
  {
    if (errno != EAGAIN && errno != EWOULDBLOCK)
    {
      RCLCPP_WARN_COLOR(rclcpp::get_logger("M5HardwareInterface"), COLOR_YELLOW, 
        "接收数据错误: " << strerror(errno));
      connected_ = false;  // 标记连接断开
      return false;
    }
    // 没有数据可读（正常情况），不打印
    return true;
  }

  if (received == 0)
  {
    RCLCPP_WARN_COLOR(rclcpp::get_logger("M5HardwareInterface"), COLOR_YELLOW, "连接已关闭");
    connected_ = false;
    return false;
  }
  
  // 收到数据，打印调试信息（已禁用）
  // static int recv_debug_count = 0;
  // recv_debug_count++;
  buffer[received] = '\0';
  // if (recv_debug_count <= 10 || recv_debug_count % 10 == 0)
  // {
  //   RCLCPP_INFO_COLOR(rclcpp::get_logger("M5HardwareInterface"), COLOR_GREEN,
  //     "[UDP接收 #" << recv_debug_count << "] 收到 " << received << " 字节: " << std::string(buffer, received));
  // }

  buffer[received] = '\0';
  std::string full_data = buffer_remainder + std::string(buffer);
  buffer_remainder = "";

  // 尝试解析JSON（可能包含多个JSON对象或部分数据）
  // 查找完整的JSON数组（以[开始，以]结束）
  size_t start_pos = full_data.find('[');
  if (start_pos != std::string::npos)
  {
    size_t bracket_count = 0;
    size_t end_pos = start_pos;
    for (size_t i = start_pos; i < full_data.length(); ++i)
    {
      if (full_data[i] == '[') bracket_count++;
      if (full_data[i] == ']') bracket_count--;
      if (bracket_count == 0)
      {
        end_pos = i + 1;
        break;
      }
    }
    
    if (bracket_count == 0)
    {
      // 找到完整的JSON
      std::string json_str = full_data.substr(start_pos, end_pos - start_pos);
      if (parse_feedback_json(json_str))
      {
        // 处理剩余数据
        if (end_pos < full_data.length())
        {
          buffer_remainder = full_data.substr(end_pos);
        }
        return true;
      }
    }
    else
    {
      // JSON不完整，保存等待更多数据
      buffer_remainder = full_data.substr(start_pos);
    }
  }
  else
  {
    // 没有找到JSON开始标记，可能是部分数据
    buffer_remainder = full_data;
  }

  return true;
}

bool M5HardwareInterface::parse_feedback_json(const std::string & json_str)
{
  try
  {
    json root = json::parse(json_str);
    if (!root.is_array())
    {
      RCLCPP_WARN_COLOR(rclcpp::get_logger("M5HardwareInterface"), COLOR_YELLOW, "JSON不是数组格式");
      return false;
    }

    // 锁外：仅复制“上一帧”用于未出现在本包里的关节，缩短持锁时间
    std::vector<double> prev;
    int new_status = 0;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      prev = feedback_positions_;
      new_status = robot_status_;
      for (size_t i = 0; i < prev.size(); ++i)
      {
        if (std::isnan(prev[i]) && i < hw_commands_.size() && !std::isnan(hw_commands_[i]))
          prev[i] = hw_commands_[i];
      }
    }

    // 锁外：解析 UDP JSON → “最新一帧”关节位置。从不使用对端/UDP 包内时间，不写任何 stamp。
    // joint_states 的 stamp 由 joint_state_broadcaster 在 update() 里用 controller 的 time（本机 ROS 时钟）设置。
    std::vector<double> new_fb = prev;
    for (const auto & item : root)
    {
      if (item.contains("num") && item.contains("value"))
      {
        int axis_num = item["num"].get<int>();
        double value = item["value"].get<double>();
        if (axis_num == 5)
        {
          // 实机 axis5：0=闭合，-1100=打开 → JointGL 闭合=1.01、打开=-1.01
          const double axis5_min = -1100.0, axis5_max = 0.0, joint_min = -1.01, joint_max = 1.01;
          double value_rad = (value - axis5_min) / (axis5_max - axis5_min) * (joint_max - joint_min) + joint_min;
          value_rad = std::max(joint_min, std::min(joint_max, value_rad));
          for (size_t i = 0; i < info_.joints.size(); ++i)
          {
            if (info_.joints[i].name == "JointGL") new_fb[i] = value_rad;
            else if (info_.joints[i].name == "JointGR") new_fb[i] = -value_rad;
          }
        }
        else
        {
          double value_rad = value * M_PI / 180.0;
          for (const auto & pair : joint_to_axis_map_)
          {
            if (pair.second == axis_num)
            {
              const std::string & joint_name = pair.first;
              for (size_t i = 0; i < info_.joints.size(); ++i)
              {
                if (info_.joints[i].name == joint_name) { new_fb[i] = value_rad; break; }
              }
              break;
            }
          }
        }
      }
      else if (item.contains("status"))
        new_status = item["status"].get<int>();
    }

    double gl_fb = 0.0, gr_fb = 0.0;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      feedback_positions_ = std::move(new_fb);
      robot_status_ = new_status;
      last_successful_comm_ = std::chrono::steady_clock::now();
      if (joint_gl_idx_ < feedback_positions_.size() && joint_gr_idx_ < feedback_positions_.size())
      {
        gl_fb = feedback_positions_[joint_gl_idx_];
        gr_fb = feedback_positions_[joint_gr_idx_];
      }
      if (!has_received_feedback_.load())
      {
        has_received_feedback_ = true;
        RCLCPP_INFO_COLOR(rclcpp::get_logger("M5HardwareInterface"), COLOR_GREEN,
          "首次收到反馈数据！同步命令值到当前位置。");
        for (size_t j = 0; j < hw_commands_.size() && j < feedback_positions_.size(); ++j)
        {
          hw_commands_[j] = feedback_positions_[j];
          hw_prev_commands_[j] = feedback_positions_[j];
        }
      }
    }
    // 写 UDP 反馈到文件，供 m5_grasp 夹爪直接读（不依赖 /joint_states）
    std::ofstream uf(UDP_FEEDBACK_FILE);
    if (uf)
      uf << gl_fb << " " << gr_fb << "\n";
    return true;
  }
  catch (const json::exception & e)
  {
    RCLCPP_WARN_COLOR(rclcpp::get_logger("M5HardwareInterface"), COLOR_YELLOW,
      "JSON解析失败: " << e.what());
    return false;
  }
}

void M5HardwareInterface::communication_thread()
{
  RCLCPP_INFO_COLOR(rclcpp::get_logger("M5HardwareInterface"), COLOR_CYAN COLOR_BOLD, "通信线程启动");
  
  auto last_reconnect_attempt = std::chrono::steady_clock::now();
  auto last_command_time = std::chrono::steady_clock::now();
  const auto reconnect_interval = std::chrono::seconds(2);  // 2秒后尝试重连
  const auto command_interval = std::chrono::milliseconds(20);  // 50Hz，每20ms发送一次（提高响应速度）

  while (!stop_thread_)
  {
    // 检查连接状态
    if (!connected_)
    {
      auto now = std::chrono::steady_clock::now();
      if (now - last_reconnect_attempt >= reconnect_interval)
      {
        RCLCPP_WARN_COLOR(rclcpp::get_logger("M5HardwareInterface"), COLOR_YELLOW, "尝试重新连接...");
        if (connect_to_robot())
        {
          RCLCPP_INFO_COLOR(rclcpp::get_logger("M5HardwareInterface"), COLOR_GREEN, "重连成功 ✓");
        }
        last_reconnect_attempt = now;
      }
    }
    else
    {
      // 接收反馈
      if (!receive_feedback())
      {
        // 接收失败，可能连接断开
        if (connected_)
        {
          RCLCPP_WARN_COLOR(rclcpp::get_logger("M5HardwareInterface"), COLOR_YELLOW, 
            "接收反馈失败，连接可能断开");
        }
      }

      // 固定周期发送命令（50Hz），不管命令是否变化都发送
      // 连接后即按周期发送，不依赖“先收到反馈”；否则对方不先发时 UDP 永远发不出去
      auto now = std::chrono::steady_clock::now();
      bool should_send = (now - last_command_time >= command_interval);
      
      if (should_send)
      {
        std::vector<double> to_send;
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          to_send = hw_commands_;
        }
        double gl_override = 0.0, gr_override = 0.0;
        if (read_gripper_direct_file(gl_override, gr_override) &&
            joint_gl_idx_ < to_send.size() && joint_gr_idx_ < to_send.size())
        {
          to_send[joint_gl_idx_] = gl_override;
          to_send[joint_gr_idx_] = gr_override;
        }
        if (send_command(to_send))
        {
          last_command_time = now;
          command_changed_ = false;
          last_successful_comm_ = std::chrono::steady_clock::now();
        }
        else if (connected_)
        {
          RCLCPP_WARN_COLOR(rclcpp::get_logger("M5HardwareInterface"), COLOR_YELLOW, "发送命令失败");
        }
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  RCLCPP_INFO_COLOR(rclcpp::get_logger("M5HardwareInterface"), COLOR_CYAN, "通信线程结束");
}

}  // namespace m5_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(m5_hardware::M5HardwareInterface, hardware_interface::SystemInterface)

