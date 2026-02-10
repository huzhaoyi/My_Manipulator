#include "sealien_payload_grasp/execution/gripper_controller.hpp"
#include "sealien_payload_grasp/logging/logger.hpp"
#include "sealien_payload_grasp/mtc/task_context.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <sys/stat.h>
#include <thread>

namespace sealien_payload_grasp
{

// 夹爪直接目标文件：与 m5_hardware 通信线程共用，写 (JointGL, JointGR) 弧度，硬件合并后发 UDP
constexpr const char* GRIPPER_DIRECT_FILE = "/tmp/gripper_direct.txt";
// UDP 反馈文件：m5_hardware 每收到 UDP 反馈即写 (JointGL, JointGR) 弧度，夹爪优先读此文件，不依赖
// /joint_states
constexpr const char* UDP_FEEDBACK_FILE = "/tmp/udp_feedback.txt";
constexpr int UDP_FEEDBACK_FRESH_SEC = 1; // 文件 mtime 在此秒数内视为有效（直接读 UDP 反馈）

GripperController::GripperController(
    rclcpp::Node* node,
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_interface,
    std::mutex& moveit_mutex)
    : node_(node), gripper_interface_(gripper_interface), moveit_mutex_(moveit_mutex)
{
}

void GripperController::set_parameters(double open_width, double close_width, double close_extra,
                                      int hold_time_ms, double angle_min, double angle_max,
                                      double width_min, int stable_samples,
                                      double stable_threshold_rad, double stable_velocity_eps_rad_s,
                                      int stable_interval_ms, int post_execute_wait_ms,
                                      double motion_start_threshold_rad,
                                      int motion_start_timeout_ms, int stable_timeout_ms)
{
    open_width_ = open_width;
    close_width_ = close_width;
    close_extra_ = close_extra;
    hold_time_ms_ = hold_time_ms;
    angle_min_ = angle_min;
    angle_max_ = angle_max;
    width_min_ = width_min;
    stable_samples_ = std::max(2, stable_samples);
    stable_threshold_rad_ = stable_threshold_rad;
    stable_velocity_eps_rad_s_ = stable_velocity_eps_rad_s;
    stable_interval_ms_ = std::max(10, stable_interval_ms);
    post_execute_wait_ms_ = std::max(0, post_execute_wait_ms);
    motion_start_threshold_rad_ = std::max(1e-6, motion_start_threshold_rad);
    motion_start_timeout_ms_ = std::max(0, motion_start_timeout_ms);
    stable_timeout_ms_ = std::max(0, stable_timeout_ms);
}

bool GripperController::is_feedback_stable()
{
    if (stable_samples_ < 2)
        return true;
    std::vector<std::pair<double, double>> samples;
    samples.reserve(static_cast<size_t>(stable_samples_));
    for (int i = 0; i < stable_samples_; ++i)
    {
        std::vector<double> j = get_current_joint_values();
        if (j.size() < 2u)
            return false;
        samples.emplace_back(j[0], j[1]);
        if (i < stable_samples_ - 1)
            std::this_thread::sleep_for(std::chrono::milliseconds(stable_interval_ms_));
    }
    for (size_t i = 1; i < samples.size(); ++i)
    {
        double d_gl = std::abs(samples[i].first - samples[i - 1].first);
        double d_gr = std::abs(samples[i].second - samples[i - 1].second);
        if (d_gl > stable_threshold_rad_ || d_gr > stable_threshold_rad_)
            return false;
    }
    return true;
}

bool GripperController::wait_for_motion_start()
{
    if (motion_start_timeout_ms_ <= 0)
        return true;
    std::vector<double> prev = get_current_joint_values();
    if (prev.size() < 2u)
        return false;
    auto start = std::chrono::steady_clock::now();
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(stable_interval_ms_));
        std::vector<double> cur = get_current_joint_values();
        if (cur.size() < 2u)
            continue;
        double d_gl = std::abs(cur[0] - prev[0]);
        double d_gr = std::abs(cur[1] - prev[1]);
        if (d_gl >= motion_start_threshold_rad_ || d_gr >= motion_start_threshold_rad_)
        {
            LOG_NAMED_INFO("gripper", "夹爪运动已开始（数值变化 d_gl={:.4f} d_gr={:.4f} rad）",
                           d_gl, d_gr);
            return true;
        }
        prev = cur;
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::steady_clock::now() - start)
                           .count();
        if (elapsed >= motion_start_timeout_ms_)
        {
            LOG_NAMED_INFO("gripper", "等待运动开始超时 {} ms（可能已在目标），进入判稳",
                           motion_start_timeout_ms_);
            return false;
        }
    }
}

bool GripperController::wait_until_stable()
{
    if (stable_timeout_ms_ <= 0)
        return is_feedback_stable();
    auto start = std::chrono::steady_clock::now();
    while (true)
    {
        if (is_feedback_stable())
            return true;
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                           std::chrono::steady_clock::now() - start)
                           .count();
        if (elapsed >= stable_timeout_ms_)
            return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(stable_interval_ms_));
    }
}

double GripperController::width_to_joint_angle(double width)
{
    // 线性映射
    const double width_min = width_min_;
    const double width_max = open_width_;
    const double angle_min = angle_min_; // 闭合（对应 axis5 = 0°）
    const double angle_max = angle_max_; // 张开（对应 axis5 = -1100°）

    // 除零保护
    if (width_max <= width_min)
    {
        LOG_NAMED_ERROR(
            "gripper",
            "widthToJointAngle: width_max ({:.3f}) <= width_min ({:.3f})，配置错误！返回默认角度",
            width_max, width_min);
        return angle_min;
    }

    width = std::max(width_min, std::min(width_max, width));
    double angle =
        (width - width_min) / (width_max - width_min) * (angle_max - angle_min) + angle_min;
    return angle;
}

double GripperController::joint_angle_to_width(double joint_gl) const
{
    const double width_min = width_min_;
    const double width_max = open_width_;
    const double angle_min = angle_min_;
    const double angle_max = angle_max_;
    if (std::abs(angle_max - angle_min) < 1e-9)
        return width_min;
    double width =
        width_min + (joint_gl - angle_min) / (angle_max - angle_min) * (width_max - width_min);
    return std::max(width_min, std::min(width_max, width));
}

std::vector<double> GripperController::get_current_joint_values()
{
    // 优先直接读 UDP 反馈文件（硬件每收到 UDP 即写），不依赖 /joint_states，避免 MoveGroup “latest
    // state time 0” 报错
    struct stat st;
    if (stat(UDP_FEEDBACK_FILE, &st) == 0)
    {
        time_t now_sec = time(nullptr);
        if (now_sec - st.st_mtime <= UDP_FEEDBACK_FRESH_SEC)
        {
            std::ifstream f(UDP_FEEDBACK_FILE);
            double gl = 0.0, gr = 0.0;
            if (f && (f >> gl >> gr) && !std::isnan(gl) && !std::isnan(gr))
                return {gl, gr};
        }
    }
    // 其次用 joint_states 诊断缓存
    if (joint_state_diag_ && node_)
    {
        std::vector<double> out;
        const std::vector<std::string> grip_names = {"JointGL", "JointGR"};
        auto now = node_->get_clock()->now();
        if (fill_joint_positions_from_diag(joint_state_diag_, grip_names, out, now, 5.0) &&
            out.size() >= 2u)
        {
            return out;
        }
    }
    // 最后才用 MoveGroup（可能触发 Failed to fetch current robot state）
    std::lock_guard<std::mutex> lock(moveit_mutex_);
    try
    {
        std::vector<double> raw = gripper_interface_->getCurrentJointValues();
        const std::vector<std::string>& names = gripper_interface_->getActiveJoints();
        if (raw.size() != names.size() || raw.size() < 2u)
        {
            return raw.size() >= 2u ? raw : std::vector<double>{};
        }
        double gl = 0.0, gr = 0.0;
        for (size_t i = 0; i < names.size(); ++i)
        {
            if (names[i] == "JointGL")
                gl = raw[i];
            else if (names[i] == "JointGR")
                gr = raw[i];
        }
        return {gl, gr};
    }
    catch (const std::exception& e)
    {
        LOG_NAMED_WARN("gripper", "getCurrentJointValues 异常: {}", e.what());
        return {};
    }
}

bool GripperController::is_open_to_target()
{
    // 保持粘性：验证通过后持续返回 true，供 FSM 在 min_dwell 满足后转 PREGRASP；仅在新发 open/close
    // 时清除
    return last_open_verified_stable_;
}

bool GripperController::is_closed_to_target()
{
    // 保持粘性：验证通过后持续返回 true，供 FSM 在 min_dwell 满足后转 LIFT；仅在新发 open/close
    // 时清除
    return last_close_verified_stable_;
}

bool GripperController::open()
{
    return open_to_width(-1.0);
}

bool GripperController::close()
{
    return close_to_width(-1.0);
}

bool GripperController::open_to_width(double width)
{
    last_close_verified_stable_ = false;
    last_open_verified_stable_ = false;

    if (width < 0.0)
    {
        width = open_width_;
        LOG_NAMED_INFO("gripper", "[夹爪打开] 使用配置参数: open_width={:.3f} m", open_width_);
    }
    else
    {
        LOG_NAMED_INFO("gripper", "[夹爪打开] 使用指定宽度: width={:.3f} m", width);
    }

    double joint_gl = width_to_joint_angle(width);
    double joint_gr = -joint_gl; // 镜像对称
    {
        std::ofstream f(GRIPPER_DIRECT_FILE);
        if (!f)
        {
            LOG_NAMED_WARN("gripper", "[夹爪打开] 无法写入 {}，夹爪直接目标未下发",
                           GRIPPER_DIRECT_FILE);
            return false;
        }
        f << joint_gl << " " << joint_gr << "\n";
    }
    LOG_NAMED_INFO(
        "gripper",
        "[夹爪打开] 已写目标 JointGL={:.4f} rad, JointGR={:.4f} rad（0=闭合 -1100=全开，直接 UDP）",
        joint_gl, joint_gr);

    if (post_execute_wait_ms_ > 0)
        std::this_thread::sleep_for(std::chrono::milliseconds(post_execute_wait_ms_));
    wait_for_motion_start(); // 先等“数值在变化”（运动开始），超时也继续
    bool stable = wait_until_stable(); // 再等“数值不变”=到位
    if (!stable)
        LOG_NAMED_WARN("gripper", "夹爪打开判稳超时，按成功继续");
    last_open_verified_stable_ = true;
    LOG_NAMED_INFO("gripper", "夹爪打开完成（{}），宽度: {:.3f} m",
                   stable ? "反馈已稳定" : "超时视为通过", width);
    return true;
}

bool GripperController::close_to_width(double width)
{
    last_close_verified_stable_ = false;

    if (width < 0.0)
    {
        width = close_width_ - close_extra_;
        LOG_NAMED_INFO("gripper",
                       "[夹爪闭合] 使用配置参数: close_width={:.3f} m, close_extra={:.3f} m, "
                       "最终宽度={:.3f} m",
                       close_width_, close_extra_, width);
    }
    else
    {
        LOG_NAMED_INFO("gripper", "[夹爪闭合] 使用传入参数: width={:.3f} m", width);
    }

    double joint_gl = width_to_joint_angle(width);
    double joint_gr = -joint_gl; // 镜像对称
    {
        std::ofstream f(GRIPPER_DIRECT_FILE);
        if (!f)
        {
            LOG_NAMED_WARN("gripper", "[夹爪闭合] 无法写入 {}，夹爪直接目标未下发",
                           GRIPPER_DIRECT_FILE);
            return false;
        }
        f << joint_gl << " " << joint_gr << "\n";
    }
    LOG_NAMED_INFO("gripper",
                   "[夹爪闭合] 已写目标 JointGL={:.4f} rad, JointGR={:.4f} rad（0=闭合，直接 UDP）",
                   joint_gl, joint_gr);

    if (post_execute_wait_ms_ > 0)
        std::this_thread::sleep_for(std::chrono::milliseconds(post_execute_wait_ms_));
    wait_for_motion_start(); // 先等“数值在变化”（运动开始）
    if (!wait_until_stable())
    {
        LOG_NAMED_WARN("gripper", "夹爪闭合判稳超时（数值未不变=未夹到或未到位），按失败处理");
        return false;
    }
    last_close_verified_stable_ = true;
    std::vector<double> fb = get_current_joint_values();
    const double target_width = (width < 0.0) ? (close_width_ - close_extra_) : width;
    if (fb.size() >= 2u)
    {
        double fb_width = joint_angle_to_width(fb[0]);
        LOG_NAMED_INFO("gripper",
                       "夹爪闭合成功（数值不变=夹到/到位），反馈宽度 {:.4f} m，目标 {:.4f} m",
                       fb_width, target_width);
    }
    else
    {
        LOG_NAMED_INFO("gripper", "夹爪闭合成功（数值不变），目标宽度 {:.4f} m", target_width);
    }
    return true;
}

} // namespace sealien_payload_grasp
