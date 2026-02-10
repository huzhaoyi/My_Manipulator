#ifndef M5_GRASP_GRIPPER_CONTROLLER_HPP
#define M5_GRASP_GRIPPER_CONTROLLER_HPP

#include <chrono>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "sealien_payload_grasp/hw/gripper_interface.hpp"

namespace sealien_payload_grasp
{
struct JointStateDiag; // 定义于 mtc/task_context.hpp，用于轨迹起点后备

/**
 * @brief 夹爪控制器类（直接 UDP，无轨迹）
 *
 * 打开/闭合：写目标 (JointGL, JointGR) 到 /tmp/gripper_direct.txt，由 m5_hardware 通信线程合并后发
 * UDP。 稳定判据：运动开始后反馈数值不再变化（连续 N 次采样变化 < 阈值）。 实现 IGripper 接口，供
 * FSM 使用。
 */
class GripperController : public IGripper
{
  public:
    /**
     * @brief 构造函数
     * @param node ROS2节点指针
     * @param gripper_interface MoveIt夹爪接口
     * @param moveit_mutex MoveIt互斥锁
     */
    GripperController(
        rclcpp::Node* node,
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_interface,
        std::mutex& moveit_mutex);

    /**
     * @brief 设置夹爪参数
     */
    void set_parameters(double open_width, double close_width, double close_extra, int hold_time_ms,
                       double angle_min, double angle_max, double width_min, int stable_samples = 3,
                       double stable_threshold_rad = 0.02, double stable_velocity_eps_rad_s = 0.01,
                       int stable_interval_ms = 50, int post_execute_wait_ms = 1000,
                       double motion_start_threshold_rad = 0.008,
                       int motion_start_timeout_ms = 3000, int stable_timeout_ms = 8000);

    /**
     * @brief 打开夹爪（IGripper接口实现）
     * @return 是否成功
     */
    bool open() override;

    /**
     * @brief 闭合夹爪（IGripper接口实现）
     * @return 是否成功
     */
    bool close() override;

    /**
     * @brief 打开夹爪到指定宽度
     * @param width 目标宽度（米），-1表示使用配置的默认打开宽度
     * @return 是否成功
     */
    bool open_to_width(double width = -1.0);

    /**
     * @brief 闭合夹爪到指定宽度
     * @param width 目标宽度（米），-1表示使用配置的默认闭合宽度
     * @return 是否成功
     */
    bool close_to_width(double width = -1.0);

    /**
     * @brief 将宽度转换为关节角度
     * @param width 夹爪宽度（米）
     * @return 对应的关节角度（弧度）
     */
    double width_to_joint_angle(double width);

    /**
     * @brief 由关节角（JointGL）反算夹爪宽度，用于日志区分“反馈宽度”与“目标宽度”
     */
    double joint_angle_to_width(double joint_gl) const;

    /**
     * @brief 获取当前夹爪关节值
     * @return 夹爪关节值向量
     */
    std::vector<double> get_current_joint_values();

    /**
     * @brief 设置 /joint_states 诊断缓存，当 get_current_joint_values 不可用时用其作为轨迹起点后备
     */
    void set_joint_state_diag(std::shared_ptr<JointStateDiag> diag)
    {
        joint_state_diag_ = std::move(diag);
    }

    /**
     * @brief 获取打开宽度
     */
    double get_open_width() const
    {
        return open_width_;
    }

    /**
     * @brief 获取闭合宽度
     */
    double get_close_width() const
    {
        return close_width_;
    }

    /**
     * @brief 检查夹爪是否已到达打开目标（供 FSM WAIT_GRIPPER 每 tick 判断，仅依据稳定性验证标志）
     */
    bool is_open_to_target() override;

    /**
     * @brief 检查夹爪是否已到达闭合目标（供 FSM WAIT_GRIPPER 每 tick 判断，仅依据稳定性验证标志）
     */
    bool is_closed_to_target() override;

  private:
    rclcpp::Node* node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_interface_;
    std::mutex& moveit_mutex_;
    std::shared_ptr<JointStateDiag> joint_state_diag_{nullptr};

    // 参数
    double open_width_{0.030};
    double close_width_{0.010};
    double close_extra_{0.002};
    int hold_time_ms_{200};
    double angle_min_{1.01};  // 闭合角度
    double angle_max_{-1.01}; // 张开角度
    double width_min_{0.0};
    int stable_samples_{3};
    double stable_threshold_rad_{0.02};
    double stable_velocity_eps_rad_s_{0.01};
    int stable_interval_ms_{50};
    /** 写夹爪直接目标后最小等待 ms，再判“运动开始/数值不变” */
    int post_execute_wait_ms_{1000};
    /** 判定“开始运动”的最小变化量 (rad)，反馈变化超过此值认为在动 */
    double motion_start_threshold_rad_{0.008};
    /** 等待“运动开始”的最大时间 (ms)，超时仍会进入判稳（如已在目标） */
    int motion_start_timeout_ms_{3000};
    /** 等待“数值不变”的最大时间 (ms)，超时未稳则失败（闭合）/ 仍成功（打开） */
    int stable_timeout_ms_{8000};
    /** 闭合通过稳定性验证后置 true，供 is_closed_to_target() 持续返回 true，仅在新发 open/close 时清除
     */
    mutable bool last_close_verified_stable_{false};
    /** 打开通过稳定性验证后置 true，供 is_open_to_target() 持续返回 true，仅在新发 open/close 时清除
     */
    mutable bool last_open_verified_stable_{false};

    /** 采样实机反馈若干次，若相邻采样变化均小于阈值则返回 true（反馈不再变化 = 到位/夹到） */
    bool is_feedback_stable();
    /** 等待“数值在变化”（运动开始），超时返回 false */
    bool wait_for_motion_start();
    /** 等待“数值不变”（稳定），超时返回 false */
    bool wait_until_stable();
};

} // namespace sealien_payload_grasp

#endif // M5_GRASP_GRIPPER_CONTROLLER_HPP
