#pragma once

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace m5_grasp
{

/**
 * @brief 参数管理器 - 集中管理所有参数的声明和加载
 */
class ParameterManager
{
  public:
    /**
     * @brief 构造函数
     * @param node ROS2节点指针
     */
    explicit ParameterManager(rclcpp::Node* node);

    /**
     * @brief 声明所有参数（带默认值）
     */
    void declare_parameters();

    /**
     * @brief 加载所有参数到成员变量
     */
    void load_parameters();

    // ========== Cable Parameters ==========
    std::string cable_name;
    double cable_diameter;
    double cable_length;
    double cable_center_offset_z;

    // ========== Grasp Parameters ==========
    double approach_offset_z;
    double descend_distance;
    double lift_distance;
    /** IK 无解时抬升用关节空间偏移 (rad)，4-DOF 开臂抬升 */
    double lift_joint_delta_J2;
    double lift_joint_delta_J3;
    bool use_cartesian;
    double eef_step;
    double jump_threshold;
    double default_planning_time;
    int num_planning_attempts;
    double max_velocity_scaling;
    double max_acceleration_scaling;
    double goal_position_tolerance;
    double goal_orientation_tolerance;
    double fallback_planning_time;
    double ik_timeout;
    double state_check_timeout;
    double state_check_interval;
    double robot_state_publisher_wait;
    double executor_startup_wait;
    double segment_execution_wait;
    double state_sync_wait;
    double main_loop_frequency;
    double yaw_offset;
    bool yaw_flip;
    double grasp_yaw_add;
    double z_clearance;
    bool yaw_validation_enabled;
    double yaw_tolerance;
    double yaw_search_step;
    bool joint4_yaw_search_enabled;
    double joint4_yaw_search_range;
    double joint4_yaw_search_step;
    double joint1_tolerance;
    double joint4_tolerance;
    bool enable_recovery;
    double recovery_timeout;
    bool yaw_candidate_search_enabled;
    double yaw_candidate_center;
    double yaw_candidate_range;
    double yaw_candidate_step;
    double ground_height;
    double ground_offset_below_base;
    double camera_error_margin;
    double min_ground_clearance;
    bool add_ground_plane;
    double max_cartesian_descend_distance;
    double tcp_offset_x;
    double tcp_offset_y;
    double tcp_offset_z;

    // ========== Gripper Parameters ==========
    double gripper_open_width;
    double gripper_close_width;
    double gripper_close_extra;
    double gripper_angle_min;
    double gripper_angle_max;
    double gripper_width_min;
    /** 打开/闭合验证：稳定性判定连续采样次数（实机反馈不变则通过） */
    int gripper_stable_samples;
    /** 稳定性判定：相邻采样变化小于此(rad)视为不变，约 1.1° */
    double gripper_stable_threshold_rad;
    /** 稳定性判定：速度判稳 (rad/s)，max(|vel|) 小于此值并持续 N 次才判到位，<=0 不检查 */
    double gripper_stable_velocity_eps_rad_s;
    /** 稳定性判定：采样间隔 (ms) */
    int gripper_stable_interval_ms;
    /** 写目标后最小等待 ms，再判“运动开始/数值不变” */
    int gripper_post_execute_wait_ms;
    /** 判定“开始运动”的最小变化量 (rad)，反馈变化超过此值认为在动 */
    double gripper_motion_start_threshold_rad;
    /** 等待“运动开始”的最大时间 (ms)，超时仍会进入判稳 */
    int gripper_motion_start_timeout_ms;
    /** 等待“数值不变”的最大时间 (ms)，闭合超时=未夹到/失败，打开超时仍成功 */
    int gripper_stable_timeout_ms;

    // ========== Vision (手眼/眼在手外) Parameters ==========
    /** sonar_link 相对 world_link 的位置 (m)，[x, y, z] */
    std::vector<double> vision_position;
    /** sonar_link 相对 world_link 的姿态 (rad)，roll, pitch, yaw */
    std::vector<double> vision_orientation_rpy;

    // ========== Scene Parameters ==========
    bool add_collision_object;
    std::vector<std::string> allow_touch_links;

    // ========== Trajectory Parameters ==========
    bool use_linear_interpolation;
    double linear_velocity;
    double linear_min_duration;
    double linear_step_scale;
    double cartesian_timeout;
    double cartesian_timeout_descend;
    double cartesian_timeout_lift;
    double cartesian_descend_threshold;
    double cartesian_lift_threshold;
    bool cartesian_retry_on_timeout;
    bool adaptive_segments;
    int min_segments;
    int max_segments;
    bool use_polynomial_interpolation;
    std::string polynomial_type;
    double polynomial_duration;
    double polynomial_dt;
    bool use_bspline;
    int bspline_degree;
    double bspline_duration;
    double bspline_dt;

    // 时间参数化参数
    bool use_time_parameterization;
    std::string time_param_algorithm;
    double time_param_velocity_scaling;
    double time_param_acceleration_scaling;

    bool use_trajectory_resampling;
    double control_frequency;
    double trajectory_min_joint_step_deg;

    /** 范围执行时间 (s)，全行程，来自 yaml trajectory.range_execution_time */
    double range_execution_time_joint1;
    double range_execution_time_joint2;
    double range_execution_time_joint3;
    double range_execution_time_joint4;
    double range_execution_time_gripper;

    /** 执行超时：余量(s) 加在 Joint1 全行程上作为最小超时；最小/最大超时(s)；估算时间的安全系数 */
    double execution_timeout_margin_s;
    double min_execution_timeout_s;
    double max_execution_timeout_s;
    double execution_timeout_safety_factor;

    /** 轨迹时间修复：最小段时长(s)、机械臂轨迹最小总时长(s) */
    double min_segment_time_s;
    double min_arm_trajectory_time_s;

    // ========== FSM Parameters ==========
    int fsm_gripper_wait_ms;
    int fsm_gripper_close_min_dwell_ms;       // 闭合后最小停留 (ms)，再允许转 LIFT
    int fsm_gripper_open_min_dwell_ms;        // 打开后最小停留 (ms)，再允许转 PREGRASP
    int fsm_arm_stable_timeout_ms;            // 机械臂稳定等待最大超时 (ms)
    double fsm_arm_stable_threshold_rad;      // 关节角度稳定阈值 (rad)
    int fsm_arm_stable_window;                // 连续满足阈值的采样次数
    double fsm_arm_stable_velocity_eps_rad_s; // 速度判稳阈值 (rad/s)，max(|vel|) 小于此值
    int fsm_arm_stable_min_dwell_ms;          // 稳定后最小停留 (ms)，再允许转 DONE

    /** 启动前 joint_states 就绪门槛：是否启用（避免全0/旧值导致先跑再回） */
    bool fsm_joint_states_ready_enabled;
    /** 至少收到的 /joint_states 次数 */
    int fsm_joint_states_ready_min_count;
    /** 最后一帧不得超过此时间 (s)，否则认为断流 */
    double fsm_joint_states_ready_max_age_s;
    /** 关节不全零判定：sum(|position|) 需大于此值 */
    double fsm_joint_states_ready_nonzero_eps;

  private:
    rclcpp::Node* node_;

    void declare_cable_parameters();
    void declare_grasp_parameters();
    void declare_gripper_parameters();
    void declare_vision_parameters();
    void declare_scene_parameters();
    void declare_trajectory_parameters();
    void declare_fsm_parameters();

    void load_cable_parameters();
    void load_grasp_parameters();
    void load_gripper_parameters();
    void load_vision_parameters();
    void load_scene_parameters();
    void load_trajectory_parameters();
    void load_fsm_parameters();

    void log_parameters();
};

} // namespace m5_grasp
