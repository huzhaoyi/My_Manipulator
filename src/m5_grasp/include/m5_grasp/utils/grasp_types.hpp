#ifndef M5_GRASP_GRASP_TYPES_HPP
#define M5_GRASP_GRASP_TYPES_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/robot_state/robot_state.h>
#include <string>
#include <vector>

namespace m5_grasp
{

/**
 * @brief 缆绳抓取任务结构体
 */
struct CableGraspTask
{
    geometry_msgs::msg::PoseStamped cable_pose;
    double original_yaw;
    double joint4_target;
};

/**
 * @brief Yaw IK求解结果
 */
struct YawIKResult
{
    bool success;                                 ///< 是否成功找到有效解
    double selected_yaw;                          ///< 选定的yaw值
    std::vector<double> joint2_3_values;          ///< Joint2/3的关节值
    std::string reason;                           ///< 失败原因
    moveit::core::RobotStatePtr successful_state; ///< 成功时的状态（避免重新计算IK）
};

/**
 * @brief 抓取参数结构体
 */
struct GraspParameters
{
    // Cable参数
    std::string cable_name{"cable_1"};
    double cable_diameter{0.008};
    double cable_length{0.20};
    std::string cable_frame_id{"world"};
    std::string cable_shape{"cylinder"};
    double cable_center_offset_z{0.0};

    // 抓取参数
    double approach_offset_z{0.10};
    double descend_distance{0.08};
    double lift_distance{0.06};
    double max_pos_error{0.01};
    bool use_cartesian{true};
    double eef_step{0.005};
    double jump_threshold{0.0};

    // 规划参数
    double planning_time{15.0};
    int num_planning_attempts{30};
    double max_velocity_scaling{0.9};
    double max_acceleration_scaling{0.9};
    double goal_position_tolerance{0.01};
    double goal_orientation_tolerance{0.5};
    double fallback_planning_time{2.0};
    double ik_timeout{0.1};

    // 时间参数
    double state_check_timeout{1.0};
    double state_check_interval{0.1};
    double robot_state_publisher_wait{2.0};
    double executor_startup_wait{0.5};
    double segment_execution_wait{0.5};
    double state_sync_wait{0.1};
    double control_frequency{10.0};
    double read_frequency{10.0};
    double main_loop_frequency{10.0};

    // Yaw参数
    double yaw_offset{0.0};
    bool yaw_flip{false};
    double grasp_yaw_add{0.0};
    double z_clearance{0.002};
    bool yaw_validation_enabled{true};
    double yaw_tolerance{0.0873};
    double yaw_search_step{0.0175};
    bool joint4_yaw_search_enabled{true};
    double joint4_yaw_search_range{M_PI};
    double joint4_yaw_search_step{0.0873};

    // Joint约束参数
    double joint1_tolerance{0.1745};
    double joint4_tolerance{0.0873};

    // 恢复参数
    bool enable_recovery{true};
    double recovery_timeout{5.0};

    // Yaw候选搜索参数
    bool yaw_candidate_search_enabled{true};
    double yaw_candidate_center{0.0};
    double yaw_candidate_range{M_PI};
    double yaw_candidate_step{0.2618};

    // 地面安全参数
    double ground_height{0.0};
    double ground_offset_below_base{-0.05};
    double camera_error_margin{0.005};
    double min_ground_clearance{0.010};
    bool add_ground_plane{false}; // 默认不添加地面障碍（支架测试时去除）
    double max_cartesian_descend_distance{0.15};

    // TCP偏移参数
    double tcp_offset_x{0.0};
    double tcp_offset_y{-0.024};
    double tcp_offset_z{-0.0086};
};

/**
 * @brief 夹爪参数结构体
 */
struct GripperParameters
{
    std::string mode{"position"};
    double open_width{0.030};
    double close_width{0.010};
    double close_extra{0.002};
    int hold_time_ms{200};
    double angle_min{1.01};
    double angle_max{-1.01};
    double width_min{0.0};
};

/**
 * @brief 放置参数结构体
 */
struct PlaceParameters
{
    double approach_offset_z{0.10};
    double descend_distance{0.08};
    double retreat_distance{0.06};
    double max_pos_error{0.01};
    double planning_time{15.0};
    int num_planning_attempts{30};
    double max_velocity_scaling{0.9};
    double max_acceleration_scaling{0.9};
    double goal_position_tolerance{0.01};
    double goal_orientation_tolerance{0.5};
};

/**
 * @brief 工作空间参数结构体
 *
 * 精确计算（与 URDF m5_updated_from_csv.urdf 一致）：
 * - link2_length + link3_length + link4_to_eef = 0.264+0.143+0.187 = 0.594 m
 * - Joint2 在 base 正上方，故从 base 起算的最大水平距离 = 0.594 m（手臂伸直时）
 * - 因此 0.6 m 比几何极限多 6 mm，0.7 m 不可达
 * - 高度：几何上 z ∈ [base_height - 0.594, base_height + 0.594]，关节限位 J2/J3 ∈ [-2.97,0] 会缩小实际范围
 * - 推荐安全范围：水平 0.08~0.59 m，高度 0.15~0.80 m；预判余量由 reach_radius_margin / 高度 margin 控制
 */
struct WorkspaceParameters
{
    double base_height{0.2145};
    double link2_length{0.264};
    double link3_length{0.143};
    double link4_to_eef{0.187};
    double reach_radius_margin{0.95};
    double max_height_offset{0.5855};  // 0.80 - base_height，对应高度上限 0.80 m
    double min_height_offset{-0.0645}; // 0.15 - base_height，对应高度下限 0.15 m
    double min_radius{0.08};
    /// 最大水平半径（m），从 base 起算。几何极限 = link2+link3+link4 = 0.594；0.6/0.7 均不可达
    double max_radius{0.594};

    // 安全区域
    double safe_x_min{0.15};
    double safe_x_max{0.40};
    double safe_y_min{-0.15};
    double safe_y_max{0.15};
    double safe_z_min{0.25};
    double safe_z_max{0.35};

    // 中等区域
    double medium_x_min{0.15};
    double medium_x_max{0.45};
    double medium_y_min{-0.20};
    double medium_y_max{0.20};
    double medium_z_min{0.20};
    double medium_z_max{0.40};
};

/**
 * @brief 轨迹参数结构体
 */
struct TrajectoryParameters
{
    bool use_linear_interpolation{false};
    double linear_velocity{0.1};
    double linear_min_duration{1.0};
    double linear_step_scale{0.5};
    double cartesian_timeout{3.0};
    double cartesian_timeout_descend{5.0};
    double cartesian_timeout_lift{3.0};
    double cartesian_descend_threshold{0.95};
    double cartesian_lift_threshold{0.90};
    bool cartesian_retry_on_timeout{true};
    bool adaptive_segments{true};
    int min_segments{2};
    int max_segments{4};
    bool use_polynomial_interpolation{false};
    std::string polynomial_type{"cubic"};
    double polynomial_duration{2.0};
    double polynomial_dt{0.01};
    bool use_bspline{false};
    int bspline_degree{3};
    double bspline_duration{3.0};
    double bspline_dt{0.01};
};

/**
 * @brief 场景参数结构体
 */
struct SceneParameters
{
    bool add_collision_object{true};
    std::vector<std::string> allow_touch_links{"LinkGG", "LinkGL", "LinkGR"};
};

} // namespace m5_grasp

#endif // M5_GRASP_GRASP_TYPES_HPP
