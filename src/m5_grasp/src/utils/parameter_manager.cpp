#include "m5_grasp/utils/parameter_manager.hpp"
#include "m5_grasp/logging/logger.hpp"

namespace m5_grasp
{

ParameterManager::ParameterManager(rclcpp::Node* node) : node_(node)
{
}

void ParameterManager::declare_parameters()
{
    declare_cable_parameters();
    declare_grasp_parameters();
    declare_gripper_parameters();
    declare_scene_parameters();
    declare_trajectory_parameters();
    declare_fsm_parameters();
}

void ParameterManager::load_parameters()
{
    load_cable_parameters();
    load_grasp_parameters();
    load_gripper_parameters();
    load_scene_parameters();
    load_trajectory_parameters();
    load_fsm_parameters();
    log_parameters();
}

// ========== Cable Parameters ==========
void ParameterManager::declare_cable_parameters()
{
    node_->declare_parameter("cable.name", "cable_1");
    node_->declare_parameter("cable.diameter", 0.008);
    node_->declare_parameter("cable.length", 0.20);
    node_->declare_parameter("cable.center_offset_z", 0.0);
}

void ParameterManager::load_cable_parameters()
{
    cable_name = node_->get_parameter("cable.name").as_string();
    cable_diameter = node_->get_parameter("cable.diameter").as_double();
    cable_length = node_->get_parameter("cable.length").as_double();
    cable_center_offset_z = node_->get_parameter("cable.center_offset_z").as_double();
}

// ========== Grasp Parameters ==========
void ParameterManager::declare_grasp_parameters()
{
    node_->declare_parameter("grasp.approach_offset_z", 0.10);
    node_->declare_parameter("grasp.descend_distance", 0.08);
    node_->declare_parameter("grasp.lift_distance", 0.06);
    node_->declare_parameter("grasp.lift_joint_delta_J2", 0.10);
    node_->declare_parameter("grasp.lift_joint_delta_J3", 0.10);
    node_->declare_parameter("grasp.use_cartesian", true);
    node_->declare_parameter("grasp.eef_step", 0.005);
    node_->declare_parameter("grasp.jump_threshold", 0.0);
    node_->declare_parameter("grasp.planning_time", 15.0);
    node_->declare_parameter("grasp.num_planning_attempts", 30);
    node_->declare_parameter("grasp.max_velocity_scaling", 0.9);
    node_->declare_parameter("grasp.max_acceleration_scaling", 0.9);
    node_->declare_parameter("grasp.goal_position_tolerance", 0.01);
    node_->declare_parameter("grasp.goal_orientation_tolerance", 0.5);
    node_->declare_parameter("grasp.fallback_planning_time", 2.0);
    node_->declare_parameter("grasp.ik_timeout", 0.1);
    node_->declare_parameter("grasp.state_check_timeout", 1.0);
    node_->declare_parameter("grasp.state_check_interval", 0.1);
    node_->declare_parameter("grasp.robot_state_publisher_wait", 2.0);
    node_->declare_parameter("grasp.executor_startup_wait", 0.5);
    node_->declare_parameter("grasp.segment_execution_wait", 0.1);
    node_->declare_parameter("grasp.state_sync_wait", 0.1);
    node_->declare_parameter("grasp.main_loop_frequency", 10.0);
    node_->declare_parameter("grasp.yaw_offset", 0.0);
    node_->declare_parameter("grasp.yaw_flip", false);
    node_->declare_parameter("grasp.grasp_yaw_add", 0.0);
    node_->declare_parameter("grasp.z_clearance", 0.002);
    node_->declare_parameter("grasp.yaw_validation_enabled", true);
    node_->declare_parameter("grasp.yaw_tolerance", 0.0873);
    node_->declare_parameter("grasp.yaw_search_step", 0.0175);
    node_->declare_parameter("grasp.joint4_yaw_search_enabled", true);
    node_->declare_parameter("grasp.joint4_yaw_search_range", M_PI);
    node_->declare_parameter("grasp.joint4_yaw_search_step", 0.0873);
    node_->declare_parameter("grasp.joint1_tolerance", 0.1745);
    node_->declare_parameter("grasp.joint4_tolerance", 0.0873);
    node_->declare_parameter("grasp.enable_recovery", true);
    node_->declare_parameter("grasp.recovery_timeout", 5.0);
    node_->declare_parameter("grasp.yaw_candidate_search_enabled", true);
    node_->declare_parameter("grasp.yaw_candidate_center", 0.0);
    node_->declare_parameter("grasp.yaw_candidate_range", M_PI);
    node_->declare_parameter("grasp.yaw_candidate_step", 0.2618);
    node_->declare_parameter("grasp.ground_height", 0.0);
    node_->declare_parameter("grasp.ground_offset_below_base", -0.05);
    node_->declare_parameter("grasp.camera_error_margin", 0.005);
    node_->declare_parameter("grasp.min_ground_clearance", 0.010);
    node_->declare_parameter("grasp.add_ground_plane",
                             false); // 默认不添加地面（支架测试、视觉可发负z）
    node_->declare_parameter("grasp.max_cartesian_descend_distance", 0.15);
    node_->declare_parameter("grasp.tcp_offset_x", 0.0);
    node_->declare_parameter("grasp.tcp_offset_y", -0.024);
    node_->declare_parameter("grasp.tcp_offset_z", -0.0086);
}

void ParameterManager::load_grasp_parameters()
{
    approach_offset_z = node_->get_parameter("grasp.approach_offset_z").as_double();
    descend_distance = node_->get_parameter("grasp.descend_distance").as_double();
    lift_distance = node_->get_parameter("grasp.lift_distance").as_double();
    lift_joint_delta_J2 = node_->get_parameter("grasp.lift_joint_delta_J2").as_double();
    lift_joint_delta_J3 = node_->get_parameter("grasp.lift_joint_delta_J3").as_double();
    use_cartesian = node_->get_parameter("grasp.use_cartesian").as_bool();
    eef_step = node_->get_parameter("grasp.eef_step").as_double();
    jump_threshold = node_->get_parameter("grasp.jump_threshold").as_double();
    default_planning_time = node_->get_parameter("grasp.planning_time").as_double();
    num_planning_attempts = node_->get_parameter("grasp.num_planning_attempts").as_int();
    max_velocity_scaling = node_->get_parameter("grasp.max_velocity_scaling").as_double();
    max_acceleration_scaling = node_->get_parameter("grasp.max_acceleration_scaling").as_double();
    goal_position_tolerance = node_->get_parameter("grasp.goal_position_tolerance").as_double();
    goal_orientation_tolerance =
        node_->get_parameter("grasp.goal_orientation_tolerance").as_double();
    fallback_planning_time = node_->get_parameter("grasp.fallback_planning_time").as_double();
    ik_timeout = node_->get_parameter("grasp.ik_timeout").as_double();
    state_check_timeout = node_->get_parameter("grasp.state_check_timeout").as_double();
    state_check_interval = node_->get_parameter("grasp.state_check_interval").as_double();
    robot_state_publisher_wait =
        node_->get_parameter("grasp.robot_state_publisher_wait").as_double();
    executor_startup_wait = node_->get_parameter("grasp.executor_startup_wait").as_double();
    segment_execution_wait = node_->get_parameter("grasp.segment_execution_wait").as_double();
    state_sync_wait = node_->get_parameter("grasp.state_sync_wait").as_double();
    main_loop_frequency = node_->get_parameter("grasp.main_loop_frequency").as_double();
    yaw_offset = node_->get_parameter("grasp.yaw_offset").as_double();
    yaw_flip = node_->get_parameter("grasp.yaw_flip").as_bool();
    grasp_yaw_add = node_->get_parameter("grasp.grasp_yaw_add").as_double();
    z_clearance = node_->get_parameter("grasp.z_clearance").as_double();
    yaw_validation_enabled = node_->get_parameter("grasp.yaw_validation_enabled").as_bool();
    yaw_tolerance = node_->get_parameter("grasp.yaw_tolerance").as_double();
    yaw_search_step = node_->get_parameter("grasp.yaw_search_step").as_double();
    joint4_yaw_search_enabled = node_->get_parameter("grasp.joint4_yaw_search_enabled").as_bool();
    joint4_yaw_search_range = node_->get_parameter("grasp.joint4_yaw_search_range").as_double();
    joint4_yaw_search_step = node_->get_parameter("grasp.joint4_yaw_search_step").as_double();
    joint1_tolerance = node_->get_parameter("grasp.joint1_tolerance").as_double();
    joint4_tolerance = node_->get_parameter("grasp.joint4_tolerance").as_double();
    enable_recovery = node_->get_parameter("grasp.enable_recovery").as_bool();
    recovery_timeout = node_->get_parameter("grasp.recovery_timeout").as_double();
    yaw_candidate_search_enabled =
        node_->get_parameter("grasp.yaw_candidate_search_enabled").as_bool();
    yaw_candidate_center = node_->get_parameter("grasp.yaw_candidate_center").as_double();
    yaw_candidate_range = node_->get_parameter("grasp.yaw_candidate_range").as_double();
    yaw_candidate_step = node_->get_parameter("grasp.yaw_candidate_step").as_double();
    ground_height = node_->get_parameter("grasp.ground_height").as_double();
    ground_offset_below_base = node_->get_parameter("grasp.ground_offset_below_base").as_double();
    camera_error_margin = node_->get_parameter("grasp.camera_error_margin").as_double();
    min_ground_clearance = node_->get_parameter("grasp.min_ground_clearance").as_double();
    add_ground_plane = node_->get_parameter("grasp.add_ground_plane").as_bool();
    max_cartesian_descend_distance =
        node_->get_parameter("grasp.max_cartesian_descend_distance").as_double();
    tcp_offset_x = node_->get_parameter("grasp.tcp_offset_x").as_double();
    tcp_offset_y = node_->get_parameter("grasp.tcp_offset_y").as_double();
    tcp_offset_z = node_->get_parameter("grasp.tcp_offset_z").as_double();
}

// ========== Gripper Parameters ==========
void ParameterManager::declare_gripper_parameters()
{
    node_->declare_parameter("gripper.open_width", 0.030);
    node_->declare_parameter("gripper.close_width", 0.010);
    node_->declare_parameter("gripper.close_extra", 0.0);
    node_->declare_parameter("gripper.angle_min", 1.01); // rad，闭合（axis5=0）
    node_->declare_parameter("gripper.angle_max", -1.01); // rad，张开（axis5=-1100，厂家实际）
    node_->declare_parameter("gripper.width_min", 0.0);
    node_->declare_parameter("gripper.stable_samples", 3); // 打开/闭合验证：稳定性采样次数
    node_->declare_parameter("gripper.stable_threshold_rad",
                             0.02); // 相邻采样变化小于此视为不变 (~1.1°)
    node_->declare_parameter("gripper.stable_velocity_eps_rad_s",
                             0.01); // 速度判稳 (rad/s)，<=0 不检查
    node_->declare_parameter("gripper.stable_interval_ms", 50); // 采样间隔 ms
    node_->declare_parameter("gripper.post_execute_wait_ms",
                             1000); // 写目标后等待 ms，再判运动开始/数值不变
    node_->declare_parameter("gripper.motion_start_threshold_rad",
                             0.008); // 判定“开始运动”的最小变化 (rad)，约 0.46°
    node_->declare_parameter("gripper.motion_start_timeout_ms",
                             3000); // 等待运动开始最大 ms，超时仍进入判稳
    node_->declare_parameter("gripper.stable_timeout_ms",
                             8000); // 等待数值不变最大 ms，闭合超时=失败
}

void ParameterManager::load_gripper_parameters()
{
    gripper_open_width = node_->get_parameter("gripper.open_width").as_double();
    gripper_close_width = node_->get_parameter("gripper.close_width").as_double();
    gripper_close_extra = node_->get_parameter("gripper.close_extra").as_double();
    gripper_angle_min = node_->get_parameter("gripper.angle_min").as_double();
    gripper_angle_max = node_->get_parameter("gripper.angle_max").as_double();
    gripper_width_min = node_->get_parameter("gripper.width_min").as_double();
    gripper_stable_samples = node_->get_parameter("gripper.stable_samples").as_int();
    gripper_stable_threshold_rad = node_->get_parameter("gripper.stable_threshold_rad").as_double();
    gripper_stable_velocity_eps_rad_s =
        node_->get_parameter("gripper.stable_velocity_eps_rad_s").as_double();
    gripper_stable_interval_ms = node_->get_parameter("gripper.stable_interval_ms").as_int();
    gripper_post_execute_wait_ms = node_->get_parameter("gripper.post_execute_wait_ms").as_int();
    gripper_motion_start_threshold_rad =
        node_->get_parameter("gripper.motion_start_threshold_rad").as_double();
    gripper_motion_start_timeout_ms =
        node_->get_parameter("gripper.motion_start_timeout_ms").as_int();
    gripper_stable_timeout_ms = node_->get_parameter("gripper.stable_timeout_ms").as_int();
    LOG_NAMED_INFO("params",
                   "夹爪参数: angle_min={:.3f} rad, angle_max={:.3f} rad, stable_samples={}, "
                   "stable_threshold_rad={:.3f}, motion_start_threshold={:.3f}, "
                   "motion_start_timeout_ms={}, stable_timeout_ms={}",
                   gripper_angle_min, gripper_angle_max, gripper_stable_samples,
                   gripper_stable_threshold_rad, gripper_motion_start_threshold_rad,
                   gripper_motion_start_timeout_ms, gripper_stable_timeout_ms);
}

// ========== Scene Parameters ==========
void ParameterManager::declare_scene_parameters()
{
    node_->declare_parameter("scene.add_collision_object", true);
    node_->declare_parameter("scene.allow_touch_links",
                             std::vector<std::string>{"LinkGG", "LinkGL", "LinkGR"});
}

void ParameterManager::load_scene_parameters()
{
    add_collision_object = node_->get_parameter("scene.add_collision_object").as_bool();
    allow_touch_links = node_->get_parameter("scene.allow_touch_links").as_string_array();
}

// ========== Trajectory Parameters ==========
void ParameterManager::declare_trajectory_parameters()
{
    node_->declare_parameter("trajectory.use_linear_interpolation", false);
    node_->declare_parameter("trajectory.linear_velocity", 0.1);
    node_->declare_parameter("trajectory.linear_min_duration", 1.0);
    node_->declare_parameter("trajectory.linear_step_scale", 0.5);
    node_->declare_parameter("trajectory.cartesian_timeout", 3.0);
    node_->declare_parameter("trajectory.cartesian_timeout_descend", 5.0);
    node_->declare_parameter("trajectory.cartesian_timeout_lift", 3.0);
    node_->declare_parameter("trajectory.cartesian_descend_threshold", 0.95);
    node_->declare_parameter("trajectory.cartesian_lift_threshold", 0.90);
    node_->declare_parameter("trajectory.cartesian_retry_on_timeout", true);
    node_->declare_parameter("trajectory.adaptive_segments", true);
    node_->declare_parameter("trajectory.min_segments", 2);
    node_->declare_parameter("trajectory.max_segments", 4);
    node_->declare_parameter("trajectory.use_polynomial_interpolation", false);
    node_->declare_parameter("trajectory.polynomial_type", "cubic");
    node_->declare_parameter("trajectory.polynomial_duration", 2.0);
    node_->declare_parameter("trajectory.polynomial_dt", 0.01);
    node_->declare_parameter("trajectory.use_bspline", false);
    node_->declare_parameter("trajectory.bspline_degree", 3);
    node_->declare_parameter("trajectory.bspline_duration", 3.0);
    node_->declare_parameter("trajectory.bspline_dt", 0.01);

    // 时间参数化参数
    node_->declare_parameter("trajectory.use_time_parameterization", true);
    node_->declare_parameter("trajectory.time_param_algorithm", "iterative_spline");
    node_->declare_parameter("trajectory.time_param_velocity_scaling", 0.5);
    node_->declare_parameter("trajectory.time_param_acceleration_scaling", 0.3);

    // 轨迹重采样与步长（厂家建议：步长约10度，点要散一点）
    node_->declare_parameter("trajectory.use_trajectory_resampling", true);
    node_->declare_parameter("trajectory.control_frequency", 10.0);
    node_->declare_parameter("trajectory.min_joint_step_deg", 5.0);

    // 范围执行时间（全行程，用于超时/速度估算，与 yaml trajectory.range_execution_time 一致）
    node_->declare_parameter("trajectory.range_execution_time.joint1", 30.0);
    node_->declare_parameter("trajectory.range_execution_time.joint2", 13.0);
    node_->declare_parameter("trajectory.range_execution_time.joint3", 13.0);
    node_->declare_parameter("trajectory.range_execution_time.joint4", 21.0);
    node_->declare_parameter("trajectory.range_execution_time.gripper", 7.5);

    node_->declare_parameter("trajectory.execution_timeout_margin_s", 5.0);
    node_->declare_parameter("trajectory.min_execution_timeout_s", 35.0);
    node_->declare_parameter("trajectory.max_execution_timeout_s", 300.0);
    node_->declare_parameter("trajectory.execution_timeout_safety_factor", 1.5);
    node_->declare_parameter("trajectory.min_segment_time_s", 0.05);
    node_->declare_parameter("trajectory.min_arm_trajectory_time_s", 1.0);
}

void ParameterManager::load_trajectory_parameters()
{
    use_linear_interpolation =
        node_->get_parameter("trajectory.use_linear_interpolation").as_bool();
    linear_velocity = node_->get_parameter("trajectory.linear_velocity").as_double();
    linear_min_duration = node_->get_parameter("trajectory.linear_min_duration").as_double();
    linear_step_scale = node_->get_parameter("trajectory.linear_step_scale").as_double();
    cartesian_timeout = node_->get_parameter("trajectory.cartesian_timeout").as_double();
    cartesian_timeout_descend =
        node_->get_parameter("trajectory.cartesian_timeout_descend").as_double();
    cartesian_timeout_lift = node_->get_parameter("trajectory.cartesian_timeout_lift").as_double();
    cartesian_descend_threshold =
        node_->get_parameter("trajectory.cartesian_descend_threshold").as_double();
    cartesian_lift_threshold =
        node_->get_parameter("trajectory.cartesian_lift_threshold").as_double();
    cartesian_retry_on_timeout =
        node_->get_parameter("trajectory.cartesian_retry_on_timeout").as_bool();
    adaptive_segments = node_->get_parameter("trajectory.adaptive_segments").as_bool();
    min_segments = node_->get_parameter("trajectory.min_segments").as_int();
    max_segments = node_->get_parameter("trajectory.max_segments").as_int();
    use_polynomial_interpolation =
        node_->get_parameter("trajectory.use_polynomial_interpolation").as_bool();
    polynomial_type = node_->get_parameter("trajectory.polynomial_type").as_string();
    polynomial_duration = node_->get_parameter("trajectory.polynomial_duration").as_double();
    polynomial_dt = node_->get_parameter("trajectory.polynomial_dt").as_double();
    use_bspline = node_->get_parameter("trajectory.use_bspline").as_bool();
    bspline_degree = node_->get_parameter("trajectory.bspline_degree").as_int();
    bspline_duration = node_->get_parameter("trajectory.bspline_duration").as_double();
    bspline_dt = node_->get_parameter("trajectory.bspline_dt").as_double();

    // 时间参数化参数
    use_time_parameterization =
        node_->get_parameter("trajectory.use_time_parameterization").as_bool();
    time_param_algorithm = node_->get_parameter("trajectory.time_param_algorithm").as_string();
    time_param_velocity_scaling =
        node_->get_parameter("trajectory.time_param_velocity_scaling").as_double();
    time_param_acceleration_scaling =
        node_->get_parameter("trajectory.time_param_acceleration_scaling").as_double();

    // 轨迹重采样与步长
    use_trajectory_resampling =
        node_->get_parameter("trajectory.use_trajectory_resampling").as_bool();
    control_frequency = node_->get_parameter("trajectory.control_frequency").as_double();
    trajectory_min_joint_step_deg =
        node_->get_parameter("trajectory.min_joint_step_deg").as_double();
    range_execution_time_joint1 =
        node_->get_parameter("trajectory.range_execution_time.joint1").as_double();
    range_execution_time_joint2 =
        node_->get_parameter("trajectory.range_execution_time.joint2").as_double();
    range_execution_time_joint3 =
        node_->get_parameter("trajectory.range_execution_time.joint3").as_double();
    range_execution_time_joint4 =
        node_->get_parameter("trajectory.range_execution_time.joint4").as_double();
    range_execution_time_gripper =
        node_->get_parameter("trajectory.range_execution_time.gripper").as_double();
    execution_timeout_margin_s =
        node_->get_parameter("trajectory.execution_timeout_margin_s").as_double();
    min_execution_timeout_s =
        node_->get_parameter("trajectory.min_execution_timeout_s").as_double();
    max_execution_timeout_s =
        node_->get_parameter("trajectory.max_execution_timeout_s").as_double();
    execution_timeout_safety_factor =
        node_->get_parameter("trajectory.execution_timeout_safety_factor").as_double();
    min_segment_time_s = node_->get_parameter("trajectory.min_segment_time_s").as_double();
    min_arm_trajectory_time_s =
        node_->get_parameter("trajectory.min_arm_trajectory_time_s").as_double();
}

// ========== FSM Parameters ==========
void ParameterManager::declare_fsm_parameters()
{
    node_->declare_parameter("fsm.gripper_wait_ms", 8000);
    node_->declare_parameter("fsm.gripper_close_min_dwell_ms",
                             2000); // 闭合后最小停留 (ms)，2~4s 更保守
    node_->declare_parameter("fsm.gripper_open_min_dwell_ms",
                             1500); // 打开后最小停留 (ms)，再允许转 PREGRASP
    node_->declare_parameter("fsm.arm_stable_timeout_ms", 15000); // 机械臂稳定等待最大超时 (ms)
    node_->declare_parameter("fsm.arm_stable_threshold_rad",
                             0.004); // 关节角度稳定阈值 (rad，约 0.23°)
    node_->declare_parameter("fsm.arm_stable_window",
                             8); // 连续满足 pos+vel 的采样次数（8~10 更易达稳）
    node_->declare_parameter("fsm.arm_stable_velocity_eps_rad_s", 0.012); // 速度判稳阈值 (rad/s)
    node_->declare_parameter("fsm.arm_stable_min_dwell_ms", 800); // 稳定后最小停留 (ms)
    node_->declare_parameter("fsm.joint_states_ready_enabled", true);
    node_->declare_parameter("fsm.joint_states_ready_min_count", 10);
    node_->declare_parameter("fsm.joint_states_ready_max_age_s", 0.3);
    node_->declare_parameter("fsm.joint_states_ready_nonzero_eps", 0.01);
}

void ParameterManager::load_fsm_parameters()
{
    fsm_gripper_wait_ms = node_->get_parameter("fsm.gripper_wait_ms").as_int();
    fsm_gripper_close_min_dwell_ms =
        node_->get_parameter("fsm.gripper_close_min_dwell_ms").as_int();
    fsm_gripper_open_min_dwell_ms = node_->get_parameter("fsm.gripper_open_min_dwell_ms").as_int();
    fsm_arm_stable_timeout_ms = node_->get_parameter("fsm.arm_stable_timeout_ms").as_int();
    fsm_arm_stable_threshold_rad = node_->get_parameter("fsm.arm_stable_threshold_rad").as_double();
    fsm_arm_stable_window = node_->get_parameter("fsm.arm_stable_window").as_int();
    fsm_arm_stable_velocity_eps_rad_s =
        node_->get_parameter("fsm.arm_stable_velocity_eps_rad_s").as_double();
    fsm_arm_stable_min_dwell_ms = node_->get_parameter("fsm.arm_stable_min_dwell_ms").as_int();
    fsm_joint_states_ready_enabled =
        node_->get_parameter("fsm.joint_states_ready_enabled").as_bool();
    fsm_joint_states_ready_min_count =
        node_->get_parameter("fsm.joint_states_ready_min_count").as_int();
    fsm_joint_states_ready_max_age_s =
        node_->get_parameter("fsm.joint_states_ready_max_age_s").as_double();
    fsm_joint_states_ready_nonzero_eps =
        node_->get_parameter("fsm.joint_states_ready_nonzero_eps").as_double();
}

void ParameterManager::log_parameters()
{
    LOG_NAMED_INFO("params", "参数加载完成");
    LOG_NAMED_INFO("params", "缆绳直径: {:.3f} m, 长度: {:.3f} m", cable_diameter, cable_length);
    LOG_NAMED_INFO("params", "夹爪打开宽度: {:.3f} m, 闭合宽度: {:.3f} m", gripper_open_width,
                   gripper_close_width);

    LOG_NAMED_INFO("params", "配置参数:");
    LOG_NAMED_INFO("params", "  grasp_yaw_add: {:.3f} rad ({:.1f} deg)", grasp_yaw_add,
                   grasp_yaw_add * 180.0 / M_PI);
    LOG_NAMED_INFO("params", "TCP偏移补偿参数: ({:.4f}, {:.4f}, {:.4f}) m", tcp_offset_x,
                   tcp_offset_y, tcp_offset_z);

    LOG_NAMED_INFO("params", "=== 轨迹规划配置 ===");
    LOG_NAMED_INFO("params", "线性插补: {} (速度: {:.3f} m/s)",
                   use_linear_interpolation ? "启用" : "禁用", linear_velocity);
    LOG_NAMED_INFO("params", "笛卡尔路径超时: {:.1f} s", cartesian_timeout);
    LOG_NAMED_INFO("params", "range_execution_time: J1={:.1f}s J2={:.1f}s J3={:.1f}s J4={:.1f}s "
                   "gripper={:.1f}s",
                   range_execution_time_joint1, range_execution_time_joint2,
                   range_execution_time_joint3, range_execution_time_joint4,
                   range_execution_time_gripper);
    LOG_NAMED_INFO("params", "execution_timeout: margin={:.1f}s min={:.1f}s max={:.1f}s factor={:.1f}",
                   execution_timeout_margin_s, min_execution_timeout_s, max_execution_timeout_s,
                   execution_timeout_safety_factor);
    LOG_NAMED_INFO("params", "===================");

    LOG_NAMED_INFO("params", "=== FSM配置 ===");
    LOG_NAMED_INFO("params",
                   "夹爪等待: {}ms, 闭合min_dwell: {}ms, 打开min_dwell: {}ms, "
                   "臂稳定: 超时{}ms/阈值{:.4f}rad/窗{}/vel_eps{:.3f}rad_s/min_dwell{}ms",
                   fsm_gripper_wait_ms, fsm_gripper_close_min_dwell_ms,
                   fsm_gripper_open_min_dwell_ms, fsm_arm_stable_timeout_ms,
                   fsm_arm_stable_threshold_rad, fsm_arm_stable_window,
                   fsm_arm_stable_velocity_eps_rad_s, fsm_arm_stable_min_dwell_ms);
    LOG_NAMED_INFO("params",
                   "joint_states就绪门槛: enabled={}, min_count={}, max_age_s={:.2f}, nonzero_eps={:.3f}",
                   fsm_joint_states_ready_enabled, fsm_joint_states_ready_min_count,
                   fsm_joint_states_ready_max_age_s, fsm_joint_states_ready_nonzero_eps);
    LOG_NAMED_INFO("params", "===================");

    LOG_NAMED_INFO("params", "=== 频率配置 ===");
    LOG_NAMED_INFO("params", "主循环频率: {:.1f} Hz", main_loop_frequency);
    LOG_NAMED_INFO("params", "===================");
}

} // namespace m5_grasp
