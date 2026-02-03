#include "m5_grasp/logging/logger.hpp"
#include "m5_grasp/utils/parameter_manager.hpp"

namespace m5_grasp {

ParameterManager::ParameterManager(rclcpp::Node* node) : node_(node) {}

void ParameterManager::declareParameters() {
  declareCableParameters();
  declareGraspParameters();
  declareGripperParameters();
  declareSceneParameters();
  declareWorkspaceParameters();
  declareToleranceParameters();
  declarePlaceParameters();
  declareTrajectoryParameters();
  declareFSMParameters();
}

void ParameterManager::loadParameters() {
  loadCableParameters();
  loadGraspParameters();
  loadGripperParameters();
  loadSceneParameters();
  loadWorkspaceParameters();
  loadToleranceParameters();
  loadPlaceParameters();
  loadTrajectoryParameters();
  loadFSMParameters();
  logParameters();
}

// ========== Cable Parameters ==========
void ParameterManager::declareCableParameters() {
  node_->declare_parameter("cable.name", "cable_1");
  node_->declare_parameter("cable.diameter", 0.008);
  node_->declare_parameter("cable.length", 0.20);
  node_->declare_parameter("cable.center_offset_z", 0.0);
}

void ParameterManager::loadCableParameters() {
  cable_name = node_->get_parameter("cable.name").as_string();
  cable_diameter = node_->get_parameter("cable.diameter").as_double();
  cable_length = node_->get_parameter("cable.length").as_double();
  cable_center_offset_z = node_->get_parameter("cable.center_offset_z").as_double();
}

// ========== Grasp Parameters ==========
void ParameterManager::declareGraspParameters() {
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
  node_->declare_parameter("grasp.add_ground_plane", true);
  node_->declare_parameter("grasp.max_cartesian_descend_distance", 0.15);
  node_->declare_parameter("grasp.tcp_offset_x", 0.0);
  node_->declare_parameter("grasp.tcp_offset_y", -0.024);
  node_->declare_parameter("grasp.tcp_offset_z", -0.0086);
  node_->declare_parameter("grasp.auto_move_to_ready", true);
  node_->declare_parameter("grasp.ready_joint1", 0.0);
  node_->declare_parameter("grasp.ready_joint2", -0.5);
  node_->declare_parameter("grasp.ready_joint3", -0.5);
  node_->declare_parameter("grasp.ready_joint4", 0.0);
  node_->declare_parameter("grasp.near_zero_threshold", 0.1);
}

void ParameterManager::loadGraspParameters() {
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
  goal_orientation_tolerance = node_->get_parameter("grasp.goal_orientation_tolerance").as_double();
  fallback_planning_time = node_->get_parameter("grasp.fallback_planning_time").as_double();
  ik_timeout = node_->get_parameter("grasp.ik_timeout").as_double();
  state_check_timeout = node_->get_parameter("grasp.state_check_timeout").as_double();
  state_check_interval = node_->get_parameter("grasp.state_check_interval").as_double();
  robot_state_publisher_wait = node_->get_parameter("grasp.robot_state_publisher_wait").as_double();
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
  yaw_candidate_search_enabled = node_->get_parameter("grasp.yaw_candidate_search_enabled").as_bool();
  yaw_candidate_center = node_->get_parameter("grasp.yaw_candidate_center").as_double();
  yaw_candidate_range = node_->get_parameter("grasp.yaw_candidate_range").as_double();
  yaw_candidate_step = node_->get_parameter("grasp.yaw_candidate_step").as_double();
  ground_height = node_->get_parameter("grasp.ground_height").as_double();
  ground_offset_below_base = node_->get_parameter("grasp.ground_offset_below_base").as_double();
  camera_error_margin = node_->get_parameter("grasp.camera_error_margin").as_double();
  min_ground_clearance = node_->get_parameter("grasp.min_ground_clearance").as_double();
  add_ground_plane = node_->get_parameter("grasp.add_ground_plane").as_bool();
  max_cartesian_descend_distance = node_->get_parameter("grasp.max_cartesian_descend_distance").as_double();
  tcp_offset_x = node_->get_parameter("grasp.tcp_offset_x").as_double();
  tcp_offset_y = node_->get_parameter("grasp.tcp_offset_y").as_double();
  tcp_offset_z = node_->get_parameter("grasp.tcp_offset_z").as_double();
  auto_move_to_ready = node_->get_parameter("grasp.auto_move_to_ready").as_bool();
  ready_joint1 = node_->get_parameter("grasp.ready_joint1").as_double();
  ready_joint2 = node_->get_parameter("grasp.ready_joint2").as_double();
  ready_joint3 = node_->get_parameter("grasp.ready_joint3").as_double();
  ready_joint4 = node_->get_parameter("grasp.ready_joint4").as_double();
  near_zero_threshold = node_->get_parameter("grasp.near_zero_threshold").as_double();
}

// ========== Gripper Parameters ==========
void ParameterManager::declareGripperParameters() {
  node_->declare_parameter("gripper.open_width", 0.030);
  node_->declare_parameter("gripper.close_width", 0.010);
  node_->declare_parameter("gripper.close_extra", 0.0);
  node_->declare_parameter("gripper.angle_min", 1.01);   // rad，闭合（axis5=0）
  node_->declare_parameter("gripper.angle_max", -1.01);  // rad，张开（axis5=-1100，厂家实际）
  node_->declare_parameter("gripper.width_min", 0.0);
  node_->declare_parameter("gripper.stable_samples", 3);            // 打开/闭合验证：稳定性采样次数
  node_->declare_parameter("gripper.stable_threshold_rad", 0.02);   // 相邻采样变化小于此视为不变 (~1.1°)
  node_->declare_parameter("gripper.stable_velocity_eps_rad_s", 0.01);  // 速度判稳 (rad/s)，<=0 不检查
  node_->declare_parameter("gripper.stable_interval_ms", 50);      // 采样间隔 ms
  node_->declare_parameter("gripper.post_execute_wait_ms", 1000);  // 写目标后等待 ms，再判运动开始/数值不变
  node_->declare_parameter("gripper.motion_start_threshold_rad", 0.008);   // 判定“开始运动”的最小变化 (rad)，约 0.46°
  node_->declare_parameter("gripper.motion_start_timeout_ms", 3000);       // 等待运动开始最大 ms，超时仍进入判稳
  node_->declare_parameter("gripper.stable_timeout_ms", 8000);            // 等待数值不变最大 ms，闭合超时=失败
}

void ParameterManager::loadGripperParameters() {
  gripper_open_width = node_->get_parameter("gripper.open_width").as_double();
  gripper_close_width = node_->get_parameter("gripper.close_width").as_double();
  gripper_close_extra = node_->get_parameter("gripper.close_extra").as_double();
  gripper_angle_min = node_->get_parameter("gripper.angle_min").as_double();
  gripper_angle_max = node_->get_parameter("gripper.angle_max").as_double();
  gripper_width_min = node_->get_parameter("gripper.width_min").as_double();
  gripper_stable_samples = node_->get_parameter("gripper.stable_samples").as_int();
  gripper_stable_threshold_rad = node_->get_parameter("gripper.stable_threshold_rad").as_double();
  gripper_stable_velocity_eps_rad_s = node_->get_parameter("gripper.stable_velocity_eps_rad_s").as_double();
  gripper_stable_interval_ms = node_->get_parameter("gripper.stable_interval_ms").as_int();
  gripper_post_execute_wait_ms = node_->get_parameter("gripper.post_execute_wait_ms").as_int();
  gripper_motion_start_threshold_rad = node_->get_parameter("gripper.motion_start_threshold_rad").as_double();
  gripper_motion_start_timeout_ms = node_->get_parameter("gripper.motion_start_timeout_ms").as_int();
  gripper_stable_timeout_ms = node_->get_parameter("gripper.stable_timeout_ms").as_int();
  LOG_NAMED_INFO("params", "夹爪参数: angle_min={:.3f} rad, angle_max={:.3f} rad, stable_samples={}, stable_threshold_rad={:.3f}, motion_start_threshold={:.3f}, motion_start_timeout_ms={}, stable_timeout_ms={}",
                gripper_angle_min, gripper_angle_max, gripper_stable_samples, gripper_stable_threshold_rad,
                gripper_motion_start_threshold_rad, gripper_motion_start_timeout_ms, gripper_stable_timeout_ms);
}

// ========== Scene Parameters ==========
void ParameterManager::declareSceneParameters() {
  node_->declare_parameter("scene.add_collision_object", true);
  node_->declare_parameter("scene.allow_touch_links", 
      std::vector<std::string>{"LinkGG", "LinkGL", "LinkGR"});
}

void ParameterManager::loadSceneParameters() {
  add_collision_object = node_->get_parameter("scene.add_collision_object").as_bool();
  allow_touch_links = node_->get_parameter("scene.allow_touch_links").as_string_array();
}

// ========== Workspace Parameters ==========
void ParameterManager::declareWorkspaceParameters() {
  node_->declare_parameter("workspace.base_height", 0.2145);
  node_->declare_parameter("workspace.link2_length", 0.264);
  node_->declare_parameter("workspace.link3_length", 0.143);
  node_->declare_parameter("workspace.link4_to_eef", 0.187);
  node_->declare_parameter("workspace.reach_radius_margin", 0.95);
  node_->declare_parameter("workspace.max_height_offset", 0.60);
  node_->declare_parameter("workspace.min_height_offset", -0.10);
  node_->declare_parameter("workspace.min_radius", 0.08);
}

void ParameterManager::loadWorkspaceParameters() {
  workspace_base_height = node_->get_parameter("workspace.base_height").as_double();
  workspace_link2_length = node_->get_parameter("workspace.link2_length").as_double();
  workspace_link3_length = node_->get_parameter("workspace.link3_length").as_double();
  workspace_link4_to_eef = node_->get_parameter("workspace.link4_to_eef").as_double();
  workspace_reach_radius_margin = node_->get_parameter("workspace.reach_radius_margin").as_double();
  workspace_max_height_offset = node_->get_parameter("workspace.max_height_offset").as_double();
  workspace_min_height_offset = node_->get_parameter("workspace.min_height_offset").as_double();
  workspace_min_radius = node_->get_parameter("workspace.min_radius").as_double();
}

// ========== Tolerance Parameters ==========
void ParameterManager::declareToleranceParameters() {
  node_->declare_parameter("tolerance.orientation_epsilon", 1e-6);
}

void ParameterManager::loadToleranceParameters() {
  orientation_epsilon = node_->get_parameter("tolerance.orientation_epsilon").as_double();
}

// ========== Place Parameters ==========
void ParameterManager::declarePlaceParameters() {
  node_->declare_parameter("place.approach_offset_z", 0.10);
  node_->declare_parameter("place.descend_distance", 0.08);
  node_->declare_parameter("place.retreat_distance", 0.06);
}

void ParameterManager::loadPlaceParameters() {
  place_approach_offset_z = node_->get_parameter("place.approach_offset_z").as_double();
  place_descend_distance = node_->get_parameter("place.descend_distance").as_double();
  place_retreat_distance = node_->get_parameter("place.retreat_distance").as_double();
}

// ========== Trajectory Parameters ==========
void ParameterManager::declareTrajectoryParameters() {
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
  node_->declare_parameter("trajectory.min_joint_step_deg", 5.0);  // 3~5° 更平滑，减少末端冲量
}

void ParameterManager::loadTrajectoryParameters() {
  use_linear_interpolation = node_->get_parameter("trajectory.use_linear_interpolation").as_bool();
  linear_velocity = node_->get_parameter("trajectory.linear_velocity").as_double();
  linear_min_duration = node_->get_parameter("trajectory.linear_min_duration").as_double();
  linear_step_scale = node_->get_parameter("trajectory.linear_step_scale").as_double();
  cartesian_timeout = node_->get_parameter("trajectory.cartesian_timeout").as_double();
  cartesian_timeout_descend = node_->get_parameter("trajectory.cartesian_timeout_descend").as_double();
  cartesian_timeout_lift = node_->get_parameter("trajectory.cartesian_timeout_lift").as_double();
  cartesian_descend_threshold = node_->get_parameter("trajectory.cartesian_descend_threshold").as_double();
  cartesian_lift_threshold = node_->get_parameter("trajectory.cartesian_lift_threshold").as_double();
  cartesian_retry_on_timeout = node_->get_parameter("trajectory.cartesian_retry_on_timeout").as_bool();
  adaptive_segments = node_->get_parameter("trajectory.adaptive_segments").as_bool();
  min_segments = node_->get_parameter("trajectory.min_segments").as_int();
  max_segments = node_->get_parameter("trajectory.max_segments").as_int();
  use_polynomial_interpolation = node_->get_parameter("trajectory.use_polynomial_interpolation").as_bool();
  polynomial_type = node_->get_parameter("trajectory.polynomial_type").as_string();
  polynomial_duration = node_->get_parameter("trajectory.polynomial_duration").as_double();
  polynomial_dt = node_->get_parameter("trajectory.polynomial_dt").as_double();
  use_bspline = node_->get_parameter("trajectory.use_bspline").as_bool();
  bspline_degree = node_->get_parameter("trajectory.bspline_degree").as_int();
  bspline_duration = node_->get_parameter("trajectory.bspline_duration").as_double();
  bspline_dt = node_->get_parameter("trajectory.bspline_dt").as_double();
  
  // 时间参数化参数
  use_time_parameterization = node_->get_parameter("trajectory.use_time_parameterization").as_bool();
  time_param_algorithm = node_->get_parameter("trajectory.time_param_algorithm").as_string();
  time_param_velocity_scaling = node_->get_parameter("trajectory.time_param_velocity_scaling").as_double();
  time_param_acceleration_scaling = node_->get_parameter("trajectory.time_param_acceleration_scaling").as_double();
  
  // 轨迹重采样与步长
  use_trajectory_resampling = node_->get_parameter("trajectory.use_trajectory_resampling").as_bool();
  control_frequency = node_->get_parameter("trajectory.control_frequency").as_double();
  trajectory_min_joint_step_deg = node_->get_parameter("trajectory.min_joint_step_deg").as_double();
}

// ========== FSM Parameters ==========
void ParameterManager::declareFSMParameters() {
  // 稳定性阈值（可在yaml中调整，适应不同环境）
  node_->declare_parameter("fsm.stable_position_threshold", 0.01);  // 1cm
  node_->declare_parameter("fsm.stable_yaw_threshold", 0.0873);     // 5度
  node_->declare_parameter("fsm.target_stale_timeout", 0.5);        // 0.5s
  node_->declare_parameter("fsm.jump_position_threshold", 0.03);    // 3cm
  node_->declare_parameter("fsm.jump_yaw_threshold", 0.1745);       // 10度
  node_->declare_parameter("fsm.stable_window_size", 5);            // 5帧
  node_->declare_parameter("fsm.gripper_wait_ms", 8000);            // 8s，夹爪范围执行时间 7.5s
  node_->declare_parameter("fsm.gripper_close_min_dwell_ms", 2000);   // 闭合后最小停留 (ms)，2~4s 更保守
  node_->declare_parameter("fsm.gripper_open_min_dwell_ms", 1500);   // 打开后最小停留 (ms)，再允许转 PREGRASP
  node_->declare_parameter("fsm.arm_stable_timeout_ms", 15000);      // 机械臂稳定等待最大超时 (ms)
  node_->declare_parameter("fsm.arm_stable_threshold_rad", 0.004);   // 关节角度稳定阈值 (rad，约 0.23°)
  node_->declare_parameter("fsm.arm_stable_window", 8);            // 连续满足 pos+vel 的采样次数（8~10 更易达稳）
  node_->declare_parameter("fsm.arm_stable_velocity_eps_rad_s", 0.012);  // 速度判稳阈值 (rad/s)
  node_->declare_parameter("fsm.arm_stable_min_dwell_ms", 800);    // 稳定后最小停留 (ms)
}

void ParameterManager::loadFSMParameters() {
  fsm_stable_position_threshold = node_->get_parameter("fsm.stable_position_threshold").as_double();
  fsm_stable_yaw_threshold = node_->get_parameter("fsm.stable_yaw_threshold").as_double();
  fsm_target_stale_timeout = node_->get_parameter("fsm.target_stale_timeout").as_double();
  fsm_jump_position_threshold = node_->get_parameter("fsm.jump_position_threshold").as_double();
  fsm_jump_yaw_threshold = node_->get_parameter("fsm.jump_yaw_threshold").as_double();
  fsm_stable_window_size = node_->get_parameter("fsm.stable_window_size").as_int();
  fsm_gripper_wait_ms = node_->get_parameter("fsm.gripper_wait_ms").as_int();
  fsm_gripper_close_min_dwell_ms = node_->get_parameter("fsm.gripper_close_min_dwell_ms").as_int();
  fsm_gripper_open_min_dwell_ms = node_->get_parameter("fsm.gripper_open_min_dwell_ms").as_int();
  fsm_arm_stable_timeout_ms = node_->get_parameter("fsm.arm_stable_timeout_ms").as_int();
  fsm_arm_stable_threshold_rad = node_->get_parameter("fsm.arm_stable_threshold_rad").as_double();
  fsm_arm_stable_window = node_->get_parameter("fsm.arm_stable_window").as_int();
  fsm_arm_stable_velocity_eps_rad_s = node_->get_parameter("fsm.arm_stable_velocity_eps_rad_s").as_double();
  fsm_arm_stable_min_dwell_ms = node_->get_parameter("fsm.arm_stable_min_dwell_ms").as_int();
}

void ParameterManager::logParameters() {
  LOG_NAMED_INFO("params", "参数加载完成");
  LOG_NAMED_INFO("params", "缆绳直径: {:.3f} m, 长度: {:.3f} m", cable_diameter, cable_length);
  LOG_NAMED_INFO("params", "夹爪打开宽度: {:.3f} m, 闭合宽度: {:.3f} m", 
              gripper_open_width, gripper_close_width);
  
  LOG_NAMED_INFO("params", "配置参数:");
  LOG_NAMED_INFO("params", "  grasp_yaw_add: {:.3f} rad ({:.1f} deg)", 
              grasp_yaw_add, grasp_yaw_add * 180.0 / M_PI);
  LOG_NAMED_INFO("params", "TCP偏移补偿参数: ({:.4f}, {:.4f}, {:.4f}) m", 
              tcp_offset_x, tcp_offset_y, tcp_offset_z);
  LOG_NAMED_INFO("params", "全零状态预运动: {}, 预备位置: [{:.3f}, {:.3f}, {:.3f}, {:.3f}] rad",
              auto_move_to_ready ? "启用" : "禁用",
              ready_joint1, ready_joint2, ready_joint3, ready_joint4);
  
  LOG_NAMED_INFO("params", "=== 轨迹规划配置 ===");
  LOG_NAMED_INFO("params", "线性插补: {} (速度: {:.3f} m/s)", 
              use_linear_interpolation ? "启用" : "禁用", linear_velocity);
  LOG_NAMED_INFO("params", "笛卡尔路径超时: {:.1f} s", cartesian_timeout);
  LOG_NAMED_INFO("params", "===================");
  
  LOG_NAMED_INFO("params", "=== FSM配置 ===");
  LOG_NAMED_INFO("params", "稳定阈值: 位置={:.3f}m, yaw={:.1f}°", 
              fsm_stable_position_threshold, fsm_stable_yaw_threshold * 180.0 / M_PI);
  LOG_NAMED_INFO("params", "跳变阈值: 位置={:.3f}m, yaw={:.1f}°", 
              fsm_jump_position_threshold, fsm_jump_yaw_threshold * 180.0 / M_PI);
  LOG_NAMED_INFO("params", "滑窗大小: {}, 夹爪等待: {}ms, 闭合min_dwell: {}ms, 打开min_dwell: {}ms, 臂稳定: 超时{}ms/阈值{:.4f}rad/窗{}/vel_eps{:.3f}rad_s/min_dwell{}ms",
              fsm_stable_window_size, fsm_gripper_wait_ms, fsm_gripper_close_min_dwell_ms, fsm_gripper_open_min_dwell_ms,
              fsm_arm_stable_timeout_ms, fsm_arm_stable_threshold_rad, fsm_arm_stable_window,
              fsm_arm_stable_velocity_eps_rad_s, fsm_arm_stable_min_dwell_ms);
  LOG_NAMED_INFO("params", "===================");
  
  LOG_NAMED_INFO("params", "=== 频率配置 ===");
  LOG_NAMED_INFO("params", "主循环频率: {:.1f} Hz", main_loop_frequency);
  LOG_NAMED_INFO("params", "===================");
}

}  // namespace m5_grasp
