#include "sealien_payload_grasp/execution/motion_executor.hpp"
#include "sealien_payload_grasp/logging/logger.hpp"
#include "sealien_payload_grasp/utils/pose_utils.hpp"
#include "sealien_payload_grasp/utils/trajectory_planner.hpp"
#include <algorithm>
#include <chrono>
#include <moveit/robot_state/robot_state.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

namespace sealien_payload_grasp
{

MotionExecutor::MotionExecutor(
    rclcpp::Node* node,
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer, TrajectoryPlanner& trajectory_planner,
    std::mutex& moveit_mutex)
    : node_(node), move_group_interface_(move_group_interface), tf_buffer_(tf_buffer),
      trajectory_planner_(trajectory_planner), moveit_mutex_(moveit_mutex)
{
}

void MotionExecutor::setParameters(
    const std::string& planning_frame, const std::string& eef_link, double ik_timeout, bool is_4dof,
    double eef_step, double jump_threshold, double cartesian_timeout_descend,
    double cartesian_timeout_lift, double cartesian_descend_threshold,
    double cartesian_lift_threshold, bool cartesian_retry_on_timeout, bool use_linear_interpolation,
    double linear_velocity, double linear_min_duration, double linear_step_scale, bool use_bspline,
    int bspline_degree, double bspline_duration, double bspline_dt,
    bool use_polynomial_interpolation, const std::string& polynomial_type,
    double polynomial_duration, double polynomial_dt, bool adaptive_segments, int min_segments,
    int max_segments, double segment_execution_wait, double state_sync_wait, bool enable_recovery,
    double recovery_timeout, double default_planning_time, double fallback_planning_time,
    bool yaw_validation_enabled, double yaw_tolerance, double yaw_search_step)
{
    planning_frame_ = planning_frame;
    eef_link_ = eef_link;
    ik_timeout_ = ik_timeout;
    is_4dof_ = is_4dof;
    eef_step_ = eef_step;
    jump_threshold_ = jump_threshold;
    cartesian_timeout_descend_ = cartesian_timeout_descend;
    cartesian_timeout_lift_ = cartesian_timeout_lift;
    cartesian_descend_threshold_ = cartesian_descend_threshold;
    cartesian_lift_threshold_ = cartesian_lift_threshold;
    cartesian_retry_on_timeout_ = cartesian_retry_on_timeout;
    use_linear_interpolation_ = use_linear_interpolation;
    linear_velocity_ = linear_velocity;
    linear_min_duration_ = linear_min_duration;
    linear_step_scale_ = linear_step_scale;
    use_bspline_ = use_bspline;
    bspline_degree_ = bspline_degree;
    bspline_duration_ = bspline_duration;
    bspline_dt_ = bspline_dt;
    use_polynomial_interpolation_ = use_polynomial_interpolation;
    polynomial_type_ = polynomial_type;
    polynomial_duration_ = polynomial_duration;
    polynomial_dt_ = polynomial_dt;
    adaptive_segments_ = adaptive_segments;
    min_segments_ = min_segments;
    max_segments_ = max_segments;
    segment_execution_wait_ = segment_execution_wait;
    state_sync_wait_ = state_sync_wait;
    enable_recovery_ = enable_recovery;
    recovery_timeout_ = recovery_timeout;
    default_planning_time_ = default_planning_time;
    fallback_planning_time_ = fallback_planning_time;
    yaw_validation_enabled_ = yaw_validation_enabled;
    yaw_tolerance_ = yaw_tolerance;
    yaw_search_step_ = yaw_search_step;
}

void MotionExecutor::setTransformCallback(
    std::function<bool(geometry_msgs::msg::PoseStamped&)> callback)
{
    transform_to_planning_callback_ = callback;
}

void MotionExecutor::setFindValidYawCallback(
    std::function<double(double, double, double, double, moveit::core::RobotStatePtr, double,
                         double)>
        callback)
{
    find_valid_yaw_callback_ = callback;
}

void MotionExecutor::setComputeOrientationCallbacks(
    std::function<geometry_msgs::msg::Quaternion()> compute_downward,
    std::function<geometry_msgs::msg::Quaternion(double)> compute_downward_with_yaw,
    std::function<std::vector<geometry_msgs::msg::Quaternion>()> compute_candidates)
{
    compute_downward_callback_ = compute_downward;
    compute_downward_with_yaw_callback_ = compute_downward_with_yaw;
    compute_orientation_candidates_callback_ = compute_candidates;
}

void MotionExecutor::setPublishStateCallback(
    std::function<void(const std::string&, const std::string&)> callback)
{
    publish_state_callback_ = callback;
}

moveit::core::RobotStatePtr MotionExecutor::get_current_state_safe(double timeout)
{
    std::lock_guard<std::mutex> lock(moveit_mutex_);

    try
    {
        const auto& robot_model = move_group_interface_->getRobotModel();
        if (robot_model)
        {
            auto state = std::make_shared<moveit::core::RobotState>(robot_model);
            state->setToDefaultValues();

            const auto* arm_jmg = robot_model->getJointModelGroup("arm_group");
            if (arm_jmg)
            {
                std::vector<double> arm_joint_values =
                    move_group_interface_->getCurrentJointValues();
                if (arm_joint_values.size() == arm_jmg->getActiveJointModelNames().size())
                {
                    state->setJointGroupPositions(arm_jmg, arm_joint_values);
                }
            }

            state->update();
            state->enforceBounds();
            return state;
        }
    }
    catch (const std::exception& e)
    {
        LOG_NAMED_WARN("motion", "get_current_state_safe异常: {}", e.what());
    }

    double actual_timeout = (timeout > 0.0) ? std::max(timeout, 1.0) : 3.0;
    auto state = move_group_interface_->getCurrentState(actual_timeout);
    if (state)
    {
        state->enforceBounds();
    }
    return state;
}

std::vector<double> MotionExecutor::get_current_joint_values_safe()
{
    std::lock_guard<std::mutex> lock(moveit_mutex_);
    return move_group_interface_->getCurrentJointValues();
}

geometry_msgs::msg::PoseStamped MotionExecutor::get_current_pose_safe()
{
    std::lock_guard<std::mutex> lock(moveit_mutex_);
    return move_group_interface_->getCurrentPose();
}

void MotionExecutor::setStartStateExplicit()
{
    try
    {
        auto current_state = move_group_interface_->getCurrentState(0.0);
        if (current_state)
        {
            current_state->enforceBounds();
            move_group_interface_->setStartState(*current_state);
        }
        else
        {
            move_group_interface_->setStartStateToCurrentState();
        }
    }
    catch (const std::exception& e)
    {
        LOG_NAMED_WARN("motion", "setStartStateExplicit异常: {}", e.what());
        move_group_interface_->setStartStateToCurrentState();
    }
}

bool MotionExecutor::waitForStateStable(double timeout_sec, int check_count, double threshold)
{
    std::vector<double> prev_joints;
    int stable_count = 0;

    auto start_time = std::chrono::steady_clock::now();
    auto timeout_duration = std::chrono::duration<double>(timeout_sec);

    while (std::chrono::steady_clock::now() - start_time < timeout_duration)
    {
        std::vector<double> current_joints = get_current_joint_values_safe();

        if (!prev_joints.empty() && current_joints.size() == prev_joints.size())
        {
            double max_diff = 0.0;
            for (size_t i = 0; i < current_joints.size(); ++i)
            {
                max_diff = std::max(max_diff, std::abs(current_joints[i] - prev_joints[i]));
            }

            if (max_diff < threshold)
            {
                stable_count++;
                if (stable_count >= check_count)
                {
                    return true;
                }
            }
            else
            {
                stable_count = 0;
            }
        }

        prev_joints = current_joints;
        // 分片 10ms，避免长时间阻塞 executor
        for (int k = 0; k < 5; ++k)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    return false;
}

double MotionExecutor::computeCartesianPathWithTimeout(
    const std::vector<geometry_msgs::msg::Pose>& waypoints, double step, double jump_threshold,
    moveit_msgs::msg::RobotTrajectory& trajectory, double timeout_sec, bool allow_retry)
{
    {
        std::lock_guard<std::mutex> lock(moveit_mutex_);
        move_group_interface_->setStartStateToCurrentState();
    }

    auto future = std::async(std::launch::async,
                             [this, &waypoints, step, jump_threshold, &trajectory]() -> double
                             {
                                 std::lock_guard<std::mutex> lock(moveit_mutex_);
                                 return move_group_interface_->computeCartesianPath(
                                     waypoints, step, jump_threshold, trajectory);
                             });

    auto status = future.wait_for(std::chrono::milliseconds(static_cast<int>(timeout_sec * 1000)));

    if (status == std::future_status::timeout)
    {
        LOG_NAMED_WARN("motion", "笛卡尔路径计算超时（{:.1f}秒，{}个waypoints）", timeout_sec,
                       waypoints.size());

        if (allow_retry && cartesian_retry_on_timeout_ && waypoints.size() > 2)
        {
            std::vector<geometry_msgs::msg::Pose> reduced_waypoints;
            reduced_waypoints.push_back(waypoints.front());
            if (waypoints.size() > 2)
            {
                size_t mid_index = waypoints.size() / 2;
                reduced_waypoints.push_back(waypoints[mid_index]);
            }
            reduced_waypoints.push_back(waypoints.back());

            moveit_msgs::msg::RobotTrajectory retry_trajectory;
            double retry_fraction =
                computeCartesianPathWithTimeout(reduced_waypoints, step, jump_threshold,
                                                retry_trajectory, timeout_sec * 0.7, false);

            if (retry_fraction >= 0.0)
            {
                trajectory = retry_trajectory;
                return retry_fraction;
            }
        }

        return -1.0;
    }

    try
    {
        return future.get();
    }
    catch (const std::exception& e)
    {
        LOG_NAMED_ERROR("motion", "笛卡尔路径计算异常: {}", e.what());
        return -1.0;
    }
}

bool MotionExecutor::executeCartesian(const std::vector<geometry_msgs::msg::Pose>& waypoints,
                                      bool use_descend)
{
    double timeout = use_descend ? cartesian_timeout_descend_ : cartesian_timeout_lift_;

    // B样条路径
    if (use_bspline_ && waypoints.size() >= static_cast<size_t>(bspline_degree_ + 1))
    {
        try
        {
            std::vector<geometry_msgs::msg::Pose> bspline_waypoints =
                trajectory_planner_.generate_bspline_cartesian_trajectory(
                    waypoints, bspline_degree_, bspline_duration_, bspline_dt_);

            if (!bspline_waypoints.empty())
            {
                moveit_msgs::msg::RobotTrajectory trajectory;
                double fraction = computeCartesianPathWithTimeout(
                    bspline_waypoints, eef_step_, jump_threshold_, trajectory, timeout, true);

                if (fraction >= 0.0)
                {
                    double threshold =
                        use_descend ? cartesian_descend_threshold_ : cartesian_lift_threshold_;
                    if (fraction >= threshold)
                    {
                        moveit::planning_interface::MoveGroupInterface::Plan plan;
                        plan.trajectory_ = trajectory;

                        {
                            std::lock_guard<std::mutex> lock(moveit_mutex_);
                            moveit::core::MoveItErrorCode result =
                                move_group_interface_->execute(plan);
                            if (result == moveit::core::MoveItErrorCode::SUCCESS)
                            {
                                return true;
                            }
                        }
                    }
                }
            }
        }
        catch (const std::exception& e)
        {
            LOG_NAMED_WARN("motion", "B样条轨迹生成失败: {}", e.what());
        }
    }

    // 线性插补
    if (use_linear_interpolation_ && waypoints.size() == 2)
    {
        double distance = std::sqrt(std::pow(waypoints[1].position.x - waypoints[0].position.x, 2) +
                                    std::pow(waypoints[1].position.y - waypoints[0].position.y, 2) +
                                    std::pow(waypoints[1].position.z - waypoints[0].position.z, 2));

        double duration = std::max(linear_min_duration_, distance / linear_velocity_);

        try
        {
            std::vector<geometry_msgs::msg::Pose> linear_waypoints =
                trajectory_planner_.generate_linear_cartesian_trajectory(waypoints[0], waypoints[1],
                                                                      duration, eef_step_);

            if (!linear_waypoints.empty())
            {
                moveit_msgs::msg::RobotTrajectory trajectory;
                double scaled_step = eef_step_ * linear_step_scale_;
                double fraction = computeCartesianPathWithTimeout(
                    linear_waypoints, scaled_step, jump_threshold_, trajectory, timeout, true);

                if (fraction >= 0.0)
                {
                    double threshold =
                        use_descend ? cartesian_descend_threshold_ : cartesian_lift_threshold_;
                    if (fraction >= threshold)
                    {
                        moveit::planning_interface::MoveGroupInterface::Plan plan;
                        plan.trajectory_ = trajectory;

                        {
                            std::lock_guard<std::mutex> lock(moveit_mutex_);
                            moveit::core::MoveItErrorCode result =
                                move_group_interface_->execute(plan);
                            if (result == moveit::core::MoveItErrorCode::SUCCESS)
                            {
                                return true;
                            }
                        }
                    }
                }
            }
        }
        catch (const std::exception& e)
        {
            LOG_NAMED_WARN("motion", "线性插补生成失败: {}", e.what());
        }
    }

    // 标准笛卡尔路径
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = computeCartesianPathWithTimeout(waypoints, eef_step_, jump_threshold_,
                                                      trajectory, timeout, true);

    if (fraction < 0.0)
    {
        return false;
    }

    double threshold = use_descend ? cartesian_descend_threshold_ : cartesian_lift_threshold_;
    if (fraction < threshold)
    {
        return false;
    }

    {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;

        std::lock_guard<std::mutex> lock(moveit_mutex_);
        moveit::core::MoveItErrorCode result = move_group_interface_->execute(plan);
        return result == moveit::core::MoveItErrorCode::SUCCESS;
    }
}

int MotionExecutor::calculate_adaptive_segments(const geometry_msgs::msg::PoseStamped& start_pose,
                                              const geometry_msgs::msg::PoseStamped& end_pose)
{
    if (!adaptive_segments_)
    {
        return 3;
    }

    double dx = end_pose.pose.position.x - start_pose.pose.position.x;
    double dy = end_pose.pose.position.y - start_pose.pose.position.y;
    double dz = end_pose.pose.position.z - start_pose.pose.position.z;
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    int segments;
    if (distance < 0.05)
    {
        segments = 2;
    }
    else if (distance < 0.10)
    {
        segments = 3;
    }
    else
    {
        segments = 4;
    }

    return std::max(min_segments_, std::min(max_segments_, segments));
}

bool MotionExecutor::execute_segmented_descend(const geometry_msgs::msg::PoseStamped& start_pose,
                                             const geometry_msgs::msg::PoseStamped& end_pose,
                                             int num_segments)
{
    if (num_segments < 0)
    {
        num_segments = calculate_adaptive_segments(start_pose, end_pose);
    }

    double dx = (end_pose.pose.position.x - start_pose.pose.position.x) / num_segments;
    double dy = (end_pose.pose.position.y - start_pose.pose.position.y) / num_segments;
    double dz = (end_pose.pose.position.z - start_pose.pose.position.z) / num_segments;

    std::vector<geometry_msgs::msg::PoseStamped> waypoints_stamped;
    waypoints_stamped.push_back(start_pose);

    moveit::core::RobotStatePtr state_for_yaw_check;
    {
        std::lock_guard<std::mutex> lock(moveit_mutex_);
        state_for_yaw_check = move_group_interface_->getCurrentState(0.0);
    }

    double target_yaw = extract_yaw_from_quaternion(end_pose.pose.orientation);

    for (int i = 1; i < num_segments; ++i)
    {
        geometry_msgs::msg::PoseStamped waypoint = start_pose;
        waypoint.pose.position.x += dx * i;
        waypoint.pose.position.y += dy * i;
        waypoint.pose.position.z += dz * i;

        if (yaw_validation_enabled_ && state_for_yaw_check && find_valid_yaw_callback_)
        {
            double valid_yaw = find_valid_yaw_callback_(
                waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z,
                target_yaw, state_for_yaw_check, yaw_tolerance_, yaw_search_step_);

            if (std::abs(valid_yaw - target_yaw) > 1e-6 && compute_downward_with_yaw_callback_)
            {
                waypoint.pose.orientation = compute_downward_with_yaw_callback_(valid_yaw);
            }
            else
            {
                waypoint.pose.orientation = end_pose.pose.orientation;
            }
        }
        else
        {
            waypoint.pose.orientation = end_pose.pose.orientation;
        }

        waypoints_stamped.push_back(waypoint);
    }

    waypoints_stamped.push_back(end_pose);

    // 转换到planning frame
    for (auto& wp : waypoints_stamped)
    {
        if (wp.header.frame_id != planning_frame_ && transform_to_planning_callback_)
        {
            if (!transform_to_planning_callback_(wp))
            {
                LOG_NAMED_ERROR("motion", "[分段下压] waypoint转换失败");
                return false;
            }
        }
    }

    // 逐段执行
    for (size_t i = 1; i < waypoints_stamped.size(); ++i)
    {
        {
            std::lock_guard<std::mutex> lock(moveit_mutex_);
            move_group_interface_->setStartStateToCurrentState();
        }

        if (!planExecuteIKJointTarget(waypoints_stamped[i]))
        {
            if (enable_recovery_)
            {
                recoverToPregrasp(start_pose);
            }
            return false;
        }

        if (i == waypoints_stamped.size() - 1)
        {
            for (int k = 0; k < 5; ++k)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }

    return true;
}

bool MotionExecutor::executeSegmentedLift(const geometry_msgs::msg::PoseStamped& start_pose,
                                          const geometry_msgs::msg::PoseStamped& end_pose,
                                          int num_segments)
{
    double dx = (end_pose.pose.position.x - start_pose.pose.position.x) / num_segments;
    double dy = (end_pose.pose.position.y - start_pose.pose.position.y) / num_segments;
    double dz = (end_pose.pose.position.z - start_pose.pose.position.z) / num_segments;

    std::vector<geometry_msgs::msg::PoseStamped> waypoints;
    waypoints.push_back(start_pose);

    for (int i = 1; i < num_segments; ++i)
    {
        geometry_msgs::msg::PoseStamped waypoint = start_pose;
        waypoint.pose.position.x += dx * i;
        waypoint.pose.position.y += dy * i;
        waypoint.pose.position.z += dz * i;
        waypoint.pose.orientation = end_pose.pose.orientation;
        waypoints.push_back(waypoint);
    }

    waypoints.push_back(end_pose);

    // 逐段执行（分片 sleep 避免长时间阻塞 executor）
    const int chunk_ms = 10;
    for (size_t i = 1; i < waypoints.size(); ++i)
    {
        int total_ms = static_cast<int>(state_sync_wait_ * 1000);
        for (int t = 0; t < total_ms; t += chunk_ms)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(chunk_ms));
        }

        {
            std::lock_guard<std::mutex> lock(moveit_mutex_);
            move_group_interface_->setStartStateToCurrentState();
        }

        if (!planExecuteIKJointTarget(waypoints[i]))
        {
            if (enable_recovery_ && !waypoints.empty())
            {
                recoverToPregrasp(waypoints[0]);
            }
            return false;
        }

        waitForStateStable(segment_execution_wait_, 5, 0.002);
    }

    return true;
}

bool MotionExecutor::checkIKOnlyImpl(const geometry_msgs::msg::PoseStamped& pose_in_planning,
                                     const moveit::core::RobotStatePtr& state)
{
    if (!state)
    {
        return false;
    }

    const auto* jmg = state->getJointModelGroup("arm_group");
    if (!jmg)
    {
        return false;
    }

    return state->setFromIK(jmg, pose_in_planning.pose, eef_link_, ik_timeout_);
}

bool MotionExecutor::planExecuteIKJointTargetImpl(
    const geometry_msgs::msg::PoseStamped& target_pose, int max_ik_attempts)
{
    geometry_msgs::msg::PoseStamped pose_in_planning = target_pose;
    if (transform_to_planning_callback_ && !transform_to_planning_callback_(pose_in_planning))
    {
        return false;
    }

    // 分片 sleep 避免长时间阻塞 executor
    {
        int total_ms = static_cast<int>(state_sync_wait_ * 1000);
        const int chunk_ms = 10;
        for (int t = 0; t < total_ms; t += chunk_ms)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(chunk_ms));
        }
    }
    // ========== 关键修复：使用更可靠的状态同步方法 ==========
    // 使用 setStartStateExplicit() 而不是 setStartStateToCurrentState()
    // 因为它会强制执行边界检查，更可靠
    setStartStateExplicit();

    auto state = move_group_interface_->getCurrentState(0.0);
    if (!state)
    {
        return false;
    }

    const auto* jmg = state->getJointModelGroup("arm_group");
    if (!jmg)
    {
        return false;
    }

    std::vector<double> joint_values;
    bool ik_found = false;

    for (int attempt = 0; attempt < max_ik_attempts; ++attempt)
    {
        if (attempt > 0)
        {
            state->setToRandomPositions(jmg);
        }
        else
        {
            state = move_group_interface_->getCurrentState(0.0);
            if (!state)
                continue;
        }

        if (state->setFromIK(jmg, pose_in_planning.pose, eef_link_, ik_timeout_))
        {
            state->copyJointGroupPositions(jmg, joint_values);
            ik_found = true;
            break;
        }
    }

    if (!ik_found)
    {
        return false;
    }

    // 多项式插值
    if (use_polynomial_interpolation_)
    {
        try
        {
            std::vector<std::string> joint_names = jmg->getActiveJointModelNames();
            std::vector<double> start_joints = move_group_interface_->getCurrentJointValues();

            trajectory_msgs::msg::JointTrajectory trajectory;
            bool use_quintic = (polynomial_type_ == "quintic");

            if (use_quintic)
            {
                std::vector<double> start_vel(joint_names.size(), 0.0);
                std::vector<double> end_vel(joint_names.size(), 0.0);
                std::vector<double> start_acc(joint_names.size(), 0.0);
                std::vector<double> end_acc(joint_names.size(), 0.0);

                trajectory = trajectory_planner_.generate_quintic_polynomial_trajectory(
                    start_joints, joint_values, start_vel, end_vel, start_acc, end_acc,
                    polynomial_duration_, joint_names, polynomial_dt_);
            }
            else
            {
                std::vector<double> start_vel(joint_names.size(), 0.0);
                std::vector<double> end_vel(joint_names.size(), 0.0);

                trajectory = trajectory_planner_.generate_cubic_polynomial_trajectory(
                    start_joints, joint_values, start_vel, end_vel, polynomial_duration_,
                    joint_names, polynomial_dt_);
            }

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_.joint_trajectory = trajectory;
            plan.trajectory_.joint_trajectory.header.stamp = node_->get_clock()->now();
            plan.trajectory_.joint_trajectory.header.frame_id = planning_frame_;

            move_group_interface_->setStartStateToCurrentState();

            moveit::core::MoveItErrorCode result = move_group_interface_->execute(plan);
            if (result == moveit::core::MoveItErrorCode::SUCCESS)
            {
                return true;
            }
        }
        catch (const std::exception& e)
        {
            LOG_NAMED_WARN("motion", "多项式插值失败: {}", e.what());
        }
    }

    // 标准MoveIt规划
    setStartStateExplicit();
    move_group_interface_->clearPoseTargets();
    move_group_interface_->clearPathConstraints();
    move_group_interface_->setJointValueTarget(joint_values);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode result = move_group_interface_->plan(plan);

    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
        for (int k = 0; k < 5; ++k)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        setStartStateExplicit();

        result = move_group_interface_->execute(plan);
        return result == moveit::core::MoveItErrorCode::SUCCESS;
    }

    return false;
}

bool MotionExecutor::planExecuteIKJointTarget(const geometry_msgs::msg::PoseStamped& target_pose,
                                              int max_ik_attempts)
{
    std::lock_guard<std::mutex> lock(moveit_mutex_);
    return planExecuteIKJointTargetImpl(target_pose, max_ik_attempts);
}

bool MotionExecutor::executePolynomialJointTrajectory(const std::vector<double>& start_joints,
                                                      const std::vector<double>& end_joints)
{
    if (!use_polynomial_interpolation_)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(moveit_mutex_);

    try
    {
        auto state = move_group_interface_->getCurrentState(0.0);
        if (!state)
            return false;

        const auto* jmg = state->getJointModelGroup("arm_group");
        if (!jmg)
            return false;

        std::vector<std::string> joint_names = jmg->getActiveJointModelNames();

        if (start_joints.size() != end_joints.size() || start_joints.size() != joint_names.size())
        {
            return false;
        }

        trajectory_msgs::msg::JointTrajectory trajectory;
        bool use_quintic = (polynomial_type_ == "quintic");

        if (use_quintic)
        {
            std::vector<double> start_vel(joint_names.size(), 0.0);
            std::vector<double> end_vel(joint_names.size(), 0.0);
            std::vector<double> start_acc(joint_names.size(), 0.0);
            std::vector<double> end_acc(joint_names.size(), 0.0);

            trajectory = trajectory_planner_.generate_quintic_polynomial_trajectory(
                start_joints, end_joints, start_vel, end_vel, start_acc, end_acc,
                polynomial_duration_, joint_names, polynomial_dt_);
        }
        else
        {
            std::vector<double> start_vel(joint_names.size(), 0.0);
            std::vector<double> end_vel(joint_names.size(), 0.0);

            trajectory = trajectory_planner_.generate_cubic_polynomial_trajectory(
                start_joints, end_joints, start_vel, end_vel, polynomial_duration_, joint_names,
                polynomial_dt_);
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_.joint_trajectory = trajectory;
        plan.trajectory_.joint_trajectory.header.stamp = node_->get_clock()->now();
        plan.trajectory_.joint_trajectory.header.frame_id = planning_frame_;

        move_group_interface_->setStartStateToCurrentState();

        moveit::core::MoveItErrorCode result = move_group_interface_->execute(plan);
        return result == moveit::core::MoveItErrorCode::SUCCESS;
    }
    catch (const std::exception& e)
    {
        LOG_NAMED_ERROR("motion", "多项式轨迹执行失败: {}", e.what());
        return false;
    }
}

bool MotionExecutor::executeBSplineJointTrajectory(
    const std::vector<std::vector<double>>& control_points)
{
    if (!use_bspline_)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(moveit_mutex_);

    try
    {
        if (control_points.size() < static_cast<size_t>(bspline_degree_ + 1))
        {
            return false;
        }

        auto state = move_group_interface_->getCurrentState(0.0);
        if (!state)
            return false;

        const auto* jmg = state->getJointModelGroup("arm_group");
        if (!jmg)
            return false;

        std::vector<std::string> joint_names = jmg->getActiveJointModelNames();

        trajectory_msgs::msg::JointTrajectory trajectory =
            trajectory_planner_.generate_bspline_joint_trajectory(
                control_points, bspline_degree_, bspline_duration_, joint_names, bspline_dt_);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_.joint_trajectory = trajectory;
        plan.trajectory_.joint_trajectory.header.stamp = node_->get_clock()->now();
        plan.trajectory_.joint_trajectory.header.frame_id = planning_frame_;

        move_group_interface_->setStartStateToCurrentState();

        moveit::core::MoveItErrorCode result = move_group_interface_->execute(plan);
        return result == moveit::core::MoveItErrorCode::SUCCESS;
    }
    catch (const std::exception& e)
    {
        LOG_NAMED_ERROR("motion", "B样条轨迹执行失败: {}", e.what());
        return false;
    }
}

bool MotionExecutor::planExecutePoseTarget(const geometry_msgs::msg::PoseStamped& target_pose)
{
    geometry_msgs::msg::PoseStamped pose_in_planning = target_pose;
    if (transform_to_planning_callback_ && !transform_to_planning_callback_(pose_in_planning))
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(moveit_mutex_);

    // 主路径：IK→joint target
    if (planExecuteIKJointTargetImpl(pose_in_planning))
    {
        return true;
    }

    // Fallback 1: 位置目标
    move_group_interface_->setPlanningTime(fallback_planning_time_);
    setStartStateExplicit();
    move_group_interface_->clearPoseTargets();
    move_group_interface_->clearPathConstraints();
    move_group_interface_->setPositionTarget(pose_in_planning.pose.position.x,
                                             pose_in_planning.pose.position.y,
                                             pose_in_planning.pose.position.z, eef_link_);
    move_group_interface_->setGoalOrientationTolerance(3.14);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode result = move_group_interface_->plan(plan);

    move_group_interface_->setPlanningTime(default_planning_time_);

    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
        result = move_group_interface_->execute(plan);
        if (result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            move_group_interface_->setGoalOrientationTolerance(0.5);
            return true;
        }
    }

    move_group_interface_->setGoalOrientationTolerance(0.5);

    // Fallback 2: 完整姿态目标
    move_group_interface_->setPlanningTime(fallback_planning_time_);

    setStartStateExplicit();
    move_group_interface_->clearPoseTargets();
    move_group_interface_->clearPathConstraints();
    move_group_interface_->setPoseTarget(pose_in_planning);
    move_group_interface_->setGoalOrientationTolerance(0.5);

    result = move_group_interface_->plan(plan);
    move_group_interface_->setPlanningTime(default_planning_time_);

    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
        result = move_group_interface_->execute(plan);
        if (result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            return true;
        }
    }

    // Fallback 3: Pilz PTP
    setStartStateExplicit();
    move_group_interface_->clearPoseTargets();
    move_group_interface_->clearPathConstraints();
    move_group_interface_->setPositionTarget(pose_in_planning.pose.position.x,
                                             pose_in_planning.pose.position.y,
                                             pose_in_planning.pose.position.z, eef_link_);
    move_group_interface_->setGoalOrientationTolerance(3.14);

    std::string old_pipeline = move_group_interface_->getPlanningPipelineId();
    std::string old_planner = move_group_interface_->getPlannerId();
    move_group_interface_->setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group_interface_->setPlannerId("PTP");

    result = move_group_interface_->plan(plan);

    move_group_interface_->setPlanningPipelineId(old_pipeline);
    move_group_interface_->setPlannerId(old_planner);
    move_group_interface_->setGoalOrientationTolerance(0.5);

    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
        result = move_group_interface_->execute(plan);
        if (result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            return true;
        }
    }

    return false;
}

bool MotionExecutor::recoverToPregrasp(const geometry_msgs::msg::PoseStamped& pregrasp_pose)
{
    if (!enable_recovery_)
    {
        return false;
    }

    if (publish_state_callback_)
    {
        publish_state_callback_("状态:回滚中", "");
    }

    geometry_msgs::msg::PoseStamped pregrasp_planning = pregrasp_pose;
    if (transform_to_planning_callback_ && !transform_to_planning_callback_(pregrasp_planning))
    {
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(moveit_mutex_);
        double original_planning_time = default_planning_time_;
        move_group_interface_->setPlanningTime(recovery_timeout_);
        move_group_interface_->setStartStateToCurrentState();

        bool recovery_success = planExecuteIKJointTargetImpl(pregrasp_planning);

        move_group_interface_->setPlanningTime(original_planning_time);

        if (recovery_success && publish_state_callback_)
        {
            publish_state_callback_("状态:回滚成功", "");
        }
        else if (publish_state_callback_)
        {
            publish_state_callback_("状态:回滚失败", "");
        }

        return recovery_success;
    }
}

bool MotionExecutor::executeWithRecovery(moveit::planning_interface::MoveGroupInterface::Plan& plan,
                                         const geometry_msgs::msg::PoseStamped& recovery_pose)
{
    moveit::core::MoveItErrorCode result;
    {
        std::lock_guard<std::mutex> lock(moveit_mutex_);
        result = move_group_interface_->execute(plan);
    }

    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
        return true;
    }

    bool recovery_pose_valid =
        !(recovery_pose.pose.position.x == 0.0 && recovery_pose.pose.position.y == 0.0 &&
          recovery_pose.pose.position.z == 0.0);

    if (enable_recovery_ && recovery_pose_valid)
    {
        return recoverToPregrasp(recovery_pose);
    }

    return false;
}

} // namespace sealien_payload_grasp
