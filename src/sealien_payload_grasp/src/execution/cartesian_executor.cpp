#include "sealien_payload_grasp/execution/cartesian_executor.hpp"
#include "sealien_payload_grasp/logging/logger.hpp"
#include <chrono>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

namespace sealien_payload_grasp
{

CartesianExecutor::CartesianExecutor(rclcpp::Node* node, MoveGroupInterfacePtr move_group,
                                     std::mutex& mutex)
    : node_(node), move_group_interface_(move_group), moveit_mutex_(mutex)
{
}

void CartesianExecutor::set_parameters(double eef_step, double jump_threshold,
                                      double cartesian_timeout, double cartesian_timeout_descend,
                                      double cartesian_timeout_lift,
                                      double cartesian_descend_threshold,
                                      double cartesian_lift_threshold,
                                      bool cartesian_retry_on_timeout)
{
    eef_step_ = eef_step;
    jump_threshold_ = jump_threshold;
    cartesian_timeout_ = cartesian_timeout;
    cartesian_timeout_descend_ = cartesian_timeout_descend;
    cartesian_timeout_lift_ = cartesian_timeout_lift;
    cartesian_descend_threshold_ = cartesian_descend_threshold;
    cartesian_lift_threshold_ = cartesian_lift_threshold;
    cartesian_retry_on_timeout_ = cartesian_retry_on_timeout;
}

void CartesianExecutor::set_linear_parameters(bool use_linear, double linear_velocity,
                                            double linear_min_duration, double linear_step_scale)
{
    use_linear_interpolation_ = use_linear;
    linear_velocity_ = linear_velocity;
    linear_min_duration_ = linear_min_duration;
    linear_step_scale_ = linear_step_scale;
}

void CartesianExecutor::set_bspline_parameters(bool use_bspline, int bspline_degree,
                                             double bspline_duration, double bspline_dt)
{
    use_bspline_ = use_bspline;
    bspline_degree_ = bspline_degree;
    bspline_duration_ = bspline_duration;
    bspline_dt_ = bspline_dt;
}

void CartesianExecutor::set_segment_parameters(bool adaptive_segments, int min_segments,
                                             int max_segments, double state_sync_wait,
                                             double segment_execution_wait, bool enable_recovery)
{
    adaptive_segments_ = adaptive_segments;
    min_segments_ = min_segments;
    max_segments_ = max_segments;
    state_sync_wait_ = state_sync_wait;
    segment_execution_wait_ = segment_execution_wait;
    enable_recovery_ = enable_recovery;
}

void CartesianExecutor::set_yaw_validation_parameters(bool enabled, double tolerance,
                                                       double search_step)
{
    yaw_validation_enabled_ = enabled;
    yaw_tolerance_ = tolerance;
    yaw_search_step_ = search_step;
}

void CartesianExecutor::set_callbacks(TransformCallback transform_cb,
                                     FindValidYawCallback find_yaw_cb,
                                     ComputeOrientationCallback compute_orientation_cb,
                                     PlanExecuteCallback plan_execute_cb,
                                     RecoverCallback recover_cb, WaitForStateCallback wait_state_cb)
{
    transform_callback_ = transform_cb;
    find_valid_yaw_callback_ = find_yaw_cb;
    compute_orientation_callback_ = compute_orientation_cb;
    plan_execute_callback_ = plan_execute_cb;
    recover_callback_ = recover_cb;
    wait_state_callback_ = wait_state_cb;
}

void CartesianExecutor::set_trajectory_planner(TrajectoryPlanner* planner)
{
    trajectory_planner_ = planner;
}

void CartesianExecutor::set_frames(const std::string& planning_frame, const std::string& eef_link)
{
    planning_frame_ = planning_frame;
    eef_link_ = eef_link;
}

void CartesianExecutor::set_ik_timeout(double timeout)
{
    ik_timeout_ = timeout;
}

double CartesianExecutor::compute_cartesian_path_with_timeout(
    const std::vector<geometry_msgs::msg::Pose>& waypoints, double step, double jump_threshold,
    moveit_msgs::msg::RobotTrajectory& trajectory, double timeout_sec, bool allow_retry)
{
    // 在锁内准备状态
    {
        std::lock_guard<std::mutex> lock(moveit_mutex_);
        move_group_interface_->setStartStateToCurrentState();
    }

    // 在单独线程中执行 computeCartesianPath，避免阻塞
    auto future = std::async(std::launch::async,
                             [this, &waypoints, step, jump_threshold, &trajectory]() -> double
                             {
                                 std::lock_guard<std::mutex> lock(moveit_mutex_);
                                 return move_group_interface_->computeCartesianPath(
                                     waypoints, step, jump_threshold, trajectory);
                             });

    // 等待结果，带超时
    auto status = future.wait_for(std::chrono::milliseconds(static_cast<int>(timeout_sec * 1000)));

    if (status == std::future_status::timeout)
    {
        LOG_NAMED_WARN("cartesian", "笛卡尔路径计算超时（{:.1f}秒，{}个waypoints）", timeout_sec,
                       waypoints.size());

        // 如果允许重试且waypoints数量足够，尝试减少waypoints后重试
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

            LOG_NAMED_INFO("cartesian",
                           "笛卡尔路径重试：减少waypoints从{}到{}，使用更短的超时时间（{:.1f}秒）",
                           waypoints.size(), reduced_waypoints.size(), timeout_sec * 0.7);

            moveit_msgs::msg::RobotTrajectory retry_trajectory;
            double retry_fraction =
                compute_cartesian_path_with_timeout(reduced_waypoints, step, jump_threshold,
                                                retry_trajectory, timeout_sec * 0.7, false);

            if (retry_fraction >= 0.0)
            {
                trajectory = retry_trajectory;
                LOG_NAMED_INFO("cartesian", "笛卡尔路径重试成功，成功率: {:.2f}%%",
                               retry_fraction * 100.0);
                return retry_fraction;
            }
            else
            {
                LOG_NAMED_WARN("cartesian", "笛卡尔路径重试也失败，触发fallback");
            }
        }

        return -1.0;
    }

    // 获取结果
    try
    {
        double fraction = future.get();
        return fraction;
    }
    catch (const std::exception& e)
    {
        LOG_NAMED_ERROR("cartesian", "笛卡尔路径计算异常: {}", e.what());
        return -1.0;
    }
}

bool CartesianExecutor::executeCartesian(const std::vector<geometry_msgs::msg::Pose>& waypoints,
                                         bool use_descend)
{
    double timeout = use_descend ? cartesian_timeout_descend_ : cartesian_timeout_lift_;

    // 如果配置启用B样条且waypoints数量足够，使用B样条生成平滑笛卡尔轨迹
    if (use_bspline_ && trajectory_planner_ &&
        waypoints.size() >= static_cast<size_t>(bspline_degree_ + 1))
    {
        LOG_NAMED_INFO("cartesian", "使用B样条生成笛卡尔轨迹（{}个控制点，阶数={}）",
                       waypoints.size(), bspline_degree_);

        try
        {
            std::vector<geometry_msgs::msg::Pose> bspline_waypoints =
                trajectory_planner_->generate_bspline_cartesian_trajectory(
                    waypoints, bspline_degree_, bspline_duration_, bspline_dt_);

            if (bspline_waypoints.empty())
            {
                LOG_NAMED_WARN("cartesian", "B样条轨迹生成失败，回退到标准方法");
            }
            else
            {
                LOG_NAMED_INFO("cartesian", "B样条生成 {} 个笛卡尔轨迹点",
                               bspline_waypoints.size());

                moveit_msgs::msg::RobotTrajectory trajectory;
                double fraction = compute_cartesian_path_with_timeout(
                    bspline_waypoints, eef_step_, jump_threshold_, trajectory, timeout, true);

                if (fraction < 0.0)
                {
                    LOG_NAMED_WARN("cartesian", "B样条笛卡尔路径计算超时，回退到标准方法");
                }
                else
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
                                LOG_NAMED_INFO("cartesian", "B样条笛卡尔路径执行成功");
                                return true;
                            }
                        }
                    }
                    LOG_NAMED_WARN("cartesian",
                                   "B样条笛卡尔路径执行失败（成功率: {:.2f}%%，阈值: "
                                   "{:.2f}%%），回退到标准方法",
                                   fraction * 100.0, threshold * 100.0);
                }
            }
        }
        catch (const std::exception& e)
        {
            LOG_NAMED_WARN("cartesian", "B样条轨迹生成失败: {}，回退到标准方法", e.what());
        }
    }

    // 如果配置启用线性插补且waypoints只有2个点
    if (use_linear_interpolation_ && trajectory_planner_ && waypoints.size() == 2)
    {
        LOG_NAMED_INFO("cartesian", "使用精确直线插补生成笛卡尔轨迹");

        double distance = std::sqrt(std::pow(waypoints[1].position.x - waypoints[0].position.x, 2) +
                                    std::pow(waypoints[1].position.y - waypoints[0].position.y, 2) +
                                    std::pow(waypoints[1].position.z - waypoints[0].position.z, 2));

        double duration = std::max(linear_min_duration_, distance / linear_velocity_);

        std::vector<geometry_msgs::msg::Pose> linear_waypoints;
        bool linear_interpolation_success = false;
        try
        {
            linear_waypoints = trajectory_planner_->generate_linear_cartesian_trajectory(
                waypoints[0], waypoints[1], duration, eef_step_);
            linear_interpolation_success = !linear_waypoints.empty();
        }
        catch (const std::exception& e)
        {
            LOG_NAMED_WARN("cartesian", "线性插补生成失败: {}，回退到MoveIt方法", e.what());
            linear_interpolation_success = false;
        }

        if (linear_interpolation_success)
        {
            LOG_NAMED_INFO("cartesian", "使用 {} 个线性插补点进行笛卡尔路径规划",
                           linear_waypoints.size());

            moveit_msgs::msg::RobotTrajectory trajectory;
            double scaled_step = eef_step_ * linear_step_scale_;
            double fraction = compute_cartesian_path_with_timeout(
                linear_waypoints, scaled_step, jump_threshold_, trajectory, timeout, true);

            if (fraction < 0.0)
            {
                LOG_NAMED_WARN("cartesian", "线性插补笛卡尔路径计算超时，回退到标准方法");
            }
            else
            {
                double threshold =
                    use_descend ? cartesian_descend_threshold_ : cartesian_lift_threshold_;
                if (fraction >= threshold)
                {
                    moveit::planning_interface::MoveGroupInterface::Plan plan;
                    plan.trajectory_ = trajectory;

                    {
                        std::lock_guard<std::mutex> lock(moveit_mutex_);
                        moveit::core::MoveItErrorCode result = move_group_interface_->execute(plan);
                        if (result == moveit::core::MoveItErrorCode::SUCCESS)
                        {
                            LOG_NAMED_INFO("cartesian", "线性插补笛卡尔路径执行成功");
                            return true;
                        }
                    }
                }
                LOG_NAMED_WARN("cartesian",
                               "线性插补笛卡尔路径执行失败（成功率: {:.2f}%%，阈值: "
                               "{:.2f}%%），回退到标准方法",
                               fraction * 100.0, threshold * 100.0);
            }
        }
    }

    // 标准方法：使用 MoveIt 的 computeCartesianPath
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = compute_cartesian_path_with_timeout(waypoints, eef_step_, jump_threshold_,
                                                      trajectory, timeout, true);

    if (fraction < 0.0)
    {
        LOG_NAMED_WARN("cartesian", "笛卡尔路径计算超时（{:.1f}秒），触发fallback", timeout);
        return false;
    }

    double threshold = use_descend ? cartesian_descend_threshold_ : cartesian_lift_threshold_;
    if (fraction < threshold)
    {
        LOG_NAMED_WARN("cartesian",
                       "笛卡尔路径规划失败，成功率: {:.2f}%%，要求 >= {:.2f}%%，尝试fallback",
                       fraction * 100.0, threshold * 100.0);
        return false;
    }

    // 执行轨迹
    {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;

        {
            std::lock_guard<std::mutex> lock(moveit_mutex_);
            moveit::core::MoveItErrorCode result = move_group_interface_->execute(plan);
            if (result == moveit::core::MoveItErrorCode::SUCCESS)
            {
                LOG_NAMED_INFO("cartesian", "笛卡尔路径执行成功");
                return true;
            }
        }

        LOG_NAMED_WARN("cartesian", "笛卡尔路径执行失败");
        return false;
    }
}

int CartesianExecutor::calculate_adaptive_segments(const geometry_msgs::msg::PoseStamped& start_pose,
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

    segments = std::max(min_segments_, std::min(max_segments_, segments));

    LOG_NAMED_INFO("cartesian", "[自适应分段] 距离={:.3f} m, 计算分段数={} (范围: {}-{})", distance,
                   segments, min_segments_, max_segments_);

    return segments;
}

bool CartesianExecutor::execute_segmented_descend(const geometry_msgs::msg::PoseStamped& start_pose,
                                                const geometry_msgs::msg::PoseStamped& end_pose,
                                                int num_segments)
{
    if (num_segments < 0)
    {
        num_segments = calculate_adaptive_segments(start_pose, end_pose);
    }

    LOG_NAMED_INFO("cartesian",
                   "[分段下压] 开始分段下压，从 ({:.3f}, {:.3f}, {:.3f}) 到 ({:.3f}, {:.3f}, "
                   "{:.3f})，分段数: {}",
                   start_pose.pose.position.x, start_pose.pose.position.y,
                   start_pose.pose.position.z, end_pose.pose.position.x, end_pose.pose.position.y,
                   end_pose.pose.position.z, num_segments);

    double dx = (end_pose.pose.position.x - start_pose.pose.position.x) / num_segments;
    double dy = (end_pose.pose.position.y - start_pose.pose.position.y) / num_segments;
    double dz = (end_pose.pose.position.z - start_pose.pose.position.z) / num_segments;

    std::vector<geometry_msgs::msg::PoseStamped> waypoints_stamped;
    waypoints_stamped.push_back(start_pose);

    // 从end_pose的orientation中提取yaw
    double target_yaw = 0.0;
    {
        tf2::Quaternion q;
        tf2::fromMsg(end_pose.pose.orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        target_yaw = yaw;
    }

    for (int i = 1; i < num_segments; ++i)
    {
        geometry_msgs::msg::PoseStamped waypoint = start_pose;
        waypoint.pose.position.x += dx * i;
        waypoint.pose.position.y += dy * i;
        waypoint.pose.position.z += dz * i;

        // yaw合法性检查
        double valid_yaw = target_yaw;
        if (yaw_validation_enabled_ && find_valid_yaw_callback_)
        {
            valid_yaw = find_valid_yaw_callback_(waypoint.pose.position.x, waypoint.pose.position.y,
                                                 waypoint.pose.position.z, target_yaw,
                                                 yaw_tolerance_, yaw_search_step_);
            if (std::isnan(valid_yaw))
                valid_yaw = target_yaw;

            if (std::abs(valid_yaw - target_yaw) > 1e-6 && compute_orientation_callback_)
            {
                waypoint.pose.orientation = compute_orientation_callback_(valid_yaw);
                LOG_NAMED_INFO(
                    "cartesian",
                    "[分段下压] waypoint #{} yaw已调整: {:.3f} -> {:.3f} rad ({:.1f} -> {:.1f}°)",
                    i, target_yaw, valid_yaw, target_yaw * 180.0 / M_PI, valid_yaw * 180.0 / M_PI);
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

    // 确保所有waypoints在planning frame中
    for (auto& wp : waypoints_stamped)
    {
        if (wp.header.frame_id != planning_frame_ && transform_callback_)
        {
            if (!transform_callback_(wp))
            {
                LOG_NAMED_ERROR("cartesian", "[分段下压] waypoint转换到planning frame失败");
                return false;
            }
        }
    }

    // 计算平均段距离
    double avg_segment_distance = 0.0;
    if (waypoints_stamped.size() > 1)
    {
        double total_distance =
            std::sqrt(std::pow(end_pose.pose.position.x - start_pose.pose.position.x, 2) +
                      std::pow(end_pose.pose.position.y - start_pose.pose.position.y, 2) +
                      std::pow(end_pose.pose.position.z - start_pose.pose.position.z, 2));
        avg_segment_distance = total_distance / (waypoints_stamped.size() - 1);
    }

    LOG_NAMED_INFO("cartesian",
                   "[分段下压] 开始逐段IK→joint执行（{}个waypoints，平均每段约{:.3f} m）",
                   waypoints_stamped.size(), avg_segment_distance);

    // 逐段执行
    for (size_t i = 1; i < waypoints_stamped.size(); ++i)
    {
        double segment_distance = 0.0;
        if (i > 0)
        {
            double dx_seg =
                waypoints_stamped[i].pose.position.x - waypoints_stamped[i - 1].pose.position.x;
            double dy_seg =
                waypoints_stamped[i].pose.position.y - waypoints_stamped[i - 1].pose.position.y;
            double dz_seg =
                waypoints_stamped[i].pose.position.z - waypoints_stamped[i - 1].pose.position.z;
            segment_distance = std::sqrt(dx_seg * dx_seg + dy_seg * dy_seg + dz_seg * dz_seg);
        }

        LOG_NAMED_INFO("cartesian",
                       "[分段下压] 执行段 {}/{}: 位置=({:.3f}, {:.3f}, {:.3f}), 距离={:.3f} m",
                       i + 1, waypoints_stamped.size(), waypoints_stamped[i].pose.position.x,
                       waypoints_stamped[i].pose.position.y, waypoints_stamped[i].pose.position.z,
                       segment_distance);

        {
            std::lock_guard<std::mutex> lock(moveit_mutex_);
            move_group_interface_->setStartStateToCurrentState();
        }

        if (!plan_execute_callback_ || !plan_execute_callback_(waypoints_stamped[i]))
        {
            LOG_NAMED_WARN("cartesian", "[分段下压] 段 {}/{} 执行失败，尝试回滚到起点", i + 1,
                           waypoints_stamped.size());
            if (enable_recovery_ && recover_callback_)
            {
                recover_callback_(start_pose);
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

    LOG_NAMED_INFO("cartesian", "[分段下压] 所有段执行成功");
    return true;
}

bool CartesianExecutor::execute_segmented_lift(const geometry_msgs::msg::PoseStamped& start_pose,
                                             const geometry_msgs::msg::PoseStamped& end_pose,
                                             int num_segments)
{
    LOG_NAMED_INFO("cartesian",
                   "[分段提升] 开始分段提升，从 ({:.3f}, {:.3f}, {:.3f}) 到 ({:.3f}, {:.3f}, "
                   "{:.3f})，分段数: {}",
                   start_pose.pose.position.x, start_pose.pose.position.y,
                   start_pose.pose.position.z, end_pose.pose.position.x, end_pose.pose.position.y,
                   end_pose.pose.position.z, num_segments);

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

    // 如果启用B样条且waypoints数量足够
    if (use_bspline_ && trajectory_planner_ &&
        waypoints.size() >= static_cast<size_t>(bspline_degree_ + 1))
    {
        LOG_NAMED_INFO("cartesian", "[分段提升] 使用B样条生成平滑轨迹（{}个控制点，阶数={}）",
                       waypoints.size(), bspline_degree_);

        std::vector<std::vector<double>> control_points;
        std::lock_guard<std::mutex> lock(moveit_mutex_);

        for (size_t i = 0; i < waypoints.size(); ++i)
        {
            geometry_msgs::msg::PoseStamped pose_in_planning = waypoints[i];
            if (transform_callback_ && !transform_callback_(pose_in_planning))
            {
                LOG_NAMED_WARN("cartesian", "[分段提升] waypoint #{} 转换失败，fallback到逐段执行",
                               i);
                control_points.clear();
                break;
            }

            auto state = move_group_interface_->getCurrentState(0.0);
            if (!state)
            {
                LOG_NAMED_WARN("cartesian", "[分段提升] 无法获取当前状态");
                control_points.clear();
                break;
            }

            const auto* jmg = state->getJointModelGroup("arm_group");
            if (!jmg)
            {
                LOG_NAMED_WARN("cartesian", "[分段提升] 无法获取arm_group，fallback到逐段执行");
                control_points.clear();
                break;
            }

            if (i > 0)
            {
                state->setJointGroupPositions(jmg, control_points.back());
            }

            if (state->setFromIK(jmg, pose_in_planning.pose, eef_link_, ik_timeout_))
            {
                std::vector<double> joint_values;
                state->copyJointGroupPositions(jmg, joint_values);
                control_points.push_back(joint_values);
            }
            else
            {
                LOG_NAMED_WARN("cartesian",
                               "[分段提升] waypoint #{} IK求解失败，fallback到逐段执行", i);
                control_points.clear();
                break;
            }
        }

        if (control_points.size() == waypoints.size())
        {
            auto state_for_bspline = move_group_interface_->getCurrentState(0.0);
            if (state_for_bspline)
            {
                const auto* jmg_for_bspline = state_for_bspline->getJointModelGroup("arm_group");
                if (jmg_for_bspline)
                {
                    try
                    {
                        std::vector<std::string> joint_names =
                            jmg_for_bspline->getActiveJointModelNames();

                        trajectory_msgs::msg::JointTrajectory trajectory =
                            trajectory_planner_->generate_bspline_joint_trajectory(
                                control_points, bspline_degree_, bspline_duration_, joint_names,
                                bspline_dt_);

                        moveit::planning_interface::MoveGroupInterface::Plan plan;
                        plan.trajectory_.joint_trajectory = trajectory;
                        plan.trajectory_.joint_trajectory.header.stamp = node_->get_clock()->now();
                        plan.trajectory_.joint_trajectory.header.frame_id = planning_frame_;

                        move_group_interface_->setStartStateToCurrentState();
                        moveit::core::MoveItErrorCode result = move_group_interface_->execute(plan);

                        if (result == moveit::core::MoveItErrorCode::SUCCESS)
                        {
                            LOG_NAMED_INFO("cartesian", "[分段提升] B样条轨迹执行成功");
                            return true;
                        }
                        else
                        {
                            LOG_NAMED_WARN(
                                "cartesian",
                                "[分段提升] B样条轨迹执行失败，fallback到逐段执行，错误代码: {}",
                                result.val);
                        }
                    }
                    catch (const std::exception& e)
                    {
                        LOG_NAMED_WARN("cartesian",
                                       "[分段提升] B样条轨迹生成失败: {}，fallback到逐段执行",
                                       e.what());
                    }
                }
            }
        }
    }

    // 逐段执行
    for (size_t i = 1; i < waypoints.size(); ++i)
    {
        LOG_NAMED_INFO("cartesian", "[分段提升] 执行段 {}/{}: 位置=({:.3f}, {:.3f}, {:.3f})", i + 1,
                       waypoints.size(), waypoints[i].pose.position.x, waypoints[i].pose.position.y,
                       waypoints[i].pose.position.z);

        LOG_NAMED_INFO("cartesian", "[分段提升] 段 {}/{} 执行前，等待%.0fms并同步状态...", i + 1,
                       waypoints.size(), state_sync_wait_ * 1000);
        {
            int total_ms = static_cast<int>(state_sync_wait_ * 1000);
            const int chunk_ms = 10;
            for (int t = 0; t < total_ms; t += chunk_ms)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(chunk_ms));
            }
        }

        {
            std::lock_guard<std::mutex> lock(moveit_mutex_);
            move_group_interface_->setStartStateToCurrentState();
        }

        if (!plan_execute_callback_ || !plan_execute_callback_(waypoints[i]))
        {
            LOG_NAMED_WARN("cartesian", "[分段提升] 段 {}/{} 执行失败，尝试回滚到起点", i + 1,
                           waypoints.size());
            if (enable_recovery_ && recover_callback_ && !waypoints.empty())
            {
                recover_callback_(waypoints[0]);
            }
            return false;
        }

        LOG_NAMED_INFO("cartesian", "[分段提升] 段 {}/{} 执行成功，等待状态稳定...", i + 1,
                       waypoints.size());

        bool state_stable = false;
        if (wait_state_callback_)
        {
            state_stable = wait_state_callback_(segment_execution_wait_, 5, 0.002);
        }

        if (!state_stable)
        {
            LOG_NAMED_WARN("cartesian", "[分段提升] 状态稳定判断超时，强制等待最小时间（%.0fms）",
                           segment_execution_wait_ * 1000);
            int total_ms = static_cast<int>(segment_execution_wait_ * 1000);
            const int chunk_ms = 10;
            for (int t = 0; t < total_ms; t += chunk_ms)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(chunk_ms));
            }
        }
        else
        {
            LOG_NAMED_DEBUG("cartesian", "[分段提升] 状态已稳定，继续执行下一段");
        }
    }

    LOG_NAMED_INFO("cartesian", "[分段提升] 所有段执行成功");
    return true;
}

} // namespace sealien_payload_grasp
