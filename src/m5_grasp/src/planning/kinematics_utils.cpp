#include "m5_grasp/planning/kinematics_utils.hpp"
#include "m5_grasp/logging/logger.hpp"
#include "m5_grasp/utils/pose_utils.hpp"
#include <algorithm>
#include <cmath>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene/planning_scene.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace m5_grasp
{

KinematicsUtils::KinematicsUtils(
    rclcpp::Node* node,
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface,
    std::mutex& moveit_mutex)
    : node_(node), move_group_interface_(move_group_interface), moveit_mutex_(moveit_mutex)
{
}

void KinematicsUtils::setParameters(const std::string& eef_link, double ik_timeout, bool is_4dof,
                                    const WorkspaceParameters& workspace_params,
                                    double yaw_tolerance, double yaw_search_step,
                                    bool joint4_yaw_search_enabled, double joint4_yaw_search_range,
                                    double joint4_yaw_search_step)
{
    eef_link_ = eef_link;
    ik_timeout_ = ik_timeout;
    is_4dof_ = is_4dof;
    workspace_params_ = workspace_params;
    yaw_tolerance_ = yaw_tolerance;
    yaw_search_step_ = yaw_search_step;
    joint4_yaw_search_enabled_ = joint4_yaw_search_enabled;
    joint4_yaw_search_range_ = joint4_yaw_search_range;
    joint4_yaw_search_step_ = joint4_yaw_search_step;
}

geometry_msgs::msg::Quaternion KinematicsUtils::compute_downward_orientation()
{
    tf2::Quaternion q;
    q.setRPY(M_PI, 0.0, 0.0);
    geometry_msgs::msg::Quaternion quat;
    quat.x = q.x();
    quat.y = q.y();
    quat.z = q.z();
    quat.w = q.w();
    return quat;
}

geometry_msgs::msg::Quaternion KinematicsUtils::compute_downward_orientation_with_yaw(double yaw)
{
    tf2::Quaternion q;
    q.setRPY(M_PI, 0.0, yaw);
    q.normalize();

    geometry_msgs::msg::Quaternion quat = tf2::toMsg(q);
    return quat;
}

moveit::core::RobotStatePtr KinematicsUtils::get_current_state_safe(double timeout)
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
        LOG_NAMED_WARN("ik", "get_current_state_safe异常: {}", e.what());
    }

    // Fallback
    double actual_timeout = (timeout > 0.0) ? std::max(timeout, 1.0) : 3.0;
    auto state = move_group_interface_->getCurrentState(actual_timeout);
    if (state)
    {
        state->enforceBounds();
    }
    return state;
}

std::vector<double> KinematicsUtils::get_current_joint_values_safe()
{
    std::lock_guard<std::mutex> lock(moveit_mutex_);
    return move_group_interface_->getCurrentJointValues();
}

geometry_msgs::msg::PoseStamped KinematicsUtils::get_current_pose_safe()
{
    std::lock_guard<std::mutex> lock(moveit_mutex_);
    return move_group_interface_->getCurrentPose();
}

bool KinematicsUtils::isReachable(double x, double y, double z)
{
    const double joint2_height = workspace_params_.base_height;
    const double link2_length = workspace_params_.link2_length;
    const double link3_length = workspace_params_.link3_length;
    const double link4_to_eef = workspace_params_.link4_to_eef;

    const double max_reach = link2_length + link3_length + link4_to_eef;
    const double max_reach_with_margin = max_reach * workspace_params_.reach_radius_margin;

    const double max_height = joint2_height + workspace_params_.max_height_offset;
    const double min_height = joint2_height + workspace_params_.min_height_offset;

    const double radius = std::sqrt(x * x + y * y);
    const double height_from_joint2 = z - joint2_height;
    const double distance_from_joint2 =
        std::sqrt(radius * radius + height_from_joint2 * height_from_joint2);

    // 水平半径上限（预判：用 reach_radius_margin 收紧）
    const double max_radius_effective =
        workspace_params_.max_radius * workspace_params_.reach_radius_margin;

    // 检查最小半径
    if (radius < workspace_params_.min_radius)
    {
        LOG_NAMED_WARN("ik",
                       "[工作空间] 目标 ({:.3f}, {:.3f}, {:.3f}) 太靠近base (r={:.3f} < {:.3f} m)",
                       x, y, z, radius, workspace_params_.min_radius);
    }

    // 检查水平半径上限（几何极限 max_radius=0.594m，0.6m/0.7m 不可达）
    if (radius > max_radius_effective)
    {
        LOG_NAMED_ERROR("ik",
                        "[工作空间] 目标 ({:.3f}, {:.3f}, {:.3f}) 超出水平半径 (r={:.3f} > {:.3f} m)",
                        x, y, z, radius, max_radius_effective);
        return false;
    }

    // 检查最大伸展距离
    if (distance_from_joint2 > max_reach_with_margin)
    {
        LOG_NAMED_ERROR(
            "ik", "[工作空间] 目标 ({:.3f}, {:.3f}, {:.3f}) 超出最大伸展距离 (d={:.3f} > {:.3f} m)",
            x, y, z, distance_from_joint2, max_reach_with_margin);
        return false;
    }

    // 检查高度范围
    if (z > max_height)
    {
        LOG_NAMED_ERROR(
            "ik", "[工作空间] 目标 ({:.3f}, {:.3f}, {:.3f}) 超出最大高度 (z={:.3f} > {:.3f} m)", x,
            y, z, z, max_height);
        return false;
    }

    if (z < min_height)
    {
        LOG_NAMED_WARN(
            "ik", "[工作空间] 目标 ({:.3f}, {:.3f}, {:.3f}) 低于最小高度 (z={:.3f} < {:.3f} m)", x,
            y, z, z, min_height);
    }

    return true;
}

void KinematicsUtils::adjustToWorkspace(double& x, double& y, double& z)
{
    const double joint2_height = workspace_params_.base_height;
    const double link2_length = workspace_params_.link2_length;
    const double link3_length = workspace_params_.link3_length;
    const double link4_to_eef = workspace_params_.link4_to_eef;

    const double max_reach = link2_length + link3_length + link4_to_eef;
    const double max_reach_with_margin = max_reach * workspace_params_.reach_radius_margin;

    const double max_height = joint2_height + workspace_params_.max_height_offset;
    const double min_height = joint2_height + workspace_params_.min_height_offset;

    double original_x = x, original_y = y, original_z = z;
    bool adjusted = false;

    double radius = std::sqrt(x * x + y * y);
    const double max_radius_effective =
        workspace_params_.max_radius * workspace_params_.reach_radius_margin;

    // 调整太近base的情况
    if (radius < workspace_params_.min_radius && radius > 0.001)
    {
        double scale = workspace_params_.min_radius / radius;
        x *= scale;
        y *= scale;
        radius = workspace_params_.min_radius;
        adjusted = true;
    }

    // 调整超出水平半径上限（预判）
    if (radius > max_radius_effective && radius > 0.001)
    {
        double scale = max_radius_effective / radius;
        x *= scale;
        y *= scale;
        radius = max_radius_effective;
        adjusted = true;
    }

    // 调整高度
    if (z > max_height)
    {
        z = max_height;
        adjusted = true;
    }
    if (z < min_height)
    {
        z = min_height;
        adjusted = true;
    }

    // 调整超出伸展距离
    double height_from_joint2 = z - joint2_height;
    double distance_from_joint2 =
        std::sqrt(radius * radius + height_from_joint2 * height_from_joint2);

    if (distance_from_joint2 > max_reach_with_margin)
    {
        double scale = max_reach_with_margin / distance_from_joint2;
        radius *= scale;
        height_from_joint2 *= scale;

        if (radius > 0.001)
        {
            double original_radius = std::sqrt(x * x + y * y);
            if (original_radius > 0.001)
            {
                x *= (radius / original_radius);
                y *= (radius / original_radius);
            }
        }
        z = joint2_height + height_from_joint2;
        adjusted = true;
    }

    if (adjusted)
    {
        LOG_NAMED_WARN(
            "ik", "[工作空间] 位置已调整: ({:.3f}, {:.3f}, {:.3f}) -> ({:.3f}, {:.3f}, {:.3f})",
            original_x, original_y, original_z, x, y, z);
    }
}

bool KinematicsUtils::isYawIKValid(double x, double y, double z, double yaw, double& valid_yaw)
{
    auto state = get_current_state_safe(1.0);
    if (!state)
    {
        return false;
    }

    const auto* jmg = state->getJointModelGroup("arm_group");
    if (!jmg)
    {
        return false;
    }

    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation = compute_downward_orientation_with_yaw(yaw);

    bool ok = state->setFromIK(jmg, pose, eef_link_, ik_timeout_);
    if (ok)
    {
        valid_yaw = yaw;
    }
    return ok;
}

double KinematicsUtils::findValidYaw(double x, double y, double z, double original_yaw,
                                     double tolerance, double step)
{
    auto state = get_current_state_safe(1.0);
    if (!state)
    {
        return original_yaw;
    }

    const auto* jmg = state->getJointModelGroup("arm_group");
    if (!jmg)
    {
        return original_yaw;
    }

    // 先尝试原始yaw
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation = compute_downward_orientation_with_yaw(original_yaw);

    if (state->setFromIK(jmg, pose, eef_link_, ik_timeout_))
    {
        LOG_NAMED_INFO("ik", "[yaw搜索] 原始yaw={:.3f} rad ({:.1f}°) 可解", original_yaw,
                       original_yaw * 180.0 / M_PI);
        return original_yaw;
    }

    LOG_NAMED_WARN("ik", "[yaw搜索] 原始yaw={:.3f} rad ({:.1f}°) 不可解，开始搜索...", original_yaw,
                   original_yaw * 180.0 / M_PI);

    int max_steps = static_cast<int>(tolerance / step);
    for (int s = 1; s <= max_steps; ++s)
    {
        // 正向偏移
        double yaw_plus = original_yaw + s * step;
        pose.orientation = compute_downward_orientation_with_yaw(yaw_plus);
        moveit::core::RobotStatePtr test_state(new moveit::core::RobotState(*state));
        if (test_state->setFromIK(jmg, pose, eef_link_, ik_timeout_))
        {
            LOG_NAMED_INFO("ik", "[yaw搜索] 找到可解yaw: {:.3f} rad ({:.1f}°)，偏移: +{:.3f} rad",
                           yaw_plus, yaw_plus * 180.0 / M_PI, s * step);
            return yaw_plus;
        }

        // 负向偏移
        double yaw_minus = original_yaw - s * step;
        pose.orientation = compute_downward_orientation_with_yaw(yaw_minus);
        test_state = std::make_shared<moveit::core::RobotState>(*state);
        if (test_state->setFromIK(jmg, pose, eef_link_, ik_timeout_))
        {
            LOG_NAMED_INFO("ik", "[yaw搜索] 找到可解yaw: {:.3f} rad ({:.1f}°)，偏移: -{:.3f} rad",
                           yaw_minus, yaw_minus * 180.0 / M_PI, s * step);
            return yaw_minus;
        }
    }

    LOG_NAMED_WARN("ik", "[yaw搜索] 在±{:.3f} rad范围内未找到可解yaw", tolerance);
    return original_yaw;
}

double KinematicsUtils::findBestYawCandidate(double x, double y, double z, double reference_yaw,
                                             double search_range, double search_step)
{
    auto state = get_current_state_safe(1.0);
    if (!state)
    {
        return reference_yaw;
    }

    const auto* jmg = state->getJointModelGroup("arm_group");
    if (!jmg)
    {
        return reference_yaw;
    }

    // 生成候选yaw集合
    std::vector<double> candidate_yaws;
    double search_min = std::max(-M_PI, reference_yaw - search_range / 2.0);
    double search_max = std::min(M_PI, reference_yaw + search_range / 2.0);

    int num_candidates = static_cast<int>((search_max - search_min) / search_step) + 1;
    for (int i = 0; i < num_candidates; ++i)
    {
        double candidate = search_min + i * search_step;
        candidate = normalize_angle(candidate);
        candidate_yaws.push_back(candidate);
    }

    // 获取Joint4限位
    const auto* joint4_model = jmg->getJointModel("Joint4");
    double joint4_min = -M_PI, joint4_max = M_PI;
    if (joint4_model)
    {
        const auto& bounds = joint4_model->getVariableBounds();
        if (!bounds.empty())
        {
            joint4_min = bounds[0].min_position_;
            joint4_max = bounds[0].max_position_;
        }
    }

    struct YawCandidate
    {
        double yaw;
        double joint4_dist;
        double yaw_error;
        double score;
    };

    std::vector<YawCandidate> valid_candidates;

    for (double candidate_yaw : candidate_yaws)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        pose.orientation = compute_downward_orientation_with_yaw(candidate_yaw);

        moveit::core::RobotStatePtr test_state(new moveit::core::RobotState(*state));
        if (test_state->setFromIK(jmg, pose, eef_link_, ik_timeout_))
        {
            std::vector<double> joint_values;
            test_state->copyJointGroupPositions(jmg, joint_values);

            if (joint_values.size() >= 4)
            {
                double joint4_value = joint_values[3];
                double dist_to_min = joint4_value - joint4_min;
                double dist_to_max = joint4_max - joint4_value;
                double joint4_dist = std::min(dist_to_min, dist_to_max);
                double yaw_error = std::abs(normalize_angle_diff(candidate_yaw - reference_yaw));

                double normalized_dist = std::min(joint4_dist / M_PI, 1.0);
                double normalized_err = 1.0 - std::min(yaw_error / M_PI, 1.0);
                double score = 0.6 * normalized_dist + 0.4 * normalized_err;

                valid_candidates.push_back({candidate_yaw, joint4_dist, yaw_error, score});
            }
        }
    }

    if (valid_candidates.empty())
    {
        LOG_NAMED_WARN("ik", "[yaw候选搜索] 未找到有效候选");
        return reference_yaw;
    }

    auto best = std::max_element(valid_candidates.begin(), valid_candidates.end(),
                                 [](const YawCandidate& a, const YawCandidate& b)
                                 { return a.score < b.score; });

    LOG_NAMED_INFO("ik",
                   "[yaw候选搜索] 找到{}个有效候选，最优yaw={:.3f} rad ({:.1f}°)，评分={:.3f}",
                   valid_candidates.size(), best->yaw, best->yaw * 180.0 / M_PI, best->score);

    return best->yaw;
}

bool KinematicsUtils::check_joint23_reachable(const moveit::core::RobotStatePtr& state,
                                            const std::string& jmg_name,
                                            std::vector<double>& joint2_3_values)
{
    const auto* jmg = state->getJointModelGroup(jmg_name);
    if (!jmg)
    {
        return false;
    }

    const std::vector<std::string>& joint_names = jmg->getJointModelNames();
    std::vector<double> joint_values;
    state->copyJointGroupPositions(jmg, joint_values);

    int joint2_idx = -1, joint3_idx = -1;
    for (size_t i = 0; i < joint_names.size(); ++i)
    {
        if (joint_names[i] == "Joint2")
            joint2_idx = i;
        else if (joint_names[i] == "Joint3")
            joint3_idx = i;
    }

    if (joint2_idx < 0 || joint3_idx < 0)
    {
        LOG_NAMED_WARN("ik", "[Joint2/3检查] 无法找到Joint2或Joint3");
        return false;
    }

    double joint2_value = joint_values[joint2_idx];
    double joint3_value = joint_values[joint3_idx];
    joint2_3_values = {joint2_value, joint3_value};

    const double lower_limit = -2.97 - 0.01;
    const double upper_limit = 0.0 + 0.01;

    bool joint2_ok = (joint2_value >= lower_limit && joint2_value <= upper_limit);
    bool joint3_ok = (joint3_value >= lower_limit && joint3_value <= upper_limit);

    if (!joint2_ok || !joint3_ok)
    {
        LOG_NAMED_WARN("ik", "[Joint2/3检查] 不可达: Joint2={:.3f} rad, Joint3={:.3f} rad",
                       joint2_value, joint3_value);
        return false;
    }

    return true;
}

YawIKResult KinematicsUtils::try_yaw_candidates_for_ik(const geometry_msgs::msg::Pose& target_pose,
                                                   const std::vector<double>& yaw_candidates,
                                                   const std::string& eef_link, double ik_timeout)
{
    YawIKResult result;
    result.success = false;
    result.selected_yaw = yaw_candidates.empty() ? 0.0 : yaw_candidates[0];

    auto current_state = get_current_state_safe(3.0);
    if (!current_state)
    {
        result.reason = "无法获取当前状态";
        return result;
    }

    const auto* jmg = current_state->getJointModelGroup("arm_group");
    if (!jmg)
    {
        result.reason = "无法获取arm_group";
        return result;
    }

    for (size_t i = 0; i < yaw_candidates.size(); ++i)
    {
        double candidate_yaw = yaw_candidates[i];

        geometry_msgs::msg::Pose pose_for_ik = target_pose;
        pose_for_ik.orientation = compute_downward_orientation_with_yaw(candidate_yaw);

        auto test_state = std::make_shared<moveit::core::RobotState>(*current_state);

        bool ik_solved = false;
        for (int ik_attempt = 0; ik_attempt < 5; ++ik_attempt)
        {
            if (ik_attempt > 0)
            {
                test_state->setToRandomPositions(jmg);
            }

            if (test_state->setFromIK(jmg, pose_for_ik, eef_link, ik_timeout))
            {
                ik_solved = true;
                break;
            }
        }

        if (!ik_solved)
        {
            continue;
        }

        // 检查Joint2/3可达性
        std::vector<double> joint2_3_values;
        if (check_joint23_reachable(test_state, "arm_group", joint2_3_values))
        {
            LOG_NAMED_INFO("ik", "[YawIK] 候选 #{} (yaw={:.3f}) 可达", i + 1, candidate_yaw);

            result.success = true;
            result.selected_yaw = candidate_yaw;
            result.joint2_3_values = joint2_3_values;
            result.successful_state = test_state;
            return result;
        }
    }

    result.reason = "所有yaw候选都不可达";
    return result;
}

bool KinematicsUtils::findOptimalYawForJoint4(double target_yaw, double x, double y, double z,
                                              double& best_yaw, double& best_joint4,
                                              moveit::core::RobotStatePtr& best_state)
{
    auto state = get_current_state_safe(1.0);
    if (!state)
    {
        return false;
    }

    const auto* jmg = state->getJointModelGroup("arm_group");
    if (!jmg)
    {
        return false;
    }

    // 先尝试原始yaw
    geometry_msgs::msg::Pose initial_pose;
    initial_pose.position.x = x;
    initial_pose.position.y = y;
    initial_pose.position.z = z;
    initial_pose.orientation = compute_downward_orientation_with_yaw(target_yaw);

    moveit::core::RobotStatePtr test_state(new moveit::core::RobotState(*state));
    if (!test_state->setFromIK(jmg, initial_pose, eef_link_, ik_timeout_))
    {
        LOG_NAMED_WARN(
            "ik", "[Joint4-yaw搜索] 初始yaw={:.3f} rad 在位置({:.3f}, {:.3f}, {:.3f})下IK不可解",
            target_yaw, x, y, z);
        return false;
    }

    std::vector<double> initial_joint_values;
    test_state->copyJointGroupPositions(jmg, initial_joint_values);
    if (initial_joint_values.size() < 4)
    {
        return false;
    }

    double initial_joint4 = initial_joint_values[3];
    double initial_diff = std::abs(normalize_angle_diff(initial_joint4 - target_yaw));

    best_yaw = target_yaw;
    best_joint4 = initial_joint4;
    best_state = test_state;
    double best_diff = initial_diff;

    // 搜索更优解
    double search_range = std::min(joint4_yaw_search_range_, M_PI);
    double search_min = std::max(-M_PI, target_yaw - search_range);
    double search_max = std::min(M_PI, target_yaw + search_range);

    int search_steps = static_cast<int>((search_max - search_min) / joint4_yaw_search_step_);
    if (search_steps < 1)
        search_steps = 1;

    for (int i = 0; i <= search_steps; ++i)
    {
        double candidate_yaw = search_min + i * joint4_yaw_search_step_;
        candidate_yaw = normalize_angle(candidate_yaw);

        geometry_msgs::msg::Pose candidate_pose;
        candidate_pose.position.x = x;
        candidate_pose.position.y = y;
        candidate_pose.position.z = z;
        candidate_pose.orientation = compute_downward_orientation_with_yaw(candidate_yaw);

        moveit::core::RobotStatePtr candidate_state(new moveit::core::RobotState(*state));
        if (!candidate_state->setFromIK(jmg, candidate_pose, eef_link_, ik_timeout_))
        {
            continue;
        }

        std::vector<double> candidate_joint_values;
        candidate_state->copyJointGroupPositions(jmg, candidate_joint_values);
        if (candidate_joint_values.size() < 4)
        {
            continue;
        }

        double candidate_joint4 = candidate_joint_values[3];
        double candidate_diff = std::abs(normalize_angle_diff(candidate_joint4 - target_yaw));

        if (candidate_diff < best_diff)
        {
            best_diff = candidate_diff;
            best_yaw = candidate_yaw;
            best_joint4 = candidate_joint4;
            best_state = candidate_state;
        }
    }

    // 判断是否找到更优解（改善>5度）
    const double improvement_threshold = 0.0873;
    if (best_diff < initial_diff - improvement_threshold)
    {
        LOG_NAMED_INFO("ik", "[Joint4-yaw搜索] 找到更优解，差值改善: {:.3f} -> {:.3f} rad",
                       initial_diff, best_diff);
        return true;
    }
    else
    {
        best_yaw = target_yaw;
        best_joint4 = initial_joint4;
        return false;
    }
}

bool KinematicsUtils::checkIKOnly(const geometry_msgs::msg::PoseStamped& pose_in_planning)
{
    auto state = get_current_state_safe(1.0);
    if (!state)
    {
        LOG_NAMED_WARN("ik", "[IK检查] 无法获取当前状态");
        return false;
    }

    const auto* jmg = state->getJointModelGroup("arm_group");
    if (!jmg)
    {
        LOG_NAMED_WARN("ik", "[IK检查] 无法获取arm_group");
        return false;
    }

    bool ok = state->setFromIK(jmg, pose_in_planning.pose, eef_link_, ik_timeout_);

    if (ok)
    {
        std::vector<double> joint_values;
        state->copyJointGroupPositions(jmg, joint_values);
        LOG_NAMED_INFO("ik", "[IK检查] 成功，关节值: [{:.3f}, {:.3f}, {:.3f}, {:.3f}]",
                       joint_values.size() > 0 ? joint_values[0] : 0.0,
                       joint_values.size() > 1 ? joint_values[1] : 0.0,
                       joint_values.size() > 2 ? joint_values[2] : 0.0,
                       joint_values.size() > 3 ? joint_values[3] : 0.0);
    }
    else
    {
        LOG_NAMED_WARN("ik", "[IK检查] 失败");
    }

    return ok;
}

bool KinematicsUtils::checkIKWithCollision(const geometry_msgs::msg::PoseStamped& pose_in_planning)
{
    planning_scene::PlanningScenePtr scene =
        std::make_shared<planning_scene::PlanningScene>(move_group_interface_->getRobotModel());

    auto current_state = get_current_state_safe(1.0);
    if (current_state)
    {
        scene->setCurrentState(*current_state);
    }
    else
    {
        scene->getCurrentStateNonConst().setToDefaultValues();
    }

    moveit::core::RobotState& state = scene->getCurrentStateNonConst();
    const auto* jmg = state.getJointModelGroup("arm_group");
    if (!jmg)
    {
        return false;
    }

    // 定义碰撞检查回调函数
    moveit::core::GroupStateValidityCallbackFn validity_callback =
        [&scene](moveit::core::RobotState* st, const moveit::core::JointModelGroup*,
                 const double*) -> bool
    {
        collision_detection::CollisionRequest req;
        collision_detection::CollisionResult res;
        scene->checkSelfCollision(req, res, *st);
        return !res.collision;
    };

    bool found =
        state.setFromIK(jmg, pose_in_planning.pose, eef_link_, ik_timeout_, validity_callback);

    return found;
}

} // namespace m5_grasp
