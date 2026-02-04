#include "m5_grasp/mtc/task_factory.hpp"
#include "m5_grasp/logging/logger.hpp"
#include "m5_grasp/mtc/task_context.hpp"

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <set>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

namespace m5_grasp
{

namespace
{
// 进程内唯一递增后缀，确保 MTC 规划 node 名称唯一，减少 "Publisher already registered"
std::atomic<int> g_mtc_planning_node_counter{0};
} // namespace

TaskFactory::TaskFactory(const TaskContext& ctx) : ctx_(ctx)
{
    // 提前创建专用 MTC 规划 node，名称带唯一后缀，减少同名 node/logger 导致的 "Publisher already
    // registered"
    if (ctx_.node)
    {
        int id = g_mtc_planning_node_counter++;
        std::string name =
            std::string(ctx_.node->get_name()) + "_mtc_planning_" + std::to_string(id);
        mtc_planning_node_ = rclcpp::Node::make_shared(name);
        LOG_NAMED_INFO("task_factory", "TaskFactory: 已创建专用 MTC 规划 node: {}", name);
    }
}

std::shared_ptr<mtc::solvers::PipelinePlanner> TaskFactory::create_ompl_planner()
{
    // 使用构造时已创建的专用 node（名称唯一）调用 PipelinePlanner::create，避免与主节点同名导致
    // "Publisher already registered"
    if (!cached_ompl_planner_)
    {
        if (!mtc_planning_node_)
        {
            LOG_NAMED_ERROR("task_factory",
                            "mtc_planning_node_ 未初始化，无法创建 PipelinePlanner");
            return nullptr;
        }
        if (!ctx_.move_group)
        {
            LOG_NAMED_ERROR("task_factory", "ctx_.move_group 为空，无法获取 robot_model");
            return nullptr;
        }
        auto robot_model = ctx_.move_group->getRobotModel();
        if (!robot_model)
        {
            LOG_NAMED_ERROR("task_factory", "robot_model 为空，无法创建 PlanningPipeline");
            return nullptr;
        }
        try
        {
            LOG_NAMED_INFO("task_factory", "创建 PlanningPipeline（一次性，不持 node）...");
            auto pipeline = mtc::solvers::PipelinePlanner::create(mtc_planning_node_, robot_model);
            if (!pipeline)
            {
                LOG_NAMED_ERROR("task_factory", "PipelinePlanner::create 返回空");
                return nullptr;
            }
            LOG_NAMED_INFO("task_factory",
                           "创建 PipelinePlanner: 使用已有 PlanningPipeline（无 node 持有）");
            cached_ompl_planner_ = std::make_shared<mtc::solvers::PipelinePlanner>(pipeline);
            cached_ompl_planner_->setPlannerId("RRTConnect");
            LOG_NAMED_DEBUG("task_factory", "创建并缓存 OMPL PipelinePlanner");
        }
        catch (const std::exception& e)
        {
            LOG_NAMED_ERROR("task_factory", "PipelinePlanner 创建失败: {}", e.what());
            cached_ompl_planner_.reset();
            return nullptr;
        }
        catch (...)
        {
            LOG_NAMED_ERROR("task_factory", "PipelinePlanner 创建失败: 未知异常");
            cached_ompl_planner_.reset();
            return nullptr;
        }
    }
    return cached_ompl_planner_;
}

std::shared_ptr<mtc::solvers::CartesianPath> TaskFactory::createCartesianPlanner()
{
    // 关键修复：缓存规划器，避免重复创建
    if (!cached_cartesian_planner_)
    {
        cached_cartesian_planner_ = std::make_shared<mtc::solvers::CartesianPath>();
        cached_cartesian_planner_->setMaxVelocityScalingFactor(ctx_.cartesian_velocity_scaling);
        cached_cartesian_planner_->setMaxAccelerationScalingFactor(
            ctx_.cartesian_acceleration_scaling);
        cached_cartesian_planner_->setStepSize(ctx_.cartesian_step_size);
    }
    return cached_cartesian_planner_;
}

void TaskFactory::add_current_state_stage(mtc::Task& task)
{
    if (!ctx_.move_group)
    {
        LOG_NAMED_ERROR("task_factory", "无法添加当前状态stage: move_group 无效");
        return;
    }
    const auto& robot_model = ctx_.move_group->getRobotModel();
    if (!robot_model)
    {
        LOG_NAMED_ERROR("task_factory", "无法添加当前状态stage: robot_model 无效");
        return;
    }
    // 与官方 MoveIt MTC 教程一致：首 stage 用 CurrentState，不在本节点调用 getCurrentJointValues。
    // 任务在 move_group 执行时，CurrentState 从 planning scene 取当前状态；
    // planning scene 由 /joint_states 更新（机械臂反馈 → parse_feedback_json →
    // joint_state_broadcaster → /joint_states）。
    // 见：https://moveit.picknik.ai/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html
    auto current_state = std::make_unique<mtc::stages::CurrentState>("current state");
    task.add(std::move(current_state));
}

void TaskFactory::add_start_state_stage_for_cartesian(mtc::Task& task)
{
    if (!ctx_.move_group)
    {
        add_current_state_stage(task);
        return;
    }
    const auto& robot_model = ctx_.move_group->getRobotModel();
    if (!robot_model)
    {
        add_current_state_stage(task);
        return;
    }
    // 若 joint_state_diag 可用且能填满 arm，用 FixedState 注入起始状态，避免 task.plan() 在
    // CurrentState 上无限等待 planning scene。 新鲜度 15s：与 cleanupMoveGroupState
    // 一致。OPEN_GRIPPER 可等 8s，last_js_wall 会超 5s，若用 5s 会退化为 CurrentState(planning
    // scene)， 全零时 planning scene
    // 可能仍是上次任务终点，导致轨迹“先到错误起点再动”，出现“先到一个位置再回 0 再执行”的现象。
    constexpr double kDiagMaxAgeSec = 15.0;
    if (ctx_.joint_state_diag && ctx_.node)
    {
        const auto* arm_jmg = robot_model->getJointModelGroup(ctx_.arm_group);
        if (arm_jmg)
        {
            rclcpp::Time now = ctx_.node->get_clock()->now();
            std::vector<double> arm_vals;
            if (fill_joint_positions_from_diag(ctx_.joint_state_diag,
                                           arm_jmg->getActiveJointModelNames(), arm_vals, now,
                                           kDiagMaxAgeSec))
            {
                moveit::core::RobotState state(robot_model);
                state.setToDefaultValues();
                state.setJointGroupPositions(arm_jmg, arm_vals);
                const auto* gripper_jmg = robot_model->getJointModelGroup(ctx_.gripper_group);
                if (gripper_jmg && ctx_.gripper_group_interface)
                {
                    std::vector<double> grip_vals;
                    if (fill_joint_positions_from_diag(ctx_.joint_state_diag,
                                                   gripper_jmg->getActiveJointModelNames(),
                                                   grip_vals, now, kDiagMaxAgeSec))
                        state.setJointGroupPositions(gripper_jmg, grip_vals);
                }
                state.update();
                state.enforceBounds();
                auto scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
                scene->setCurrentState(state);
                auto fixed = std::make_unique<mtc::stages::FixedState>("current state");
                fixed->setState(scene);
                task.add(std::move(fixed));
                LOG_NAMED_INFO("task_factory",
                               "使用 FixedState（joint_state_diag），避免 planning scene 阻塞");
                return;
            }
        }
    }
    add_current_state_stage(task);
}

bool TaskFactory::add_start_state_from_arm_joint_map(mtc::Task& task,
                                               const std::map<std::string, double>& arm_joint_map)
{
    if (!ctx_.move_group || arm_joint_map.empty())
    {
        return false;
    }
    const auto& robot_model = ctx_.move_group->getRobotModel();
    if (!robot_model)
    {
        return false;
    }
    const auto* arm_jmg = robot_model->getJointModelGroup(ctx_.arm_group);
    if (!arm_jmg)
    {
        return false;
    }
    const auto& jnames = arm_jmg->getActiveJointModelNames();
    std::vector<double> arm_vals;
    arm_vals.reserve(jnames.size());
    for (const auto& name : jnames)
    {
        auto it = arm_joint_map.find(name);
        if (it == arm_joint_map.end())
        {
            LOG_NAMED_DEBUG("task_factory", "add_start_state_from_arm_joint_map: 缺少关节 {}，退回 diag",
                            name);
            return false;
        }
        arm_vals.push_back(it->second);
    }
    moveit::core::RobotState state(robot_model);
    state.setToDefaultValues();
    state.setJointGroupPositions(arm_jmg, arm_vals);
    const auto* gripper_jmg = robot_model->getJointModelGroup(ctx_.gripper_group);
    if (gripper_jmg && ctx_.joint_state_diag && ctx_.node)
    {
        std::vector<double> grip_vals;
        rclcpp::Time now = ctx_.node->get_clock()->now();
        if (fill_joint_positions_from_diag(ctx_.joint_state_diag,
                                       gripper_jmg->getActiveJointModelNames(), grip_vals, now,
                                       15.0))
        {
            state.setJointGroupPositions(gripper_jmg, grip_vals);
        }
    }
    state.update();
    state.enforceBounds();
    auto scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
    scene->setCurrentState(state);
    auto fixed = std::make_unique<mtc::stages::FixedState>("current state");
    fixed->setState(scene);
    task.add(std::move(fixed));
    LOG_NAMED_INFO("task_factory", "使用 FixedState（上一阶段关节目标），避免 Joint4 等阶段间跳动");
    return true;
}

std::unique_ptr<mtc::stages::MoveTo>
TaskFactory::make_move_to_pose(const std::string& name, const geometry_msgs::msg::PoseStamped& pose,
                            const std::shared_ptr<mtc::solvers::PlannerInterface>& planner)
{
    auto move_to = std::make_unique<mtc::stages::MoveTo>(name, planner);
    move_to->setGroup(ctx_.arm_group);
    move_to->setIKFrame(ctx_.eef_link);
    move_to->setGoal(pose);
    // 超时 60s：范围执行时间 Joint1=30s，加规划余量（见 trajectory.range_execution_time）
    move_to->setTimeout(60.0);
    return move_to;
}

std::unique_ptr<mtc::stages::MoveTo>
TaskFactory::make_move_to_joints(const std::string& name,
                              const std::map<std::string, double>& joint_map,
                              const std::shared_ptr<mtc::solvers::PlannerInterface>& planner)
{
    auto move_to = std::make_unique<mtc::stages::MoveTo>(name, planner);
    move_to->setGroup(ctx_.arm_group);
    move_to->setGoal(joint_map);
    // 超时 60s：范围执行时间 Joint1=30s，加规划余量（见 trajectory.range_execution_time）
    move_to->setTimeout(60.0);
    return move_to;
}

std::unique_ptr<mtc::stages::MoveRelative>
TaskFactory::makeMoveRelativeZ(const std::string& name, double distance,
                               const std::shared_ptr<mtc::solvers::CartesianPath>& planner)
{
    auto move_rel = std::make_unique<mtc::stages::MoveRelative>(name, planner);
    move_rel->setGroup(ctx_.arm_group);
    move_rel->setIKFrame(ctx_.eef_link);

    geometry_msgs::msg::Vector3Stamped direction;
    direction.header.frame_id = ctx_.eef_link; // 工具系
    direction.vector.z = distance;
    move_rel->setDirection(direction);

    double abs_dist = std::abs(distance);
    // 修复A：放宽距离范围，允许更灵活的下探路径
    // 从 0.8-1.2 改为 0.5-1.5，给规划器更多灵活性
    move_rel->setMinMaxDistance(abs_dist * 0.5, abs_dist * 1.5);

    // 修复A：设置超时时间，避免规划器卡住
    move_rel->setTimeout(30.0);

    return move_rel;
}

mtc::Task TaskFactory::make_pregrasp(const TaskTarget& target)
{
    LOG_NAMED_INFO("task_factory", "[make_pregrasp] 构建预抓取任务");

    mtc::Task task;
    // 关键修复：必须使用共享的 robot model，禁止 loadRobotModel
    // loadRobotModel() 会创建新的 RobotModelLoader，导致重复初始化并可能创建新 node
    // 这会导致 "Publisher already registered" 警告和回调问题
    if (!ctx_.move_group)
    {
        LOG_NAMED_ERROR("task_factory", "[make_pregrasp] move_group 无效，无法构建任务");
        return task; // 返回空任务
    }

    const auto& robot_model = ctx_.move_group->getRobotModel();
    if (!robot_model)
    {
        LOG_NAMED_ERROR("task_factory", "[make_pregrasp] 无法获取 robot model，无法构建任务");
        return task; // 返回空任务
    }
    // 说明：task.setRobotModel() 或后续 task.init() 可能触发 MTC 内部创建与主节点同名的
    // logger，产生 "Publisher already registered" 警告，非致命
    task.setRobotModel(robot_model);
    LOG_NAMED_INFO("task_factory", "[make_pregrasp] setRobotModel 完成");
    task.stages()->setName("pregrasp");
    LOG_NAMED_INFO("task_factory", "[make_pregrasp] stages()->setName 完成");
    // 关键修复：禁用 introspection 避免创建新的 publisher，导致 "Publisher already registered" 警告
    task.enableIntrospection(false);
    LOG_NAMED_INFO("task_factory", "[make_pregrasp] enableIntrospection(false) 完成");

    // 关节目标用 JointInterpolationPlanner，不创建 PlanningPipeline，避免 "Publisher already
    // registered" / SIGABRT
    std::shared_ptr<mtc::solvers::PlannerInterface> planner;
    if (target.use_joint_goal && !target.pregrasp_joint_map.empty())
    {
        if (!cached_joint_planner_)
        {
            cached_joint_planner_ = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
            LOG_NAMED_INFO("task_factory",
                           "[make_pregrasp] 使用 JointInterpolationPlanner（关节目标，无 node）");
        }
        planner = cached_joint_planner_;
    }
    else
    {
        planner = create_ompl_planner();
        if (!planner)
        {
            LOG_NAMED_ERROR("task_factory",
                            "[make_pregrasp] PipelinePlanner 创建失败，无法构建任务");
            return task;
        }
    }

    // 1. 添加起始状态：优先 FixedState（diag），避免 MTC CurrentState 从 planning scene
    // 取到无效状态导致 OMPL "Unable to sample any valid states for goal tree"
    LOG_NAMED_INFO("task_factory",
                   "[make_pregrasp] 即将 add_start_state_stage_for_cartesian（diag 可用则 FixedState）");
    add_start_state_stage_for_cartesian(task);
    LOG_NAMED_INFO("task_factory", "[make_pregrasp] add_start_state_stage_for_cartesian 返回");

    // 2. 移动到预抓取位置
    // 注意：碰撞对象在pregrasp之后添加，避免goal state被判碰撞
    LOG_NAMED_INFO("task_factory", "[make_pregrasp] 即将 task.add(move to pregrasp)");
    if (target.use_joint_goal && !target.pregrasp_joint_map.empty())
    {
        task.add(make_move_to_joints("move to pregrasp (joint)", target.pregrasp_joint_map, planner));
        LOG_NAMED_INFO("task_factory", "[make_pregrasp] 使用关节目标模式");
    }
    else
    {
        task.add(make_move_to_pose("move to pregrasp (pose)", target.pregrasp_pose, planner));
        LOG_NAMED_INFO("task_factory", "[make_pregrasp] 使用位姿目标模式");
    }
    LOG_NAMED_INFO("task_factory", "[make_pregrasp] task.add(move to pregrasp) 返回");

    // 3. 添加碰撞对象（在pregrasp之后，避免goal state被判碰撞）
    LOG_NAMED_INFO("task_factory", "[make_pregrasp] 即将 task.add(make_add_collision_object)");
    task.add(make_add_collision_object(target));
    LOG_NAMED_INFO("task_factory", "[make_pregrasp] task.add(make_add_collision_object) 返回");

    // 4. 设置ACM允许夹爪与缆绳碰撞（对于后续的descend/close阶段至关重要）
    LOG_NAMED_INFO("task_factory", "[make_pregrasp] 即将 task.add(make_allow_collisions)");
    task.add(make_allow_collisions());
    LOG_NAMED_INFO("task_factory",
                   "[make_pregrasp] task.add(make_allow_collisions) 返回，make_pregrasp 完成");

    return task;
}

mtc::Task TaskFactory::make_align(const TaskTarget& target)
{
    LOG_NAMED_INFO("task_factory", "[makeAlign] 构建对齐任务, Joint4目标={:.1f}°",
                   target.joint4_target * 180.0 / M_PI);

    mtc::Task task;
    // 关键修复：必须使用共享的 robot model，禁止 loadRobotModel
    if (!ctx_.move_group)
    {
        LOG_NAMED_ERROR("task_factory", "[makeAlign] move_group 无效，无法构建任务");
        return task;
    }

    const auto& robot_model = ctx_.move_group->getRobotModel();
    if (!robot_model)
    {
        LOG_NAMED_ERROR("task_factory", "[makeAlign] 无法获取 robot model，无法构建任务");
        return task;
    }

    task.setRobotModel(robot_model);
    task.stages()->setName("align");
    // 关键修复：禁用 introspection 避免创建新的 publisher
    task.enableIntrospection(false);

    // align 仅设 Joint4，本质为单关节目标，与 pregrasp/descend 一致使用
    // JointInterpolationPlanner，避免 CHOMP 碰撞/无运动轨迹问题
    std::shared_ptr<mtc::solvers::PlannerInterface> planner;
    if (!cached_joint_planner_)
    {
        cached_joint_planner_ = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
        LOG_NAMED_INFO("task_factory",
                       "[makeAlign] 使用 JointInterpolationPlanner（关节目标，无 node）");
    }
    planner = cached_joint_planner_;

    // 1. 起始状态：优先用预抓取关节目标（pregrasp_joint_map），使 Joint4 与上一阶段一致，避免
    // joint_state_diag 滞后导致 Joint4 先动再回
    if (!add_start_state_from_arm_joint_map(task, target.pregrasp_joint_map))
    {
        add_start_state_stage_for_cartesian(task);
    }

    // 2. ACM 允许臂与缆绳碰撞（与 make_descend 一致），避免 CHOMP 在仅动 Joint4 时判臂-缆绳碰撞失败
    std::string cable_object_id = ctx_.cable_name + "_world";
    const auto* arm_jmg_align = robot_model->getJointModelGroup(ctx_.arm_group);
    std::vector<std::string> arm_link_names;
    if (arm_jmg_align)
    {
        arm_link_names = arm_jmg_align->getLinkModelNames();
    }
    if (arm_link_names.empty())
    {
        LOG_NAMED_WARN(
            "task_factory",
            "[makeAlign] arm_group '{}' 未找到或无 link，尝试备用：用非夹爪 link 设置臂-缆绳 ACM",
            ctx_.arm_group);
        const std::vector<std::string>& all_names = robot_model->getLinkModelNames();
        std::set<std::string> gripper_set(ctx_.allow_touch_links.begin(),
                                          ctx_.allow_touch_links.end());
        for (const std::string& link_name : all_names)
        {
            if (gripper_set.count(link_name))
                continue;
            arm_link_names.push_back(link_name);
        }
        if (!arm_link_names.empty())
        {
            LOG_NAMED_INFO("task_factory", "[makeAlign] 备用: 为 {} 个非夹爪 link 设置臂-缆绳 ACM",
                           arm_link_names.size());
        }
    }
    if (!arm_link_names.empty())
    {
        auto allow_arm = std::make_unique<mtc::stages::ModifyPlanningScene>(
            "allow arm-cable collisions (align)");
        for (const std::string& link_name : arm_link_names)
        {
            allow_arm->allowCollisions(cable_object_id, link_name, true);
        }
        task.add(std::move(allow_arm));
        LOG_NAMED_INFO("task_factory",
                       "[makeAlign] 已设置ACM允许 arm_group 与缆绳碰撞（对齐），{} 个 link",
                       static_cast<int>(arm_link_names.size()));
    }

    // 3. align 目标只设 Joint4，其余关节由起始状态提供
    std::map<std::string, double> align_joint_map;
    const auto* arm_jmg = robot_model->getJointModelGroup(ctx_.arm_group);
    if (arm_jmg)
    {
        const auto& joint_names = arm_jmg->getActiveJointModelNames();
        if (joint_names.size() >= 4)
        {
            align_joint_map[joint_names[3]] = target.joint4_target;
        }
        else
        {
            for (const auto& n : joint_names)
            {
                if (n.find("4") != std::string::npos || n.find("Joint4") != std::string::npos)
                {
                    align_joint_map[n] = target.joint4_target;
                    break;
                }
            }
        }
        LOG_NAMED_INFO("task_factory", "[makeAlign] 仅设 {}={:.1f}°，起始由 CurrentState 提供",
                       align_joint_map.empty() ? "Joint4" : align_joint_map.begin()->first.c_str(),
                       target.joint4_target * 180.0 / M_PI);
    }
    task.add(make_move_to_joints("align joint4", align_joint_map, planner));

    return task;
}

mtc::Task TaskFactory::make_descend(const TaskTarget& target)
{
    LOG_NAMED_INFO("task_factory", "[make_descend] 构建下探任务（优先关节目标，否则 OMPL 位姿）");

    mtc::Task task;
    if (!ctx_.move_group)
    {
        LOG_NAMED_ERROR("task_factory", "[make_descend] move_group 无效，无法构建任务");
        return task;
    }

    const auto& robot_model = ctx_.move_group->getRobotModel();
    if (!robot_model)
    {
        LOG_NAMED_ERROR("task_factory", "[make_descend] 无法获取 robot model，无法构建任务");
        return task;
    }

    task.setRobotModel(robot_model);
    task.stages()->setName("descend");
    task.enableIntrospection(false);

    std::shared_ptr<mtc::solvers::PlannerInterface> planner;
    if (!target.grasp_joint_map.empty())
    {
        if (!cached_joint_planner_)
        {
            cached_joint_planner_ = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
        }
        planner = cached_joint_planner_;
        LOG_NAMED_INFO("task_factory", "[make_descend] 使用关节目标 + JointInterpolationPlanner");
    }
    else
    {
        planner = create_ompl_planner();
        if (!planner)
        {
            LOG_NAMED_ERROR("task_factory", "[make_descend] PipelinePlanner 创建失败，无法构建任务");
            return task;
        }
        LOG_NAMED_INFO("task_factory", "[make_descend] grasp_joint_map 为空，使用 OMPL 位姿目标");
    }

    // 1. 起始状态：优先用预抓取关节目标（与 align 终点一致），避免 joint_state_diag 滞后导致 Joint4
    // 等关节阶段间跳动
    if (!add_start_state_from_arm_joint_map(task, target.pregrasp_joint_map))
    {
        add_start_state_stage_for_cartesian(task);
    }

    // 2. ACM 允许夹爪与缆绳碰撞
    task.add(make_allow_collisions());
    LOG_NAMED_INFO("task_factory", "[make_descend] 已设置ACM允许夹爪与缆绳碰撞");

    // 2b. 下探时臂杆会靠近缆绳，需允许 arm_group 所有 link 与缆绳碰撞，否则
    // JointInterpolationPlanner 会因碰撞判失败
    std::string cable_object_id = ctx_.cable_name + "_world";
    const auto* arm_jmg = robot_model->getJointModelGroup(ctx_.arm_group);
    std::vector<std::string> arm_link_names;
    if (arm_jmg)
    {
        arm_link_names = arm_jmg->getLinkModelNames();
    }
    if (arm_link_names.empty())
    {
        LOG_NAMED_WARN(
            "task_factory",
            "[make_descend] arm_group '{}' 未找到或无 link，尝试备用：用非夹爪 link 设置臂-缆绳 ACM",
            ctx_.arm_group);
        const std::vector<std::string>& all_names = robot_model->getLinkModelNames();
        std::set<std::string> gripper_set(ctx_.allow_touch_links.begin(),
                                          ctx_.allow_touch_links.end());
        for (const std::string& link_name : all_names)
        {
            if (gripper_set.count(link_name))
                continue;
            arm_link_names.push_back(link_name);
        }
        if (arm_link_names.empty())
        {
            LOG_NAMED_ERROR("task_factory",
                            "[make_descend] 备用也未找到任何 link，下探可能因碰撞失败");
        }
        else
        {
            LOG_NAMED_INFO("task_factory",
                           "[make_descend] 备用: 为 {} 个非夹爪 link 设置臂-缆绳 ACM",
                           arm_link_names.size());
        }
    }
    if (!arm_link_names.empty())
    {
        auto allow_arm = std::make_unique<mtc::stages::ModifyPlanningScene>(
            "allow arm-cable collisions (descend)");
        for (const std::string& link_name : arm_link_names)
        {
            allow_arm->allowCollisions(cable_object_id, link_name, true);
        }
        task.add(std::move(allow_arm));
        LOG_NAMED_INFO("task_factory",
                       "[make_descend] 已设置ACM允许 arm_group 与缆绳碰撞（下探），{} 个 link",
                       static_cast<int>(arm_link_names.size()));
    }

    // 3. 移到抓取：有关节解则用关节目标，否则用位姿目标
    if (!target.grasp_joint_map.empty())
    {
        task.add(make_move_to_joints("move to grasp", target.grasp_joint_map, planner));
        LOG_NAMED_INFO("task_factory", "[make_descend] 下探目标: 关节目标 ({} 个关节)",
                       target.grasp_joint_map.size());
    }
    else
    {
        task.add(make_move_to_pose("move to grasp", target.grasp_pose, planner));
        LOG_NAMED_INFO("task_factory",
                       "[make_descend] 下探目标: grasp_pose=({:.3f}, {:.3f}, {:.3f})",
                       target.grasp_pose.pose.position.x, target.grasp_pose.pose.position.y,
                       target.grasp_pose.pose.position.z);
    }

    return task;
}

mtc::Task TaskFactory::make_lift(double distance, std::optional<double> joint4_fix_rad,
                                const std::map<std::string, double>* start_joints)
{
    if (distance < 0)
    {
        distance = ctx_.lift_distance;
    }
    LOG_NAMED_INFO("task_factory", "[makeLift] 构建抬升任务, 距离={:.3f}m, joint4_fix={}", distance,
                   joint4_fix_rad ? "是" : "否");

    mtc::Task task;
    if (!ctx_.move_group)
    {
        LOG_NAMED_ERROR("task_factory", "[makeLift] move_group 无效，无法构建任务");
        return task;
    }

    const auto& robot_model = ctx_.move_group->getRobotModel();
    if (!robot_model)
    {
        LOG_NAMED_ERROR("task_factory", "[makeLift] 无法获取 robot model，无法构建任务");
        return task;
    }

    task.setRobotModel(robot_model);
    task.stages()->setName("lift");
    task.enableIntrospection(false);

    const auto* arm_jmg = robot_model->getJointModelGroup(ctx_.arm_group);
    std::map<std::string, double> lift_joint_map;
    bool use_joint_lift = false;
    bool use_start_joints_fixed_state = false; // 是否用 start_joints 构造 FixedState（不用 diag）

    // 分支一：以 start_joints（如 grasp_joint_map）为基准计算抬升目标，避免 joint_state_diag
    // 滞后导致无运动
    if (start_joints && !start_joints->empty() && arm_jmg)
    {
        const auto& jnames = arm_jmg->getActiveJointModelNames();
        std::vector<double> arm_vals;
        arm_vals.reserve(jnames.size());
        for (const auto& name : jnames)
        {
            auto it = start_joints->find(name);
            if (it == start_joints->end())
            {
                break;
            }
            arm_vals.push_back(it->second);
        }
        if (arm_vals.size() == jnames.size())
        {
            moveit::core::RobotState state(robot_model);
            state.setToDefaultValues();
            state.setJointGroupPositions(arm_jmg, arm_vals);
            state.update();
            const Eigen::Isometry3d& eef = state.getGlobalLinkTransform(ctx_.eef_link);
            Eigen::Vector3d delta = eef.rotation() * Eigen::Vector3d(0, 0, std::abs(distance));
            Eigen::Isometry3d target = eef;
            target.translation() += delta;
            geometry_msgs::msg::Pose target_pose;
            target_pose.position.x = target.translation().x();
            target_pose.position.y = target.translation().y();
            target_pose.position.z = target.translation().z();
            Eigen::Quaterniond q(target.rotation());
            target_pose.orientation.x = q.x();
            target_pose.orientation.y = q.y();
            target_pose.orientation.z = q.z();
            target_pose.orientation.w = q.w();
            if (state.setFromIK(arm_jmg, target_pose, ctx_.eef_link, 1.0))
            {
                std::vector<double> jvals;
                state.copyJointGroupPositions(arm_jmg, jvals);
                for (size_t i = 0; i < jnames.size() && i < jvals.size(); ++i)
                {
                    lift_joint_map[jnames[i]] = jvals[i];
                }
                if (joint4_fix_rad && jnames.size() > 3)
                {
                    lift_joint_map[jnames[3]] = *joint4_fix_rad;
                    LOG_NAMED_INFO("task_factory", "[makeLift] Joint4(yaw) 固定为 {:.1f}°",
                                   *joint4_fix_rad * 180.0 / M_PI);
                }
                use_joint_lift = !lift_joint_map.empty();
                use_start_joints_fixed_state = true;
                if (use_joint_lift)
                {
                    LOG_NAMED_INFO(
                        "task_factory",
                        "[makeLift] 抬升以 grasp_joint_map 为起始，共 {} 个关节，关节解成功",
                        static_cast<int>(start_joints->size()));
                }
            }
        }
    }

    // 分支二：未提供 start_joints 或 IK 失败时，从 joint_state_diag 取当前状态计算抬升目标
    if (!use_joint_lift && ctx_.joint_state_diag && ctx_.moveit_mutex && ctx_.node && arm_jmg)
    {
        std::lock_guard<std::mutex> lock(*ctx_.moveit_mutex);
        std::vector<double> arm_vals;
        if (fill_joint_positions_from_diag(ctx_.joint_state_diag, arm_jmg->getActiveJointModelNames(),
                                       arm_vals, ctx_.node->get_clock()->now(), 0.5))
        {
            moveit::core::RobotState state(robot_model);
            state.setToDefaultValues();
            state.setJointGroupPositions(arm_jmg, arm_vals);
            state.update();
            const Eigen::Isometry3d& eef = state.getGlobalLinkTransform(ctx_.eef_link);
            Eigen::Vector3d delta = eef.rotation() * Eigen::Vector3d(0, 0, std::abs(distance));
            Eigen::Isometry3d target = eef;
            target.translation() += delta;
            geometry_msgs::msg::Pose target_pose;
            target_pose.position.x = target.translation().x();
            target_pose.position.y = target.translation().y();
            target_pose.position.z = target.translation().z();
            Eigen::Quaterniond q(target.rotation());
            target_pose.orientation.x = q.x();
            target_pose.orientation.y = q.y();
            target_pose.orientation.z = q.z();
            target_pose.orientation.w = q.w();
            if (state.setFromIK(arm_jmg, target_pose, ctx_.eef_link, 1.0))
            {
                const auto& jnames = arm_jmg->getActiveJointModelNames();
                std::vector<double> jvals;
                state.copyJointGroupPositions(arm_jmg, jvals);
                for (size_t i = 0; i < jnames.size() && i < jvals.size(); ++i)
                {
                    lift_joint_map[jnames[i]] = jvals[i];
                }
                if (joint4_fix_rad && jnames.size() > 3)
                {
                    lift_joint_map[jnames[3]] = *joint4_fix_rad;
                    LOG_NAMED_INFO("task_factory", "[makeLift] Joint4(yaw) 固定为 {:.1f}°",
                                   *joint4_fix_rad * 180.0 / M_PI);
                }
                use_joint_lift = !lift_joint_map.empty();
                if (use_joint_lift)
                {
                    LOG_NAMED_INFO(
                        "task_factory",
                        "[makeLift] 抬升关节解成功，使用关节目标 + JointInterpolationPlanner");
                }
            }
        }
    }

    // 4-DOF：抬升仅用关节/IK，不用笛卡尔。若 IK 解与起始差异过小（会产生 0
    // 时长轨迹），改用关节空间偏移
    const double lift_joint_eps_rad = 0.0087; // 约 0.5°
    if (use_joint_lift && !lift_joint_map.empty() && start_joints && !start_joints->empty())
    {
        double max_diff = 0.0;
        for (const auto& p : lift_joint_map)
        {
            auto it = start_joints->find(p.first);
            if (it != start_joints->end())
            {
                double d = std::abs(p.second - it->second);
                if (d > max_diff)
                    max_diff = d;
            }
        }
        if (max_diff < lift_joint_eps_rad)
        {
            LOG_NAMED_INFO(
                "task_factory",
                "[makeLift] 抬升 IK 与起始差异过小 (max_diff={:.4f} rad)，改用关节空间偏移抬升",
                max_diff);
            use_joint_lift = false;
            lift_joint_map.clear();
        }
    }

    // IK 无解或差异过小：用关节空间偏移（J2/J3 开臂）作为抬升，避免笛卡尔
    if (!use_joint_lift && arm_jmg)
    {
        const auto& jnames = arm_jmg->getActiveJointModelNames();
        std::vector<double> arm_vals;
        if (start_joints && !start_joints->empty())
        {
            arm_vals.reserve(jnames.size());
            for (const auto& name : jnames)
            {
                auto it = start_joints->find(name);
                if (it == start_joints->end())
                    break;
                arm_vals.push_back(it->second);
            }
        }
        if (arm_vals.size() != jnames.size() && ctx_.joint_state_diag && ctx_.moveit_mutex &&
            ctx_.node)
        {
            std::lock_guard<std::mutex> lock(*ctx_.moveit_mutex);
            arm_vals.clear();
            if (fill_joint_positions_from_diag(ctx_.joint_state_diag, jnames, arm_vals,
                                           ctx_.node->get_clock()->now(), 0.5))
            {
                // keep arm_vals
            }
            else
            {
                arm_vals.clear();
            }
        }
        if (arm_vals.size() == jnames.size())
        {
            for (size_t i = 0; i < jnames.size(); ++i)
            {
                double v = arm_vals[i];
                if (jnames[i] == "Joint2")
                    v += ctx_.lift_joint_delta_J2;
                else if (jnames[i] == "Joint3")
                    v += ctx_.lift_joint_delta_J3;
                lift_joint_map[jnames[i]] = v;
            }
            if (joint4_fix_rad && jnames.size() > 3)
            {
                lift_joint_map[jnames[3]] = *joint4_fix_rad;
            }
            // 按模型限位裁剪，避免越界
            moveit::core::RobotState state(robot_model);
            state.setToDefaultValues();
            std::vector<double> lift_vals(jnames.size());
            for (size_t i = 0; i < jnames.size(); ++i)
            {
                lift_vals[i] = lift_joint_map.at(jnames[i]);
            }
            state.setJointGroupPositions(arm_jmg, lift_vals);
            state.enforceBounds();
            state.copyJointGroupPositions(arm_jmg, lift_vals);
            for (size_t i = 0; i < jnames.size(); ++i)
            {
                lift_joint_map[jnames[i]] = lift_vals[i];
            }
            use_joint_lift = true;
            use_start_joints_fixed_state = (start_joints && !start_joints->empty());
            LOG_NAMED_INFO(
                "task_factory",
                "[makeLift] IK 无解，使用关节空间偏移抬升 (J2+{:.3f}, J3+{:.3f} rad，已限位)",
                ctx_.lift_joint_delta_J2, ctx_.lift_joint_delta_J3);
        }
    }

    if (use_joint_lift && !lift_joint_map.empty())
    {
        std::shared_ptr<mtc::solvers::PlannerInterface> planner;
        if (!cached_joint_planner_)
        {
            cached_joint_planner_ = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
        }
        planner = cached_joint_planner_;
        if (use_start_joints_fixed_state && start_joints && !start_joints->empty() && arm_jmg)
        {
            // 用 start_joints 构造 FixedState，保证起始 = 抓取位，轨迹为抓取位 -> 抬升位
            std::vector<double> arm_vals;
            const auto& jnames = arm_jmg->getActiveJointModelNames();
            arm_vals.reserve(jnames.size());
            for (const auto& name : jnames)
            {
                auto it = start_joints->find(name);
                if (it == start_joints->end())
                {
                    break;
                }
                arm_vals.push_back(it->second);
            }
            if (arm_vals.size() == jnames.size())
            {
                moveit::core::RobotState state(robot_model);
                state.setToDefaultValues();
                state.setJointGroupPositions(arm_jmg, arm_vals);
                const auto* gripper_jmg = robot_model->getJointModelGroup(ctx_.gripper_group);
                if (gripper_jmg && ctx_.joint_state_diag && ctx_.node && ctx_.moveit_mutex)
                {
                    std::lock_guard<std::mutex> lock(*ctx_.moveit_mutex);
                    std::vector<double> grip_vals;
                    if (fill_joint_positions_from_diag(ctx_.joint_state_diag,
                                                   gripper_jmg->getActiveJointModelNames(),
                                                   grip_vals, ctx_.node->get_clock()->now(), 5.0))
                    {
                        state.setJointGroupPositions(gripper_jmg, grip_vals);
                    }
                }
                state.update();
                state.enforceBounds();
                auto scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
                scene->setCurrentState(state);
                auto fixed = std::make_unique<mtc::stages::FixedState>("current state");
                fixed->setState(scene);
                task.add(std::move(fixed));
            }
            else
            {
                add_start_state_stage_for_cartesian(task);
            }
        }
        else
        {
            add_start_state_stage_for_cartesian(task);
        }
        task.add(make_move_to_joints("lift", lift_joint_map, planner));
        LOG_NAMED_INFO("task_factory", "[makeLift] 抬升目标: 关节目标 ({} 个关节)",
                       lift_joint_map.size());
    }
    else
    {
        // 4-DOF：不再使用笛卡尔抬升；无关节解且无起始状态时仅记录并返回空任务（FSM 会按失败处理）
        LOG_NAMED_ERROR(
            "task_factory",
            "[makeLift] 无抬升关节解且无起始关节状态，无法构建抬升任务（已禁用笛卡尔抬升）");
    }

    return task;
}

std::unique_ptr<mtc::stages::ModifyPlanningScene>
TaskFactory::make_add_collision_object(const TaskTarget& target)
{
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("add cable object");

    // 创建碰撞对象
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = ctx_.planning_frame;
    collision_object.id = ctx_.cable_name + "_world";
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

    // 创建圆柱体形状（表示缆绳）
    shape_msgs::msg::SolidPrimitive cylinder;
    cylinder.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    cylinder.dimensions.resize(2);
    cylinder.dimensions[0] = ctx_.cable_length;         // 高度
    cylinder.dimensions[1] = ctx_.cable_diameter / 2.0; // 半径
    collision_object.primitives.push_back(cylinder);

    // 计算圆柱体位姿
    // 圆柱体默认轴沿Z轴，需要旋转使其与缆绳方向对齐
    geometry_msgs::msg::Pose cylinder_pose;
    cylinder_pose.position = target.cable_pose.pose.position;

    // 旋转圆柱体使其沿缆绳方向（pitch=90°使轴水平）
    tf2::Quaternion q;
    q.setRPY(0.0, M_PI / 2.0, target.cable_yaw); // pitch=90°, yaw=缆绳方向
    cylinder_pose.orientation = tf2::toMsg(q);

    collision_object.primitive_poses.push_back(cylinder_pose);

    stage->addObject(collision_object);

    LOG_NAMED_INFO("task_factory",
                   "[make_add_collision_object] 添加碰撞对象: id={}, pos=({:.3f}, {:.3f}, {:.3f})",
                   collision_object.id.c_str(), cylinder_pose.position.x, cylinder_pose.position.y,
                   cylinder_pose.position.z);

    return stage;
}

std::unique_ptr<mtc::stages::ModifyPlanningScene> TaskFactory::make_allow_collisions()
{
    auto stage =
        std::make_unique<mtc::stages::ModifyPlanningScene>("allow gripper-cable collisions");

    std::string cable_object_id = ctx_.cable_name + "_world";

    // 允许夹爪相关link与缆绳碰撞
    for (const auto& link : ctx_.allow_touch_links)
    {
        stage->allowCollisions(cable_object_id, link, true);
        LOG_NAMED_DEBUG("task_factory", "[ACM] 允许 {} 与 {} 碰撞", cable_object_id.c_str(),
                        link.c_str());
    }

    LOG_NAMED_INFO("task_factory", "[make_allow_collisions] 设置ACM: 允许{}个夹爪link与缆绳碰撞",
                   ctx_.allow_touch_links.size());

    return stage;
}

std::unique_ptr<mtc::stages::ModifyPlanningScene> TaskFactory::make_attach_object()
{
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach cable object");

    std::string cable_object_id = ctx_.cable_name + "_world";

    // 附着到末端执行器
    stage->attachObject(cable_object_id, ctx_.eef_link);

    LOG_NAMED_INFO("task_factory", "[make_attach_object] 附着碰撞对象 {} 到 {}",
                   cable_object_id.c_str(), ctx_.eef_link.c_str());

    return stage;
}

std::unique_ptr<mtc::stages::ModifyPlanningScene> TaskFactory::make_detach_object()
{
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach cable object");

    std::string cable_object_id = ctx_.cable_name + "_world";

    // 从末端执行器分离
    stage->detachObject(cable_object_id, ctx_.eef_link);

    LOG_NAMED_INFO("task_factory", "[make_detach_object] 分离碰撞对象 {} 从 {}",
                   cable_object_id.c_str(), ctx_.eef_link.c_str());

    return stage;
}

} // namespace m5_grasp
