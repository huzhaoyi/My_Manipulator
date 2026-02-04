#include "m5_grasp/planning/mtc_task_creator.hpp"
#include "m5_grasp/logging/logger.hpp"
#include "m5_grasp/utils/pose_utils.hpp"
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace m5_grasp
{

MTCTaskCreator::MTCTaskCreator(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface,
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_interface,
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer, std::mutex& moveit_mutex)
    : node_(node), move_group_interface_(move_group_interface),
      gripper_group_interface_(gripper_group_interface),
      planning_scene_interface_(planning_scene_interface), tf_buffer_(tf_buffer),
      moveit_mutex_(moveit_mutex)
{
    // 默认允许接触的link
    allow_touch_links_ = {"LinkGG", "LinkGL", "LinkGR"};
}

void MTCTaskCreator::set_basic_parameters(const std::string& planning_frame,
                                        const std::string& eef_link, const std::string& cable_name,
                                        double ik_timeout, bool add_collision_object)
{
    planning_frame_ = planning_frame;
    eef_link_ = eef_link;
    cable_name_ = cable_name;
    ik_timeout_ = ik_timeout;
    add_collision_object_ = add_collision_object;
}

void MTCTaskCreator::set_grasp_parameters(double approach_offset_z, double descend_distance,
                                        double lift_height, double cable_length,
                                        double cable_diameter)
{
    approach_offset_z_ = approach_offset_z;
    descend_distance_ = descend_distance;
    lift_height_ = lift_height;
    cable_length_ = cable_length;
    cable_diameter_ = cable_diameter;
}

void MTCTaskCreator::set_gripper_parameters(double gripper_open_width, double gripper_close_width,
                                          double gripper_min_joint, double gripper_max_joint,
                                          double gripper_min_width, double gripper_max_width)
{
    gripper_open_width_ = gripper_open_width;
    gripper_close_width_ = gripper_close_width;
    gripper_min_joint_ = gripper_min_joint;
    gripper_max_joint_ = gripper_max_joint;
    gripper_min_width_ = gripper_min_width;
    gripper_max_width_ = gripper_max_width;
}

void MTCTaskCreator::set_tcp_parameters(double tcp_offset_x, double tcp_offset_y, double tcp_offset_z)
{
    tcp_offset_x_ = tcp_offset_x;
    tcp_offset_y_ = tcp_offset_y;
    tcp_offset_z_ = tcp_offset_z;
}

void MTCTaskCreator::set_joint_tolerances(double joint1_tolerance, double joint4_tolerance)
{
    joint1_tolerance_ = joint1_tolerance;
    joint4_tolerance_ = joint4_tolerance;
}

void MTCTaskCreator::set_allow_touch_links(const std::vector<std::string>& links)
{
    allow_touch_links_ = links;
}

void MTCTaskCreator::set_transform_callback(
    std::function<bool(geometry_msgs::msg::PoseStamped&)> callback)
{
    transform_callback_ = callback;
}

void MTCTaskCreator::set_width_to_joint_callback(std::function<double(double)> callback)
{
    width_to_joint_callback_ = callback;
}

void MTCTaskCreator::set_orientation_callbacks(
    std::function<geometry_msgs::msg::Quaternion()> compute_downward_fixed,
    std::function<geometry_msgs::msg::Pose(const geometry_msgs::msg::PoseStamped&, double)>
        make_cable_cylinder_pose)
{
    compute_downward_fixed_callback_ = compute_downward_fixed;
    make_cable_cylinder_pose_callback_ = make_cable_cylinder_pose;
}

void MTCTaskCreator::set_ik_check_callback(
    std::function<YawIKResult(const geometry_msgs::msg::Pose&, const std::vector<double>&,
                              const std::string&, double)>
        callback)
{
    ik_check_callback_ = callback;
}

mtc::solvers::PipelinePlannerPtr MTCTaskCreator::create_pipeline_planner(mtc::Task& task)
{
    mtc::solvers::PipelinePlannerPtr pipeline_planner;

    try
    {
        auto robot_model = task.getRobotModel();
        if (!robot_model)
        {
            throw std::runtime_error("Task机器人模型未加载");
        }

        auto planning_pipeline =
            std::make_shared<planning_pipeline::PlanningPipeline>(robot_model, node_, "ompl");
        pipeline_planner = std::make_shared<mtc::solvers::PipelinePlanner>(planning_pipeline);

        LOG_NAMED_INFO("mtc", "[MTC] 使用PlanningPipeline创建PipelinePlanner成功");
    }
    catch (const std::exception& e)
    {
        LOG_NAMED_WARN("mtc", "无法使用PlanningPipeline创建PipelinePlanner: {}", e.what());
        pipeline_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl");
    }

    return pipeline_planner;
}

double MTCTaskCreator::calculate_joint1_auto(const geometry_msgs::msg::PoseStamped& target_pose)
{
    double joint1_auto = 0.0;

    try
    {
        geometry_msgs::msg::TransformStamped transform_to_base = tf_buffer_->lookupTransform(
            "base_link", target_pose.header.frame_id, tf2::TimePointZero);
        geometry_msgs::msg::PoseStamped target_in_base;
        tf2::doTransform(target_pose, target_in_base, transform_to_base);

        double x_in_base = target_in_base.pose.position.x;
        double y_in_base = target_in_base.pose.position.y;

        joint1_auto = std::atan2(y_in_base, x_in_base);
        LOG_NAMED_INFO("mtc",
                       "[Joint1自动计算] 目标在base_link: ({:.3f}, {:.3f}) -> Joint1={:.3f} rad",
                       x_in_base, y_in_base, joint1_auto);
    }
    catch (const tf2::TransformException& ex)
    {
        LOG_NAMED_WARN("mtc", "[Joint1自动计算] TF转换失败: {}", ex.what());
        joint1_auto = 0.0;
    }

    return joint1_auto;
}

std::map<std::string, double>
MTCTaskCreator::prepare_pregrasp_joint_map(const geometry_msgs::msg::PoseStamped& pregrasp_pose,
                                        double cable_yaw, double joint1_auto, double joint4_target,
                                        double& selected_yaw, bool& use_joint_goal)
{
    std::map<std::string, double> joint_map;
    use_joint_goal = false;
    selected_yaw = cable_yaw;

    if (!ik_check_callback_)
    {
        return joint_map;
    }

    try
    {
        // 准备yaw候选
        std::vector<double> yaw_candidates;
        yaw_candidates.push_back(cable_yaw);
        double yaw_flipped = normalize_angle(cable_yaw + M_PI);
        yaw_candidates.push_back(yaw_flipped);

        YawIKResult ik_result =
            ik_check_callback_(pregrasp_pose.pose, yaw_candidates, eef_link_, 0.5);

        if (ik_result.success && ik_result.successful_state)
        {
            const auto* jmg = ik_result.successful_state->getJointModelGroup("arm_group");
            if (jmg)
            {
                std::vector<double> joint_values;
                ik_result.successful_state->copyJointGroupPositions(jmg, joint_values);
                const std::vector<std::string>& joint_names = jmg->getActiveJointModelNames();

                size_t num_joints = std::min(joint_values.size(), joint_names.size());
                if (num_joints > 0)
                {
                    for (size_t i = 0; i < num_joints; ++i)
                    {
                        joint_map[joint_names[i]] = joint_values[i];
                    }

                    // 覆盖Joint1和Joint4
                    joint_map["Joint1"] = joint1_auto;
                    joint_map["Joint4"] = joint4_target;

                    use_joint_goal = true;
                    selected_yaw = ik_result.selected_yaw;

                    LOG_NAMED_INFO("mtc", "[预抓取IK] 成功，使用yaw={:.3f} rad", selected_yaw);
                }
            }
        }
    }
    catch (const std::exception& e)
    {
        LOG_NAMED_WARN("mtc", "[预抓取IK] 异常: {}", e.what());
    }

    return joint_map;
}

void MTCTaskCreator::add_fixed_start_state(mtc::Task& task)
{
    // 与 task_factory::addCurrentStateStage 对齐：使用 CurrentState，由执行端在规划时从 planning
    // scene 取状态。 planning scene 由 /joint_states 更新（机械臂 → json → parse_feedback →
    // joint_state_broadcaster → /joint_states）， 不再在本节点调用
    // getCurrentJointValues/getCurrentState，避免时间戳/拉取失败导致流程卡住。
    auto current_state = std::make_unique<mtc::stages::CurrentState>("current state");
    task.add(std::move(current_state));
    LOG_NAMED_INFO("mtc", "[MTC] 已添加 CurrentState stage（与 task_factory 一致）");
}

void MTCTaskCreator::add_open_gripper_stage(mtc::Task& task, mtc::solvers::PipelinePlannerPtr planner)
{
    auto open_gripper_stage = std::make_unique<mtc::stages::MoveTo>("open gripper", planner);
    open_gripper_stage->setGroup("gripper_group");

    double open_joint_gl = 0.0;
    if (width_to_joint_callback_)
    {
        open_joint_gl = width_to_joint_callback_(gripper_open_width_);
    }
    double open_joint_gr = -open_joint_gl;

    std::map<std::string, double> open_gripper_joint_map;
    open_gripper_joint_map["JointGL"] = open_joint_gl;
    open_gripper_joint_map["JointGR"] = open_joint_gr;
    open_gripper_stage->setGoal(open_gripper_joint_map);

    task.add(std::move(open_gripper_stage));
    LOG_NAMED_INFO("mtc", "[MTC] 已添加open gripper stage (JointGL={:.3f})", open_joint_gl);
}

void MTCTaskCreator::add_close_gripper_stage(mtc::Task& task, mtc::solvers::PipelinePlannerPtr planner)
{
    auto close_gripper_stage = std::make_unique<mtc::stages::MoveTo>("close gripper", planner);
    close_gripper_stage->setGroup("gripper_group");

    double close_joint_gl = 0.0;
    if (width_to_joint_callback_)
    {
        close_joint_gl = width_to_joint_callback_(gripper_close_width_);
    }
    double close_joint_gr = -close_joint_gl;

    std::map<std::string, double> close_gripper_joint_map;
    close_gripper_joint_map["JointGL"] = close_joint_gl;
    close_gripper_joint_map["JointGR"] = close_joint_gr;
    close_gripper_stage->setGoal(close_gripper_joint_map);

    task.add(std::move(close_gripper_stage));
    LOG_NAMED_INFO("mtc", "[MTC] 已添加close gripper stage (JointGL={:.3f})", close_joint_gl);
}

void MTCTaskCreator::add_move_to_pregrasp_stage(mtc::Task& task,
                                            mtc::solvers::PipelinePlannerPtr planner,
                                            const geometry_msgs::msg::PoseStamped& pregrasp_pose,
                                            const std::map<std::string, double>& joint_map,
                                            bool use_joint_goal, double joint1_auto,
                                            double joint4_target)
{
    auto move_to_pregrasp = std::make_unique<mtc::stages::MoveTo>("move to pregrasp", planner);
    move_to_pregrasp->setGroup("arm_group");
    move_to_pregrasp->setTimeout(30.0);

    if (use_joint_goal && !joint_map.empty())
    {
        move_to_pregrasp->setGoal(joint_map);
        LOG_NAMED_INFO("mtc", "[MTC] move to pregrasp使用joint goal");
    }
    else
    {
        move_to_pregrasp->setIKFrame(eef_link_);
        move_to_pregrasp->setGoal(pregrasp_pose);

        // 设置约束
        moveit_msgs::msg::Constraints constraints;

        // Joint1约束
        moveit_msgs::msg::JointConstraint joint1_constraint;
        joint1_constraint.joint_name = "Joint1";
        joint1_constraint.position = joint1_auto;
        joint1_constraint.tolerance_above = joint1_tolerance_;
        joint1_constraint.tolerance_below = joint1_tolerance_;
        joint1_constraint.weight = 1.0;
        constraints.joint_constraints.push_back(joint1_constraint);

        // Joint4约束
        moveit_msgs::msg::JointConstraint joint4_constraint;
        joint4_constraint.joint_name = "Joint4";
        joint4_constraint.position = joint4_target;
        joint4_constraint.tolerance_above = joint4_tolerance_;
        joint4_constraint.tolerance_below = joint4_tolerance_;
        joint4_constraint.weight = 1.0;
        constraints.joint_constraints.push_back(joint4_constraint);

        // Orientation约束
        if (compute_downward_fixed_callback_)
        {
            moveit_msgs::msg::OrientationConstraint orientation_constraint;
            orientation_constraint.header.frame_id = planning_frame_;
            orientation_constraint.link_name = eef_link_;
            orientation_constraint.orientation = compute_downward_fixed_callback_();
            orientation_constraint.absolute_x_axis_tolerance = 0.1;
            orientation_constraint.absolute_y_axis_tolerance = 0.1;
            orientation_constraint.absolute_z_axis_tolerance = M_PI;
            orientation_constraint.weight = 1.0;
            constraints.orientation_constraints.push_back(orientation_constraint);
        }

        move_to_pregrasp->setPathConstraints(constraints);
        LOG_NAMED_INFO("mtc", "[MTC] move to pregrasp使用pose goal + 约束");
    }

    task.add(std::move(move_to_pregrasp));
}

void MTCTaskCreator::add_cable_collision_object(mtc::Task& task,
                                             const geometry_msgs::msg::PoseStamped& cable_pose,
                                             double cable_yaw)
{
    auto add_object = std::make_unique<mtc::stages::ModifyPlanningScene>("add cable object");

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = planning_frame_;
    collision_object.id = cable_name_ + "_world";
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive cylinder;
    cylinder.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    cylinder.dimensions.resize(2);
    cylinder.dimensions[0] = cable_length_;
    cylinder.dimensions[1] = cable_diameter_ / 2.0;

    collision_object.primitives.push_back(cylinder);

    if (make_cable_cylinder_pose_callback_)
    {
        geometry_msgs::msg::Pose cylinder_pose =
            make_cable_cylinder_pose_callback_(cable_pose, cable_yaw);
        collision_object.primitive_poses.push_back(cylinder_pose);
    }
    else
    {
        // 默认位姿
        collision_object.primitive_poses.push_back(cable_pose.pose);
    }

    add_object->addObject(collision_object);
    task.add(std::move(add_object));

    LOG_NAMED_INFO("mtc", "[MTC] 已添加cable collision object");
}

void MTCTaskCreator::add_allow_collisions_stage(mtc::Task& task)
{
    auto allow_collisions =
        std::make_unique<mtc::stages::ModifyPlanningScene>("allow gripper-cable collisions");
    std::string cable_object_id = cable_name_ + "_world";

    for (const auto& link : allow_touch_links_)
    {
        allow_collisions->allowCollisions(cable_object_id, link, true);
    }

    task.add(std::move(allow_collisions));
    LOG_NAMED_INFO("mtc", "[MTC] 已设置AllowedCollisionMatrix");
}

void MTCTaskCreator::add_descend_stage(mtc::Task& task, mtc::solvers::PipelinePlannerPtr planner,
                                     const geometry_msgs::msg::PoseStamped& grasp_pose)
{
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(0.5);
    cartesian_planner->setMaxAccelerationScalingFactor(0.5);
    cartesian_planner->setStepSize(0.01);

    auto move_to_grasp =
        std::make_unique<mtc::stages::MoveTo>("descend to grasp", cartesian_planner);
    move_to_grasp->setGroup("arm_group");
    move_to_grasp->setIKFrame(eef_link_);
    move_to_grasp->setGoal(grasp_pose);

    task.add(std::move(move_to_grasp));
    LOG_NAMED_INFO("mtc", "[MTC] 已添加descend stage");
}

void MTCTaskCreator::add_attach_object_stage(mtc::Task& task)
{
    auto attach_object = std::make_unique<mtc::stages::ModifyPlanningScene>("attach cable");
    attach_object->attachObject(cable_name_ + "_world", eef_link_);
    task.add(std::move(attach_object));
    LOG_NAMED_INFO("mtc", "[MTC] 已添加attach object stage");
}

void MTCTaskCreator::add_lift_stage(mtc::Task& task, mtc::solvers::PipelinePlannerPtr planner)
{
    auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(0.5);
    cartesian_planner->setMaxAccelerationScalingFactor(0.5);
    cartesian_planner->setStepSize(0.01);

    auto lift = std::make_unique<mtc::stages::MoveRelative>("lift", cartesian_planner);
    lift->setGroup("arm_group");

    geometry_msgs::msg::Vector3Stamped lift_direction;
    lift_direction.header.frame_id = planning_frame_;
    lift_direction.vector.z = lift_height_;
    lift->setDirection(lift_direction);

    lift->setMinMaxDistance(lift_height_ * 0.5, lift_height_);

    task.add(std::move(lift));
    LOG_NAMED_INFO("mtc", "[MTC] 已添加lift stage (height={:.3f})", lift_height_);
}

mtc::Task MTCTaskCreator::create_grasp_task(const geometry_msgs::msg::PoseStamped& cable_pose,
                                          double cable_yaw, double joint4_target)
{
    mtc::Task task;

    {
        std::lock_guard<std::mutex> lock(moveit_mutex_);
        move_group_interface_->clearPathConstraints();
        move_group_interface_->clearPoseTargets();
    }

    if (!move_group_interface_)
    {
        LOG_NAMED_ERROR("mtc", "MoveGroupInterface未初始化");
        return task;
    }

    task.loadRobotModel(node_);
    task.stages()->setName("cable grasp task");
    task.enableIntrospection(true);

    auto pipeline_planner = create_pipeline_planner(task);

    // 转换位姿到planning frame
    geometry_msgs::msg::PoseStamped cable_pose_planning = cable_pose;
    if (transform_callback_ && !transform_callback_(cable_pose_planning))
    {
        LOG_NAMED_ERROR("mtc", "无法转换缆绳位姿到planning frame");
        return task;
    }

    // 计算Joint1
    double joint1_auto = calculate_joint1_auto(cable_pose_planning);

    // 计算预抓取位姿
    double pregrasp_clearance = std::max(approach_offset_z_, 0.20);
    geometry_msgs::msg::PoseStamped pregrasp_pose_center = cable_pose_planning;
    pregrasp_pose_center.pose.position.z += pregrasp_clearance;
    if (compute_downward_fixed_callback_)
    {
        pregrasp_pose_center.pose.orientation = compute_downward_fixed_callback_();
    }

    // TCP偏移补偿
    tf2::Quaternion q_center;
    tf2::fromMsg(pregrasp_pose_center.pose.orientation, q_center);
    tf2::Matrix3x3 R_center(q_center);
    tf2::Vector3 offset_center_from_linkgg(tcp_offset_x_, tcp_offset_y_, tcp_offset_z_);
    tf2::Vector3 offset_world = R_center * offset_center_from_linkgg;

    geometry_msgs::msg::PoseStamped pregrasp_pose;
    pregrasp_pose.header.frame_id = planning_frame_;
    pregrasp_pose.pose.position.x = pregrasp_pose_center.pose.position.x - offset_world.x();
    pregrasp_pose.pose.position.y = pregrasp_pose_center.pose.position.y - offset_world.y();
    pregrasp_pose.pose.position.z = pregrasp_pose_center.pose.position.z - offset_world.z();
    pregrasp_pose.pose.orientation = pregrasp_pose_center.pose.orientation;

    // 准备joint map
    double selected_yaw = cable_yaw;
    bool use_joint_goal = false;
    std::map<std::string, double> pregrasp_joint_map = prepare_pregrasp_joint_map(
        pregrasp_pose, cable_yaw, joint1_auto, joint4_target, selected_yaw, use_joint_goal);

    // 抓取位姿
    geometry_msgs::msg::PoseStamped grasp_pose = cable_pose_planning;
    if (compute_downward_fixed_callback_)
    {
        grasp_pose.pose.orientation = compute_downward_fixed_callback_();
    }
    grasp_pose.header.frame_id = planning_frame_;

    // 添加各个阶段
    add_fixed_start_state(task);
    add_open_gripper_stage(task, pipeline_planner);
    add_move_to_pregrasp_stage(task, pipeline_planner, pregrasp_pose, pregrasp_joint_map,
                           use_joint_goal, joint1_auto, joint4_target);

    if (add_collision_object_)
    {
        add_cable_collision_object(task, cable_pose_planning, selected_yaw);
        add_allow_collisions_stage(task);
    }

    add_descend_stage(task, pipeline_planner, grasp_pose);
    add_close_gripper_stage(task, pipeline_planner);
    add_attach_object_stage(task);
    add_lift_stage(task, pipeline_planner);

    LOG_NAMED_INFO("mtc", "[MTC] 抓取任务创建完成");

    return task;
}

mtc::Task MTCTaskCreator::create_place_task(const geometry_msgs::msg::PoseStamped& place_pose,
                                          double cable_yaw)
{
    mtc::Task task;

    task.loadRobotModel(node_);
    task.stages()->setName("cable place task");
    task.enableIntrospection(true);

    auto pipeline_planner = create_pipeline_planner(task);

    // 转换位姿
    geometry_msgs::msg::PoseStamped place_pose_planning = place_pose;
    if (transform_callback_ && !transform_callback_(place_pose_planning))
    {
        LOG_NAMED_ERROR("mtc", "无法转换放置位姿到planning frame");
        return task;
    }

    // 添加阶段
    add_fixed_start_state(task);

    // MoveTo place
    auto move_to_place = std::make_unique<mtc::stages::MoveTo>("move to place", pipeline_planner);
    move_to_place->setGroup("arm_group");
    move_to_place->setIKFrame(eef_link_);
    move_to_place->setGoal(place_pose_planning);
    task.add(std::move(move_to_place));

    // Open gripper to release
    add_open_gripper_stage(task, pipeline_planner);

    // Detach object
    auto detach_object = std::make_unique<mtc::stages::ModifyPlanningScene>("detach cable");
    detach_object->detachObject(cable_name_ + "_world", eef_link_);
    task.add(std::move(detach_object));

    // Lift after place
    add_lift_stage(task, pipeline_planner);

    LOG_NAMED_INFO("mtc", "[MTC] 放置任务创建完成");

    return task;
}

mtc::Task MTCTaskCreator::create_grasp_and_place_task(const geometry_msgs::msg::PoseStamped& cable_pose,
                                                  const geometry_msgs::msg::PoseStamped& place_pose,
                                                  double cable_yaw, double joint4_target)
{
    // 暂时只实现抓取任务，放置任务需要更复杂的逻辑
    // 可以在后续版本中扩展
    return create_grasp_task(cable_pose, cable_yaw, joint4_target);
}

} // namespace m5_grasp
