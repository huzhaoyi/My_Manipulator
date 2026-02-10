/**
 * @file sealien_payload_grasp.cpp
 * @brief Sealien CtrlPilot Payload 机械臂抓取节点 - 模块化重构版本
 *
 * 职责：
 * - 管理ROS2资源（订阅/发布/Timer）
 * - 初始化MoveIt接口
 * - 协调各模块工作
 *
 * 设计原则：
 * - Node只做资源管理，不写具体算法
 * - 所有逻辑委托给模块化组件
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <memory>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_task_constructor_msgs/msg/solution.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>
#include <std_msgs/msg/string.hpp>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

// 日志模块
#include "sealien_payload_grasp/logging/logger.hpp"

// 工具模块
#include "sealien_payload_grasp/utils/parameter_manager.hpp"
#include "sealien_payload_grasp/utils/scene_manager.hpp"

// 新的模块化组件
#include "sealien_payload_grasp/fsm/grasp_fsm.hpp"
#include "sealien_payload_grasp/hw/gripper_interface.hpp"
#include "sealien_payload_grasp/mtc/task_context.hpp"
#include "sealien_payload_grasp/mtc/task_runner.hpp"

// 执行模块
#include "sealien_payload_grasp/execution/gripper_controller.hpp"

// 可视化模块
#include "sealien_payload_grasp/visualization/web_visualizer.hpp"

// 消息
#include "sealien_payload_msgs/msg/cable_pose_with_yaw.hpp"
#include <moveit_msgs/msg/collision_object.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class SealienPayloadGrasp : public rclcpp::Node
{
  public:
    SealienPayloadGrasp() : Node("sealien_payload_grasp")
    {
        // 初始化 TF2 buffer 和 listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 订阅将在 start_executor() 后重新创建并绑定到快速 callback group
        // 这里先创建临时订阅（稍后在 init_move_it() 中重新创建）
        cable_pose_sub_ = this->create_subscription<sealien_payload_msgs::msg::CablePoseWithYaw>(
            "/cable_pose_with_yaw", 10,
            std::bind(&SealienPayloadGrasp::cable_pose_callback, this, std::placeholders::_1));

        // 订阅急停话题
        emergency_stop_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/emergency_stop", 10,
            std::bind(&SealienPayloadGrasp::emergency_stop_callback, this, std::placeholders::_1));

        // 发布抓取状态
        state_publisher_ = this->create_publisher<std_msgs::msg::String>("/grasp_state", 10);

        // 可视化参数
        this->declare_parameter("viz.enable", true);
        this->declare_parameter("viz.marker_topic", "/grasp_markers");
        this->declare_parameter("viz.path_topic", "/eef_path");

        enable_viz_ = this->get_parameter("viz.enable").as_bool();
        auto marker_topic = this->get_parameter("viz.marker_topic").as_string();
        auto path_topic = this->get_parameter("viz.path_topic").as_string();

        marker_pub_ =
            this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic, 10);
        eef_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic, 10);
        cable_pose_viz_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/cable_pose_visualization", 10);
        cable_pose_eye_to_hand_received_pub_ =
            this->create_publisher<geometry_msgs::msg::PoseStamped>(
                "/cable_pose_eye_to_hand_received", 10);
        cable_pose_eye_to_hand_transformed_pub_ =
            this->create_publisher<geometry_msgs::msg::PoseStamped>(
                "/cable_pose_eye_to_hand_transformed", 10);

        // 网页3D可视化模块
        web_visualizer_ = std::make_unique<sealien_payload_grasp::WebVisualizer>(this, 1.0);

        // 声明MTC/OMPL规划器所需的参数（避免CHOMP fallback）
        this->declare_parameter("ompl.planning_plugin", "ompl_interface/OMPLPlanner");
        this->declare_parameter("ompl.request_adapters", "");
        this->declare_parameter("ompl.start_state_max_bounds_error", 0.1);

        // 声明参数
        declare_parameters();

        LOG_NAMED_INFO("sealien_payload_grasp", "========================================");
        LOG_NAMED_INFO("sealien_payload_grasp", "sealien_payload_grasp节点已启动（模块化版本）");
        LOG_NAMED_INFO("sealien_payload_grasp", "========================================");
    }

    ~SealienPayloadGrasp()
    {
        stop_executor_ = true;
        if (executor_thread_.joinable())
        {
            executor_thread_.join();
        }
        LOG_NAMED_INFO("sealien_payload_grasp", "sealien_payload_grasp节点已销毁");
    }

    /**
     * @brief 启动executor（在单独线程中运行）
     *
     * 关键修复：使用 callback group 分离快速回调和慢速工作
     * - cb_fast: joint_states 订阅、action 回调（必须畅通，Reentrant）
     * - cb_work: FSM timer、plan/execute（可慢，MutuallyExclusive）
     */
    void start_executor()
    {
        // 创建 callback groups
        // cb_fast: 用于快速回调（joint_states, action client callbacks）- Reentrant
        // 允许多个回调并发
        // ========== 关键修复：确保 callback group 自动添加到 executor ==========
        // 问题：如果 automatically_add_to_executor_with_node=false，即使 add_node 了，callback
        // group 也不会被 executor 处理 解决：显式设置为 true，确保 callback group 被 executor
        // 正确调度
        cb_fast_ =
            this->create_callback_group(rclcpp::CallbackGroupType::Reentrant,
                                        true); // automatically_add_to_executor_with_node=true
        // cb_work: 用于慢速工作（FSM timer, plan/execute）- MutuallyExclusive 确保不会并发执行
        cb_work_ =
            this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,
                                        true); // automatically_add_to_executor_with_node=true

        // 重新创建订阅，绑定到快速 callback group
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = cb_fast_;

        cable_pose_sub_ = this->create_subscription<sealien_payload_msgs::msg::CablePoseWithYaw>(
            "/cable_pose_with_yaw", 10,
            std::bind(&SealienPayloadGrasp::cable_pose_callback, this, std::placeholders::_1), sub_options);

        emergency_stop_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/emergency_stop", 10,
            std::bind(&SealienPayloadGrasp::emergency_stop_callback, this, std::placeholders::_1), sub_options);

        // ========== 关键修复：使用多线程Executor，确保action回调能并发执行 ==========
        // 问题：单线程executor会导致FSM timer callback阻塞时，action client回调无法执行
        // 解决：使用MultiThreadedExecutor，至少4个线程，确保快速回调（action
        // client）和慢速回调（FSM timer）能并发
        executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
            rclcpp::ExecutorOptions(),
            4); // 4个线程：足够处理FSM timer + action callbacks + 订阅回调
        // ========== 关键修复：只 add_node，callback groups 会自动添加 ==========
        // 问题：如果 automatically_add_to_executor_with_node=true，callback groups 会随着 add_node
        // 自动添加 如果此时再手动 add_callback_group，会导致重复添加而崩溃 解决：只调用
        // add_node，不手动 add_callback_group（因为 auto_add=true）
        executor_->add_node(shared_from_this());
        LOG_NAMED_INFO("sealien_payload_grasp",
                       "已添加 node 到 executor，callback groups 将自动添加（auto_add=true）");

        executor_thread_ = std::thread(
            [this]()
            {
                LOG_NAMED_INFO("sealien_payload_grasp", "Executor线程启动（MultiThreadedExecutor，4线程，callbac"
                                           "k group分离：快速/慢速）");
                int spin_count = 0;
                while (!stop_executor_ && rclcpp::ok())
                {
                    // ========== 关键修复：使用更短的超时，确保回调能被及时处理 ==========
                    // 问题：spin_some(10ms) 可能不够频繁，导致 action client 回调延迟
                    // 解决：使用更短的超时（1ms），确保回调能被及时处理
                    executor_->spin_some(std::chrono::milliseconds(1));
                    spin_count++;
                    // 每1000次spin（约1秒）记录一次，确认executor在运行
                    if (spin_count % 1000 == 0)
                    {
                        LOG_NAMED_DEBUG("sealien_payload_grasp", "Executor运行中（已spin {}次）", spin_count);
                    }
                }
                LOG_NAMED_INFO("sealien_payload_grasp", "Executor线程退出（总共spin {}次）", spin_count);
            });
    }

    /**
     * @brief 初始化MoveIt接口和模块化组件
     */
    void init_move_it()
    {
        LOG_NAMED_INFO("sealien_payload_grasp", "开始初始化MoveIt接口...");

        // 创建MoveGroupInterface
        // 注意：MoveGroupInterface 内部会订阅 joint_states，但无法直接指定 callback group
        // 我们通过 callback group 分离确保 FSM timer 不会阻塞 joint_states 回调
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "arm_group");
        gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "gripper_group");
        planning_scene_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        // 获取配置
        planning_frame_ = move_group_->getPlanningFrame();
        eef_link_ = move_group_->getEndEffectorLink();

        LOG_NAMED_INFO("sealien_payload_grasp", "规划坐标系: {}", planning_frame_.c_str());
        LOG_NAMED_INFO("sealien_payload_grasp", "末端执行器: {}", eef_link_.c_str());

        // 初始化参数管理器
        param_manager_ = std::make_unique<sealien_payload_grasp::ParameterManager>(this);
        param_manager_->declare_parameters();
        param_manager_->load_parameters();

        // 根据配置文件发布手眼（眼在手外）静态 TF：world_link -> sonar_link，供 tf2 自动参与坐标变换
        vision_static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        vision_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        publish_vision_static_tf();
        // 周期性重发（约 10 Hz）：/tf_static + /tf 双发，确保后启动的 tf2_echo 能从 /tf 收到（避免 /tf_static 的 transient_local 在某些环境下未生效）
        vision_static_tf_timer_ =
            this->create_wall_timer(std::chrono::milliseconds(100),
                                    std::bind(&SealienPayloadGrasp::publish_vision_static_tf, this),
                                    cb_work_);

        // 去除地面障碍：不添加地面碰撞体时，移除场景中已有的 ground_plane（支架测试、视觉可发负 z）
        if (!param_manager_->add_ground_plane && planning_scene_)
        {
            moveit_msgs::msg::CollisionObject remove_ground;
            remove_ground.id = "ground_plane";
            remove_ground.operation = moveit_msgs::msg::CollisionObject::REMOVE;
            std::vector<moveit_msgs::msg::CollisionObject> objs;
            objs.push_back(remove_ground);
            planning_scene_->applyCollisionObjects(objs);
            LOG_NAMED_INFO("sealien_payload_grasp", "已移除地面碰撞体 ground_plane（支架测试模式）");
        }

        // 初始化夹爪控制器
        gripper_controller_ =
            std::make_shared<sealien_payload_grasp::GripperController>(this, gripper_group_, moveit_mutex_);
        gripper_controller_->set_parameters(
            param_manager_->gripper_open_width, param_manager_->gripper_close_width,
            param_manager_->gripper_close_extra,
            200, // hold_time_ms
            param_manager_->gripper_angle_min, param_manager_->gripper_angle_max,
            param_manager_->gripper_width_min, param_manager_->gripper_stable_samples,
            param_manager_->gripper_stable_threshold_rad,
            param_manager_->gripper_stable_velocity_eps_rad_s,
            param_manager_->gripper_stable_interval_ms,
            param_manager_->gripper_post_execute_wait_ms,
            param_manager_->gripper_motion_start_threshold_rad,
            param_manager_->gripper_motion_start_timeout_ms,
            param_manager_->gripper_stable_timeout_ms);

        // 初始化TaskContext
        init_task_context();
        gripper_controller_->set_joint_state_diag(task_context_.joint_state_diag);

        // 主控收到 /joint_states 后转发到 /web/joint_states，供后端/网页使用（RELIABLE+TRANSIENT_LOCAL，后启动的仿真器也能拿到最新一帧）
        {
            auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                               .reliability(rclcpp::ReliabilityPolicy::Reliable)
                               .durability(rclcpp::DurabilityPolicy::TransientLocal);
            web_joint_states_pub_ =
                this->create_publisher<sensor_msgs::msg::JointState>("/web/joint_states", pub_qos);
        }

        // 自建 /joint_states 诊断订阅，供 IK 种子、夹爪当前位置、FSM 判稳等使用。
        // 发布端（joint_state_broadcaster）为 best_effort，订阅必须 best_effort 才能收到；若需
        // reliable 判稳，需改广播端或增加中继。
        {
            auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
            rclcpp::SubscriptionOptions sub_opts;
            sub_opts.callback_group = cb_fast_;
            joint_states_diag_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", qos,
                [this](const sensor_msgs::msg::JointState::SharedPtr msg)
                {
                    if (!task_context_.joint_state_diag)
                        return;
                    auto& d = *task_context_.joint_state_diag;
                    std::lock_guard<std::mutex> lock(d.m);
                    d.last_js_wall = this->now();
                    d.last_js_stamp =
                        rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec);
                    d.js_count++;
                    // 缓存 name/position/velocity，供判稳与后备使用
                    if (!msg->name.empty() && msg->position.size() == msg->name.size())
                    {
                        d.last_names = msg->name;
                        d.last_position = msg->position;
                        if (msg->velocity.size() == msg->name.size())
                        {
                            d.last_velocity = msg->velocity;
                        }
                        else
                        {
                            d.last_velocity.clear();
                        }
                    }
                    // 主控收到数据后转发给后端/网页（仿真器订阅 /web/joint_states）
                    if (web_joint_states_pub_)
                    {
                        web_joint_states_pub_->publish(*msg);
                    }
                },
                sub_opts);
            LOG_NAMED_INFO("sealien_payload_grasp",
                           "✓ /joint_states 诊断订阅已创建 (best_effort/volatile，与广播端一致)");
        }

        // 初始化TaskRunner
        task_runner_ = std::make_shared<sealien_payload_grasp::TaskRunner>(task_context_);

        // ========== 关键修复：初始化 TaskRunner 的 action client ==========
        // 问题：action client 需要在 ctx_ 完全准备好后初始化
        // 解决：在 init_move_it() 中，TaskContext 已设置完成后初始化
        if (!task_runner_->initialize_action_client())
        {
            LOG_NAMED_ERROR("sealien_payload_grasp", "TaskRunner action client 初始化失败");
            return;
        }
        LOG_NAMED_INFO("sealien_payload_grasp", "✓ TaskRunner action client 初始化成功");

        // ========== 关键修复：手动将 callback_group_fast 添加到 executor ==========
        // 问题：action client 创建时使用的 callback_group_fast 虽然设置了 auto_add=true，
        // 但 executor 在 add_node 时已经接管了当时存在的 callback groups。
        // 后来创建的 action client 使用的 callback group 可能没有被 executor 正确接管。
        // 解决：在创建 action client 后，手动将 callback_group_fast 添加到 executor
        if (executor_ && cb_fast_)
        {
            try
            {
                executor_->add_callback_group(cb_fast_,
                                              shared_from_this()->get_node_base_interface());
                LOG_NAMED_INFO("sealien_payload_grasp", "✓ 已手动将 callback_group_fast 添加到 executor（确保 "
                                           "action client 回调能被处理）");
            }
            catch (const std::exception& e)
            {
                // 如果 callback group 已经被添加，会抛出异常，这是正常的
                LOG_NAMED_INFO(
                    "sealien_payload_grasp",
                    "callback_group_fast 可能已经添加到 executor: {}（这是正常的，如果之前已添加）",
                    e.what());
            }
        }

        // 下发前回调：将修复后的轨迹发布到网页，保证 JSON 与执行轨迹一致
        if (web_visualizer_ && move_group_)
        {
            auto* tr = dynamic_cast<sealien_payload_grasp::TaskRunner*>(task_runner_.get());
            if (tr)
            {
                tr->set_on_solution_before_send(
                    [this](const moveit_task_constructor_msgs::msg::Solution& sol_msg)
                    {
                        auto robot_model = move_group_->getRobotModel();
                        if (!robot_model)
                            return;
                        moveit::core::RobotStatePtr state =
                            std::make_shared<moveit::core::RobotState>(robot_model);
                        state->setToDefaultValues();
                        for (const auto& sub_traj : sol_msg.sub_trajectory)
                        {
                            const auto& jt = sub_traj.trajectory.joint_trajectory;
                            bool is_arm = true;
                            for (const auto& name : jt.joint_names)
                            {
                                if (name == "JointGL" || name == "JointGR")
                                {
                                    is_arm = false;
                                    break;
                                }
                            }
                            if (is_arm && !jt.points.empty())
                            {
                                web_visualizer_->publish_planned_path_from_trajectory(jt, state,
                                                                                  eef_link_, 50);
                                break;
                            }
                        }
                    });
            }
        }

        grasp_fsm_ = std::make_unique<sealien_payload_grasp::GraspFSM>(task_runner_, gripper_controller_);

        sealien_payload_grasp::GraspFSM::Config fsm_config;
        fsm_config.plan_retry_max = 3;
        fsm_config.exec_retry_max = 2;
        fsm_config.gripper_wait_ms = param_manager_->fsm_gripper_wait_ms;
        fsm_config.gripper_close_min_dwell_ms = param_manager_->fsm_gripper_close_min_dwell_ms;
        fsm_config.gripper_open_min_dwell_ms = param_manager_->fsm_gripper_open_min_dwell_ms;
        fsm_config.arm_stable_timeout_ms = param_manager_->fsm_arm_stable_timeout_ms;
        fsm_config.arm_stable_threshold_rad = param_manager_->fsm_arm_stable_threshold_rad;
        fsm_config.arm_stable_window = param_manager_->fsm_arm_stable_window;
        fsm_config.arm_stable_velocity_eps_rad_s =
            param_manager_->fsm_arm_stable_velocity_eps_rad_s;
        fsm_config.arm_stable_min_dwell_ms = param_manager_->fsm_arm_stable_min_dwell_ms;
        grasp_fsm_->set_config(fsm_config);

        // 设置状态变化回调
        // 当进入 WAIT_EXECUTION_RESULT 时，发布“正在执行的步骤”（old_state），
        // 避免网页进度条快一个阶段（显示下一阶段而实际仍在执行上一阶段）
        grasp_fsm_->set_state_change_callback(
            [this](sealien_payload_grasp::GraspState old_state, sealien_payload_grasp::GraspState new_state)
            {
                sealien_payload_grasp::GraspState display_state = new_state;
                if (new_state == sealien_payload_grasp::GraspState::WAIT_EXECUTION_RESULT)
                {
                    display_state = old_state;
                }
                last_display_state_str_ = sealien_payload_grasp::grasp_state_to_string(display_state);
                publish_state(std::string("FSM:") + last_display_state_str_);
            });

        // 启动FSM定时器（10Hz）- 绑定到慢速 callback group
        fsm_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                             std::bind(&SealienPayloadGrasp::fsm_tick, this),
                                             cb_work_); // 使用慢速 callback group

        // 启动主控心跳（供网页显示“主控”在线）
        if (web_visualizer_)
        {
            web_visualizer_->start_heartbeat();
        }

        LOG_NAMED_INFO("sealien_payload_grasp", "✓ MoveIt接口初始化完成");
        LOG_NAMED_INFO("sealien_payload_grasp", "✓ 模块化组件初始化完成");
        LOG_NAMED_INFO("sealien_payload_grasp", "✓ FSM定时器已启动 (10Hz)");
        LOG_NAMED_INFO("sealien_payload_grasp", "✓ 主控心跳已启动 (/heartbeat/sealien_payload_grasp)");

        moveit_initialized_ = true;
    }

    void publish_vision_static_tf()
    {
        if (!param_manager_ || !vision_static_tf_broadcaster_)
        {
            return;
        }
        const std::string parent_frame = "world_link";
        const std::string child_frame = "sonar_link";
        const auto& pos = param_manager_->vision_position;
        const auto& rpy = param_manager_->vision_orientation_rpy;
        double px = (pos.size() >= 3) ? pos[0] : 0.0;
        double py = (pos.size() >= 3) ? pos[1] : 0.0;
        double pz = (pos.size() >= 3) ? pos[2] : 0.0;
        double roll = (rpy.size() >= 3) ? rpy[0] : 0.0;
        double pitch = (rpy.size() >= 3) ? rpy[1] : 0.0;
        double yaw = (rpy.size() >= 3) ? rpy[2] : 0.0;

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now();
        t.header.frame_id = parent_frame;
        t.child_frame_id = child_frame;
        t.transform.translation.x = px;
        t.transform.translation.y = py;
        t.transform.translation.z = pz;
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        vision_static_tf_broadcaster_->sendTransform(t);
        if (vision_tf_broadcaster_)
        {
            vision_tf_broadcaster_->sendTransform(t);
        }
        // LOG_THROTTLE_INFO(10000,
        //                   "手眼静态 TF: {} -> {}, position=({:.4f}, {:.4f}, {:.4f}) m",
        //                   parent_frame.c_str(), child_frame.c_str(), px, py, pz);
    }

  private:
    /**
     * @brief 声明所有参数
     */
    void declare_parameters()
    {
        // 参数将由ParameterManager在init_move_it中声明和加载
    }

    /**
     * @brief 初始化TaskContext
     */
    void init_task_context()
    {
        task_context_.node = shared_from_this();
        task_context_.move_group = move_group_;
        task_context_.gripper_group_interface = gripper_group_;
        task_context_.planning_scene = planning_scene_;
        task_context_.moveit_mutex = &moveit_mutex_;
        task_context_.executor = executor_; // 传递executor引用，用于spin_until_future_complete
        task_context_.callback_group_fast =
            cb_fast_; // 传递快速callback group，用于action client回调

        // ========== 关键修复：打印主 node 指针地址，用于验证是否是同一个 node ==========
        void* main_node_ptr = shared_from_this().get();
        void* main_node_base_ptr = this->get_node_base_interface().get();
        LOG_NAMED_INFO("sealien_payload_grasp", "主 node 指针地址: node_ptr={}, node_base_ptr={}", main_node_ptr,
                       main_node_base_ptr);

        task_context_.arm_group = "arm_group";
        task_context_.gripper_group = "gripper_group";
        task_context_.eef = "gripper";
        task_context_.eef_link = eef_link_;
        task_context_.planning_frame = planning_frame_;

        task_context_.velocity_scaling = param_manager_->max_velocity_scaling;
        task_context_.acceleration_scaling = param_manager_->max_acceleration_scaling;
        task_context_.planning_time = param_manager_->default_planning_time;
        task_context_.planning_attempts = param_manager_->num_planning_attempts;

        task_context_.approach_offset_z = param_manager_->approach_offset_z;
        task_context_.descend_distance = param_manager_->descend_distance;
        task_context_.lift_distance = param_manager_->lift_distance;
        task_context_.lift_joint_delta_J2 = param_manager_->lift_joint_delta_J2;
        task_context_.lift_joint_delta_J3 = param_manager_->lift_joint_delta_J3;
        task_context_.grasp_yaw_add = param_manager_->grasp_yaw_add;
        task_context_.yaw_offset = param_manager_->yaw_offset;
        task_context_.yaw_flip = param_manager_->yaw_flip;
        task_context_.tcp_offset_x = param_manager_->tcp_offset_x;
        task_context_.tcp_offset_y = param_manager_->tcp_offset_y;
        task_context_.tcp_offset_z = param_manager_->tcp_offset_z;

        // 碰撞对象配置
        task_context_.cable_name = param_manager_->cable_name;
        task_context_.cable_diameter = param_manager_->cable_diameter;
        task_context_.cable_length = param_manager_->cable_length;
        task_context_.allow_touch_links = param_manager_->allow_touch_links;
        task_context_.min_joint_step_rad =
            param_manager_->trajectory_min_joint_step_deg * M_PI / 180.0;
        task_context_.range_execution_time_joint1 = param_manager_->range_execution_time_joint1;
        task_context_.range_execution_time_joint2 = param_manager_->range_execution_time_joint2;
        task_context_.range_execution_time_joint3 = param_manager_->range_execution_time_joint3;
        task_context_.range_execution_time_joint4 = param_manager_->range_execution_time_joint4;
        task_context_.range_execution_time_gripper = param_manager_->range_execution_time_gripper;
        task_context_.execution_timeout_margin_s = param_manager_->execution_timeout_margin_s;
        task_context_.min_execution_timeout_s = param_manager_->min_execution_timeout_s;
        task_context_.max_execution_timeout_s = param_manager_->max_execution_timeout_s;
        task_context_.execution_timeout_safety_factor =
            param_manager_->execution_timeout_safety_factor;
        task_context_.min_segment_time_s = param_manager_->min_segment_time_s;
        task_context_.min_arm_trajectory_time_s = param_manager_->min_arm_trajectory_time_s;

        // 轻量 joint_states 诊断
        task_context_.joint_state_diag = std::make_shared<sealien_payload_grasp::JointStateDiag>();
    }

    /**
     * @brief 缆绳位置回调（轻量级，只更新目标）
     */
    void cable_pose_callback(const sealien_payload_msgs::msg::CablePoseWithYaw::SharedPtr msg)
    {
        // 检查是否已初始化
        if (!moveit_initialized_ || !grasp_fsm_)
        {
            return;
        }

        // 眼在手外：打印并发布 收到坐标 / 转化后坐标（sonar_link -> world_link）
        // 勾选眼在手外时用转化后的坐标配置缆绳抓取，未勾选用原始坐标
        const bool is_eye_to_hand = (msg->header.frame_id == "sonar_link");
        const sealien_payload_msgs::msg::CablePoseWithYaw* msg_for_task = msg.get();
        sealien_payload_msgs::msg::CablePoseWithYaw transformed_msg;

        if (is_eye_to_hand)
        {
            LOG_NAMED_INFO("sealien_payload_grasp",
                           "[眼在手外] 收到坐标 sonar_link: x={:.4f}, y={:.4f}, z={:.4f}, "
                           "yaw={:.2f} rad ({:.1f}°)",
                           msg->position.x, msg->position.y, msg->position.z, msg->yaw,
                           msg->yaw * 180.0 / M_PI);

            if (cable_pose_eye_to_hand_received_pub_)
            {
                geometry_msgs::msg::PoseStamped received;
                received.header = msg->header;
                received.pose.position = msg->position;
                tf2::Quaternion q;
                q.setRPY(0, 0, msg->yaw);
                received.pose.orientation = tf2::toMsg(q);
                cable_pose_eye_to_hand_received_pub_->publish(received);
            }

            if (tf_buffer_ && !planning_frame_.empty())
            {
                try
                {
                    geometry_msgs::msg::PoseStamped pose_in_camera;
                    pose_in_camera.header = msg->header;
                    if (pose_in_camera.header.frame_id.empty())
                    {
                        pose_in_camera.header.frame_id = "sonar_link";
                    }
                    pose_in_camera.pose.position = msg->position;
                    tf2::Quaternion q;
                    q.setRPY(0, 0, msg->yaw);
                    pose_in_camera.pose.orientation = tf2::toMsg(q);

                    std::chrono::nanoseconds timeout_ns(100000000);
                    tf2::Duration timeout(timeout_ns);
                    if (tf_buffer_->canTransform(planning_frame_, pose_in_camera.header.frame_id,
                                                 tf2::TimePointZero, timeout))
                    {
                        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                            planning_frame_, pose_in_camera.header.frame_id, tf2::TimePointZero);
                        geometry_msgs::msg::PoseStamped pose_in_world;
                        tf2::doTransform(pose_in_camera, pose_in_world, transform);
                        pose_in_world.header.frame_id = planning_frame_;

                        LOG_NAMED_INFO("sealien_payload_grasp",
                                       "[眼在手外] 转化后坐标 {}: x={:.4f}, y={:.4f}, z={:.4f}",
                                       planning_frame_.c_str(), pose_in_world.pose.position.x,
                                       pose_in_world.pose.position.y,
                                       pose_in_world.pose.position.z);

                        if (cable_pose_eye_to_hand_transformed_pub_)
                        {
                            cable_pose_eye_to_hand_transformed_pub_->publish(pose_in_world);
                        }

                        transformed_msg.header = pose_in_world.header;
                        transformed_msg.position = pose_in_world.pose.position;
                        transformed_msg.yaw = msg->yaw;
                        msg_for_task = &transformed_msg;
                    }
                    else
                    {
                        LOG_NAMED_WARN("sealien_payload_grasp",
                                       "[眼在手外] TF 不可用，无法计算转化后坐标，忽略本帧目标");
                    }
                }
                catch (const tf2::TransformException& ex)
                {
                    LOG_NAMED_WARN("sealien_payload_grasp", "[眼在手外] 坐标转换失败: {}，忽略本帧目标",
                                  ex.what());
                }
            }
            else
            {
                LOG_NAMED_WARN("sealien_payload_grasp",
                               "[眼在手外] TF 未就绪，无法转化坐标，忽略本帧目标");
            }
        }

        // 眼在手外且未得到转化坐标时，不配置缆绳抓取
        if (is_eye_to_hand && msg_for_task == msg.get())
        {
            return;
        }

        // 获取当前FSM状态
        auto current_state = grasp_fsm_->get_current_state();

        // 只有在IDLE状态才接受新目标；一旦开始执行则忽略新目标
        if (current_state == sealien_payload_grasp::GraspState::IDLE)
        {
            sealien_payload_grasp::TaskTarget task_target;
            if (task_target.compute_from_message(task_context_, *msg_for_task))
            {
                if (param_manager_->fsm_joint_states_ready_enabled)
                {
                    pending_target_ = task_target;
                    pending_start_grasp_ = true;
                    LOG_NAMED_INFO("sealien_payload_grasp",
                                   "目标已接受，等待 joint_states 就绪后启动: ({:.3f}, {:.3f}, "
                                   "{:.3f}), yaw={:.1f}°",
                                   msg_for_task->position.x, msg_for_task->position.y,
                                   msg_for_task->position.z, msg_for_task->yaw * 180.0 / M_PI);
                }
                else
                {
                    grasp_fsm_->set_current_target(task_target);
                    grasp_fsm_->trigger_start_grasp();
                    LOG_NAMED_INFO(
                        "sealien_payload_grasp", "目标已接受并启动: ({:.3f}, {:.3f}, {:.3f}), yaw={:.1f}°",
                        msg_for_task->position.x, msg_for_task->position.y,
                        msg_for_task->position.z, msg_for_task->yaw * 180.0 / M_PI);
                }
            }
        }

        // 可视化（始终更新，方便调试）：使用实际参与计算的坐标
        if (enable_viz_ && cable_pose_viz_pub_)
        {
            geometry_msgs::msg::PoseStamped viz_pose;
            viz_pose.header = msg_for_task->header;
            viz_pose.pose.position = msg_for_task->position;
            tf2::Quaternion q;
            q.setRPY(0, 0, msg_for_task->yaw);
            viz_pose.pose.orientation.x = q.x();
            viz_pose.pose.orientation.y = q.y();
            viz_pose.pose.orientation.z = q.z();
            viz_pose.pose.orientation.w = q.w();
            cable_pose_viz_pub_->publish(viz_pose);
        }
    }

    /** 急停时写入，硬件据此发送“当前反馈位置”作为目标使机械臂立即停止；IDLE 时删除以恢复正常控制 */
    static constexpr const char* EMERGENCY_HOLD_FILE = "/tmp/emergency_hold.txt";

    /**
     * @brief 急停回调
     */
    void emergency_stop_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        LOG_NAMED_ERROR("sealien_payload_grasp", "========================================");
        LOG_NAMED_ERROR("sealien_payload_grasp", "⚠️ 收到急停信号: {}", msg->data.c_str());
        LOG_NAMED_ERROR("sealien_payload_grasp", "========================================");

        emergency_stop_ = true;

        {
            std::ofstream f(EMERGENCY_HOLD_FILE);
            if (f)
            {
                f << "1";
            }
        }

        if (grasp_fsm_)
        {
            grasp_fsm_->emergency_stop();
        }
        if (task_runner_)
        {
            task_runner_->set_emergency_stop(true);
        }

        publish_state("急停:任务已停止");
    }

    /**
     * @brief 判断 /joint_states 是否已就绪（连续收到、非全零、未断流），用于启动前门槛
     */
    bool is_joint_states_ready() const
    {
        if (!param_manager_->fsm_joint_states_ready_enabled || !task_context_.joint_state_diag)
        {
            return true;
        }
        auto& d = *task_context_.joint_state_diag;
        std::lock_guard<std::mutex> lock(d.m);
        if (d.js_count < static_cast<uint64_t>(param_manager_->fsm_joint_states_ready_min_count))
        {
            return false;
        }
        rclcpp::Time now = this->now();
        double age_s = (now - d.last_js_wall).seconds();
        if (age_s < 0.0)
        {
            age_s = 0.0;
        }
        if (age_s > param_manager_->fsm_joint_states_ready_max_age_s)
        {
            return false;
        }
        double sum_abs = 0.0;
        for (double p : d.last_position)
        {
            sum_abs += std::abs(p);
        }
        if (sum_abs <= param_manager_->fsm_joint_states_ready_nonzero_eps)
        {
            return false;
        }
        return true;
    }

    /**
     * @brief FSM定时器回调
     */
    void fsm_tick()
    {
        if (!moveit_initialized_ || !grasp_fsm_)
        {
            return;
        }
        if (grasp_fsm_->get_current_state() == sealien_payload_grasp::GraspState::IDLE && pending_start_grasp_ &&
            is_joint_states_ready())
        {
            grasp_fsm_->set_current_target(pending_target_);
            grasp_fsm_->trigger_start_grasp();
            pending_start_grasp_ = false;
            LOG_NAMED_INFO("sealien_payload_grasp", "joint_states 已就绪，开始抓取");
        }
        grasp_fsm_->tick(this->now());
        if (grasp_fsm_->get_current_state() == sealien_payload_grasp::GraspState::IDLE)
        {
            std::remove(EMERGENCY_HOLD_FILE);
        }
        // 每 5 拍（约 0.5s）周期发布一次当前显示状态，保证网页轮询能拿到最新状态
        fsm_tick_count_++;
        if (fsm_tick_count_ >= 5)
        {
            fsm_tick_count_ = 0;
            publish_state(std::string("FSM:") + last_display_state_str_);
        }
    }

    /**
     * @brief 发布状态
     */
    void publish_state(const std::string& state)
    {
        std_msgs::msg::String msg;
        msg.data = state;
        state_publisher_->publish(msg);

        // WebVisualizer 通过 IK status 发布状态
        if (web_visualizer_)
        {
            // 根据状态类型发布不同的IK状态
            bool is_error = state.find("错误") != std::string::npos ||
                            state.find("失败") != std::string::npos ||
                            state.find("急停") != std::string::npos;
            web_visualizer_->publish_ik_status(!is_error, is_error ? 0.0 : 1.0, state);
        }
    }

    // ========== ROS2资源 ==========
    rclcpp::Subscription<sealien_payload_msgs::msg::CablePoseWithYaw>::SharedPtr cable_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr emergency_stop_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_diag_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr web_joint_states_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr eef_path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cable_pose_viz_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        cable_pose_eye_to_hand_received_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        cable_pose_eye_to_hand_transformed_pub_;

    rclcpp::TimerBase::SharedPtr fsm_timer_;

    // ========== Callback Groups ==========
    rclcpp::CallbackGroup::SharedPtr cb_fast_; // 快速回调：joint_states, action callbacks
    rclcpp::CallbackGroup::SharedPtr cb_work_; // 慢速工作：FSM timer, plan/execute

    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    std::thread executor_thread_;
    std::atomic<bool> stop_executor_{false};

    // ========== TF ==========
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> vision_static_tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> vision_tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr vision_static_tf_timer_;

    // ========== MoveIt ==========
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;
    std::mutex moveit_mutex_;
    std::string planning_frame_;
    std::string eef_link_;
    std::atomic<bool> moveit_initialized_{false};

    // ========== 参数 ==========
    std::unique_ptr<sealien_payload_grasp::ParameterManager> param_manager_;
    bool enable_viz_{true};

    // ========== 状态 ==========
    std::atomic<bool> emergency_stop_{false};
    std::string last_display_state_str_{"IDLE"}; // 由状态变化回调更新，用于周期发布
    int fsm_tick_count_{0};                      // 用于周期发布 /grasp_state
    bool pending_start_grasp_{false};            // 已接受目标，等待 joint_states 就绪后启动
    sealien_payload_grasp::TaskTarget pending_target_;        // 待启动时的目标

    // ========== 模块化组件 ==========
    std::shared_ptr<sealien_payload_grasp::GripperController> gripper_controller_;
    std::shared_ptr<sealien_payload_grasp::TaskRunner> task_runner_;
    std::unique_ptr<sealien_payload_grasp::GraspFSM> grasp_fsm_;
    std::unique_ptr<sealien_payload_grasp::WebVisualizer> web_visualizer_;

    sealien_payload_grasp::TaskContext task_context_;
};

int main(int argc, char* argv[])
{
    try
    {
        // 初始化日志系统
        sealien_payload_grasp::logging::LogConfig log_config;
        log_config.global_level = sealien_payload_grasp::logging::LogLevel::INFO;
        log_config.log_dir = "/tmp/sealien_payload_grasp_logs";
        log_config.file_prefix = "sealien_payload_grasp";
        log_config.max_file_size_mb = 50;
        log_config.max_files = 20;
        log_config.async_mode = true;
        log_config.console_enabled = true;
        log_config.include_source_location = false;
        sealien_payload_grasp::logging::LogManager::instance().init(log_config);

        rclcpp::init(argc, argv);
        LOG_NAMED_INFO("main", "ROS2已初始化");

        // 设置 rclcpp_action 模块的日志级别为 WARN，屏蔽 "unknown goal" 调试日志
        // 这些日志是正常的，只是 MoveIt action client 收到了不属于当前 goal 的状态
        auto ret =
            rcutils_logging_set_logger_level("m5_grasp.rclcpp_action", RCUTILS_LOG_SEVERITY_WARN);
        if (ret != RCUTILS_RET_OK)
        {
            LOG_NAMED_WARN("main", "无法设置 rclcpp_action 日志级别");
        }

        {
            LOG_NAMED_INFO("main", "创建sealien_payload_grasp节点...");
            auto node = std::make_shared<SealienPayloadGrasp>();
            LOG_NAMED_INFO("main", "节点创建成功");

            LOG_NAMED_INFO("main", "启动executor...");
            node->start_executor();
            LOG_NAMED_INFO("main", "Executor已启动");

            // 等待一小段时间让 executor 开始运行（分片 10 x 50ms，避免长时间阻塞）
            for (int i = 0; i < 10; ++i)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }

            LOG_NAMED_INFO("main", "初始化MoveIt接口...");
            node->init_move_it();
            LOG_NAMED_INFO("main", "MoveIt接口已初始化");

            LOG_NAMED_INFO("main", "sealien_payload_grasp节点已完全启动，等待消息...");

            while (rclcpp::ok())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }

        LOG_NAMED_INFO("main", "正在关闭ROS2...");
        sealien_payload_grasp::logging::LogManager::instance().shutdown();
        rclcpp::shutdown();
        return 0;
    }
    catch (const std::exception& e)
    {
        LOG_NAMED_ERROR("main", "错误: {}", e.what());
        sealien_payload_grasp::logging::LogManager::instance().shutdown();
        if (rclcpp::ok())
        {
            rclcpp::shutdown();
        }
        return 1;
    }
}
