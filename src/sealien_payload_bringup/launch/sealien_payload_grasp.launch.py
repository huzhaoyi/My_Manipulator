from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os
import sys
import contextlib
import yaml

# 抑制 MoveItConfigsBuilder 的 SRDF 推断警告
class WarningFilter:
    """过滤 stderr 中的 SRDF 推断警告"""
    def __init__(self, original):
        self.original = original
        
    def write(self, message):
        if "Cannot infer SRDF" not in message and "cartesian_limits.yaml is deprecated" not in message:
            self.original.write(message)
            
    def flush(self):
        self.original.flush()
        
    def __getattr__(self, name):
        return getattr(self.original, name)

@contextlib.contextmanager
def suppress_moveit_warnings():
    """临时抑制 MoveIt 相关警告"""
    old_stderr = sys.stderr
    sys.stderr = WarningFilter(old_stderr)
    try:
        yield
    finally:
        sys.stderr = old_stderr


def generate_launch_description():
    # 获取包路径
    sealien_payload_moveit_config_dir = get_package_share_directory("sealien_payload_moveit_config")
    sealien_payload_bringup_dir = get_package_share_directory("sealien_payload_bringup")
    sealien_payload_grasp_dir = get_package_share_directory("sealien_payload_grasp")
    
    # 构建 MoveIt 配置
    srdf = os.path.join(sealien_payload_moveit_config_dir, "config", "m5.srdf")
    
    with suppress_moveit_warnings():
        moveit_config = (
            MoveItConfigsBuilder("m5", package_name="sealien_payload_moveit_config")
            .robot_description_semantic(file_path=srdf)
            .robot_description_kinematics(file_path=os.path.join(sealien_payload_moveit_config_dir, "config", "kinematics.yaml"))
            .sensors_3d(file_path=os.path.join(sealien_payload_moveit_config_dir, "config", "sensors_3d.yaml"))
            .trajectory_execution(file_path=os.path.join(sealien_payload_moveit_config_dir, "config", "moveit_controllers.yaml"))
            .to_moveit_configs()
        )
    
    # 包含 static_transform_publisher launch
    static_transform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sealien_payload_bringup_dir, "launch", "static_virtual_joint_tfs.launch.py")
        )
    )
    
    # 包含 robot_state_publisher launch
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sealien_payload_bringup_dir, "launch", "rsp.launch.py")
        )
    )
    
    # 启动 ros2_control_node（QoS 由 ros2_controllers.yaml 内 qos_overrides 统一配置）
    # use_sim_time 必须 False，否则 joint_states header.stamp 会为 0，MoveIt 报 "latest received state has time 0.000000"
    # robot_description 改为从 robot_state_publisher 的 /robot_description 话题获取，避免 Deprecated 提示及未来版本移除参数导致启动失败
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            os.path.join(sealien_payload_moveit_config_dir, "config", "ros2_controllers.yaml"),
            {"use_sim_time": False},
        ],
        remappings=[("~/robot_description", "/robot_description")],
        output="screen",
    )
    
    # 加载pilz_cartesian_limits.yaml并添加到robot_description_planning命名空间
    pilz_cartesian_limits_path = os.path.join(sealien_payload_moveit_config_dir, "config", "pilz_cartesian_limits.yaml")
    robot_description_planning = {}
    if os.path.exists(pilz_cartesian_limits_path):
        with open(pilz_cartesian_limits_path, 'r') as f:
            pilz_limits = yaml.safe_load(f)
            if 'cartesian_limits' in pilz_limits:
                robot_description_planning = {
                    "robot_description_planning": {
                        "cartesian_limits": pilz_limits['cartesian_limits']
                    }
                }
    
    # 构建 move_group 参数列表（与上一版 607e1b8 一致，不叠加超时/QoS 等“修复”参数）
    # 注意：启用MTC的execute_task_solution capability，用于执行MTC规划的任务
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        # 启用MTC的ExecuteTaskSolutionCapability插件
        # 这个capability提供/execute_task_solution action server
        "capabilities": "move_group/ExecuteTaskSolutionCapability",
        "disable_capabilities": "",
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
        "planning_scene_monitor.octomap_resolution": 0.1,  # 显式设置，消除「Resolution not specified for Octomap」WARN（无 3D 传感器时仍会报 No 3D sensor plugin，已用 sensors: [] 缓解）
        "planning_scene_monitor.publish_planning_scene": True,
        "planning_scene_monitor.publish_geometry_updates": True,
        "planning_scene_monitor.publish_state_updates": True,
        "planning_scene_monitor.publish_transforms_updates": True,
    }
    
    move_group_params = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.planning_pipelines,
        moveit_config.joint_limits,
        moveit_config.sensors_3d,
        moveit_config.trajectory_execution,
        move_group_configuration,
        robot_description_planning,  # 添加robot_description_planning参数
        {"use_sim_time": False},
    ]
    
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
    )
    
    # 包含 spawn_controllers launch
    # 修复：移除 SetRemap，让 joint_state_broadcaster 直接发布到 /joint_states
    # 这是 MoveIt/ros2_control 的标准用法，避免双发布源导致的时间戳不一致问题
    spawn_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sealien_payload_bringup_dir, "launch", "spawn_controllers.launch.py")
        )
    )
    
    # 修复：移除 joint_state_publisher_node 重发布节点
    # 原因：双 /joint_states 发布源会导致 MoveIt 的 current_state_monitor 时间戳检查失败
    # 如果确实需要调整关节顺序，应该在 URDF/SRDF 或 ros2_control 配置中修复
    # joint_state_publisher_node = Node(
    #     package="m5_control",
    #     executable="joint_state_publisher_node",
    #     name="m5_joint_state_publisher",
    #     output="screen",
    #     parameters=[
    #         {"republish": True},
    #         {"source_topic": "/joint_states_raw"},
    #         {"output_topic": "/joint_states"},
    #     ],
    # )
    
    # 创建 sealien_payload_grasp 节点，加载配置文件（与上一版 607e1b8 一致，不传额外超时参数）
    # 显式 use_sim_time: False，与 ros2_control/move_group 一致，避免 joint_states 时间戳被当成 0
    sealien_payload_grasp = Node(
        package="sealien_payload_grasp",
        executable="sealien_payload_grasp_node",
        name="sealien_payload_grasp",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            os.path.join(sealien_payload_grasp_dir, "config", "cable_grasp.yaml"),  # 加载缆绳抓取配置
            {"use_sim_time": False},
        ],
        arguments=[
            '--ros-args',
            '--log-level', 'sealien_payload_grasp:=debug',
            '--log-level', 'moveit_task_constructor:=debug',
            '--log-level', 'moveit_task_constructor.core:=debug',
            # 静默「Publisher already registered」：同一节点被 MoveGroupInterface/PlanningSceneInterface 等多处使用时 rcl 会打 WARN
            '--log-level', 'rcl.logging_rosout:=error',
        ],
    )
    
    # 包含 RViz launch（用于可视化规划）
    # 暂时禁用RViz以加快启动速度
    # rviz_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(sealien_payload_bringup_dir, "launch", "moveit_rviz.launch.py")
    #     )
    # )
    
    return LaunchDescription([
        static_transform_launch,
        rsp_launch,
        ros2_control_node,
        move_group_node,
        spawn_controllers_launch,
        # joint_state_publisher_node,  # 已移除：避免双 /joint_states 发布源导致时间戳不一致
        sealien_payload_grasp,
        # rviz_launch,  # 暂时禁用RViz
    ])
