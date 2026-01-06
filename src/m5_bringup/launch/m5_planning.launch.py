from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os
import sys
import contextlib

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
    m5_moveit_config_dir = get_package_share_directory("m5_moveit_config")
    m5_bringup_dir = get_package_share_directory("m5_bringup")
    
    # 构建 MoveIt 配置，确保 kinematics.yaml 被正确加载
    # 加载传感器配置以禁用 Octomap
    # 添加 trajectory_execution 配置（与 rviz.launch.py 一致）
    # 显式指定SRDF文件，确保arm_group的chain base_link被正确解析
    # 使用绝对路径避免 MoveItConfigsBuilder 的路径推断警告
    # 按照用户建议的方式显式指定 SRDF 路径
    srdf = os.path.join(m5_moveit_config_dir, "config", "m5.srdf")
    
    # 使用警告过滤器抑制 MoveItConfigsBuilder 的 SRDF 推断警告
    with suppress_moveit_warnings():
        moveit_config = (
            MoveItConfigsBuilder("m5", package_name="m5_moveit_config")
            .robot_description_semantic(file_path=srdf)  # 显式指定 SRDF 路径
            .robot_description_kinematics(file_path=os.path.join(m5_moveit_config_dir, "config", "kinematics.yaml"))
            .sensors_3d(file_path=os.path.join(m5_moveit_config_dir, "config", "sensors_3d.yaml"))
            .trajectory_execution(file_path=os.path.join(m5_moveit_config_dir, "config", "moveit_controllers.yaml"))  # 使用 MoveItConfigsBuilder 方式加载
            .to_moveit_configs()
        )
    
    # 包含 static_transform_publisher launch（发布world到base_link的变换）
    static_transform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(m5_bringup_dir, "launch", "static_virtual_joint_tfs.launch.py")
        )
    )
    
    # 包含 robot_state_publisher launch（发布机器人状态）
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(m5_bringup_dir, "launch", "rsp.launch.py")
        )
    )
    
    # 启动 ros2_control_node（控制器管理器，包含硬件接口）
    # 注意：虽然直接传递 robot_description 参数已弃用，但硬件接口初始化需要它
    # 保留参数传递以确保硬件接口（UDP通信）能正常工作
    # robot_state_publisher 也会发布 robot_description 到 topic（供其他节点使用）
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,  # 硬件接口需要 robot_description 来初始化 UDP 配置
            os.path.join(m5_moveit_config_dir, "config", "ros2_controllers.yaml"),
        ],
        output="screen",
    )
    
    # 手动创建 move_group 节点，确保所有参数包括 kinematics 和 controllers 被正确传递
    from moveit_configs_utils.launches import DeclareBooleanLaunchArg, add_debuggable_node
    from launch.substitutions import LaunchConfiguration
    from launch_ros.parameter_descriptions import ParameterValue
    
    # 构建 move_group 参数列表
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "capabilities": "",
        "disable_capabilities": "",
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
        # 禁用 Octomap 监控（不需要 3D 传感器）
        "planning_scene_monitor.octomap_resolution": 0.0,
        "planning_scene_monitor.publish_planning_scene": True,
        "planning_scene_monitor.publish_geometry_updates": True,
        "planning_scene_monitor.publish_state_updates": True,
        "planning_scene_monitor.publish_transforms_updates": True,
    }
    
    move_group_params = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,   # ✅关键：显式加上
        moveit_config.planning_pipelines,             # 建议也加上
        moveit_config.joint_limits,                   # 建议也加上
        moveit_config.sensors_3d,                     # 你需要禁用octomap的话也要加上
        moveit_config.trajectory_execution,          # ✅使用 MoveItConfigsBuilder 加载的 trajectory_execution（与 rviz.launch.py 一致）
        move_group_configuration,
        {"use_sim_time": False},
        ]

    
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
    )



    
    # 包含 spawn_controllers launch（启动硬件接口和控制器）
    spawn_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(m5_bringup_dir, "launch", "spawn_controllers.launch.py")
        )
    )
    
    # 创建 joint_state_publisher_node，用于重新排序关节状态
    # 此节点订阅 /joint_states（来自 joint_state_broadcaster），重新排序后发布到 /joint_states
    # 为了避免循环，我们需要先停止 joint_state_broadcaster 的发布，或者使用不同的 QoS
    # 最简单的方法：让 joint_state_broadcaster 发布到 /joint_states_raw，然后此节点重新排序后发布到 /joint_states
    # 但 joint_state_broadcaster 是在 spawn_controllers 中启动的，我们无法直接 remap
    # 所以这里我们让节点订阅 /joint_states，但需要确保 joint_state_broadcaster 先停止发布
    # 或者，我们可以通过修改 ros2_controllers.yaml 来配置 joint_state_broadcaster 发布到 /joint_states_raw
    joint_state_publisher_node = Node(
        package="m5_control",
        executable="joint_state_publisher_node",
        name="m5_joint_state_publisher",
        output="screen",
        parameters=[
            {"republish": True},
            {"source_topic": "/joint_states"},  # 订阅 joint_state_broadcaster 发布的话题
        ],
    )
    
    # 创建 m5_planning 节点，并传递 MoveIt 配置参数
    m5_planning = Node(
        package="m5_planning",
        executable="m5_planning",
        name="m5_planning",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )
    
    return LaunchDescription([
        static_transform_launch,
        rsp_launch,
        ros2_control_node,  # 必须在spawn_controllers之前启动
        move_group_node,  # 手动创建的节点，确保 kinematics 和 controllers 都被正确传递
        spawn_controllers_launch,
        joint_state_publisher_node,  # 重新排序关节状态
        m5_planning,
    ])
