from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch_ros.actions import Node, SetRemap
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
    m5_moveit_config_dir = get_package_share_directory("m5_moveit_config")
    m5_bringup_dir = get_package_share_directory("m5_bringup")
    m5_grasp_dir = get_package_share_directory("m5_grasp")
    
    # 构建 MoveIt 配置
    srdf = os.path.join(m5_moveit_config_dir, "config", "m5.srdf")
    
    with suppress_moveit_warnings():
        moveit_config = (
            MoveItConfigsBuilder("m5", package_name="m5_moveit_config")
            .robot_description_semantic(file_path=srdf)
            .robot_description_kinematics(file_path=os.path.join(m5_moveit_config_dir, "config", "kinematics.yaml"))
            .sensors_3d(file_path=os.path.join(m5_moveit_config_dir, "config", "sensors_3d.yaml"))
            .trajectory_execution(file_path=os.path.join(m5_moveit_config_dir, "config", "moveit_controllers.yaml"))
            .to_moveit_configs()
        )
    
    # 包含 static_transform_publisher launch
    static_transform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(m5_bringup_dir, "launch", "static_virtual_joint_tfs.launch.py")
        )
    )
    
    # 包含 robot_state_publisher launch
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(m5_bringup_dir, "launch", "rsp.launch.py")
        )
    )
    
    # 启动 ros2_control_node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            os.path.join(m5_moveit_config_dir, "config", "ros2_controllers.yaml"),
        ],
        output="screen",
    )
    
    # 加载pilz_cartesian_limits.yaml并添加到robot_description_planning命名空间
    pilz_cartesian_limits_path = os.path.join(m5_moveit_config_dir, "config", "pilz_cartesian_limits.yaml")
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
        "planning_scene_monitor.octomap_resolution": 0.0,
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
    # 通过GroupAction和SetRemap，让joint_state_broadcaster发布到/joint_states_raw
    # 这样m5_joint_state_publisher可以订阅/joint_states_raw，发布到/joint_states，避免循环
    spawn_controllers_launch = GroupAction(
        actions=[
            SetRemap(src='/joint_states', dst='/joint_states_raw'),  # remap joint_state_broadcaster的输出
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(m5_bringup_dir, "launch", "spawn_controllers.launch.py")
                )
            )
        ]
    )
    
    # 创建 joint_state_publisher_node
    # 订阅 joint_state_broadcaster 发布的 /joint_states_raw（包含所有6个关节）
    # 重新排序后发布到 /joint_states，供 MoveIt 使用
    # 这样避免了循环：joint_state_broadcaster -> /joint_states_raw -> m5_joint_state_publisher -> /joint_states -> MoveIt
    joint_state_publisher_node = Node(
        package="m5_control",
        executable="joint_state_publisher_node",
        name="m5_joint_state_publisher",
        output="screen",
        parameters=[
            {"republish": True},
            {"source_topic": "/joint_states_raw"},  # 订阅 joint_state_broadcaster 的原始输出（已remap到/joint_states_raw）
            {"output_topic": "/joint_states"},  # 发布到 /joint_states，供 MoveIt 使用
        ],
    )
    
    # 创建 m5_grasp 节点，加载配置文件
    m5_grasp = Node(
        package="m5_grasp",
        executable="m5_grasp_node",
        name="m5_grasp",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            os.path.join(m5_grasp_dir, "config", "cable_grasp.yaml"),  # 加载缆绳抓取配置
        ],
    )
    
    # 包含 RViz launch（用于可视化规划）
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(m5_bringup_dir, "launch", "moveit_rviz.launch.py")
        )
    )
    
    return LaunchDescription([
        static_transform_launch,
        rsp_launch,
        ros2_control_node,
        move_group_node,
        spawn_controllers_launch,
        joint_state_publisher_node,
        m5_grasp,
        rviz_launch,
    ])
