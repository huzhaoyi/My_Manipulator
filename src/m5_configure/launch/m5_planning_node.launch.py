from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os


def generate_launch_description():
    # 获取包路径
    m5_configure_dir = get_package_share_directory("m5_configure")
    
    # 构建 MoveIt 配置，确保 kinematics.yaml 被正确加载
    moveit_config = (
        MoveItConfigsBuilder("m5", package_name="m5_configure")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .sensors_3d(file_path="config/sensors_3d.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )
    
    # 包含 static_transform_publisher launch（发布world到base_link的变换）
    static_transform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(m5_configure_dir, "launch", "static_virtual_joint_tfs.launch.py")
        )
    )
    
    # 包含 robot_state_publisher launch（发布机器人状态）
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(m5_configure_dir, "launch", "rsp.launch.py")
        )
    )
    
    # 启动 ros2_control_node（控制器管理器，包含硬件接口和UDP通信）
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,  # 硬件接口需要 robot_description 来初始化 UDP 配置
            os.path.join(m5_configure_dir, "config", "ros2_controllers.yaml"),
        ],
        output="screen",
    )
    
    # 包含 move_group launch，确保 kinematics.yaml 被正确加载
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(m5_configure_dir, "launch", "move_group.launch.py")
        )
    )
    
    # 包含 spawn_controllers launch（启动硬件接口和控制器，包括joint_state_broadcaster）
    spawn_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(m5_configure_dir, "launch", "spawn_controllers.launch.py")
        )
    )
    
    # 创建 joint_state_publisher_node，用于重新排序关节状态
    # 此节点订阅 joint_state_broadcaster 发布的消息，重新排序后发布到 /joint_states
    joint_state_publisher_node = Node(
        package="m5_configure",
        executable="joint_state_publisher_node",
        name="m5_joint_state_publisher",
        output="screen",
        parameters=[
            {"republish": True},
            {"source_topic": "/joint_states"},  # 订阅 joint_state_broadcaster 发布的话题
        ],
    )
    
    # 创建 m5_planning_node 节点，并传递 MoveIt 配置参数
    # 这样 MoveGroupInterface 在初始化时就能正确加载运动学配置
    m5_planning_node = Node(
        package="m5_configure",
        executable="m5_planning_node",
        name="m5_planning_node",
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
        ros2_control_node,  # 必须在spawn_controllers之前启动（启动UDP通信）
        move_group_launch,
        spawn_controllers_launch,  # 启动控制器和joint_state_broadcaster
        joint_state_publisher_node,  # 重新排序关节状态
        m5_planning_node,
    ])

