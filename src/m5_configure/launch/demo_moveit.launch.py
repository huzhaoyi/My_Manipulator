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
        .to_moveit_configs()
    )
    
    # 包含 static_transform_publisher launch（发布world到dummy的变换）
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
    
    # 启动 ros2_control_node（控制器管理器，包含硬件接口）
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
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
    
    # 包含 spawn_controllers launch（启动硬件接口和控制器）
    spawn_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(m5_configure_dir, "launch", "spawn_controllers.launch.py")
        )
    )
    
    # 创建 demo_moveit 节点，并传递 MoveIt 配置参数
    demo_moveit_node = Node(
        package="m5_configure",
        executable="demo_moveit",
        name="demo_moveit",
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
        move_group_launch,
        spawn_controllers_launch,
        demo_moveit_node,
    ])
