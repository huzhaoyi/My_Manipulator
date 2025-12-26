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
    
    # 包含 move_group launch，确保 kinematics.yaml 被正确加载
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(m5_configure_dir, "launch", "move_group.launch.py")
        )
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
        move_group_launch,
        m5_planning_node,
    ])

