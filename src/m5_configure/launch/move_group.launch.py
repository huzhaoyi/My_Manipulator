from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取包路径
    m5_configure_dir = get_package_share_directory("m5_configure")
    
    # 构建 MoveIt 配置，显式指定 kinematics.yaml 和 planning_pipelines
    moveit_config = (
        MoveItConfigsBuilder("m5", package_name="m5_configure")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl"])  # 明确指定使用OMPL规划管道
        .trajectory_execution(file_path="config/moveit_controllers.yaml")  # 添加控制器配置
        .to_moveit_configs()
    )
    
    # 生成 move_group launch，这会自动传递所有配置参数包括 kinematics
    move_group_launch = generate_move_group_launch(moveit_config)
    
    # 验证 kinematics 参数是否在配置中
    if hasattr(moveit_config, 'robot_description_kinematics'):
        print(f"[move_group.launch.py] Kinematics config loaded: {moveit_config.robot_description_kinematics}")
    else:
        print("[move_group.launch.py] WARNING: robot_description_kinematics not found in moveit_config!")
    
    return move_group_launch
