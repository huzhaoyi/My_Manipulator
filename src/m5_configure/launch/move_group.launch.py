from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    # 构建 MoveIt 配置，显式指定 kinematics.yaml
    moveit_config = (
        MoveItConfigsBuilder("m5", package_name="m5_configure")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
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
