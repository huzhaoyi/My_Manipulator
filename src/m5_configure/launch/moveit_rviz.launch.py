from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    # 构建 MoveIt 配置，显式指定 SRDF 文件，确保规划组被正确加载
    moveit_config = (
        MoveItConfigsBuilder("m5", package_name="m5_configure")
        .robot_description_semantic(file_path="config/m5.srdf")  # 显式指定SRDF文件
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )
    return generate_moveit_rviz_launch(moveit_config)
