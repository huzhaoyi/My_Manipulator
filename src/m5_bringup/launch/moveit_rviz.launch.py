from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取包路径
    m5_moveit_config_dir = get_package_share_directory("m5_moveit_config")
    
    # 构建 MoveIt 配置，显式指定 SRDF 文件，确保规划组被正确加载
    # 使用绝对路径避免 MoveItConfigsBuilder 的路径推断警告
    # 按照用户建议的方式显式指定 SRDF 路径
    srdf = os.path.join(m5_moveit_config_dir, "config", "m5.srdf")
    
    moveit_config = (
        MoveItConfigsBuilder("m5", package_name="m5_moveit_config")
        .robot_description_semantic(file_path=srdf)  # 显式指定 SRDF 路径
        .robot_description_kinematics(file_path=os.path.join(m5_moveit_config_dir, "config", "kinematics.yaml"))
        .to_moveit_configs()
    )
    return generate_moveit_rviz_launch(moveit_config)
