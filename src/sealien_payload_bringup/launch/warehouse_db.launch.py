from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_warehouse_db_launch
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取包路径
    sealien_payload_moveit_config_dir = get_package_share_directory("sealien_payload_moveit_config")
    
    # 显式指定 SRDF 路径以避免警告
    # 按照用户建议的方式显式指定 SRDF 路径
    srdf = os.path.join(sealien_payload_moveit_config_dir, "config", "m5.srdf")
    
    moveit_config = (
        MoveItConfigsBuilder("m5", package_name="sealien_payload_moveit_config")
        .robot_description_semantic(file_path=srdf)
        .to_moveit_configs()
    )
    return generate_warehouse_db_launch(moveit_config)
