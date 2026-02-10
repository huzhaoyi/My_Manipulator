from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from ament_index_python.packages import get_package_share_directory
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
    sealien_payload_moveit_config_dir = get_package_share_directory("sealien_payload_moveit_config")
    
    # 构建 MoveIt 配置，显式指定 SRDF、kinematics.yaml 和 planning_pipelines
    # 使用绝对路径避免 MoveItConfigsBuilder 的路径推断警告
    # 显式指定 SRDF 路径，按照用户建议的方式
    srdf = os.path.join(sealien_payload_moveit_config_dir, "config", "m5.srdf")
    
    with suppress_moveit_warnings():
        moveit_config = (
            MoveItConfigsBuilder("m5", package_name="sealien_payload_moveit_config")
            .robot_description_semantic(file_path=srdf)  # 显式指定 SRDF 路径
            .robot_description_kinematics(file_path=os.path.join(sealien_payload_moveit_config_dir, "config", "kinematics.yaml"))
            .planning_pipelines(pipelines=["ompl"])  # 明确指定使用OMPL规划管道
            .trajectory_execution(file_path=os.path.join(sealien_payload_moveit_config_dir, "config", "moveit_controllers.yaml"))  # 添加控制器配置
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
