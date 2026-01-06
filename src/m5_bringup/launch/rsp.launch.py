from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch
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
    m5_moveit_config_dir = get_package_share_directory("m5_moveit_config")
    
    # 显式指定 SRDF 路径以避免警告
    # 按照用户建议的方式显式指定 SRDF 路径
    srdf = os.path.join(m5_moveit_config_dir, "config", "m5.srdf")
    
    with suppress_moveit_warnings():
        moveit_config = (
            MoveItConfigsBuilder("m5", package_name="m5_moveit_config")
            .robot_description_semantic(file_path=srdf)
            .to_moveit_configs()
        )
    return generate_rsp_launch(moveit_config)
