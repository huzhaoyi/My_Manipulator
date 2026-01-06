from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os
import sys
import contextlib

# 抑制 MoveItConfigsBuilder 的 SRDF 推断警告
# 这些警告来自 MoveIt 库内部，不影响功能
class WarningFilter:
    """过滤 stderr 中的 SRDF 推断警告"""
    def __init__(self, original):
        self.original = original
        
    def write(self, message):
        # 过滤掉 SRDF 推断警告和 cartesian_limits 弃用警告
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
    m5_bringup_dir = get_package_share_directory("m5_bringup")
    
    # 构建 MoveIt 配置，确保包含所有必要的配置
    # 使用绝对路径避免 MoveItConfigsBuilder 的路径推断警告
    # 注意：MoveItConfigsBuilder 在初始化时会尝试自动推断 SRDF 路径，产生警告
    # 这些警告不影响功能，我们已显式指定了正确的路径
    # 按照用户建议的方式显式指定 SRDF 路径
    srdf = os.path.join(m5_moveit_config_dir, "config", "m5.srdf")
    
    with suppress_moveit_warnings():
        moveit_config = (
            MoveItConfigsBuilder("m5", package_name="m5_moveit_config")
            .robot_description_semantic(file_path=srdf)  # 显式指定 SRDF 路径
            .robot_description_kinematics(file_path=os.path.join(m5_moveit_config_dir, "config", "kinematics.yaml"))
            .planning_pipelines(pipelines=["ompl"])  # 添加OMPL规划管道配置
            .joint_limits(file_path=os.path.join(m5_moveit_config_dir, "config", "joint_limits.yaml"))
            .sensors_3d(file_path=os.path.join(m5_moveit_config_dir, "config", "sensors_3d.yaml"))  # 添加传感器配置（禁用Octomap）
            .trajectory_execution(file_path=os.path.join(m5_moveit_config_dir, "config", "moveit_controllers.yaml"))  # 添加控制器配置
            .to_moveit_configs()
        )
    
    # 包含 static_transform_publisher launch（发布world到world_link的变换）
    static_transform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(m5_bringup_dir, "launch", "static_virtual_joint_tfs.launch.py")
        )
    )
    
    # 包含 robot_state_publisher launch（发布机器人状态）
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(m5_bringup_dir, "launch", "rsp.launch.py")
        )
    )
    
    # 包含 move_group launch
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(m5_bringup_dir, "launch", "move_group.launch.py")
        )
    )
    
    # 包含 RViz launch
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(m5_bringup_dir, "launch", "moveit_rviz.launch.py")
        )
    )
    
    return LaunchDescription([
        static_transform_launch,
        rsp_launch,
        move_group_launch,
        rviz_launch,
    ])
