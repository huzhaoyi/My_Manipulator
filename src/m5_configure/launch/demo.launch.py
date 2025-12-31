from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    # 构建 MoveIt 配置，确保包含所有必要的配置
    # MoveItConfigsBuilder 会自动从 config/ 目录查找 m5.srdf 文件
    # 但为了确保 SRDF 被正确加载，显式指定 SRDF 文件路径
    moveit_config = (
        MoveItConfigsBuilder("m5", package_name="m5_configure")
        .robot_description_semantic(file_path="config/m5.srdf")  # 显式指定SRDF文件，确保规划组被正确加载
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl"])  # 添加OMPL规划管道配置
        .joint_limits(file_path="config/joint_limits.yaml")  # 显式指定关节限制文件
        .sensors_3d(file_path="config/sensors_3d.yaml")  # 添加传感器配置（禁用Octomap）
        .trajectory_execution(file_path="config/moveit_controllers.yaml")  # 添加控制器配置（修复moveit_controller_manager参数缺失问题）
        .to_moveit_configs()
    )
    return generate_demo_launch(moveit_config)
