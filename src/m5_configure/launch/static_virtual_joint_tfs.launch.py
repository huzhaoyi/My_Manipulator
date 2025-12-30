from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 手动创建 static_transform_publisher 节点，发布 world -> world_link 的变换
    # world_link 是 URDF 的根链接（无惯性），通过 world_joint 连接到 base_link
    # 使用新式命名参数格式（ROS2 Humble+）
    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "0",
            "--frame-id", "world",
            "--child-frame-id", "world_link"
        ],
        output="screen",
    )
    
    return LaunchDescription([
        static_transform_publisher,
    ])
