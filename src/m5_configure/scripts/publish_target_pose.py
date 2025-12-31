#!/usr/bin/env python3
"""
发布目标位姿到 /target_pose 话题
用于测试 demo_moveit 节点
参考: arm_planner-main/src/fairino3_v6_planner/scripts/send_pose_goal.py

注意: 
- frame_id使用world_link（与RViz fixed frame一致，MoveIt会自动处理坐标系转换）
- 支持的坐标系：world（planning frame）、world_link（RViz fixed frame）、base_link
- 如果只提供位置(x, y, z)，orientation使用默认值(0,0,0,1)
- 如果提供orientation，将使用指定的orientation
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import sys


class TargetPosePublisher(Node):
    def __init__(self):
        super().__init__('target_pose_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.get_logger().info('目标位姿发布节点已启动')
        
    def publish_pose(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        """
        发布目标位姿
        参考: arm_planner-main/src/fairino3_v6_planner/scripts/send_pose_goal.py
        
        Args:
            x, y, z: 目标位置（必需）
            qx, qy, qz, qw: 目标orientation四元数（可选，默认为单位四元数）
        """
        msg = PoseStamped()
        # 参考成熟实现：使用当前时间戳
        msg.header.stamp = self.get_clock().now().to_msg()
        # 使用world_link作为frame_id（与RViz fixed frame一致）
        # MoveIt会自动处理坐标系转换（world/world_link/base_link -> planning frame）
        msg.header.frame_id = 'world_link'
        
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        
        msg.pose.orientation.x = float(qx)
        msg.pose.orientation.y = float(qy)
        msg.pose.orientation.z = float(qz)
        msg.pose.orientation.w = float(qw)
        
        self.publisher_.publish(msg)
        
        # 记录发布信息
        if qx == 0.0 and qy == 0.0 and qz == 0.0 and qw == 1.0:
            self.get_logger().info(
                f'已发布目标位姿: 位置 ({x:.3f}, {y:.3f}, {z:.3f}), '
                f'orientation使用默认值 (0, 0, 0, 1)'
            )
        else:
            self.get_logger().info(
                f'已发布目标位姿: 位置 ({x:.3f}, {y:.3f}, {z:.3f}), '
                f'四元数 ({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})'
            )


def main(args=None):
    rclpy.init(args=args)
    
    publisher = TargetPosePublisher()
    
    # 解析命令行参数
    if len(sys.argv) >= 4:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
        
        # 可选：如果提供了7个参数，则包含orientation
        if len(sys.argv) >= 8:
            qx = float(sys.argv[4])
            qy = float(sys.argv[5])
            qz = float(sys.argv[6])
            qw = float(sys.argv[7])
            publisher.publish_pose(x, y, z, qx, qy, qz, qw)
        else:
            # 只提供位置，使用默认orientation
            publisher.publish_pose(x, y, z)
    else:
        # 使用默认坐标（参考成熟实现的示例）
        # 默认坐标：机械臂前方可达位置
        x = 0.25
        y = 0.0
        z = 0.35
        
        print("使用默认坐标: x=0.25, y=0.0, z=0.35")
        print("\n用法:")
        print("  1. 只提供位置（orientation使用默认值）:")
        print("     python3 publish_target_pose.py <x> <y> <z>")
        print("  2. 提供位置和orientation:")
        print("     python3 publish_target_pose.py <x> <y> <z> <qx> <qy> <qz> <qw>")
        print("\n示例:")
        print("  python3 publish_target_pose.py 0.25 0.0 0.35")
        print("  python3 publish_target_pose.py 0.25 0.0 0.35 0.0 0.0 0.0 1.0")
        print("\n注意: frame_id使用world_link（与RViz fixed frame一致）")
        print("      MoveIt会自动处理坐标系转换（world/world_link/base_link -> planning frame）")
        
        # 发布默认位姿
        publisher.publish_pose(x, y, z)
    
    # 等待一下确保消息发送
    rclpy.spin_once(publisher, timeout_sec=0.1)
    
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
