#!/usr/bin/env python3
"""
发布目标位置到 /target_pose 话题
用于测试 demo_moveit 节点
注意: 4DOF机械臂只需要位置坐标(x, y, z)，orientation由MoveIt自动确定
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
        self.get_logger().info('等待1秒后发布目标位姿...')
        
    def publish_pose(self, x, y, z):
        """发布目标位置（4DOF机械臂只需要位置，orientation由MoveIt自动确定）"""
        msg = PoseStamped()
        msg.header.frame_id = 'world_link'  # 使用 world_link 作为 planning frame
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        
        # 4DOF机械臂不需要设置orientation，MoveIt会自动找到合适的orientation
        # 设置为单位四元数（默认值）
        msg.pose.orientation.w = 1.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'已发布目标位置: x={x:.3f}, y={y:.3f}, z={z:.3f}')


def main(args=None):
    rclpy.init(args=args)
    
    # 从命令行参数获取坐标，如果没有则使用默认值
    if len(sys.argv) >= 4:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
    else:
        # 默认坐标（根据你的机械臂配置）
        x = 0.30
        y = 0.0
        z = 0.30
        print("使用默认坐标: x=0.30, y=0.0, z=0.30")
        print("用法: python3 publish_target_pose.py <x> <y> <z>")
        print("注意: 4DOF机械臂只需要位置坐标，orientation由MoveIt自动确定")
    
    publisher = TargetPosePublisher()
    
    # 等待1秒确保连接建立
    rclpy.spin_once(publisher, timeout_sec=1.0)
    
    # 发布目标位置
    publisher.publish_pose(x, y, z)
    
    # 等待一下确保消息发送
    rclpy.spin_once(publisher, timeout_sec=0.1)
    
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
