#!/usr/bin/env python3
"""
发布目标位姿到 /target_pose 话题
用于测试 demo_moveit 节点
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
        
    def publish_pose(self, x, y, z, qw=1.0, qx=0.0, qy=0.0, qz=0.0):
        """发布目标位姿"""
        msg = PoseStamped()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        
        msg.pose.orientation.w = float(qw)
        msg.pose.orientation.x = float(qx)
        msg.pose.orientation.y = float(qy)
        msg.pose.orientation.z = float(qz)
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'已发布目标位姿: x={x:.3f}, y={y:.3f}, z={z:.3f}, w={qw:.3f}')


def main(args=None):
    rclpy.init(args=args)
    
    # 从命令行参数获取坐标，如果没有则使用默认值
    if len(sys.argv) >= 4:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
        qw = float(sys.argv[4]) if len(sys.argv) > 4 else 1.0
        qx = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
        qy = float(sys.argv[6]) if len(sys.argv) > 6 else 0.0
        qz = float(sys.argv[7]) if len(sys.argv) > 7 else 0.0
    else:
        # 默认坐标（根据你的机械臂配置）
        x = 0.30
        y = 0.0
        z = 0.30
        qw = 1.0
        qx = 0.0
        qy = 0.0
        qz = 0.0
        print("使用默认坐标: x=0.30, y=0.0, z=0.30")
        print("用法: python3 publish_target_pose.py <x> <y> <z> [qw] [qx] [qy] [qz]")
    
    publisher = TargetPosePublisher()
    
    # 等待1秒确保连接建立
    rclpy.spin_once(publisher, timeout_sec=1.0)
    
    # 发布目标位姿
    publisher.publish_pose(x, y, z, qw, qx, qy, qz)
    
    # 等待一下确保消息发送
    rclpy.spin_once(publisher, timeout_sec=0.1)
    
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
