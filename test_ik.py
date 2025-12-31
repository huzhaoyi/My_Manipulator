#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time

class TestIKPublisher(Node):
    def __init__(self):
        super().__init__('test_ik_publisher')
        self.publisher = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.test_positions = [
            (0.184, 0.081, 0.398),  # ROS1 demo示例位置
            (0.300, 0.000, 0.300),  # 中心前方
            (0.250, 0.000, 0.350),  # 中心前方（高）
            (0.200, 0.000, 0.300),  # 前方近
        ]
        self.current_index = 0
        self.timer = self.create_timer(15.0, self.publish_next)

    def publish_next(self):
        if self.current_index < len(self.test_positions):
            x, y, z = self.test_positions[self.current_index]
            msg = PoseStamped()
            msg.header.frame_id = 'base_link'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = float(x)
            msg.pose.position.y = float(y)
            msg.pose.position.z = float(z)
            msg.pose.orientation.w = 1.0
            
            self.publisher.publish(msg)
            self.get_logger().info(f'发布目标位姿 {self.current_index + 1}/{len(self.test_positions)}: x={x:.3f} y={y:.3f} z={z:.3f}')
            self.current_index += 1
        else:
            self.get_logger().info('所有测试位置已发布完成')
            self.timer.cancel()

def main():
    rclpy.init()
    node = TestIKPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
