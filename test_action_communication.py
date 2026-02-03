#!/usr/bin/env python3
"""
测试 Action 通信的 Python 脚本
用于验证 FastDDS 配置是否解决了回包丢失问题
"""

import rclpy
from rclpy.action import ActionClient
from moveit_task_constructor_msgs.action import ExecuteTaskSolution
import sys
import time

def test_action_communication():
    """测试 action 通信"""
    rclpy.init()
    
    node = rclpy.create_node('test_action_client')
    
    # 创建 action client
    action_client = ActionClient(node, ExecuteTaskSolution, '/execute_task_solution')
    
    print("等待 action server 可用...")
    if not action_client.wait_for_server(timeout_sec=10.0):
        print("❌ Action server 不可用（10秒内未响应）")
        return False
    
    print("✓ Action server 可用")
    
    # 创建一个空的 goal（仅用于测试连接）
    # 注意：ExecuteTaskSolution.Goal 的 solution 字段是 moveit_task_constructor_msgs.msg.Solution 类型
    from moveit_task_constructor_msgs.msg import Solution
    goal_msg = ExecuteTaskSolution.Goal()
    goal_msg.solution = Solution()  # 空的 solution
    
    print("发送 goal...")
    send_goal_future = action_client.send_goal_async(goal_msg)
    
    # 等待 goal response
    print("等待 goal response...")
    rclpy.spin_until_future_complete(node, send_goal_future, timeout_sec=5.0)
    
    if send_goal_future.done():
        goal_handle = send_goal_future.result()
        if goal_handle is not None:
            print("✓✓✓ Goal response 收到！goal_handle 非空")
            print(f"   Status: {goal_handle.status}")
            return True
        else:
            print("❌ Goal response 收到，但 goal_handle 为空（被拒绝）")
            return False
    else:
        print("❌ 等待 goal response 超时（5秒）")
        print("   这说明 action 回包没有返回，可能是 DDS 网络配置问题")
        return False

if __name__ == '__main__':
    try:
        success = test_action_communication()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n测试被中断")
        sys.exit(1)
    finally:
        rclpy.shutdown()
