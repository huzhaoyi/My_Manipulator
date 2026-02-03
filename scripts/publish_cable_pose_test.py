#!/usr/bin/env python3
"""
发布缆绳坐标测试脚本，并并发打印 /grasp_state 以观察抓取流程。
用法:
  先在一个终端运行: ./start_robot.sh
  再在本脚本所在目录或工作空间下运行:
    python3 scripts/publish_cable_pose_test.py [--once] [--x 0.25] [--y 0] [--z 0.25] [--yaw 0]
  --once: 只发送一次后退出（默认会每 3 秒发一次，便于观察流程）
  --watch-only: 只订阅 /grasp_state 不发布，用于单独看流程
"""

import argparse
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    RCLPY_AVAILABLE = True
except ImportError:
    RCLPY_AVAILABLE = False

try:
    from m5_msgs.msg import CablePoseWithYaw
    M5_MSGS_AVAILABLE = True
except ImportError:
    M5_MSGS_AVAILABLE = False


def main():
    parser = argparse.ArgumentParser(description="发布缆绳坐标并观察 /grasp_state 流程")
    parser.add_argument("--once", action="store_true", help="只发送一次坐标后退出")
    parser.add_argument("--watch-only", action="store_true", help="只订阅 /grasp_state，不发布")
    parser.add_argument("--x", type=float, default=0.25, help="缆绳 x (m)")
    parser.add_argument("--y", type=float, default=0.0, help="缆绳 y (m)")
    parser.add_argument("--z", type=float, default=0.25, help="缆绳 z (m)")
    parser.add_argument("--yaw", type=float, default=0.0, help="yaw (rad)，0 即 0°")
    parser.add_argument("--interval", type=float, default=3.0, help="连续发送时的间隔秒数")
    args = parser.parse_args()

    if not RCLPY_AVAILABLE:
        print("需要安装 ROS2 与 rclpy")
        return 1
    if not M5_MSGS_AVAILABLE and not args.watch_only:
        print("需要先 source 工作空间以便使用 m5_msgs")
        return 1

    rclpy.init()
    node = rclpy.create_node("publish_cable_pose_test")

    # 订阅 /grasp_state 用于观察流程
    last_state = [None]

    def on_grasp_state(msg):
        if last_state[0] != msg.data:
            last_state[0] = msg.data
            ts = time.strftime("%H:%M:%S", time.localtime())
            print(f"[{ts}] /grasp_state: {msg.data}")

    node.create_subscription(String, "/grasp_state", on_grasp_state, 10)
    pub = None
    if not args.watch_only and M5_MSGS_AVAILABLE:
        pub = node.create_publisher(CablePoseWithYaw, "/cable_pose_with_yaw", 10)
        print(f"将发布坐标: x={args.x}, y={args.y}, z={args.z}, yaw={args.yaw} rad")
        if args.once:
            print("模式: 发送一次后退出")
        else:
            print(f"模式: 每 {args.interval} 秒发送一次，Ctrl+C 停止")

    def publish_once():
        if not pub:
            return
        msg = CablePoseWithYaw()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = "world_link"
        msg.position.x = args.x
        msg.position.y = args.y
        msg.position.z = args.z
        msg.yaw = args.yaw
        pub.publish(msg)
        ts = time.strftime("%H:%M:%S", time.localtime())
        print(f"[{ts}] 已发布 /cable_pose_with_yaw: ({args.x}, {args.y}, {args.z}), yaw={args.yaw} rad")

    print("监听 /grasp_state，等待消息…")
    if args.watch_only:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
    else:
        if args.once:
            time.sleep(1.0)
            publish_once()
            for _ in range(20):
                rclpy.spin_once(node, timeout_sec=0.5)
            node.destroy_node()
            rclpy.shutdown()
            return 0
        next_pub = time.monotonic()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            t = time.monotonic()
            if t >= next_pub:
                publish_once()
                next_pub = t + args.interval

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
