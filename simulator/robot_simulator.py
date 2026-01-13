#!/usr/bin/env python3
"""
机械臂上位机
- 接收机械臂的JSON反馈（关节角度、状态）
- 发送坐标到ROS2的/target_pose话题
- 管理并发送状态（规划中、执行中等）到机械臂
- 提供Web界面用于可视化
"""

import socket
import json
import threading
import time
import math
import atexit
import signal
import sys
from http.server import HTTPServer, SimpleHTTPRequestHandler
from socketserver import UDPServer
import os

# ROS2相关导入
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    try:
        from m5_msgs.msg import CablePoseWithYaw
        M5_MSGS_AVAILABLE = True
    except ImportError:
        M5_MSGS_AVAILABLE = False
        print("警告: m5_msgs未安装，xyz+yaw抓取功能将不可用")
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    M5_MSGS_AVAILABLE = False
    print("警告: ROS2未安装，坐标下发功能将不可用")

class RobotSimulator:
    def __init__(self, host='0.0.0.0', udp_port=7001, web_port=8080):
        self.host = host
        self.udp_port = udp_port
        self.web_port = web_port
        
        # 机械臂状态（5个轴，单位：度）
        self.axes = {
            1: 0.0,   # Joint1
            2: 0.0,   # Joint2
            3: 0.0,   # Joint3
            4: 0.0,   # Joint4
            5: 0.0    # 夹爪（axis5，范围：-1100° 到 0°）
        }
        
        self.status = 0  # 0=正常
        self.running = False
        
        # 状态管理（用于发送到机械臂）
        self.robot_state = "idle"  # idle, planning, executing, error, received, rejected, invalid
        self.planner_name = ""  # 当前规划器名称（如果有）
        self.last_state_update = time.time()
        self.last_state_received_time = None  # 最后一次收到状态的时间
        self.has_received_state = False  # 是否收到过状态消息
        
        # 抓取状态管理
        self.grasp_state = "idle"  # idle, received, planning:pregrasp, executing:pregrasp, etc.
        self.grasp_state_detail = ""  # 详细状态信息
        self.last_grasp_state_received_time = None
        self.has_received_grasp_state = False
        
        # UDP socket（仅用于发送状态到机械臂，不用于接收关节角度数据）
        # 关节角度数据应该从ROS2的/joint_states话题获取（由m5_hardware_interface发布）
        self.udp_socket = None
        self.udp_thread = None
        self.robot_addr = None  # 机械臂的地址（从接收到的UDP反馈中识别，用于发送状态）
        
        # 数据接收状态
        self.has_received_robot_data = False  # 是否收到过机械臂的JSON数据
        self.last_data_received_time = None  # 最后一次收到数据的时间
        
        # Web服务器
        self.web_server = None
        self.web_thread = None
        
                # ROS2节点、发布器和订阅器
        self.ros2_node = None
        self.pose_publisher = None
        self.cable_pose_publisher = None  # 缆绳位置发布器
        self.cable_pose_with_yaw_publisher = None  # 带yaw的缆绳位置发布器
        self.emergency_stop_publisher = None  # 急停发布器
        self.joint_state_subscriber = None
        self.arm_trajectory_publisher = None
        self.gripper_trajectory_publisher = None
        self.grasp_state_subscriber = None  # 抓取状态订阅器
        if ROS2_AVAILABLE:
            try:
                if not rclpy.ok():
                    rclpy.init()
                    print("✓ ROS2已初始化")
                else:
                    print("✓ ROS2已就绪")
                self.ros2_node = Node('robot_web_interface')
                print(f"✓ ROS2节点已创建: {self.ros2_node.get_name()}")
                self.pose_publisher = self.ros2_node.create_publisher(PoseStamped, '/target_pose', 10)
                print(f"✓ 发布器已创建: /target_pose")
                self.cable_pose_publisher = self.ros2_node.create_publisher(PoseStamped, '/cable_pose', 10)
                print(f"✓ 发布器已创建: /cable_pose")
                if M5_MSGS_AVAILABLE:
                    self.cable_pose_with_yaw_publisher = self.ros2_node.create_publisher(CablePoseWithYaw, '/cable_pose_with_yaw', 10)
                    print(f"✓ 发布器已创建: /cable_pose_with_yaw")
                else:
                    print("⚠ 发布器未创建: /cable_pose_with_yaw (m5_msgs不可用)")
                self.emergency_stop_publisher = self.ros2_node.create_publisher(String, '/emergency_stop', 10)
                print(f"✓ 急停发布器已创建: /emergency_stop (队列大小: 10)")
                
                # 等待一小段时间让发布器注册到ROS2
                time.sleep(0.1)
                print(f"✓ 急停发布器已注册到ROS2")
                
                # 订阅关节状态（从ROS2获取实时关节角度）
                self.joint_state_subscriber = self.ros2_node.create_subscription(
                    JointState,
                    '/joint_states',
                    self.joint_state_callback,
                    10
                )
                
                # 订阅机器人状态（从m5_planning获取规划/执行状态）
                self.robot_state_subscriber = self.ros2_node.create_subscription(
                    String,
                    '/robot_state',
                    self.robot_state_callback,
                    10
                )
                
                # 订阅抓取状态（从m5_grasp获取抓取状态）
                self.grasp_state_subscriber = self.ros2_node.create_subscription(
                    String,
                    '/grasp_state',
                    self.grasp_state_callback,
                    10
                )
                
                # 发布关节轨迹命令到控制器
                self.arm_trajectory_publisher = self.ros2_node.create_publisher(
                    JointTrajectory,
                    '/arm_group_controller/joint_trajectory',
                    10
                )
                self.gripper_trajectory_publisher = self.ros2_node.create_publisher(
                    JointTrajectory,
                    '/gripper_group_controller/joint_trajectory',
                    10
                )
                
                print("✓ ROS2节点已初始化")
                print("  - 发布话题: /target_pose, /cable_pose")
                print("  - 发布话题: /arm_group_controller/joint_trajectory")
                print("  - 发布话题: /gripper_group_controller/joint_trajectory")
                print("  - 订阅话题: /joint_states, /robot_state, /grasp_state")
            except Exception as e:
                print(f"警告: ROS2初始化失败: {e}")
                self.ros2_node = None
                self.pose_publisher = None
                self.joint_state_subscriber = None
                self.arm_trajectory_publisher = None
                self.gripper_trajectory_publisher = None
        
    def start_udp_server(self):
        """
        启动UDP服务器（仅用于识别机械臂地址和发送状态）
        注意：关节角度数据应该从ROS2的/joint_states话题获取（由m5_hardware_interface发布）
        """
        try:
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # 设置socket选项，允许地址重用
            self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.udp_socket.bind((self.host, self.udp_port))
            self.udp_socket.settimeout(1.0)
            
            print(f"✓ UDP服务器启动在 {self.host}:{self.udp_port}")
            print(f"  功能: 识别机械臂地址（用于发送状态），发送状态信息")
            print(f"  注意: 关节角度数据从ROS2的/joint_states话题获取（由m5_hardware_interface发布）")
            
            print(f"UDP服务器监听中，等待机械臂反馈（仅用于识别地址）...")
            recv_count = 0
            while self.running:
                try:
                    data, addr = self.udp_socket.recvfrom(4096)
                    recv_count += 1
                    if recv_count <= 5 or recv_count % 50 == 0:
                        print(f"[UDP接收 #{recv_count}] 收到 {len(data)} 字节 from {addr}（仅用于识别地址）")
                    # 保存机械臂地址，用于后续发送状态
                    if self.robot_addr is None:
                        self.robot_addr = addr
                        print(f"✓ 已识别机械臂地址: {addr}（用于发送状态）")
                    self.handle_robot_feedback(data, addr)
                except socket.timeout:
                    # 定期发送状态到机械臂
                    self.send_state_to_robot()
                    continue
                except socket.error as e:
                    if self.running:  # 只有在正常运行时才打印错误
                        print(f"UDP socket错误: {e}")
                    break
                except Exception as e:
                    if self.running:  # 只有在正常运行时才打印错误
                        print(f"UDP错误: {e}")
        except Exception as e:
            print(f"启动UDP服务器失败: {e}")
        finally:
            # 确保socket被关闭
            if self.udp_socket:
                try:
                    self.udp_socket.close()
                except:
                    pass
    
    def handle_robot_feedback(self, data, addr):
        """
        处理接收到的机械臂UDP反馈（仅用于识别机械臂地址）
        注意：关节角度数据应该从ROS2的/joint_states话题获取，而不是直接从这里获取
        """
        try:
            feedback_str = data.decode('utf-8').strip()
            feedback_data = json.loads(feedback_str)
            
            if not isinstance(feedback_data, list):
                feedback_data = [feedback_data]
            
            # 仅用于识别机械臂地址，不更新关节角度数据
            # 关节角度数据应该从ROS2的/joint_states话题获取（由m5_hardware_interface发布）
            for item in feedback_data:
                if 'status' in item:
                    # 可以更新机械臂状态（如果需要）
                    self.status = item['status']
            
        except json.JSONDecodeError as e:
            # JSON解析错误不影响，因为数据主要来自ROS2
            pass
        except Exception as e:
            # 错误不影响，因为数据主要来自ROS2
            pass
    
    def send_state_to_robot(self):
        """发送状态（规划中、执行中等）到机械臂"""
        if self.robot_addr is None:
            return
        
        # 定期发送状态（每1秒发送一次）
        current_time = time.time()
        if current_time - self.last_state_update < 1.0:
            return
        
        self.last_state_update = current_time
        
        try:
            state_msg = {
                "state": self.robot_state,
                "timestamp": current_time
            }
            message = json.dumps(state_msg)
            self.udp_socket.sendto(message.encode('utf-8'), self.robot_addr)
        except Exception as e:
            print(f"发送状态到机械臂失败: {e}")
    
    def send_joint_command(self, axis1, axis2, axis3, axis4, axis5):
        """
        发送关节角度命令到机械臂（通过ROS2控制器）
        
        Args:
            axis1-4: 关节角度（度），范围根据关节限制
            axis5: 夹爪开合度（度），范围：-1100° 到 0°
        
        Returns:
            dict: 包含成功状态和消息的字典
        """
        if not ROS2_AVAILABLE or self.ros2_node is None:
            return {
                "success": False,
                "message": "ROS2未初始化，无法发送关节角度命令"
            }
        
        if self.arm_trajectory_publisher is None or self.gripper_trajectory_publisher is None:
            return {
                "success": False,
                "message": "ROS2控制器发布器未初始化"
            }
        
        try:
            # 将度转换为弧度
            joint1_rad = math.radians(float(axis1))
            joint2_rad = math.radians(float(axis2))
            joint3_rad = math.radians(float(axis3))
            joint4_rad = math.radians(float(axis4))
            
            # 夹爪：axis5 [-1100°, 0°] -> JointGL/JointGR [-1.01, 1.01] 弧度
            axis5_min = -1100.0  # 度
            axis5_max = 0.0      # 度
            joint_min = -1.01    # 弧度
            joint_max = 1.01     # 弧度
            axis5_value = float(axis5)
            # 限制在有效范围内
            axis5_value = max(axis5_min, min(axis5_max, axis5_value))
            # 线性映射
            joint_gl_rad = (axis5_value - axis5_min) / (axis5_max - axis5_min) * (joint_max - joint_min) + joint_min
            joint_gr_rad = -joint_gl_rad  # 镜像对称
            
            # 创建机械臂轨迹消息
            arm_trajectory = JointTrajectory()
            arm_trajectory.header.stamp = self.ros2_node.get_clock().now().to_msg()
            arm_trajectory.header.frame_id = ""
            arm_trajectory.joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4']
            
            point = JointTrajectoryPoint()
            point.positions = [joint1_rad, joint2_rad, joint3_rad, joint4_rad]
            point.velocities = [0.0, 0.0, 0.0, 0.0]
            point.accelerations = []
            point.effort = []
            point.time_from_start = Duration(sec=1, nanosec=0)  # 1秒到达目标位置
            
            arm_trajectory.points = [point]
            
            # 创建夹爪轨迹消息
            gripper_trajectory = JointTrajectory()
            gripper_trajectory.header.stamp = self.ros2_node.get_clock().now().to_msg()
            gripper_trajectory.header.frame_id = ""
            gripper_trajectory.joint_names = ['JointGL', 'JointGR']
            
            gripper_point = JointTrajectoryPoint()
            gripper_point.positions = [joint_gl_rad, joint_gr_rad]
            gripper_point.velocities = [0.0, 0.0]
            gripper_point.accelerations = []
            gripper_point.effort = []
            gripper_point.time_from_start = Duration(sec=1, nanosec=0)  # 1秒到达目标位置
            
            gripper_trajectory.points = [gripper_point]
            
            # 发布轨迹命令
            self.arm_trajectory_publisher.publish(arm_trajectory)
            self.gripper_trajectory_publisher.publish(gripper_trajectory)
            
            print(f"[关节控制] 通过ROS2发送命令: J1={axis1:.2f}° J2={axis2:.2f}° J3={axis3:.2f}° J4={axis4:.2f}° 夹爪={axis5:.2f}°")
            
            return {
                "success": True,
                "message": f"已通过ROS2发送关节角度命令: J1={axis1:.2f}° J2={axis2:.2f}° J3={axis3:.2f}° J4={axis4:.2f}° 夹爪={axis5:.2f}°"
            }
        except Exception as e:
            error_msg = f"发送关节角度命令失败: {str(e)}"
            print(f"[关节控制] {error_msg}")
            import traceback
            traceback.print_exc()
            return {
                "success": False,
                "message": error_msg
            }
    
    def set_robot_state(self, state):
        """设置机器人状态（idle, planning, executing, error）"""
        if state != self.robot_state:
            print(f"[状态变更] {self.robot_state} -> {state}")
            self.robot_state = state
            self.last_state_update = time.time()  # 立即发送状态更新
            self.send_state_to_robot()
    
    def joint_state_callback(self, msg):
        """
        ROS2关节状态回调（从/joint_states话题获取）
        这是主要的数据源：m5_hardware_interface从真实机械臂UDP接收数据后，通过ROS2发布
        关节名称映射：Joint1 -> axis 1, Joint2 -> axis 2, etc.
        """
        joint_to_axis = {
            'Joint1': 1,
            'Joint2': 2,
            'Joint3': 3,
            'Joint4': 4,
        }
        
        # 标记已收到机械臂数据（从ROS2话题）
        if not self.has_received_robot_data:
            self.has_received_robot_data = True
            print("✓ 首次从ROS2 /joint_states收到机械臂数据")
        self.last_data_received_time = time.time()
        
        updated = False
        joint_gl_pos = None
        joint_gr_pos = None
        
        for i, joint_name in enumerate(msg.name):
            if joint_name in joint_to_axis:
                axis_num = joint_to_axis[joint_name]
                if i < len(msg.position):
                    # 将弧度转换为度
                    angle_deg = msg.position[i] * 180.0 / math.pi
                    old_value = self.axes.get(axis_num, 0.0)
                    self.axes[axis_num] = angle_deg
                    updated = True
                    # 只在有变化时打印（减少日志）
                    if abs(old_value - angle_deg) > 0.1:
                        print(f"[ROS2关节状态] axis{axis_num}: {old_value:.2f}° -> {angle_deg:.2f}°")
            elif joint_name == 'JointGL':
                if i < len(msg.position):
                    joint_gl_pos = msg.position[i]  # 弧度
            elif joint_name == 'JointGR':
                if i < len(msg.position):
                    joint_gr_pos = msg.position[i]  # 弧度
        
        # 处理夹爪（axis5）：将JointGL和JointGR映射到axis5
        # 反向映射：JointGL [-1.01, 1.01] 弧度 -> axis5 [-1100, 0] 度
        if joint_gl_pos is not None:
            joint_min = -1.01  # 弧度
            joint_max = 1.01   # 弧度
            axis5_min = -1100.0  # 度
            axis5_max = 0.0      # 度
            
            # 限制在有效范围内
            joint_gl_pos = max(joint_min, min(joint_max, joint_gl_pos))
            
            # 线性映射
            axis5_value = (joint_gl_pos - joint_min) / (joint_max - joint_min) * (axis5_max - axis5_min) + axis5_min
            old_value = self.axes.get(5, 0.0)
            self.axes[5] = axis5_value
            updated = True
            # 只在有变化时打印（减少日志）
            if abs(old_value - axis5_value) > 0.1:
                print(f"[ROS2关节状态] axis5 (夹爪): {old_value:.2f}° -> {axis5_value:.2f}°")
    
    def robot_state_callback(self, msg):
        """ROS2机器人状态回调（从m5_planning获取）"""
        state_data = msg.data
        # 支持格式：state 或 state:planner_name
        # 例如：planning:RRTConnect, error:RRTstar
        parts = state_data.split(':', 1)
        state = parts[0]
        planner_name = parts[1] if len(parts) > 1 else ""
        
        # 支持所有状态：idle, planning, executing, error, received, rejected, invalid
        valid_states = ['idle', 'planning', 'executing', 'error', 'received', 'rejected', 'invalid']
        if state in valid_states:
            if not self.has_received_state:
                self.has_received_state = True
                print("✓ 首次收到ROS2状态消息")
            self.last_state_received_time = time.time()
            self.planner_name = planner_name  # 保存规划器名称
            self.set_robot_state(state)
            if planner_name:
                print(f"[ROS2状态] 收到状态: {state} (规划器: {planner_name})")
            else:
                print(f"[ROS2状态] 收到状态: {state}")
        else:
            print(f"[ROS2状态] 收到未知状态: {state_data}")
    
    def grasp_state_callback(self, msg):
        """ROS2抓取状态回调（从m5_grasp获取）"""
        state_data = msg.data
        # 支持格式：state 或 state:detail
        # 例如：executing:descend, error:close_gripper
        parts = state_data.split(':', 1)
        state = parts[0]
        detail = parts[1] if len(parts) > 1 else ""
        
        if not self.has_received_grasp_state:
            self.has_received_grasp_state = True
            print("✓ 首次收到ROS2抓取状态消息")
        self.last_grasp_state_received_time = time.time()
        self.grasp_state = state
        self.grasp_state_detail = detail
        if detail:
            print(f"[ROS2抓取状态] 收到状态: {state} ({detail})")
        else:
            print(f"[ROS2抓取状态] 收到状态: {state}")
    
    def publish_cable_pose(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        """
        发布缆绳位置到 /cable_pose 话题
        
        Args:
            x, y, z: 缆绳位置（必需）
            qx, qy, qz, qw: 缆绳orientation四元数（可选，默认为单位四元数）
        
        Returns:
            dict: 包含成功状态和消息的字典
        """
        if not ROS2_AVAILABLE or self.cable_pose_publisher is None:
            print(f"[缆绳抓取] ROS2未初始化或发布器为None (ROS2_AVAILABLE={ROS2_AVAILABLE}, cable_pose_publisher={self.cable_pose_publisher})")
            return {
                "success": False,
                "message": "ROS2未初始化，无法发布缆绳位置"
            }
        
        try:
            msg = PoseStamped()
            msg.header.stamp = self.ros2_node.get_clock().now().to_msg()
            # 使用world_link作为frame_id（与publish_target_pose一致，MoveIt会自动处理坐标系转换）
            # 注意：m5_grasp会通过transform_pose_to_planning转换到planning frame
            msg.header.frame_id = 'world_link'
            
            msg.pose.position.x = float(x)
            msg.pose.position.y = float(y)
            msg.pose.position.z = float(z)
            
            msg.pose.orientation.x = float(qx)
            msg.pose.orientation.y = float(qy)
            msg.pose.orientation.z = float(qz)
            msg.pose.orientation.w = float(qw)
            
            print(f"[缆绳抓取] 准备发布消息: frame_id={msg.header.frame_id}, position=({x:.3f}, {y:.3f}, {z:.3f}), orientation=({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})")
            print(f"[缆绳抓取] 发布器状态: {self.cable_pose_publisher}, 节点状态: {self.ros2_node}, rclpy.ok(): {rclpy.ok()}")
            # ROS2发布是异步的，不需要spin_once，但需要确保节点上下文有效
            if not rclpy.ok():
                raise RuntimeError("ROS2上下文无效")
            if self.cable_pose_publisher is None:
                raise RuntimeError("缆绳位置发布器未初始化")
            self.cable_pose_publisher.publish(msg)
            print(f"[缆绳抓取] 消息已发布到 /cable_pose 话题 (frame_id={msg.header.frame_id})")
            
            log_msg = f'已发布缆绳位置: ({x:.3f}, {y:.3f}, {z:.3f})'
            print(f"[缆绳抓取] {log_msg}")
            
            return {
                "success": True,
                "message": log_msg
            }
        except Exception as e:
            error_msg = f"发布缆绳位置失败: {e}"
            print(f"[缆绳抓取错误] {error_msg}")
            return {
                "success": False,
                "message": error_msg
            }
    
    def publish_cable_pose_with_yaw(self, x, y, z, yaw):
        """
        发布带yaw的缆绳位置到 /cable_pose_with_yaw 话题
        
        Args:
            x, y, z: 缆绳位置（必需）
            yaw: 绕y轴的旋转角度（弧度），表示线缆切向方向
        
        Returns:
            dict: 包含成功状态和消息的字典
        """
        if not ROS2_AVAILABLE:
            error_msg = f"ROS2未初始化 (ROS2_AVAILABLE={ROS2_AVAILABLE})"
            print(f"[缆绳抓取xyz+yaw] {error_msg}")
            return {
                "success": False,
                "message": error_msg + "。请确保ROS2环境已正确设置。"
            }
        
        if not M5_MSGS_AVAILABLE:
            error_msg = f"m5_msgs未安装 (M5_MSGS_AVAILABLE={M5_MSGS_AVAILABLE})"
            print(f"[缆绳抓取xyz+yaw] {error_msg}")
            return {
                "success": False,
                "message": error_msg + "。请先构建m5_msgs包：cd src && colcon build --packages-select m5_msgs"
            }
        
        if self.cable_pose_with_yaw_publisher is None:
            error_msg = "发布器未初始化"
            print(f"[缆绳抓取xyz+yaw] {error_msg}")
            return {
                "success": False,
                "message": error_msg + "。请检查ROS2节点是否正常启动。"
            }
        
        try:
            msg = CablePoseWithYaw()
            msg.header.stamp = self.ros2_node.get_clock().now().to_msg()
            msg.header.frame_id = 'world_link'
            
            msg.position.x = float(x)
            msg.position.y = float(y)
            msg.position.z = float(z)
            msg.yaw = float(yaw)
            
            print(f"[缆绳抓取xyz+yaw] 准备发布消息: frame_id={msg.header.frame_id}, position=({x:.3f}, {y:.3f}, {z:.3f}), yaw={yaw:.3f} rad ({yaw*180.0/math.pi:.1f} deg)")
            
            if not rclpy.ok():
                raise RuntimeError("ROS2上下文无效")
            if self.cable_pose_with_yaw_publisher is None:
                raise RuntimeError("缆绳位置（带yaw）发布器未初始化")
            
            self.cable_pose_with_yaw_publisher.publish(msg)
            print(f"[缆绳抓取xyz+yaw] 消息已发布到 /cable_pose_with_yaw 话题 (frame_id={msg.header.frame_id})")
            
            log_msg = f'已发布缆绳位置（xyz+yaw）: ({x:.3f}, {y:.3f}, {z:.3f}), yaw={yaw*180.0/math.pi:.1f}°'
            print(f"[缆绳抓取xyz+yaw] {log_msg}")
            
            return {
                "success": True,
                "message": log_msg
            }
        except Exception as e:
            print(f"[缆绳抓取xyz+yaw] 发布失败: {e}")
            import traceback
            traceback.print_exc()
            return {
                "success": False,
                "message": f"发布缆绳位置（xyz+yaw）时出错: {str(e)}"
            }
    
    def publish_emergency_stop(self):
        """
        发布急停消息到 /emergency_stop 话题
        
        Returns:
            dict: 包含成功状态和消息的字典
        """
        if not ROS2_AVAILABLE or self.emergency_stop_publisher is None:
            print(f"[急停] ROS2未初始化或发布器为None")
            return {
                "success": False,
                "message": "ROS2未初始化，无法发布急停消息"
            }
        
        try:
            msg = String()
            msg.data = "EMERGENCY_STOP"
            
            print(f"[急停] ========================================")
            print(f"[急停] 准备发布急停消息到 /emergency_stop 话题")
            print(f"[急停] 发布器状态: {self.emergency_stop_publisher}")
            print(f"[急停] 节点状态: {self.ros2_node}")
            print(f"[急停] ROS2上下文: rclpy.ok()={rclpy.ok()}")
            
            # 检查订阅者数量
            if self.emergency_stop_publisher:
                try:
                    sub_count = self.emergency_stop_publisher.get_subscription_count()
                    print(f"[急停] 订阅者数量: {sub_count}")
                    if sub_count == 0:
                        print(f"[急停] ⚠️ 警告：没有订阅者！m5_grasp节点可能未订阅此话题")
                    else:
                        print(f"[急停] ✓ 检测到 {sub_count} 个订阅者")
                except Exception as e:
                    print(f"[急停] 无法获取订阅者数量: {e}")
            
            # 发布消息（ROS2发布是异步的）
            # 注意：由于ROS2节点在单独的spin线程中运行，消息会自动发送
            # 但为了确保消息立即发送，我们调用一次spin_once
            self.emergency_stop_publisher.publish(msg)
            print(f"[急停] 消息已发布到发布器，内容: {msg.data}")
            
            # 调用spin_once确保消息立即发送（即使有spin线程，这也确保消息队列被处理）
            try:
                rclpy.spin_once(self.ros2_node, timeout_sec=0.1)
                print(f"[急停] spin_once执行完成")
            except Exception as e:
                print(f"[急停] spin_once警告: {e}（可能正常，因为spin线程也在运行）")
            
            # 额外等待一小段时间确保消息通过ROS2网络传输
            time.sleep(0.2)
            
            print(f"[急停] 急停消息发布完成")
            print(f"[急停] ========================================")
            
            return {
                "success": True,
                "message": "急停消息已发布"
            }
        except Exception as e:
            error_msg = f"发布急停消息失败: {e}"
            print(f"[急停错误] {error_msg}")
            return {
                "success": False,
                "message": error_msg
            }
    
    def publish_target_pose(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        """
        发布目标位姿到 /target_pose 话题
        参考: src/m5_configure/scripts/publish_target_pose.py
        
        Args:
            x, y, z: 目标位置（必需）
            qx, qy, qz, qw: 目标orientation四元数（可选，默认为单位四元数）
        
        Returns:
            dict: 包含成功状态和消息的字典
        """
        if not ROS2_AVAILABLE or self.pose_publisher is None:
            print(f"[坐标下发] ROS2未初始化或发布器为None (ROS2_AVAILABLE={ROS2_AVAILABLE}, pose_publisher={self.pose_publisher})")
            return {
                "success": False,
                "message": "ROS2未初始化，无法发布位姿"
            }
        
        try:
            # 不再在这里设置状态，状态由C++端发布
            msg = PoseStamped()
            # 使用当前时间戳
            msg.header.stamp = self.ros2_node.get_clock().now().to_msg()
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
            
            print(f"[坐标下发] 准备发布消息: frame_id={msg.header.frame_id}, position=({x:.3f}, {y:.3f}, {z:.3f}), orientation=({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})")
            print(f"[坐标下发] 发布器状态: {self.pose_publisher}, 节点状态: {self.ros2_node}, rclpy.ok(): {rclpy.ok()}")
            # ROS2发布是异步的，不需要spin_once，但需要确保节点上下文有效
            if not rclpy.ok():
                raise RuntimeError("ROS2上下文无效")
            self.pose_publisher.publish(msg)
            print(f"[坐标下发] 消息已发布到 /target_pose 话题")
            
            # 记录发布信息
            if qx == 0.0 and qy == 0.0 and qz == 0.0 and qw == 1.0:
                log_msg = f'已发布目标位姿: 位置 ({x:.3f}, {y:.3f}, {z:.3f}), orientation使用默认值 (0, 0, 0, 1)'
            else:
                log_msg = f'已发布目标位姿: 位置 ({x:.3f}, {y:.3f}, {z:.3f}), 四元数 ({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})'
            
            print(f"[坐标下发] {log_msg}")
            
            return {
                "success": True,
                "message": log_msg
            }
        except Exception as e:
            error_msg = f"发布位姿失败: {e}"
            print(f"[坐标下发错误] {error_msg}")
            # 不再在这里设置状态，状态由C++端发布
            return {
                "success": False,
                "message": error_msg
            }
    
    def start_web_server(self):
        """启动Web服务器"""
        web_dir = os.path.join(os.path.dirname(__file__), 'web')
        if not os.path.exists(web_dir):
            os.makedirs(web_dir)
        original_dir = os.getcwd()
        os.chdir(web_dir)
        
        class CustomHandler(SimpleHTTPRequestHandler):
            def __init__(self, *args, simulator=None, original_dir=None, **kwargs):
                self.simulator = simulator
                self.original_dir = original_dir
                super().__init__(*args, **kwargs)
            
            def log_message(self, format, *args):
                # 禁用默认的访问日志打印（但保留404错误的日志）
                if '404' not in format % args:
                    pass
                else:
                    # 只打印404错误，帮助调试
                    print(f"[Web服务器] {format % args}")
            
            def log_error(self, format, *args):
                # 记录错误信息
                print(f"[Web服务器错误] {format % args}")
            
            def do_GET(self):
                if self.path == '/api/status':
                    self.send_response(200)
                    self.send_header('Content-type', 'application/json')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    
                    # 判断硬件状态：
                    # 1. 如果从未收到过数据，显示"未知"
                    # 2. 如果JSON中有status字段且不为0，显示"异常"
                    # 3. 否则显示"正常"
                    # 注意：不再判断超时，由ROS端持续上报状态
                    hardware_status = 0  # 默认正常
                    if not self.simulator.has_received_robot_data:
                        hardware_status = -1  # 未知（未收到过数据）
                    elif self.simulator.status != 0:
                        hardware_status = self.simulator.status  # 使用JSON中的status
                    
                    # 判断运行状态：
                    # 直接使用ROS上报的状态，不做超时判断
                    # 如果从未收到过状态，返回特殊值表示未知
                    # 如果ROS还在执行，应该持续上报状态（如executing），而不是由网页端判断超时
                    robot_state = self.simulator.robot_state
                    planner_name = self.simulator.planner_name  # 获取规划器名称
                    if not self.simulator.has_received_state:
                        robot_state = "__unknown__"  # 从未收到过状态
                    
                    # 获取抓取状态
                    grasp_state = self.simulator.grasp_state
                    grasp_state_detail = self.simulator.grasp_state_detail
                    if not self.simulator.has_received_grasp_state:
                        grasp_state = "__unknown__"
                    
                    status = {
                        "axes": self.simulator.axes.copy(),
                        "status": hardware_status,  # 使用计算后的硬件状态
                        "robot_state": robot_state,  # 使用计算后的运行状态
                        "planner_name": planner_name,  # 规划器名称（如果有）
                        "grasp_state": grasp_state,  # 抓取状态
                        "grasp_state_detail": grasp_state_detail,  # 抓取状态详情
                        "has_received_robot_data": self.simulator.has_received_robot_data,  # 是否收到过机械臂数据
                        "last_data_received_time": self.simulator.last_data_received_time,  # 最后一次收到数据的时间
                        "has_received_state": self.simulator.has_received_state,  # 是否收到过状态消息
                        "last_state_received_time": self.simulator.last_state_received_time,  # 最后一次收到状态的时间
                        "has_received_grasp_state": self.simulator.has_received_grasp_state,  # 是否收到过抓取状态
                        "last_grasp_state_received_time": self.simulator.last_grasp_state_received_time  # 最后一次收到抓取状态的时间
                    }
                    self.wfile.write(json.dumps(status).encode())
                elif self.path == '/api/urdf':
                    # 提供URDF文件
                    urdf_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'src', 'm5', 'urdf', 'm5_updated_from_csv.urdf')
                    if os.path.exists(urdf_path):
                        self.send_response(200)
                        self.send_header('Content-type', 'application/xml')
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.end_headers()
                        with open(urdf_path, 'rb') as f:
                            self.wfile.write(f.read())
                    else:
                        self.send_response(404)
                        self.end_headers()
                elif self.path == '/SEALIEN-LOGO.png' or self.path.endswith('SEALIEN-LOGO.png'):
                    # 提供LOGO文件访问
                    # 当前工作目录是web_dir，LOGO文件应该在web目录下
                    logo_path = 'SEALIEN-LOGO.png'  # 直接使用当前目录下的文件
                    
                    if os.path.exists(logo_path):
                        self.send_response(200)
                        self.send_header('Content-type', 'image/png')
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.end_headers()
                        with open(logo_path, 'rb') as f:
                            self.wfile.write(f.read())
                    else:
                        self.send_response(404)
                        self.end_headers()
                elif self.path == '/favicon.ico':
                    # 提供favicon（可选，避免404错误）
                    # 返回一个空的favicon响应，避免404错误
                    self.send_response(200)
                    self.send_header('Content-type', 'image/x-icon')
                    self.send_header('Content-Length', '0')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    # 不发送任何内容
                elif self.path.startswith('/meshes/') and self.path.endswith('.STL'):
                    # 提供STL文件访问 (meshes目录)
                    stl_filename = os.path.basename(self.path)
                    stl_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'src', 'm5', 'meshes', stl_filename)
                    if os.path.exists(stl_path):
                        self.send_response(200)
                        self.send_header('Content-type', 'application/octet-stream')
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.end_headers()
                        with open(stl_path, 'rb') as f:
                            self.wfile.write(f.read())
                    else:
                        self.send_response(404)
                        self.end_headers()
                elif self.path.startswith('/M5--三维模型.STL') or self.path.endswith('.STL'):
                    # 提供完整STL文件访问（向后兼容）
                    stl_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'M5--三维模型.STL')
                    if os.path.exists(stl_path):
                        self.send_response(200)
                        self.send_header('Content-type', 'application/octet-stream')
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.end_headers()
                        with open(stl_path, 'rb') as f:
                            self.wfile.write(f.read())
                    else:
                        self.send_response(404)
                        self.end_headers()
                else:
                    # 其他请求，使用默认的静态文件服务
                    # 当前工作目录已经是web_dir，SimpleHTTPRequestHandler会自动处理静态文件
                    try:
                        super().do_GET()
                    except Exception as e:
                        # 如果文件不存在，返回404
                        print(f"[Web服务器] 处理请求 {self.path} 时出错: {e}")
                        self.send_response(404)
                        self.end_headers()
            
            def do_POST(self):
                if self.path == '/api/publish_pose':
                    content_length = int(self.headers['Content-Length'])
                    post_data = self.rfile.read(content_length)
                    
                    try:
                        data = json.loads(post_data.decode('utf-8'))
                        print(f"[Web服务器] 收到位姿发布请求: {data}")
                        result = self.simulator.publish_target_pose(
                            data.get('x'), data.get('y'), data.get('z'),
                            data.get('qx', 0.0), data.get('qy', 0.0), 
                            data.get('qz', 0.0), data.get('qw', 1.0)
                        )
                        print(f"[Web服务器] 位姿发布结果: {result}")
                        
                        self.send_response(200)
                        self.send_header('Content-type', 'application/json')
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.end_headers()
                        self.wfile.write(json.dumps(result).encode())
                    except Exception as e:
                        self.send_response(400)
                        self.send_header('Content-type', 'application/json')
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.end_headers()
                        error_msg = {"error": str(e)}
                        self.wfile.write(json.dumps(error_msg).encode())
                elif self.path == '/api/send_joint_command':
                    content_length = int(self.headers['Content-Length'])
                    post_data = self.rfile.read(content_length)
                    
                    try:
                        data = json.loads(post_data.decode('utf-8'))
                        result = self.simulator.send_joint_command(
                            data.get('axis1', 0.0),
                            data.get('axis2', 0.0),
                            data.get('axis3', 0.0),
                            data.get('axis4', 0.0),
                            data.get('axis5', 0.0)
                        )
                        
                        self.send_response(200)
                        self.send_header('Content-type', 'application/json')
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.end_headers()
                        self.wfile.write(json.dumps(result).encode())
                    except Exception as e:
                        self.send_response(400)
                        self.send_header('Content-type', 'application/json')
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.end_headers()
                        error_msg = {"error": str(e)}
                        self.wfile.write(json.dumps(error_msg).encode())
                elif self.path == '/api/publish_cable_pose':
                    content_length = int(self.headers['Content-Length'])
                    post_data = self.rfile.read(content_length)
                    
                    try:
                        data = json.loads(post_data.decode('utf-8'))
                        print(f"[Web服务器] 收到缆绳位置发布请求: {data}")
                        result = self.simulator.publish_cable_pose(
                            data.get('x'), data.get('y'), data.get('z'),
                            data.get('qx', 0.0), data.get('qy', 0.0), 
                            data.get('qz', 0.0), data.get('qw', 1.0)
                        )
                        print(f"[Web服务器] 缆绳位置发布结果: {result}")
                        
                        self.send_response(200)
                        self.send_header('Content-type', 'application/json')
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.end_headers()
                        self.wfile.write(json.dumps(result).encode())
                    except Exception as e:
                        self.send_response(400)
                        self.send_header('Content-type', 'application/json')
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.end_headers()
                        error_msg = {"error": str(e)}
                        self.wfile.write(json.dumps(error_msg).encode())
                elif self.path == '/api/publish_cable_pose_with_yaw':
                    content_length = int(self.headers['Content-Length'])
                    post_data = self.rfile.read(content_length)
                    
                    try:
                        data = json.loads(post_data.decode('utf-8'))
                        print(f"[Web服务器] 收到缆绳位置（xyz+yaw）发布请求: {data}")
                        result = self.simulator.publish_cable_pose_with_yaw(
                            data.get('x'), data.get('y'), data.get('z'),
                            data.get('yaw')
                        )
                        print(f"[Web服务器] 缆绳位置（xyz+yaw）发布结果: {result}")
                        
                        self.send_response(200)
                        self.send_header('Content-type', 'application/json')
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.end_headers()
                        self.wfile.write(json.dumps(result).encode())
                    except Exception as e:
                        self.send_response(400)
                        self.send_header('Content-type', 'application/json')
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.end_headers()
                        error_msg = {"error": str(e)}
                        self.wfile.write(json.dumps(error_msg).encode())
                elif self.path == '/api/emergency_stop':
                    # 急停接口
                    try:
                        print(f"[Web服务器] 收到急停请求")
                        result = self.simulator.publish_emergency_stop()
                        print(f"[Web服务器] 急停发布结果: {result}")
                        
                        self.send_response(200)
                        self.send_header('Content-type', 'application/json')
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.end_headers()
                        self.wfile.write(json.dumps(result).encode())
                    except Exception as e:
                        self.send_response(400)
                        self.send_header('Content-type', 'application/json')
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.end_headers()
                        error_msg = {"error": str(e)}
                        self.wfile.write(json.dumps(error_msg).encode())
                # 移除 /api/set_state 接口，状态只能由C++端发布
                else:
                    self.send_response(404)
                    self.end_headers()
            
        
        handler = lambda *args, **kwargs: CustomHandler(*args, simulator=self, original_dir=original_dir, **kwargs)
        self.web_server = HTTPServer(('0.0.0.0', self.web_port), handler)
        
        # 获取本机IP地址（用于局域网访问）
        def get_local_ip():
            try:
                import socket
                # 连接到一个远程地址来获取本机IP（不会实际发送数据）
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.connect(('8.8.8.8', 80))
                ip = s.getsockname()[0]
                s.close()
                return ip
            except Exception:
                return 'localhost'
        
        local_ip = get_local_ip()
        print(f"✓ Web服务器启动在:")
        print(f"  - 本地访问: http://localhost:{self.web_port}")
        print(f"  - 局域网访问: http://{local_ip}:{self.web_port}")
        print(f"  - 所有接口: http://0.0.0.0:{self.web_port}")
        try:
            self.web_server.serve_forever()
        except Exception as e:
            if self.running:  # 只有在正常运行时才打印错误
                print(f"Web服务器错误: {e}")
        finally:
            os.chdir(original_dir)
            # 确保服务器完全关闭
            try:
                if self.web_server:
                    self.web_server.server_close()
            except:
                pass
    
    def start(self):
        """启动所有服务"""
        self.running = True
        
        # 启动ROS2 spin线程（如果可用）
        if self.ros2_node is not None:
            def ros2_spin():
                try:
                    rclpy.spin(self.ros2_node)
                except Exception as e:
                    print(f"ROS2 spin错误: {e}")
            ros2_thread = threading.Thread(target=ros2_spin, daemon=True)
            ros2_thread.start()
        
        # 启动UDP服务器
        self.udp_thread = threading.Thread(target=self.start_udp_server, daemon=True)
        self.udp_thread.start()
        
        # 启动Web服务器
        self.web_thread = threading.Thread(target=self.start_web_server, daemon=True)
        self.web_thread.start()
        
        # 获取本机IP地址（用于局域网访问）
        def get_local_ip():
            try:
                import socket
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.connect(('8.8.8.8', 80))
                ip = s.getsockname()[0]
                s.close()
                return ip
            except Exception:
                return 'localhost'
        
        local_ip = get_local_ip()
        
        print("\n" + "="*50)
        print("S3机械臂上位机已启动")
        print("="*50)
        print(f"UDP服务器: {self.host}:{self.udp_port}")
        print(f"Web界面:")
        print(f"  - 本地访问: http://localhost:{self.web_port}")
        print(f"  - 局域网访问: http://{local_ip}:{self.web_port}")
        if self.pose_publisher is not None:
            print(f"ROS2话题: /target_pose (frame_id: world_link)")
        print("="*50)
        print("\n按 Ctrl+C 停止\n")
        
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            self.stop()
    
    def stop(self):
        """停止所有服务"""
        if not self.running:
            return  # 已经停止过了
        
        print("\n正在停止服务器...")
        self.running = False
        
        # 停止Web服务器
        if self.web_server:
            try:
                print("  停止Web服务器...")
                self.web_server.shutdown()
                self.web_server.server_close()
                self.web_server = None
            except Exception as e:
                print(f"  停止Web服务器时出错: {e}")
        
        # 停止UDP服务器
        if self.udp_socket:
            try:
                print("  停止UDP服务器...")
                self.udp_socket.close()
                self.udp_socket = None
            except Exception as e:
                print(f"  停止UDP服务器时出错: {e}")
        
        # 等待线程结束（最多等待3秒）
        if self.udp_thread and self.udp_thread.is_alive():
            print("  等待UDP线程结束...")
            self.udp_thread.join(timeout=2.0)
        
        if self.web_thread and self.web_thread.is_alive():
            print("  等待Web线程结束...")
            self.web_thread.join(timeout=2.0)
        
        # 清理ROS2资源
        if self.ros2_node is not None:
            try:
                print("  清理ROS2资源...")
                # 先取消所有订阅和发布
                if hasattr(self, 'pose_publisher') and self.pose_publisher:
                    self.pose_publisher = None
                if hasattr(self, 'cable_pose_publisher') and self.cable_pose_publisher:
                    self.cable_pose_publisher = None
                if hasattr(self, 'joint_state_subscriber') and self.joint_state_subscriber:
                    self.joint_state_subscriber = None
                if hasattr(self, 'robot_state_subscriber') and self.robot_state_subscriber:
                    self.robot_state_subscriber = None
                if hasattr(self, 'grasp_state_subscriber') and self.grasp_state_subscriber:
                    self.grasp_state_subscriber = None
                if hasattr(self, 'emergency_stop_publisher') and self.emergency_stop_publisher:
                    self.emergency_stop_publisher = None
                if hasattr(self, 'arm_trajectory_publisher') and self.arm_trajectory_publisher:
                    self.arm_trajectory_publisher = None
                if hasattr(self, 'gripper_trajectory_publisher') and self.gripper_trajectory_publisher:
                    self.gripper_trajectory_publisher = None
                
                # 销毁节点
                self.ros2_node.destroy_node()
                self.ros2_node = None
                
                # 关闭ROS2
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception as e:
                print(f"  清理ROS2资源时出错: {e}")
        
        print("服务器已停止")

if __name__ == '__main__':
    simulator = None
    
    def signal_handler(signum, frame):
        """信号处理函数"""
        print(f"\n收到信号 {signum}，正在退出...")
        if simulator:
            simulator.stop()
        sys.exit(0)
    
    def cleanup_on_exit():
        """退出时清理资源"""
        if simulator:
            simulator.stop()
    
    # 注册信号处理
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # 注册退出处理
    atexit.register(cleanup_on_exit)
    
    try:
        simulator = RobotSimulator(host='0.0.0.0', udp_port=7001, web_port=8081)
        simulator.start()
    except KeyboardInterrupt:
        print("\n收到中断信号...")
        if simulator:
            simulator.stop()
    except Exception as e:
        print(f"\n发生错误: {e}")
        import traceback
        traceback.print_exc()
        if simulator:
            simulator.stop()
        sys.exit(1)
