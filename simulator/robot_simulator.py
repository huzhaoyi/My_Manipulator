#!/usr/bin/env python3
"""
机械臂上位机
- 接收机械臂的JSON反馈（关节角度、状态）
- 发送缆绳位置到ROS2的/cable_pose_with_yaw话题（抓取）
- 发送关节角度命令到ROS2控制器
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
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
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
        
        # 抓取状态管理
        self.grasp_state = "idle"
        self.grasp_state_detail = ""
        self.last_grasp_state_received_time = None
        self.has_received_grasp_state = False
        
        # 机械臂详细状态
        self.robot_enabled = False  # 机械臂使能状态
        self.robot_mode = "unknown"  # idle/auto/teleop
        self.emergency_stopped = False  # 急停状态
        self.driver_error = ""  # 驱动报错信息
        self.driver_error_code = 0  # 驱动错误码
        
        # 节点心跳：主控 = /heartbeat/m5_grasp，硬件 = 由 joint_states 推断（无独立心跳话题）
        self.node_heartbeats = {
            'm5_grasp': {'last_time': None, 'alive': False},
            'hardware': {'last_time': None, 'alive': False},
        }
        self.heartbeat_timeout = 3.0  # 心跳超时时间（秒）
        # 关节静止判定：用于“空闲=关节不变”时才能接受新目标
        self._last_joint_change_time = 0.0
        self._last_axes_for_stable = None
        self.joints_stable_interval = 0.5  # 连续 0.5s 关节无变化视为静止
        
        # 3D可视化数据
        self.planned_trajectory = []  # 规划轨迹 [{x, y, z}, ...]
        self.executed_trajectory = []  # 执行轨迹 [{x, y, z}, ...]
        self.ik_status = {
            'reachable': None,
            'confidence': 0.5,
            'message': ''
        }
        self.current_target = None  # 当前目标位置
        
        # UDP socket（仅用于发送状态到机械臂）
        self.udp_socket = None
        self.udp_thread = None
        self.robot_addr = None
        
        # 数据接收状态
        self.has_received_robot_data = False
        self.last_data_received_time = None
        
        # Web服务器
        self.web_server = None
        self.web_thread = None
        
        # ROS2节点、发布器和订阅器
        self.ros2_node = None
        self.cable_pose_with_yaw_publisher = None
        self.emergency_stop_publisher = None
        self.joint_state_subscriber = None
        self.arm_trajectory_publisher = None
        self.gripper_trajectory_publisher = None
        self.grasp_state_subscriber = None
        
        if ROS2_AVAILABLE:
            try:
                if not rclpy.ok():
                    rclpy.init()
                    print("✓ ROS2已初始化")
                else:
                    print("✓ ROS2已就绪")
                self.ros2_node = Node('robot_web_interface')
                print(f"✓ ROS2节点已创建: {self.ros2_node.get_name()}")
                
                # 缆绳抓取发布器（xyz+yaw）
                if M5_MSGS_AVAILABLE:
                    self.cable_pose_with_yaw_publisher = self.ros2_node.create_publisher(CablePoseWithYaw, '/cable_pose_with_yaw', 10)
                    print(f"✓ 发布器已创建: /cable_pose_with_yaw")
                else:
                    print("⚠ 发布器未创建: /cable_pose_with_yaw (m5_msgs不可用)")
                
                # 急停发布器
                self.emergency_stop_publisher = self.ros2_node.create_publisher(String, '/emergency_stop', 10)
                print(f"✓ 急停发布器已创建: /emergency_stop")
                
                time.sleep(0.1)
                
                # 订阅关节状态
                # /joint_states 是典型的 sensor-data 流：发布端（joint_state_broadcaster）为 BEST_EFFORT+VOLATILE，
                # 这里必须匹配，否则会触发 "incompatible QoS ... RELIABILITY" 并收不到关节状态。
                joint_states_qos = QoSProfile(
                    depth=10,
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                    durability=QoSDurabilityPolicy.VOLATILE,
                )
                self.joint_state_subscriber = self.ros2_node.create_subscription(
                    JointState,
                    '/joint_states',
                    self.joint_state_callback,
                    joint_states_qos,
                )
                
                # 订阅抓取状态
                self.grasp_state_subscriber = self.ros2_node.create_subscription(
                    String,
                    '/grasp_state',
                    self.grasp_state_callback,
                    10
                )
                
                # 订阅机械臂硬件状态
                self.hardware_state_subscriber = self.ros2_node.create_subscription(
                    String,
                    '/hardware_state',
                    self.hardware_state_callback,
                    10
                )
                
                # 订阅主控心跳（仅 /heartbeat/m5_grasp，硬件由 joint_states 推断）
                self.m5_grasp_heartbeat_subscriber = self.ros2_node.create_subscription(
                    String,
                    '/heartbeat/m5_grasp',
                    lambda msg: self.heartbeat_callback('m5_grasp', msg),
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
                
                # 订阅可视化相关话题
                self.planned_path_subscriber = self.ros2_node.create_subscription(
                    String,
                    '/visualization/planned_path',
                    self.planned_path_callback,
                    10
                )
                self.ik_status_subscriber = self.ros2_node.create_subscription(
                    String,
                    '/visualization/ik_status',
                    self.ik_status_callback,
                    10
                )
                
                print("✓ ROS2节点已初始化")
                if M5_MSGS_AVAILABLE:
                    print("  - 发布话题: /cable_pose_with_yaw")
                print("  - 发布话题: /emergency_stop")
                print("  - 发布话题: /arm_group_controller/joint_trajectory")
                print("  - 发布话题: /gripper_group_controller/joint_trajectory")
                print("  - 订阅话题: /joint_states, /grasp_state, /hardware_state")
                print("  - 订阅话题: /heartbeat/m5_grasp（主控存活）")
                print("  - 订阅话题: /visualization/planned_path, /visualization/ik_status")
            except Exception as e:
                print(f"警告: ROS2初始化失败: {e}")
                self.ros2_node = None
                self.cable_pose_with_yaw_publisher = None
                self.joint_state_subscriber = None
                self.arm_trajectory_publisher = None
                self.gripper_trajectory_publisher = None
        
    def start_udp_server(self):
        """启动UDP服务器（仅用于识别机械臂地址）"""
        try:
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.udp_socket.bind((self.host, self.udp_port))
            self.udp_socket.settimeout(1.0)
            
            print(f"✓ UDP服务器启动在 {self.host}:{self.udp_port}")
            print(f"  注意: 关节角度数据从ROS2的/joint_states话题获取")
            
            while self.running:
                try:
                    data, addr = self.udp_socket.recvfrom(4096)
                    if self.robot_addr is None:
                        self.robot_addr = addr
                        print(f"✓ 已识别机械臂地址: {addr}")
                    self.handle_robot_feedback(data, addr)
                except socket.timeout:
                    continue
                except socket.error as e:
                    if self.running:
                        print(f"UDP socket错误: {e}")
                    break
                except Exception as e:
                    if self.running:
                        print(f"UDP错误: {e}")
        except Exception as e:
            print(f"启动UDP服务器失败: {e}")
        finally:
            if self.udp_socket:
                try:
                    self.udp_socket.close()
                except:
                    pass
    
    def handle_robot_feedback(self, data, addr):
        """处理接收到的机械臂UDP反馈（仅用于识别机械臂地址）"""
        try:
            feedback_str = data.decode('utf-8').strip()
            feedback_data = json.loads(feedback_str)
            
            if not isinstance(feedback_data, list):
                feedback_data = [feedback_data]
            
            for item in feedback_data:
                if 'status' in item:
                    self.status = item['status']
            
        except json.JSONDecodeError:
            pass
        except Exception:
            pass
    
    def send_joint_command(self, axis1, axis2, axis3, axis4, axis5):
        """发送关节角度命令到机械臂（通过ROS2控制器）"""
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
            
            # 夹爪：axis5 [-1100, 0]（厂家实际）-> JointGL/JointGR [-0.9, 0.9] 弧度
            # axis5: -1100=完全打开, 0=完全闭合
            axis5_min = -1100.0  # 完全打开（厂家建议）
            axis5_max = 0.0      # 完全闭合
            joint_min = -0.9     # 打开角度 (JointGL)
            joint_max = 0.9      # 闭合角度 (JointGL)
            axis5_value = float(axis5)
            axis5_value = max(axis5_min, min(axis5_max, axis5_value))
            # 线性映射
            joint_gl_rad = (axis5_value - axis5_min) / (axis5_max - axis5_min) * (joint_max - joint_min) + joint_min
            joint_gr_rad = -joint_gl_rad
            
            # 两点轨迹：起点=当前（t=0 立即开始），终点=目标（t=0.4），避免单点轨迹被控制器解释为“延后执行”
            trajectory_duration_sec = 0.4
            now_stamp = self.ros2_node.get_clock().now().to_msg()
            # 当前机械臂关节（度→弧度），无反馈时用 0
            j1_now = math.radians(float(self.axes.get(1, 0.0)))
            j2_now = math.radians(float(self.axes.get(2, 0.0)))
            j3_now = math.radians(float(self.axes.get(3, 0.0)))
            j4_now = math.radians(float(self.axes.get(4, 0.0)))
            # 当前夹爪（axis5 → JointGL rad）
            a5_now = float(self.axes.get(5, axis5_min))
            a5_now = max(axis5_min, min(axis5_max, a5_now))
            gl_now = (a5_now - axis5_min) / (axis5_max - axis5_min) * (joint_max - joint_min) + joint_min
            gr_now = -gl_now

            arm_trajectory = JointTrajectory()
            arm_trajectory.header.stamp = now_stamp
            arm_trajectory.header.frame_id = ""
            arm_trajectory.joint_names = ['Joint1', 'Joint2', 'Joint3', 'Joint4']
            p0_arm = JointTrajectoryPoint()
            p0_arm.positions = [j1_now, j2_now, j3_now, j4_now]
            p0_arm.velocities = [0.0, 0.0, 0.0, 0.0]
            p0_arm.accelerations = []
            p0_arm.effort = []
            p0_arm.time_from_start = Duration(sec=0, nanosec=0)
            p1_arm = JointTrajectoryPoint()
            p1_arm.positions = [joint1_rad, joint2_rad, joint3_rad, joint4_rad]
            p1_arm.velocities = [0.0, 0.0, 0.0, 0.0]
            p1_arm.accelerations = []
            p1_arm.effort = []
            p1_arm.time_from_start = Duration(sec=int(trajectory_duration_sec), nanosec=int((trajectory_duration_sec % 1) * 1e9))
            arm_trajectory.points = [p0_arm, p1_arm]

            gripper_trajectory = JointTrajectory()
            gripper_trajectory.header.stamp = now_stamp
            gripper_trajectory.header.frame_id = ""
            gripper_trajectory.joint_names = ['JointGL', 'JointGR']
            p0_gr = JointTrajectoryPoint()
            p0_gr.positions = [gl_now, gr_now]
            p0_gr.velocities = [0.0, 0.0]
            p0_gr.accelerations = []
            p0_gr.effort = []
            p0_gr.time_from_start = Duration(sec=0, nanosec=0)
            p1_gr = JointTrajectoryPoint()
            p1_gr.positions = [joint_gl_rad, joint_gr_rad]
            p1_gr.velocities = [0.0, 0.0]
            p1_gr.accelerations = []
            p1_gr.effort = []
            p1_gr.time_from_start = Duration(sec=int(trajectory_duration_sec), nanosec=int((trajectory_duration_sec % 1) * 1e9))
            gripper_trajectory.points = [p0_gr, p1_gr]
            
            # 发布轨迹命令
            self.arm_trajectory_publisher.publish(arm_trajectory)
            self.gripper_trajectory_publisher.publish(gripper_trajectory)
            
            print(f"[关节控制] 通过ROS2发送命令: J1={axis1:.2f}° J2={axis2:.2f}° J3={axis3:.2f}° J4={axis4:.2f}° 夹爪={axis5:.2f}°")
            
            return {
                "success": True,
                "message": f"已发送关节角度命令: J1={axis1:.2f}° J2={axis2:.2f}° J3={axis3:.2f}° J4={axis4:.2f}° 夹爪={axis5:.2f}°"
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
    
    def joint_state_callback(self, msg):
        """ROS2关节状态回调（从/joint_states话题获取）"""
        joint_to_axis = {
            'Joint1': 1,
            'Joint2': 2,
            'Joint3': 3,
            'Joint4': 4,
        }
        
        if not self.has_received_robot_data:
            self.has_received_robot_data = True
            print("✓ 首次从ROS2 /joint_states收到机械臂数据")
        self.last_data_received_time = time.time()
        
        # 收到joint_states说明硬件在线（无独立心跳话题，由此推断）
        self.node_heartbeats['hardware']['last_time'] = time.time()
        self.node_heartbeats['hardware']['alive'] = True
        
        prev_axes = dict(self._last_axes_for_stable) if self._last_axes_for_stable is not None else None
        joint_gl_pos = None
        
        for i, joint_name in enumerate(msg.name):
            if joint_name in joint_to_axis:
                axis_num = joint_to_axis[joint_name]
                if i < len(msg.position):
                    angle_deg = msg.position[i] * 180.0 / math.pi
                    self.axes[axis_num] = angle_deg
            elif joint_name == 'JointGL':
                if i < len(msg.position):
                    joint_gl_pos = msg.position[i]
        
        # 处理夹爪（axis5）
        # JointGL: [-0.9, 0.9] rad -> axis5: [-1100, 0]（厂家实际）
        if joint_gl_pos is not None:
            joint_min = -0.9    # 打开
            joint_max = 0.9     # 闭合
            axis5_min = -1100.0  # 完全打开
            axis5_max = 0.0      # 完全闭合
            
            joint_gl_pos = max(joint_min, min(joint_max, joint_gl_pos))
            axis5_value = (joint_gl_pos - joint_min) / (joint_max - joint_min) * (axis5_max - axis5_min) + axis5_min
            self.axes[5] = axis5_value
        
        # 若关节与上次不同，更新“最后变化时间”；用于 joints_stable（空闲=关节不变才能接新目标）
        now = time.time()
        try:
            cur = {k: round(self.axes.get(k, 0), 2) for k in (1, 2, 3, 4, 5)}
            if prev_axes is not None:
                pr = {k: round(prev_axes.get(k, 0), 2) for k in (1, 2, 3, 4, 5)}
                if cur != pr:
                    self._last_joint_change_time = now
            else:
                self._last_joint_change_time = now
        except Exception:
            self._last_joint_change_time = now
        self._last_axes_for_stable = dict(self.axes)
    
    def grasp_state_callback(self, msg):
        """ROS2抓取状态回调（从m5_grasp获取）
        
        状态消息格式：
        - 简单状态: "已接收", "完成", "idle"
        - 带前缀状态: "执行:打开夹爪", "规划中:抓取", "错误:碰撞"
        - 带进度的执行状态: "执行:打开夹爪:1/5" (state="执行:打开夹爪", detail="1/5")
        """
        state_data = msg.data
        
        # 解析状态和detail
        # 格式可能是:
        # 1. "简单状态"
        # 2. "前缀:内容"
        # 3. "前缀:内容:detail" (如 "执行:打开夹爪:1/5")
        parts = state_data.split(':')
        
        if len(parts) >= 3 and parts[0] == '执行':
            # 执行状态带进度: "执行:步骤名:进度"
            state = parts[0] + ':' + parts[1]  # "执行:打开夹爪"
            detail = parts[2]  # "1/5"
        elif len(parts) >= 2:
            # 带前缀的状态: "前缀:内容"
            state = state_data
            detail = ""
            # 检查最后一部分是否像进度格式 "数字/数字"
            if '/' in parts[-1] and parts[-1].replace('/', '').isdigit():
                state = ':'.join(parts[:-1])
                detail = parts[-1]
        else:
            state = state_data
            detail = ""
        
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
        
        # 处理急停状态
        if state.startswith('急停:'):
            self.emergency_stopped = True
        elif state == 'idle' or state == '已接收':
            self.emergency_stopped = False
    
    def hardware_state_callback(self, msg):
        """ROS2硬件状态回调（从m5_hardware获取）"""
        try:
            # 解析硬件状态消息
            # 格式: "enabled:mode:error_code:error_msg" 或 JSON格式
            state_data = msg.data
            
            if state_data.startswith('{'):
                # JSON格式
                data = json.loads(state_data)
                self.robot_enabled = data.get('enabled', False)
                self.robot_mode = data.get('mode', 'unknown')
                self.emergency_stopped = data.get('emergency_stop', False)
                self.driver_error_code = data.get('error_code', 0)
                self.driver_error = data.get('error_msg', '')
            else:
                # 简单格式: "enabled:mode:error_code:error_msg"
                parts = state_data.split(':')
                if len(parts) >= 2:
                    self.robot_enabled = parts[0].lower() == 'true' or parts[0] == '1'
                    self.robot_mode = parts[1] if len(parts) > 1 else 'unknown'
                    self.driver_error_code = int(parts[2]) if len(parts) > 2 else 0
                    self.driver_error = parts[3] if len(parts) > 3 else ''
            
            # 更新controller心跳（收到硬件状态说明硬件节点活着）
            self.node_heartbeats['hardware']['last_time'] = time.time()
            self.node_heartbeats['hardware']['alive'] = True
            
        except Exception as e:
            print(f"[硬件状态] 解析失败: {e}, 原始数据: {msg.data}")
    
    def heartbeat_callback(self, node_name, msg):
        """节点心跳回调"""
        current_time = time.time()
        if node_name in self.node_heartbeats:
            self.node_heartbeats[node_name]['last_time'] = current_time
            self.node_heartbeats[node_name]['alive'] = True
    
    def update_heartbeat_status(self):
        """按当前时间重算在线/离线：超时未收到则置为离线（供网页显示实际状态）"""
        now = time.time()
        for node_name, info in self.node_heartbeats.items():
            if info['last_time'] is not None:
                if now - info['last_time'] > self.heartbeat_timeout:
                    info['alive'] = False

    def get_node_status(self):
        """返回主控用到的节点真实在线/离线状态，供网页直接展示（不造假）"""
        self.update_heartbeat_status()
        return {
            name: {
                'alive': bool(info['alive']),
                'last_time': info['last_time']
            }
            for name, info in self.node_heartbeats.items()
        }
    
    def is_joints_stable(self):
        """关节是否已静止（连续 joints_stable_interval 秒无变化），用于“空闲=关节不变”"""
        if not self.has_received_robot_data or self._last_axes_for_stable is None:
            return False
        return (time.time() - self._last_joint_change_time) >= self.joints_stable_interval
    
    def can_accept_new_target(self):
        """是否可接受新坐标点：FSM 空闲且关节已静止"""
        idle = self.grasp_state == 'FSM:IDLE' or self.grasp_state == 'idle'
        return idle and self.is_joints_stable()
    
    def planned_path_callback(self, msg):
        """规划路径回调（JSON格式：[{x,y,z}, ...]）"""
        try:
            path_data = json.loads(msg.data)
            if isinstance(path_data, list):
                self.planned_trajectory = path_data
                print(f"[可视化] 收到规划路径: {len(path_data)} 个点")
        except Exception as e:
            print(f"[可视化] 解析规划路径失败: {e}")
    
    def ik_status_callback(self, msg):
        """IK状态回调（JSON格式：{reachable, confidence, message}）"""
        try:
            status_data = json.loads(msg.data)
            self.ik_status = {
                'reachable': status_data.get('reachable'),
                'confidence': status_data.get('confidence', 0.5),
                'message': status_data.get('message', '')
            }
            reachable_str = "可达" if self.ik_status['reachable'] else "不可达"
            print(f"[可视化] IK状态更新: {reachable_str}, 置信度: {self.ik_status['confidence']:.2f}")
        except Exception as e:
            print(f"[可视化] 解析IK状态失败: {e}")
    
    def get_visualization_data(self):
        """获取可视化数据"""
        return {
            'planned_trajectory': self.planned_trajectory,
            'executed_trajectory': self.executed_trajectory,
            'ik_status': self.ik_status,
            'current_target': self.current_target
        }
    
    def publish_cable_pose_with_yaw(self, x, y, z, yaw):
        """发布带yaw的缆绳位置到 /cable_pose_with_yaw 话题"""
        if not ROS2_AVAILABLE:
            return {
                "success": False,
                "message": "ROS2未初始化"
            }
        
        if not M5_MSGS_AVAILABLE:
            return {
                "success": False,
                "message": "m5_msgs未安装。请先构建m5_msgs包：cd src && colcon build --packages-select m5_msgs"
            }
        
        if self.cable_pose_with_yaw_publisher is None:
            return {
                "success": False,
                "message": "发布器未初始化"
            }
        
        # 空闲=关节不变时才能接受新目标，否则拒绝并提示
        if not self.can_accept_new_target():
            if not (self.grasp_state == 'FSM:IDLE' or self.grasp_state == 'idle'):
                return {
                    "success": False,
                    "message": "机械臂未处于空闲状态，请等待当前任务完成后再发送坐标"
                }
            if not self.is_joints_stable():
                return {
                    "success": False,
                    "message": "机械臂未静止（关节仍在变化），请等待静止后再发送坐标"
                }
        
        try:
            msg = CablePoseWithYaw()
            msg.header.stamp = self.ros2_node.get_clock().now().to_msg()
            msg.header.frame_id = 'world_link'
            
            msg.position.x = float(x)
            msg.position.y = float(y)
            msg.position.z = float(z)
            msg.yaw = float(yaw)
            
            print(f"[缆绳抓取] 发布消息: position=({x:.3f}, {y:.3f}, {z:.3f}), yaw={yaw:.3f} rad ({yaw*180.0/math.pi:.1f} deg)")
            
            if not rclpy.ok():
                raise RuntimeError("ROS2上下文无效")
            
            self.cable_pose_with_yaw_publisher.publish(msg)
            
            log_msg = f'已发布缆绳位置: ({x:.3f}, {y:.3f}, {z:.3f}), yaw={yaw*180.0/math.pi:.1f}°'
            print(f"[缆绳抓取] {log_msg}")
            
            return {
                "success": True,
                "message": log_msg
            }
        except Exception as e:
            print(f"[缆绳抓取] 发布失败: {e}")
            import traceback
            traceback.print_exc()
            return {
                "success": False,
                "message": f"发布缆绳位置时出错: {str(e)}"
            }
    
    def publish_emergency_stop(self):
        """发布急停消息到 /emergency_stop 话题"""
        if not ROS2_AVAILABLE or self.emergency_stop_publisher is None:
            return {
                "success": False,
                "message": "ROS2未初始化，无法发布急停消息"
            }
        
        try:
            msg = String()
            msg.data = "EMERGENCY_STOP"
            
            print(f"[急停] 发布急停消息到 /emergency_stop 话题")
            
            self.emergency_stop_publisher.publish(msg)
            
            try:
                rclpy.spin_once(self.ros2_node, timeout_sec=0.1)
            except Exception:
                pass
            
            time.sleep(0.2)
            
            print(f"[急停] 急停消息发布完成")
            
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
                if '404' in format % args:
                    print(f"[Web服务器] {format % args}")
            
            def log_error(self, format, *args):
                print(f"[Web服务器错误] {format % args}")
            
            def do_GET(self):
                if self.path == '/api/status':
                    self.send_response(200)
                    self.send_header('Content-type', 'application/json')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    
                    hardware_status = 0
                    if not self.simulator.has_received_robot_data:
                        hardware_status = -1
                    elif self.simulator.status != 0:
                        hardware_status = self.simulator.status
                    
                    grasp_state = self.simulator.grasp_state
                    grasp_state_detail = self.simulator.grasp_state_detail
                    if not self.simulator.has_received_grasp_state:
                        grasp_state = "__unknown__"
                    
                    # 获取节点心跳状态
                    node_status = self.simulator.get_node_status()
                    
                    # 判断ROS2连接状态
                    ros2_connected = ROS2_AVAILABLE and self.simulator.ros2_node is not None
                    
                    status = {
                        "axes": self.simulator.axes.copy(),
                        "status": hardware_status,
                        "grasp_state": grasp_state,
                        "grasp_state_detail": grasp_state_detail,
                        "has_received_robot_data": self.simulator.has_received_robot_data,
                        "last_data_received_time": self.simulator.last_data_received_time,
                        "has_received_grasp_state": self.simulator.has_received_grasp_state,
                        "last_grasp_state_received_time": self.simulator.last_grasp_state_received_time,
                        # 系统状态详情
                        "ros2_connected": ros2_connected,
                        "node_heartbeats": node_status,
                        "robot_enabled": self.simulator.robot_enabled,
                        "robot_mode": self.simulator.robot_mode,
                        "emergency_stopped": self.simulator.emergency_stopped,
                        "driver_error_code": self.simulator.driver_error_code,
                        "driver_error": self.simulator.driver_error,
                        # 空闲=关节不变：用于“未静止不能发新目标”的弹框
                        "joints_stable": self.simulator.is_joints_stable(),
                        "can_accept_new_target": self.simulator.can_accept_new_target(),
                    }
                    self.wfile.write(json.dumps(status).encode())
                elif self.path == '/api/visualization':
                    self.send_response(200)
                    self.send_header('Content-type', 'application/json')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    
                    viz_data = self.simulator.get_visualization_data()
                    self.wfile.write(json.dumps(viz_data).encode())
                elif self.path == '/api/urdf':
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
                    logo_path = 'SEALIEN-LOGO.png'
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
                    self.send_response(200)
                    self.send_header('Content-type', 'image/x-icon')
                    self.send_header('Content-Length', '0')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                elif self.path.startswith('/meshes/') and self.path.endswith('.STL'):
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
                else:
                    try:
                        super().do_GET()
                    except Exception as e:
                        print(f"[Web服务器] 处理请求 {self.path} 时出错: {e}")
                        self.send_response(404)
                        self.end_headers()
            
            def do_POST(self):
                if self.path == '/api/send_joint_command':
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
                else:
                    self.send_response(404)
                    self.end_headers()
        
        handler = lambda *args, **kwargs: CustomHandler(*args, simulator=self, original_dir=original_dir, **kwargs)
        self.web_server = HTTPServer(('0.0.0.0', self.web_port), handler)
        
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
        print(f"✓ Web服务器启动在:")
        print(f"  - 本地访问: http://localhost:{self.web_port}")
        print(f"  - 局域网访问: http://{local_ip}:{self.web_port}")
        try:
            self.web_server.serve_forever()
        except Exception as e:
            if self.running:
                print(f"Web服务器错误: {e}")
        finally:
            os.chdir(original_dir)
            try:
                if self.web_server:
                    self.web_server.server_close()
            except:
                pass
    
    def start(self):
        """启动所有服务"""
        self.running = True
        
        # 启动ROS2 spin线程
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
        print("功能:")
        print("  - 缆绳抓取 (/cable_pose_with_yaw)")
        print("  - 关节角度控制 (/arm_group_controller/joint_trajectory)")
        print("  - 急停 (/emergency_stop)")
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
            return
        
        print("\n正在停止服务器...")
        self.running = False
        
        if self.web_server:
            try:
                print("  停止Web服务器...")
                self.web_server.shutdown()
                self.web_server.server_close()
                self.web_server = None
            except Exception as e:
                print(f"  停止Web服务器时出错: {e}")
        
        if self.udp_socket:
            try:
                print("  停止UDP服务器...")
                self.udp_socket.close()
                self.udp_socket = None
            except Exception as e:
                print(f"  停止UDP服务器时出错: {e}")
        
        if self.udp_thread and self.udp_thread.is_alive():
            print("  等待UDP线程结束...")
            self.udp_thread.join(timeout=2.0)
        
        if self.web_thread and self.web_thread.is_alive():
            print("  等待Web线程结束...")
            self.web_thread.join(timeout=2.0)
        
        if self.ros2_node is not None:
            try:
                print("  清理ROS2资源...")
                self.cable_pose_with_yaw_publisher = None
                self.joint_state_subscriber = None
                self.grasp_state_subscriber = None
                self.emergency_stop_publisher = None
                self.arm_trajectory_publisher = None
                self.gripper_trajectory_publisher = None
                
                self.ros2_node.destroy_node()
                self.ros2_node = None
                
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception as e:
                print(f"  清理ROS2资源时出错: {e}")
        
        print("服务器已停止")

if __name__ == '__main__':
    simulator = None
    
    def signal_handler(signum, frame):
        print(f"\n收到信号 {signum}，正在退出...")
        if simulator:
            simulator.stop()
        sys.exit(0)
    
    def cleanup_on_exit():
        if simulator:
            simulator.stop()
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
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
