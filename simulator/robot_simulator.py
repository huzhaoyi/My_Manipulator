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
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
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
            5: 0.0    # 备用轴
        }
        
        self.status = 0  # 0=正常
        self.running = False
        
        # 状态管理（用于发送到机械臂）
        self.robot_state = "idle"  # idle, planning, executing, error, received, rejected, invalid
        self.last_state_update = time.time()
        self.last_state_received_time = None  # 最后一次收到状态的时间
        self.has_received_state = False  # 是否收到过状态消息
        
        # UDP socket（用于接收机械臂反馈和发送状态）
        self.udp_socket = None
        self.udp_thread = None
        self.robot_addr = None  # 机械臂的地址（从接收到的反馈中获取）
        
        # 数据接收状态
        self.has_received_robot_data = False  # 是否收到过机械臂的JSON数据
        self.last_data_received_time = None  # 最后一次收到数据的时间
        
        # Web服务器
        self.web_server = None
        self.web_thread = None
        
        # ROS2节点、发布器和订阅器
        self.ros2_node = None
        self.pose_publisher = None
        self.joint_state_subscriber = None
        if ROS2_AVAILABLE:
            try:
                if not rclpy.ok():
                    rclpy.init()
                self.ros2_node = Node('robot_web_interface')
                self.pose_publisher = self.ros2_node.create_publisher(PoseStamped, '/target_pose', 10)
                
                # 订阅关节状态（从ROS2获取实时关节角度）
                self.joint_state_subscriber = self.ros2_node.create_subscription(
                    JointState,
                    '/joint_states',
                    self.joint_state_callback,
                    10
                )
                
                # 订阅机器人状态（从demo_moveit获取规划/执行状态）
                self.robot_state_subscriber = self.ros2_node.create_subscription(
                    String,
                    '/robot_state',
                    self.robot_state_callback,
                    10
                )
                
                print("✓ ROS2节点已初始化")
                print("  - 发布话题: /target_pose")
                print("  - 订阅话题: /joint_states, /robot_state")
            except Exception as e:
                print(f"警告: ROS2初始化失败: {e}")
                self.ros2_node = None
                self.pose_publisher = None
                self.joint_state_subscriber = None
        
    def start_udp_server(self):
        """启动UDP服务器（接收机械臂反馈，发送状态）"""
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind((self.host, self.udp_port))
        self.udp_socket.settimeout(1.0)
        
        print(f"✓ UDP服务器启动在 {self.host}:{self.udp_port}")
        print(f"  功能: 接收机械臂反馈，发送状态信息")
        
        print(f"UDP服务器监听中，等待机械臂反馈...")
        recv_count = 0
        while self.running:
            try:
                data, addr = self.udp_socket.recvfrom(4096)
                recv_count += 1
                if recv_count <= 5 or recv_count % 50 == 0:
                    print(f"[UDP接收 #{recv_count}] 收到 {len(data)} 字节 from {addr}")
                # 保存机械臂地址，用于后续发送状态
                if self.robot_addr is None:
                    self.robot_addr = addr
                    print(f"✓ 已识别机械臂地址: {addr}")
                self.handle_robot_feedback(data, addr)
            except socket.timeout:
                # 定期发送状态到机械臂
                self.send_state_to_robot()
                continue
            except Exception as e:
                print(f"UDP错误: {e}")
    
    def handle_robot_feedback(self, data, addr):
        """处理接收到的机械臂反馈（关节角度、状态）"""
        try:
            feedback_str = data.decode('utf-8').strip()
            feedback_data = json.loads(feedback_str)
            
            if not isinstance(feedback_data, list):
                feedback_data = [feedback_data]
            
            # 标记已收到机械臂数据
            if not self.has_received_robot_data:
                self.has_received_robot_data = True
                print("✓ 首次收到机械臂JSON数据")
            self.last_data_received_time = time.time()
            
            # 更新轴位置和状态（从机械臂反馈中获取）
            updated = False
            for item in feedback_data:
                if 'num' in item and 'value' in item:
                    axis_num = item['num']
                    value = item['value']
                    if value is not None and axis_num in self.axes:
                        old_value = self.axes[axis_num]
                        self.axes[axis_num] = value
                        updated = True
                        # 只在有变化时打印（减少日志）
                        if abs(old_value - value) > 0.1:
                            print(f"[机械臂反馈] axis{axis_num}: {old_value:.2f}° -> {value:.2f}°")
                elif 'status' in item:
                    # 更新机械臂状态
                    self.status = item['status']
            
        except json.JSONDecodeError as e:
            print(f"JSON解析错误: {e}, 数据: {data[:100]}")
        except Exception as e:
            print(f"处理反馈错误: {e}")
    
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
    
    def set_robot_state(self, state):
        """设置机器人状态（idle, planning, executing, error）"""
        if state != self.robot_state:
            print(f"[状态变更] {self.robot_state} -> {state}")
            self.robot_state = state
            self.last_state_update = time.time()  # 立即发送状态更新
            self.send_state_to_robot()
    
    def joint_state_callback(self, msg):
        """ROS2关节状态回调（从/joint_states话题获取）"""
        # 如果UDP没有收到反馈，可以使用ROS2的关节状态作为备用
        # 关节名称映射：Joint1 -> axis 1, Joint2 -> axis 2, etc.
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
    
    def robot_state_callback(self, msg):
        """ROS2机器人状态回调（从demo_moveit获取）"""
        state = msg.data
        # 支持所有状态：idle, planning, executing, error, received, rejected, invalid
        valid_states = ['idle', 'planning', 'executing', 'error', 'received', 'rejected', 'invalid']
        if state in valid_states:
            if not self.has_received_state:
                self.has_received_state = True
                print("✓ 首次收到ROS2状态消息")
            self.last_state_received_time = time.time()
            self.set_robot_state(state)
            print(f"[ROS2状态] 收到状态: {state}")
        else:
            print(f"[ROS2状态] 收到未知状态: {state}")
    
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
            
            self.pose_publisher.publish(msg)
            
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
                    # 2. 如果收到过数据但超过5秒没更新，显示"超时"
                    # 3. 如果JSON中有status字段且不为0，显示"异常"
                    # 4. 否则显示"正常"
                    hardware_status = 0  # 默认正常
                    if not self.simulator.has_received_robot_data:
                        hardware_status = -1  # 未知（未收到过数据）
                    elif self.simulator.last_data_received_time is not None:
                        time_since_last = time.time() - self.simulator.last_data_received_time
                        if time_since_last > 5.0:  # 超过5秒没收到数据
                            hardware_status = -2  # 超时
                        elif self.simulator.status != 0:
                            hardware_status = self.simulator.status  # 使用JSON中的status
                    
                    # 判断运行状态：
                    # 如果从未收到过状态，返回特殊值表示未知
                    # 如果收到过但超过5秒没更新，返回特殊值表示超时
                    robot_state = self.simulator.robot_state
                    if not self.simulator.has_received_state:
                        robot_state = "__unknown__"  # 从未收到过状态
                    elif self.simulator.last_state_received_time is not None:
                        time_since_last = time.time() - self.simulator.last_state_received_time
                        if time_since_last > 5.0:  # 超过5秒没收到状态更新
                            robot_state = "__timeout__"  # 状态超时
                    
                    status = {
                        "axes": self.simulator.axes.copy(),
                        "status": hardware_status,  # 使用计算后的硬件状态
                        "robot_state": robot_state,  # 使用计算后的运行状态
                        "has_received_robot_data": self.simulator.has_received_robot_data,  # 是否收到过机械臂数据
                        "last_data_received_time": self.simulator.last_data_received_time,  # 最后一次收到数据的时间
                        "has_received_state": self.simulator.has_received_state,  # 是否收到过状态消息
                        "last_state_received_time": self.simulator.last_state_received_time  # 最后一次收到状态的时间
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
                        result = self.simulator.publish_target_pose(
                            data.get('x'), data.get('y'), data.get('z'),
                            data.get('qx', 0.0), data.get('qy', 0.0), 
                            data.get('qz', 0.0), data.get('qw', 1.0)
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
        finally:
            os.chdir(original_dir)
    
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
        print("\n正在停止服务器...")
        self.running = False
        if self.udp_socket:
            self.udp_socket.close()
        if self.web_server:
            self.web_server.shutdown()
        if self.ros2_node is not None:
            self.ros2_node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        print("服务器已停止")

if __name__ == '__main__':
    simulator = RobotSimulator(host='0.0.0.0', udp_port=7001, web_port=8081)
    simulator.start()
