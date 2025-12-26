#!/usr/bin/env python3
"""
S3机械臂模拟器
模拟UDP服务器，接收JSON命令并返回反馈
同时提供Web界面用于可视化
"""

import socket
import json
import threading
import time
import math
from http.server import HTTPServer, SimpleHTTPRequestHandler
from socketserver import UDPServer
import os

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
        
        # UDP socket
        self.udp_socket = None
        self.udp_thread = None
        
        # Web服务器
        self.web_server = None
        self.web_thread = None
        
    def start_udp_server(self):
        """启动UDP服务器"""
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.bind((self.host, self.udp_port))
        self.udp_socket.settimeout(1.0)
        
        print(f"✓ UDP服务器启动在 {self.host}:{self.udp_port}")
        
        print(f"UDP服务器监听中，等待数据...")
        recv_count = 0
        while self.running:
            try:
                data, addr = self.udp_socket.recvfrom(4096)
                recv_count += 1
                if recv_count <= 5 or recv_count % 50 == 0:
                    print(f"[UDP接收 #{recv_count}] 收到 {len(data)} 字节 from {addr}")
                self.handle_command(data, addr)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"UDP错误: {e}")
                
    def handle_command(self, data, addr):
        """处理接收到的命令"""
        try:
            command_str = data.decode('utf-8').strip()
            commands = json.loads(command_str)
            
            if not isinstance(commands, list):
                commands = [commands]
            
            # 更新轴位置
            updated = False
            for cmd in commands:
                if 'num' in cmd and 'value' in cmd:
                    axis_num = cmd['num']
                    value = cmd['value']
                    if value is not None and axis_num in self.axes:
                        # 直接设置目标位置（不再限制速度，因为ROS2已经做了轨迹规划）
                        old_value = self.axes[axis_num]
                        self.axes[axis_num] = value
                        updated = True
                        # 只在有变化时打印（减少日志）
                        if abs(old_value - value) > 0.1:
                            print(f"[UDP接收] axis{axis_num}: {old_value:.2f}° -> {value:.2f}°")
                        
            if updated:
                # 发送反馈
                self.send_feedback(addr)
            
        except json.JSONDecodeError as e:
            print(f"JSON解析错误: {e}, 数据: {data[:100]}")
        except Exception as e:
            print(f"处理命令错误: {e}")
    
    def send_feedback(self, addr):
        """发送反馈数据"""
        feedback = []
        for axis_num in sorted(self.axes.keys()):
            feedback.append({
                "num": axis_num,
                "value": round(self.axes[axis_num], 2)
            })
        feedback.append({"status": self.status})
        
        response = json.dumps(feedback)
        self.udp_socket.sendto(response.encode('utf-8'), addr)
    
    def start_web_server(self):
        """启动Web服务器"""
        web_dir = os.path.join(os.path.dirname(__file__), 'web')
        if not os.path.exists(web_dir):
            os.makedirs(web_dir)
        original_dir = os.getcwd()
        os.chdir(web_dir)
        
        class CustomHandler(SimpleHTTPRequestHandler):
            def __init__(self, *args, simulator=None, **kwargs):
                self.simulator = simulator
                super().__init__(*args, **kwargs)
            
            def log_message(self, format, *args):
                # 禁用默认的访问日志打印
                pass
            
            def do_GET(self):
                if self.path == '/api/status':
                    self.send_response(200)
                    self.send_header('Content-type', 'application/json')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    status = {
                        "axes": self.simulator.axes.copy(),
                        "status": self.simulator.status
                    }
                    self.wfile.write(json.dumps(status).encode())
                else:
                    super().do_GET()
            
        
        handler = lambda *args, **kwargs: CustomHandler(*args, simulator=self, **kwargs)
        self.web_server = HTTPServer(('0.0.0.0', self.web_port), handler)
        print(f"✓ Web服务器启动在 http://0.0.0.0:{self.web_port}")
        try:
            self.web_server.serve_forever()
        finally:
            os.chdir(original_dir)
    
    def start(self):
        """启动所有服务"""
        self.running = True
        
        # 启动UDP服务器
        self.udp_thread = threading.Thread(target=self.start_udp_server, daemon=True)
        self.udp_thread.start()
        
        # 启动Web服务器
        self.web_thread = threading.Thread(target=self.start_web_server, daemon=True)
        self.web_thread.start()
        
        print("\n" + "="*50)
        print("S3机械臂模拟器已启动")
        print("="*50)
        print(f"UDP服务器: {self.host}:{self.udp_port}")
        print(f"Web界面: http://localhost:{self.web_port}")
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
        print("服务器已停止")

if __name__ == '__main__':
    simulator = RobotSimulator(host='0.0.0.0', udp_port=7001, web_port=8081)
    simulator.start()
