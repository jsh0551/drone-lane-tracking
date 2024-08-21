#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rclpy                        # ROS2 Python接口库
from rclpy.node import Node         # ROS2 节点类
from sensor_msgs.msg import Image   # 图像消息类型
from custom_msgs.msg import ImageWithInfo
from cv_bridge import CvBridge      # ROS与OpenCV图像转换类
import cv2                          # Opencv图像处理库
import socket
import struct
import numpy as np

PORT = 50101

class ImagePublisher(Node):

    def __init__(self, name):
        super().__init__(name)                                           # ROS2节点父类初始化
        self.publisher_ = self.create_publisher(Image, '/camera1/video_frames', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)         # 创建一个定时器（单位为秒的周期，定时执行的回调函数）

        self.cv_bridge = CvBridge()
        self.msg = Image()
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(('0.0.0.0', PORT))
        self.server_socket.listen(5)

        print("Listening for incoming connections...")

        self.client_socket, addr = self.server_socket.accept()
    
    
    def recvall(self, sock, count):
        buf = b''
        while count:
            newbuf = sock.recv(count)
            if not newbuf:
                return None
            buf += newbuf
            count -= len(newbuf)
        return buf

    def timer_callback(self):

        # length_buf = self.recvall(self.client_socket, 4)
        # length, = struct.unpack('>I', length_buf)
        
        # # 이미지 데이터 수신
        # data = self.recvall(self.client_socket, length)
        # pitch, roll = struct.unpack('>ff', self.client_socket.recv(8))
        data_size = struct.unpack('>I', self.client_socket.recv(4))[0]
        # 이미지 데이터와 각도 데이터 수신
        remaining = data_size + 12  # 이미지 데이터 길이 + float 2개 (pitch, roll)
        received_data = b''

        while len(received_data) < remaining:
            packet = self.client_socket.recv(4096)
            if not packet:
                break
            received_data += packet

        t = self.get_clock().now().to_msg().sec
        nt = self.get_clock().now().to_msg().nanosec/1e6
        self.get_logger().info(f't = {t}.{nt:.6f}')
        # 이미지 데이터와 각도 데이터 분리
        img_data = received_data[:data_size]
        if img_data is not None:
            yaw, pitch, roll = struct.unpack('>fff', received_data[data_size:data_size+12])
            nparr = np.frombuffer(img_data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            self.msg = self.cv_bridge.cv2_to_imgmsg(frame, 'bgr8')
            # self.msg.yaw = yaw
            # self.msg.pitch = pitch                                   
            # self.msg.roll = roll                                   
            self.publisher_.publish(self.msg)

            # self.get_logger().info(f'Publishing video frame ({frame.shape})')


def main(args=None):                                 # ROS2节点主入口main函数()
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = ImagePublisher("node_camera1")        # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口
