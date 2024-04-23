#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rclpy                        # ROS2 Python接口库
from rclpy.node import Node         # ROS2 节点类
from sensor_msgs.msg import Image   # 图像消息类型
from cv_bridge import CvBridge      # ROS与OpenCV图像转换类
import cv2                          # Opencv图像处理库
import socket
import struct
import numpy as np

PORT = 50101

class ImagePublisher(Node):

    def __init__(self, name):
        super().__init__(name)                                           # ROS2节点父类初始化
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)         # 创建一个定时器（单位为秒的周期，定时执行的回调函数）

        self.cv_bridge = CvBridge()
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
        length_buf = self.recvall(self.client_socket, 4)
        length, = struct.unpack('>I', length_buf)
        
        # 이미지 데이터 수신
        data = self.recvall(self.client_socket, length)
        if data is not None:
            nparr = np.frombuffer(data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)                                    # 如果图像读取成功
            self.publisher_.publish(
                self.cv_bridge.cv2_to_imgmsg(frame, 'bgr8'))

            self.get_logger().info(f'Publishing video frame ({frame.shape})')


def main(args=None):                                 # ROS2节点主入口main函数()
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = ImagePublisher("topic_webcam_sub")        # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口
