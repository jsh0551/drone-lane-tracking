#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rclpy                        # ROS2 Python接口库
from rclpy.node import Node         # ROS2 节点类
from sensor_msgs.msg import Image   # 图像消息类型
from cv_bridge import CvBridge      # ROS与OpenCV图像转换类
import cv2                          # Opencv图像处理库

W=320
H=240
video_path = '/home/base/Workspace/test_video/20240304_152954.mp4'
class ImagePublisher(Node):

    def __init__(self, name):
        super().__init__(name)                                           # ROS2节点父类初始化
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)         # 创建一个定时器（单位为秒的周期，定时执行的回调函数）
        # self.cap = cv2.VideoCapture(-1)
        self.cap = cv2.VideoCapture(video_path)
        self.cap.set(cv2.CAP_PROP_FPS, 60)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)

        self.cv_bridge = CvBridge()                                      # 创建一个图像转换对象，用于稍后将OpenCV的图像转换成ROS的图像消息
        self.count = 0
    def timer_callback(self):

        ret, frame = self.cap.read()                                     # 一帧一帧读取图像
        if not ret:
        # 비디오의 처음으로 돌아가기
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        elif ret == True:                                                  # 如果图像读取成功
            self.publisher_.publish(
                self.cv_bridge.cv2_to_imgmsg(frame, 'bgr8'))

            self.get_logger().info('Publishing video frame')


def main(args=None):                                 # ROS2节点主入口main函数()
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = ImagePublisher("topic_webcam_pub")        # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口
