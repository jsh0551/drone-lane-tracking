#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2话题示例-订阅图像话题
"""

import rclpy                            # ROS2 Python接口库
from rclpy.node import Node             # ROS2 节点类
from sensor_msgs.msg import Image       # 图像消息类型
from cv_bridge import CvBridge          # ROS与OpenCV图像转换类
import cv2                              # Opencv图像处理库
import numpy as np             
import sys
import torch
import socket
import struct

# Load a model
lower_red = np.array([0, 90, 128])      # 红色的HSV阈值下限
upper_red = np.array([180, 255, 255])   # 红色的HSV阈值上限
sys.path.append(r"/home/base/ros_ws/customModule")
import DRL_model
import DQLL
det_model = DQLL.detector()
loc_model = DQLL.localizator()
PORT = 50100

class ImageSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)                                  # ROS2节点父类初始化
        self.sub = self.create_subscription(
            Image, 'image_raw', self.listener_callback, 10)     # 创建订阅者对象（消息类型、话题名、订阅者回调函数、队列长度）
        self.cv_bridge = CvBridge()
    
    def recvall(self, sock, count):
    # 바이트 데이터를 수신하여 반환
        buf = b''
        while count:
            newbuf = sock.recv(count)
            if not newbuf: return None
            buf += newbuf
            count -= len(newbuf)
        return buf

    def stream_camera(self, bgr_img):        # 图像从BGR颜色模型转换为HSV模型
        cv2.imshow("object", bgr_img)                             # 使用OpenCV显示处理后的图像效果
        cv2.waitKey(10)

    def object_localize(self, np_img, predictions):
        boxes = predictions[0]['boxes']
        if len(boxes)==0:
            return []
        labels = predictions[0]['labels']
        scores = predictions[0]['scores']
        boxes = boxes[scores>0.2].to(torch.int)
        
        np_boxes = boxes.cpu().detach().numpy()
        np_labels = labels.cpu().detach().numpy()
        crop_imgs = []
        crop_coords = []
        crop_ratios = []

        for b in np_boxes:
            x1,y1,x2,y2 = b
            w_ratio, h_ratio = (x2-x1)/100, (y2-y1)/100
            crop = np_img[y1:y2+1, x1:x2+1,:]
            crop = cv2.resize(crop, (100,100))
            crop_imgs.append(crop)
            crop_coords.append((x1,y1))
            crop_ratios.append((w_ratio,h_ratio))
        
        landmarks = loc_model.inference(crop_imgs, np_labels, crop_coords, crop_ratios)
        return landmarks
    
    def pointing(self, bgr_img, landmarks):
        if not landmarks:
            return bgr_img
        for landmark in landmarks:
            for p in landmark:
                x,y = p
                x,y = int(x), int(y)
                bgr_img = cv2.line(bgr_img,(x,y),(x,y),(0,0,255),5)
        return bgr_img
    
    def numpy_to_tensor(self, x):
        x = torch.Tensor(x.astype('float32')).permute(2,0,1)
        x = x.cuda() if torch.cuda.is_available() else x.cpu()
        x = [x/255]
        return x

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')         # 输出日志信息，提示已进入回调函数
        bgr_image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')
        rgb_image =  cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        tensor_image = self.numpy_to_tensor(rgb_image)

        predictions = det_model.inference(tensor_image)
        landmarks = self.object_localize(rgb_image, predictions)

        dotted_image = self.pointing(bgr_image, landmarks)
        self.stream_camera(dotted_image)                               
        # self.stream_camera(bgr_image)                               

def main(args=None):                                        # ROS2节点主入口main函数
    rclpy.init(args=args)                                   # ROS2 Python接口初始化
    node = ImageSubscriber("streaming_sub")              # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                        # 循环等待ROS2退出
    node.destroy_node()                                     # 销毁节点对象
    rclpy.shutdown()                                        # 关闭ROS2 Python接口
