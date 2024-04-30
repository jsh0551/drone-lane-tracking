#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2话题示例-订阅图像话题
"""

import rclpy                            # ROS2 Python接口库
from rclpy.node import Node             # ROS2 节点类
from sensor_msgs.msg import Image
from custom_msgs.msg import ImageWithInfo       # 图像消息类型
from cv_bridge import CvBridge          # ROS与OpenCV图像转换类
import cv2                              # Opencv图像处理库
import numpy as np                      # Python数值计算库
import sys
import math
import torch
# Load a model
lower_red = np.array([0, 90, 128])      # 红色的HSV阈值下限
upper_red = np.array([180, 255, 255])   # 红色的HSV阈值上限
sys.path.append(r"/home/base/Workspace/ros_ws/customModule")
sys.path.append(r"/home/base/Workspace/ros_ws/customModule/yolox")
import DQLL
from models.yolox import Detector
from yolox_config import opt
det_model = Detector(opt)
loc_model = DQLL.localizator()
WIDTH, HEIGHT = 480, 360
s_points = np.array([(0,HEIGHT),(WIDTH,HEIGHT),(WIDTH/4,HEIGHT),(WIDTH*3/4,HEIGHT),(0,HEIGHT*3/4),(WIDTH,HEIGHT*3/4)]).astype(np.int16)

def cal_slope(bgr_img, yaw, pitch, roll, f=240.0):
    '''
    x : track lane의 중점으로부터의 거리(px)
    w,h : 이미지 너비,높이(px)
    alpha : 카메라 기울기 각도. 바닥과 수평시 0도 (radian?)
    f : 초점거리
    '''
    cx = WIDTH/2
    cy = HEIGHT/2 * (1+np.tan(pitch))
    for tmp_x, tmp_y in s_points:
        m = (cy-tmp_y) / (cx-tmp_x)
        uy = int(max(0,cy))
        ux = int((uy-cy)/m + cx)
        bgr_img = cv2.line(bgr_img,(tmp_x,tmp_y),(ux,uy),(0,255,255),2)
    # h/2나 2h/3에서 만나는 점 구하기.
    return bgr_img

class ImageSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)                                  # ROS2节点父类初始化
        self.sub = self.create_subscription(
            ImageWithInfo, 'image_raw', self.listener_callback, 10)     # 创建订阅者对象（消息类型、话题名、订阅者回调函数、队列长度）
        self.cv_bridge = CvBridge()

    def stream_camera(self, bgr_img):        # 图像从BGR颜色模型转换为HSV模型
        cv2.imshow("object", bgr_img)                             # 使用OpenCV显示处理后的图像效果
        cv2.waitKey(10)

    def object_localize(self, np_img, predictions):
        if len(predictions)==0:
            return []
        
        np_labels = []
        crop_imgs = []
        crop_coords = []
        crop_ratios = []

        for label, conf, b in predictions:
            x1,y1,x2,y2 = np.array(b).astype(np.uint16)
            w_ratio, h_ratio = (x2-x1)/100, (y2-y1)/100
            crop = np_img[y1:y2+1, x1:x2+1,:]
            crop = cv2.resize(crop, (100,100))
            crop_imgs.append(crop)
            crop_coords.append((x1,y1))
            crop_ratios.append((w_ratio,h_ratio))
            np_labels.append(label)
        
        landmarks = loc_model.inference(crop_imgs, np_labels, crop_coords, crop_ratios)
        return landmarks
    
    def pointing(self, bgr_img, landmarks):
        if not landmarks:
            return bgr_img
        for landmark in landmarks:
            landmark = list(landmark)
            for i in range(len(landmark)-1):
                x1,y1 = landmark[i]
                x1,y1 = int(x1), int(y1)
                x2,y2 = landmark[i+1]
                x2,y2 = int(x2), int(y2)
                bgr_img = cv2.line(bgr_img,(x1,y1),(x2,y2),(255,0,0),2)
                bgr_img = cv2.line(bgr_img,(x1,y1),(x1,y1),(0,0,255),5)
            bgr_img = cv2.line(bgr_img,(x2,y2),(x2,y2),(255,0,0),5)
        
        return bgr_img
    
    def numpy_to_tensor(self, x):
        x = torch.Tensor(x.astype('float32')).permute(2,0,1)
        x = x.cuda() if torch.cuda.is_available() else x.cpu()
        x = [x/255]
        return x

    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')         # 输出日志信息，提示已进入回调函数
        bgr_image = self.cv_bridge.imgmsg_to_cv2(data.image, 'bgr8')
        yaw = data.yaw
        pitch = data.pitch
        roll = data.roll
        rgb_image =  cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)

        predictions = det_model.run(bgr_image, vis_thresh=0.3)
        landmarks = self.object_localize(rgb_image, predictions)

        for l,c,b in predictions:
            x1,y1,x2,y2 = np.array(b).astype(np.uint16)
            bgr_image = cv2.rectangle(bgr_image, (x1,y1),(x2,y2),(0,255,0),2)
        bgr_image = cal_slope(bgr_image, yaw, pitch, roll)
        dotted_image = self.pointing(bgr_image, landmarks)
        self.stream_camera(dotted_image)                               # 苹果检测

def main(args=None):                                        # ROS2节点主入口main函数
    rclpy.init(args=args)                                   # ROS2 Python接口初始化
    node = ImageSubscriber("topic_webcam_sub")              # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                        # 循环等待ROS2退出
    node.destroy_node()                                     # 销毁节点对象
    rclpy.shutdown()                                        # 关闭ROS2 Python接口