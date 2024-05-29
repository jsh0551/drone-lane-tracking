#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud, ChannelFloat32
from geometry_msgs.msg import Point32
# from custom_msgs.msg import ImageWithInfo
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import sys
import math
import torch
import os
BASE = os.getcwd()

sys.path.append(os.path.join(BASE, "src"))
sys.path.append(os.path.join(BASE, "src", "model_linedet"))
sys.path.append(os.path.join(BASE, "src", "model_linedet", "model_yolox"))

import DQLL
from models.yolox import Detector
from yolox_config import opt

from custom_message.msg import LandmarkCloud

det_model = Detector(opt)
loc_model = DQLL.localizator()
WIDTH, HEIGHT = 640, 480
s_points = np.array([(0,HEIGHT),(WIDTH,HEIGHT),(WIDTH/4,HEIGHT),(WIDTH*3/4,HEIGHT),(0,HEIGHT*3/4),(WIDTH,HEIGHT*3/4)]).astype(np.int16)

# Load a model
lower_red = np.array([0, 90, 128])
upper_red = np.array([180, 255, 255])



class ImageDetector(Node):
    def __init__(self, name):
        super().__init__(name)
        self.sub = self.create_subscription(
            Image, '/camera1/video_frames', self.listener_callback, 10)
        self.pub = self.create_publisher(
            LandmarkCloud, '/landmark_points', 10)
        self.cv_bridge = CvBridge()


    def stream_camera(self, bgr_img):
        cv2.imshow("object", bgr_img)
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
            # bgr_img = cv2.line(bgr_img,(x2,y2),(x2,y2),(255,0,0),5)
        
        return bgr_img
    

    def numpy_to_tensor(self, x):
        x = torch.Tensor(x.astype('float32')).permute(2,0,1)
        x = x.cuda() if torch.cuda.is_available() else x.cpu()
        x = [x/255]
        return x


    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        bgr_image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')
        rgb_image =  cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)

        predictions = det_model.run(bgr_image, vis_thresh=0.3)
        landmarks = self.object_localize(rgb_image, predictions)

        for l,c,b in predictions:
            x1,y1,x2,y2 = np.array(b).astype(np.uint16)
            bgr_image = cv2.rectangle(bgr_image, (x1,y1),(x2,y2),(0,255,0),2)
            # bgr_image = cv2.line(bgr_image, (130, 0), (130, 480), (0, 0, 255), 5)
            # bgr_image = cv2.line(bgr_image, (420, 0), (420, 480), (0, 0, 255), 5)
        
        self.publish_landmarks(landmarks)

        dotted_image = self.pointing(bgr_image, landmarks)
        self.stream_camera(dotted_image)


    def publish_landmarks(self, landmarks):
        
        landmark_cloud = LandmarkCloud()
        if landmarks:
            # print("New Image")
            for landmark in landmarks:
                # print("line")
                point_cloud = PointCloud()
                for pair in landmark:
                    x_pair, y_pair = pair
                    point = Point32(x=x_pair, y=y_pair)
                    point_cloud.points.append(point)
                landmark_cloud.clouds.append(point_cloud)
        self.pub.publish(landmark_cloud)



def main(args=None):
    rclpy.init(args=args)
    node = ImageDetector("node_line_detection")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()