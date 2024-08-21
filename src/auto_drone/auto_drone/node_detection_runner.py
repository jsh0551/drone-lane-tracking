#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
# from custom_msgs.msg import ImageWithInfo
import cv2
from cv_bridge import CvBridge
import sys

import os
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
BASE = os.getcwd()

sys.path.append(os.path.join(BASE, "src"))
sys.path.append(os.path.join(BASE, "src","utils"))
from tools import *
from ultralytics import YOLO

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

font = cv2.FONT_HERSHEY_SIMPLEX  # 글꼴 스타일
font_scale = 0.4  # 글꼴 크기
thickness = 1  # 텍스트 굵기
EVENT = 0
HFOV = 110
WIDTH = 480
HEIGHT = 360
ASPECT_RATIO = 4 / 3
CAM_TILT = -45
VIZ = True

class RunnerDetector(Node):
    def __init__(self, name):
        super().__init__(name)
        self.sub = self.create_subscription(
            Image, '/camera2/video_frames', self.listener_callback, 10)
        self.cv_bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')

    def stream_camera(self, annot_image):
        cv2.imshow("runner", annot_image)
        cv2.waitKey(10)

    def where_runner(self, results):
        boxes = results[0].boxes.xywh.cpu()
        xywh = results[0].boxes.xyxyn.cpu().detach().numpy().tolist()
        nCnt = 0
        runner_pos = []
        for box in zip(boxes):
            #print(box)
            x=xywh[nCnt][0]
            y=xywh[nCnt][1]
            tmp = [x, y]
            nCnt += 1
            runner_pos.append(tmp)
        return runner_pos

    def listener_callback(self, data):
        bgr_image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')
        results = self.model.track(bgr_image, persist=True, classes=[0], conf = 0.1)
        runner_pos = self.where_runner(results)
        self.get_logger().info(f'runner num : {len(runner_pos)}')
        if VIZ:
            annot_image = results[0].plot()
            self.stream_camera(annot_image)


def main(args=None):
    rclpy.init(args=args)
    node = RunnerDetector("node_line_detection")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()