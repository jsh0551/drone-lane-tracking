#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
import cv2
from cv_bridge import CvBridge
import sys
import os
BASE = os.getcwd()
sys.path.append(os.path.join(BASE))
sys.path.append(os.path.join(BASE, "src"))
from tools import *
from config import cfg
from ultralytics import YOLO

VIZ = cfg.SETTING.VIZ_SIDE
EVENT = cfg.CONTROL.EVENT
HFOV = cfg.HFOV
ASPECT_RATIO = cfg.ASPECT_RATIO
CAM_TILT = cfg.CAM_TILT
WIDTH = cfg.WIDTH
HEIGHT = cfg.HEIGHT
qos_profile = cfg.qos_profile
font = cfg.font
font_scale = cfg.font_scale
thickness = cfg.thickness

class RunnerDetector(Node):
    def __init__(self, name):
        super().__init__(name)
        self.publisher = self.create_publisher(Float64MultiArray, '/vel_data/runner_position', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.sub = self.create_subscription(
            Image, '/camera2/video_frames', self.listener_callback, 10)
        self.cv_bridge = CvBridge()
        self.runner_positions = Float64MultiArray()
        self.runner_positions.data = [0.5]
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
            w=xywh[nCnt][2]
            # tmp = [x, y]
            # px = max(1,0, x + w/2)
            px = x
            nCnt += 1
            runner_pos.append(px)
            # runner_pos.append(y)
        self.get_logger().info(f'-----------------------------detected num : {len(boxes)}')
        if len(runner_pos) > 0:
            return runner_pos
        else:
            return []

    def listener_callback(self, data):
        bgr_image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')
        results = self.model.track(bgr_image, persist=True, classes=[0], conf = 0.1)
        runner_pos = self.where_runner(results)
        self.runner_positions.data = runner_pos
        # self.get_logger().info(f'runner num : {len(runner_pos)}')
        # if len(runner_pos) > 0:
        #     self.get_logger().info(f'runner x : {runner_pos[0]}')
        if VIZ:
            annot_image = results[0].plot()
            self.stream_camera(annot_image)

    def timer_callback(self):
        self.publisher.publish(self.runner_positions)

def main(args=None):
    rclpy.init(args=args)
    node = RunnerDetector("node_line_detection")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()