#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud, ChannelFloat32, Imu
from geometry_msgs.msg import Point32, Quaternion
# from custom_msgs.msg import ImageWithInfo
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import sys
import math
import torch
import os
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from tf_transformations import euler_from_quaternion
BASE = os.getcwd()

sys.path.append(os.path.join(BASE, "src"))
sys.path.append(os.path.join(BASE, "src", "model_linedet"))
sys.path.append(os.path.join(BASE, "src", "model_linedet", "model_yolox"))
sys.path.append(os.path.join(BASE, "src","utils"))
from tools import *
import DQLL
from models.yolox import Detector
from yolox_config import opt

from custom_message.msg import LandmarkCloud, TargetPolyInfo

det_model = Detector(opt)
loc_model = DQLL.localizator()
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

font = cv2.FONT_HERSHEY_SIMPLEX  # 글꼴 스타일
font_scale = 0.3  # 글꼴 크기
thickness = 1  # 텍스트 굵기

def calculate_angle(polyline):
    mi = len(polyline)//2
    bx,by = polyline[-1]
    mx,my = polyline[0] # TODO change this in the future
    theta = np.arctan((mx-bx)/(my-by))
    return theta

def draw_slope(bgr_img, pitch):
    '''
    x : track lane의 중점으로부터의 거리(px)
    w,h : 이미지 너비,높이(px)
    alpha : 카메라 기울기 각도. 바닥과 수평시 0도 (radian?)
    f : 초점거리
    '''
    h,w,_ = bgr_img.shape
    s_points = np.array([(0,h),(w,h),(w/4,h),(w*3/4,h),(0,h*3/4),(w,h*3/4)]).astype(np.int16)
    cx = w//2
    cy = h//2 + 240*np.tan(pitch)
    for tmp_x, tmp_y in s_points:
        m = (cy-tmp_y) / (cx-tmp_x)
        uy = int(max(0,cy))
        ux = int((uy-cy)/m + cx)
        bgr_img = cv2.line(bgr_img,(tmp_x,tmp_y),(ux,uy),(255,0,255),2)
    # h/2나 2h/3에서 만나는 점 구하기.
    return bgr_img


class ImageDetector(Node):
    def __init__(self, name):
        super().__init__(name)
        self.sub = self.create_subscription(
            Image, '/camera1/video_frames', self.listener_callback, 10)
        self.pub = self.create_publisher(
            TargetPolyInfo, '/targetpoly_info', 10)
        self.subscriber_data = self.create_subscription(
            Imu, '/mavros/imu/data', self.get_data, qos_profile)
        self.cv_bridge = CvBridge()
        self.quaternion = Quaternion()
        self.pitch = np.radians(-30)
        self.rpoly, self.lpoly = [], []
        self.targetpoly = []
        self.lerror, self.rerror = 0.0, 0.0
        x_coords = np.linspace(180, 360, 100)
        y_coords = np.ones(100) * 240
        self.initial_polyline = np.column_stack([x_coords, y_coords]).astype(np.float32)
        
        self.tracker = KalmanFilterPolylineTracker(self.initial_polyline)

    def tracking(self, detected_polyline, frame):
        detected_polyline = np.array(detected_polyline)
        predicted_polyline = np.array(self.tracker.predict())
        estimated_polyline = np.array(self.tracker.correct(detected_polyline))

        # 감지된 polyline 그리기 (흰색)
        for point in detected_polyline.astype(int):
            cv2.circle(frame, tuple(point), 2, (255, 255, 255), -1)

        # 예측된 polyline 그리기 (녹색)
        for point in predicted_polyline.astype(int):
            cv2.circle(frame, tuple(point), 2, (0, 255, 0), -1)

        # 보정된 polyline 그리기 (빨간색)
        for point in estimated_polyline.astype(int):
            cv2.circle(frame, tuple(point), 2, (0, 0, 255), -1)

        self.initial_polyline = estimated_polyline
        return frame

    def get_data(self, data):
        self.quaternion.x = data.orientation.x
        self.quaternion.y = data.orientation.y
        self.quaternion.z = data.orientation.z
        self.quaternion.w = data.orientation.w
        roll, pitch, yaw = euler_from_quaternion((self.quaternion.x,self.quaternion.y,self.quaternion.z,self.quaternion.w))
        self.pitch = -pitch + np.radians(-30)

    def stream_camera(self, bgr_img):
        cv2.imshow("object", bgr_img)
        cv2.waitKey(10)


    def object_localize(self, np_img, predictions):
        if len(predictions)==0:
            return [],[],[]
        h, w, _ = np_img.shape
        predictions = self.scaling_bbox(predictions, w, h)
        predictions = nms(predictions, 0.3)
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
        
        landmarksX, landmarksY = loc_model.inference(crop_imgs, np_labels, crop_coords, crop_ratios)
        return landmarksX, landmarksY, predictions

    def scaling_bbox(self, predictions, w, h, scale_limit = 0.15, boundary = 0.75):
        scaled_predictions = []
        for item in predictions:
            _, _, b = item
            x1,y1,x2,y2 = b
            pad_x1, pad_y1, pad_x2, pad_y2 = x1, y1, w-x2, h-y2
            upleft_scale = min(pad_x1/(x2-x1), pad_y1/(y2-y1), scale_limit)
            downright_scale = min(pad_x2/(x2-x1), pad_y2/(y2-y1), scale_limit)
            b[0] = x1 - (x2-x1)*upleft_scale
            b[1] = y1 - (y2-y1)*upleft_scale
            b[2] = x2 + (x2-x1)*downright_scale
            b[3] = y2 + (y2-y1)*downright_scale
            # if b[0] >= w*side_decision and b[2] <= w*(1-side_decision):
            if b[3] >= h*boundary:
                scaled_predictions.append(item)
        return scaled_predictions
    
    def draw_tracklines(self, bgr_img, predictions, landmarksX, landmarksY, points_num = 100):
        if not landmarksX:
            return bgr_img, []
        h,w,_ = bgr_img.shape
        polypoints = []
        for pred, landmarkX, landmarkY in zip(predictions, landmarksX, landmarksY):
            _, _, b = pred
            x1,y1,x2,y2 = np.array(b).astype(np.uint16)
            y1 += h//2
            y2 += h//2
            # draw track line
            if x1 < (h-y2):
                coefficients = np.polyfit(landmarkX, landmarkY, 2)
                a, b, c = coefficients
                x_fit = np.linspace(0, int(x2), points_num)
                y_fit = a*x_fit**2 + b*x_fit + c
            elif (w-x2) < (h-y2):
                coefficients = np.polyfit(landmarkX, landmarkY, 2)
                a, b, c = coefficients
                x_fit = np.linspace(int(x1), w, points_num)
                y_fit = a*x_fit**2 + b*x_fit + c
            else:
                coefficients = np.polyfit(landmarkY, landmarkX, 2)
                a, b, c = coefficients
                y_fit = np.linspace(h//2, h, points_num)
                x_fit = a*y_fit**2 + b*y_fit + c
            if abs(a) >0.003:
                continue

            polypoint = [] # for return
            polypoint_int = [] # for draw
            for x,y in zip(x_fit, y_fit):
                polypoint.append((x, y))
                polypoint_int.append((int(x), int(y)))
            cv2.polylines(bgr_img, [np.array(polypoint_int)], isClosed=False, color=(255, 0, 0), thickness=2)
            polypoint = sorted(polypoint, key=lambda x:x[1])
            polypoints.append(polypoint)
            # draw points
            for x,y in zip(landmarkX, landmarkY):
                x,y = int(x), int(y)
                bgr_img = cv2.line(bgr_img,(x,y),(x,y),(0,0,255),5)
        # draw target line
        leftpolys, rightpolys = [], []
        similarity_array = []
        for polypoint in polypoints:
            bot_x = polypoint[-1][0]
            if bot_x - w//2 >= 0:
                if self.rpoly:
                    sim = calculate_similarity(self.rpoly, polypoint)
                    cv2.putText(bgr_img, f'{sim:.4f}', (int(polypoint[0][0]),int(polypoint[0][1])), font, font_scale, (0,0,0), thickness, cv2.LINE_AA)
                    similarity_array.append(sim)
                    if sim < 0.85:
                        continue
                rightpolys.append(polypoint)
            else:
                if self.lpoly:
                    sim = calculate_similarity(self.lpoly, polypoint)
                    cv2.putText(bgr_img, f'{sim:.4f}', (int(polypoint[0][0]),int(polypoint[0][1])), font, font_scale, (0,0,0), thickness, cv2.LINE_AA)
                    if sim < 0.85:
                        continue
                leftpolys.append(polypoint)
        if not rightpolys and not leftpolys:
            return bgr_img, []
        if not self.rpoly:
            rightpolys = sorted(rightpolys, key=lambda x:x[-1][0])
        else:
            rightpolys = sorted(rightpolys, key=lambda x:calculate_similarity(self.rpoly, x), reverse=True)
        if not self.lpoly:
            leftpolys = sorted(leftpolys, key=lambda x:x[-1][0], reverse=True)
        else:
            leftpolys = sorted(leftpolys, key=lambda x:calculate_similarity(self.lpoly, x), reverse=True)
        if not rightpolys or not leftpolys:
            rpoly = self.rpoly
            lpoly = self.lpoly
        else:
            self.rpoly, self.lpoly = rightpolys[0], leftpolys[0]
            rpoly,lpoly = rightpolys[0], leftpolys[0]
        # bgr_img = cv2.line(bgr_img,(int(rpoly[-1][0]),int(rpoly[-1][1])),(int(rpoly[-1][0]),int(rpoly[-1][1])),(0,0,0),15)
        # bgr_img = cv2.line(bgr_img,(int(lpoly[-1][0]),int(lpoly[-1][1])),(int(lpoly[-1][0]),int(lpoly[-1][1])),(0,0,0),15)
        targetpoly = []
        # sim_str = "   ".join([f"{sim:.4f}" for sim in similarity_array])
        # self.get_logger().info(f'similarity : [{sim_str}]')
        # if rpoly:
        #     bgr_img = self.tracking(rpoly, bgr_img)
        for p1,p2 in zip(rpoly,lpoly):
            x1,y1 = p1
            x2,y2 = p2
            x,y = (x1+x2)/2, (y1+y2)/2
            targetpoly.append((x,y))
        self.targetpoly = targetpoly
        targetpoly_int = [(int(x),int(y)) for x,y in self.targetpoly]
        bot_point = self.targetpoly[-1]
        slope = calculate_slope(bot_point, self.pitch, w, h)
        t_targetpoly = affine_transform(np.array(self.targetpoly), slope)
        t_bot_point = t_targetpoly[-1]
        gap = t_bot_point[0] - w//2
        t_targetpoly_int = [(int(x-gap),int(y)) for x,y in t_targetpoly]
        cv2.polylines(bgr_img, [np.array(targetpoly_int)], isClosed=False, color=(0, 255, 255), thickness=2)
        cv2.polylines(bgr_img, [np.array(t_targetpoly_int)], isClosed=False, color=(127, 127, 127), thickness=2)
        return bgr_img, self.targetpoly


    def listener_callback(self, data):
        bgr_image = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')
        h, w, _ = bgr_image.shape
        bgr_image_lower = bgr_image[h//2:,:,:]
        rgb_image_lower =  cv2.cvtColor(bgr_image_lower, cv2.COLOR_BGR2RGB)

        predictions = det_model.run(bgr_image_lower, vis_thresh=0.3)
        landmarksX, landmarksY, predictions = self.object_localize(rgb_image_lower, predictions)
        landmarksY = np.array(landmarksY)
        landmarksY += h//2
        for l,c,b in predictions:
            x1,y1,x2,y2 = np.array(b).astype(np.uint16)
            y1 += h//2
            y2 += h//2
            bgr_image = cv2.rectangle(bgr_image, (x1,y1),(x2,y2),(0,255,0),2)
            # bgr_image = cv2.line(bgr_image, (130, 0), (130, 480), (0, 0, 255), 5)
            # bgr_image = cv2.line(bgr_image, (420, 0), (420, 480), (0, 0, 255), 5)
        bgr_image = draw_slope(bgr_image, self.pitch)
        dotted_image, polypoint = self.draw_tracklines(bgr_image, predictions, landmarksX, landmarksY)
        self.stream_camera(dotted_image)
        self.publish_polypoints(polypoint, h, w)


    def publish_polypoints(self, polypoint, h, w):
        targetpoly_info = TargetPolyInfo()
        if polypoint:
            # print("New Image")
            point_cloud = PointCloud()
            for x_pair, y_pair in polypoint:
                point = Point32(x=x_pair, y=y_pair)
                point_cloud.points.append(point)
            targetpoly_info.clouds = point_cloud
            targetpoly_info.height = int(h)
            targetpoly_info.width = int(w)
        self.pub.publish(targetpoly_info)


def main(args=None):
    rclpy.init(args=args)
    node = ImageDetector("node_line_detection")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()