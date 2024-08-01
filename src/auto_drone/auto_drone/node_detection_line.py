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
font_scale = 0.4  # 글꼴 크기
thickness = 1  # 텍스트 굵기
EVENT = 0
HFOV = 110
ASPECT_RATIO = 4 / 3
CAM_TILT = -45
POS_RATIO = 0.25
SIMILARITY = [0.75, 0.75, 0.6, 0.6]

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
        # init
        self.pitch = np.radians(CAM_TILT)
        self.yaw = 0.0
        self.roll = 0.0
        self.with_position = True if EVENT == 0 else False
        # self.with_position = False
        points = [(240 - 80, 360), (240 + 80, 360)]
        # 
        cx = 240
        cy = 180 + 240 * np.tan(self.pitch)
        tmps = []
        for p in points:
            tmp_poly = []
            x_points = np.array([cx, p[0]])
            y_points = np.array([cy, p[1]])
            slope, intercept = np.polyfit(y_points, x_points, 1)
            y_range = np.linspace(180, 360, 100)
            x_values = slope * y_range + intercept
            for x,y in zip(x_values, y_range):
                tmp_poly.append((x, y))
            tmps.append(tmp_poly)
        self.rpoly, self.lpoly = tmps[1] , tmps[0]
        self.targetpoly = []
        self.pub_data = []
        self.prev_error = 0.0
        self.track_width = None

    def draw_slope(self, bgr_img, pitch, roll):
        h,w,_ = bgr_img.shape
        s_points = np.array([(0,h),(w,h),(w/4,h),(w*3/4,h),(0,h*3/4),(w,h*3/4)]).astype(np.int16)
        fov = np.radians(HFOV)
        cy = h//2 + w/(2*np.tan(fov/2))*np.tan(pitch)
        cx = w//2
        for tmp_x, tmp_y in s_points:
            m = (cy-tmp_y) / (cx-tmp_x)
            uy = int(max(0,cy))
            ux = int((uy-cy)/m + cx)
            bgr_img = cv2.line(bgr_img,(tmp_x,tmp_y),(ux,uy),(255,0,255),2)
        return bgr_img

    def get_data(self, data):
        self.quaternion.x = data.orientation.x
        self.quaternion.y = data.orientation.y
        self.quaternion.z = data.orientation.z
        self.quaternion.w = data.orientation.w
        roll, pitch, yaw = euler_from_quaternion((self.quaternion.x,self.quaternion.y,self.quaternion.z,self.quaternion.w))
        self.pitch = -pitch + np.radians(CAM_TILT)
        self.yaw = yaw
        self.roll = roll

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
            # 1.5 : width comprehension
            b[0] = max(x1 - (x2-x1)*upleft_scale*1., 0.0)
            b[1] = y1 - (y2-y1)*upleft_scale
            b[2] = min(x2 + (x2-x1)*downright_scale*1., float(w-1))
            b[3] = y2 + (y2-y1)*downright_scale
            # if b[0] >= w*side_decision and b[2] <= w*(1-side_decision):
            if b[3] >= h*boundary:
                scaled_predictions.append(item)
        return scaled_predictions
    
    def draw_tracklines(self, bgr_img, predictions, landmarksX, landmarksY, points_num = 100):
        # if no landmarks, return
        if not landmarksX:
            rpoly = self.rpoly
            lpoly = self.lpoly
            self.targetpoly = calculate_targetpoly(rpoly, lpoly)
            return bgr_img, self.targetpoly, rpoly, lpoly
        # get polylines
        h,w,_ = bgr_img.shape
        polypoints = []
        for pred, landmarkX, landmarkY in zip(predictions, landmarksX, landmarksY):
            _, _, b = pred
            x1,y1,x2,y2 = np.array(b).astype(np.uint16)
            y1 += h//2
            y2 += h//2
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
            polypoint = []
            polypoint_int = []
            for x,y in zip(x_fit, y_fit):
                polypoint.append((x, y))
                polypoint_int.append((int(x), int(y)))
            # draw track lines
            cv2.polylines(bgr_img, [np.array(polypoint_int)], isClosed=False, color=(255, 0, 0), thickness=2)
            polypoint = sorted(polypoint, key=lambda x:x[1])
            polypoints.append(polypoint)
            # draw landmarks
            for x,y in zip(landmarkX, landmarkY):
                x,y = int(x), int(y)
                bgr_img = cv2.line(bgr_img,(x,y),(x,y),(0,0,255),5)
        # calcuate similarlity
        leftpolys, rightpolys = [], []
        similarity_array = []
        for polypoint in polypoints:
            bot_x = polypoint[-1][0]
            if bot_x - w//2 >= 0:
                if self.rpoly:
                    sim = calculate_similarity(self.rpoly, polypoint, w, h, with_position=self.with_position, pos_ratio=POS_RATIO)
                    cv2.putText(bgr_img, f'{sim:.4f}', (int(polypoint[0][0]),int(polypoint[0][1])), font, font_scale, (0,0,0), thickness, cv2.LINE_AA)
                    similarity_array.append(sim)
                    if sim < SIMILARITY[EVENT]:
                        continue
                rightpolys.append(polypoint)
            else:
                if self.lpoly:
                    sim = calculate_similarity(self.lpoly, polypoint, w, h, with_position=self.with_position, pos_ratio=POS_RATIO)
                    cv2.putText(bgr_img, f'{sim:.4f}', (int(polypoint[0][0]),int(polypoint[0][1])), font, font_scale, (0,0,0), thickness, cv2.LINE_AA)
                    if sim < SIMILARITY[EVENT]:
                        continue
                leftpolys.append(polypoint)
        # return targetpoly if no similar polyline
        if not rightpolys and not leftpolys:
            rpoly = self.rpoly
            lpoly = self.lpoly
            self.targetpoly = calculate_targetpoly(rpoly, lpoly)
            return bgr_img, self.targetpoly, rpoly, lpoly
        # select most similar polyline
        if not self.rpoly:
            rightpolys = sorted(rightpolys, key=lambda x:x[-1][0])
        else:
            rightpolys = sorted(rightpolys, key=lambda x:calculate_similarity(self.rpoly, x, w, h, with_position=self.with_position, pos_ratio=POS_RATIO), reverse=True)
        if not self.lpoly:
            leftpolys = sorted(leftpolys, key=lambda x:x[-1][0], reverse=True)
        else:
            leftpolys = sorted(leftpolys, key=lambda x:calculate_similarity(self.lpoly, x, w, h, with_position=self.with_position, pos_ratio=POS_RATIO), reverse=True)
        rpoly = self.rpoly = rightpolys[0] if rightpolys else self.rpoly
        lpoly = self.lpoly = leftpolys[0] if leftpolys else self.lpoly
        # caculate targetpoly and draw lines
        targetpoly = calculate_targetpoly(rpoly, lpoly)
        self.targetpoly = targetpoly
        targetpoly_int = [(int(x),int(y)) for x,y in self.targetpoly]
        bot_point = self.targetpoly[-1]
        slope = calculate_slope(bot_point, self.pitch, w, h, fov=HFOV)
        t_targetpoly = affine_transform(np.array(self.targetpoly), slope)
        t_bot_point = t_targetpoly[-1]
        gap = t_bot_point[0] - w//2
        t_targetpoly_int = [(int(x-gap),int(y)) for x,y in t_targetpoly]
        cv2.polylines(bgr_img, [np.array(targetpoly_int)], isClosed=False, color=(0, 255, 255), thickness=2)
        cv2.polylines(bgr_img, [np.array(t_targetpoly_int)], isClosed=False, color=(127, 127, 127), thickness=2)
        # write information on the window
        bot_point = np.array(targetpoly)[-1]
        bot_point_r = np.array(rpoly)[-1]
        bot_point_l = np.array(lpoly)[-1]
        factor = bot_point_r[0] - bot_point_l[0]
        error = (bgr_img.shape[1]/2 - bot_point[0])/factor
        if self.track_width is not None:
            if abs((self.track_width-factor)/self.track_width) > 0.6 and rightpolys and leftpolys:
                rightpolys = sorted(rightpolys, key=lambda x:x[0])
                leftpolys = sorted(leftpolys, key=lambda x:x[0], reverse=True)
                self.rpoly, self.lpoly = rightpolys[0], leftpolys[0]
                rpoly,lpoly = rightpolys[0], leftpolys[0]
        self.track_width = factor
        self.prev_error = error
        t_slope = calculate_slope(t_bot_point, self.pitch, w, h, fov=HFOV)
        theta = np.arctan(1/t_slope)
        vfov = 2 * np.arctan(np.tan(np.radians(HFOV / 2)) / ASPECT_RATIO)
        cv2.putText(bgr_img, f'yaw : {np.degrees(self.yaw):.4f}', (20,20), font, font_scale, (122,122,0), thickness, cv2.LINE_AA)
        cv2.putText(bgr_img, f'prev error : {self.prev_error:.4f}, error : {error:.4f}, slope : {np.degrees((theta + self.roll)/np.tan(vfov/2)):.4f}', (20,40), font, font_scale, (122,122,0), thickness, cv2.LINE_AA)
        return bgr_img, self.targetpoly, rpoly, lpoly


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
        bgr_image = self.draw_slope(bgr_image, self.pitch, self.roll)
        dotted_image, polypoint, rpoly, lpoly = self.draw_tracklines(bgr_image, predictions, landmarksX, landmarksY)
        rpoly_int, lpoly_int = [(int(x),int(y)) for x,y in rpoly], [(int(x),int(y)) for x,y in lpoly]
        cv2.polylines(dotted_image, [np.array(rpoly_int)], isClosed=False, color=(127, 0, 0), thickness=1)
        cv2.polylines(dotted_image, [np.array(lpoly_int)], isClosed=False, color=(127, 0, 0), thickness=1)
        self.stream_camera(dotted_image)
        # self.pub_data = [polypoint, rpoly, lpoly, h, w]
        self.publish_polypoints(polypoint, rpoly, lpoly, h, w)

    def pub_callback(self):
        if self.pub_data:
            polypoint, rpoly, lpoly, h, w = self.pub_data
            self.publish_polypoints(polypoint, rpoly, lpoly, h, w)

    def publish_polypoints(self, polypoint, rpoly, lpoly, h, w):
        targetpoly_info = TargetPolyInfo()
        if polypoint and rpoly and lpoly:
            polyline_cloud, rpoly_cloud, lpoly_cloud = PointCloud(), PointCloud(), PointCloud()
            for x_pair, y_pair in polypoint:
                point = Point32(x=x_pair, y=y_pair)
                polyline_cloud.points.append(point)
            for x_pair, y_pair in rpoly:
                point = Point32(x=x_pair, y=y_pair)
                rpoly_cloud.points.append(point)
            for x_pair, y_pair in lpoly:
                point = Point32(x=x_pair, y=y_pair)
                lpoly_cloud.points.append(point)
            targetpoly_info.polyline = polyline_cloud
            targetpoly_info.rpoly = rpoly_cloud
            targetpoly_info.lpoly = lpoly_cloud
            targetpoly_info.height = int(h)
            targetpoly_info.width = int(w)
        self.pub.publish(targetpoly_info)


def main(args=None):
    rclpy.init(args=args)
    node = ImageDetector("node_line_detection")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()