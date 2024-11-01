#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud, Imu
from geometry_msgs.msg import Point32, Quaternion
# from custom_msgs.msg import ImageWithInfo
import cv2
from cv_bridge import CvBridge
import numpy as np
import sys
import math
import torch
import os
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from tf_transformations import euler_from_quaternion
BASE = os.getcwd()

sys.path.append(os.path.join(BASE))
sys.path.append(os.path.join(BASE, "src"))
sys.path.append(os.path.join(BASE, "src", "model_linedet"))
sys.path.append(os.path.join(BASE, "src", "model_linedet", "model_yolox"))
from tools import *
from config import cfg
import DQLL
from models.yolox import Detector
from yolox_config import opt

from custom_message.msg import TargetPolyInfo

det_model = Detector(opt)
loc_model = DQLL.localizator()

VIZ = cfg.SETTING.VIZ_FRONT
POS_RATIO = cfg.DETECT.POS_RATIO
SIMILARITY_LIMIT = cfg.DETECT.SIMILARITY_LIMIT
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

class VizCamera:
    def __init__(self):
        self.h, self.w = HEIGHT, WIDTH
        self.s_points = np.array([(0, self.h),(self.w, self.h),(self.w/4, self.h),(self.w*3/4, self.h),(0, self.h*3/4),(self.w, self.h*3/4)]).astype(np.int16)
        self.fov = np.radians(HFOV)
        self.predictions = []
        self.polypoints = []
        self.landmarksX, self.landmarksY = [], []
        self.similaritys = []
        self.targetpoly, self.t_targetpoly = [], []

    def get_predictions(self, predictions):
        self.predictions = predictions

    def get_polyline(self, polypoints, similaritys, landmarksX, landmarksY):
        self.polypoints = polypoints
        self.similaritys = similaritys
        self.landmarksX = landmarksX
        self.landmarksY = landmarksY

    def get_targetpoly(self, targetpoly, t_targetpoly):
        self.targetpoly = targetpoly
        self.t_targetpoly = t_targetpoly

    def drawing(self, bgr_image, pitch):
        self.cy = self.h//2 + self.w/(2*np.tan(self.fov/2))*np.tan(pitch)
        self.cx = self.w//2
        # draw bounding boxes
        for l,c,b in self.predictions:
            x1,y1,x2,y2 = np.array(b).astype(np.uint16)
            y1 += self.h//2
            y2 += self.h//2
            bgr_image = cv2.rectangle(bgr_image, (x1,y1),(x2,y2),(0,255,0),2)
        # draw baselines
        for tmp_x, tmp_y in self.s_points:
            m = (self.cy - tmp_y) / (self.cx - tmp_x)
            uy = int(max(0,self.cy))
            ux = int((uy - self.cy)/m + self.cx)
            bgr_image = cv2.line(bgr_image, (tmp_x,tmp_y), (ux,uy), (255,0,255),2)
        # draw track lines
        for polypoint, sim, landmarkX, landmarkY in zip(self.polypoints, self.similaritys, self.landmarksX, self.landmarksY):
            polypoint_int = [(int(p[0]), int(p[1])) for p in polypoint]
            cv2.polylines(bgr_image, [np.array(polypoint_int)], isClosed=False, color=(255, 0, 0), thickness=2)
            # draw landmarks
            for x,y in zip(landmarkX, landmarkY):
                x,y = int(x), int(y)
                bgr_image = cv2.line(bgr_image,(x,y),(x,y),(0,0,255),5)
            cv2.putText(bgr_image, f'{sim:.4f}', (int(polypoint[0][0]),int(polypoint[0][1])), font, font_scale, (0,0,0), thickness, cv2.LINE_AA)
        # draw targetpoly
        targetpoly_int = [(int(x),int(y)) for x,y in self.targetpoly]
        t_bot_point = self.t_targetpoly[-1]
        gap = t_bot_point[0] - self.w//2
        t_targetpoly_int = [(int(x-gap),int(y)) for x,y in self.t_targetpoly]
        cv2.polylines(bgr_image, [np.array(targetpoly_int)], isClosed=False, color=(0, 255, 255), thickness=2)
        cv2.polylines(bgr_image, [np.array(t_targetpoly_int)], isClosed=False, color=(127, 127, 127), thickness=2)
        return bgr_image


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
        points = [(WIDTH//3, HEIGHT//2), ((2*WIDTH)//3, HEIGHT//2)]
        # default track line
        cx = WIDTH//2
        cy = HEIGHT//2 + WIDTH//2 * np.tan(self.pitch)
        tmps = []
        for p in points:
            tmp_poly = []
            x_points = np.array([cx, p[0]])
            y_points = np.array([cy, p[1]])
            slope, intercept = np.polyfit(y_points, x_points, 1)
            y_range = np.linspace(HEIGHT//2, HEIGHT, 100)
            x_values = slope * y_range + intercept
            for x,y in zip(x_values, y_range):
                tmp_poly.append((x, y))
            tmps.append(tmp_poly)
        self.rpoly, self.lpoly = tmps[1] , tmps[0]
        self.targetpoly, self.t_targetpoly = [], []
        self.pub_data = []
        self.prev_error = 0.0
        self.track_width = None
        self.vizcam = VizCamera()

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
        h,w,_ = bgr_img.shape
        # if no landmarks, return
        if not landmarksX:
            rpoly = self.rpoly
            lpoly = self.lpoly
            self.targetpoly = calculate_targetpoly(rpoly, lpoly)
            bot_point = self.targetpoly[-1]
            slope = calculate_slope(bot_point, self.pitch, w, h, fov=HFOV)
            t_targetpoly = affine_transform(np.array(self.targetpoly), slope)
            self.t_targetpoly = t_targetpoly
            if VIZ:
                self.vizcam.get_targetpoly(self.targetpoly, self.t_targetpoly)
                bgr_img = self.vizcam.drawing(bgr_img, self.pitch)
            return bgr_img, self.targetpoly, rpoly, lpoly
        # get polylines
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
            if abs(a) > 0.003:
                continue
            polypoint = []
            for x,y in zip(x_fit, y_fit):
                polypoint.append((x, y))
            polypoint = sorted(polypoint, key=lambda x:x[1])
            polypoints.append(polypoint)
        # calcuate similarlity
        leftpolys, rightpolys = [], []
        similaritys = []
        for polypoint in polypoints:
            bot_x = polypoint[-1][0]
            if bot_x - w//2 >= 0:
                if self.rpoly:
                    sim = calculate_similarity(self.rpoly, polypoint, w, h, with_position=self.with_position, pos_ratio=POS_RATIO)
                    similaritys.append(sim)
                    if sim < SIMILARITY_LIMIT[EVENT]:
                        continue
                rightpolys.append(polypoint)
            else:
                if self.lpoly:
                    sim = calculate_similarity(self.lpoly, polypoint, w, h, with_position=self.with_position, pos_ratio=POS_RATIO)
                    similaritys.append(sim)
                    if sim < SIMILARITY_LIMIT[EVENT]:
                        continue
                leftpolys.append(polypoint)
        if VIZ:
            self.vizcam.get_polyline(polypoints, similaritys, landmarksX, landmarksY)
        # return targetpoly if no similar polyline
        if not rightpolys and not leftpolys:
            rpoly = self.rpoly
            lpoly = self.lpoly
            self.targetpoly = calculate_targetpoly(rpoly, lpoly)
            bot_point = self.targetpoly[-1]
            slope = calculate_slope(bot_point, self.pitch, w, h, fov=HFOV)
            t_targetpoly = affine_transform(np.array(self.targetpoly), slope)
            self.t_targetpoly = t_targetpoly
            if VIZ:
                self.vizcam.get_targetpoly(self.targetpoly, self.t_targetpoly)
                bgr_img = self.vizcam.drawing(bgr_img, self.pitch)
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
        bot_point = self.targetpoly[-1]
        slope = calculate_slope(bot_point, self.pitch, w, h, fov=HFOV)
        t_targetpoly = affine_transform(np.array(self.targetpoly), slope)
        self.t_targetpoly = t_targetpoly
        t_bot_point = t_targetpoly[-1]
        if VIZ:
            self.vizcam.get_targetpoly(self.targetpoly, self.t_targetpoly)
            bgr_img = self.vizcam.drawing(bgr_img, self.pitch)
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

        self.vizcam.get_predictions(predictions)
        dotted_image, polypoint, rpoly, lpoly = self.draw_tracklines(bgr_image, predictions, landmarksX, landmarksY)

        if VIZ:
            rpoly_int, lpoly_int = [(int(x),int(y)) for x,y in rpoly], [(int(x),int(y)) for x,y in lpoly]
            cv2.polylines(dotted_image, [np.array(rpoly_int)], isClosed=False, color=(127, 0, 0), thickness=1)
            cv2.polylines(dotted_image, [np.array(lpoly_int)], isClosed=False, color=(127, 0, 0), thickness=1)
            self.stream_camera(dotted_image)
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